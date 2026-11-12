"""Đo log gốc, xuất bảng sự kiện và số đo VX1; không sửa dữ liệu."""
from pathlib import Path
import csv
import json
import sys
import numpy as np
import cv2
sys.stdout.reconfigure(encoding='utf-8')

ROOT=Path(__file__).resolve().parents[3]
OUT=ROOT/'round3_final/test_logs/V1p_1213'
OUT.mkdir(parents=True, exist_ok=True)


def read(p):
    with p.open(encoding='utf-8-sig',newline='') as f:
        return list(csv.DictReader(f))


def num(r,k):
    try: return float(r[k])
    except (ValueError,KeyError): return float('nan')


def write(name, rows):
    if not rows: return
    with (OUT/name).open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)


def inventory():
    found={}
    for directory in sorted((ROOT/'data/auto/runs_auto').glob('runs_1212_l*')):
        try: level=int(directory.name.split('_l')[-1])
        except ValueError: continue
        if not 17<=level<=27: continue
        for p in sorted(directory.glob('x/*/session.json')):
            found.setdefault(p.parent.name,[]).append(p.parent)
    return found


def main():
    found=inventory(); catalog=[]; events=[]; segments=[]; vx=[]
    specified={'7795':[188],'7886':[867,1013],'8111':[203],'9084':[2505,3032],
               '10547':[2315,2724],'13546':[1656,2295],'16925':[762,1096]}
    good={'9393':[(2013,2127)],'10547':[(2798,2933)],'16488':[(2088,2196)]}
    for name, paths in found.items():
        path=paths[0]; raw=read(path/'control_log.csv'); rows=[r for r in raw if np.isfinite(num(r,'t_ms')) and r.get('reason') is not None]; session=json.loads((path/'session.json').read_text(encoding='utf-8'))
        sha=session.get('unet_onnx_sha256',''); model='t94' if sha.startswith('1e1964') else 't95' if sha.startswith('86c703') else sha
        t0=num(rows[0],'t_ms'); key=name.split('_')[-1]
        catalog.append({'session':name,'model':model,'paths':';'.join(str(p.relative_to(ROOT)) for p in paths),'rows':len(rows),'invalid_tail_rows':len(raw)-len(rows),'seconds':(num(rows[-1],'t_ms')-t0)/1000,
                        'mask_rows':sum(bool(r['mask_file']) for r in rows),'go':sum(r['go']=='1' for r in rows),'stop':sum(r['stop']=='1' for r in rows),'sha':sha})
        def evidence(i):
            r=rows[i]
            return {'session':name,'model':model,'row':i,'t':round((num(r,'t_ms')-t0)/1000,3),
                    **{k:r.get(k,'') for k in ('reason','planner_state','route_step','route_note','turn_dir','traj_mode','e','yaw_deg','odo_cm','cmd_deg','cmd_cms','frame_id','mask_file')}}
        last=None; start=0
        for i,r in enumerate(rows):
            if r['go']=='1' or r['stop']=='1':
                event=evidence(i);event['event']='G' if r['go']=='1' else 'Space' if r['reason']=='SPACE' else 'Stop';events.append(event)
            state=(r['reason'],r['planner_state'],r.get('route_step',''),r.get('turn_dir',''))
            if state!=last:
                if last is not None:
                    item=evidence(start);item['last']=i-1;item['t_end']=round((num(rows[i-1],'t_ms')-t0)/1000,3);segments.append(item)
                last=state;start=i
        item=evidence(start);item['last']=len(rows)-1;item['t_end']=round((num(rows[-1],'t_ms')-t0)/1000,3);segments.append(item)
        for end in specified.get(key,[]):
            # Lần mở gần nhất sau Space trước, từ reason VX_QUANH_DAO; log không có rb_in_pass.
            prev=max([i for i in range(end) if rows[i]['stop']=='1']+[-1])
            candidates=[i for i in range(prev+1,end) if rows[i]['reason']=='VX_QUANH_DAO' and (i==0 or rows[i-1]['reason'] not in ('VX_QUANH_DAO','VX_THOAT','GIU_HUONG'))]
            if candidates:
                vx.append(measure(name,model,rows,candidates[0],end,t0,'Space: lần mở đầu sau G'))
                # Nếu đã kết thúc rồi mở lại đảo trong cùng lượt G, tách mốc gần nhất.
                gaps=[i for i in range(prev+1,end) if rows[i]['reason']=='VX_QUANH_DAO' and i>0 and rows[i-1]['traj_mode']!='VONG_XUYEN']
                if gaps and gaps[-1]!=candidates[0]: vx.append(measure(name,model,rows,gaps[-1],end,t0,'Space: lần mở gần nhất'))
        for start,end in good.get(key,[]):
            vx.append(measure(name,model,rows,start,end,t0,'thoát tốt theo handoff'))
    write('catalog.csv',catalog);write('events.csv',events);write('reason_segments.csv',segments);write('vx_measurements.csv',vx)
    (OUT/'inventory.json').write_text(json.dumps({k:[str(p) for p in v] for k,v in found.items()},indent=2),encoding='utf-8')
    print('Phiên duy nhất:',len(catalog),'sự kiện:',len(events),'đoạn reason/state/route/intent:',len(segments))
    for item in catalog: print(item['session'],item['model'],item['rows'],round(item['seconds'],1),'G/Space',item['go'],item['stop'])
    print('VX:',len(vx),'khoảng đo; chi tiết ở',OUT)


def measure(name,model,rows,start,end,t0,label):
    a,b=rows[start],rows[end]; interval=rows[start:end+1]
    def span(k):
        values=[num(r,k) for r in interval];return str(round(float(np.nanmin(values)),3))+'..'+str(round(float(np.nanmax(values)),3))
    return {'session':name,'model':model,'label':label,'first':start,'last':end,
            't_first':round((num(a,'t_ms')-t0)/1000,3),'t_last':round((num(b,'t_ms')-t0)/1000,3),
            'dyaw':num(b,'yaw_deg')-num(a,'yaw_deg'),'dodo':num(b,'odo_cm')-num(a,'odo_cm'),
            'yaw_first':a['yaw_deg'],'yaw_last':b['yaw_deg'],'odo_first':a['odo_cm'],'odo_last':b['odo_cm'],
            'e_range':span('e'),'cmd_range':span('cmd_deg'),'e_last':b['e'],'cmd_last':b['cmd_deg'],
            'e_pre_end':rows[max(start,end-1)]['e'],'cmd_pre_end':rows[max(start,end-1)]['cmd_deg']}


if __name__=='__main__':main()
