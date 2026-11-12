"""Thử ngoại tuyến chọn dải phải; chỉ đo điểm nhìn, không giả lập vị trí xe."""
from pathlib import Path
import csv
import json
import re
import sys
import cv2
import numpy as np
from analyze_sessions import ROOT, OUT, read, inventory, write
sys.stdout.reconfigure(encoding='utf-8')


def params(path):
    result={}
    for line in path.read_text(encoding='utf-8').splitlines():
        m=re.match(r'^(\w+):\s*([^#]+)',line)
        if not m: continue
        try: result[m[1]]=json.loads(m[2].strip())
        except ValueError: pass
    return result


def corridor(mask,p,rightmost=False,midpoint=False):
    h,w=mask.shape;axis=w/2+p['center_offset_frac']*w
    rows=[];lower=None;stopped=False
    near=np.floor(p['corridor_bottom_frac']*h+.5);far=np.floor(p['corridor_top_frac']*h+.5)
    for frac in np.linspace(p['corridor_bottom_frac'],p['corridor_top_frac'],p['corridor_rows']):
        y=int(np.clip(np.floor(frac*h+.5),0,h-1));t=np.clip((near-y)/(near-far),0,1)
        expected=w*(p['lane1_width_frac'][0]+np.diff(p['lane1_width_frac'])[0]*t)
        bits=np.r_[False,mask[y]>0,False].astype(int);edges=np.diff(bits)
        segs=[(a,b-1) for a,b in zip(np.where(edges==1)[0],np.where(edges==-1)[0]) if b-a>=.15*expected]
        if stopped and not rightmost: continue
        if rightmost: pick=segs[-1] if segs else None
        elif lower is None:
            pick=None;best=expected
            for s in segs:
                d=max(0,s[0]-axis,axis-s[1])
                if d<=best:
                    best=d;pick=s
                    if d==0: break
        else:
            overlap=[s for s in segs if s[1]>=lower['raw_l'] and s[0]<=lower['raw_r']]
            pick=overlap[-1] if overlap else None
            if not overlap: stopped=True
        if pick is None: continue
        l,r=map(float,pick);raw_l,raw_r=l,r
        fl='edge' if l<=0 else 'jump' if lower and lower['fl']=='real' and abs(l-lower['raw_l'])>p['max_jump_frac']*w else 'real'
        fr='edge' if r>=w-1 else 'jump' if lower and lower['fr']=='real' and abs(r-lower['raw_r'])>p['max_jump_frac']*w else 'real'
        reference=lower['r']-lower['l'] if lower and lower['fl']==lower['fr']=='real' else expected
        if fl!='real' and fr=='real': l=r-reference
        elif fl=='real' and fr!='real':
            r=l+reference
            if fr=='edge' and raw_r-raw_l>=p['two_lane_ratio']*expected and (lower is None or lower['lanes']==2):
                r=l+max(reference,2*expected)
        elif fl!='real' and fr!='real':
            center=(lower['l']+lower['r'])/2 if lower else (raw_l+raw_r)/2
            l,r=center-reference/2,center+reference/2
        l,r=max(l,raw_l),min(r,raw_r)
        if l>r: l,r=raw_l,raw_r
        width=r-l;target=r-.25*width if width>=p['two_lane_ratio']*expected else (l+r)/2
        if midpoint: target=(raw_l+raw_r)/2
        bad=int(fl!='real')+int(fr!='real');odd=width<.6*expected or width>p['two_lane_ratio']*1.5*expected
        conf=np.clip(1-.3*(bad+int(odd))-(.1 if bad==2 else 0),0,1)
        lower={'y':y,'l':l,'r':r,'raw_l':raw_l,'raw_r':raw_r,'target':target,'conf':conf,'fl':fl,'fr':fr,'segs':len(segs),'lanes':2 if width>=p['two_lane_ratio']*expected else 1};rows.append(lower)
    usable=[r for r in rows if r['conf']>=p['corridor_conf_min']]
    if len(usable)<max(2,p['min_valid_rows']): return None,rows
    y=np.array([r['y'] for r in usable]);targets=np.array([r['target'] for r in usable]);weights=np.array([r['conf'] for r in usable]);center=np.average(y,weights=weights)
    degree=2 if len(usable)>=3 else 1
    a=np.vander(y-center,degree+1,increasing=True)*np.sqrt(weights[:,None]);b=targets*np.sqrt(weights)
    k=np.linalg.lstsq(a,b,rcond=None)[0];look=p['lookahead_frac']*h
    x=sum(c*(look-center)**i for i,c in enumerate(k));nearest=min(usable,key=lambda r:abs(r['y']-look))
    return float(np.clip(round(x,6),nearest['l'],nearest['r'])),rows


def main():
    found=inventory();results=[];selected=[]
    ranges={'7795':[(101,188)],'7886':[(788,867),(989,1013)],'8111':[(99,203)],'9393':[(2013,2155)],
            '9084':[(2386,2505),(2925,3032)],'10547':[(1925,2315),(2670,2724),(2798,2933)],
            '16925':[(629,762),(1013,1096)],'13546':[(1538,1656),(2217,2295)],'16488':[(2088,2196)]}
    probes={'7795':[150,185],'7886':[850,1005],'8111':[180],'9393':[2040,2110],
            '9084':[2430,2475,2990],'10547':[2250,2700,2850],'16925':[700,1050,1080],'13546':[1600],'16488':[2150]}
    for name,paths in found.items():
        key=name.split('_')[-1]
        if key not in ranges: continue
        path=paths[0];p=params(path/'control.yaml');records=read(path/'control_log.csv');seen=set();candidates=[]
        model='t94' if json.loads((path/'session.json').read_text(encoding='utf-8'))['unet_onnx_sha256'].startswith('1e1964') else 't95'
        for i,r in enumerate(records):
            if not any(a<=i<=b for a,b in ranges[key]) or not r['mask_file'] or r['mask_file'] in seen: continue
            seen.add(r['mask_file']);mask=cv2.imread(str(path/r['mask_file']),0)
            if mask is None: continue
            base,br=corridor(mask,p);right,rr=corridor(mask,p,True);mid,mr=corridor(mask,p,True,True)
            item={'session':name,'model':model,'row':i,'frame_id':r['frame_id'],'recorded_look':r['look_x'],'baseline':base,'rightmost':right,'midpoint':mid,
                  'delta':right-base if right is not None and base is not None else None,'width':mask.shape[1],
                  'split_rows':sum(x['segs']>1 for x in br),'wide_rows':sum(x['raw_r']-x['raw_l']>=.9*mask.shape[1] for x in br),
                  'e':r['e'],'cmd_deg':r['cmd_deg'],'mask':str(path/r['mask_file'])}
            results.append(item);candidates.append((i,r,mask,base,right,mid,br,rr))
        for target in probes.get(key,[]):
            if not candidates: continue
            candidate=min(candidates,key=lambda x:abs(x[0]-target));selected.append((name,path,candidate))
    write('mask_points.csv',results)
    # Ảnh bằng chứng: ảnh gốc trên, mặt nạ chỉ số lớp và hàng/điểm nhìn phía dưới.
    tiles=[]
    for name,path,(i,r,mask,base,right,mid,br,rr) in selected:
        image=cv2.imread(str(path/'images'/(r['frame_id']+'.jpg')))
        if image is None: image=np.zeros((180,320,3),np.uint8)
        image=cv2.resize(image,(320,180));h,w=mask.shape
        overlay=np.zeros((h,w,3),np.uint8);overlay[mask==1]=(65,110,65);overlay[mask==2]=(110,80,50)
        for row in br: cv2.circle(overlay,(int(row['target']),row['y']),2,(0,200,255),-1)
        for row in rr: cv2.circle(overlay,(int(row['target']),row['y']),2,(255,100,255),-1)
        for x,color in [(base,(0,255,255)),(right,(255,0,255)),(mid,(255,255,0))]:
            if x is not None:cv2.circle(overlay,(int(np.clip(x,0,w-1)),int(.75*h)),4,color,1)
        overlay=cv2.resize(overlay,(320,180),interpolation=cv2.INTER_NEAREST)
        tile=np.vstack([image,overlay]);cv2.putText(tile,name.split('_')[-1]+' row '+str(i),(4,15),0,.45,(255,255,255),1)
        tiles.append(tile)
    for offset in range(0,len(tiles),6):
        batch=tiles[offset:offset+6]
        while len(batch)<6: batch.append(np.zeros_like(tiles[0]))
        cv2.imwrite(str(OUT/('evidence_'+str(offset//6+1)+'.jpg')),np.vstack([np.hstack(batch[:3]),np.hstack(batch[3:])]))
    print('Mặt nạ đã đo:',len(results),'ảnh minh chứng:',len(selected))
    for name in sorted(set(r['session'] for r in results)):
        subset=[r for r in results if r['session']==name and r['delta'] is not None];d=[r['delta'] for r in subset]
        print(name,'n',len(subset),'đổi',sum(abs(v)>1e-5 for v in d),'delta px min/median/max',np.round([min(d),np.median(d),max(d)],2) if d else [])


if __name__=='__main__':main()
