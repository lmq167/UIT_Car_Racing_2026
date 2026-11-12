"""Ghép ảnh gốc theo mốc dòng để duyệt bằng mắt; giữ bảng sai lệch mốc."""
import cv2
import numpy as np
from analyze_sessions import inventory,read,OUT,write

TARGETS={'19218':[100,200],'19437':[1100,1757],'6720':[30],'6840':[260],'6930':[900],'7406':[150],
         '7504':[1300,1588,2103],'7795':[180],'7886':[850,1000],'8111':[180],
         '9393':[1500,1834,2013,2425,2515,2692,2787,2980,3606,4403,6575,6627,7402,7605,8378],
         '9084':[2386,2474,2925,6985,7152],
         '10547':[1925,2234,2298,2670,2798,2942,3353,5530,5580,5843,6537,7118,7666,8017],
         '13546':[1538,1632,2217,5090,7962,8014],
         '16488':[1887,2196,2425,2646,3116],
         '16656':[223,534,3358,3641,4106,4481,4497,4626],
         '16925':[400,629,762,1013,1095,1386,1569,1572,1583,1692]}


def main():
    tiles=[];index=[]
    for name,paths in inventory().items():
        key=name.split('_')[-1];p=paths[0];rows=read(p/'control_log.csv')
        available=[(i,r,p/'images'/(r['frame_id']+'.jpg')) for i,r in enumerate(rows) if r.get('frame_id') and (p/'images'/(r['frame_id']+'.jpg')).exists()]
        for target in TARGETS.get(key,[]):
            if not available: continue
            i,r,file=min(available,key=lambda t:abs(t[0]-target));im=cv2.imread(str(file))
            if im is None:continue
            im=cv2.resize(im,(400,225));tile=np.zeros((255,400,3),np.uint8);tile[30:]=im
            label=key+' row '+str(i)+' (target '+str(target)+')'
            cv2.putText(tile,label,(4,12),0,.4,(255,255,255),1)
            cv2.putText(tile,r['reason']+' c'+r['route_step']+' yaw '+str(round(float(r['yaw_deg']),1)) if r['yaw_deg'] else r['reason'],(4,26),0,.4,(255,255,255),1)
            sheet=len(tiles)//9+1
            index.append({'session':name,'target':target,'row':i,'offset_rows':i-target,'frame_id':r['frame_id'],'reason':r['reason'],'image':str(file),'sheet':'stages_'+str(sheet)+'.jpg'})
            tiles.append(tile)
    for offset in range(0,len(tiles),9):
        batch=tiles[offset:offset+9]
        while len(batch)<9:batch.append(np.zeros_like(tiles[0]))
        cv2.imwrite(str(OUT/('stages_'+str(offset//9+1)+'.jpg')),np.vstack([np.hstack(batch[i:i+3]) for i in (0,3,6)]))
    write('image_evidence_index.csv',index)
    print('Ảnh duyệt:',len(tiles),'bảng:',(len(tiles)+8)//9)


if __name__=='__main__':main()
