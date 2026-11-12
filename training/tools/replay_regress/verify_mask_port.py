"""Kiểm phép đo Python bằng LaneController C++ nguyên bản trên mặt nạ thật."""
import subprocess
from pathlib import Path
from replay_regress import ENV,run,SCRATCH
from mask_probe import corridor,params
from analyze_sessions import read
import cv2


def main():
    source=max(SCRATCH.glob('build_*/round3_final/ucr_car'),key=lambda p:p.stat().st_mtime)
    cpp=SCRATCH/'verify_mask.cpp'
    cpp.write_text('''#include "control/LaneController.h"
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <iomanip>
int main(int argc,char** argv) {
 auto p=control::ControlParams::Load(argv[1],std::cerr);
 for(int i=2;i<argc;++i) { control::LaneController lane;
   auto mask=cv::imread(argv[i],cv::IMREAD_UNCHANGED);
   auto r=lane.Step(mask,100,100,p,control::TrajMode::ROUNDABOUT,false);
   std::cout<<std::setprecision(12)<<r.debug.look_x<<"\\n";
 }
}''',encoding='utf-8')
    flags=subprocess.check_output(['pkg-config','--cflags','opencv4'],env=ENV,text=True).split()
    exe=SCRATCH/'verify_mask.exe'
    with (SCRATCH/'verify_mask_build.log').open('w',encoding='utf-8') as log:
        run(['g++','-std=c++17','-O2','-I'+str(source/'include'),cpp,source/'src/control/LaneController.cpp',source/'src/control/ControlParams.cpp']+flags+['-lopencv_core','-lopencv_imgproc','-lopencv_imgcodecs','-o',exe],stdout=log,stderr=subprocess.STDOUT)
    allrows=read(SCRATCH/'mask_points.csv');differences=[];count=0
    for name in sorted(set(r['session'] for r in allrows)):
        rows=[r for r in allrows if r['session']==name and r['baseline']]
        chosen=rows[::max(1,len(rows)//8)][:8]
        if not chosen:continue
        paths=[Path(r['mask']) for r in chosen];yaml=paths[0].parents[1]/'control.yaml'
        result=subprocess.run([str(exe),str(yaml)]+[str(p) for p in paths],env=ENV,capture_output=True,text=True,encoding='utf-8',errors='replace',timeout=60,check=True)
        actual=[float(v) for v in result.stdout.splitlines()]
        for row,value in zip(chosen,actual):
            expected=float(row['baseline']);differences.append(abs(expected-value));count+=1
            if abs(expected-value)>1e-5: print(name,row['row'],'Python',expected,'C++',value)
    maximum=max(differences)
    text='Mau kiem: {}; sai lech lon nhat Python/C++: {:.9g} px\n'.format(count,maximum)
    (SCRATCH/'verify_mask_result.txt').write_text(text,encoding='utf-8');print(text)
    if maximum>1e-5:raise RuntimeError('Phép đo Python chưa khớp C++')


if __name__=='__main__':main()
