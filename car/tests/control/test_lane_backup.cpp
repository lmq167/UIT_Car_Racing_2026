#include "control/LaneController.h"
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <cstdio>
using namespace control;
void Check(bool ok,const char *message) { if(!ok) throw std::runtime_error(message); }
bool Bits(double a,double b) { return std::memcmp(&a,&b,sizeof(a))==0; }
cv::Mat Road(int l,int r) {
    cv::Mat m=cv::Mat::zeros(192,320,CV_8UC1);
    if(r>=l) m.colRange(l,r+1).setTo(1);
    return m;
}
int main(int argc,char **argv) {
    try {
        Check(argc==2,"Can duong dan YAML tam");
        ControlParams p; p.lane1_width_frac={0.3,0.3};
        LaneController a,b;
        int64_t ms=0;
        for(const auto &m:{Road(112,208),Road(64,256),Road(100,319),Road(0,319),Road(0,-1),Road(120,216)}) {
            ms+=50; auto q=p; q.lane_backup=0;
            auto x=a.Step(m,ms,ms,p), y=b.Step(m,ms,ms,q);
            Check(Bits(x.cmd.cms,y.cmd.cms) && Bits(x.cmd.deg,y.cmd.deg) &&
                  Bits(x.debug.e,y.debug.e) && Bits(x.debug.look_x,y.debug.look_x) &&
                  x.debug.fit==y.debug.fit && x.debug.state==y.debug.state &&
                  x.debug.reason==y.debug.reason,"Tat phai bit-giong");
        }
        p.lane_backup=1;
        auto step=[&](const cv::Mat &m) { LaneController c; return c.Step(m,50,50,p); };
        auto one=step(Road(112,208));
        Check(one.debug.look_x>=0 && std::abs(one.debug.e)<0.01,"Mot lan thang");
        auto two=step(Road(64,256));
        Check(std::abs(two.debug.look_x-208)<1e-5 && two.debug.e>0,"Hai lan bam phai");
        auto cut=step(Road(100,319));
        Check(cut.debug.e>0 && cut.debug.look_x>160,"Cat mep phai van bam phai");
        auto full=step(Road(0,319));
        Check(std::abs(full.debug.look_x-159.5)<1e-5,"Hai mep lay tam");
        Check(full.debug.fit.empty() && full.debug.valid_rows==p.corridor_rows,"Debug du phong");
        Check(step(Road(0,-1)).debug.look_x<0 && step(cv::Mat{}).debug.state=="MAT_LAN","Rong mat lan");
        double lx=0; int rows=0;
        Check(BackupLookahead(Road(140,180),p,lx,rows) && std::abs(lx-160)<1e-5,"Doan hep lay tam");
        Check(!BackupLookahead(Road(0,30),p,lx,rows),"Hang gan phai chua truc");
        Check(!BackupLookahead(Road(159,161),p,lx,rows),"Bo nhieu hep");
        cv::Mat broken=Road(112,208);
        const int second=int(std::lround((p.corridor_bottom_frac+
            (p.corridor_top_frac-p.corridor_bottom_frac)/(p.corridor_rows-1))*broken.rows));
        broken.row(second).setTo(0);
        Check(!BackupLookahead(broken,p,lx,rows),"Khong noi qua hang trong");
        for(auto mode:{TrajMode::AVOID_LEFT,TrajMode::RETURN_RIGHT,TrajMode::ROUNDABOUT,TrajMode::TURN_LEFT,TrajMode::TURN_RIGHT}) {
            LaneController c,d; auto off=p; off.lane_backup=0;
            auto x=c.Step(Road(100,319),50,50,p,mode), y=d.Step(Road(100,319),50,50,off,mode);
            Check(Bits(x.cmd.deg,y.cmd.deg) && Bits(x.debug.look_x,y.debug.look_x),"Che do khac khong doi");
        }
        auto load=[&](const std::string &body) {
            { std::ofstream f(argv[1]); f<<"%YAML:1.0\n---\n"<<body; }
            std::ostringstream report; return ControlParams::Load(argv[1],report);
        };
        auto missing=load("kp: 35\n");
        Check(missing.lane_backup==0 && missing.lane_backup_right_frac==0.5,"YAML thieu khoa");
        for(const auto &body:{"lane_backup: -1\n","lane_backup: 2\n","lane_backup: 0.5\n",
            "lane_backup_right_frac: 0.349\n","lane_backup_right_frac: 0.651\n"}) {
            bool rejected=false; try { load(body); } catch(const std::exception &) { rejected=true; }
            Check(rejected,"YAML ngoai mien phai bi tu choi");
        }
        for(double f:{0.35,0.65}) Check(load("lane_backup: 1\nlane_backup_right_frac: "+std::to_string(f)+"\n").lane_backup_right_frac==f,"Bien mien hop le");
        std::remove(argv[1]);
        std::cout<<"DAT: lane_backup va YAML\n"; return 0;
    } catch(const std::exception &e) { std::cerr<<e.what()<<'\n'; return 1; }
}
