// Chuyển làn khi né (pair_mode=1): theo vạch phân làn, kiểm bằng mặt nạ tổng hợp hai làn.
#include "control/LaneController.h"
#include "control/Planner.h"
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void Check(bool ok, const std::string &message) { if(!ok) throw std::runtime_error(message); }
control::ControlParams Params(int pair_mode) {
    control::ControlParams p; p.pair_mode=pair_mode; p.kd=0; p.kp=35; return p;
}
// Mặt nạ 96x160, hai làn: vạch phân làn D, vạch phải R, (tuỳ chọn) vạch trái O; dịch ngang `shift` điểm ảnh sang phải.
cv::Mat Road(int shift, bool with_right=true, bool with_left=true, bool with_divider=true) {
    cv::Mat m(96,160,CV_8UC1,cv::Scalar(1));
    const int y_row[3]={81,62,43};
    const int d[3]={30,36,48}, w[3]={112,88,64};
    for(int y=0;y<96;++y) {
        int i=y>=72 ? 0 : (y>=52 ? 1 : 2);
        (void)y_row;
        const int D=d[i]+shift;
        auto put=[&](int x) { if(x>=0 && x<160) m.at<unsigned char>(y,x)=2; };
        if(with_divider) put(D);
        if(with_right) put(D+w[i]);
        if(with_left) put(D-w[i]);
    }
    return m;
}
control::ControlResult Run(control::LaneController &c, const cv::Mat &mask, const control::ControlParams &p, int &t, int steps=3) {
    control::ControlResult r;
    for(int k=0;k<steps;++k) { r=c.Step(mask,t,t,p,control::LaneRef::RIGHT); t+=50; }
    return r;
}
}
int main(int argc, char **argv) {
    try {
        Check(argc==2,"Thiếu tên ca");
        const std::string name=argv[1]; int t=1000;
        const auto p=Params(1);
        if(name=="left") {
            control::LaneController c; c.SetSwerve(1);
            auto r=Run(c,Road(0,true,false),p,t);
            Check(r.debug.state=="BAM_LAN" && r.cmd.deg>20,"Né trái phải lái trái mạnh khi xe ở làn phải");
            control::LaneController n; auto normal=Run(n,Road(0,true,false),p,t);
            Check(std::abs(normal.cmd.deg)<10,"Đối chứng: bám làn thường ở giữa làn phải");
        } else if(name=="arrived") {
            control::LaneController c; c.SetSwerve(1);
            Run(c,Road(0,true,false),p,t);
            int shift=0; control::ControlResult r;
            for(shift=0;shift<=106;shift+=10) r=Run(c,Road(shift,true,false),p,t,2);
            r=Run(c,Road(106,false,true),p,t,3);
            Check(r.debug.state=="BAM_LAN" && std::abs(r.cmd.deg)<8,"Xe đã ở giữa làn trái thì không lái thêm");
        } else if(name=="lost_divider") {
            control::LaneController c; c.SetSwerve(1);
            Run(c,Road(0,true,false),p,t);
            control::ControlResult r;
            for(int i=0;i<200;++i) r=Run(c,Road(0,false,false,false),p,t,1);
            Check(r.debug.state!="BAM_LAN" && r.cmd.cms==0,"Mất vạch phân làn quá lâu thì phải dừng, không lái mù");
        } else if(name=="return_right") {
            control::LaneController c; c.SetSwerve(-1);
            auto r=Run(c,Road(106,false,true),p,t);
            Check(r.debug.state=="BAM_LAN" && r.cmd.deg<-15 && r.cmd.deg>-20,"Về phải từ làn trái lái phải vừa phải (sai số kẹp 0,5)");
        } else if(name=="keep_across_reset") {
            control::LaneController c; c.SetSwerve(1);
            Run(c,Road(0,true,false),p,t);
            c.Reset(true); c.SetSwerve(-1);
            auto r=Run(c,Road(80,false,false),p,t);   // chỉ thấy vạch phân làn D
            Check(r.debug.state=="BAM_LAN" && r.cmd.deg<0,"Giữ vạch phân làn qua Reset khi đổi sang về phải");
        } else if(name=="reset_clears") {
            // Reset thường (Space/G/MAT_LAN_NE) không được giữ vạch phân làn cũ.
            control::LaneController c; c.SetSwerve(1);
            Run(c,Road(0,true,false),p,t);
            c.Reset(); c.SetSwerve(1);
            auto r=Run(c,Road(0,false,false,false),p,t);   // không còn vạch nào: phải mất làn, không dùng D cũ
            Check(r.debug.state!="BAM_LAN" || r.cmd.cms==0 || std::abs(r.cmd.deg)<1,"Reset thường phải xoá vạch phân làn cũ");
        } else if(name=="pair0_return_keeps_right_lost") {
            // pair_mode=0: chuyển làn không tắt luật mất vạch phải (kiểm M3).
            control::ControlParams q=Params(0); q.right_lost_ms=500;
            control::LaneController c; c.SetSwerve(-1);
            control::ControlResult r;
            for(int i=0;i<40;++i) r=Run(c,Road(106,true,false,false),q,t,1);
            control::LaneController ref; control::ControlResult rr;
            int t2=1000;
            for(int i=0;i<40;++i) rr=Run(ref,Road(106,true,false,false),q,t2,1);
            Check(r.debug.state==rr.debug.state && r.cmd.cms==rr.cmd.cms,"pair_mode=0 chuyển làn phải giống bám làn thường");
        } else if(name=="pair0_unchanged") {
            control::ControlParams q=Params(0);
            control::LaneController a,b; b.SetSwerve(1);
            const auto ra=Run(a,Road(0),q,t), rb=Run(b,Road(0),q,t);
            Check(ra.cmd.deg==rb.cmd.deg && ra.cmd.cms==rb.cmd.cms,"pair_mode=0 không đổi khi bật chuyển làn");
        } else if(name=="planner_cycle") {
            // Planner đủ chu kỳ NE_TRAI -> NE_QUA -> VE_PHAI trên đường hai làn (pair_mode=1): trái rồi về phải.
            control::ControlParams q=Params(1); q.obstacle_action=2; q.turn_enable=0; q.start_wait_green=0;
            control::Planner planner; control::PlannerInput in; int64_t now=1000;
            auto step=[&](const cv::Mat &mask) {
                in.mask=mask; in.mask_ms=now; in.image_ms=now; in.sensor_ok=true; in.sensor_ms=now;
                const auto plan=planner.Step(in,now,q); now+=50; return plan;
            };
            in.go=true; step(Road(0,true,false)); in.go=false;
            cv::Mat blocked=Road(0,true,false); blocked(cv::Rect(90,36,30,55)).setTo(0);
            control::Plan plan; int shift=0;
            for(int i=0;i<4;++i) plan=step(blocked);
            Check(plan.reason=="NE_TRAI","Có hố trước mặt phải vào NE_TRAI, được: "+plan.reason);
            Check(plan.cmd.deg>20,"NE_TRAI phải lái trái mạnh, được "+std::to_string(plan.cmd.deg));
            // Xe dịch sang trái dần (vạch phân làn trôi sang phải), vật cản hết khỏi ảnh: sang NE_QUA rồi VE_PHAI.
            bool qua=false, ve_phai=false; double min_deg=0;
            for(int i=0;i<400 && !ve_phai;++i) {
                if(shift<106) shift+=2;
                plan=step(Road(shift,false,true));
                qua=qua || plan.reason=="NE_QUA";
                if(plan.reason=="VE_PHAI") { ve_phai=true; min_deg=plan.cmd.deg; }
            }
            Check(qua,"Phải qua pha NE_QUA");
            Check(ve_phai,"Phải sang VE_PHAI sau quãng obstacle_pass_cm");
            for(int i=0;i<5;++i) { plan=step(Road(106,false,true)); if(plan.reason=="VE_PHAI") min_deg=std::min(min_deg,plan.cmd.deg); }
            Check(min_deg<-15 && min_deg>-20,"VE_PHAI lái phải êm (kẹp sai số 0,5), được "+std::to_string(min_deg));
        } else if(name=="return_end") {
            // VE_PHAI chỉ kết thúc khi đã thẳng hàng làn phải (|e| nhỏ) hoặc quá 5*obstacle_pass_cm; không cắt ở 25 cm khi còn lệch.
            control::ControlParams q=Params(1); q.obstacle_action=2; q.turn_enable=0; q.start_wait_green=0;
            auto ve_phai_steps=[&](bool aligned) {
                control::Planner planner; control::PlannerInput in; int64_t now=1000;
                auto step=[&](const cv::Mat &mask) {
                    in.mask=mask; in.mask_ms=now; in.image_ms=now; in.sensor_ok=true; in.sensor_ms=now;
                    const auto plan=planner.Step(in,now,q); now+=50; return plan;
                };
                in.go=true; step(Road(0,true,false)); in.go=false;
                cv::Mat blocked=Road(0,true,false); blocked(cv::Rect(90,36,30,55)).setTo(0);
                control::Plan plan; int shift=0;
                for(int i=0;i<4;++i) plan=step(blocked);
                for(int i=0;i<400 && plan.reason!="VE_PHAI";++i) { if(shift<106) shift+=2; plan=step(Road(shift,false,true)); }
                Check(plan.reason=="VE_PHAI","Phải vào VE_PHAI");
                int n=0;
                for(;n<2000 && plan.reason=="VE_PHAI";++n) {
                    if(aligned && shift>0) shift-=2;  // xe quay về làn phải dần: vạch phân làn trôi về trái
                    plan=step(aligned ? Road(shift,false,true) : Road(106,false,true));
                }
                return n;
            };
            const int aligned=ve_phai_steps(true), lost=ve_phai_steps(false);
            Check(aligned<lost,"Đã thẳng hàng làn phải thì kết thúc sớm hơn: "+std::to_string(aligned)+" so với "+std::to_string(lost));
            Check(lost<2000,"Còn lệch vẫn phải dừng ở trần 5*obstacle_pass_cm, được "+std::to_string(lost)+" bước");
        } else throw std::runtime_error("Ca không tồn tại: "+name);
        std::cout<<"ĐẠT: "<<name<<"\n"; return 0;
    } catch(const std::exception &e) { std::cerr<<"KHÔNG ĐẠT: "<<e.what()<<"\n"; return 1; }
}
