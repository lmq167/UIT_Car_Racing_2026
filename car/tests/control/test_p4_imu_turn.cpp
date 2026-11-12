#include "control/Planner.h"
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>

// P4_TEST, Quân 11/11: kỳ vọng chỉ lấy từ phiếu, không từ mã triển khai.
namespace {
using namespace control;
void Check(bool ok, const std::string &s) { if (!ok) throw std::runtime_error(s); }
bool Turning(const Plan &a) { return a.state==PlannerState::RE_TRAI || a.state==PlannerState::RE_PHAI; }
struct Rig {
    Planner planner; ControlParams p; PlannerInput in; Plan last;
    int64_t now=1000; uint64_t frame=0;
    Rig(bool healthy=true, double yaw=0) {
        p.start_wait_green=0; p.turn_enable=1; p.park_enable=0;
        p.steer_max_deg=40; p.max_speed_cms=20; p.kd=0;
        p.lane1_width_frac={0.4,0.4};
        in.sensor_ok=healthy; in.encoder_ok=true; in.yolo_enabled=true;
        in.yaw=in.yaw_deg=yaw; mask(); in.go=true; tick(); in.go=false;
        for(int i=0;i<8;++i) tick();
    }
    // Tỉ lệ hình học của đầu vào giả: 0 làn; 1 mở trái; 2 mở phải; 3 đường rộng.
    void mask(int shape=0) {
        const int h=100,w=200;
        in.mask=cv::Mat(h,w,CV_8UC1,cv::Scalar(0));
        for(int y=0;y<h;++y) {
            int l=static_cast<int>(w*0.30), r=static_cast<int>(w*0.70);
            if(shape==3) { l=0; r=w-1; }
            if(y>=h*0.79 && shape==1) l=0;
            if(y>=h*0.79 && shape==2) r=w-1;
            in.mask.row(y).colRange(l,r+1).setTo(1);
        }
    }
    std::string info() const {
        std::ostringstream s;
        s<<"t="<<now<<" odo="<<in.odo_cm<<" yaw="<<in.yaw_deg
         <<" state="<<StateName(last.state)<<" reason="<<last.reason
         <<" deg="<<last.cmd.deg<<" cms="<<last.cmd.cms
         <<" target="<<planner.Turn().target<<" end="<<planner.Turn().end;
        return s.str();
    }
    void bounded() const {
        Check(std::isfinite(last.cmd.deg) && std::abs(last.cmd.deg)<=p.steer_max_deg+1e-8,
              "Lệnh lái vượt trần: "+info());
        Check(std::isfinite(last.cmd.cms) && last.cmd.cms>=0 && last.cmd.cms<=p.max_speed_cms+1e-8,
              "Lệnh tốc độ vượt trần: "+info());
    }
    Plan tick(const std::string &label="", double cm=0, int dt=40, int age=0,
              bool mask_new=true, float width=80, float cx=640) {
        now+=dt; in.odo_cm+=cm; in.encoder=in.odo_cm;
        in.sensor_ms=now-age; in.image_ms=now;
        if(mask_new) in.mask_ms=now;
        in.detections={{},now,++frame};
        if(!label.empty()) in.detections.items.push_back({label=="box" ? "obstacle" : label,
            cv::Rect2f(cx-width/2, label=="box" ? 500:100,width,160),0.9f,now});
        last=planner.Step(in,now,p); bounded(); return last;
    }
    void start(bool right=false) {
        for(int i=0;i<p.vote_min;++i) tick(right?"right":"left");
        mask(right?2:1);
        for(int i=0;i<p.vote_min;++i) tick();
        Check(last.state==(right?PlannerState::RE_PHAI:PlannerState::RE_TRAI)
              && planner.Turn().active,"Thiết lập chưa vào rẽ IMU: "+info());
        Check(planner.Turn().target>10,"Đích rẽ không hợp lệ: "+info());
    }
};
int passed=0,failed=0;
void Case(const std::string &name,const std::function<void()> &fn) {
    try { fn(); ++passed; std::cout<<name<<" PASS\n"; }
    catch(const std::exception &e) { ++failed; std::cout<<name<<" FAIL "<<e.what()<<'\n'; }
}
double Wrap(double yaw) { return std::remainder(yaw,360.0); }
void Yaw(bool right,bool wrapped,double initial) {
    Rig r(true,initial); r.start(right);
    const double target=r.planner.Turn().target, yaw0=r.planner.Turn().yaw0;
    const double sign=right?-1:1;
    r.mask();
    for(double progress : {2.0,20.0,target-10.1}) {
        r.in.yaw=r.in.yaw_deg=wrapped?Wrap(yaw0+sign*progress):yaw0+sign*progress;
        for(int i=0;i<r.p.vote_min+1;++i) {
            r.tick(); Check(Turning(r.last),"Bàn giao khi yaw chưa đủ: "+r.info());
        }
    }
    r.in.yaw=r.in.yaw_deg=wrapped?Wrap(yaw0+sign*(target-9.9)):yaw0+sign*(target-9.9);
    for(int i=0;i<r.p.vote_min+2 && Turning(r.last);++i) r.tick();
    Check(r.last.state==PlannerState::BAM_LAN && !r.planner.Turn().active,
          "Đủ yaw/hành lang chưa bàn giao: "+r.info());
}
void Deadline(bool distance) {
    Rig r; r.start(); const int64_t begin=r.now; const double odo=r.in.odo_cm;
    int count=0;
    while(Turning(r.last) && r.now-begin<=r.p.turn_max_ms+40 && count++<2000) {
        r.tick("",distance?5:0);
        if(distance && r.in.odo_cm-odo>r.p.turn_max_cm)
            Check(!Turning(r.last),"Vượt hạn cm vẫn rẽ: "+r.info());
    }
    Check(!Turning(r.last) && !r.planner.Turn().active,"Quá hạn ms vẫn rẽ: "+r.info());
    std::cout<<"  hạn "<<(distance?"cm":"ms")<<" dt="<<r.now-begin<<" cm="<<r.in.odo_cm-odo<<'\n';
}
void Return(bool blocked) {
    Rig r; r.p.obstacle_action=2; r.p.obstacle_pass_cm=10;
    r.p.lane1_width_frac={0.8,0.8}; r.p.avoid_min_width_ratio=0.4;
    r.in.mask=cv::Mat(100,200,CV_8UC1,cv::Scalar(0));
    r.in.mask.colRange(20,181).setTo(1);
    r.in.mask(cv::Rect(90,40,40,55)).setTo(0);
    bool avoided=false,returned=false;
    for(int i=0;i<40;++i) {
        r.tick("obstacle",1,40,0,true,128,664);
        avoided=avoided || r.last.mode==TrajMode::AVOID_LEFT;
        if(avoided) break;
    }
    Check(avoided,"Thiết lập chưa vào NE_TRAI: "+r.info());
    r.in.mask.colRange(20,181).setTo(1);
    for(int i=0;i<300;++i) {
        r.tick("",1);
        if(r.last.mode==TrajMode::RETURN_RIGHT) { returned=true; break; }
    }
    Check(returned,"Thiết lập chưa vào VE_PHAI: "+r.info());
    if(blocked) {
        for(int i=0;i<r.p.vote_window+2;++i) {
            r.tick("box",0,40,0,true,128,640);
            if(r.last.reason=="KHONG_DUONG_NE") break;
        }
        Check(r.last.reason=="KHONG_DUONG_NE" && r.last.cmd.cms==0,
              "Vật sát khi về phải chưa dừng: "+r.info());
    } else {
        r.in.mask.setTo(0); r.in.mask.colRange(30,191).setTo(1); r.tick("",1);
        Check(r.last.mode==TrajMode::RETURN_RIGHT,"Chưa giữ VE_PHAI: "+r.info());
        Check(std::abs(r.last.debug.e)>1e-6,"Thiết lập sai số về phải bằng 0");
        // Độ lợi đã là 3,5 tại HEAD; đổi hàng nhìn không đổi hệ số này.
        const double expected=3.5*r.p.kp*r.last.debug.e;
        Check(std::abs(r.last.debug.p-expected)<1e-6,
              "P về phải: thực="+std::to_string(r.last.debug.p)+" kỳ vọng="+std::to_string(expected)+" "+r.info());
    }
}
}
int main() {
    // Nhóm 1: dấu yaw, quấn và hai điều kiện bàn giao bắt buộc (P4).
    for(bool right:{false,true}) for(bool wrapped:{false,true})
        for(double magnitude:{179.0,540.0})
            Case("N1_yaw_"+std::to_string(right)+"_"+std::to_string(wrapped)+"_"+std::to_string(magnitude),
                 [=]{Yaw(right,wrapped,right?-magnitude:magnitude);});
    Case("N1_du_yaw_thieu_hanh_lang",[]{
        Rig r; r.start(); r.in.mask.setTo(0); r.in.mask.colRange(0,141).setTo(1);
        r.in.yaw=r.in.yaw_deg=r.planner.Turn().yaw0+r.planner.Turn().target-9;
        for(int i=0;i<r.p.vote_min+3;++i) { r.tick(); Check(Turning(r.last),"Thiếu hành lang vẫn bàn giao: "+r.info()); }
    });
    // Nhóm 2: tiến độ yaw và hạn cố định; mọi bước kiểm trần lệnh.
    Case("N2_cham_tien_do",[]{
        Rig r; r.start(); const auto begin=r.now; const double odo=r.in.odo_cm;
        bool boosted=false;
        for(int i=0;i<200 && Turning(r.last);++i) {
            r.tick("",1);
            if(r.now-begin<r.p.turn_max_ms && r.in.odo_cm-odo<r.p.turn_max_cm)
                boosted=boosted || std::abs(r.last.cmd.deg)>r.p.turn_steer_deg+1e-6;
        }
        Check(boosted,"Yaw đứng yên chưa tăng lái trước hạn: "+r.info());
    });
    Case("N2_dung_tien_do",[]{
        Rig r; r.start(); const double target=r.planner.Turn().target, yaw0=r.planner.Turn().yaw0;
        for(int i=1;i<=12;++i) {
            r.in.yaw=r.in.yaw_deg=yaw0+target*0.06*i;
            r.tick("",r.p.turn_max_cm*0.03);
            Check(std::abs(r.last.cmd.deg)<=r.p.turn_steer_deg+1e-6,
                  "Đủ tiến độ vẫn tăng lái: "+r.info());
        }
    });
    Case("N2_han_ms",[]{Deadline(false);}); Case("N2_han_cm",[]{Deadline(true);});
    Case("N2_han_gio_thuan",[]{
        Rig r; r.start(); const auto begin=r.now; const double odo=r.in.odo_cm;
        // Bánh lăn 2 cm mỗi 400 ms: không kẹt, tổng quãng vẫn dưới hạn cm.
        while(r.now-begin<=r.p.turn_max_ms && Turning(r.last)) r.tick("",0.2);
        Check(r.in.odo_cm-odo<r.p.turn_max_cm,"Thiết lập chạm hạn cm: "+r.info());
        Check(r.now-begin>r.p.turn_max_ms,"Thoát trước hạn giờ thuần: "+r.info());
        Check(!Turning(r.last) && !r.planner.Turn().active,"Quá hạn giờ thuần vẫn rẽ: "+r.info());
        std::cout<<"  hạn giờ thuần dt="<<r.now-begin<<" cm="<<r.in.odo_cm-odo<<'\n';
    });
    // Nhóm 3: lỗi từ đầu, lỗi giữa lượt và hành vi về làn phải (P4).
    Case("N3_IMU_loi_tu_dau",[]{
        Rig r(false); r.in.yaw=r.in.yaw_deg=std::numeric_limits<double>::quiet_NaN();
        for(int i=0;i<r.p.vote_min;++i) r.tick("left");
        r.mask(1);
        for(int i=0;i<30;++i) {
            r.tick("",1); Check(r.last.state==PlannerState::BAM_LAN,"IMU lỗi từ đầu vẫn rẽ: "+r.info());
        }
    });
    for(int age:{201,250,400}) Case("N3_mau_cu_"+std::to_string(age),[=]{
        Rig r; r.start(); r.tick("",0,40,age);
        Check(r.last.cmd.cms==0 || r.last.reason=="HUY_RE_CAM_BIEN","Mẫu cũ không dừng/hủy: "+r.info());
    });
    Case("N3_encoder_mat_mau",[]{
        Rig r; r.start(); r.in.sensor_ok=r.in.encoder_ok=false;
        r.tick(); const auto bad=r.now;
        while(r.now-bad<=250) {
            Check(r.last.cmd.cms==0 && r.last.reason=="CAM_BIEN_CHO" && Turning(r.last),
                  "Encoder mất mẫu trong 250 ms chưa dừng chờ: "+r.info());
            r.tick();
        }
        Check(!Turning(r.last) && !r.planner.Turn().active &&
              (r.last.reason=="HUY_RE_CAM_BIEN" || r.planner.Turn().end=="HUY_CAM_BIEN"),
              "Encoder mất mẫu quá 250 ms chưa hủy: "+r.info());
    });
    Case("N3_ve_phai_P",[]{Return(false);}); Case("N3_ve_phai_vat_sat",[]{Return(true);});
    // Nhóm 4: seed cố định; chia lượt để bảo đảm có lượt rẽ thực, không chỉ chạy bám làn.
    Case("N4_ngau_nhien_3000_buoc",[]{
        std::mt19937 rng(0x5044494d); int steps=0, turns=0; double yaw=0;
        std::string first_error; int violations=0;
        for(int episode=0;episode<30;++episode) {
            Rig r; r.start(episode%2); ++turns;
            int64_t begin=r.now; double odo=r.in.odo_cm;
            for(int i=0;i<100;++i) {
                yaw+=static_cast<int>(rng()%41)-20+0.2;
                const unsigned kind=rng()%20;
                r.in.yaw=r.in.yaw_deg=kind==0?std::numeric_limits<double>::quiet_NaN():
                    kind==1?std::numeric_limits<double>::infinity():kind==2?
                    -std::numeric_limits<double>::infinity():yaw;
                r.in.encoder_ok=rng()%10!=0;
                r.in.sensor_ok=(rng()%10!=0) && r.in.encoder_ok &&
                    std::isfinite(r.in.yaw_deg) && std::isfinite(r.in.odo_cm);
                r.mask(static_cast<int>(rng()%4));
                const bool wasActive=r.planner.Turn().active;
                const int age=static_cast<int>(rng()%401);
                r.tick("",rng()%4,40,age,rng()%5!=0); ++steps;
                if(!wasActive && r.planner.Turn().active) { begin=r.now; odo=r.in.odo_cm; ++turns; }
                const bool validOdo=r.in.encoder_ok && age<=200 && std::isfinite(r.in.odo_cm);
                if((r.now-begin>r.p.turn_max_ms || (validOdo && r.in.odo_cm-odo>r.p.turn_max_cm))
                   && (Turning(r.last) || r.planner.Turn().active)) {
                    ++violations;
                    if(first_error.empty()) first_error="Ngẫu nhiên quá hạn vẫn rẽ: "+r.info();
                }
            }
            // Kết thúc bằng đầu vào tươi sau hạn, kể cả lượt cuối chưa tới hạn trong 100 bước.
            r.in.sensor_ok=r.in.encoder_ok=true; r.in.yaw=r.in.yaw_deg=0;
            r.tick("",0,r.p.turn_max_ms+1); ++steps;
            Check(!Turning(r.last) && !r.planner.Turn().active,"Lượt ngẫu nhiên không kết thúc: "+r.info());
        }
        std::cout<<"  seed=0x5044494d steps="<<steps<<" turns="<<turns<<'\n';
        Check(first_error.empty(),first_error+" số bước vi phạm="+std::to_string(violations));
    });
    std::cout<<"TOTAL PASS="<<passed<<" FAIL="<<failed<<'\n';
    return failed?1:0;
}
