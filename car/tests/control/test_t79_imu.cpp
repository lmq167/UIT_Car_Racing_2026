#include "control/Planner.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
using namespace control;
void Check(bool ok,const std::string &why) { if(!ok) throw std::runtime_error(why); }
struct Rig {
    Planner planner;
    ControlParams p;
    PlannerInput in;
    Plan last;
    int64_t now=1000;
    uint64_t frame=0;
    Rig(bool enabled=true,bool imu=false) {
        p.imu_fallback=enabled; p.start_wait_green=0; p.obstacle_action=0;
        p.lane1_width_frac={0.4,0.4}; p.kd=0;
        in.sensor_ok=imu; in.encoder_ok=true; in.yolo_enabled=true;
        in.yaw_deg=imu ? 0 : std::numeric_limits<double>::quiet_NaN();
        mask(); in.go=true; step(); in.go=false;
        for(int i=0;i<8;++i) step();
    }
    // 0: làn thẳng; 1/2: mở trái/phải; 3: đảo ở xa; 4: mất hoàn toàn.
    void mask(int shape=0) {
        in.mask=cv::Mat(100,200,CV_8UC1,cv::Scalar(0));
        if(shape==4) return;
        for(int y=0;y<100;++y) {
            int l=shape==1 ? 0 : 60, r=shape==2 ? 199 : 140;
            if(shape==3) { l=20; r=180; }
            in.mask.row(y).colRange(l,r+1).setTo(1);
            if(shape==3 && y<90) in.mask.row(y).colRange(85,116).setTo(0);
        }
    }
    Plan step(const std::string &object="",double cm=1,int dt=50,bool new_mask=true,bool new_batch=true) {
        now+=dt; in.odo_cm+=cm; in.image_ms=in.sensor_ms=now;
        if(new_mask) in.mask_ms=now;
        if(new_batch) {
            in.detections={{},now,++frame};
            if(!object.empty()) in.detections.items.push_back({object,
                object=="red" ? cv::Rect2f(1150,100,100,80) :
                (object=="person" ? cv::Rect2f(600,626,80,80) : object=="round_barrier" ? cv::Rect2f(490,100,300,80) : cv::Rect2f(600,100,80,80)),0.9f,now});
        }
        last=planner.Step(in,now,p);
        Check(std::isfinite(last.cmd.cms) && last.cmd.cms>=0 && last.cmd.cms<=p.max_speed_cms,"Vượt trần tốc độ");
        Check(std::isfinite(last.cmd.deg) && std::abs(last.cmd.deg)<=p.steer_max_deg,"Vượt trần góc");
        return last;
    }
    void arm(bool right=false) { for(int i=0;i<p.vote_min;++i) step(right?"right":"left"); }
    void start(bool right=false) {
        arm(right); mask(right?2:1);
        const double anchor=in.odo_cm;
        while(in.odo_cm-anchor<0.5*p.turn_start_cm) step();
        Check(last.reason==(right?"RE_HANH_LANG_PHAI":"RE_HANH_LANG_TRAI"),"Không bắt đầu cua hành lang: "+last.reason);
        Check(last.cmd.cms==p.min_cms,"Cua phải dùng min_cms");
    }
    void pass(bool flicker=false) {
        mask(3);
        bool approach=false, around=false;
        for(int i=0;i<8;++i) {
            step("round_barrier");
            approach=approach || last.reason=="VX_TIEP_CAN";
            around=around || last.reason=="VX_QUANH_DAO";
            if(flicker && i>=3) { step(); step("round_barrier"); }
        }
        Check(last.mode==TrajMode::ROUNDABOUT,"Phải bám vòng xuyến");
        Check(around && (!p.imu_fallback || approach),"Thiếu lý do VX tiếp cận/quanh đảo");
        mask();
        for(int i=0;i<7;++i) step();
        Check(last.reason=="VX_THOAT","Phải chờ đủ quãng mất phiếu: "+last.reason);
        step("",p.rb_lost_cm);
        Check(last.mode==TrajMode::LANE,"Phải kết thúc lần qua");
    }
    void exit(bool right) {
        step("",right?p.rb_turn2_cm:p.rb_turn1_cm);
        mask(right?2:1);
        for(int i=0;i<15;++i) step();
        Check(last.reason==(right?"RE_HANH_LANG_PHAI":"RE_HANH_LANG_TRAI"),"Sai nhánh thoát: "+last.reason);
        mask(); for(int i=0;i<p.vote_min;++i) step();
        Check(last.state==PlannerState::BAM_LAN && last.mode==TrajMode::LANE,"Không kết thúc cua theo mask liên tiếp");
    }
};
void Run(const std::string &name) {
    if(name=="roundabout" || name=="flicker") {
        Rig r; r.pass(name=="flicker"); r.exit(false); r.pass(name=="flicker"); r.exit(true);
    } else if(name=="disabled") {
        Rig r(false); r.arm(); r.mask(1);
        for(int i=0;i<100;++i) Check(r.step().state==PlannerState::BAM_LAN,"Tắt dự phòng phải không cua khi IMU lỗi");
        r.pass();
        for(int i=0;i<100;++i) Check(r.step().state==PlannerState::BAM_LAN,"Tắt dự phòng không cua sau vòng xuyến");
    } else if(name=="glitch") {
        Rig r(true,true); r.arm(); r.mask(1);
        for(int i=0;i<r.p.vote_min;++i) r.step();
        Check(r.last.state==PlannerState::RE_TRAI,"Phải vào cua IMU trước Q9");
        for(int repeat=0;repeat<3;++repeat) {
            r.in.sensor_ok=false;
            for(int i=0;i<5;++i) {
                auto p=r.step(); Check(p.reason=="CAM_BIEN_CHO" && p.cmd.cms==0,"Q9 <250 ms phải giữ cua, dừng tốc độ");
                Check(p.mode!=TrajMode::TURN_LEFT,"Q9 không được chuyển dự phòng sớm");
            }
            r.in.sensor_ok=true;
            Check(r.step().reason=="RE_TRAI","Q9 hồi mẫu tươi phải tiếp tục cua");
        }
        // Cờ gộp vẫn đúng nhưng mẫu quá 200 ms cũng phải dừng giữ cua.
        r.now+=50; r.in.mask_ms=r.in.image_ms=r.now; r.in.sensor_ms=r.now-201;
        const auto stale=r.planner.Step(r.in,r.now,r.p);
        Check(stale.reason=="CAM_BIEN_CHO" && stale.cmd.cms==0,"Tuổi mẫu >200 ms phải đi qua Q9");
    } else if(name=="encoder") {
        Rig r; r.in.encoder_ok=false; r.arm(); r.mask(1);
        for(int i=0;i<100;++i) Check(r.step().state==PlannerState::BAM_LAN,"Encoder lỗi không được cua");
        r.mask(4); for(int i=0;i<20;++i) r.step();
        Check(r.last.cmd.cms==0 && r.last.cmd.deg==0,"Mất làn và encoder lỗi phải dừng");
        Rig active; active.start(); active.in.encoder_ok=false;
        Check(active.step().state==PlannerState::DUNG && active.last.cmd.cms==0 && active.last.cmd.deg==0,"Encoder lỗi giữa cua phải dừng");
        Rig rb; rb.pass(); rb.in.encoder_ok=false;
        for(int i=0;i<100;++i) Check(rb.step().state==PlannerState::BAM_LAN,"Encoder lỗi không được dùng quãng cũ để cua");
        Rig counting; counting.mask(3);
        for(int i=0;i<5;++i) counting.step("round_barrier");
        counting.in.encoder_ok=false; counting.mask();
        for(int i=0;i<8;++i) counting.step("",50);
        counting.in.encoder_ok=true;
        Check(counting.step().mode==TrajMode::ROUNDABOUT,"Không được tính quãng đi khi encoder lỗi");
        counting.step("",counting.p.rb_lost_cm);
        Check(counting.last.mode==TrajMode::LANE,"Encoder hồi phải đo lại từ mốc tốt");
        counting.exit(false);
    } else if(name=="repeat") {
        Rig r; r.step("round_barrier");
        for(int i=0;i<5;++i) Check(r.step("",0,50,true,false).mode==TrajMode::LANE,"Lặp cùng lô không được thêm phiếu");
        r.arm(); r.mask(1);
        const double anchor=r.in.odo_cm;
        for(int i=0;i<15;++i) r.step("left");
        Check(r.last.reason=="RE_HANH_LANG_TRAI","Biển lặp không được dời mốc");
        r.mask(); r.step();
        for(int i=0;i<4;++i) Check(r.step("",0,50,false).state==PlannerState::RE_TRAI,"Mask lặp không được kết thúc cua");
        for(int i=0;i<3;++i) r.step();
        Check(r.last.state==PlannerState::BAM_LAN,"Đủ mask mới phải kết thúc cua");
        Check(r.in.odo_cm-anchor<r.p.turn_arm_cm,"Fixture trong hạn chờ");
    } else if(name=="no_junction") {
        Rig r; r.arm(); bool cancelled=false;
        for(int i=0;i<100;++i) {
            auto p=r.step(); cancelled=cancelled || p.reason=="HUY_RE";
            Check(p.state==PlannerState::BAM_LAN,"Không giao lộ không được rẽ mù");
        }
        Check(cancelled,"Quá turn_arm_cm phải HUY_RE");
        Rig wrong; wrong.arm(); wrong.mask(2);
        for(int i=0;i<70;++i) Check(wrong.step().state==PlannerState::BAM_LAN,"Giao lộ ngược hướng không được cua");
    } else if(name=="barrier_fault" || name=="barrier_junction") {
        Rig r;
        Check(r.now-1000>=250 && !r.in.sensor_ok && r.in.encoder_ok,
              "IMU phải lỗi ít nhất 250 ms, encoder vẫn tốt");
        const auto barrier=[&](double cm=0) {
            const int64_t captured=r.now+50;
            r.in.detections={{{"work_barrier",cv::Rect2f(575,500,130,100),0.9f,captured}},captured,++r.frame};
            return r.step("",cm,50,true,false);
        };
        for(int i=1;i<=r.p.vote_min;++i) barrier();
        const auto stopped=[&]() {
            Check(r.last.cmd.cms==0 && r.last.reason=="DUNG_RAO_LOI_CAM_BIEN",
                  "Rào đủ phiếu phải dừng mọi khung, không đi thẳng: "+r.last.reason);
            Check(r.last.mode==TrajMode::LANE,"Không giao lộ thì không được cua");
        };
        stopped();
        // Giữ rào đủ phiếu, kể cả đã vượt quãng dự phòng vẫn không đi thẳng.
        for(int i=0;i<20;++i) { barrier(); stopped(); }
        if(name=="barrier_fault") {
            for(int i=0;i<2*r.p.turn_start_cm;++i) { barrier(1); stopped(); }
        } else {
            r.mask(1);
            for(int i=1;i<r.p.vote_min;++i) { barrier(); stopped(); }
            barrier();
            // Khung bắt đầu có thể còn lệnh dừng Q7; khung sau phải bám cua trái.
            Check(r.last.state==PlannerState::RE_TRAI && r.last.mode==TrajMode::TURN_LEFT,
                  "Giao lộ trái đủ quãng phải vào cua hành lang");
            for(int i=0;i<5;++i) {
                barrier();
                Check(r.last.reason=="RE_HANH_LANG_TRAI" && r.last.cmd.cms==r.p.min_cms,
                      "Phải tiếp tục cua hành lang trái: "+r.last.reason);
            }
        }
    } else if(name=="branch") {
        Rig r; r.mask(3);
        const auto l=ExtractCorridor(r.in.mask,r.p,TrajMode::TURN_LEFT);
        const auto rr=ExtractCorridor(r.in.mask,r.p,TrajMode::TURN_RIGHT);
        Check(l.rows.back().raw_r<rr.rows.back().raw_l,"Phải chọn hai đoạn trái/phải khác nhau quanh đảo");
        for(const auto *c:{&l,&rr}) for(const auto &row:c->rows) if(row.valid)
            Check(row.target>=row.raw_l && row.target<=row.raw_r,"Mục tiêu phải nằm trên nhánh được chọn");
    } else {
        Rig r; r.start();
        if(name=="max_cm" || name=="max_ms") {
            if(name=="max_cm") r.step("",r.p.turn_max_cm+1);
            else r.step("",0,r.p.turn_max_ms+1);
            Check(r.last.state==PlannerState::DUNG && r.last.reason=="RE_HANH_LANG_HET_HAN" && r.last.cmd.cms==0 && r.last.cmd.deg==0,"Quá hạn phải dừng 0,0: "+r.last.reason);
        } else if(name=="space" || name=="image") {
            if(name=="space") { r.in.stop=true; r.step(); }
            else { r.now+=kStaleImageMs+1; r.last=r.planner.Step(r.in,r.now,r.p); }
            Check(r.last.state==PlannerState::DUNG && r.last.cmd.cms==0 && r.last.cmd.deg==0,"Space/mất ảnh phải dừng 0,0");
        } else if(name=="edge") {
            r.mask(2); for(int i=0;i<8;++i) r.step();
            r.mask(4); for(int i=0;i<20;++i) r.step();
            Check(r.last.cmd.cms==0 && r.last.cmd.deg==0,"Mất làn trong cua phải dừng theo hạn làn");
            Rig rb; rb.mask(3);
            for(int i=0;i<5;++i) rb.step("round_barrier");
            rb.mask(4);
            for(int i=0;i<20;++i) Check(rb.step("round_barrier").state!=PlannerState::GIU_HUONG,"IMU lỗi không được giữ hướng bằng yaw");
            Check(rb.last.cmd.cms==0 && rb.last.cmd.deg==0,"Mất làn quanh đảo phải dừng theo hạn làn");
        } else if(name=="finish") {
            r.mask(); for(int i=1;i<r.p.vote_min;++i) Check(r.step().state==PlannerState::RE_TRAI,"Chưa đủ mask kết thúc");
            r.mask(1); r.step(); r.mask();
            for(int i=1;i<r.p.vote_min;++i) Check(r.step().state==PlannerState::RE_TRAI,"Mask giao lộ phải xóa chuỗi kết thúc");
            Check(r.step().state==PlannerState::BAM_LAN,"Đủ mask hợp lệ liên tiếp phải về làn");
        } else if(name=="red" || name=="person") {
            for(int i=0;i<r.p.vote_min;++i) {
                r.step(name=="red" ? "red" : "person",0);
            }
            Check(r.last.cmd.cms==0,"Đỏ/người phải dừng cua");
            if(name=="red") Check(r.last.state==PlannerState::DUNG_DEN_DO,"Đỏ phải chốt trạng thái dừng");
        } else throw std::runtime_error("Ca lạ: "+name);
    }
}
}
int main(int argc,char **argv) {
    try { Check(argc==2,"Cần tên ca"); Run(argv[1]); std::cout<<"PASS "<<argv[1]<<'\n'; return 0; }
    catch(const std::exception &e) { std::cerr<<"FAIL: "<<e.what()<<'\n'; return 1; }
}
