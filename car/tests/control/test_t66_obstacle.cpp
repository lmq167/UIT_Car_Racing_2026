#include "control/Planner.h"
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace {
using namespace control;
void Check(bool ok, const char *why) { if(!ok) throw std::runtime_error(why); }
cv::Mat Mask(int first=90, int width=40) {
    cv::Mat mask(100,200,CV_8UC1,cv::Scalar(0));
    mask.colRange(20,181).setTo(1);
    mask.col(20).setTo(2); mask.col(180).setTo(2);
    if(width) mask(cv::Rect(first,40,width,55)).setTo(0);
    return mask;
}
struct Rig {
    Planner planner;
    ControlParams p;
    PlannerInput in;
    int64_t now=1000;
    Rig() {
        p.obstacle_action=1; p.start_wait_green=0; p.right_lost_ms=0;
        in.mask=Mask(0,0); in.go=true; step(); in.go=false;
    }
    Plan step(bool new_mask=true) {
        now+=50; in.image_ms=now;
        if(new_mask) in.mask_ms=now;
        return planner.Step(in,now,p);
    }
    Plan latch() {
        in.mask=Mask(); Plan plan;
        for(int n=1;n<=p.vote_min;++n) {
            plan=step();
            Check((plan.state==PlannerState::DUNG_VAT_CAN)==(n==p.vote_min),"Dừng sai số phiếu");
            if(n<p.vote_min) Check(step(false).state==PlannerState::BAM_LAN,"Đếm lại mặt nạ trùng");
        }
        Check(plan.reason=="VAT_CAN" && plan.cmd.cms==0 && plan.cmd.deg==0,"Lệnh dừng vật cản");
        Check(plan.debug.state=="DUNG_VAT_CAN" && plan.debug.reason=="VAT_CAN","Thông tin dừng vật cản");
        return plan;
    }
};
void Params(const char *yaml, const char *temporary) {
    struct Cleanup { const char *p; ~Cleanup() { std::remove(p); } } cleanup{temporary};
    std::ostringstream report;
    const auto defaults=ControlParams::Load(yaml,report);
    Check(defaults.obstacle_action==0 && defaults.obstacle_hole_frac==0.10,"Mặc định YAML");
    Check(report.str().find("obstacle_action =")==std::string::npos &&
          report.str().find("obstacle_hole_frac =")==std::string::npos,"YAML thiếu khóa mới");
    auto load=[&](const std::string &body) {
        { std::ofstream out(temporary); out << "%YAML:1.0\n---\n" << body; }
        return ControlParams::Load(temporary,report);
    };
    const auto missing=load("{}");
    Check(missing.obstacle_action==0 && missing.obstacle_hole_frac==0.10,"Mặc định khi thiếu khóa");
    for(bool action:{false,true}) {
        auto invalid=missing;
        if(action) invalid.obstacle_action=2; else invalid.obstacle_hole_frac=0.01;
        bool rejected=false;
        try { invalid.Validate(); } catch(const std::exception &) { rejected=true; }
        Check(rejected,"Validate bỏ qua miền khóa mới");
    }
    for(const auto &body:{"obstacle_action: 0\nobstacle_hole_frac: 0.02\n",
                          "obstacle_action: 1\nobstacle_hole_frac: 0.5\n"}) load(body);
    for(const auto &body:{"obstacle_action: -1", "obstacle_action: 2", "obstacle_action: 0.5",
                          "obstacle_action: abc", "obstacle_action: [1]", "obstacle_hole_frac: 0.019",
                          "obstacle_hole_frac: 0.501", "obstacle_hole_frac: .Nan",
                          "obstacle_hole_frac: abc", "obstacle_hole_frac: [0.1]"}) {
        bool rejected=false;
        try { load(std::string(body)+"\n"); } catch(const std::exception &) { rejected=true; }
        Check(rejected,"Phải từ chối sai kiểu/miền");
    }
}
void Unit() {
    for(bool above:{false,true}) {
        Rig r; r.in.mask=Mask(); r.p.obstacle_hole_frac=40.0/161;
        if(above) r.p.obstacle_hole_frac=std::nextafter(r.p.obstacle_hole_frac,1.0);
        Plan plan;
        for(int n=0;n<r.p.vote_min;++n) plan=r.step();
        Check((plan.state==PlannerState::DUNG_VAT_CAN)==!above,"Biên tỉ lệ lỗ dùng sai >=");
    }
    for(int center:{69,70,170,171}) {
        Rig r; r.in.mask=Mask(center-10,21);
        r.in.mask.colRange(181,196).setTo(1); r.in.mask.col(195).setTo(2);
        Plan plan;
        for(int n=0;n<r.p.vote_min;++n) plan=r.step();
        Check((plan.state==PlannerState::DUNG_VAT_CAN)==(center>=70 && center<=170),"Biên hành lang trọng tâm");
    }
    { Rig r; r.latch(); r.in.mask=Mask(0,0);
      for(int n=1;n<=r.p.vote_window;++n) {
          const auto plan=r.step();
          Check((plan.state==PlannerState::BAM_LAN)==(n==r.p.vote_window),"Thả sai số mặt nạ sạch");
          if(n<r.p.vote_window) Check(r.step(false).cmd.cms==0,"Mặt nạ trùng làm thả");
          else Check(plan.cmd.cms>0,"Không phục hồi bám làn");
      }
    }
    { Rig r; r.latch(); r.in.mask=Mask(0,0); r.step(); r.step();
      r.in.mask=Mask(); r.step(); r.in.mask=Mask(0,0);
      for(int n=1;n<r.p.vote_window;++n) Check(r.step().cmd.cms==0,"Chuỗi sạch phải liên tiếp");
      Check(r.step().state==PlannerState::BAM_LAN,"Không thả sau chuỗi sạch mới"); }
    for(auto mask:{Mask(25,35),Mask(100,5),cv::Mat(100,200,CV_8UC1,cv::Scalar(0))}) {
        Rig r; r.in.mask=mask;
        for(int n=0;n<12;++n) Check(r.step().state==PlannerState::BAM_LAN,"Lỗ ngoài hành lang/nhỏ/rỗng gây dừng");
    }
    { Rig r; r.in.mask=Mask(); r.step();
      r.now+=r.p.mask_max_age_ms;
      for(int n=0;n<5;++n) Check(r.step(false).state==PlannerState::BAM_LAN,"Mặt nạ cũ tạo phiếu");
      Check(r.step().state==PlannerState::BAM_LAN,"Mặt nạ cũ đã tạo phiếu ẩn");
      Check(r.step().state==PlannerState::DUNG_VAT_CAN,"Thiếu phiếu mới sau mặt nạ cũ"); }
    { Rig r; r.in.mask=Mask();
      for(int n=0;n<5;++n) {
          r.now+=50; r.in.image_ms=r.now; r.in.mask_ms=r.now+1;
          Check(r.planner.Step(r.in,r.now,r.p).state==PlannerState::BAM_LAN,"Mặt nạ tương lai tạo phiếu");
      }
      for(int n=1;n<=r.p.vote_min;++n)
          Check((r.step().state==PlannerState::DUNG_VAT_CAN)==(n==r.p.vote_min),"Phiếu sau mặt nạ tương lai"); }
    { Rig r; r.in.yolo_enabled=true; r.latch();
      for(int n=1;n<=r.p.vote_min;++n) {
          const auto ms=r.now+50;
          r.in.detections={{{"red",cv::Rect2f(1050,100,200,100),1.0f,ms}},ms,uint64_t(n)};
          const auto plan=r.step();
          Check(plan.cmd.cms==0 && plan.cmd.deg==0,"Đèn đỏ/vật cản phát lệnh chạy");
          if(n==r.p.vote_min) Check(plan.state==PlannerState::DUNG_DEN_DO,"Vật cản che chốt đèn đỏ");
      }
    }
    { Rig r; r.latch(); r.in.stop=true;
      Check(r.step().reason=="SPACE","Space không dừng"); r.in.stop=false; r.in.mask=Mask(0,0);
      for(int n=0;n<8;++n) Check(r.step().state==PlannerState::DUNG,"Tự mở chốt Space");
      r.in.go=true; Check(r.step().state==PlannerState::BAM_LAN,"G mới không chạy lại"); }
    { Rig r; r.latch(); r.now+=kStaleImageMs+1;
      auto plan=r.planner.Step(r.in,r.now,r.p);
      Check(plan.state==PlannerState::DUNG && plan.reason=="MAT_ANH" && plan.cmd.cms==0 && plan.cmd.deg==0,"Mất ảnh không chốt dừng");
      Check(r.step().state==PlannerState::DUNG,"Ảnh về tự chạy");
      r.in.go=true; r.in.mask=Mask(0,0); Check(r.step().state==PlannerState::BAM_LAN,"G sau mất ảnh"); }
    { Rig r; r.in.mask=Mask(); r.step(); r.step(); r.planner.Reset(true);
      r.in.go=true; Check(r.step().state==PlannerState::BAM_LAN,"Reset giữ phiếu vật cản"); }
    { Rig r; r.p.vote_window=3; r.p.vote_min=2; r.p.green_frames=2;
      r.in.mask=Mask(); r.step(); r.in.mask=Mask(0,0); r.step(); r.step(); r.step();
      r.in.mask=Mask(); Check(r.step().state==PlannerState::BAM_LAN,"Phiếu ngoài cửa sổ còn hiệu lực");
      Check(r.step().state==PlannerState::DUNG_VAT_CAN,"Cửa sổ 2/3 không dừng"); }
    { Rig r; r.p.obstacle_action=0; r.planner.Reset(); Planner old; ControlParams p=r.p;
      for(int n=0;n<20;++n) {
          r.in.mask=n%2 ? Mask() : Mask(0,0); r.in.go=n==0; r.in.stop=n==10;
          r.now+=50; r.in.mask_ms=r.now; r.in.image_ms=r.now;
          const auto a=r.planner.Step(r.in,r.now,r.p), b=old.Step(r.in,r.now,p);
          Check(a.state==b.state && a.cmd.cms==b.cmd.cms && a.cmd.deg==b.cmd.deg && a.reason==b.reason,"Tắt vật cản đổi hành vi");
      }
    }
}
}
int main(int argc, char **argv) {
    try {
        Check(argc==3,"Cần control.yaml và đường dẫn YAML tạm");
        Params(argv[1],argv[2]); Unit(); std::cout << "PASS T66\n";
        return 0;
    } catch(const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
