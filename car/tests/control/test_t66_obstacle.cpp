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
    Plan left() {
        p.obstacle_action=2; in.mask=Mask(); Plan plan;
        for(int n=1;n<=p.vote_min;++n) {
            plan=step();
            Check((plan.reason=="NE_TRAI")== (n==p.vote_min),"Né sai số phiếu");
            if(n<p.vote_min) Check(step(false).reference==LaneRef::RIGHT,"Đếm trùng phiếu né");
        }
        Check(plan.state==PlannerState::BAM_LAN && plan.reference==LaneRef::LEFT &&
              plan.cmd.cms>0 && plan.cmd.cms<=p.approach_cms,"Pha trái sai tham chiếu/tốc độ");
        LaneController expected;
        const auto left=expected.Step(in.mask,in.mask_ms,now,p,LaneRef::LEFT);
        Check(std::abs(plan.debug.e-left.debug.e)<1e-12,"Planner không truyền LEFT vào bộ bám làn");
        return plan;
    }
    Plan pass() {
        left(); in.mask=Mask(0,0); Plan plan;
        for(int n=1;n<=p.vote_window;++n) {
            plan=step();
            Check(plan.reason==(n==p.vote_window ? "NE_QUA" : "NE_TRAI"),"Qua sai chuỗi sạch");
            if(n<p.vote_window) Check(step(false).reason=="NE_TRAI","Mặt nạ trùng làm qua");
        }
        return plan;
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
    Check(defaults.obstacle_action==0 && defaults.obstacle_hole_frac==0.10 && defaults.obstacle_pass_cm==40,"Mặc định YAML");
    Check(report.str().find("obstacle_action =")==std::string::npos &&
          report.str().find("obstacle_hole_frac =")==std::string::npos &&
          report.str().find("obstacle_pass_cm =")==std::string::npos,"YAML thiếu khóa mới");
    auto load=[&](const std::string &body) {
        { std::ofstream out(temporary); out << "%YAML:1.0\n---\n" << body; }
        return ControlParams::Load(temporary,report);
    };
    const auto missing=load("{}");
    Check(missing.obstacle_action==0 && missing.obstacle_hole_frac==0.10,"Mặc định khi thiếu khóa");
    for(bool action:{false,true}) {
        auto invalid=missing;
        if(action) invalid.obstacle_action=3; else invalid.obstacle_hole_frac=0.01;
        bool rejected=false;
        try { invalid.Validate(); } catch(const std::exception &) { rejected=true; }
        Check(rejected,"Validate bỏ qua miền khóa mới");
    }
    for(const auto &body:{"obstacle_action: 0\nobstacle_hole_frac: 0.02\n",
                          "obstacle_action: 1\nobstacle_hole_frac: 0.5\n",
                          "obstacle_action: 2\nobstacle_pass_cm: 10\n", "obstacle_pass_cm: 200\n"}) load(body);
    for(const auto &body:{"obstacle_action: -1", "obstacle_action: 3", "obstacle_action: 0.5",
                          "obstacle_action: abc", "obstacle_action: [1]", "obstacle_hole_frac: 0.019",
                          "obstacle_hole_frac: 0.501", "obstacle_hole_frac: .Nan",
                          "obstacle_hole_frac: abc", "obstacle_hole_frac: [0.1]",
                          "obstacle_pass_cm: 9.99", "obstacle_pass_cm: 200.01",
                          "obstacle_pass_cm: .Nan", "obstacle_pass_cm: abc", "obstacle_pass_cm: [40]"}) {
        bool rejected=false;
        try { load(std::string(body)+"\n"); } catch(const std::exception &) { rejected=true; }
        Check(rejected,"Phải từ chối sai kiểu/miền");
    }
}
void SwerveTests() {
    { Rig r; auto plan=r.pass(); double distance=0;
      for(int n=0;n<1000;++n) {
          distance+=plan.cmd.cms*0.05;
          plan=r.step();
          if(distance+1e-9<r.p.obstacle_pass_cm) {
              Check(plan.reason=="NE_QUA" && plan.reference==LaneRef::LEFT,"Về trước đủ quãng");
          } else {
              Check(plan.reason=="VE_PHAI" && plan.reference==LaneRef::RIGHT,"Không về đúng quãng");
              Check(r.step().reason=="BAM_LAN","VE_PHAI không chỉ một nhịp");
              r.left(); break;
          }
          Check(n<999,"Không kết thúc né");
      }
    }
    { Rig r; r.p.obstacle_pass_cm=10; r.pass();
      for(int n=0;n<10;++n) r.step();
      r.in.mask=Mask();
      for(int n=1;n<=r.p.vote_min;++n)
          Check(r.step().reason==(n==r.p.vote_min ? "NE_TRAI" : "NE_QUA"),"Lỗ lại không đủ phiếu về trái");
      r.in.mask=Mask(0,0); Plan plan;
      for(int n=0;n<r.p.vote_window;++n) plan=r.step();
      double distance=0;
      while(distance+plan.cmd.cms*0.05<r.p.obstacle_pass_cm-1e-9) {
          distance+=plan.cmd.cms*0.05; plan=r.step();
          Check(plan.reason=="NE_QUA","Không xóa quãng khi lỗ lại");
      }
      Check(r.step().reason=="VE_PHAI","Lỗ lại làm kẹt pha");
    }
    { Rig r; r.p.obstacle_hole_frac=0.2; r.left();
      r.in.mask=Mask(100,20); // Tỉ lệ giữa ngưỡng thả 0,1 và ngưỡng vào 0,2.
      for(int n=0;n<10;++n) Check(r.step().reason=="NE_TRAI","Ngưỡng trễ né bị thả sớm");
      r.in.mask=Mask(100,5); for(int n=0;n<r.p.vote_window-1;++n) r.step();
      r.in.mask=Mask(100,20); Check(r.step().reason=="NE_TRAI","Không ngắt chuỗi sạch");
      r.in.mask=Mask(0,0); for(int n=1;n<=r.p.vote_window;++n)
          Check(r.step().reason==(n==r.p.vote_window ? "NE_QUA" : "NE_TRAI"),"Chuỗi ngưỡng trễ né");
    }
    for(bool passing:{false,true}) for(int cancel=0;cancel<4;++cancel) {
        Rig r; if(passing) r.pass(); else r.left();
        if(cancel==0) { r.in.stop=true; Check(r.step().reason=="SPACE","Space khi né"); r.in.stop=false; }
        if(cancel==1) { r.now+=kStaleImageMs+1;
            Check(r.planner.Step(r.in,r.now,r.p).reason=="MAT_ANH","Mất ảnh khi né"); }
        if(cancel==2) r.planner.Reset(true);
        if(cancel==3) { r.in.mask=cv::Mat(100,200,CV_8UC1,cv::Scalar(0));
            auto plan=r.step();
            Check(plan.state==PlannerState::DUNG && plan.cmd.cms==0 && plan.cmd.deg==0 &&
                  plan.reason=="MAT_LAN_NE" && plan.reference==LaneRef::RIGHT &&
                  plan.debug.state=="DUNG" && plan.debug.reason=="MAT_LAN_NE" &&
                  plan.debug.speed_cms==0 && plan.debug.angle_deg==0,"Mất làn khi né phải dừng hẳn"); }
        r.in.mask=Mask(0,0);
        if(cancel!=2) for(int n=0;n<r.p.regain_frames+2;++n) {
            const auto stopped=r.step();
            Check(stopped.state==PlannerState::DUNG && stopped.cmd.cms==0 && stopped.cmd.deg==0 &&
                  stopped.reference==LaneRef::RIGHT,"Hủy né tự chạy trước G mới");
            if(cancel==3) Check(stopped.reason=="MAT_LAN_NE","Mất lý do chốt mất làn né");
        }
        r.in.go=true;
        auto plan=r.step(); r.in.go=false;
        Check(plan.state==PlannerState::BAM_LAN && plan.reference==LaneRef::RIGHT &&
              plan.reason=="BAM_LAN" && plan.cmd.cms>0,"G mới không bám lại RIGHT");
        for(int n=0;n<r.p.regain_frames+1;++n) plan=r.step();
        Check(plan.reference==LaneRef::RIGHT && plan.reason=="BAM_LAN","Hủy pha giữ tham chiếu trái");
        r.left();
    }
    for(bool passing:{false,true}) for(bool stale:{false,true}) {
        Rig r; if(passing) r.pass(); else r.left();
        if(stale) r.now+=r.p.mask_max_age_ms;
        else { r.in.mask=cv::Mat(100,200,CV_8UC1,cv::Scalar(0)); r.now+=r.p.lost_stop_ms; }
        const auto plan=r.step(!stale);
        Check(plan.state==PlannerState::DUNG && plan.reason=="MAT_LAN_NE" &&
              plan.cmd.cms==0 && plan.cmd.deg==0 && plan.reference==LaneRef::RIGHT,
              "MAT_LAN/MAT_NAN_CU khi né phải dừng hẳn");
    }
    { Rig r; r.p.obstacle_pass_cm=10; auto plan=r.pass(); double distance=0;
      while(distance+plan.cmd.cms*0.05<r.p.obstacle_pass_cm-1e-9) {
          distance+=plan.cmd.cms*0.05; plan=r.step();
      }
      r.in.mask=cv::Mat(100,200,CV_8UC1,cv::Scalar(0)); plan=r.step();
      Check(plan.state==PlannerState::BAM_LAN && plan.reference==LaneRef::RIGHT &&
            plan.reason=="MAT_LAN" && plan.cmd.cms==0 && plan.cmd.deg==0,
            "Đã đủ quãng về phải không được chốt MAT_LAN_NE"); }
    for(int action:{0,1,2}) {
        Rig r; r.p.obstacle_action=action; LaneController expected;
        expected.Step(r.in.mask,r.in.mask_ms,r.now,r.p,LaneRef::RIGHT);
        for(int n=0;n<30;++n) {
            r.in.mask=n<2 || n>=20 ? Mask(0,0) : cv::Mat(100,200,CV_8UC1,cv::Scalar(0));
            const auto plan=r.step();
            const auto lane=expected.Step(r.in.mask,r.in.mask_ms,r.now,r.p,LaneRef::RIGHT);
            Check(plan.state==PlannerState::BAM_LAN && plan.reference==LaneRef::RIGHT &&
                  plan.cmd.cms==lane.cmd.cms && plan.cmd.deg==lane.cmd.deg &&
                  plan.reason==lane.debug.reason && plan.debug.state==lane.debug.state,
                  "Mất làn ngoài pha né/action 0/1 đổi hành vi cũ");
        }
    }
    { Rig r; r.p.obstacle_pass_cm=10; auto plan=r.pass(); double distance=0;
      for(int n=0;n<100;++n) {
          const int dt=n%2 ? 130 : 70;
          distance+=plan.cmd.cms*dt/1000.0;
          r.p.approach_cms=n%2 ? 2 : 5;
          r.now+=dt-50; plan=r.step();
          if(distance<r.p.obstacle_pass_cm-1e-9)
              Check(plan.reason=="NE_QUA","Nhịp không đều/tốc độ thay đổi cộng sai quãng");
          else { Check(plan.reason=="VE_PHAI","Nhịp không đều không về đủ quãng"); break; }
          Check(n<99,"Nhịp không đều kẹt pha");
      }
    }
    { Rig r; r.p.obstacle_pass_cm=10; r.pass(); r.p.approach_cms=0;
      for(int n=0;n<250;++n) Check(r.step().reason=="NE_QUA","Tốc độ 0 vẫn cộng quãng"); }
    for(int action:{1,2}) {
        Rig r; r.p.obstacle_hole_frac=16.0/161;
        if(action==1) r.latch(); else r.left();
        r.in.mask=Mask(100,8);
        for(int n=0;n<r.p.vote_window+1;++n) {
            const auto plan=r.step();
            Check(action==1 ? plan.state==PlannerState::DUNG_VAT_CAN : plan.reason=="NE_TRAI",
                  "Ngưỡng thả phải dùng nhỏ hơn, không gồm bằng");
        }
        r.p.obstacle_hole_frac=std::nextafter(r.p.obstacle_hole_frac,1.0);
        for(int n=1;n<=r.p.vote_window;++n) {
            const auto plan=r.step();
            Check(action==1 ? (plan.state==PlannerState::BAM_LAN)==(n==r.p.vote_window) :
                  plan.reason==(n==r.p.vote_window ? "NE_QUA" : "NE_TRAI"),"Biên dưới ngưỡng thả");
        }
    }
    { Rig r; r.p.obstacle_hole_frac=0.2; r.latch(); r.in.mask=Mask(100,20);
      for(int n=0;n<10;++n) Check(r.step().cmd.cms==0,"Ngưỡng trễ dừng bị thả sớm");
      r.in.mask=Mask(100,5);
      for(int n=1;n<=r.p.vote_window;++n)
          Check((r.step().state==PlannerState::BAM_LAN)==(n==r.p.vote_window),"Ngưỡng thả G1b mới"); }
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
    { Rig r; r.p.obstacle_action=0; r.planner.Reset(); LaneController lane;
      for(int n=0;n<20;++n) {
          r.in.mask=n%2 ? Mask() : Mask(0,0); r.in.go=n==0 || n==12; r.in.stop=n==10;
          r.now+=50; r.in.mask_ms=r.now; r.in.image_ms=r.now;
          const auto a=r.planner.Step(r.in,r.now,r.p);
          if(n>=10 && n<12) {
              Check(a.state==PlannerState::DUNG && a.cmd.cms==0 && a.cmd.deg==0,"Action 0 bỏ qua Space");
              lane.Reset();
          } else {
              const auto b=lane.Step(r.in.mask,r.now,r.now,r.p,r.in.reference);
              Check(a.state==PlannerState::BAM_LAN && a.cmd.cms==b.cmd.cms && a.cmd.deg==b.cmd.deg &&
                    a.reason==b.debug.reason && a.reference==r.in.reference,"Action 0 đổi bám làn gốc");
          }
      }
    }
}
}
int main(int argc, char **argv) {
    try {
        Check(argc==3,"Cần control.yaml và đường dẫn YAML tạm");
        Params(argv[1],argv[2]); Unit(); SwerveTests(); std::cout << "PASS T66\n";
        return 0;
    } catch(const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
