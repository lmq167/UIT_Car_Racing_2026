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
    uint64_t frame=0;
    bool auto_box=true; // Các ca cũ giữ hình học mask, bổ sung nguồn hộp độc lập ở ca T85.
    Rig() {
        p.obstacle_action=1; p.start_wait_green=0;
        p.avoid_min_width_ratio=0.4;
        p.lane1_width_frac={0.8,0.8};  // đường cột 20..180 là một làn
        in.yolo_enabled=true; in.mask=Mask(0,0); in.go=true; step(); in.go=false;
    }
    Plan step(bool new_mask=true) {
        now+=50; in.image_ms=now;
        if(new_mask) in.mask_ms=now;
        if(new_mask && auto_box) {
            in.detections={{},now,++frame};
            if(in.mask.at<unsigned char>(70,100)==0 && cv::countNonZero(in.mask)>0)
                in.detections.items.push_back({"obstacle",cv::Rect2f(600,100,128,100),0.9f,now});
        }
        return planner.Step(in,now,p);
    }
    Plan left() {
        p.obstacle_action=2; in.mask=Mask(); Plan plan;
        for(int n=1;n<=p.vote_min;++n) {
            plan=step();
            Check((plan.reason=="NE_TRAI")== (n==p.vote_min),"Né sai số phiếu");
            if(n<p.vote_min) Check(step(false).mode==TrajMode::LANE,"Đếm trùng phiếu né");
        }
        Check(plan.state==PlannerState::BAM_LAN && plan.mode==TrajMode::AVOID_LEFT &&
              plan.cmd.cms>0 && plan.cmd.cms<=8.0+1e-9 && plan.cmd.cms>p.approach_cms,"Pha trái sai tham chiếu/tốc độ");
        LaneController expected;
        const auto left=expected.Step(in.mask,in.mask_ms,now,p,TrajMode::AVOID_LEFT);
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
    Check(defaults.obstacle_action==2 && defaults.obstacle_hole_ratio==0.25 && defaults.obstacle_pass_cm==85,"Mặc định YAML");
    Check(report.str().find("obstacle_action =")==std::string::npos &&
          report.str().find("obstacle_hole_ratio =")==std::string::npos &&
          report.str().find("obstacle_pass_cm =")==std::string::npos,"YAML thiếu khóa mới");
    auto load=[&](const std::string &body) {
        { std::ofstream out(temporary); out << "%YAML:1.0\n---\n" << body; }
        return ControlParams::Load(temporary,report);
    };
    { std::ifstream input(yaml); std::ostringstream old; std::string line;
      while(std::getline(input,line)) if(line.find("obstacle_near_w_frac:")!=0) old<<line<<'\n';
      { std::ofstream out(temporary); out<<old.str(); }
      const auto legacy=ControlParams::Load(temporary,report);
      Check(legacy.obstacle_near_w_frac==0.10,"YAML xe cũ thiếu khóa mới phải nạp mặc định");
    }
    load("obstacle_near_w_frac: 0.6\n");
    load("obstacle_near_w_frac: 0.020001\n");
    const auto missing=load("{}");
    Check(missing.obstacle_near_w_frac==0.10,"Mặc định khóa mới");
    Check(missing.obstacle_action==2 && missing.obstacle_hole_ratio==0.25,"Mặc định khi thiếu khóa");
    for(bool action:{false,true}) {
        auto invalid=missing;
        if(action) invalid.obstacle_action=3; else invalid.obstacle_hole_ratio=0;
        bool rejected=false;
        try { invalid.Validate(); } catch(const std::exception &) { rejected=true; }
        Check(rejected,"Validate bỏ qua miền khóa mới");
    }
    for(const auto &body:{"obstacle_action: 0\nobstacle_hole_ratio: 0.02\n",
                          "obstacle_action: 1\nobstacle_hole_ratio: 0.5\n",
                          "obstacle_action: 2\nobstacle_pass_cm: 10\n", "obstacle_pass_cm: 200\n"}) load(body);
    for(const auto &body:{"obstacle_near_w_frac: 0.02", "obstacle_near_w_frac: 0.6001",
                          "obstacle_near_w_frac: .Nan", "obstacle_near_w_frac: abc", "obstacle_near_w_frac: [0.1]",
                          "obstacle_action: -1", "obstacle_action: 3", "obstacle_action: 0.5",
                          "obstacle_action: abc", "obstacle_action: [1]", "obstacle_hole_ratio: 0",
                          "obstacle_hole_ratio: 1.001", "obstacle_hole_ratio: .Nan",
                          "obstacle_hole_ratio: abc", "obstacle_hole_ratio: [0.1]",
                          "obstacle_pass_cm: 9.99", "obstacle_pass_cm: 200.01",
                          "obstacle_pass_cm: .Nan", "obstacle_pass_cm: abc", "obstacle_pass_cm: [40]"}) {
        bool rejected=false;
        try { load(std::string(body)+"\n"); } catch(const std::exception &) { rejected=true; }
        Check(rejected,"Phải từ chối sai kiểu/miền");
    }
}
void SwerveTests() {
    { Rig r; r.p.slow_per_deg=0; const double normal_cms=r.step().cmd.cms;
      Check(normal_cms>8.0,"Thiết lập tốc độ thường phải cao hơn tốc độ né");
      auto plan=r.pass(); double distance=0;
      for(int n=0;n<1000;++n) {
          distance+=plan.cmd.cms*0.05;
          plan=r.step();
          if(distance+1e-9<r.p.obstacle_pass_cm) {
              Check(plan.reason=="NE_QUA" && plan.mode==TrajMode::AVOID_LEFT,"Về trước đủ quãng");
          } else {
              Check(plan.reason=="VE_PHAI" && plan.mode==TrajMode::RETURN_RIGHT,"Không về đúng quãng");
              double return_distance=0;
              for(int m=0;m<1000;++m) {
                  Check(plan.state==PlannerState::BAM_LAN && plan.reason=="VE_PHAI" &&
                        plan.mode==TrajMode::RETURN_RIGHT && plan.cmd.cms>0 &&
                        plan.cmd.cms<=8.0+1e-9 && plan.debug.reason=="VE_PHAI",
                        "Suốt pha về phải giữ RIGHT/VE_PHAI và kẹp tốc độ né");
                  return_distance+=plan.cmd.cms*0.05;
                  plan=r.step();
                  if(return_distance+1e-9<r.p.obstacle_pass_cm)
                      Check(plan.reason=="VE_PHAI","Kết thúc về phải trước đủ quãng");
                  else {
                      Check(plan.state==PlannerState::BAM_LAN && plan.reason=="BAM_LAN" &&
                            plan.mode==TrajMode::LANE && plan.cmd.cms==normal_cms,
                            "Hết quãng về phải phải trả BAM_LAN và tốc độ thường");
                      break;
                  }
                  Check(m<999,"Không kết thúc pha về phải");
              }
              r.left(); break;
          }
          Check(n<999,"Không kết thúc né");
      }
    }
    { Rig r; r.p.obstacle_pass_cm=10; r.pass(); r.in.mask=Mask();
      for(int n=0;n<r.p.vote_window;++n)
          Check(r.step().reason=="NE_QUA","Hộp mới không được bật lại né trong PASS");
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
                  plan.reason=="MAT_LAN_NE" && plan.mode==TrajMode::LANE &&
                  plan.debug.state=="DUNG" && plan.debug.reason=="MAT_LAN_NE" &&
                  plan.debug.speed_cms==0 && plan.debug.angle_deg==0,"Mất làn khi né phải dừng hẳn"); }
        r.in.mask=Mask(0,0);
        if(cancel!=2) for(int n=0;n<r.p.regain_frames+2;++n) {
            const auto stopped=r.step();
            Check(stopped.state==PlannerState::DUNG && stopped.cmd.cms==0 && stopped.cmd.deg==0 &&
                  stopped.mode==TrajMode::LANE,"Hủy né tự chạy trước G mới");
            if(cancel==3) Check(stopped.reason=="MAT_LAN_NE","Mất lý do chốt mất làn né");
        }
        r.in.go=true;
        auto plan=r.step(); r.in.go=false;
        Check(plan.state==PlannerState::BAM_LAN && plan.mode==TrajMode::LANE &&
              plan.reason=="BAM_LAN" && plan.cmd.cms>0,"G mới không bám lại RIGHT");
        for(int n=0;n<r.p.regain_frames+1;++n) plan=r.step();
        Check(plan.mode==TrajMode::LANE && plan.reason=="BAM_LAN","Hủy pha giữ tham chiếu trái");
        r.left();
    }
    for(bool passing:{false,true}) for(bool stale:{false,true}) {
        Rig r; if(passing) r.pass(); else r.left();
        if(stale) r.now+=r.p.mask_max_age_ms;
        else { r.in.mask=cv::Mat(100,200,CV_8UC1,cv::Scalar(0)); r.now+=r.p.lost_stop_ms; }
        const auto plan=r.step(!stale);
        Check(plan.state==PlannerState::DUNG && plan.reason=="MAT_LAN_NE" &&
              plan.cmd.cms==0 && plan.cmd.deg==0 && plan.mode==TrajMode::LANE,
              "MAT_LAN/MAT_NAN_CU khi né phải dừng hẳn");
    }
    { Rig r; r.p.obstacle_pass_cm=10; auto plan=r.pass(); double distance=0;
      while(distance+plan.cmd.cms*0.05<r.p.obstacle_pass_cm-1e-9) {
          distance+=plan.cmd.cms*0.05; plan=r.step();
      }
      r.in.mask=cv::Mat(100,200,CV_8UC1,cv::Scalar(0)); plan=r.step();
      Check(plan.state==PlannerState::BAM_LAN && plan.mode==TrajMode::RETURN_RIGHT &&
            plan.reason=="MAT_LAN" && plan.cmd.cms==0 && plan.cmd.deg==0,
            "Đã đủ quãng về phải không được chốt MAT_LAN_NE"); }
    for(int action:{0,1,2}) {
        Rig r; r.p.obstacle_action=action; LaneController expected;
        expected.Step(r.in.mask,r.in.mask_ms,r.now,r.p,TrajMode::LANE);
        for(int n=0;n<30;++n) {
            r.in.mask=n<2 || n>=20 ? Mask(0,0) : cv::Mat(100,200,CV_8UC1,cv::Scalar(0));
            const auto plan=r.step();
            const auto lane=expected.Step(r.in.mask,r.in.mask_ms,r.now,r.p,TrajMode::LANE);
            Check(plan.state==PlannerState::BAM_LAN && plan.mode==TrajMode::LANE &&
                  plan.cmd.cms==lane.cmd.cms && plan.cmd.deg==lane.cmd.deg &&
                  plan.reason==lane.debug.reason && plan.debug.state==lane.debug.state,
                  "Mất làn ngoài pha né/action 0/1 đổi hành vi cũ");
        }
    }
    { Rig r; r.p.obstacle_pass_cm=10; auto plan=r.pass(); double distance=0;
      for(int n=0;n<100;++n) {
          const int dt=n%2 ? 130 : 70;
          distance+=plan.cmd.cms*dt/1000.0;
          r.p.max_speed_cms=r.p.cruise_cms=r.p.min_cms=r.p.lost_cms=n%2 ? 2 : 5;
          r.now+=dt-50; plan=r.step();
          if(distance<r.p.obstacle_pass_cm-1e-9)
              Check(plan.reason=="NE_QUA","Nhịp không đều/tốc độ thay đổi cộng sai quãng");
          else { Check(plan.reason=="VE_PHAI","Nhịp không đều không về đủ quãng"); break; }
          Check(n<99,"Nhịp không đều kẹt pha");
      }
      distance=0;
      for(int n=0;n<100;++n) {
          const int dt=n%2 ? 130 : 70;
          distance+=plan.cmd.cms*dt/1000.0;
          r.p.max_speed_cms=r.p.cruise_cms=r.p.min_cms=r.p.lost_cms=n>=10 && n<15 ? 0 : (n%2 ? 2 : 5);
          r.now+=dt-50; plan=r.step();
          if(distance<r.p.obstacle_pass_cm-1e-9)
              Check(plan.state==PlannerState::BAM_LAN && plan.reason=="VE_PHAI" &&
                    plan.mode==TrajMode::RETURN_RIGHT && plan.cmd.cms>=0 &&
                    plan.cmd.cms<=r.p.max_speed_cms,
                    "Pha về phải nhịp/tốc độ thay đổi phải tích phân lệnh đã gửi");
          else {
              Check(plan.state==PlannerState::BAM_LAN && plan.reason=="BAM_LAN" &&
                    plan.mode==TrajMode::LANE,"Nhịp không đều không kết thúc về phải đủ quãng");
              break;
          }
          Check(n<99,"Nhịp không đều kẹt pha về phải");
      }
    }
    { Rig r; r.p.obstacle_pass_cm=10; r.pass(); r.p.max_speed_cms=r.p.cruise_cms=r.p.min_cms=r.p.lost_cms=0;
      for(int n=0;n<250;++n) Check(r.step().reason=="NE_QUA","Tốc độ 0 vẫn cộng quãng"); }

}

void Unit() {
    for(int scenario=0;scenario<10;++scenario) {
        Rig r; r.auto_box=false; r.p.obstacle_action=2;
        r.in.mask=scenario==5 ? Mask() : Mask(0,0);
        Plan plan;
        for(int n=0;n<r.p.vote_window;++n) {
            const auto ms=r.now+50;
            const float w=scenario==0 ? 127.f : 128.f;
            // Ca 8: cx 0,35 (vật làn phải khi xe lệch hướng, log 10/11) phải né; ca 9: cx 0,245 (vật làn trái) không né.
            const float x=scenario==2 ? 100.f : (scenario==0 ? 600.f : (scenario==8 ? 384.f : (scenario==9 ? 250.f : 512.f)));
            r.in.detections={{},ms,++r.frame};
            if(scenario!=5) r.in.detections.items.push_back({"obstacle",cv::Rect2f(x,100,w,100),scenario==4 ? 0.4f : 0.5f,ms});
            if(scenario==3) { r.in.detections.captured_ms-=r.p.det_max_age_ms+1; r.in.detections.items.front().captured_ms=r.in.detections.captured_ms; }
            if(scenario==7 && n>0) --r.frame, r.in.detections.frame_id=r.frame;
            if(scenario==6) r.in.detections.items.front().captured_ms--;
            plan=r.step();
        }
        Check((plan.reason=="NE_TRAI")== (scenario==1 || scenario==8),"Ngưỡng hộp/tâm/tin cậy/lô cũ/lỗ không hộp");
    }
    { Rig r; r.auto_box=false; r.p.obstacle_action=2;
      auto box=[&](bool present) {
          const auto ms=r.now+50; r.in.detections={{},ms,++r.frame};
          if(present) r.in.detections.items.push_back({"obstacle",cv::Rect2f(600,100,128,100),0.9f,ms});
          return r.step();
      };
      for(int n=0;n<r.p.vote_min;++n) box(true);
      for(int n=0;n<r.p.vote_window-1;++n) Check(box(false).reason=="NE_TRAI","Chưa đủ lô vắng");
      box(true); // Ngắt chuỗi vắng.
      for(int n=0;n<r.p.vote_window+1;++n) {
          const auto ms=r.now+50;
          // Hộp còn >= nửa bề rộng đỉnh (128) của vật đang né, dù lệch trái, vẫn giữ pha né đang chạy.
          r.in.detections={{{"obstacle",cv::Rect2f(100,100,70,70),0.5f,ms}},ms,++r.frame};
          Check(r.step().reason=="NE_TRAI","Hộp hợp lệ lớn bằng nửa vật đang né vẫn giữ pha né");
      }
      for(int n=0;n<r.p.vote_window+2;++n) Check(r.step().reason=="NE_TRAI","Lô lặp không được đếm vắng");
      r.now+=r.p.det_max_age_ms;
      Check(r.step().reason=="NE_TRAI","Lô cũ giữ pha trái");
      for(int n=1;n<=r.p.vote_window;++n)
          Check(box(false).reason==(n==r.p.vote_window ? "NE_QUA" : "NE_TRAI"),"PASS theo lô vắng tươi");
    }
    { // Log 10/11 18:01: vật thứ hai ở xa (hộp nhỏ) còn trong ảnh không được giữ pha trái mãi; vật đang né đã qua thì NE_QUA.
      Rig r; r.auto_box=false; r.p.obstacle_action=2;
      for(int n=0;n<r.p.vote_min;++n) {
          const auto ms=r.now+50; r.in.detections={{{"obstacle",cv::Rect2f(600,100,128,100),0.9f,ms}},ms,++r.frame};
          r.step();
      }
      for(int n=1;n<=r.p.vote_window;++n) {
          const auto ms=r.now+50; r.in.detections={{{"obstacle",cv::Rect2f(600,100,40,40),0.9f,ms}},ms,++r.frame};
          Check(r.step().reason==(n==r.p.vote_window ? "NE_QUA" : "NE_TRAI"),"Vật xa nhỏ giữ pha trái mãi");
      }
    }
    { // Lỗi 1 (log lan3 10/11, b9bedce): trong NE_QUA vật thứ hai nằm ngay trước trên làn trái (tâm 0,46–0,48, đáy hộp
      // 0,57→0,63) và xe chỉ về phải sau đủ quãng pass -> va vật thứ hai. Vật trước mặt đủ gần thì kết thúc PASS sớm,
      // nhưng không sớm hơn 0,85·obstacle_pass_cm (giữ khoảng hở đuôi xe với vật đang né).
      auto steps_in_pass=[](int kind) {  // 0: không hộp; 1: hộp giữa gần; 2: hộp giữa xa; 3: hộp làn phải gần
          Rig r; r.p.obstacle_pass_cm=85; r.pass(); r.auto_box=false; int n=0;
          for(;n<2000;++n) {
              const auto ms=r.now+50; r.in.detections={{},ms,++r.frame};
              const float x=kind==3 ? 1000.f : 576.f, y2=kind==2 ? 360.f : 500.f;
              if(kind) r.in.detections.items.push_back({"obstacle",cv::Rect2f(x,y2-150,128,150),0.9f,ms});
              if(r.step().reason!="NE_QUA") break;
          }
          return n;
      };
      const int none=steps_in_pass(0), close=steps_in_pass(1), far=steps_in_pass(2), right=steps_in_pass(3);
      Check(close<none-3,"Vật trước mặt gần trong NE_QUA phải kết thúc PASS sớm");
      Check(close>=int(0.85*none)-2,"Kết thúc PASS sớm không được dưới 0,85·obstacle_pass_cm");
      Check(far==none && right==none,"Vật xa hoặc ở làn phải không đổi quãng PASS");
      // Vật trước mặt rất gần ngay đầu PASS (chưa đủ 0,85·pass): dừng KHONG_DUONG_NE, không lao vào vật.
      Rig r; r.p.obstacle_pass_cm=85; r.pass(); r.auto_box=false;
      const auto ms=r.now+50; r.in.detections={{{"obstacle",cv::Rect2f(576,450,128,200),0.9f,ms}},ms,++r.frame};
      const auto plan=r.step();
      Check(plan.state==PlannerState::DUNG_VAT_CAN && plan.reason=="KHONG_DUONG_NE" && plan.cmd.cms==0,
            "Vật rất gần trước mặt khi chưa đủ quãng PASS phải dừng");
      // Mặt nạ sạch không được tự nhả khi lô YOLO tươi còn thấy vật gần trước mặt (duyệt an toàn lượt 3, MEDIUM).
      for(int n=0;n<4*r.p.vote_window;++n) {
          const auto t=r.now+50; r.in.detections={{{"obstacle",cv::Rect2f(576,450,128,200),0.9f,t}},t,++r.frame};
          const auto held=r.step();
          Check(held.state==PlannerState::DUNG_VAT_CAN && held.cmd.cms==0,"Mặt nạ sạch nhả dừng khi YOLO còn vật gần");
      }
      // Lô YOLO thưa (không có lô mới) rồi YOLO cũ: mặt nạ sạch vẫn không được nhả (duyệt an toàn lượt 4).
      for(int n=0;n<3*r.p.vote_window;++n)
          Check(r.step().cmd.cms==0,"Mặt nạ sạch nhả dừng giữa hai lô YOLO");
      r.now+=r.p.det_max_age_ms;
      for(int n=0;n<3*r.p.vote_window;++n)
          Check(r.step().cmd.cms==0,"YOLO cũ: mặt nạ sạch nhả dừng vật trước mặt");
      // Lô tươi có một lô lỡ hộp xen giữa: chưa đủ vote_window lô vắng liên tiếp thì chưa nhả.
      for(int n=0;n<r.p.vote_window-1;++n) {
          const auto t=r.now+50; r.in.detections={{},t,++r.frame};
          Check(r.step().cmd.cms==0,"Nhả trước khi đủ lô YOLO vắng liên tiếp");
      }
      { const auto t=r.now+50; r.in.detections={{{"obstacle",cv::Rect2f(576,450,128,200),0.9f,t}},t,++r.frame};
        Check(r.step().cmd.cms==0,"Vật quay lại vẫn dừng"); }
      // Vật đã khuất khỏi YOLO và mặt nạ sạch: nhả như cũ (không kẹt dừng mãi).
      bool released=false;
      for(int n=0;n<4*r.p.vote_window && !released;++n) {
          const auto t=r.now+50; r.in.detections={{},t,++r.frame};
          released=r.step().state==PlannerState::BAM_LAN;
      }
      Check(released,"Hết vật trước mặt phải nhả KHONG_DUONG_NE");
    }
    { Rig r; r.p.obstacle_pass_cm=10; auto plan=r.pass(); r.auto_box=false;
      bool returned=false;
      for(int n=0;n<1000;++n) {
          const auto ms=r.now+50;
          r.in.detections={{{"obstacle",cv::Rect2f(600,100,128,100),0.9f,ms}},ms,++r.frame};
          plan=r.step();
          Check(plan.reason!="NE_TRAI","Hộp mới trong PASS/VE_PHAI không được bật lại né");
          if(plan.reason=="BAM_LAN") { returned=true; break; }
          Check(plan.reason=="NE_QUA" || plan.reason=="VE_PHAI","Mất pha né/về khi có hộp mới");
      }
      Check(returned,"Hộp mới làm kẹt về làn");
      for(int n=0;n<r.p.vote_window;++n) {
          const auto ms=r.now+50;
          r.in.detections={{{"obstacle",cv::Rect2f(100,100,128,100),0.9f,ms}},ms,++r.frame};
          Check(r.step().reason=="BAM_LAN","Vật làn trái sau khi về phải không được kích né");
      }
    }
    { Rig r; r.p.obstacle_action=0; r.in.mask=Mask(0,0); r.auto_box=false;
      for(int n=0;n<r.p.vote_window;++n) {
          const auto ms=r.now+50;
          r.in.detections={{{"obstacle",cv::Rect2f(600,100,128,100),0.9f,ms}},ms,++r.frame};
          Check(r.step().mode==TrajMode::LANE,"Action 0 phải bỏ hộp hợp lệ");
      }
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
    for(auto mask:{Mask(20,35),Mask(100,5),cv::Mat(100,200,CV_8UC1,cv::Scalar(0))}) {
        Rig r; r.auto_box=false; r.in.mask=mask;
        for(int n=0;n<12;++n) Check(r.step().state==PlannerState::BAM_LAN,"Lỗ ngoài hành lang/nhỏ/rỗng gây dừng");
    }
    { Rig r; r.in.yolo_enabled=true; r.latch(); r.auto_box=false;
      for(int n=1;n<=r.p.vote_min;++n) {
          const auto ms=r.now+50;
          r.in.detections={{{"red",cv::Rect2f(1050,100,200,100),1.0f,ms}},ms,++r.frame};
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
      r.in.mask=Mask(0,0); r.in.go=true; Check(r.step().state==PlannerState::BAM_LAN,"Reset giữ phiếu vật cản"); }
    { Rig r; r.p.vote_window=3; r.p.vote_min=2; r.p.green_frames=2;
      r.in.mask=Mask(); r.step(); r.in.mask=Mask(0,0); r.step(); r.step(); r.step();
      r.in.mask=Mask(); Check(r.step().state==PlannerState::BAM_LAN,"Phiếu ngoài cửa sổ còn hiệu lực");
      Check(r.step().state==PlannerState::DUNG_VAT_CAN,"Cửa sổ 2/3 không dừng"); }
    { Rig r; r.p.obstacle_action=0; r.in.yolo_enabled=false; r.planner.Reset(); LaneController lane;
      for(int n=0;n<20;++n) {
          r.in.mask=n%2 ? Mask() : Mask(0,0); r.in.go=n==0 || n==12; r.in.stop=n==10;
          r.now+=50; r.in.mask_ms=r.now; r.in.image_ms=r.now;
          const auto a=r.planner.Step(r.in,r.now,r.p);
          if(n>=10 && n<12) {
              Check(a.state==PlannerState::DUNG && a.cmd.cms==0 && a.cmd.deg==0,"Action 0 bỏ qua Space");
              lane.Reset();
          } else {
              const auto b=lane.Step(r.in.mask,r.now,r.now,r.p,TrajMode::LANE);
              Check(a.state==PlannerState::BAM_LAN && a.cmd.cms==b.cmd.cms && a.cmd.deg==b.cmd.deg &&
                    a.reason==b.debug.reason && a.mode==TrajMode::LANE,"Action 0 đổi bám làn gốc");
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
