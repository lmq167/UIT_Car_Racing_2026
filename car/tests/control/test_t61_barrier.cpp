#include "control/Planner.h"
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>

using namespace control;
namespace {
void Check(bool ok, const char *message) {
    if(!ok) throw std::runtime_error(message);
}
cv::Mat Mask(bool hole=false) {
    cv::Mat mask(100,200,CV_8UC1,cv::Scalar(0));
    mask.colRange(20,181).setTo(1);
    mask.col(20).setTo(2); mask.col(180).setTo(2);
    if(hole) mask(cv::Rect(90,40,40,55)).setTo(0);
    return mask;
}
struct Rig {
    Planner planner;
    ControlParams p;
    PlannerInput in;
    int64_t now=1000;
    uint64_t frame=0;
    Rig() {
        p.work_barrier_action=1; p.start_wait_green=0; p.right_lost_ms=0;
        p.obstacle_pass_cm=10;
        in.mask=Mask(); in.yolo_enabled=true; in.go=true;
        send(false); in.go=false;
    }
    Plan step(bool new_mask=true) {
        now+=50; in.image_ms=now;
        if(new_mask) in.mask_ms=now;
        return planner.Step(in,now,p);
    }
    Plan send(bool barrier=true, float bottom=504, float confidence=0.5f) {
        in.detections={}; in.detections.frame_id=++frame;
        in.detections.captured_ms=now+50;
        if(barrier) in.detections.items.push_back(
            {"work_barrier",cv::Rect2f(100,bottom-100,100,100),confidence,now+50});
        return step();
    }
    Plan left() {
        Plan plan;
        for(int n=1;n<=p.vote_min;++n) {
            plan=send();
            Check((plan.reason=="NE_TRAI")== (n==p.vote_min),"Rào kích hoạt sai số phiếu");
            if(n<p.vote_min) Check(step(false).reference==LaneRef::RIGHT,"Lô trùng tạo phiếu rào");
        }
        Check(plan.state==PlannerState::BAM_LAN && plan.reference==LaneRef::LEFT &&
              plan.cmd.cms>0 && plan.cmd.cms<=p.approach_cms,"Rào không né trái/tốc độ tiếp cận");
        LaneController expected;
        const auto lane=expected.Step(in.mask,in.mask_ms,now,p,LaneRef::LEFT);
        Check(plan.debug.e==lane.debug.e,"Không truyền LEFT vào bộ bám làn");
        return plan;
    }
    Plan pass() {
        left(); Plan plan;
        for(int n=1;n<=p.vote_window-p.vote_min+1;++n) {
            plan=send(false);
            Check(plan.reason==(n==p.vote_window-p.vote_min+1 ? "NE_QUA" : "NE_TRAI"),
                  "Rào phải hết đủ phiếu mới sang QUA");
        }
        return plan;
    }
};
void Params(const char *yaml, const char *temporary) {
    struct Cleanup { const char *path; ~Cleanup() { std::remove(path); } } cleanup{temporary};
    std::ostringstream report;
    Check(ControlParams().work_barrier_action==0,"Mặc định struct sai");
    Check(ControlParams::Load(yaml,report).work_barrier_action==0,"Mặc định YAML sai");
    Check(report.str().find("work_barrier_action =")==std::string::npos,"YAML thiếu khóa rào");
    auto load=[&](const std::string &body) {
        { std::ofstream out(temporary); out << "%YAML:1.0\n---\n" << body << '\n'; }
        return ControlParams::Load(temporary,report);
    };
    Check(load("{}").work_barrier_action==0,"Thiếu khóa không mặc định 0");
    for(int value:{0,1}) Check(load("work_barrier_action: "+std::to_string(value)).work_barrier_action==value,"Miền hợp lệ");
    for(const auto *value:{"-1","2","0.5","abc","[1]",".Nan"}) {
        bool rejected=false;
        try { load(std::string("work_barrier_action: ")+value); }
        catch(const std::exception &) { rejected=true; }
        Check(rejected,"Chấp nhận khóa sai kiểu/miền");
    }
    for(int value:{-1,2}) {
        auto p=ControlParams(); p.work_barrier_action=value; bool rejected=false;
        try { p.Validate(); } catch(const std::exception &) { rejected=true; }
        Check(rejected,"Validate bỏ qua miền khóa rào");
    }
}
void Votes() {
    for(int invalid=0;invalid<11;++invalid) {
        Rig r;
        for(int n=0;n<12;++n) {
            r.in.detections={}; r.in.detections.frame_id=++r.frame;
            r.in.detections.captured_ms=r.now+50;
            Detection d{"work_barrier",cv::Rect2f(100,404,100,100),0.5f,r.now+50};
            if(invalid==0) d.box.y=403.99f;
            if(invalid==1) d.confidence=0.499f;
            if(invalid==2) d.confidence=1.01f;
            if(invalid==3) d.confidence=std::numeric_limits<float>::quiet_NaN();
            if(invalid==4) d.box.x=-1;
            if(invalid==5) d.box.width=0;
            if(invalid==6) d.box.y=621;
            if(invalid==7) d.box.x=1200;
            if(invalid==8) d.box.height=std::numeric_limits<float>::infinity();
            if(invalid==9) --d.captured_ms;
            if(invalid==10) d.class_name="barrier";
            r.in.detections.items.push_back(d);
            Check(r.step().reference==LaneRef::RIGHT,"Hộp không hợp lệ vẫn kích hoạt");
        }
    }
    { Rig r; r.left();
      for(int n=0;n<12;++n) Check(r.send().reason=="NE_TRAI","Phiếu rào gần không giữ LEFT"); }
    { Rig r; r.send(); r.send(); r.now+=r.p.det_max_age_ms+1;
      Check(r.step().reference==LaneRef::RIGHT,"YOLO cũ gây kích hoạt");
      Check(r.send().reference==LaneRef::RIGHT,"Phiếu cũ chưa bị loại");
      Check(r.send().reference==LaneRef::RIGHT,"Phiếu cũ còn ẩn");
      Check(r.send().reason=="NE_TRAI","Không nhận phiếu mới sau YOLO cũ"); }
    { Rig r; r.left(); r.now+=r.p.det_max_age_ms+1;
      Check(r.step().reason=="NE_QUA","YOLO cũ vẫn giữ phiếu rào"); }
    { Rig r; r.p.vote_window=3; r.p.vote_min=2; r.p.green_frames=2;
      r.send(); r.send(false); r.send(false); r.send(false);
      Check(r.send().reference==LaneRef::RIGHT,"Phiếu ngoài cửa sổ vẫn còn");
      Check(r.send().reason=="NE_TRAI","Không dùng cửa sổ tùy chỉnh"); }
    { Rig r; r.in.yolo_enabled=false;
      for(int n=0;n<8;++n) Check(r.send().reference==LaneRef::RIGHT,"Tắt YOLO vẫn tạo phiếu"); }
}
void Phases() {
    { Rig r; auto plan=r.pass(); double distance=0;
      for(int n=0;n<100;++n) {
          distance+=plan.cmd.cms*0.05; plan=r.send(false);
          if(distance<r.p.obstacle_pass_cm-1e-9)
              Check(plan.reason=="NE_QUA" && plan.reference==LaneRef::LEFT,"QUA chưa đủ quãng");
          else {
              Check(plan.reason=="VE_PHAI" && plan.reference==LaneRef::RIGHT,"Không về phải đủ quãng");
              Check(r.send(false).reason=="BAM_LAN","VE_PHAI kéo dài"); break;
          }
          Check(n<99,"Pha QUA không kết thúc");
      }
    }
    for(int action:{1,2}) {
        Rig r; r.p.obstacle_action=action; r.in.mask=Mask(true); r.left();
        for(int n=0;n<r.p.vote_window+2;++n)
            Check(r.send(false).reason=="NE_TRAI","Còn lỗ đã hết né/ưu tiên rào sai");
        r.in.mask=Mask();
        for(int n=1;n<=r.p.vote_window;++n)
            Check(r.send(false).reason==(n==r.p.vote_window ? "NE_QUA" : "NE_TRAI"),"Hết lỗ không theo chuỗi sạch cũ");
    }
    for(int action:{1,2}) {
        Rig r; r.p.obstacle_action=action; r.in.mask=Mask(true); Plan plan;
        for(int n=0;n<r.p.vote_min;++n) plan=r.send(false);
        Check(action==1 ? plan.state==PlannerState::DUNG_VAT_CAN : plan.reason=="NE_TRAI",
              "Lỗ riêng không theo obstacle_action");
        r.in.mask=Mask();
        for(int n=0;n<r.p.vote_window;++n) plan=r.send(false);
        Check(action==1 ? plan.state==PlannerState::BAM_LAN : plan.reason=="NE_QUA",
              "Lỗ riêng không thả sau chuỗi sạch");
    }
    for(bool passing:{false,true}) for(int cancel=0;cancel<4;++cancel) {
        Rig r; if(passing) r.pass(); else r.left();
        Plan stopped;
        if(cancel==0) { r.in.stop=true; stopped=r.step(); r.in.stop=false; }
        if(cancel==1) { r.in.mask=cv::Mat(100,200,CV_8UC1,cv::Scalar(0)); stopped=r.step(); }
        if(cancel==2) { r.now+=r.p.mask_max_age_ms+1; r.in.image_ms=r.now;
            stopped=r.planner.Step(r.in,r.now,r.p); }
        if(cancel==3) { r.now+=kStaleImageMs+1; stopped=r.planner.Step(r.in,r.now,r.p); }
        Check(stopped.state==PlannerState::DUNG && stopped.cmd.cms==0 && stopped.cmd.deg==0,
              "Hủy pha phải dừng hẳn");
        Check(stopped.reason==(cancel==0 ? "SPACE" : cancel==3 ? "MAT_ANH" : "MAT_LAN_NE"),"Sai lý do hủy pha");
        r.in.mask=Mask();
        for(int n=0;n<r.p.vote_window+2;++n) {
            const auto plan=r.send(false);
            Check(plan.state==PlannerState::DUNG && plan.cmd.cms==0 && plan.cmd.deg==0,"Tự chạy lại trước G");
        }
        r.in.go=true; const auto plan=r.send(false); r.in.go=false;
        Check(plan.state==PlannerState::BAM_LAN && plan.reference==LaneRef::RIGHT && plan.cmd.cms>0,"G mới không bỏ pha cũ");
    }
    { Rig r; r.left(); r.planner.Reset(true);
      r.in.go=true; Check(r.send(false).reference==LaneRef::RIGHT,"Reset giữ pha rào"); }
    { Rig r; r.left();
      for(int n=1;n<=r.p.vote_min;++n) {
          r.in.detections={{{"red",cv::Rect2f(1050,100,200,100),1.0f,r.now+50}},r.now+50,++r.frame};
          const auto plan=r.step();
          if(n==r.p.vote_min) Check(plan.state==PlannerState::DUNG_DEN_DO && plan.cmd.cms==0,"Rào che ưu tiên đèn đỏ");
      }
    }
}
void Disabled() {
    for(int action:{0,1,2}) {
        Rig with,without; with.p.work_barrier_action=without.p.work_barrier_action=0;
        with.p.obstacle_action=without.p.obstacle_action=action;
        for(int n=0;n<120;++n) {
            with.in.mask=without.in.mask=Mask(n>=5 && n<20);
            with.in.stop=without.in.stop=n==80;
            with.in.go=without.in.go=n==90;
            const auto a=with.send(), b=without.send(false);
            Check(a.state==b.state && a.reason==b.reason && a.reference==b.reference &&
                  a.cmd.cms==b.cmd.cms && a.cmd.deg==b.cmd.deg && a.debug.e==b.debug.e &&
                  a.debug.state==b.debug.state,"Action rào 0 đổi hành vi cũ");
        }
    }
}
}
int main(int argc, char **argv) {
    try {
        Check(argc==3,"Cần control.yaml và YAML tạm");
        Params(argv[1],argv[2]); Votes(); Phases(); Disabled();
        std::cout << "PASS T61\n"; return 0;
    } catch(const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
