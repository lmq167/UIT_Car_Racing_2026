#include "control/Planner.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using control::PlannerState;
void Check(bool ok, const std::string& message) {
    if (!ok) throw std::runtime_error(message);
}
void Expect(const control::Plan& a, PlannerState state, bool moving,
            const std::string& message) {
    Check(a.state==state && (moving ? a.cmd.cms>0 : a.cmd.cms==0),
          message+"; nhận "+control::StateName(a.state)+"/"+a.reason+
          " cms="+std::to_string(a.cmd.cms));
}
struct Rig {
    control::Planner planner;
    control::ControlParams p;
    control::PlannerInput in;
    int64_t now=1000;
    uint64_t frame=0;
    Rig() {
        p.start_wait_green=0; p.turn_enable=1; p.kd=0;
        p.lane1_width_frac={0.4,0.4};
        in.yolo_enabled=true; in.sensor_ok=in.encoder_ok=true;
        mask(); in.go=true; tick(); in.go=false;
    }
    void mask() {
        in.mask=cv::Mat(100,200,CV_8UC1,cv::Scalar(0));
        in.mask.colRange(60,141).setTo(1);
    }
    control::Plan tick(const std::vector<std::string>& labels={}) {
        now+=40; in.odo_cm+=1;
        in.image_ms=in.mask_ms=in.sensor_ms=now;
        in.detections={{},now,++frame};
        for (const auto& label:labels) {
            cv::Rect2f box(490,100,300,80);
            if (label=="obstacle") box=cv::Rect2f(576,100,128,100);
            if (label=="red" || label=="green")
                box=cv::Rect2f(static_cast<float>(1280*p.red_stop_x_frac-128),100,128,72);
            in.detections.items.push_back({label,box,0.9f,now});
        }
        return planner.Step(in,now,p);
    }
    control::Plan votes(const std::vector<std::string>& labels, int n=-1) {
        control::Plan a;
        for(int i=0;i<(n<0?p.vote_window+4:n);++i) a=tick(labels);
        return a;
    }
    void block() {
        p.obstacle_action=1;
        in.mask(cv::Rect(90,40,40,55)).setTo(0);
        Expect(votes({"obstacle"}),PlannerState::DUNG_VAT_CAN,false,"Thiết lập dừng vật");
    }
};
// P3 §1: đỏ không được ghi đè dừng vật; xanh không được làm xe chạy khi vật còn.
void RedGreenBlocked() {
    Rig r; r.block();
    for(int i=0;i<r.p.vote_window+4;++i)
        Expect(r.tick({"obstacle","red"}),PlannerState::DUNG_VAT_CAN,false,"Đỏ khi vật còn");
    for(int i=0;i<r.p.vote_window+r.p.green_frames+4;++i)
        Expect(r.tick({"obstacle","green"}),PlannerState::DUNG_VAT_CAN,false,"Xanh khi vật còn");
}
// P3 §1: chốt đỏ phải còn hiệu lực sau khi nhả vật, chỉ xanh mới cho chạy.
void RedAfterClear() {
    Rig r; r.block();
    Expect(r.votes({"obstacle","red"}),PlannerState::DUNG_VAT_CAN,false,
           "Trước khi nhả vật: đỏ chốt vẫn phải giữ dừng vật");
    r.mask();
    Expect(r.votes({"red"},2*r.p.vote_window+10),PlannerState::DUNG_DEN_DO,false,
           "Vật hết nhưng đỏ còn chốt");
    Expect(r.votes({"green"},r.p.vote_window+r.p.green_frames+4),PlannerState::BAM_LAN,true,
           "Vật hết và đủ xanh");
}
// P3 §2: G-sau-Space chỉ miễn giữ hướng cho lần mở đảo đầu tiên, dù chưa đồng bộ chặng.
void SecondIslandHold() {
    Rig r;
    r.votes({"round_barrier"});
    r.in.yaw=r.in.yaw_deg=-90;
    r.votes({},static_cast<int>(r.p.rb_lost_cm)+r.p.vote_window+50);
    Check(r.planner.RouteStep()==1,"Thiết lập neo rồi tới chặng 1");
    r.in.stop=true; r.tick(); r.in.stop=false; r.planner.Reset(true);
    r.in.yaw=r.in.yaw_deg=-45; r.in.go=true; r.tick(); r.in.go=false;
    r.votes({"round_barrier"});
    r.in.mask.setTo(0);
    for(int i=0;i<12;++i) {
        const auto a=r.tick({"round_barrier"});
        Check(a.state!=PlannerState::GIU_HUONG,"Đảo đầu sau G yaw lệch trục phải bỏ giữ hướng");
    }
    r.mask();
    r.votes({},static_cast<int>(r.p.rb_lost_cm)+r.p.vote_window+50);
    Check(r.planner.RouteStep()==1,"Yaw lệch trục chưa được đồng bộ sang chặng khác");
    r.votes({"round_barrier"});
    r.in.yaw=r.in.yaw_deg=-35; r.in.mask.setTo(0);
    const auto a=r.votes({"round_barrier"},12);
    Expect(a,PlannerState::GIU_HUONG,true,"Đảo thứ hai phải giữ hướng dù chưa đồng bộ chặng");
    Check(a.cmd.deg<0,"Yaw tăng so với lúc mở đảo thứ hai phải lái bù âm");
}
}
int main() {
    int failed=0;
    const std::pair<const char*,void(*)()> cases[]={
        {"red_green_blocked",RedGreenBlocked}, {"red_after_clear",RedAfterClear},
        {"second_island_hold",SecondIslandHold}};
    for(const auto& c:cases) {
        try { c.second(); std::cout<<c.first<<" PASS\n"; }
        catch(const std::exception& e) { ++failed; std::cout<<c.first<<" FAIL: "<<e.what()<<'\n'; }
    }
    return failed?1:0;
}
