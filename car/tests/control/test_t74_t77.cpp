#include "control/Planner.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace {
using control::Plan;
using control::PlannerState;
void Check(bool ok, const std::string& message) {
    if (!ok) throw std::runtime_error(message);
}
std::string Observed(const Plan& p) {
    std::ostringstream out;
    out << control::StateName(p.state) << '/' << p.reason << " cms=" << p.cmd.cms
        << " deg=" << p.cmd.deg << " lane=" << p.debug.state;
    return out.str();
}
void Expect(bool ok, const Plan& p, const std::string& message) {
    Check(ok, message + "; nhận " + Observed(p));
}
bool Near(double a, double b) { return std::abs(a-b)<1e-6; }
struct Rig {
    control::Planner planner;
    control::ControlParams p;
    control::PlannerInput in;
    int64_t now=1000;
    uint64_t frame=0;
    bool sensor_fresh=true;
    Rig() {
        p.turn_enable=1; p.start_wait_green=0; p.kd=0;
        p.lane_width_frac={0.4,0.4,0.4};
        p.steer_max_deg=30; p.steer_rate_dps=1000;
        p.cruise_cms=p.max_speed_cms=15; p.approach_cms=5; p.min_cms=4;
        p.rb_lost_cm=30; p.rb_turn1_cm=p.rb_turn2_cm=60;
        in.yolo_enabled=true; in.sensor_ok=true;
        mask(60,140); in.go=true;
        const auto plan=step(); in.go=false;
        Expect(plan.state==PlannerState::BAM_LAN && plan.cmd.cms>0,plan,"Thiết lập G/làn tốt");
    }
    void mask(int begin=60, int end=140) {
        in.mask=cv::Mat(60,200,CV_8UC1,cv::Scalar(0));
        for (int y=0;y<60;++y) {
            for (int x=begin;x<end;++x) in.mask.at<unsigned char>(y,x)=1;
            if (begin>0) in.mask.at<unsigned char>(y,begin)=2;
            if (end<200) in.mask.at<unsigned char>(y,end)=2;
        }
    }
    Plan step(bool fresh_mask=true, bool fresh_sensor=true, bool fresh_image=true) {
        if (fresh_image) in.image_ms=now;
        if (fresh_mask) in.mask_ms=now;
        if (fresh_sensor && sensor_fresh) in.sensor_ms=now;
        return planner.Step(in,now,p);
    }
    Plan send(const std::vector<std::string>& names={}, float confidence=0.9f,
              float cx=0.5f, float y2=0.8f, int dt=40) {
        now+=dt;
        std::vector<control::Detection> items;
        for (const auto& name:names)
            items.push_back({name,cv::Rect2f(cx*1280-40,y2*720-60,80,60),confidence,now});
        in.detections={items,now,++frame};
        return step();
    }
    Plan votes(const std::vector<std::string>& names, int n=-1, float confidence=0.9f,
               float cx=0.5f, float y2=0.8f) {
        Plan plan;
        for (int i=0;i<(n<0?p.vote_min:n);++i) plan=send(names,confidence,cx,y2);
        return plan;
    }
    Plan clear() { return votes({},p.vote_window); }
    Plan at(double odo, const std::vector<std::string>& names={}) {
        in.odo_cm=odo; return send(names);
    }
    Plan enter() { return votes({"round_barrier"}); }
    Plan leave() {
        clear(); return at(in.odo_cm+p.rb_lost_cm);
    }
    Plan rbturn(int pass) {
        enter(); leave();
        const auto plan=at(in.odo_cm+(pass==1?p.rb_turn1_cm:p.rb_turn2_cm));
        Expect(plan.state==(pass==1?PlannerState::RE_TRAI:PlannerState::RE_PHAI),plan,"Thiết lập rẽ sau lượt "+std::to_string(pass));
        return plan;
    }
    Plan finish(bool left=true) {
        in.yaw_deg+=(left?1:-1)*p.turn_yaw_deg;
        const auto plan=send();
        Expect(plan.state==PlannerState::BAM_LAN,plan,"Thiết lập hoàn tất rẽ theo yaw");
        return plan;
    }
    Plan hold(double yaw0=0, double yaw=10) {
        in.yaw_deg=yaw0; enter(); in.yaw_deg=yaw;
        in.mask.setTo(0);
        const auto plan=votes({"round_barrier"},12);
        Expect(plan.state==PlannerState::GIU_HUONG && plan.cmd.cms>0,plan,"Thiết lập GIU_HUONG");
        return plan;
    }
    Plan turn() {
        p.turn_start_cm=0;
        const auto plan=votes({"left"});
        Expect(plan.state==PlannerState::RE_TRAI,plan,"Thiết lập rẽ trái từ biển");
        return plan;
    }
};
void Lane(const Plan& p, const std::string& why) {
    Expect(p.state==PlannerState::BAM_LAN && p.cmd.cms>0,p,why);
}
void Same(const Plan& a, const Plan& b, const std::string& why) {
    bool same=a.state==b.state && a.reason==b.reason && a.reference==b.reference &&
        a.cmd.cms==b.cmd.cms && a.cmd.deg==b.cmd.deg && a.debug.e==b.debug.e &&
        a.debug.p==b.debug.p && a.debug.d==b.debug.d && a.debug.angle_deg==b.debug.angle_deg &&
        a.debug.speed_cms==b.debug.speed_cms && a.debug.valid_rows==b.debug.valid_rows &&
        a.debug.state==b.debug.state && a.debug.reason==b.debug.reason &&
        a.debug.rows.size()==b.debug.rows.size();
    for (size_t i=0;same && i<a.debug.rows.size();++i) {
        const auto& x=a.debug.rows[i]; const auto& y=b.debug.rows[i];
        same=x.y==y.y && x.lines==y.lines && x.left==y.left && x.right==y.right &&
             x.center==y.center && x.valid==y.valid;
    }
    Expect(same,a,why+"; đối chứng "+Observed(b));
}
void LostStop(Rig& r, const std::string& why) {
    Plan plan;
    for (int i=0;i<(r.p.lost_stop_ms+160)/40;++i) {
        plan=r.send({"round_barrier"});
        Expect(plan.state!=PlannerState::GIU_HUONG,plan,why+": không tái giữ hướng trong lượt dở");
    }
    Expect(plan.cmd.cms==0,plan,why+": dừng theo mất làn cũ");
    r.in.sensor_ok=true; r.sensor_fresh=true; r.mask();
    plan=r.votes({"round_barrier"},r.p.regain_frames+2);
    Lane(plan,why+": cảm biến/làn tốt phải tự chạy lại, không chốt dừng mới");
}
void NoVotes(const std::string& label, const std::string& mode) {
    Rig r;
    if (mode=="few") r.votes({label},r.p.vote_min-1);
    else if (mode=="repeat") {
        r.send({label});
        for (int i=0;i<r.p.vote_window+2;++i) { r.now+=40; Lane(r.step(),"Lô lặp không thêm phiếu"); }
    } else {
        for (int i=0;i<r.p.vote_min;++i) {
            r.now+=40;
            const auto old=r.now-r.p.det_max_age_ms-1;
            r.in.detections={{{label,cv::Rect2f(600,500,80,60),0.9f,mode=="stale_batch"?r.now:old}},
                             mode=="stale_batch"?old:r.now,++r.frame};
            Lane(r.step(),"Phiếu cũ phải bị bỏ");
        }
    }
    Lane(r.step(),"Chưa đủ phiếu "+label+"/"+mode);
    if (label=="round_barrier") {
        r.clear();
        for (int i=1;i<=6;++i) Lane(r.at(50*i),"Thiếu phiếu không được tính một lượt");
    }
}
// T74_notes §1–6: rào gần dùng máy rẽ IMU, khóa tái kích tới khi phiếu về 0.
void T74(const std::string& name) {
    if (name=="thresholds") {
        struct Sample { float conf,cx,y2_px; bool turn; };
        const Sample samples[]={{.499f,.5f,576,false},{.5f,.5f,576,true},
            {.9f,.5f,467.9f,false},{.9f,.5f,468,true},
            {.9f,.25f,576,true},{.9f,.75f,576,true},
            {.9f,.2499f,576,false},{.9f,.7501f,576,false},{.9f,.86f,576,false}};
        for (const auto& s:samples) {
            Rig r; Plan plan;
            for (int i=0;i<r.p.vote_min;++i) {
                r.now+=40;
                r.in.detections={{{"work_barrier",cv::Rect2f(s.cx*1280-40,s.y2_px-60,80,60),s.conf,r.now}},r.now,++r.frame};
                plan=r.step();
            }
            Expect((plan.state==PlannerState::RE_TRAI)==s.turn,plan,
                   "Biên rào conf="+std::to_string(s.conf)+" cx="+std::to_string(s.cx)+" y2_px="+std::to_string(s.y2_px));
            if (!s.turn) Lane(plan,"Hộp ngoài miền không rẽ và không dừng");
            if (s.conf<.5f) Expect(plan.cmd.cms>r.p.approach_cms,plan,"Hộp rào tin cậy thấp không kẹp tiếp cận");
        }
    } else if (name=="votes") {
        for (const auto& mode:{"few","repeat","stale_batch","stale_item"}) NoVotes("work_barrier",mode);
    } else if (name=="approach") {
        for (bool near:{false,true}) {
            Rig r;
            const float y2=static_cast<float>(r.p.work_barrier_y2_frac-.10+(near?.0001:-.0001));
            const auto plan=r.votes({"work_barrier"},r.p.vote_window,.9f,.5f,y2);
            Lane(plan,"Rào tiếp cận chưa rẽ");
            Expect(near?plan.cmd.cms<=r.p.approach_cms:plan.cmd.cms>r.p.approach_cms,plan,"Biên giảm tốc rào");
        }
    } else if (name=="turn_yaw") {
        for (double sign:{-1.,1.}) {
            Rig r; r.in.yaw_deg=5;
            auto plan=r.votes({"work_barrier"});
            Expect(plan.state==PlannerState::RE_TRAI && Near(plan.cmd.cms,r.p.min_cms) &&
                   Near(plan.cmd.deg,r.p.turn_steer_deg),plan,"Rào gần vào lệnh rẽ IMU min_cms/+turn_steer_deg");
            r.in.yaw_deg=5+sign*(r.p.turn_yaw_deg-.01); plan=r.send({"work_barrier"});
            Expect(plan.state==PlannerState::RE_TRAI,plan,"Chưa đủ yaw còn rẽ");
            r.in.yaw_deg=5+sign*r.p.turn_yaw_deg; plan=r.send({"work_barrier"});
            Expect(plan.state==PlannerState::BAM_LAN && plan.reference==control::LaneRef::RIGHT,plan,"Đủ |yaw-yaw0| về BAM_LAN RIGHT");
        }
    } else if (name=="box_moves") {
        Rig r; r.votes({"work_barrier"});
        for (int i=0;i<6;++i) {
            const auto plan=r.send({"work_barrier"},.9f,.86f);
            Expect(plan.state==PlannerState::RE_TRAI && plan.cmd.deg>0,plan,"Hộp đổi bên không đổi quyết định rẽ");
        }
    } else if (name=="no_repeat" || name=="space_go") {
        Rig r; r.votes({"work_barrier"}); r.in.yaw_deg=r.p.turn_yaw_deg;
        Lane(r.send({"work_barrier"}),"Rào xong rẽ");
        if (name=="space_go") {
            r.in.stop=true; ++r.now; Check(r.step().cmd.cms==0,"Space phải dừng"); r.in.stop=false;
            r.in.go=true; ++r.now; r.step(); r.in.go=false;
        }
        for (int i=0;i<r.p.vote_window+3;++i) Lane(r.send({"work_barrier"}),"Cùng rào không rẽ lại, kể cả Space/G");
        if (name=="no_repeat") {
            r.votes({},r.p.vote_window-r.p.vote_min+1);
            Lane(r.votes({"work_barrier"}),"Phiếu rào chưa về 0 thì không được tái kích, dù từng dưới vote_min");
            r.clear();
            const auto plan=r.votes({"work_barrier"});
            Expect(plan.state==PlannerState::RE_TRAI,plan,"Hết phiếu rồi rào mới phải được rẽ");
        }
    } else if (name=="sign_priority") {
        Rig r; r.p.turn_start_cm=30; r.votes({"right"});
        Lane(r.votes({"work_barrier"}),"Biển armed thắng rào");
        const auto plan=r.at(30,{"work_barrier"});
        Expect(plan.state==PlannerState::RE_PHAI,plan,"Đủ quãng vẫn theo biển phải");
        const auto turning=r.votes({"work_barrier"});
        Expect(turning.state==PlannerState::RE_PHAI,turning,"Đang rẽ T57 không kích hoạt rào");
    } else if (name=="sensor_fault") {
        for (bool stale:{false,true}) {
            Rig r;
            if (!stale) r.in.sensor_ok=false;
            else { r.sensor_fresh=false; r.now+=201; r.step(true,false); }
            for (int i=0;i<r.p.vote_window+1;++i) Lane(r.send({"work_barrier"}),"IMU lỗi không rẽ, không dừng");
        }
    } else if (name=="disabled") {
        Rig a,b; a.p.work_barrier_action=b.p.work_barrier_action=0;
        for (int i=0;i<12;++i) Same(a.send({"work_barrier"}),b.send(),"Tắt rào giống không hộp");
    } else throw std::runtime_error("Ca T74 không tồn tại: "+name);
}
// T68b Chỉ làm §1–4: đếm lần bằng phiếu/encoder, không dùng yaw để đếm.
void T68b(const std::string& name) {
    if (name=="pass1_distance" || name=="pass2_distance") {
        Rig r;
        if (name=="pass2_distance") { r.rbturn(1); r.finish(); }
        r.enter(); r.clear(); const double lost0=r.in.odo_cm;
        Lane(r.at(lost0+29.99),"Mất phiếu chưa 30cm");
        Lane(r.at(lost0+30),"Đúng 30cm hết lượt, chưa tới chỗ rẽ");
        Lane(r.at(lost0+89.99),"Sau lượt chưa đủ 60cm");
        const auto plan=r.at(lost0+90);
        Expect(plan.state==(name=="pass1_distance"?PlannerState::RE_TRAI:PlannerState::RE_PHAI),plan,"Đúng 60cm sau lượt phải rẽ đúng hướng");
    } else if (name=="flicker") {
        Rig r; r.enter(); r.clear(); Lane(r.at(29),"Nhấp nháy chưa hết lượt");
        r.enter(); r.clear(); Lane(r.at(59),"Bắt đầu tính lại mất phiếu từ 29cm");
        Lane(r.at(118.99),"Chưa đủ quãng sau một lượt");
        const auto plan=r.at(119);
        Expect(plan.state==PlannerState::RE_TRAI,plan,"Nhấp nháy vẫn chỉ lượt thứ nhất");
        r.finish(); r.rbturn(2);
    } else if (name=="after_two") {
        for (int reset=-1;reset<2;++reset) {
            Rig r; r.rbturn(1); r.finish();
            if (reset>=0) {
                r.in.stop=true; ++r.now; r.step(); r.in.stop=false;
                r.planner.Reset(reset!=0); r.in.go=true; ++r.now; r.step(); r.in.go=false;
            }
            r.rbturn(2); r.finish(false);
            r.enter(); r.leave(); const double start=r.in.odo_cm;
            for (int i=1;i<=10;++i) Lane(r.at(start+40*i),"Sau hai lượt không rẽ nữa, kể cả Reset/G");
        }
    } else if (name=="visible") {
        for (const std::string other:{"","left","work_barrier"}) {
            Rig r; r.enter();
            for (int i=1;i<=10;++i) Lane(r.at(30*i,other.empty()?std::vector<std::string>{"round_barrier"}:std::vector<std::string>{"round_barrier",other}),"Còn phiếu vòng xuyến không rẽ, lớp kèm="+other);
        }
    } else if (name=="sign_priority") {
        Rig r; r.enter(); r.leave(); r.p.turn_start_cm=60;
        r.votes({"right"});
        const auto plan=r.at(r.in.odo_cm+60);
        Expect(plan.state==PlannerState::RE_PHAI,plan,"Biển phải thắng luật rẽ trái sau lượt 1");
    } else if (name=="votes") {
        for (const auto& mode:{"few","repeat","stale_batch","stale_item"}) NoVotes("round_barrier",mode);
    } else if (name=="sensor_fault") {
        for (bool stale:{false,true}) {
            Rig r; r.enter(); r.leave();
            if (!stale) r.in.sensor_ok=false;
            else { r.sensor_fresh=false; r.now+=201; r.step(true,false); }
            Lane(r.at(r.in.odo_cm+60),"IMU lỗi: luật sau vòng xuyến không rẽ, không dừng");
        }
    } else if (name=="zero_distance") {
        for (int pass:{1,2}) {
            Rig r; if (pass==2) { r.rbturn(1); r.finish(); }
            if (pass==1) r.p.rb_turn1_cm=0; else r.p.rb_turn2_cm=0;
            r.enter(); const auto plan=r.leave();
            Expect(plan.state==(pass==1?PlannerState::RE_TRAI:PlannerState::RE_PHAI),plan,"Quãng sau lượt 0cm phải rẽ ngay khi hết lượt");
        }
    } else throw std::runtime_error("Ca T68b không tồn tại: "+name);
}
// T77 phương án B: làn tốt theo làn, mất làn trong lượt mới giữ yaw lúc vào.
void T77(const std::string& name) {
    if (name=="only_on_loss") {
        Rig r; r.in.yaw_deg=3; r.enter(); r.in.yaw_deg=13;
        const auto lane=r.votes({"round_barrier"},6);
        Expect(lane.state==PlannerState::BAM_LAN && Near(lane.cmd.deg,0),lane,"B chưa mất làn vẫn bám làn");
        r.in.mask.setTo(0); const auto plan=r.votes({"round_barrier"},12);
        Expect(plan.state==PlannerState::GIU_HUONG && Near(plan.cmd.deg,-10*r.p.rb_yaw_kp),plan,"Mất làn dùng yaw0 đầu lượt");
    } else if (name=="yaw_sign" || name=="wrap") {
        for (double sign:{-1.,1.}) {
            Rig r; const bool wrap=name=="wrap";
            const auto plan=r.hold(wrap?sign*179:0,wrap?-sign*179:sign*10);
            const double expected=(wrap?-sign*2:-sign*10)*r.p.rb_yaw_kp;
            Expect(Near(plan.cmd.deg,expected),plan,"Dấu lái/yaw và gói ±180; kỳ vọng "+std::to_string(expected));
        }
    } else if (name=="steer_max") {
        for (double sign:{-1.,1.}) {
            Rig r; r.p.rb_yaw_kp=5;
            const auto plan=r.hold(0,sign*80);
            Expect(Near(plan.cmd.deg,-sign*r.p.steer_max_deg),plan,"Giữ hướng phải kẹp trần góc");
        }
    } else if (name=="steer_rate") {
        Rig r; r.p.steer_rate_dps=10;
        r.enter(); r.in.mask.setTo(0); r.in.yaw_deg=10;
        double previous=0; Plan plan;
        for (int i=0;i<35;++i) {
            plan=r.send({"round_barrier"});
            Expect(std::abs(plan.cmd.deg-previous)<=.4+1e-6 && std::abs(plan.cmd.deg)<=r.p.steer_max_deg,plan,"Giữ hướng đổi góc không quá rate×dt");
            previous=plan.cmd.deg;
        }
        Expect(plan.state==PlannerState::GIU_HUONG && Near(plan.cmd.deg,-10*r.p.rb_yaw_kp),plan,"Rate hội tụ tới góc giữ hướng");
    } else if (name=="speed") {
        for (int mode=0;mode<3;++mode) {
            Rig r; r.p.rb_yaw_kp=1; r.p.slow_per_deg=1;
            if (mode==0) r.p.cruise_cms=r.p.max_speed_cms;
            const auto plan=r.hold(0,mode==0?0:(mode==1?5:80));
            const double speed=std::max(r.p.min_cms,std::min(r.p.max_speed_cms,r.p.cruise_cms-r.p.slow_per_deg*std::abs(plan.cmd.deg)));
            Expect(Near(plan.cmd.cms,speed),plan,"Tốc độ giữ hướng theo góc cuối và kẹp min/max, kỳ vọng "+std::to_string(speed));
        }
    } else if (name=="regain") {
        Rig r; r.hold(); r.mask();
        for (int i=1;i<r.p.regain_frames;++i) {
            const auto plan=r.send({"round_barrier"});
            Expect(plan.state==PlannerState::GIU_HUONG,plan,"Chưa đủ regain_frames vẫn giữ hướng");
        }
        Lane(r.send({"round_barrier"}),"Đủ regain_frames phải về bám làn");
    } else if (name=="pass_end") {
        Rig r; r.hold(); r.clear();
        const auto plan=r.at(r.p.rb_lost_cm);
        Expect(plan.state!=PlannerState::GIU_HUONG,plan,"Hết lượt phải thoát giữ hướng");
        r.in.detections={};
        for (int i=0;i<(r.p.lost_stop_ms+160)/40;++i) r.send();
        const auto stopped=r.send(); Expect(stopped.cmd.cms==0,stopped,"Hết lượt nhưng vẫn mất làn phải dừng");
        r.mask(); Lane(r.votes({},r.p.regain_frames+2),"Hết lượt rồi bắt lại làn phải chạy lại không G");
    } else if (name=="hold_distance") {
        Rig r; r.enter(); r.in.odo_cm=r.p.rb_hold_max_cm-1; r.in.mask.setTo(0); r.in.yaw_deg=10;
        auto plan=r.votes({"round_barrier"},12);
        Expect(plan.state==PlannerState::GIU_HUONG,plan,"Mất làn cuối quãng vẫn được giữ hướng");
        plan=r.at(r.p.rb_hold_max_cm,{"round_barrier"});
        Expect(plan.state==PlannerState::GIU_HUONG,plan,"Đúng hold_max_cm chưa quá hạn");
        plan=r.at(r.p.rb_hold_max_cm+.01,{"round_barrier"});
        Expect(plan.state!=PlannerState::GIU_HUONG,plan,"Quá hold_max tính từ đầu lượt phải thoát");
        LostStop(r,"Quá quãng giữ hướng");
    } else if (name=="sensor_entry" || name=="sensor_mid") {
        for (bool stale:{false,true}) {
            Rig r; if (name=="sensor_mid") r.hold();
            if (!stale) r.in.sensor_ok=false;
            else { r.sensor_fresh=false; r.now+=201; r.step(true,false); }
            if (name=="sensor_entry") { r.enter(); r.in.mask.setTo(0); }
            LostStop(r,"Cảm biến "+std::string(stale?"cũ":"lỗi")+" "+name);
        }
    } else if (name=="stale_mask" || name=="stale_image") {
        Rig r; r.hold(); const bool mask=name=="stale_mask";
        r.now+=mask?r.p.mask_max_age_ms+1:control::kStaleImageMs+1;
        const auto plan=r.step(!mask,true,mask);
        Expect(plan.cmd.cms==0 && plan.cmd.deg==0,plan,"Giữ hướng không vượt ưu tiên ảnh/mask cũ");
    } else if (name=="space_go" || name=="reset") {
        for (bool preserve:{false,true}) {
            Rig r; r.hold();
            if (name=="space_go") {
                r.in.stop=true; ++r.now; const auto plan=r.step();
                Expect(plan.cmd.cms==0 && plan.cmd.deg==0,plan,"Space khi giữ hướng phải dừng"); r.in.stop=false;
            } else r.planner.Reset(preserve);
            r.mask(); r.in.go=true; ++r.now; r.step(); r.in.go=false;
            r.votes({"round_barrier"},r.p.regain_frames+1); r.in.mask.setTo(0);
            LostStop(r,"Space/G/R kết thúc giữ hướng lượt dở");
        }
    } else if (name=="block_turn" || name=="block_barrier" || name=="block_obstacle") {
        for (const std::string& sign:(name=="block_turn"?std::vector<std::string>{"left","right"}:std::vector<std::string>{"work_barrier"})) {
            Rig r; r.hold(); r.p.turn_start_cm=0;
            if (name=="block_obstacle") {
                Rig baseline; baseline.p.obstacle_action=2;
                r.p.obstacle_action=2; r.p.regain_frames=20;
                r.mask(); baseline.mask();
                for (int y=24;y<57;++y) for (int x=90;x<120;++x) {
                    r.in.mask.at<unsigned char>(y,x)=0;
                    baseline.in.mask.at<unsigned char>(y,x)=0;
                }
                const auto obstacle=baseline.votes({},baseline.p.vote_window+2);
                Expect(obstacle.reason=="NE_TRAI" || obstacle.reference==control::LaneRef::LEFT,obstacle,"Đối chứng cùng mask ngoài vòng xuyến phải kích né");
            }
            for (int i=0;i<r.p.vote_window+2;++i) {
                r.in.odo_cm+=1;
                const auto plan=r.send(name=="block_obstacle"?std::vector<std::string>{"round_barrier"}:std::vector<std::string>{"round_barrier",sign});
                Expect(plan.state==PlannerState::GIU_HUONG && plan.reason!="NE_TRAI" && plan.reason!="VE_PHAI",plan,"Đang giữ hướng không bắt đầu rẽ/rào/né");
            }
        }
    } else if (name=="red" || name=="person") {
        Rig r; r.hold(); const auto plan=r.votes({"round_barrier",name=="red"?"red":"person"},-1,.9f,.95f);
        Expect(plan.cmd.cms==0,plan,"Đỏ/người phải kẹp tốc độ giữ hướng");
        if (name=="person") Expect(plan.state==PlannerState::GIU_HUONG && plan.reason=="NGUOI_DI_BO",plan,"Người không đổi state giữ hướng");
    } else throw std::runtime_error("Ca T77 không tồn tại: "+name);
}
// T75 Chỉ làm §1–5: người chỉ kẹp tốc độ, giữ state và tiếp tục không cần G.
void T75(const std::string& name) {
    if (name=="people" || name=="person") {
        Rig r,baseline; r.mask(80,160); baseline.mask(80,160);
        Plan plan;
        for (int i=0;i<r.p.vote_min;++i) {
            plan=r.send({name},.5f); const auto lane=baseline.send();
            Expect(Near(plan.cmd.deg,lane.cmd.deg),plan,"Người dừng tốc độ, servo vẫn theo làn");
            if (i<r.p.vote_min-1) Lane(plan,"Chưa đủ phiếu người vẫn chạy");
        }
        Expect(plan.state==PlannerState::BAM_LAN && plan.cmd.cms==0 && plan.reason=="NGUOI_DI_BO",plan,"Alias người/conf=0,5 đủ phiếu phải dừng");
    } else if (name=="votes") {
        for (const auto& mode:{"few","repeat","stale_batch","stale_item"}) NoVotes("person",mode);
        Rig r;
        Lane(r.send({"people","person","person"}),"Nhiều hộp cùng frame chỉ một phiếu");
        for (int i=1;i<r.p.vote_min;++i) {
            const auto plan=r.send({i%2?"person":"people"});
            if (i==r.p.vote_min-1) Expect(plan.cmd.cms==0 && plan.reason=="NGUOI_DI_BO",plan,"Hai alias cộng phiếu chung");
        }
    } else if (name=="low_conf") {
        for (const std::string label:{"people","person"}) {
            Rig r;
            for (int i=0;i<r.p.vote_window+2;++i) Lane(r.send({label},.499f),"Tin cậy dưới 0,5 phải bỏ");
        }
    } else if (name=="release") {
        for (bool age:{false,true}) {
            Rig r; auto plan=r.votes({"person"});
            Expect(plan.cmd.cms==0,plan,"Thiết lập dừng người"); const double odo=r.in.odo_cm;
            if (age) { r.now+=r.p.det_max_age_ms+1; plan=r.step(); }
            else {
                for (int i=1;i<=r.p.vote_window-r.p.vote_min+1;++i) {
                    plan=r.send();
                    if (i<r.p.vote_window-r.p.vote_min+1) Expect(plan.cmd.cms==0,plan,"Còn đủ phiếu vẫn dừng");
                }
            }
            Lane(plan,"Hết phiếu tươi phải chạy tiếp không G");
            Check(r.in.odo_cm==odo,"Đầu vào encoder giữ nguyên khi dừng");
        }
    } else if (name=="absent") {
        Rig a,b;
        for (int i=0;i<30;++i) {
            a.mask(60+i%4*4,140+i%4*4); b.mask(60+i%4*4,140+i%4*4);
            if (i>=12 && i<16) { a.in.mask.setTo(0); b.in.mask.setTo(0); }
            a.in.odo_cm=b.in.odo_cm=i; a.in.yaw_deg=b.in.yaw_deg=i*.1;
            Same(a.send({"car"}),b.send(),"Lớp người vắng, các trường đầu ra bằng đối chứng tổng hợp");
        }
    } else if (name=="turn_pause_stuck" || name=="turn_pause_timeout") {
        Rig r; r.p.vote_window=r.p.vote_min;
        r.p.turn_max_ms=name=="turn_pause_timeout"?1000:20000;
        r.turn(); auto plan=r.votes({"person"});
        const int pause=name=="turn_pause_timeout"?1200:2000;
        for (int elapsed=0;elapsed<pause;elapsed+=100) {
            plan=r.send({"person"},.9f,.5f,.8f,100);
            Expect(plan.state==PlannerState::RE_TRAI && plan.reason=="NGUOI_DI_BO" && plan.cmd.cms==0,plan,"Người dừng không làm hủy rẽ do thời gian/kẹt");
        }
        plan=r.send();
        Expect(plan.state==PlannerState::RE_TRAI && plan.cmd.cms>0,plan,"Nhả người phải tiếp tục RE_TRAI");
        for (int i=0;i<3;++i) {
            r.in.odo_cm+=.5; plan=r.send();
            Expect(plan.state==PlannerState::RE_TRAI && plan.cmd.cms>0,plan,"Sau thả không dùng lại hạn thời gian/kẹt đã tạm dừng");
        }
    } else if (name=="space_priority") {
        Rig r; r.votes({"person"}); r.in.stop=true; ++r.now;
        const auto plan=r.step(); Expect(plan.cmd.cms==0 && plan.cmd.deg==0,plan,"Space vẫn ưu tiên người");
    } else throw std::runtime_error("Ca T75 không tồn tại: "+name);
}
// Miền khóa theo T74_notes §5, T68b §3 và T77 §5; kiểm cả Validate lẫn Load YAML.
void Params(const std::string& group, const std::string& path) {
    struct Cleanup { std::string path; ~Cleanup() { std::remove(path.c_str()); } } cleanup{path};
    control::ControlParams defaults;
    struct Field { const char* name; double control::ControlParams::*member; double low,high,value; };
    std::vector<Field> fields;
    if (group=="t74") {
        Check(defaults.work_barrier_action==1 && Near(defaults.work_barrier_y2_frac,.65),"Sai mặc định rào 1/0,65");
        fields={{"work_barrier_y2_frac",&control::ControlParams::work_barrier_y2_frac,.30,.95,.65}};
        for (int value:{-1,0,1,2}) {
            auto p=defaults; p.work_barrier_action=value;
            bool rejected=false; try { p.Validate(); } catch (const std::exception&) { rejected=true; }
            Check(rejected==(value<0 || value>1),"Sai miền work_barrier_action");
            { std::ofstream out(path); Check(bool(out),"Không mở YAML tạm"); out<<"work_barrier_action: "<<value<<'\n'; }
            rejected=false; int loaded_value=-99;
            try { std::ostringstream report; auto loaded=control::ControlParams::Load(path,report); loaded.Validate(); loaded_value=loaded.work_barrier_action; }
            catch (const std::exception&) { rejected=true; }
            Check(rejected==(value<0 || value>1),"Sai miền Load work_barrier_action");
            if (!rejected) Check(loaded_value==value,"Load action sai");
        }
    } else if (group=="t68b") {
        fields={{"rb_lost_cm",&control::ControlParams::rb_lost_cm,10,100,30},
                {"rb_turn1_cm",&control::ControlParams::rb_turn1_cm,0,400,60},
                {"rb_turn2_cm",&control::ControlParams::rb_turn2_cm,0,400,60}};
    } else fields={{"rb_yaw_kp",&control::ControlParams::rb_yaw_kp,.1,5,1},
                   {"rb_hold_max_cm",&control::ControlParams::rb_hold_max_cm,20,400,150}};
    for (const auto& f:fields) {
        Check(Near(defaults.*(f.member),f.value),std::string("Sai mặc định ")+f.name);
        for (double value:{f.low,f.high,f.low-.001,f.high+.001}) {
            const bool invalid=value<f.low || value>f.high;
            auto p=defaults; p.*(f.member)=value;
            bool rejected=false; try { p.Validate(); } catch (const std::exception&) { rejected=true; }
            Check(rejected==invalid,std::string("Sai Validate ")+f.name+"="+std::to_string(value));
            { std::ofstream out(path); Check(bool(out),"Không mở YAML tạm"); out<<f.name<<": "<<value<<'\n'; }
            rejected=false; double loaded_value=std::numeric_limits<double>::quiet_NaN();
            try { std::ostringstream report; auto loaded=control::ControlParams::Load(path,report); loaded.Validate(); loaded_value=loaded.*(f.member); }
            catch (const std::exception&) { rejected=true; }
            Check(rejected==invalid,std::string("Sai miền Load YAML ")+f.name+"="+std::to_string(value));
            if (!rejected) Check(Near(loaded_value,value),std::string("Load không giữ giá trị ")+f.name);
        }
        for (double value:{std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}) {
            auto p=defaults; p.*(f.member)=value;
            bool rejected=false; try { p.Validate(); } catch (const std::exception&) { rejected=true; }
            Check(rejected,std::string("Validate phải từ chối NaN/Inf ")+f.name);
        }
    }
}
}
int main(int argc, char** argv) {
    try {
        Check(argc>=2,"Thiếu tên scenario");
        const std::string name=argv[1]; const auto split=name.find('_');
        Check(split!=std::string::npos,"Tên phải có tiền tố phiếu");
        const auto group=name.substr(0,split), scenario=name.substr(split+1);
        if (scenario=="params") { Check(argc==3,"Thiếu đường YAML tạm"); Params(group,argv[2]); }
        else if (group=="t74") T74(scenario);
        else if (group=="t68b") T68b(scenario);
        else if (group=="t77") T77(scenario);
        else if (group=="t75") T75(scenario);
        else throw std::runtime_error("Tiền tố không hợp lệ: "+group);
        std::cout<<"ĐẠT: acc_"<<name<<'\n'; return 0;
    } catch (const std::exception& e) {
        std::cerr<<"KHÔNG ĐẠT: "<<(argc>1?argv[1]:"?")<<": "<<e.what()<<'\n'; return 1;
    }
}
