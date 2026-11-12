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
// Tốc độ rẽ IMU có chặn bởi trần cấu hình.
double TurnCms(const control::ControlParams &p) {
    return std::min(p.max_speed_cms,std::max(p.min_cms,8.0));
}
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
    Rig(bool sensor_ok=true) {
        p.turn_enable=1; p.start_wait_green=0; p.kd=0;
        p.lane1_width_frac={0.4,0.4};
        p.steer_max_deg=30; p.turn_steer_deg=25; p.steer_rate_dps=1000;
        p.cruise_cms=p.max_speed_cms=15; p.approach_cms=5; p.min_cms=4;
        p.rb_lost_cm=30; p.rb_turn1_cm=p.rb_turn2_cm=60;
        in.yolo_enabled=true; in.sensor_ok=sensor_ok;
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
    // Ngã T giả, chờ đủ phiếu mặt nạ mới; không thay đổi quãng encoder.
    void junction_mask() {
        mask();
        const int y=static_cast<int>(0.72*in.mask.rows);
        in.mask.rowRange(0,y).setTo(0);
        in.mask.rowRange(y,static_cast<int>(0.79*in.mask.rows)).setTo(1);
    }
    Plan junction() {
        junction_mask();
        Plan plan;
        for(int i=0;i<p.vote_min;++i) { now+=40; plan=step(); }
        return plan;
    }
    // Kéo dài lỗi cảm biến đủ 250 ms để kiểm việc hủy rẽ.
    bool was_bad=false;
    void PersistBad(bool fresh_mask, Plan &plan) {
        const bool bad=!in.sensor_ok || !std::isfinite(in.yaw_deg) || !std::isfinite(in.odo_cm) ||
                       in.sensor_ms<0 || in.sensor_ms>now || now-in.sensor_ms>200;
        const bool first=!was_bad; was_bad=bad;
        if(!bad || !first) return;  // chỉ kéo dài ở mẫu hỏng đầu tiên của đợt
        bool cancelled=false; auto cancel_plan=plan;
        for(int k=0;k<5;++k) {
            now+=50; in.image_ms=now; if(fresh_mask) in.mask_ms=now;
            // Camera vẫn gửi lô YOLO mới (cùng nội dung) trong lúc cảm biến hỏng; không có thì phiếu hết hạn, sai thực tế.
            if(!in.detections.items.empty()) { in.detections.captured_ms=now; in.detections.frame_id=++frame; for(auto &d:in.detections.items) d.captured_ms=now; }
            plan=planner.Step(in,now,p);
            if(!cancelled && plan.reason=="HUY_RE_CAM_BIEN") { cancelled=true; cancel_plan=plan; }
        }
        if(cancelled) plan=cancel_plan;  // ca kiểm lượt hủy cần thấy đúng lệnh của khung hủy
    }
    Plan step(bool fresh_mask=true, bool fresh_sensor=true, bool fresh_image=true) {
        if (fresh_image) in.image_ms=now;
        if (fresh_mask) in.mask_ms=now;
        if (fresh_sensor && sensor_fresh) in.sensor_ms=now;
        auto plan=planner.Step(in,now,p);
        PersistBad(fresh_mask,plan);
        return plan;
    }
    Plan send(const std::vector<std::string>& names={}, float confidence=0.9f,
              float cx=0.5f, float y2=0.8f, int dt=40) {
        now+=dt;
        std::vector<control::Detection> items;
        for (const auto& name:names) {
            const float width=name=="round_barrier" ? 300.f : (name=="work_barrier" ? 130.f : (name=="obstacle" ? 128.f : 80.f));
            items.push_back({name,cv::Rect2f(cx*1280-width/2,((name=="people" || name=="person") && y2==0.8f ? 0.98f : y2)*720-60,width,60),confidence,now});
        }
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
    Plan enter() { mask(); return votes({"round_barrier"}); }
    Plan leave() {
        clear(); return at(in.odo_cm+p.rb_lost_cm);
    }
    Plan rbturn(int pass) {
        enter(); leave();
        at(in.odo_cm+(pass==1?p.rb_turn1_cm:p.rb_turn2_cm));
        const auto plan=junction();
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
        votes({"left"});
        const auto plan=junction();
        Expect(plan.state==PlannerState::RE_TRAI,plan,"Thiết lập rẽ trái từ biển");
        return plan;
    }
};
void Lane(const Plan& p, const std::string& why) {
    Expect(p.state==PlannerState::BAM_LAN && p.cmd.cms>0,p,why);
}
void Same(const Plan& a, const Plan& b, const std::string& why) {
    bool same=a.state==b.state && a.reason==b.reason && a.mode==b.mode &&
        a.cmd.cms==b.cmd.cms && a.cmd.deg==b.cmd.deg && a.debug.e==b.debug.e &&
        a.debug.p==b.debug.p && a.debug.d==b.debug.d && a.debug.angle_deg==b.debug.angle_deg &&
        a.debug.speed_cms==b.debug.speed_cms && a.debug.valid_rows==b.debug.valid_rows &&
        a.debug.state==b.debug.state && a.debug.reason==b.debug.reason &&
        a.debug.rows.size()==b.debug.rows.size();
    for (size_t i=0;same && i<a.debug.rows.size();++i) {
        const auto& x=a.debug.rows[i]; const auto& y=b.debug.rows[i];
        same=x.y==y.y && x.left==y.left && x.right==y.right && x.conf==y.conf &&
             x.target==y.target && x.valid==y.valid;
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
            const float width=label=="round_barrier" ? 300.f : (label=="work_barrier" ? 130.f : 80.f);
            r.in.detections={{{label,cv::Rect2f(640-width/2,label=="person"?646:500,width,60),0.9f,mode=="stale_batch"?r.now:old}},
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
            // 624c8bd: y2_frac mặc định 0,55; 9c71115: ranh tâm rộng 0,55 (state.md:11). Rào hẹp sớm 0,05: 360 px; rộng: 396 px.
            {.9f,.5f,359.9f,false},{.9f,.5f,360,true},
            {.9f,.75f,395.9f,false},{.9f,.75f,396,true},
            {.9f,.25f,576,true},{.9f,.81f,576,true},
            {.9f,.2499f,576,false},{.9f,.8201f,576,false},{.9f,.86f,576,false}};
        for (const auto& s:samples) {
            Rig r; Plan plan;
            for (int i=0;i<r.p.vote_min;++i) {
                r.now+=40;
                r.in.detections={{{"work_barrier",cv::Rect2f(s.cx*1280-65,s.y2_px-60,130,60),s.conf,r.now}},r.now,++r.frame};
                plan=r.step();
            }
            Lane(plan,"Rào chỉ chốt ý định khi chưa giao lộ");
            plan=r.junction();
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
            r.votes({"work_barrier"}); auto plan=r.junction();
            Expect(plan.state==PlannerState::RE_TRAI && Near(plan.cmd.cms,TurnCms(r.p)) &&
                   Near(plan.cmd.deg,1.0*r.p.turn_steer_deg),plan,"Rào hẹp (cx 0,5) vào lệnh rẽ IMU min_cms/+1,0*turn_steer_deg (rào 1: rẽ trái vào đường 1 làn, bán kính nhỏ)");
            r.in.yaw_deg=5+sign*(r.p.turn_yaw_deg-.01); plan=r.send({"work_barrier"});
            Expect(plan.state==PlannerState::RE_TRAI,plan,"Chưa đủ yaw còn rẽ");
            r.in.yaw_deg=5+sign*r.p.turn_yaw_deg; plan=r.send({"work_barrier"});
            Expect(plan.state==PlannerState::BAM_LAN && plan.mode==control::TrajMode::LANE,plan,"Đủ |yaw-yaw0| về BAM_LAN RIGHT");
        }
    } else if (name=="barrier_types") {
        // Work 1 (hộp giữa, vào đường 1 làn): rẽ gắt 1,12*turn_steer_deg, kích hoạt sớm 0,05; work 2 (hộp lệch phải): 0,8*turn_steer_deg.
        const double frac=Rig().p.work_barrier_y2_frac;
        { Rig r; r.votes({"work_barrier"},-1,.9f,.5f,static_cast<float>(frac-.04)); auto plan=r.junction();
          Expect(plan.state==PlannerState::RE_TRAI && Near(plan.cmd.deg,1.0*r.p.turn_steer_deg),plan,"Work 1: sớm 0,04 và lái 1,0*turn_steer_deg"); }
        { Rig r; auto plan=r.votes({"work_barrier"},-1,.9f,.5f,static_cast<float>(frac-.06));
          Lane(plan,"Work 1: chưa tới đáy hộp thì chưa rẽ"); }
        { Rig r; auto plan=r.votes({"work_barrier"},-1,.9f,.75f,static_cast<float>(frac-.04));
          Lane(plan,"Work 2: theo y2_frac đủ, không rẽ sớm hơn"); }
        { Rig r; r.votes({"work_barrier"},-1,.9f,.78f,static_cast<float>(frac+.001)); auto plan=r.junction();
          Expect(plan.state==PlannerState::RE_TRAI && Near(plan.cmd.deg,0.8*r.p.turn_steer_deg),plan,"Work 2 (hộp cx 0,78): vào đường 2 làn, lái 0,8*turn_steer_deg"); }
    } else if (name=="box_moves") {
        Rig r; r.votes({"work_barrier"}); r.junction();
        for (int i=0;i<6;++i) {
            const auto plan=r.send({"work_barrier"},.9f,.86f);
            Expect(plan.state==PlannerState::RE_TRAI && plan.cmd.deg>0,plan,"Hộp đổi bên không đổi quyết định rẽ");
        }
    } else if (name=="no_repeat" || name=="space_go") {
        Rig r; r.votes({"work_barrier"}); r.junction(); r.in.yaw_deg=r.p.turn_yaw_deg;
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
            r.votes({"work_barrier"}); auto plan=r.junction();
            Expect(plan.state==PlannerState::RE_TRAI,plan,"Hết phiếu rồi rào mới phải được rẽ");
        }
    } else if (name=="sign_priority") {
        Rig r; r.p.turn_start_cm=30; r.votes({"right"});
        Lane(r.votes({"work_barrier"}),"Biển armed thắng rào");
        r.at(30,{"work_barrier"}); const auto plan=r.junction();
        Expect(plan.state==PlannerState::RE_PHAI,plan,"Đủ quãng vẫn theo biển phải");
        const auto turning=r.votes({"work_barrier"});
        Expect(turning.state==PlannerState::RE_PHAI,turning,"Đang rẽ T57 không kích hoạt rào");
    } else if (name=="sensor_fault") {
        std::string context;
        auto stopped=[&](const Plan& plan) {
            Expect(plan.state==PlannerState::BAM_LAN && plan.cmd.cms==0 &&
                   plan.reason=="DUNG_RAO_LOI_CAM_BIEN",plan,"Q7: rào gần khi không dùng được rẽ phải dừng "+context);
        };
        for (int fault=0;fault<5;++fault) {
            const bool startup=fault==0;
            context="lỗi="+std::to_string(fault);
            Rig r(!startup);
            if (fault==1) r.in.sensor_ok=false;
            if (fault==2) { r.sensor_fresh=false; r.now+=201; r.step(true,false); }
            if (fault==3) r.in.yaw_deg=std::numeric_limits<double>::quiet_NaN();
            if (fault==4) r.in.odo_cm=std::numeric_limits<double>::quiet_NaN();
            const auto far=r.votes({"work_barrier"},r.p.vote_window,.9f,.5f,
                                   static_cast<float>(r.p.work_barrier_y2_frac-.1));
            Lane(far,"Q7: rào xa trong vùng tiếp cận chưa dừng khi cảm biến lỗi");
            Expect(far.cmd.cms<=r.p.approach_cms,far,"Rào xa vẫn kẹp tốc độ tiếp cận");
            for (int i=1;i<=r.p.vote_min;++i) {
                const auto plan=r.send({"work_barrier"});
                if (i<r.p.vote_min) Lane(plan,"Q7: rào gần chưa đủ phiếu không dừng");
                else stopped(plan);
            }
            for (int i=0;i<r.p.vote_window+1;++i) stopped(r.send({"work_barrier"}));

            Rig bypass=r;
            bypass.in.stop=true; ++bypass.now; const auto space=bypass.step();
            Expect(space.state==PlannerState::DUNG && space.reason=="SPACE" &&
                   space.cmd.cms==0 && space.cmd.deg==0,space,"Q7: Space vẫn dừng motor/servo");
            bypass.in.stop=false;
            for (int i=0;i<3;++i) {
                const auto held=bypass.send({"work_barrier"});
                Expect(held.state==PlannerState::DUNG && held.cmd.cms==0 && held.cmd.deg==0,
                       held,"Q7: chưa G mới thì vẫn dừng");
            }
            bypass.in.go=true; Lane(bypass.send({"work_barrier"}),"Q7: G từ dừng cho qua rào đang thấy");
            bypass.in.go=false;
            for (int i=0;i<bypass.p.vote_window+3;++i)
                Lane(bypass.send({"work_barrier"}),"Q7: cùng rào sau Space/G không dừng/rẽ lại");
            context="lỗi="+std::to_string(fault)+" rào mới sau Space/G";
            bypass.clear(); stopped(bypass.votes({"work_barrier"}));

            if (startup) {
                // Lỗi khởi tạo giữ cảm biến không khả dụng suốt phiên, khác lỗi đọc tạm.
                for (int i=0;i<30;++i) stopped(r.send({"work_barrier"}));
                continue;
            }
            r.in.sensor_ok=true; r.sensor_fresh=true; r.in.yaw_deg=r.in.odo_cm=0;
            r.junction_mask();
            const auto fresh_ms=r.now+40;
            context="lỗi="+std::to_string(fault)+" trong 999 ms tươi";
            stopped(r.send({"work_barrier"}));
            while (r.now+40<fresh_ms+999) stopped(r.send({"work_barrier"}));
            stopped(r.send({"work_barrier"},.9f,.5f,.8f,static_cast<int>(fresh_ms+999-r.now)));
            Lane(r.send({"work_barrier"},.9f,.5f,.8f,1),"Q7: đủ 1000 ms hồi cảm biến, chờ phiếu giao lộ sau khi chốt lại ý định");
            const auto recovered=r.junction();
            Expect(recovered.state==PlannerState::RE_TRAI && Near(recovered.cmd.cms,TurnCms(r.p)),
                   recovered,"Q7: hồi đủ 1000 ms và đủ phiếu giao lộ mới rẽ trái qua rào gần");
        }
        struct Sample { float conf,cx,y2_px; bool stop; };
        Rig defaults;
        const float near=static_cast<float>(defaults.p.work_barrier_y2_frac*720);
        const Sample samples[]={{.5f,.25f,near,true},{.5f,.75f,near,true},
            {.499f,.5f,576,false},{.9f,.2499f,576,false},{.9f,.8201f,576,false},
            {.9f,.75f,near-.1f,false},{.9f,.5f,near-36.1f,false},{.5f,.5f,near-36.f,true}};
        for (const auto& s:samples) {
            context="biên conf="+std::to_string(s.conf)+" tâm="+std::to_string(s.cx)+
                    " đáy="+std::to_string(s.y2_px);
            Rig r(false);
            Plan plan;
            for (int i=0;i<r.p.vote_window;++i) {
                r.now+=40;
                r.in.detections={{{"work_barrier",cv::Rect2f(s.cx*1280-65,s.y2_px-60,130,60),s.conf,r.now}},r.now,++r.frame};
                plan=r.step();
            }
            if (s.stop) stopped(plan);
            else Lane(plan,"Q7: rào ngoài miền gần/tâm/conf không gây dừng");
        }
    } else if (name=="disabled") {
        Rig a,b; a.p.work_barrier_action=b.p.work_barrier_action=0;
        for (int i=0;i<12;++i) Same(a.send({"work_barrier"}),b.send(),"Tắt rào giống không hộp");
    } else throw std::runtime_error("Ca T74 không tồn tại: "+name);
}
// Q7/DUNG: chỉ rào gần đủ phiếu mới dừng khi máy rẽ không khả dụng.
void Q7(const std::string& name) {
    auto stopped=[](const Plan& plan) {
        Expect(plan.state==PlannerState::BAM_LAN && plan.cmd.cms==0 &&
               plan.reason=="DUNG_RAO_LOI_CAM_BIEN",plan,"Q7: hủy rẽ và dừng trước rào");
    };
    if (name=="action0") {
        for (bool startup:{false,true}) {
            Rig r(!startup), baseline(!startup);
            r.p.work_barrier_action=baseline.p.work_barrier_action=0;
            r.in.sensor_ok=baseline.in.sensor_ok=false;
            for (int i=0;i<40;++i) {
                const auto plan=r.send({"work_barrier"});
                Same(plan,baseline.send(),"Q7: action 0 bỏ qua rào cả khi cảm biến lỗi");
                Lane(plan,"Q7: action 0 vẫn bám làn chạy");
                Expect(plan.reason!="DUNG_RAO_LOI_CAM_BIEN",plan,"Q7 không áp dụng khi action 0");
            }
        }
    } else if (name=="recover_barrier_moved") {
        for (int moved=0;moved<4;++moved) {
            Rig r; r.in.sensor_ok=false;
            stopped(r.votes({"work_barrier"}));
            const std::vector<std::string> labels=moved==0?std::vector<std::string>{}:
                std::vector<std::string>{"work_barrier"};
            const float cx=moved==2?.2499f:(moved==3?.8201f:.5f);
            const float y2=moved==1?static_cast<float>(r.p.work_barrier_y2_frac-.1):.8f;
            Lane(r.votes(labels,r.p.vote_window,.9f,cx,y2),"Q7: rào hết phiếu/xa/ngoài tâm thì nhả dừng");
            r.in.sensor_ok=true;
            Lane(r.send(labels,.9f,cx,y2,1),"Q7: bắt đầu hồi khi rào đã chuyển");
            for (int elapsed=40;elapsed<=1000;elapsed+=40)
                Lane(r.send(labels,.9f,cx,y2),"Q7: hồi đủ 1 s không rẽ theo vị trí rào cũ");
        }
    } else if (name=="fault_while_barrier_turn") {
        for (int fault=0;fault<5;++fault) {
            Rig r;
            r.votes({"work_barrier"}); auto plan=r.junction();
            Expect(plan.state==PlannerState::RE_TRAI && plan.cmd.cms>0,plan,"Q7: đang rẽ do rào");
            if (fault==0) r.in.sensor_ok=false;
            if (fault==1) { r.sensor_fresh=false; r.now+=201; }
            if (fault==2) r.in.odo_cm+=r.p.turn_max_cm+.01;
            if (fault>=3) {
                const int duration=fault==3?1000:1500;
                if (fault==3) r.p.turn_max_ms=duration;
                const auto start=r.now;
                while (r.now+100<start+duration) {
                    if (fault==3) r.in.odo_cm+=.2;
                    plan=r.send({"work_barrier"},.9f,.5f,.8f,100);
                    Expect(plan.state==PlannerState::RE_TRAI,plan,"Q7: chưa quá hạn/chưa kẹt vẫn rẽ");
                }
                plan=r.send({"work_barrier"},.9f,.5f,.8f,
                            static_cast<int>(start+duration-(fault==3?0:1)-r.now));
                Expect(plan.state==PlannerState::RE_TRAI,plan,"Q7: đúng hạn ms hoặc 1499 ms chưa kẹt");
                plan=r.send({"work_barrier"},.9f,.5f,.8f,1);
            } else plan=r.send({"work_barrier"});
            stopped(plan);
            r.in.sensor_ok=true; r.sensor_fresh=true;
            stopped(r.send({"work_barrier"},.9f,.5f,.8f,1));
            for (int elapsed=100;elapsed<=900;elapsed+=100)
                stopped(r.send({"work_barrier"},.9f,.5f,.8f,100));
            stopped(r.send({"work_barrier"},.9f,.5f,.8f,99));
            plan=r.send({"work_barrier"},.9f,.5f,.8f,1);
            Lane(plan,"Q7: hồi 1000 ms, chờ đủ phiếu giao lộ sau khi chốt lại ý định");
            for(int i=1;i<r.p.vote_min;++i) plan=r.send({"work_barrier"},.9f,.5f,.8f,1);
            Expect(plan.state==PlannerState::RE_TRAI && Near(plan.cmd.cms,TurnCms(r.p)) &&
                   plan.cmd.deg>0 && plan.cmd.deg<=r.p.turn_steer_deg,plan,
                   "Q7: hồi 1000 ms và đủ phiếu giao lộ rẽ lại rào còn gần; lỗi="+std::to_string(fault));
        }
    } else if (name=="vs_red") {
        for (bool turning:{false,true}) {
            Rig r;
            if (turning) r.votes({"work_barrier"});
            r.in.sensor_ok=false;
            for (int i=0;i<r.p.vote_window+5;++i) {
                r.now+=40;
                r.in.detections={{{"work_barrier",cv::Rect2f(575,516,130,60),.9f,r.now},
                                   {"red",cv::Rect2f(1160,500,80,80),.9f,r.now}},r.now,++r.frame};
                const auto plan=r.step();
                if (i>=r.p.vote_min-1)
                    Expect(plan.state==PlannerState::DUNG_DEN_DO && plan.cmd.cms==0 && plan.cmd.deg==0 &&
                           plan.reason=="DUNG_DEN_DO",plan,"Q7: đỏ thắng dừng lỗi rào, cả khi đang rẽ");
            }
        }
    } else if (name=="startup_fault_barrier") {
        Rig r(false);
        for (int i=0;i<80;++i) {
            r.in.odo_cm+=.5;
            const auto plan=r.send({"work_barrier"});
            Check(!r.in.sensor_ok && r.in.sensor_ms==r.now,"Q7: IMU khởi động lỗi, encoder vẫn tươi");
            if (i>=r.p.vote_min-1) stopped(plan);
        }
        Check(r.now-1000>1000,"Q7: đã quan sát quá 1 s, không hồi lỗi khởi động");
    } else throw std::runtime_error("Ca Q7 không tồn tại: "+name);
}
// Q9/TAM: đồng hồ hồi lỗi chỉ nhận khoảng tươi liên tục; bỏ lượt rẽ cũ.
void Q9(const std::string& name) {
    if (name=="flicker_resets_clock") {
        Rig r; r.p.turn_start_cm=0; r.in.sensor_ok=false;
        Lane(r.send({"left"}),"Q9: lỗi tạm khi đang bám làn");
        for (int period=0;period<2;++period) {
            r.in.sensor_ok=true;
            Lane(r.send({"left"},.9f,.5f,.8f,1),"Q9: mẫu tươi đầu chưa cho rẽ");
            for (int elapsed=40;elapsed<=600;elapsed+=40) {
                r.in.odo_cm+=1;
                Lane(r.send({"left"}),"Q9: 600 ms tươi chưa hồi, kể cả sau lần chớp lỗi");
            }
            if (period==0) {
                r.in.sensor_ok=false;
                Lane(r.send({"left"},.9f,.5f,.8f,1),"Q9: một mẫu lỗi phải đặt lại đồng hồ");
            }
        }
        for (int elapsed=640;elapsed<=960;elapsed+=40)
            Lane(r.send({"left"}),"Q9: chưa đủ 1000 ms liên tục vẫn khóa rẽ");
        Lane(r.send({"left"},.9f,.5f,.8f,39),"Q9: 999 ms vẫn khóa rẽ");
        Lane(r.send({},.9f,.5f,.8f,1),"Q9: đủ 1000 ms không hồi sinh biển trái cũ");
        r.p.turn_start_cm=10;
        Lane(r.votes({"right"}),"Q9: biển mới đủ phiếu nhưng chưa đủ quãng");
        const double armed=r.in.odo_cm;
        Lane(r.at(armed+9.99),"Q9: biển mới chưa đủ 10 cm");
        r.at(armed+10); const auto plan=r.junction();
        Expect(plan.state==PlannerState::RE_PHAI && plan.cmd.cms>0,plan,"Q9: biển phải mới đủ phiếu/quãng rẽ được sau hồi");
    } else if (name=="one_source_stale") {
        for (bool stale:{false,true}) {
            Rig r; r.p.turn_start_cm=0;
            r.in.sensor_ok=false;
            Lane(r.send(),"Q9: thiết lập lỗi tạm");
            r.in.sensor_ok=stale; r.sensor_fresh=!stale;
            if (stale) r.in.sensor_ms=r.now-201;
            for (int i=0;i<35;++i) {
                const auto plan=r.send({"left"});
                Check(stale?(r.in.sensor_ok && r.now-r.in.sensor_ms>200):
                            (!r.in.sensor_ok && r.in.sensor_ms==r.now),"Q9: đúng một điều kiện cảm biến còn lỗi");
                Lane(plan,"Q9: chỉ timestamp cũ hoặc sensor_ok sai cũng không được hồi sau >1 s");
            }
            r.in.sensor_ok=true; r.sensor_fresh=true;
            Lane(r.send({},.9f,.5f,.8f,1),"Q9: sửa nguồn cuối mới bắt đầu đồng hồ");
            for (int elapsed=40;elapsed<=960;elapsed+=40)
                Lane(r.send({"left"}),"Q9: sau sửa nguồn cuối chưa đủ 1 s");
            Lane(r.send({"left"},.9f,.5f,.8f,39),"Q9: nguồn cuối tươi 999 ms chưa hồi");
            Lane(r.send({},.9f,.5f,.8f,1),"Q9: nguồn cuối tươi đủ 1000 ms, không rẽ biển cũ");
            r.votes({"right"}); const auto plan=r.junction();
            Expect(plan.state==PlannerState::RE_PHAI,plan,"Q9: cả hai điều kiện tốt đủ 1 s thì biển mới rẽ được");
        }
    } else throw std::runtime_error("Ca Q9 không tồn tại: "+name);
}
// T68b Chỉ làm §1–4: đếm lần bằng phiếu/encoder, không dùng yaw để đếm (ca rẽ theo rb_turn1/2_cm đã bỏ).
void T68b(const std::string& name) {
    if (name=="visible") {
        for (const std::string other:{"","left","work_barrier"}) {
            Rig r; r.enter();
            for (int i=1;i<=10;++i) Lane(r.at(30*i,other.empty()?std::vector<std::string>{"round_barrier"}:std::vector<std::string>{"round_barrier",other}),"Còn phiếu vòng xuyến không rẽ, lớp kèm="+other);
        }
    } else if (name=="sign_priority") {
        Rig r; r.enter(); r.leave(); r.p.turn_start_cm=60;
        r.votes({"right"});
        r.at(r.in.odo_cm+60); const auto plan=r.junction();
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
        // Quân 12/11: hết lần qua mà mất làn thì giữ hướng IMU theo hướng lúc mất làn tối đa 60 cm / 2 s; bánh đứng 1500 ms thì thôi giữ.
        // Hết giữ hướng mà vẫn mất làn thì dừng như cũ (không đổi giới hạn dừng).
        Expect(plan.state==PlannerState::GIU_HUONG || plan.state==PlannerState::BAM_LAN,plan,"Hết lượt: giữ hướng ngắn hoặc bám làn");
        r.in.detections={};
        for (int i=0;i<(1500+r.p.lost_stop_ms+200)/40;++i) r.send();
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
    } else if (name=="space_go") {
        // Luật mới: Space bỏ lượt dở; vòng xuyến còn thấy sau G là lượt mới.
        Rig r; r.hold(3,13);
        r.in.stop=true; ++r.now; const auto stopped=r.step();
        Expect(stopped.cmd.cms==0 && stopped.cmd.deg==0,stopped,"Space khi giữ hướng phải dừng");
        r.in.stop=false; r.mask(); r.in.yaw_deg=40;
        r.in.go=true; r.send({"round_barrier"}); r.in.go=false;
        r.votes({"round_barrier"},r.p.regain_frames+1);
        r.in.yaw_deg=45; r.in.mask.setTo(0);
        const auto plan=r.votes({"round_barrier"},12);
        Expect(plan.state==PlannerState::GIU_HUONG && Near(plan.cmd.deg,-5*r.p.rb_yaw_kp),
               plan,"Lượt mới dùng yaw0=40 sau G, không dùng yaw0=3 trước Space");
    } else if (name=="reset") {
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
                const auto obstacle=baseline.votes({"obstacle"},baseline.p.vote_window+2);
                Expect(obstacle.reason=="NE_TRAI" || obstacle.mode==control::TrajMode::AVOID_LEFT || obstacle.reason=="KHONG_DUONG_NE",obstacle,"Đối chứng cùng mask ngoài vòng xuyến phải nhận vật cản (né hoặc dừng vì không có đường né)");
            }
            for (int i=0;i<r.p.vote_window+2;++i) {
                r.in.odo_cm+=1;
                const auto plan=r.send(name=="block_obstacle"?std::vector<std::string>{"round_barrier","obstacle"}:std::vector<std::string>{"round_barrier",sign});
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
        Rig r; auto plan=r.votes({"person"});
        Expect(plan.cmd.cms==0,plan,"Thiết lập dừng người");
        plan=r.send({"people"},.4f,.5f,.5f);
        Expect(plan.cmd.cms==0,plan,"Hộp xa và yếu vẫn giữ chốt");
        const auto seen=r.now; const double odo=r.in.odo_cm;
        r.now=seen+1900-40; plan=r.send();
        Expect(plan.cmd.cms==0,plan,"Vắng 1,9 s vẫn dừng");
        r.now=seen+2000; plan=r.step();
        Expect(plan.cmd.cms==0,plan,"Lô rỗng cũ chưa đủ 2 s không nhả");
        r.now=seen+2000; r.in.detections.captured_ms=seen-r.p.det_max_age_ms; plan=r.step();
        Expect(plan.cmd.cms==0,plan,"Mất YOLO không nhả");
        plan=r.send({},.9f,.5f,.8f,0);
        Lane(plan,"Lô tươi vắng đúng 2 s tự đi tiếp không G");
        Check(r.in.odo_cm==odo,"Đầu vào encoder giữ nguyên khi dừng");
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
        for(int elapsed=100;elapsed<2000;elapsed+=100) {
            plan=r.send({},.9f,.5f,.8f,100);
            Expect(plan.state==PlannerState::RE_TRAI && plan.cmd.cms==0,plan,"Chờ đủ 2 s vẫn giữ lượt rẽ");
        }
        plan=r.send({},.9f,.5f,.8f,100);
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
        Check(defaults.work_barrier_action==1 && Near(defaults.work_barrier_y2_frac,.55),"Sai mặc định rào 1/0,55");
        fields={{"work_barrier_y2_frac",&control::ControlParams::work_barrier_y2_frac,.30,.95,.55}};
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
        else if (group=="q7") Q7(scenario);
        else if (group=="q9") Q9(scenario);
        else throw std::runtime_error("Tiền tố không hợp lệ: "+group);
        std::cout<<"ĐẠT: acc_"<<name<<'\n'; return 0;
    } catch (const std::exception& e) {
        std::cerr<<"KHÔNG ĐẠT: "<<(argc>1?argv[1]:"?")<<": "<<e.what()<<'\n'; return 1;
    }
}
