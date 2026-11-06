#include "control/Planner.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace {
void Check(bool ok, const std::string& message) {
    if (!ok) throw std::runtime_error(message);
}
struct Rig {
    control::Planner planner;
    control::ControlParams p;
    control::PlannerInput in;
    int64_t now=1000;
    uint64_t frame=0;
    Rig(bool sensor_ok=true) {
        p.turn_enable=1; p.start_wait_green=0; p.kd=0;
        p.lane_width_frac={0.4,0.4,0.4};
        in.yolo_enabled=true; in.sensor_ok=sensor_ok;
        mask(60,140);
        in.go=true; step(); in.go=false;
    }
    void mask(int begin, int end) {
        in.mask=cv::Mat(60,200,CV_8UC1,cv::Scalar(0));
        for (int y=0;y<60;++y) {
            for (int x=begin;x<end;++x) in.mask.at<unsigned char>(y,x)=1;
            if (begin>0) in.mask.at<unsigned char>(y,begin)=2;
            if (end<200) in.mask.at<unsigned char>(y,end)=2;
        }
    }
    control::Plan step(bool fresh_mask=true, bool fresh_sensor=true) {
        in.image_ms=now;
        if (fresh_mask) in.mask_ms=now;
        if (fresh_sensor) in.sensor_ms=now;
        return planner.Step(in,now,p);
    }
    control::Plan send(const std::vector<std::string>& names, float confidence=0.9f) {
        now+=40;
        std::vector<control::Detection> items;
        for (const auto& name:names)
            items.push_back({name,cv::Rect2f(1024,100,128,72),confidence,now});
        in.detections={items,now,++frame};
        return step();
    }
    void arm(const std::string& name="left") {
        for (int i=0;i<p.vote_min;++i) {
            auto result=send({name});
            Check(result.state==control::PlannerState::BAM_LAN,"Xác nhận vẫn bám làn");
        }
    }
    control::Plan advance(double cm) {
        in.odo_cm+=cm; now+=40; return step();
    }
    control::Plan turn(bool left=true) {
        arm(left?"left":"right"); advance(p.turn_start_cm/2); auto result=advance(p.turn_start_cm/2);
        Check(control::StateName(result.state)==std::string(left?"RE_TRAI":"RE_PHAI"),"Thiết lập phải đang rẽ");
        return result;
    }
    void obstacle(bool present) {
        in.mask=cv::Mat(100,200,CV_8UC1,cv::Scalar(0));
        in.mask.colRange(20,181).setTo(1);
        in.mask.col(20).setTo(2); in.mask.col(180).setTo(2);
        if (present) in.mask(cv::Rect(90,40,40,55)).setTo(0);
    }
    void avoid() {
        p.obstacle_action=2; obstacle(true);
        control::Plan plan;
        for (int i=0;i<p.vote_min;++i) { now+=40; plan=step(); }
        Check(plan.state==control::PlannerState::BAM_LAN && plan.reason=="NE_TRAI" &&
              plan.reference==control::LaneRef::LEFT,"Thiết lập phải đang né trái");
    }
};
void Cancelled(const control::Plan& plan, const control::ControlParams& p) {
    Check(plan.state==control::PlannerState::BAM_LAN && plan.reason=="HUY_RE_CAM_BIEN" &&
          plan.debug.state=="BAM_LAN" && plan.debug.reason==plan.reason &&
          plan.cmd.cms>0 && plan.cmd.cms<=p.max_speed_cms && std::abs(plan.cmd.deg)<=p.steer_max_deg,
          "Lỗi rẽ phải về BAM_LAN/HUY_RE_CAM_BIEN, lệnh trong giới hạn");
}
void TurnUnavailable(Rig& r) {
    r.mask(60,140); r.in.detections={};
    for (bool left:{true,false}) {
        r.arm(left?"left":"right"); r.advance(r.p.turn_start_cm/2);
        Check(r.advance(r.p.turn_start_cm/2).state==control::PlannerState::BAM_LAN,"Lỗi cảm biến phải bỏ rẽ hết phiên");
    }
}
void RecoveredSensor(Rig& r) {
    r.in.sensor_ok=true; r.in.yaw_deg=0; r.in.odo_cm=0;
    TurnUnavailable(r);
    r.in.stop=true; ++r.now; Check(r.step().cmd.cms==0,"Space vẫn phải dừng"); r.in.stop=false;
    r.in.go=true; ++r.now; Check(r.step().cmd.cms>0,"Space/G vẫn cho xe chạy"); r.in.go=false;
    TurnUnavailable(r);
    for (bool preserve:{false,true}) {
        r.planner.Reset(preserve); r.in.go=true; ++r.now;
        Check(r.step().cmd.cms>0,"Reset/G vẫn cho xe chạy"); r.in.go=false;
        TurnUnavailable(r);
    }
}

// T57_design:3,17: sáu mặc định, hai biên hợp lệ, ngoài miền và quan hệ góc.
void Params(const std::string& path) {
    control::ControlParams p;
    Check(p.turn_enable==1 && p.turn_yaw_deg==80 && p.turn_steer_deg==25 &&
          p.turn_arm_cm==80 && p.turn_max_cm==70 && p.turn_max_ms==8000,"Sai sáu mặc định");
    const char* names[]={"turn_enable","turn_yaw_deg","turn_steer_deg","turn_arm_cm","turn_max_cm","turn_max_ms"};
    const double low[]={0,30,5,10,10,1000}, high[]={1,120,30,300,200,20000};
    for (int i=0;i<6;++i) {
        for (double value:{low[i],high[i],low[i]-1,high[i]+1}) {
            { std::ofstream out(path); Check(bool(out),"Không mở được YAML tạm");
              out<<"steer_max_deg: 30\n"<<names[i]<<": "<<value<<'\n';
              if (i==3) out<<"turn_start_cm: 0\n";
              if (i==2) out<<"turn_enable: 1\n"; }
            bool rejected=false;
            try { std::ostringstream report; auto loaded=control::ControlParams::Load(path,report); loaded.Validate(); }
            catch (const std::exception&) { rejected=true; }
            Check(rejected==(value<low[i] || value>high[i]),std::string("Sai miền YAML ")+names[i]);
        }
    }
    p.steer_max_deg=30; p.turn_steer_deg=p.steer_max_deg+1;
    p.turn_enable=1;
    bool rejected=false; try { p.Validate(); } catch (const std::exception&) { rejected=true; }
    Check(rejected,"Validate phải từ chối góc rẽ quá trần khi bật rẽ");
    p.turn_enable=0;
    rejected=false; try { p.Validate(); } catch (const std::exception&) { rejected=true; }
    Check(!rejected,"Validate phải chấp nhận góc rẽ quá trần khi tắt rẽ");
}

// T68a Chỉ làm 3,5: kiểm cả loader lẫn Validate, không thay khóa sản phẩm.
void T68Params(const std::string& path) {
    control::ControlParams defaults;
    Check(defaults.turn_start_cm==30,"Mặc định turn_start_cm phải là30");
    auto load=[&](const std::string& body) {
        { std::ofstream out(path); Check(bool(out),"Không mở được YAML tạm");
          out<<"steer_max_deg: 30\n"<<body; }
        std::ostringstream report;
        return control::ControlParams::Load(path,report);
    };
    Check(load("").turn_start_cm==30,"Thiếu khóa phải dùng mặc định30");
    for (double value:{0.0,30.0,150.0}) {
        const auto p=load("turn_start_cm: "+std::to_string(value)+"\nturn_arm_cm: 151\n");
        Check(p.turn_start_cm==value,"Loader phải nhận đúng turn_start_cm");
        p.Validate();
    }
    for (double value:{-0.1,150.1}) {
        auto p=defaults; p.turn_start_cm=value; p.turn_arm_cm=200;
        bool rejected=false; try { p.Validate(); } catch (const std::exception&) { rejected=true; }
        Check(rejected,"Validate phải từ chối -0,1/150,1");
        rejected=false;
        try { load("turn_start_cm: "+std::to_string(value)+"\nturn_arm_cm: 200\n").Validate(); }
        catch (const std::exception&) { rejected=true; }
        Check(rejected,"YAML ngoài miền phải bị từ chối");
    }
    for (double value:{80.0,80.1}) {
        auto p=defaults; p.turn_start_cm=value;
        bool rejected=false; try { p.Validate(); } catch (const std::exception&) { rejected=true; }
        Check(rejected,"Validate phải từ chối start >= arm");
    }
}

void Case(const std::string& name) {
    // T57_design:5,20,25: đủ phiếu bốn lớp, không thêm ngưỡng tin cậy.
    if (name=="signs" || name=="t68_signs") {
        for (const std::string sign:{"left","no_right","right","no_left"}) {
            Rig r; const bool left=sign=="left" || sign=="no_right";
            r.arm(sign); Check(r.advance(r.p.turn_start_cm-1).state==control::PlannerState::BAM_LAN,"Chưa đủ quãng chưa rẽ");
            const auto plan=r.advance(1);
            Check(control::StateName(plan.state)==std::string(left?"RE_TRAI":"RE_PHAI") &&
                  plan.reason==control::StateName(plan.state) && plan.debug.state==plan.reason &&
                  plan.debug.reason==plan.reason,"Sai hướng/lý do/debug biển");
        }
    } else if (name=="t68_no_sign") {
        Rig r;
        for (bool open:{false,true}) {
            r.mask(open?0:60,140);
            for (int i=0;i<4;++i)
                Check(r.advance(r.p.turn_start_cm).state==control::PlannerState::BAM_LAN,"Không biển thì encoder/mép mở không được gây rẽ");
        }
    } else if (name=="t68_start_distance") {
        for (bool left:{false,true}) {
            Rig r;
            for (int i=0;i<r.p.vote_min;++i) {
                r.in.odo_cm=100+10*i;
                Check(r.send({left?"left":"right"}).state==control::PlannerState::BAM_LAN,"Chưa đủ quãng từ phiếu cuối");
            }
            Check(r.advance(29).state==control::PlannerState::BAM_LAN,"29cm từ lúc đủ phiếu chưa rẽ");
            Check(control::StateName(r.advance(1).state)==std::string(left?"RE_TRAI":"RE_PHAI"),"Đúng30cm từ phiếu cuối phải rẽ");
        }
    } else if (name=="t68_conflict") {
        for (const auto& signs: {std::vector<std::string>{"left","right"},
                {"no_left","no_right"},{"left","no_left"},{"right","no_right"}}) {
            Rig r;
            for (int i=0;i<r.p.vote_window;++i) r.send(signs);
            Check(r.advance(r.p.turn_start_cm).state==control::PlannerState::BAM_LAN,"Hai hướng xung đột không được armed");
            r.mask(0,200); Check(r.advance(1).state==control::PlannerState::BAM_LAN,"Mép mở không giải quyết xung đột biển");
        }
    } else if (name=="t68_red_hold") {
        for (bool left:{false,true}) {
            Rig r; r.in.odo_cm=100; r.arm(left?"left":"right");
            control::Plan plan;
            for (int i=0;i<r.p.vote_min;++i) plan=r.send({"red"});
            const auto red_start=r.now;
            for (int i=0;i<250;++i) {
                plan=r.send({"red",left?"right":"left"});
                Check(plan.state==control::PlannerState::DUNG_DEN_DO && plan.cmd.cms==0 && plan.cmd.deg==0,
                      "Giữ đỏ10s phải dừng motor/servo");
            }
            Check(r.now-red_start==10000 && r.in.odo_cm==100,"Thiết lập đỏ10s, encoder đứng yên");
            for (int i=0;i<r.p.vote_window+r.p.green_frames;++i) plan=r.send({"green"});
            Check(plan.state==control::PlannerState::BAM_LAN,"Đủ xanh nhả đỏ");
            Check(r.advance(29).state==control::PlannerState::BAM_LAN,"Đỏ không đổi mốc đủ phiếu");
            Check(control::StateName(r.advance(1).state)==std::string(left?"RE_TRAI":"RE_PHAI"),"Đỏ10s và biển ngược không đổi hướng chốt");
        }
    } else if (name=="t68_avoidance" || name=="t68_arm_distance") {
        Rig r; r.arm(); r.avoid();
        const bool cancel=name=="t68_arm_distance";
        auto plan=r.advance(cancel?r.p.turn_arm_cm:r.p.turn_start_cm);
        Check(plan.state==control::PlannerState::BAM_LAN && plan.reason=="NE_TRAI" &&
              plan.reference==control::LaneRef::LEFT,"Đủ quãng khi NE_TRAI không được rẽ/hủy ở đúng80");
        if (cancel) {
            plan=r.advance(0.01);
            Check(plan.state==control::PlannerState::BAM_LAN && plan.reason=="HUY_RE","Quá80cm khi né phải HUY_RE");
            r.in.odo_cm=r.p.turn_start_cm;
        }
        r.obstacle(false);
        for (int i=0;i<r.p.vote_window;++i) { r.now+=40; plan=r.step(); }
        Check(plan.state==control::PlannerState::BAM_LAN && plan.reason=="NE_QUA" &&
              plan.reference==control::LaneRef::LEFT,"Đủ quãng khi NE_QUA không được rẽ");
        bool returned=false;
        for (int i=0;i<400;++i) {
            r.now+=100; plan=r.step();
            if (plan.reason=="VE_PHAI") {
                Check(plan.state==control::PlannerState::BAM_LAN && plan.reference==control::LaneRef::RIGHT,"VE_PHAI phải trả làn trước khi rẽ");
                returned=true; break;
            }
            Check(plan.state==control::PlannerState::BAM_LAN && plan.reason=="NE_QUA","Chưa hết pha né không được bắt đầu rẽ");
        }
        Check(returned,"Phải kết thúc pha né trong giới hạn mô phỏng");
        r.now+=40; plan=r.step();
        Check(plan.state==(cancel?control::PlannerState::BAM_LAN:control::PlannerState::RE_TRAI),
              cancel?"Đã hủy thì về bám làn không rẽ lại":"Về bám làn thường mới được rẽ");
    } else if (name=="t68_hold_heading") {
        Rig r; r.arm();
        for (int i=0;i<r.p.vote_min;++i) r.send({"round_barrier"});
        r.in.mask.setTo(0);
        control::Plan plan;
        for (int i=0;i<12;++i) plan=r.send({"round_barrier"});
        Check(plan.state==control::PlannerState::GIU_HUONG && plan.cmd.cms>0,"Thiết lập phải GIU_HUONG");
        plan=r.advance(r.p.turn_start_cm);
        Check(plan.state==control::PlannerState::GIU_HUONG,"Đang GIU_HUONG đủ quãng cũng không bắt đầu rẽ");
    } else if (name=="votes") {
        for (const std::string mode:{"conflict","repeat","few","stale","barrier"}) {
            Rig r;
            for (int i=0;i<r.p.vote_min;++i) {
                if (mode=="conflict") r.send({"left","right"});
                else if (mode=="repeat" && i>0) { r.now+=40; r.step(); }
                else if (mode=="few" && i==r.p.vote_min-1) break;
                else r.send({mode=="barrier"?"work_barrier":"left"});
                if (mode=="stale") r.now+=r.p.det_max_age_ms+1;
            }
            auto plan=r.advance(r.p.turn_start_cm/2); plan=r.advance(r.p.turn_start_cm/2);
            Check(control::StateName(plan.state)==std::string("BAM_LAN"),"Sai luật phiếu "+mode);
        }
    } else if (name=="obstacle_stop_no_turn") {
        // Đang DUNG_VAT_CAN (obstacle_action 1), đủ quãng cũng không được bắt đầu rẽ.
        Rig r; r.p.obstacle_action=1; r.arm();
        r.mask(60,140);
        for (int y=24;y<57;++y) for (int x=90;x<120;++x) r.in.mask.at<unsigned char>(y,x)=0;
        control::Plan plan;
        for (int i=0;i<r.p.vote_window;++i) { r.now+=40; plan=r.step(); }
        Check(plan.state==control::PlannerState::DUNG_VAT_CAN && plan.cmd.cms==0,"Thiết lập phải DUNG_VAT_CAN");
        r.in.odo_cm+=r.p.turn_start_cm;
        for (int i=0;i<4;++i) {
            r.mask(0,140);
            for (int y=24;y<57;++y) for (int x=90;x<120;++x) r.in.mask.at<unsigned char>(y,x)=0;
            r.now+=40; plan=r.step();
            Check(plan.state==control::PlannerState::DUNG_VAT_CAN && plan.cmd.cms==0 && plan.cmd.deg==0,
                  "Đang dừng vật cản không được rẽ");
        }
    } else if (name=="barrier_turn") {
        // Rào gần đủ phiếu → RE_TRAI bằng máy rẽ IMU; yaw đủ → BAM_LAN; rào còn trong ảnh không rẽ lại;
        // rào xa hoặc lệch ngang không kích hoạt; work_barrier_action 0 không kích hoạt.
        auto box=[&](Rig& r,float cx,float y2) {
            r.now+=40;
            r.in.detections={{{"work_barrier",cv::Rect2f(cx*1280-40,y2*720-60,80,60),0.9f,r.now}},r.now,++r.frame};
            return r.step();
        };
        { Rig r; control::Plan plan; for (int i=0;i<5;++i) plan=box(r,0.5f,0.58f);
          Check(plan.state==control::PlannerState::BAM_LAN && plan.cmd.cms<=r.p.approach_cms,"Rào xa: giảm tốc, chưa rẽ"); }
        { Rig r; control::Plan plan; for (int i=0;i<5;++i) plan=box(r,0.86f,0.80f);
          Check(plan.state==control::PlannerState::BAM_LAN,"Rào lệch ngang không rẽ"); }
        { Rig r; r.p.work_barrier_action=0; control::Plan plan; for (int i=0;i<5;++i) plan=box(r,0.5f,0.80f);
          Check(plan.state==control::PlannerState::BAM_LAN && plan.cmd.cms>r.p.approach_cms,"Tắt rào: không đổi"); }
        { Rig r; r.in.yaw_deg=5; control::Plan plan;
          for (int i=0;i<r.p.vote_min;++i) plan=box(r,0.5f,0.80f);
          Check(plan.state==control::PlannerState::RE_TRAI && plan.cmd.cms==r.p.min_cms && plan.cmd.deg>0,"Rào gần đủ phiếu phải rẽ trái");
          plan=box(r,0.83f,0.80f); Check(plan.state==control::PlannerState::RE_TRAI,"Hộp đổi bên khi đang rẽ không đổi quyết định");
          r.in.yaw_deg=5+r.p.turn_yaw_deg; plan=box(r,0.5f,0.80f);
          Check(plan.state==control::PlannerState::BAM_LAN,"Đủ yaw về bám làn");
          for (int i=0;i<6;++i) { plan=box(r,0.5f,0.80f); Check(plan.state==control::PlannerState::BAM_LAN,"Rào còn trong ảnh không rẽ lại"); } }
        { Rig r; r.in.sensor_ok=false; control::Plan plan; for (int i=0;i<5;++i) plan=box(r,0.5f,0.80f);
          Check(plan.state==control::PlannerState::BAM_LAN && plan.cmd.cms>0,"IMU lỗi: không rẽ, không dừng"); }
    } else if (name=="roundabout") {
        // Đếm lần qua bằng phiếu round_barrier; sau lần 1 đi rb_turn1_cm rồi RE_TRAI; sau lần 2 RE_PHAI;
        // trong lần qua mất làn thì GIU_HUONG lái về yaw lúc vào.
        Rig r; r.in.yaw_deg=10; r.in.odo_cm=0;
        auto rb=[&](bool seen) {
            r.now+=40;
            std::vector<control::Detection> items;
            if (seen) items.push_back({"round_barrier",cv::Rect2f(600,300,80,80),0.9f,r.now});
            r.in.detections={items,r.now,++r.frame};
            return r.step();
        };
        control::Plan plan;
        for (int i=0;i<r.p.vote_min;++i) plan=rb(true);
        Check(plan.state==control::PlannerState::BAM_LAN,"Thấy vòng xuyến vẫn bám làn");
        r.in.mask=cv::Mat(60,200,CV_8UC1,cv::Scalar(0)); r.in.yaw_deg=25;
        for (int i=0;i<12;++i) plan=rb(true);
        Check(plan.state==control::PlannerState::GIU_HUONG && plan.cmd.deg<0 && plan.cmd.cms>0,"Mất làn trong lần qua phải giữ hướng (lái phải về yaw0)");
        r.now+=40; r.in.detections={{{"round_barrier",cv::Rect2f(600,300,80,80),0.9f,r.now},{"work_barrier",cv::Rect2f(600,500,80,60),0.9f,r.now}},r.now,++r.frame};
        plan=r.step(); r.now+=40; r.in.detections.frame_id=++r.frame; r.in.detections.captured_ms=r.now;
        for (auto& d:r.in.detections.items) d.captured_ms=r.now;
        plan=r.step(); r.now+=40; r.in.detections.frame_id=++r.frame; r.in.detections.captured_ms=r.now;
        for (auto& d:r.in.detections.items) d.captured_ms=r.now;
        plan=r.step();
        Check(plan.state==control::PlannerState::GIU_HUONG,"Đang giữ hướng thì rào không bắt đầu rẽ");
        r.in.yaw_deg=10+360; plan=rb(true); Check(std::abs(plan.cmd.deg)<=std::abs(r.p.rb_yaw_kp*15)+1,"Gói góc 360");
        r.in.yaw_deg=10; r.mask(60,140);
        for (int i=0;i<6;++i) plan=rb(true);
        Check(plan.state==control::PlannerState::BAM_LAN,"Bắt lại làn thì bám làn");
        for (int i=0;i<8;++i) { r.in.odo_cm+=1; plan=rb(false); }
        Check(plan.state==control::PlannerState::BAM_LAN,"Chưa đủ rb_lost_cm chưa hết lần qua");
        for (int i=0;i<30;++i) { r.in.odo_cm+=1; plan=rb(false); }
        Check(plan.state==control::PlannerState::BAM_LAN,"Hết lần qua 1, chưa đủ rb_turn1_cm");
        for (int i=0;i<90 && plan.state==control::PlannerState::BAM_LAN;++i) { r.in.odo_cm+=1; plan=rb(false); }
        Check(plan.state==control::PlannerState::RE_TRAI,"Sau lần 1 phải rẽ trái");
        r.in.yaw_deg=10+r.p.turn_yaw_deg; plan=rb(false); Check(plan.state==control::PlannerState::BAM_LAN,"Xong rẽ");
        for (int i=0;i<r.p.vote_min;++i) plan=rb(true);
        for (int i=0;i<35;++i) { r.in.odo_cm+=1; plan=rb(false); }
        for (int i=0;i<120 && plan.state==control::PlannerState::BAM_LAN;++i) { r.in.odo_cm+=1; plan=rb(false); }
        Check(plan.state==control::PlannerState::RE_PHAI,"Sau lần 2 phải rẽ phải");
        r.in.yaw_deg-=r.p.turn_yaw_deg; plan=rb(false);
        for (int i=0;i<r.p.vote_min;++i) plan=rb(true);
        for (int i=0;i<200;++i) { r.in.odo_cm+=1; plan=rb(false);
            Check(plan.state==control::PlannerState::BAM_LAN,"Sau lần 2 không rẽ nữa"); }
    } else if (name=="roundabout_stall") {
        // Giữ hướng mà bánh không tiến (kẹt đảo/rào) thì hết GIU_HUONG sau 1500 ms, về dừng mất làn như cũ.
        Rig r; r.in.yaw_deg=10; r.in.odo_cm=0;
        auto rb=[&]() {
            r.now+=40;
            r.in.detections={{{"round_barrier",cv::Rect2f(600,300,80,80),0.9f,r.now}},r.now,++r.frame};
            return r.step();
        };
        control::Plan plan;
        for (int i=0;i<r.p.vote_min;++i) plan=rb();
        r.in.mask=cv::Mat(60,200,CV_8UC1,cv::Scalar(0));
        plan=rb(); Check(plan.state==control::PlannerState::GIU_HUONG,"Mất làn trong lần qua phải giữ hướng");
        for (int i=0;i<30;++i) plan=rb();  // 1200 ms, odo đứng yên
        Check(plan.state==control::PlannerState::GIU_HUONG,"Chưa tới 1500 ms vẫn giữ hướng");
        for (int i=0;i<10;++i) plan=rb();
        Check(plan.state!=control::PlannerState::GIU_HUONG,"Bánh không tiến 1500 ms phải hết giữ hướng");
        for (int i=0;i<60;++i) plan=rb();
        Check(plan.cmd.cms==0 && plan.state!=control::PlannerState::GIU_HUONG,"Sau đó dừng do mất làn, không giữ hướng lại");
    } else if (name=="fault_recovers") {
        // Một mẫu cảm biến cũ chỉ tắt rẽ tạm; cảm biến tươi lại >= 1 s thì biển mới rẽ được.
        Rig r; r.in.odo_cm=0;
        r.now+=40; r.in.sensor_ms=r.now-300; r.step(true,false);  // một mẫu cũ 300 ms
        control::Plan plan;
        for (int i=0;i<r.p.vote_min;++i) plan=r.send({"left"},0.9f);
        r.in.odo_cm+=r.p.turn_start_cm+1; plan=r.step();
        Check(plan.state==control::PlannerState::BAM_LAN,"Ngay sau lỗi không được rẽ");
        for (int i=0;i<30;++i) { r.now+=40; r.step(); }  // 1,2 s cảm biến tươi
        for (int i=0;i<r.p.vote_min;++i) plan=r.send({"left"},0.9f);
        r.in.odo_cm+=r.p.turn_start_cm+1; plan=r.step();
        Check(plan.state==control::PlannerState::RE_TRAI,"Cảm biến tươi lại 1 s thì phải rẽ được");
    } else if (name=="low_conf_armed") {
        Rig r;
        for (int i=0;i<r.p.vote_min;++i) r.send({"left"},0.01f);
        r.advance(r.p.turn_start_cm/2); Check(control::StateName(r.advance(r.p.turn_start_cm/2).state)==std::string("RE_TRAI"),"Conf thấp đủ phiếu vẫn armed");
    } else if (name=="space_during_turn") {
        for (bool turning:{false,true}) for (bool preserve:{false,true}) {
            Rig r; r.in.yaw_deg=37; r.in.odo_cm=100;
            if (turning) r.turn(); else r.arm();
            r.in.stop=true; ++r.now; Check(r.step().cmd.cms==0,"Space phải dừng");
            r.planner.Reset(preserve); r.in.stop=false; r.in.detections={};
            r.in.yaw_deg=-300; r.in.odo_cm=-300; r.in.go=true; ++r.now;
            Check(r.step().state==control::PlannerState::BAM_LAN,"G sau Space bỏ mốc cũ"); r.in.go=false;
            r.advance(r.p.turn_start_cm/2); Check(r.advance(r.p.turn_start_cm/2).state==control::PlannerState::BAM_LAN,"Space xóa armed/hướng cũ");
            r.mask(60,140); r.in.yaw_deg=10; r.in.odo_cm=0;
            r.turn(false); r.in.yaw_deg=10+r.p.turn_yaw_deg; ++r.now;
            Check(r.step().state==control::PlannerState::BAM_LAN,"Phiên mới dùng yaw0 mới");
        }
    } else if (name=="nonfinite_sensor" || name=="armed_sensor_fault") {
        if (name=="armed_sensor_fault") {
            Rig startup(false);
            const auto plan=startup.step();
            Check(plan.state==control::PlannerState::BAM_LAN && plan.cmd.cms>0,
                  "Cảm biến lỗi khởi động vẫn bám làn");
            TurnUnavailable(startup); RecoveredSensor(startup);
            Rig fresh; fresh.turn();
            for (bool stale_image:{false,true}) {
                Rig ignored; ignored.planner.Reset(false);
                const auto waiting=ignored.step();
                ignored.now+=1000; ignored.in.go=true; ignored.in.sensor_ok=false;
                ignored.in.sensor_ms=ignored.in.image_ms=ignored.in.mask_ms=ignored.now;
                if (stale_image) ignored.in.image_ms-=501;
                else ignored.in.mask_ms-=ignored.p.mask_max_age_ms+1;
                const auto plan=ignored.planner.Step(ignored.in,ignored.now,ignored.p);
                Check(plan.state==waiting.state && plan.cmd.cms==0 && plan.cmd.deg==0,
                      "G cùng ảnh/mask cũ vẫn phải giữ chờ dù cảm biến lỗi");
                ignored.in.sensor_ok=true; ++ignored.now;
                const auto resumed=ignored.step(); ignored.in.go=false;
                Check(resumed.state==control::PlannerState::BAM_LAN && resumed.cmd.cms>0,
                      "G với ảnh/mask tươi vẫn chạy sau lỗi cảm biến lúc G bị bỏ qua");
                RecoveredSensor(ignored);
            }
        }
        for (int fault=(name=="armed_sensor_fault"?0:1);fault<(name=="armed_sensor_fault"?1:8);++fault) {
            for (int phase=0;phase<3;++phase) {
                Rig r;
                if (phase==1) r.arm();
                if (phase==2) r.turn();
                if (fault==0) r.in.sensor_ok=false;
                if (fault>=1 && fault<=3) r.in.yaw_deg=fault==1?std::numeric_limits<double>::quiet_NaN():
                    (fault==2?1:-1)*std::numeric_limits<double>::infinity();
                if (fault>=4 && fault<=6) r.in.odo_cm=fault==4?std::numeric_limits<double>::quiet_NaN():
                    (fault==5?1:-1)*std::numeric_limits<double>::infinity();
                if (fault==7) r.in.sensor_ms=r.now+1;
                const auto plan=r.step(true,fault!=7);
                if (phase==2) Cancelled(plan,r.p);
                else Check(plan.state==control::PlannerState::BAM_LAN && plan.cmd.cms>0,
                           "Lỗi cảm biến khi chưa rẽ vẫn bám làn");
                RecoveredSensor(r);
            }
        }
    } else if (name=="armed_red" || name=="red_release_after_turn_red") {
        Rig r; r.p.vote_window=r.p.vote_min; r.in.odo_cm=100;
        const bool turning=name=="red_release_after_turn_red";
        if (turning) r.turn(); else r.arm();
        control::Plan plan;
        for (int i=0;i<r.p.vote_min;++i) plan=r.send({"red"});
        Check(plan.state==control::PlannerState::DUNG_DEN_DO && plan.cmd.cms==0,"Đỏ dừng phiên rẽ/armed");
        for (int i=0;i<r.p.vote_window+r.p.green_frames;++i) plan=r.send({"green"});
        Check(plan.state==control::PlannerState::BAM_LAN,"Nhả đỏ về bám làn");
        if (turning) { r.advance(r.p.turn_start_cm/2); Check(r.advance(r.p.turn_start_cm/2).state==control::PlannerState::BAM_LAN,"Không rẽ tiếp yaw0 trước đỏ"); }
        else {
            r.in.odo_cm=100+r.p.turn_arm_cm+0.01; ++r.now;
            Check(r.step().reason=="HUY_RE","Đỏ giữ armed và mốc odo cũ");
        }
        if (!turning) {
            Rig kept; kept.p.vote_window=kept.p.vote_min; kept.arm();
            for (int i=0;i<kept.p.vote_min;++i) kept.send({"red"});
            for (int i=0;i<kept.p.vote_window+kept.p.green_frames;++i) kept.send({"green"});
            kept.advance(kept.p.turn_start_cm/2); Check(control::StateName(kept.advance(kept.p.turn_start_cm/2).state)==std::string("RE_TRAI"),"Nhả đỏ còn armed khi trong quãng");
            for (bool release:{false,true}) {
                Rig fault; fault.p.vote_window=fault.p.vote_min; fault.arm();
                for (int i=0;i<fault.p.vote_min;++i) plan=fault.send({"red"});
                Check(plan.state==control::PlannerState::DUNG_DEN_DO,"Thiết lập phải giữ đỏ khi armed");
                fault.in.odo_cm+=fault.p.turn_start_cm;
                if (!release) {
                    fault.in.sensor_ok=false; ++fault.now; plan=fault.step();
                    Check(plan.state==control::PlannerState::DUNG_DEN_DO && plan.cmd.cms==0 && plan.cmd.deg==0,
                          "Lỗi cảm biến khi giữ đỏ vẫn phải dừng đèn");
                    fault.in.sensor_ok=true;
                }
                for (int i=0;i<fault.p.green_frames;++i) {
                    if (release && i==fault.p.green_frames-1) fault.in.sensor_ok=false;
                    plan=fault.send({"green"});
                }
                Check(plan.state==control::PlannerState::BAM_LAN && plan.cmd.cms>0,
                      "Lỗi khi giữ hoặc nhả đỏ phải về bám làn, không bắt đầu rẽ");
                RecoveredSensor(fault);
            }
        }
    } else if (name=="sign_while_turning") {
        Rig r; r.in.odo_cm=100; r.arm();
        r.in.odo_cm=110;
        for (int i=0;i<r.p.vote_window;++i) r.send({"right","no_left"});
        Check(control::StateName(r.advance(r.p.turn_start_cm-10).state)==std::string("RE_TRAI"),"Armed bỏ biển ngược");
        for (int i=0;i<r.p.vote_window;++i) Check(control::StateName(r.send({"right"}).state)==std::string("RE_TRAI"),"Đang rẽ không đổi hướng");
        r.in.yaw_deg=r.p.turn_yaw_deg; ++r.now; Check(r.step().state==control::PlannerState::BAM_LAN,"Biển mới không đổi yaw0");
        Rig distance; distance.in.odo_cm=100; distance.arm(); distance.in.odo_cm=120;
        for (int i=0;i<distance.p.vote_window;++i) distance.send({"left"});
        distance.in.odo_cm=100+distance.p.turn_arm_cm+0.01; ++distance.now;
        Check(distance.step().reason=="HUY_RE","Biển mới không đặt lại mốc armed");
        Rig travel; travel.turn(); const double turn_odo=travel.in.odo_cm; travel.in.odo_cm+=10;
        for (int i=0;i<travel.p.vote_window;++i) travel.send({"right"});
        travel.in.odo_cm=turn_odo+travel.p.turn_max_cm+0.01; ++travel.now;
        Cancelled(travel.step(),travel.p);
        Rig clock; clock.p.turn_max_ms=1000; clock.turn(); const auto start=clock.now;
        while (clock.now<start+clock.p.turn_max_ms) {
            const auto plan=clock.send({"right"});
            Check(control::StateName(plan.state)==std::string("RE_TRAI"),"Biển mới không đổi phiên trước hạn ms");
        }
        ++clock.now; Cancelled(clock.step(),clock.p);
    } else if (name=="after_turn_lane") {
        Rig r; r.p.kd=2; r.mask(40,120); ++r.now; r.step(); r.turn();
        r.mask(80,160); r.in.yaw_deg=r.p.turn_yaw_deg; ++r.now;
        auto plan=r.step();
        Check(plan.state==control::PlannerState::BAM_LAN && std::abs(plan.debug.d)<1e-9,"Kết thúc rẽ reset đạo hàm");
        ++r.now; plan=r.step(); Check(std::abs(plan.debug.d)<1e-9,"Không xung đạo hàm ở mask kế tiếp");
    } else if (name=="j1") {
        Rig r; r.arm();
        r.mask(0,140);
        for (int i=0;i<4;++i) { r.now+=40; Check(r.step().state==control::PlannerState::BAM_LAN,"J1 đúng nhưng chưa đi đủ quãng không được rẽ"); }
        Check(r.advance(r.p.turn_start_cm-1).state==control::PlannerState::BAM_LAN,"J1 đúng và29cm vẫn chưa rẽ");
    } else if (name=="arm_distance") {
        for (double sign:{-1.0,1.0}) {
            Rig r; r.in.odo_cm=100; r.arm();
            r.avoid();
            r.in.odo_cm=100+sign*r.p.turn_arm_cm; ++r.now;
            Check(r.step().reason!="HUY_RE","Đúng biên chưa hủy");
            r.in.odo_cm+=sign*0.01; ++r.now;
            auto plan=r.step(); Check(plan.state==control::PlannerState::BAM_LAN && plan.reason=="HUY_RE" && plan.debug.reason==plan.reason,"Quá quãng phải hủy");
            r.advance(r.p.turn_start_cm/2); Check(r.advance(r.p.turn_start_cm/2).state==control::PlannerState::BAM_LAN,"Hủy không được rẽ lại");
        }
    } else if (name=="yaw") {
        for (bool left:{false,true}) for (double sign:{-1.0,1.0}) {
            Rig r; r.in.yaw_deg=37; r.turn(left);
            r.in.yaw_deg=37+sign*(r.p.turn_yaw_deg-0.01); ++r.now;
            Check(control::StateName(r.step().state)==std::string(left?"RE_TRAI":"RE_PHAI"),"Chưa đủ yaw còn rẽ");
            r.in.yaw_deg=37+sign*r.p.turn_yaw_deg; ++r.now;
            const auto plan=r.step(); Check(plan.state==control::PlannerState::BAM_LAN,"Đúng yaw đích phải xong, kể cả yaw âm");
        }
    } else if (name=="command") {
        double left_angle=0;
        for (bool left:{false,true}) {
            Rig r; r.p.turn_steer_deg=r.p.steer_max_deg=25; r.p.steer_rate_dps=10;
            r.arm(left?"left":"right"); auto before=r.advance(r.p.turn_start_cm-1); auto plan=r.advance(1);
            Check(std::abs(plan.cmd.deg-before.cmd.deg)<=0.4+1e-6,"Giới hạn rate từ góc vừa gửi");
            double previous=plan.cmd.deg;
            for (int i=0;i<30;++i) {
                r.now+=100; r.in.odo_cm+=1; plan=r.step();
                Check(std::abs(plan.cmd.deg-previous)<=1+1e-6 && std::abs(plan.cmd.deg)<=25 && plan.cmd.cms==r.p.min_cms,"Rẽ giữ rate/max/min_cms");
                previous=plan.cmd.deg;
            }
            Check(std::abs(plan.cmd.deg)==25,"Phải đạt góc cố định, không chỉ kẹp về0");
            if (left) Check(left_angle*plan.cmd.deg<0,"Trái/phải phải có dấu góc ngược nhau");
            else left_angle=plan.cmd.deg;
            const double angle=plan.cmd.deg;
            r.in.mask.setTo(0); r.now+=40; plan=r.step();
            Check(plan.cmd.cms==r.p.min_cms && plan.cmd.deg==angle,"Đang rẽ bỏ qua mất làn");
        }
    } else if (name=="disabled") {
        Rig a,b; a.p.turn_enable=b.p.turn_enable=0;
        for (int i=0;i<12;++i) {
            a.in.sensor_ok=false; a.in.sensor_ms=-1; a.in.yaw_deg=999; a.in.odo_cm=-999;
            b.in.sensor_ok=true; b.in.yaw_deg=0; b.in.odo_cm=0;
            a.send({"left"}); b.send({"left"});
            auto x=a.advance(a.p.turn_start_cm), y=b.advance(b.p.turn_start_cm);
            Check(x.state==y.state && x.reason==y.reason && x.cmd.cms==y.cmd.cms && x.cmd.deg==y.cmd.deg && x.state==control::PlannerState::BAM_LAN,"Tắt phải bỏ qua cảm biến/biển rẽ");
        }
    } else if (name=="red") {
        Rig r; r.turn(); control::Plan plan;
        for (int i=0;i<r.p.vote_min;++i) plan=r.send({"red"});
        Check(plan.state==control::PlannerState::DUNG_DEN_DO && plan.cmd.cms==0 && plan.cmd.deg==0,"Đỏ khi rẽ phải dừng đèn");
    } else {
        // T73: lỗi cảm biến bỏ rẽ hết phiên, không chốt dừng bám làn.
        Rig r; r.turn(); const auto start=r.now;
        std::string reason=name;
        if (name=="fault_red") { for (int i=1;i<r.p.vote_min;++i) r.send({"red"}); reason="LOI_CAM_BIEN"; }
        if (reason=="LOI_CAM_BIEN") r.in.sensor_ok=false;
        else if (reason=="CAM_BIEN_CU") {
            r.now+=200; Check(r.step(true,false).state==control::PlannerState::RE_TRAI,"200ms vẫn hợp lệ"); ++r.now;
        } else if (reason=="sensor_invalid") { r.in.sensor_ms=-1; reason="CAM_BIEN_CU"; }
        else if (reason=="RE_QUA_CM") {
            r.in.odo_cm-=r.p.turn_max_cm; ++r.now;
            Check(r.step().state==control::PlannerState::RE_TRAI,"Đúng biên cm chưa lỗi"); r.in.odo_cm-=0.01;
        } else if (reason=="RE_QUA_MS") {
            for (int64_t t=start+100;t<=start+r.p.turn_max_ms;t+=100) { r.now=t; r.in.odo_cm+=0.1; r.step(); }
            Check(r.step().state==control::PlannerState::RE_TRAI,"Đúng biên ms chưa lỗi"); ++r.now;
        } else if (reason=="KET_RE") {
            Rig progress; progress.turn(); progress.now+=1500; progress.in.odo_cm+=1;
            Check(progress.step().state==control::PlannerState::RE_TRAI,"Đúng1cm/1500ms không phải kẹt");
            r.now=start+1499; Check(r.step().state==control::PlannerState::RE_TRAI,"Chưa1500ms không kẹt"); ++r.now;
        } else if (reason=="mask_stale") { r.now+=r.p.mask_max_age_ms+1; }
        else throw std::runtime_error("Ca không tồn tại: "+name);
        control::Plan plan;
        if (name=="fault_red") plan=r.send({"red"});
        else plan=r.step(name!="mask_stale",name!="CAM_BIEN_CU" && name!="sensor_invalid");
        if (name=="mask_stale") {
            // Đối chứng hành vi hiện có bằng Planner tắt T57, không suy luật chốt lỗi T57.
            Rig old; old.p.turn_enable=0; old.now=r.now; old.in.mask_ms=r.in.mask_ms;
            const auto baseline=old.step(false);
            Check(plan.cmd.cms==0 && plan.cmd.deg==0 && plan.state==baseline.state &&
                  plan.reason==baseline.reason && plan.debug.state==baseline.debug.state,
                  "Mask cũ giữ state/reason/debug hiện có khi tắt T57");
            r.mask(60,140); old.mask(60,140);
            for (int i=0;i<r.p.regain_frames+1;++i) {
                ++r.now; old.now=r.now; const auto x=r.step(), y=old.step();
                Check((x.cmd.cms>0)==(y.cmd.cms>0),"Mask tươi giữ luật hồi phục hiện có, không thêm latch T57");
            }
            return;
        }
        if (name=="fault_red") {
            Check(plan.state==control::PlannerState::DUNG_DEN_DO && plan.cmd.cms==0 && plan.cmd.deg==0,
                  "Đỏ đủ phiếu vẫn thắng lỗi cảm biến");
            r.in.sensor_ok=true;
            for (int i=0;i<r.p.vote_window+r.p.green_frames;++i) plan=r.send({"green"});
            Check(plan.state==control::PlannerState::BAM_LAN,"Đủ xanh vẫn mở chốt đỏ sau lỗi cảm biến");
        } else Cancelled(plan,r.p);
        RecoveredSensor(r);
        if (name=="RE_QUA_CM" || name=="RE_QUA_MS") {
            Rig limit; limit.turn(); limit.in.yaw_deg=limit.p.turn_yaw_deg;
            if (name=="RE_QUA_CM") { limit.in.odo_cm+=limit.p.turn_max_cm+0.01; ++limit.now; }
            else limit.now+=limit.p.turn_max_ms+1;
            Cancelled(limit.step(),limit.p);
            RecoveredSensor(limit);
        }
        if (name=="CAM_BIEN_CU") for (bool armed:{false,true}) {
            Rig lane; if (armed) lane.arm();
            lane.now+=201;
            const auto continued=lane.step(true,false);
            Check(continued.state==control::PlannerState::BAM_LAN && continued.cmd.cms>0,
                  "Cảm biến cũ khi bám làn không được dừng");
            RecoveredSensor(lane);
        }
    }
}
}
int main(int argc, char** argv) {
    try {
        Check(argc>=2,"Thiếu tên ca");
        if (std::string(argv[1])=="params" || std::string(argv[1])=="t68_params") {
            Check(argc==3,"Thiếu YAML tạm");
            if (std::string(argv[1])=="params") Params(argv[2]); else T68Params(argv[2]);
        }
        else Case(argv[1]);
        std::cout<<"ĐẠT: "<<argv[1]<<'\n'; return 0;
    } catch (const std::exception& e) { std::cerr<<"KHÔNG ĐẠT: "<<e.what()<<'\n'; return 1; }
}
