#include "control/Planner.h"
#include <algorithm>
#include <cmath>
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
        p.turn_enable=1; p.start_wait_green=0; p.kd=0; p.avoid_min_width_ratio=0.4;
        p.lane1_width_frac={0.8,0.8};  // cả hai fixture (cột 60..140, 20..180) là đường một làn
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
    // Ngã T giả: đường phía trước kết thúc tại khoảng 0,72 chiều cao.
    void junction_mask() {
        mask(60,140);
        const int y=static_cast<int>(0.72*in.mask.rows);
        in.mask.rowRange(0,y).setTo(0);
        in.mask.rowRange(y,static_cast<int>(0.79*in.mask.rows)).setTo(1);
    }
    control::Plan junction() {
        junction_mask();
        control::Plan plan;
        for(int i=0;i<p.vote_min;++i) { now+=40; plan=step(); }
        return plan;
    }
    // Kéo dài lỗi cảm biến đủ 250 ms để kiểm việc hủy rẽ.
    bool was_bad=false;
    void PersistBad(bool fresh_mask, control::Plan &plan) {
        const bool bad=!in.sensor_ok || !std::isfinite(in.yaw_deg) || !std::isfinite(in.odo_cm) ||
                       in.sensor_ms<0 || now-in.sensor_ms>200;  // mốc giờ tương lai chỉ là một mẫu thoáng qua
        const bool first=!was_bad; was_bad=bad;
        if(!bad || !first) return;  // chỉ kéo dài ở mẫu hỏng đầu tiên của đợt
        bool cancelled=false; auto cancel_plan=plan;
        for(int k=0;k<5;++k) {
            now+=50; in.image_ms=now; if(fresh_mask) in.mask_ms=now;
            plan=planner.Step(in,now,p);
            if(!cancelled && plan.reason=="HUY_RE_CAM_BIEN") { cancelled=true; cancel_plan=plan; }
        }
        if(cancelled) plan=cancel_plan;  // ca kiểm lượt hủy cần thấy đúng lệnh của khung hủy
    }
    control::Plan step(bool fresh_mask=true, bool fresh_sensor=true) {
        in.image_ms=now;
        if (fresh_mask) in.mask_ms=now;
        if (fresh_sensor) in.sensor_ms=now;
        auto plan=planner.Step(in,now,p);
        PersistBad(fresh_mask,plan);
        return plan;
    }
    control::Plan send(const std::vector<std::string>& names, float confidence=0.9f) {
        now+=40;
        std::vector<control::Detection> items;
        for (const auto& name:names) {
            const float width=name=="round_barrier" ? 300.f : (name=="work_barrier" ? 130.f : 128.f);
            items.push_back({name,cv::Rect2f(1088-width/2,100,width,72),confidence,now});
        }
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
        arm(left?"left":"right"); auto result=junction();
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
        for (int i=0;i<p.vote_min;++i) plan=send({"obstacle"});
        Check(plan.state==control::PlannerState::BAM_LAN && plan.reason=="NE_TRAI" &&
              plan.mode==control::TrajMode::AVOID_LEFT,"Thiết lập phải đang né trái");
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
        Check(r.advance(r.p.turn_start_cm/2).state==control::PlannerState::BAM_LAN,"Cảm biến chưa khả dụng không được rẽ");
    }
}
void RecoveredSensor(Rig& r, int64_t fresh_ms=-1) {
    r.in.sensor_ok=true; r.in.yaw_deg=0; r.in.odo_cm=0;
    r.mask(60,140); r.in.detections={};
    if (fresh_ms<0) { fresh_ms=++r.now; r.step(); }
    for (int reset=-1;reset<2;++reset) {
        Rig resumed=r;
        if (reset<0) {
            resumed.in.stop=true; ++resumed.now; Check(resumed.step().cmd.cms==0,"Space vẫn phải dừng"); resumed.in.stop=false;
            resumed.in.go=true; ++resumed.now; Check(resumed.step().cmd.cms>0,"Space/G vẫn cho xe chạy"); resumed.in.go=false;
        } else {
            resumed.planner.Reset(reset!=0); resumed.in.go=true; ++resumed.now;
            Check(resumed.step().cmd.cms>0,"Reset/G vẫn cho xe chạy"); resumed.in.go=false;
        }
        TurnUnavailable(resumed);
    }
    TurnUnavailable(r);
    r.in.odo_cm+=r.p.turn_start_cm;
    Check(r.now<fresh_ms+999,"Thiết lập phải còn trong cửa sổ hồi cảm biến");
    while (r.now+40<fresh_ms+999) {
        const auto plan=r.send({"left"});
        Check(plan.state==control::PlannerState::BAM_LAN && plan.cmd.cms>0,
              "Chưa tươi liên tục 1000 ms thì đủ phiếu/quãng vẫn không rẽ");
    }
    r.now=fresh_ms+999;
    Check(r.step().state==control::PlannerState::BAM_LAN,"Tươi 999 ms vẫn chưa được rẽ");
    ++r.now; r.in.detections={};
    Check(r.step().state==control::PlannerState::BAM_LAN,"Đủ 1000 ms không khôi phục lần rẽ/biển cũ");
    for (int i=0;i<r.p.vote_min;++i) {
        Check(r.send({"left"}).state==control::PlannerState::BAM_LAN,"Biển mới phải chờ đủ quãng");
        if (i<r.p.vote_min-1)
            Check(r.advance(r.p.turn_start_cm).state==control::PlannerState::BAM_LAN,"Chưa đủ phiếu mới sau hồi thì không rẽ");
    }
    Check(r.advance(r.p.turn_start_cm-1).state==control::PlannerState::BAM_LAN,"Biển mới chưa đủ quãng không rẽ");
    const auto plan=r.junction();
    Check(plan.state==control::PlannerState::RE_TRAI,"Biển mới đủ phiếu/quãng sau 1000 ms phải rẽ");
    Check(plan.cmd.cms>0 && plan.cmd.cms<=r.p.max_speed_cms && std::abs(plan.cmd.deg)<=r.p.steer_max_deg,
          "Biển mới sau 1000 ms phải rẽ với lệnh trong giới hạn");
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
            const auto plan=r.junction();
            Check(control::StateName(plan.state)==std::string(left?"RE_TRAI":"RE_PHAI") &&
                  plan.reason=="RE_GIAO_LO" && plan.debug.state==control::StateName(plan.state) &&
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
            Check(control::StateName(r.junction().state)==std::string(left?"RE_TRAI":"RE_PHAI"),"Giao lộ đủ phiếu phải rẽ đúng hướng");
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
            Check(control::StateName(r.junction().state)==std::string(left?"RE_TRAI":"RE_PHAI"),"Đỏ10s và biển ngược không đổi hướng chốt");
        }
    } else if (name=="t68_avoidance" || name=="t68_arm_distance") {
        Rig r; r.arm(); r.mask(60,140);
        for(int n=0;n<r.p.vote_window;++n) {
            const auto plan=r.send({"obstacle"});
            Check(plan.state==control::PlannerState::BAM_LAN && plan.mode==control::TrajMode::LANE,
                  "Đã có ý định rẽ không được bắt đầu né");
        }
        if(name=="t68_arm_distance") {
            r.in.odo_cm=r.p.turn_arm_cm+0.01; ++r.now;
            Check(r.step().reason=="HUY_RE","Quá quãng vẫn phải hủy ý định rẽ");
            r.obstacle(false); r.in.detections={};
            Check(r.junction().state==control::PlannerState::BAM_LAN,"Đã hủy không rẽ lại");
        } else Check(r.junction().state==control::PlannerState::RE_TRAI,"Hộp vật không đổi ý định rẽ");
    } else if (name=="q8_sign_blocked_until_end" || name=="q8_speed_min" ||
               name=="q8_space_in_ve_phai" || name=="q8_pass_cm_scale" || name=="q8_lane_lost") {
        auto returning=[](Rig& r) {
            r.p.slow_per_deg=0;
            r.avoid(); r.obstacle(false);
            control::Plan plan;
            for (int i=0;i<r.p.vote_window;++i) plan=r.send({});
            Check(plan.reason=="NE_QUA","Thiết lập Q8 phải qua NE_QUA");
            for (int i=0;i<1000 && plan.reason=="NE_QUA";++i) { r.in.odo_cm+=plan.cmd.cms*0.1; r.now+=100; plan=r.step(); }
            Check(plan.state==control::PlannerState::BAM_LAN && plan.reason=="VE_PHAI" &&
                  plan.mode==control::TrajMode::RETURN_RIGHT && plan.cmd.cms>0,"Thiết lập Q8 phải đang VE_PHAI");
            return plan;
        };
        // Quãng kiểm tra lấy từ lệnh vừa gửi; encoder chỉ phục vụ điều kiện rẽ biển.
        auto tick=[](Rig& r, control::Plan& plan, double& distance, bool sign=false) {
            const double cm=plan.cmd.cms*0.1;
            distance+=cm; r.in.odo_cm+=cm; r.now+=100;
            std::vector<control::Detection> items;
            if (sign) items.push_back({"left",cv::Rect2f(1024,100,128,72),0.9f,r.now});
            r.in.detections={items,r.now,++r.frame};
            plan=r.step();
        };
        if (name=="q8_sign_blocked_until_end") {
            for (double fraction:{0.0,0.5}) {
                Rig r; r.p.obstacle_pass_cm=90; r.p.turn_arm_cm=200;
                auto plan=returning(r); double distance=0; int votes=0;
                for (int i=0;i<1000 && distance+1e-9<r.p.obstacle_pass_cm;++i) {
                    const bool sign=distance>=fraction*r.p.obstacle_pass_cm;
                    tick(r,plan,distance,sign); if (sign) ++votes;
                    if (distance+1e-9<r.p.obstacle_pass_cm)
                        Check(plan.state==control::PlannerState::BAM_LAN && plan.reason=="VE_PHAI" &&
                              plan.mode==control::TrajMode::RETURN_RIGHT,
                              "Biển ở đầu/giữa VE_PHAI phải chờ hết quãng; nhận "+plan.reason+
                              " state="+control::StateName(plan.state)+" quãng="+std::to_string(distance));
                }
                Check(votes>=r.p.vote_min && distance+1e-9>=r.p.obstacle_pass_cm,
                      "Thiết lập biển phải đủ phiếu và hoàn tất quãng VE_PHAI");
                plan=r.junction();
                // Hết VE_PHAI, đủ phiếu giao lộ mới rẽ dù biển còn trong ảnh.
                Check(plan.state==control::PlannerState::RE_TRAI && plan.reason=="RE_GIAO_LO",
                      "Hết VE_PHAI, đủ phiếu giao lộ phải rẽ; nhận "+plan.reason);
            }
            Rig fresh; auto plan=returning(fresh); double distance=0;
            for (int i=0;i<1000 && distance+1e-9<fresh.p.obstacle_pass_cm;++i) tick(fresh,plan,distance);
            Check(distance+1e-9>=fresh.p.obstacle_pass_cm && plan.state==control::PlannerState::BAM_LAN,
                  "Không biển trong VE_PHAI phải trở về bám làn");
            fresh.arm();
            Check(fresh.advance(fresh.p.turn_start_cm-1).state==control::PlannerState::BAM_LAN,
                  "Biển mới sau VE_PHAI chưa đủ quãng không được rẽ");
            Check(fresh.junction().state==control::PlannerState::RE_TRAI,
                  "Biển mới đủ phiếu và quãng sau VE_PHAI phải rẽ bình thường");
        } else if (name=="q8_speed_min") {
            Rig r; r.p.cruise_cms=5; r.p.approach_cms=6;
            auto plan=returning(r); double distance=0;
            for (int i=0;i<1000 && distance+1e-9<r.p.obstacle_pass_cm;++i) {
                Check(plan.state==control::PlannerState::BAM_LAN && plan.reason=="VE_PHAI" &&
                      plan.mode==control::TrajMode::RETURN_RIGHT && plan.cmd.cms==r.p.cruise_cms &&
                      plan.debug.speed_cms==plan.cmd.cms,
                      "VE_PHAI phải giữ cruise_cms=5 khi approach_cms=6, không tăng tốc");
                tick(r,plan,distance);
            }
            Check(distance+1e-9>=r.p.obstacle_pass_cm && plan.reason=="BAM_LAN",
                  "Cruise thấp vẫn phải hoàn tất đủ quãng VE_PHAI; nhận "+plan.reason+
                  " quãng="+std::to_string(distance)+" đích="+std::to_string(r.p.obstacle_pass_cm));
        } else if (name=="q8_space_in_ve_phai") {
            Rig r; auto plan=returning(r); double distance=0;
            for (int i=0;i<1000 && distance<r.p.obstacle_pass_cm/2;++i) tick(r,plan,distance);
            Check(distance>=r.p.obstacle_pass_cm/2 && distance<r.p.obstacle_pass_cm &&
                  plan.reason=="VE_PHAI","Thiết lập Space ở giữa VE_PHAI");
            r.in.stop=true; tick(r,plan,distance); r.in.stop=false;
            Check(plan.state==control::PlannerState::DUNG && plan.reason=="SPACE" &&
                  plan.cmd.cms==0 && plan.cmd.deg==0,"Space giữa VE_PHAI phải dừng motor/servo");
            for (int i=0;i<3;++i) {
                tick(r,plan,distance);
                Check(plan.state==control::PlannerState::DUNG && plan.cmd.cms==0,
                      "Chưa có G mới không được tự chạy tiếp VE_PHAI");
            }
            r.in.go=true; tick(r,plan,distance); r.in.go=false;
            for (int i=0;i<10;++i) {
                Check(plan.state==control::PlannerState::BAM_LAN && plan.reason=="BAM_LAN" &&
                      plan.mode==control::TrajMode::LANE && plan.cmd.cms==r.p.cruise_cms,
                      "G sau Space phải BAM_LAN/RIGHT ở cruise; nhận "+plan.reason+
                      " state="+control::StateName(plan.state)+" cms="+std::to_string(plan.cmd.cms)+
                      " cruise="+std::to_string(r.p.cruise_cms));
                tick(r,plan,distance);
            }
        } else if (name=="q8_pass_cm_scale") {
            double measured[2]={0,0}; int index=0;
            for (double cm:{45.0,90.0}) {
                Rig r; r.p.obstacle_pass_cm=cm;
                auto plan=returning(r); double distance=0, last_cm=0;
                for (int i=0;i<1000 && distance+1e-9<cm;++i) {
                    last_cm=plan.cmd.cms*0.1;
                    tick(r,plan,distance);
                    Check(plan.state==control::PlannerState::BAM_LAN && plan.mode==(distance+1e-9<cm?control::TrajMode::RETURN_RIGHT:control::TrajMode::LANE) &&
                          plan.reason==(distance+1e-9<cm?"VE_PHAI":"BAM_LAN"),
                          "VE_PHAI phải kết thúc đúng obstacle_pass_cm, không dùng quãng cố định");
                }
                Check(distance+1e-9>=cm && distance<cm+last_cm+1e-9,"Sai quãng VE_PHAI theo bước mô phỏng");
                measured[index++]=distance;
                std::cout<<"Q8 obstacle_pass_cm="<<cm<<" VE_PHAI_cm="<<distance<<'\n';
            }
            Check(std::abs(measured[1]-2*measured[0])<1,
                  "obstacle_pass_cm 90 phải cho quãng VE_PHAI gấp đôi 45 trong sai số một bước");
        } else {
            Rig r; auto plan=returning(r);
            control::LaneController expected;
            expected.Step(r.in.mask,r.in.mask_ms,r.now,r.p,control::TrajMode::RETURN_RIGHT);
            const auto last_valid=r.now;
            r.in.mask.setTo(0);
            for (int elapsed:{1,r.p.lost_hold_ms-1,r.p.lost_hold_ms,r.p.lost_stop_ms-1,r.p.lost_stop_ms}) {
                r.now=last_valid+elapsed; plan=r.step();
                const auto lane=expected.Step(r.in.mask,r.in.mask_ms,r.now,r.p,control::TrajMode::RETURN_RIGHT);
                const double speed=lane.cmd.cms<r.p.approach_cms?lane.cmd.cms:r.p.approach_cms;
                Check(plan.state==control::PlannerState::BAM_LAN && plan.mode==control::TrajMode::RETURN_RIGHT &&
                      plan.reason!="MAT_LAN_NE" && plan.debug.reason!="MAT_LAN_NE" && plan.cmd.cms==speed,
                      "Mất làn trong VE_PHAI phải theo LaneController thường, không chốt MAT_LAN_NE");
            }
            Check(plan.cmd.cms==0 && plan.cmd.deg==0,"Đúng lost_stop_ms trong VE_PHAI phải dừng và trả góc 0");
            r.obstacle(false);
            for (int i=0;i<r.p.regain_frames+1;++i) { r.now+=40; plan=r.step(); }
            Check(plan.state==control::PlannerState::BAM_LAN && plan.cmd.cms>0 && plan.reason!="MAT_LAN_NE",
                  "Làn hồi đủ khung phải tự phục hồi theo luật thường, không cần G mở chốt né");
        }
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
    } else if (name=="glitch_hold") {
        // Mẫu cảm biến hỏng ngắn (< 250 ms) giữa lúc rẽ: xe dừng giữ góc, về lại thì rẽ tiếp; hỏng từ 250 ms thì hủy.
        Rig r; r.turn();
        for (int i=0;i<10;++i) { r.now+=40; r.step(); }
        const double angle=r.step().cmd.deg;
        r.in.sensor_ok=false; ++r.now;
        auto plan=r.planner.Step(r.in,r.now,r.p);   // mẫu hỏng đầu tiên (không qua Rig::step để không kéo dài)
        Check(plan.state==control::PlannerState::RE_TRAI && plan.reason=="CAM_BIEN_CHO" && plan.cmd.cms==0 &&
              std::abs(plan.cmd.deg-angle)<1e-9,"Mẫu hỏng thoáng qua: dừng giữ góc");
        r.now+=100; r.in.image_ms=r.in.mask_ms=r.in.sensor_ms=r.now;
        plan=r.planner.Step(r.in,r.now,r.p);
        Check(plan.reason=="CAM_BIEN_CHO","Hỏng 100 ms vẫn giữ");
        r.in.sensor_ok=true; r.now+=50; r.in.image_ms=r.in.mask_ms=r.in.sensor_ms=r.now;
        plan=r.planner.Step(r.in,r.now,r.p);
        Check(plan.state==control::PlannerState::RE_TRAI && plan.cmd.cms==TurnCms(r.p),"Cảm biến về trong 250 ms: rẽ tiếp");
        r.in.sensor_ok=false;
        for (int i=0;i<6;++i) { r.now+=50; r.in.image_ms=r.in.mask_ms=r.in.sensor_ms=r.now; plan=r.planner.Step(r.in,r.now,r.p); }
        Check(plan.state==control::PlannerState::BAM_LAN,"Hỏng quá 250 ms phải hủy rẽ");
    } else if (name=="sign_nan_odo") {
        // Mẫu hỏng (odo NaN) đúng lúc đủ phiếu biển: không chốt hướng với mốc NaN; sau khi hồi vẫn rẽ được.
        Rig r; r.in.odo_cm=std::numeric_limits<double>::quiet_NaN(); r.in.sensor_ok=false;
        for (int i=0;i<r.p.vote_min;++i) { r.now+=40; r.in.detections={{{"left",cv::Rect2f(600,100,80,80),0.9f,r.now}},r.now,++r.frame}; r.planner.Step((r.in.image_ms=r.in.mask_ms=r.in.sensor_ms=r.now,r.in),r.now,r.p); }
        r.in.odo_cm=100; r.in.sensor_ok=true;
        r.advance(0);   // không còn lô biển mới để dời mốc: phiếu cũ phải chốt hướng với mốc quãng hợp lệ
        r.advance(r.p.turn_start_cm);
        Check(control::StateName(r.junction().state)==std::string("RE_TRAI"),"Sau khi cảm biến hồi biển vẫn được rẽ (không kẹt mốc NaN)");
    } else if (name=="barrier_sensor_hold") {
        // Rào gần mà cảm biến vừa hỏng (chưa tới 250 ms): không tiến thẳng vào rào với tốc độ bám làn.
        Rig r; r.in.sensor_ok=false; control::Plan plan;
        for (int i=0;i<4;++i) { r.now+=40; r.in.detections={{{"work_barrier",cv::Rect2f(575,400,130,80),0.9f,r.now}},r.now,++r.frame}; r.in.image_ms=r.in.mask_ms=r.in.sensor_ms=r.now; plan=r.planner.Step(r.in,r.now,r.p); }
        Check(plan.cmd.cms==0 && plan.reason=="DUNG_RAO_LOI_CAM_BIEN","Rào gần + cảm biến hỏng thoáng qua: dừng ngay");
    } else if (name=="sign_min_width") {
        // Biển hẹp (xa/chặng khác) không tính phiếu; đủ rộng thì tính.
        for (float width:{30.0f,44.0f,45.0f,80.0f}) {
            Rig r; control::Plan plan;
            for (int i=0;i<r.p.vote_window;++i) {
                r.now+=40; r.in.detections={{{"left",cv::Rect2f(600,100,width,width),0.99f,r.now}},r.now,++r.frame};
                plan=r.step();
            }
            r.advance(r.p.turn_start_cm);
            const bool turned=control::StateName(r.junction().state)==std::string("RE_TRAI");
            Check(turned==(width>=45.0f),"Chỉ biển rộng >= 45 px được tính, rộng "+std::to_string(width));
        }
    } else if (name=="sign_exit_anchor") {
        // Biển còn trong ảnh chỉ giữ ý định; phải chờ giao lộ.
        Rig r; r.arm("left");
        for(int cm=1;cm<=r.p.turn_start_cm;++cm) {
            r.in.odo_cm=cm;
            const auto plan=r.send({"left"});
            Check(plan.state==control::PlannerState::BAM_LAN,"Biển còn thấy, chưa giao lộ");
        }
        Check(r.junction().reason=="RE_GIAO_LO","Gặp giao lộ mới rẽ");
    } else if (name=="turn_steer_by_type") {
        // Rẽ phải lái nhiều hơn (bán kính đường xe ~65 cm), rẽ trái do rào lái ít hơn (vào đường 2 làn ~95 cm).
        Rig left; left.turn(true); Rig right; right.turn(false);
        control::Plan pl, pr;
        for (int i=0;i<10;++i) { left.now+=40; pl=left.step(); right.now+=40; pr=right.step(); }
        Check(std::abs(pl.cmd.deg-left.p.turn_steer_deg)<1e-9,"Rẽ trái theo biển giữ turn_steer_deg");
        Check(std::abs(pr.cmd.deg+std::min(right.p.steer_max_deg,1.12*right.p.turn_steer_deg))<1e-9,"Rẽ phải lái 1,12 lần");
    } else if (name=="pre_turn_steer_scale") {
        // Lỗi 2 (Quân chọn C 11/11): bám làn đã quay sẵn về phía rẽ trong 60 cm trước RE_* thì giảm góc rẽ cố định
        // theo hệ số 1 - quay_sẵn/90°, kẹp [0,75; 1]. Quay ngược chiều hoặc quay trước 60 cm không tính.
        auto run=[](bool left, const std::vector<std::pair<double,double>>& path) {
            Rig r; r.arm(left?"left":"right");
            for (const auto& s:path) { r.in.odo_cm=s.first; r.in.yaw_deg=s.second; r.now+=40; r.step(); }
            const auto start=r.junction();
            Check(control::StateName(start.state)==std::string(left?"RE_TRAI":"RE_PHAI"),"Thiết lập phải đang rẽ");
            control::Plan plan;
            for (int i=0;i<10;++i) { r.now+=40; plan=r.step(); }
            return std::make_pair(plan.cmd.deg,r.p);
        };
        const auto full=[](const control::ControlParams& p,bool left) {
            return left ? std::min(p.steer_max_deg,p.turn_steer_deg) : -std::min(p.steer_max_deg,1.12*p.turn_steer_deg);
        };
        // Phải: quay -20° trong 60 cm cuối -> 1-20/90.
        auto a=run(false,{{0,0},{20,-5},{40,-12},{60,-20}});
        Check(std::abs(a.first-full(a.second,false)*(1-20.0/90))<1e-6,"Quay sẵn 20° phải giảm góc rẽ phải");
        // Quay sẵn rất nhiều: kẹp 0,75.
        auto b=run(false,{{0,0},{30,-25},{60,-45}});
        Check(std::abs(b.first-full(b.second,false)*0.75)<1e-6,"Hệ số góc rẽ phải kẹp 0,75");
        // Không quay sẵn: giữ nguyên.
        auto c=run(false,{{0,0},{30,0},{60,0}});
        Check(std::abs(c.first-full(c.second,false))<1e-6,"Không quay sẵn giữ góc rẽ");
        // Quay ngược chiều rẽ: không tăng góc.
        auto d=run(true,{{0,0},{30,-5},{60,-10}});
        Check(std::abs(d.first-full(d.second,true))<1e-6,"Quay ngược chiều không đổi góc rẽ");
        // Rẽ trái đã quay sẵn về trái: chỉ rẽ phải được giảm góc.
        auto f=run(true,{{0,0},{30,10},{60,20}});
        Check(std::abs(f.first-full(f.second,true))<1e-6,"Rẽ trái không được giảm góc");
        // Phần quay xảy ra trước cửa sổ 60 cm không tính: quay -30° ở 0-10 cm rồi thẳng 100 cm.
        auto e=run(false,{{0,0},{10,-30},{40,-30},{80,-30},{110,-30}});
        Check(std::abs(e.first-full(e.second,false))<1e-6,"Quay trước 60 cm cuối không được tính");
    } else if (name=="l_turn_after_bridge") {
        // Cua L phải hai làn sau cầu trước VX1 (Quân duyệt 11/11): đảo nhỏ ở mép phải ảnh rồi các hàng gần phủ kín ảnh
        // -> rẽ phải cung cố định (RE_L_CAU), một lần mỗi G; thiếu điều kiện nào cũng chỉ bám làn.
        auto island=[](Rig& r, double cx, float width) {
            r.now+=40;
            r.in.detections={{{"round_barrier",cv::Rect2f(float(cx*1280-width/2),300,width,60),0.9f,r.now}},r.now,++r.frame};
            return r.step();
        };
        auto empty=[](Rig& r) { r.now+=40; r.in.detections={{},r.now,++r.frame}; return r.step(); };
        auto run=[&](double cx, double gap_cm, bool& turned, std::string& reason, double& deg) {
            Rig r; r.in.yaw_deg=0; r.in.odo_cm=0;
            for (int i=0;i<r.p.vote_min;++i) { r.in.odo_cm+=2; island(r,cx,130); }
            r.in.odo_cm+=gap_cm;
            r.mask(0,200);  // góc cua rộng: mọi hàng chạm hai mép ảnh
            control::Plan plan;
            turned=false;
            for (int i=0;i<r.p.vote_min+1 && !turned;++i) {
                r.in.odo_cm+=1; plan=empty(r);
                turned=plan.state==control::PlannerState::RE_PHAI;
            }
            reason=plan.reason; deg=plan.cmd.deg;
            return r.p;
        };
        bool turned; std::string reason; double deg;
        auto p=run(0.90,60,turned,reason,deg);
        Check(turned && reason=="RE_L_CAU","Đảo bên phải rồi hàng gần phủ kín ảnh phải rẽ L phải");
        Check(deg<0 && std::abs(deg)<=p.steer_max_deg,"Rẽ L phải lái phải trong giới hạn");
        run(0.50,60,turned,reason,deg);
        Check(!turned,"Đảo giữa ảnh không được kích hoạt cua L");
        run(0.90,200,turned,reason,deg);
        Check(!turned,"Quá 150 cm sau khi thấy đảo bên phải không được rẽ");
        { // Không thấy đảo: góc rộng phủ kín ảnh chỉ bám làn.
            Rig r; r.mask(0,200); control::Plan plan;
            for (int i=0;i<3*r.p.vote_min;++i) { r.in.odo_cm+=1; plan=empty(r); }
            Check(plan.state==control::PlannerState::BAM_LAN,"Không thấy đảo bên phải thì không rẽ L");
        }
        { // Một lần mỗi G: rẽ xong, gặp lại điều kiện thì không rẽ nữa.
            Rig r; r.in.yaw_deg=0;
            for (int i=0;i<r.p.vote_min;++i) { r.in.odo_cm+=2; island(r,0.90,130); }
            r.mask(0,200); control::Plan plan;
            for (int i=0;i<r.p.vote_min;++i) { r.in.odo_cm+=1; plan=empty(r); }
            Check(plan.state==control::PlannerState::RE_PHAI,"Lượt đầu phải rẽ");
            r.in.yaw_deg=-r.p.turn_yaw_deg; r.in.odo_cm+=1; plan=empty(r);
            Check(plan.state!=control::PlannerState::RE_PHAI,"Đủ yaw phải xong rẽ");
            r.in.yaw_deg-=1; r.mask(60,140); empty(r);
            for (int i=0;i<r.p.vote_min;++i) { r.in.odo_cm+=2; island(r,0.90,130); }
            r.mask(0,200); bool again=false;
            for (int i=0;i<2*r.p.vote_min;++i) { r.in.odo_cm+=1; again=again || empty(r).state==control::PlannerState::RE_PHAI; }
            Check(!again,"Cua L chỉ rẽ một lần mỗi G");
        }
        { // Review 11/11 M1: đang chờ cua L mà cảm biến lỗi >= 250 ms rồi hồi < 1 s thì không được rẽ L.
            Rig r; r.in.yaw_deg=0;
            for (int i=0;i<r.p.vote_min;++i) { r.in.odo_cm+=2; island(r,0.90,130); }
            r.in.sensor_ok=false;
            for (int i=0;i<8;++i) { r.now+=50; r.step(); }
            r.in.sensor_ok=true; r.mask(0,200); bool any=false;
            for (int i=0;i<2*r.p.vote_min;++i) { r.in.odo_cm+=1; any=any || empty(r).state==control::PlannerState::RE_PHAI; }
            Check(!any,"Lỗi cảm biến khi chờ cua L phải bỏ cua L");
        }
    } else if (name=="park_reverse") {
        // Đỗ xe lùi song song (Quân duyệt viết với số tạm 11/11): biển parking -> tiến park_forward_cm -> chờ -> lùi 2 cung -> đứng.
        auto sign=[](Rig& r) {
            r.now+=40; r.in.detections={{{"parking",cv::Rect2f(576,200,128,128),0.9f,r.now}},r.now,++r.frame};
            return r.step();
        };
        auto tick=[](Rig& r, int ms=40) { r.now+=ms; r.in.detections={{},r.now,++r.frame}; return r.step(); };
        // Đưa tới đầu pha lùi 1; trả về Rig đang ở DO_XE_LUI_1.
        auto to_reverse=[&](Rig& r, int side) {
            r.p.park_enable=1; r.p.park_side=side; r.in.yaw_deg=0; r.in.odo_cm=0;
            for (int i=0;i<r.p.vote_min;++i) { r.in.odo_cm+=1; sign(r); }
            control::Plan plan;
            for (int i=0;i<200 && plan.reason!="DO_XE_CHO";++i) { r.in.odo_cm+=1; plan=tick(r); }
            Check(plan.state==control::PlannerState::DO_XE && plan.reason=="DO_XE_CHO" && plan.cmd.cms==0,"Đủ park_forward_cm phải dừng chờ lùi");
            Check(r.in.odo_cm>=r.p.park_forward_cm,"Không được dừng trước park_forward_cm");
            for (int i=0;i<20 && plan.reason=="DO_XE_CHO";++i) plan=tick(r,50);
            Check(plan.reason=="DO_XE_LUI_1","Chờ xong phải lùi cung 1");
            return plan;
        };
        { // Ô bên phải: lùi, lái phải (âm) tới khi yaw tăng park_arc_deg, rồi lái trái tới khi về hướng đầu.
            Rig r; auto plan=to_reverse(r,-1);
            for (int i=0;i<10;++i) { r.in.odo_cm-=0.3; plan=tick(r); }
            Check(plan.cmd.cms<0 && plan.cmd.cms>=-r.p.park_rev_cms && plan.cmd.deg<0 &&
                  plan.cmd.deg>=-std::min(r.p.park_steer_deg,r.p.steer_max_deg),"Lùi cung 1 ô phải: tốc độ âm, lái phải");
            r.in.yaw_deg=r.p.park_arc_deg; r.in.odo_cm-=0.3; plan=tick(r);
            Check(plan.reason=="DO_XE_LUI_2","Quay đủ park_arc_deg phải sang cung 2");
            for (int i=0;i<20;++i) { r.in.odo_cm-=0.3; plan=tick(r); }
            Check(plan.cmd.cms<0 && plan.cmd.deg>0,"Lùi cung 2 ô phải: lái trái");
            r.in.yaw_deg=2; r.in.odo_cm-=0.3; plan=tick(r);
            Check(plan.state==control::PlannerState::DO_XE_XONG && plan.cmd.cms==0,"Về hướng đầu phải đứng yên");
            for (int i=0;i<20;++i) Check(tick(r).cmd.cms==0,"Đã đỗ phải đứng yên");
        }
        { // Ô bên trái: lái trái ở cung 1, quay theo chiều yaw giảm.
            Rig r; auto plan=to_reverse(r,1);
            for (int i=0;i<10;++i) { r.in.odo_cm-=0.3; plan=tick(r); }
            Check(plan.cmd.cms<0 && plan.cmd.deg>0,"Lùi cung 1 ô trái: lái trái");
            r.in.yaw_deg=-r.p.park_arc_deg; r.in.odo_cm-=0.3;
            Check(tick(r).reason=="DO_XE_LUI_2","Ô trái quay đủ phải sang cung 2");
        }
        { // Cảm biến lỗi khi lùi: dừng.
            Rig r; to_reverse(r,-1); r.in.odo_cm-=0.3; tick(r);
            r.in.sensor_ok=false; auto plan=tick(r);
            Check(plan.state==control::PlannerState::DUNG && plan.cmd.cms==0,"Cảm biến lỗi khi lùi phải dừng");
        }
        { // Không nhích trong 2 s: dừng DO_XE_KET.
            Rig r; to_reverse(r,-1); control::Plan plan;
            for (int i=0;i<60 && plan.state!=control::PlannerState::DUNG;++i) plan=tick(r,50);
            Check(plan.state==control::PlannerState::DUNG && plan.reason=="DO_XE_KET" && plan.cmd.cms==0,"Kẹt khi lùi phải dừng");
        }
        { // Lùi quá 80 cm mà chưa quay đủ: dừng DO_XE_HET_HAN.
            Rig r; to_reverse(r,-1); control::Plan plan;
            for (int i=0;i<200 && plan.state!=control::PlannerState::DUNG;++i) { r.in.odo_cm-=1; plan=tick(r); }
            Check(plan.reason=="DO_XE_HET_HAN" && plan.cmd.cms==0,"Lùi quá quãng tối đa phải dừng");
        }
        { // Space khi lùi: dừng.
            Rig r; to_reverse(r,-1); r.in.odo_cm-=0.3; tick(r);
            r.in.stop=true; auto plan=tick(r); r.in.stop=false;
            Check(plan.state==control::PlannerState::DUNG && plan.cmd.cms==0 && plan.cmd.deg==0,"Space khi lùi phải dừng");
        }
        { // park_enable 0: biển parking không làm gì.
            Rig r; r.in.odo_cm=0;
            for (int i=0;i<r.p.vote_min;++i) { r.in.odo_cm+=1; sign(r); }
            control::Plan plan;
            for (int i=0;i<100;++i) { r.in.odo_cm+=1; plan=tick(r); Check(plan.cmd.cms>=0,"Tắt đỗ không được lùi"); }
            Check(plan.state==control::PlannerState::BAM_LAN,"Tắt đỗ phải bám làn");
        }
    } else if (name=="obstacle_blocked_in_roundabout") {
        // Lòng vòng xuyến là hố trong mặt nạ: đủ phiếu round_barrier thì không né, ngoài vòng xuyến thì né.
        Rig r; r.p.obstacle_action=2;
        for (int i=0;i<r.p.vote_min;++i) r.send({"round_barrier"});
        r.obstacle(true);
        for (int i=0;i<r.p.vote_window+3;++i) {
            const auto plan=r.send({"round_barrier","obstacle"});
            Check(plan.reason!="NE_TRAI" && plan.mode==control::TrajMode::ROUNDABOUT,"Đang qua vòng xuyến không được né");
        }
        Rig outside; outside.avoid();
    } else if (name=="obstacle_stop_no_turn") {
        // Đang DUNG_VAT_CAN (obstacle_action 1), đủ quãng cũng không được bắt đầu rẽ.
        Rig r; r.p.obstacle_action=1;
        r.obstacle(true);
        control::Plan plan;
        for (int i=0;i<r.p.vote_window;++i) plan=r.send({"obstacle"});
        Check(plan.state==control::PlannerState::DUNG_VAT_CAN && plan.cmd.cms==0,"Thiết lập phải DUNG_VAT_CAN");
        r.in.odo_cm+=r.p.turn_start_cm;
        for (int i=0;i<4;++i) {
            r.obstacle(true);
            plan=r.send({"obstacle","left"});
            Check(plan.state==control::PlannerState::DUNG_VAT_CAN && plan.cmd.cms==0 && plan.cmd.deg==0,
                  "Đang dừng vật cản không được rẽ");
        }
    } else if (name=="barrier_turn") {
        // Rào gần đủ phiếu → RE_TRAI bằng máy rẽ IMU; yaw đủ → BAM_LAN; rào còn trong ảnh không rẽ lại;
        // rào xa hoặc lệch ngang không kích hoạt; work_barrier_action 0 không kích hoạt.
        auto box=[&](Rig& r,float cx,float y2) {
            r.now+=40;
            r.in.detections={{{"work_barrier",cv::Rect2f(cx*1280-65,y2*720-60,130,60),0.9f,r.now}},r.now,++r.frame};
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
          Check(plan.state==control::PlannerState::BAM_LAN,"Rao chot y dinh, chua giao lo"); plan=r.junction();
          Check(plan.state==control::PlannerState::RE_TRAI && plan.cmd.cms==TurnCms(r.p) && plan.cmd.deg>0,"Rào gần đủ phiếu phải rẽ trái");
          plan=box(r,0.83f,0.80f); Check(plan.state==control::PlannerState::RE_TRAI,"Hộp đổi bên khi đang rẽ không đổi quyết định");
          r.in.yaw_deg=5+r.p.turn_yaw_deg; plan=box(r,0.5f,0.80f);
          Check(plan.state==control::PlannerState::BAM_LAN,"Đủ yaw về bám làn");
          for (int i=0;i<6;++i) { plan=box(r,0.5f,0.80f); Check(plan.state==control::PlannerState::BAM_LAN,"Rào còn trong ảnh không rẽ lại"); } }
        { Rig r; r.in.sensor_ok=false; control::Plan plan; for (int i=0;i<5;++i) plan=box(r,0.5f,0.80f);
          Check(plan.state==control::PlannerState::BAM_LAN && plan.cmd.cms==0 && plan.reason=="DUNG_RAO_LOI_CAM_BIEN",
                "IMU lỗi + rào gần: dừng trước rào, không rẽ");
          for (int i=0;i<5;++i) plan=box(r,0.5f,0.58f);
          Check(plan.cmd.cms>0 && plan.reason!="DUNG_RAO_LOI_CAM_BIEN","IMU lỗi + rào còn xa: chưa dừng");
          r.in.stop=true; plan=box(r,0.5f,0.80f); r.in.stop=false;
          r.in.go=true; plan=box(r,0.5f,0.80f); r.in.go=false;
          for (int i=0;i<5;++i) plan=box(r,0.5f,0.80f);
          Check(plan.cmd.cms>0 && plan.state==control::PlannerState::BAM_LAN,"Space rồi G: đi tiếp qua rào đang thấy, không rẽ"); }
        { Rig r; r.in.yaw_deg=5; control::Plan plan;
          r.now+=40; r.in.sensor_ms=r.now-300; r.step(true,false);  // lỗi tạm
          for (int i=0;i<r.p.vote_min;++i) plan=box(r,0.5f,0.80f);
          Check(plan.cmd.cms==0 && plan.reason=="DUNG_RAO_LOI_CAM_BIEN","Lỗi tạm + rào gần: dừng");
          r.junction_mask();
          for (int i=0;i<30 && plan.state!=control::PlannerState::RE_TRAI;++i) plan=box(r,0.5f,0.80f);
          Check(plan.state==control::PlannerState::RE_TRAI,"Hồi lỗi tạm (1 s tươi): rẽ trái qua rào"); }
        { Rig r; r.in.yaw_deg=5; control::Plan plan;  // lỗi giữa lúc rẽ qua rào, rào còn gần
          for (int i=0;i<r.p.vote_min;++i) plan=box(r,0.5f,0.80f);
          Check(plan.state==control::PlannerState::BAM_LAN,"Rao chot y dinh, chua giao lo"); plan=r.junction();
          Check(plan.state==control::PlannerState::RE_TRAI,"Rào gần: đang rẽ trái");
          r.now+=40; r.in.sensor_ms=r.now-300; plan=r.step(true,false);
          plan=box(r,0.5f,0.80f);
          Check(plan.state==control::PlannerState::BAM_LAN && plan.cmd.cms==0 && plan.reason=="DUNG_RAO_LOI_CAM_BIEN",
                "Rẽ qua rào bị hủy vì lỗi, rào còn gần: dừng, không đi thẳng vào rào");
          r.junction_mask();
          for (int i=0;i<30 && plan.state!=control::PlannerState::RE_TRAI;++i) plan=box(r,0.5f,0.80f);
          Check(plan.state==control::PlannerState::RE_TRAI,"Hồi lỗi: rẽ trái lại qua rào"); }
        { Rig r; r.in.yaw_deg=5; control::Plan plan;  // mặt nạ cũ giữa lúc rẽ rào: không bỏ qua rào còn gần
          for (int i=0;i<r.p.vote_min;++i) plan=box(r,0.5f,0.80f);
          Check(plan.state==control::PlannerState::BAM_LAN,"Rao chot y dinh, chua giao lo"); plan=r.junction();
          Check(plan.state==control::PlannerState::RE_TRAI,"Rào gần: đang rẽ trái (mặt nạ)");
          r.now+=r.p.mask_max_age_ms+40; plan=r.step(false,true);  // mặt nạ quá tuổi, cảm biến tươi
          for (int i=0;i<r.p.vote_window+r.p.vote_min && plan.state!=control::PlannerState::RE_TRAI;++i) plan=box(r,0.5f,0.80f);
          Check(plan.state==control::PlannerState::RE_TRAI,"Mặt nạ tươi lại, rào còn gần: rẽ lại, không bám làn vào rào"); }
    } else if (name=="roundabout") {
        // Đếm lần qua bằng phiếu round_barrier; sau lần 1 đi rb_turn1_cm rồi RE_TRAI; sau lần 2 RE_PHAI;
        // trong lần qua mất làn thì GIU_HUONG lái về yaw lúc vào.
        Rig r; r.in.yaw_deg=10; r.in.odo_cm=0;
        auto rb=[&](bool seen) {
            r.now+=40;
            std::vector<control::Detection> items;
            if (seen) items.push_back({"round_barrier",cv::Rect2f(490,300,300,80),0.9f,r.now});
            r.in.detections={items,r.now,++r.frame};
            return r.step();
        };
        control::Plan plan;
        for (int i=0;i<r.p.vote_min;++i) plan=rb(true);
        Check(plan.state==control::PlannerState::BAM_LAN,"Thấy vòng xuyến vẫn bám làn");
        r.in.mask=cv::Mat(60,200,CV_8UC1,cv::Scalar(0)); r.in.yaw_deg=25;
        for (int i=0;i<12;++i) plan=rb(true);
        Check(plan.state==control::PlannerState::GIU_HUONG && plan.cmd.deg<0 && plan.cmd.cms>0,"Mất làn trong lần qua phải giữ hướng (lái phải về yaw0)");
        r.now+=40; r.in.detections={{{"round_barrier",cv::Rect2f(490,300,300,80),0.9f,r.now},{"work_barrier",cv::Rect2f(575,500,130,60),0.9f,r.now}},r.now,++r.frame};
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
        r.junction_mask();
        for (int i=0;i<90 && plan.state==control::PlannerState::BAM_LAN;++i) { r.in.odo_cm+=1; plan=rb(false); }
        Check(plan.state==control::PlannerState::RE_TRAI,"Sau lần 1 phải rẽ trái");
        r.in.yaw_deg=10+r.p.turn_yaw_deg; plan=rb(false); r.mask(60,140); Check(plan.state==control::PlannerState::BAM_LAN,"Xong rẽ");
        for (int i=0;i<r.p.vote_min;++i) plan=rb(true);
        for (int i=0;i<35;++i) { r.in.odo_cm+=1; plan=rb(false); }
        r.junction_mask();
        for (int i=0;i<120 && plan.state==control::PlannerState::BAM_LAN;++i) { r.in.odo_cm+=1; plan=rb(false); }
        Check(plan.state==control::PlannerState::RE_PHAI,"Sau lần 2 phải rẽ phải");
        r.in.yaw_deg-=r.p.turn_yaw_deg; plan=rb(false);
        for (int i=0;i<r.p.vote_min;++i) plan=rb(true);
        for (int i=0;i<200;++i) { r.in.odo_cm+=1; plan=rb(false);
            Check(plan.state==control::PlannerState::BAM_LAN,"Sau lần 2 không rẽ nữa"); }
    } else if (name=="roundabout_after_space") {
        // Lỗi 3 (log 18:01 10/11, 289 s): Space rồi G giữa đảo mở lần qua mới với yaw0 = hướng giữa đảo;
        // mất làn ở lối ra thì GIU_HUONG lái về hướng đó (sang trái, lệch lối ra phải). Lần qua mở ngay sau
        // G-sau-Space không được giữ hướng theo yaw0 đó; lần qua bình thường (không Space) vẫn giữ hướng.
        auto run=[](bool after_space) {
            Rig r; r.in.yaw_deg=10; r.in.odo_cm=0;
            if (after_space) {
                r.in.stop=true; r.now+=40; r.step(); r.in.stop=false;
                r.in.go=true; r.now+=40; r.step(); r.in.go=false;
            }
            auto rb=[&]() {
                r.now+=40; r.in.odo_cm+=0.5;
                r.in.detections={{{"round_barrier",cv::Rect2f(490,300,300,80),0.9f,r.now}},r.now,++r.frame};
                return r.step();
            };
            control::Plan plan;
            for (int i=0;i<r.p.vote_min;++i) plan=rb();
            r.in.mask=cv::Mat(60,200,CV_8UC1,cv::Scalar(0)); r.in.yaw_deg=-10;
            bool held=false;
            for (int i=0;i<10;++i) { plan=rb(); held=held || plan.state==control::PlannerState::GIU_HUONG; }
            return held;
        };
        Check(run(false),"Lần qua bình thường mất làn vẫn phải giữ hướng (không đổi hành vi cũ)");
        Check(!run(true),"Lần qua mở ngay sau G-sau-Space không được giữ hướng theo yaw giữa đảo");
    } else if (name=="roundabout_stall") {
        // Giữ hướng mà bánh không tiến (kẹt đảo/rào) thì hết GIU_HUONG sau 1500 ms, về dừng mất làn như cũ.
        Rig r; r.in.yaw_deg=10; r.in.odo_cm=0;
        auto rb=[&]() {
            r.now+=40;
            r.in.detections={{{"round_barrier",cv::Rect2f(490,300,300,80),0.9f,r.now}},r.now,++r.frame};
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
        r.in.odo_cm+=r.p.turn_start_cm+1; plan=r.junction();
        Check(plan.state==control::PlannerState::RE_TRAI,"Cảm biến tươi lại 1 s thì phải rẽ được");
    } else if (name=="roundabout_fault_drops_pending") {
        // Lỗi cảm biến sau lần qua 1 bỏ luôn lượt rẽ vòng xuyến đang chờ; hồi lỗi rồi không rẽ muộn sai chỗ.
        Rig r; r.in.yaw_deg=10; r.in.odo_cm=0;
        auto rb=[&](bool seen) {
            r.now+=40;
            std::vector<control::Detection> items;
            if (seen) items.push_back({"round_barrier",cv::Rect2f(490,300,300,80),0.9f,r.now});
            r.in.detections={items,r.now,++r.frame};
            return r.step();
        };
        control::Plan plan;
        for (int i=0;i<r.p.vote_min;++i) plan=rb(true);
        for (int i=0;i<40;++i) { r.in.odo_cm+=1; plan=rb(false); }  // hết lần qua 1, chờ rb_turn1_cm
        r.now+=40; r.in.sensor_ms=r.now-300; r.step(true,false);      // một mẫu cũ
        for (int i=0;i<30;++i) plan=rb(false);                        // 1,2 s tươi, đứng yên
        for (int i=0;i<150;++i) { r.in.odo_cm+=1; plan=rb(false);
            Check(plan.state==control::PlannerState::BAM_LAN,"Không rẽ vòng xuyến muộn sau khi hồi lỗi"); }
    } else if (name=="low_conf_armed") {
        Rig r;
        for (int i=0;i<r.p.vote_min;++i) r.send({"left"},0.01f);
        r.advance(r.p.turn_start_cm/2); Check(control::StateName(r.junction().state)==std::string("RE_TRAI"),"Conf thấp đủ phiếu vẫn armed");
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
            TurnUnavailable(startup);
            startup.in.stop=true; ++startup.now; Check(startup.step().cmd.cms==0,"Space vẫn dừng khi IMU lỗi khởi động"); startup.in.stop=false;
            startup.in.go=true; ++startup.now; Check(startup.step().cmd.cms>0,"Space/G vẫn chạy khi IMU lỗi khởi động"); startup.in.go=false;
            TurnUnavailable(startup);
            for (bool preserve:{false,true}) {
                startup.planner.Reset(preserve); startup.in.go=true; ++startup.now;
                Check(startup.step().cmd.cms>0,"Reset/G vẫn chạy khi IMU lỗi khởi động"); startup.in.go=false;
                TurnUnavailable(startup);
            }
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
                if (phase==2 && fault==7) {  // một mẫu mốc giờ tương lai: thoáng qua, xe dừng giữ góc, chưa hủy
                    Check(plan.state==control::PlannerState::RE_TRAI && plan.reason=="CAM_BIEN_CHO" && plan.cmd.cms==0,
                          "Một mẫu cảm biến thoáng qua giữa lúc rẽ: dừng giữ góc, chưa hủy");
                    continue;
                }
                if (phase==2) Cancelled(plan,r.p);
                else Check(plan.state==control::PlannerState::BAM_LAN && plan.cmd.cms>0,
                           "Lỗi cảm biến khi chưa rẽ vẫn bám làn");
                if (fault==7) continue;  // mẫu thoáng qua: không có lỗi để hồi
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
            kept.advance(kept.p.turn_start_cm/2); Check(control::StateName(kept.junction().state)==std::string("RE_TRAI"),"Nhả đỏ còn armed khi trong quãng");
            for (bool release:{false,true}) {
                Rig fault; fault.p.vote_window=fault.p.vote_min; fault.arm();
                int64_t fresh_ms=-1;
                for (int i=0;i<fault.p.vote_min;++i) plan=fault.send({"red"});
                Check(plan.state==control::PlannerState::DUNG_DEN_DO,"Thiết lập phải giữ đỏ khi armed");
                fault.in.odo_cm+=fault.p.turn_start_cm;
                if (!release) {
                    fault.in.sensor_ok=false; ++fault.now; plan=fault.step();
                    Check(plan.state==control::PlannerState::DUNG_DEN_DO && plan.cmd.cms==0 && plan.cmd.deg==0,
                          "Lỗi cảm biến khi giữ đỏ vẫn phải dừng đèn");
                    fault.in.sensor_ok=true; fresh_ms=fault.now+40;
                }
                for (int i=0;i<fault.p.green_frames;++i) {
                    if (release && i==fault.p.green_frames-1) fault.in.sensor_ok=false;
                    plan=fault.send({"green"});
                }
                Check(plan.state==control::PlannerState::BAM_LAN && plan.cmd.cms>0,
                      "Lỗi khi giữ hoặc nhả đỏ phải về bám làn, không bắt đầu rẽ");
                RecoveredSensor(fault,fresh_ms);
            }
        }
    } else if (name=="sign_while_turning") {
        Rig r; r.in.odo_cm=100; r.arm();
        r.in.odo_cm=110;
        for (int i=0;i<r.p.vote_window;++i) r.send({"right","no_left"});
        Check(control::StateName(r.junction().state)==std::string("RE_TRAI"),"Armed bỏ biển ngược");
        for (int i=0;i<r.p.vote_window;++i) Check(control::StateName(r.send({"right"}).state)==std::string("RE_TRAI"),"Đang rẽ không đổi hướng");
        r.in.yaw_deg=r.p.turn_yaw_deg; ++r.now; Check(r.step().state==control::PlannerState::BAM_LAN,"Biển mới không đổi yaw0");
        Rig distance; distance.in.odo_cm=100; distance.arm(); distance.in.odo_cm=120;
        for (int i=0;i<distance.p.vote_window;++i) distance.send({"left"});
        distance.in.detections={{},distance.now,++distance.frame};
        distance.in.odo_cm=120+distance.p.turn_arm_cm+0.01; ++distance.now;
        // Biển cùng chiều cập nhật mốc; chỉ hủy khi đi quá hạn sau lần cuối thấy biển.
        Check(distance.step().reason=="HUY_RE","Quá quãng từ lần cuối thấy biển phải hủy");
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
        Check(r.junction().reason=="RE_GIAO_LO","Giao lộ đủ phiếu, không chờ quãng");
    } else if (name=="arm_distance") {
        for (double sign:{-1.0,1.0}) {
            Rig r; r.in.odo_cm=100; r.arm();
            r.send({}); r.mask(60,140);
            for(int i=0;i<r.p.vote_window;++i) Check(r.send({"obstacle"}).mode==control::TrajMode::LANE,"Armed chặn né");
            r.in.odo_cm=100+sign*r.p.turn_arm_cm; ++r.now;
            Check(r.step().reason!="HUY_RE","Đúng biên chưa hủy");
            r.in.odo_cm+=sign*0.01; ++r.now;
            auto plan=r.step(); Check(plan.state==control::PlannerState::BAM_LAN && plan.reason=="HUY_RE" && plan.debug.reason==plan.reason,"Quá quãng phải hủy");
            r.advance(r.p.turn_start_cm/2); Check(r.advance(r.p.turn_start_cm/2).state==control::PlannerState::BAM_LAN,"Hủy không được rẽ lại");
        }
    } else if (name=="yaw") {
        for (bool left:{false,true}) for (double sign:{-1.0,1.0}) {
            Rig r; r.in.yaw_deg=37; r.turn(left);
            const double yaw_end=left?r.p.turn_yaw_deg:0.92*r.p.turn_yaw_deg; // rẽ phải thoát sớm 8% (đo log: vượt 4-7 độ)
            r.in.yaw_deg=37+sign*(yaw_end-0.01); ++r.now;
            Check(control::StateName(r.step().state)==std::string(left?"RE_TRAI":"RE_PHAI"),"Chưa đủ yaw còn rẽ");
            r.in.yaw_deg=37+sign*yaw_end; ++r.now;
            const auto plan=r.step(); Check(plan.state==control::PlannerState::BAM_LAN,"Đúng yaw đích phải xong, kể cả yaw âm");
        }
    } else if (name=="command") {
        double left_angle=0;
        for (bool left:{false,true}) {
            Rig r; r.p.turn_steer_deg=r.p.steer_max_deg=25; r.p.steer_rate_dps=10;
            r.arm(left?"left":"right"); auto before=r.advance(r.p.turn_start_cm-1); auto plan=r.junction();
            Check(std::abs(plan.cmd.deg-before.cmd.deg)<=0.4+1e-6,"Giới hạn rate từ góc vừa gửi");
            double previous=plan.cmd.deg;
            for (int i=0;i<30;++i) {
                r.now+=100; r.in.odo_cm+=1; plan=r.step();
                Check(std::abs(plan.cmd.deg-previous)<=1+1e-6 && std::abs(plan.cmd.deg)<=25 && plan.cmd.cms==TurnCms(r.p),"Rẽ giữ rate/max/min_cms");
                previous=plan.cmd.deg;
            }
            Check(std::abs(plan.cmd.deg)==25,"Phải đạt góc cố định, không chỉ kẹp về0");
            if (left) Check(left_angle*plan.cmd.deg<0,"Trái/phải phải có dấu góc ngược nhau");
            else left_angle=plan.cmd.deg;
            const double angle=plan.cmd.deg;
            r.in.mask.setTo(0); r.now+=40; plan=r.step();
            Check(plan.cmd.cms==TurnCms(r.p) && plan.cmd.deg==angle,"Đang rẽ bỏ qua mất làn");
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
        // Q9/TAM: lỗi cảm biến hủy rẽ tạm, tươi liên tục 1000 ms thì biển mới rẽ được.
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
        int64_t fresh_ms=-1;
        if (name=="fault_red") {
            Check(plan.state==control::PlannerState::DUNG_DEN_DO && plan.cmd.cms==0 && plan.cmd.deg==0,
                  "Đỏ đủ phiếu vẫn thắng lỗi cảm biến");
            r.in.sensor_ok=true; fresh_ms=r.now+40;
            for (int i=0;i<r.p.vote_window+r.p.green_frames;++i) plan=r.send({"green"});
            Check(plan.state==control::PlannerState::BAM_LAN,"Đủ xanh vẫn mở chốt đỏ sau lỗi cảm biến");
        } else Cancelled(plan,r.p);
        RecoveredSensor(r,fresh_ms);
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
