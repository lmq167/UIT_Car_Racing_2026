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
    Rig() {
        p.turn_enable=1; p.start_wait_green=0; p.kd=0;
        p.lane_width_frac={0.4,0.4,0.4};
        in.yolo_enabled=true; in.sensor_ok=true;
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
    control::Plan edge(bool left=true) {
        mask(left?0:60,left?140:200); now+=40; return step();
    }
    control::Plan turn(bool left=true) {
        arm(left?"left":"right"); edge(left); auto result=edge(left);
        Check(control::StateName(result.state)==std::string(left?"RE_TRAI":"RE_PHAI"),"Thiết lập phải đang rẽ");
        return result;
    }
};
void Stopped(const control::Plan& plan, const std::string& reason) {
    Check(plan.state==control::PlannerState::DUNG && plan.reason==reason &&
          plan.debug.reason==reason && plan.cmd.cms==0 && plan.cmd.deg==0,
          "Lỗi phải DUNG motor0/servo0 đúng lý do: "+reason);
}

// T57_design:3,17: sáu mặc định, hai biên hợp lệ, ngoài miền và quan hệ góc.
void Params(const std::string& path) {
    control::ControlParams p;
    Check(p.turn_enable==0 && p.turn_yaw_deg==80 && p.turn_steer_deg==25 &&
          p.turn_arm_cm==80 && p.turn_max_cm==70 && p.turn_max_ms==8000,"Sai sáu mặc định");
    const char* names[]={"turn_enable","turn_yaw_deg","turn_steer_deg","turn_arm_cm","turn_max_cm","turn_max_ms"};
    const double low[]={0,30,5,10,10,1000}, high[]={1,120,30,300,200,20000};
    for (int i=0;i<6;++i) {
        for (double value:{low[i],high[i],low[i]-1,high[i]+1}) {
            { std::ofstream out(path); Check(bool(out),"Không mở được YAML tạm");
              out<<"steer_max_deg: 30\n"<<names[i]<<": "<<value<<'\n';
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

void Case(const std::string& name) {
    // T57_design:5,20,25: đủ phiếu bốn lớp, không thêm ngưỡng tin cậy.
    if (name=="signs") {
        for (const std::string sign:{"left","no_right","right","no_left"}) {
            Rig r; const bool left=sign=="left" || sign=="no_right";
            r.arm(sign); Check(r.edge(left).state==control::PlannerState::BAM_LAN,"J1 cần hai mask");
            const auto plan=r.edge(left);
            Check(control::StateName(plan.state)==std::string(left?"RE_TRAI":"RE_PHAI") &&
                  plan.reason==control::StateName(plan.state) && plan.debug.state==plan.reason &&
                  plan.debug.reason==plan.reason,"Sai hướng/lý do/debug biển");
        }
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
            auto plan=r.edge(); plan=r.edge();
            Check(control::StateName(plan.state)==std::string("BAM_LAN"),"Sai luật phiếu "+mode);
        }
    } else if (name=="low_conf_armed") {
        Rig r;
        for (int i=0;i<r.p.vote_min;++i) r.send({"left"},0.01f);
        r.edge(); Check(control::StateName(r.edge().state)==std::string("RE_TRAI"),"Conf thấp đủ phiếu vẫn armed");
    } else if (name=="space_during_turn") {
        for (bool turning:{false,true}) for (bool preserve:{false,true}) {
            Rig r; r.in.yaw_deg=37; r.in.odo_cm=100;
            if (turning) r.turn(); else r.arm();
            r.in.stop=true; ++r.now; Check(r.step().cmd.cms==0,"Space phải dừng");
            r.planner.Reset(preserve); r.in.stop=false; r.in.detections={};
            r.in.yaw_deg=-300; r.in.odo_cm=-300; r.in.go=true; ++r.now;
            Check(r.step().state==control::PlannerState::BAM_LAN,"G sau Space bỏ mốc cũ"); r.in.go=false;
            r.edge(); Check(r.edge().state==control::PlannerState::BAM_LAN,"Space xóa armed/hướng cũ");
            r.mask(60,140); r.in.yaw_deg=10; r.in.odo_cm=0;
            r.turn(false); r.in.yaw_deg=10+r.p.turn_yaw_deg; ++r.now;
            Check(r.step().state==control::PlannerState::BAM_LAN,"Phiên mới dùng yaw0 mới");
        }
    } else if (name=="nonfinite_sensor" || name=="armed_sensor_fault") {
        for (int fault=(name=="armed_sensor_fault"?0:1);fault<(name=="armed_sensor_fault"?1:8);++fault) {
            for (int phase=0;phase<3;++phase) {
                Rig r;
                if (phase==1) r.arm();
                if (phase==2) r.turn();
                std::string reason="LOI_CAM_BIEN";
                if (fault==0) r.in.sensor_ok=false;
                if (fault>=1 && fault<=3) r.in.yaw_deg=fault==1?std::numeric_limits<double>::quiet_NaN():
                    (fault==2?1:-1)*std::numeric_limits<double>::infinity();
                if (fault>=4 && fault<=6) r.in.odo_cm=fault==4?std::numeric_limits<double>::quiet_NaN():
                    (fault==5?1:-1)*std::numeric_limits<double>::infinity();
                if (fault==7) { r.in.sensor_ms=r.now+1; reason="CAM_BIEN_CU"; }
                r.in.go=true; const auto plan=r.step(true,fault!=7); r.in.go=false;
                if (phase==0) { Check(plan.state==control::PlannerState::BAM_LAN && plan.cmd.cms>0,"Chưa armed bỏ qua lỗi cảm biến"); continue; }
                Stopped(plan,reason);
                r.in.sensor_ok=true; r.in.yaw_deg=0; r.in.odo_cm=0; ++r.now;
                r.in.go=true; Stopped(r.step(),reason); r.in.go=false;
                r.in.stop=true; ++r.now; r.step(); r.in.stop=false; r.in.go=true; ++r.now;
                Check(r.step().cmd.cms>0,"Lỗi cảm biến chỉ mở sau Space/G");
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
        if (turning) { r.edge(); Check(r.edge().state==control::PlannerState::BAM_LAN,"Không rẽ tiếp yaw0 trước đỏ"); }
        else {
            r.in.odo_cm=100+r.p.turn_arm_cm+0.01; ++r.now;
            Check(r.step().reason=="HUY_RE","Đỏ giữ armed và mốc odo cũ");
        }
        if (!turning) {
            Rig kept; kept.p.vote_window=kept.p.vote_min; kept.arm();
            for (int i=0;i<kept.p.vote_min;++i) kept.send({"red"});
            for (int i=0;i<kept.p.vote_window+kept.p.green_frames;++i) kept.send({"green"});
            kept.edge(); Check(control::StateName(kept.edge().state)==std::string("RE_TRAI"),"Nhả đỏ còn armed khi trong quãng");
        }
    } else if (name=="sign_while_turning") {
        Rig r; r.in.odo_cm=100; r.arm();
        r.in.odo_cm=110;
        for (int i=0;i<r.p.vote_window;++i) r.send({"right","no_left"});
        r.edge(); Check(control::StateName(r.edge().state)==std::string("RE_TRAI"),"Armed bỏ biển ngược");
        for (int i=0;i<r.p.vote_window;++i) Check(control::StateName(r.send({"right"}).state)==std::string("RE_TRAI"),"Đang rẽ không đổi hướng");
        r.in.yaw_deg=r.p.turn_yaw_deg; ++r.now; Check(r.step().state==control::PlannerState::BAM_LAN,"Biển mới không đổi yaw0");
        Rig distance; distance.in.odo_cm=100; distance.arm(); distance.in.odo_cm=120;
        for (int i=0;i<distance.p.vote_window;++i) distance.send({"left"});
        distance.in.odo_cm=100+distance.p.turn_arm_cm+0.01; ++distance.now;
        Check(distance.step().reason=="HUY_RE","Biển mới không đặt lại mốc armed");
        Rig travel; travel.turn(); travel.in.odo_cm=10;
        for (int i=0;i<travel.p.vote_window;++i) travel.send({"right"});
        travel.in.odo_cm=travel.p.turn_max_cm+0.01; ++travel.now;
        Stopped(travel.step(),"RE_QUA_CM");
        Rig clock; clock.p.turn_max_ms=1000; clock.turn(); const auto start=clock.now;
        while (clock.now<start+clock.p.turn_max_ms) {
            const auto plan=clock.send({"right"});
            Check(control::StateName(plan.state)==std::string("RE_TRAI"),"Biển mới không đổi phiên trước hạn ms");
        }
        ++clock.now; Stopped(clock.step(),"RE_QUA_MS");
    } else if (name=="after_turn_lane") {
        Rig r; r.p.kd=2; r.mask(40,120); ++r.now; r.step(); r.turn();
        r.mask(80,160); r.in.yaw_deg=r.p.turn_yaw_deg; ++r.now;
        auto plan=r.step();
        Check(plan.state==control::PlannerState::BAM_LAN && std::abs(plan.debug.d)<1e-9,"Kết thúc rẽ reset đạo hàm");
        ++r.now; plan=r.step(); Check(std::abs(plan.debug.d)<1e-9,"Không xung đạo hàm ở mask kế tiếp");
    } else if (name=="j1") {
        Rig r; r.arm();
        for (int y=0;y<20;++y) for (int x=0;x<=60;++x) r.in.mask.at<unsigned char>(y,x)=1;
        for (int i=0;i<2;++i) { r.now+=40; Check(r.step().state==control::PlannerState::BAM_LAN,"Mép ở hàng xa không thay scan_rows[0]"); }
        Check(r.edge(false).state==control::PlannerState::BAM_LAN,"Sai mép không rẽ");
        Check(r.edge().state==control::PlannerState::BAM_LAN,"Mask mới thứ nhất");
        for (int i=0;i<3;++i) { ++r.now; Check(r.step(false).state==control::PlannerState::BAM_LAN,"Không đếm lại mask_ms"); }
        r.mask(60,140); ++r.now; r.step();
        Check(r.edge().state==control::PlannerState::BAM_LAN,"Mask đóng mép ngắt liên tiếp");
        Check(control::StateName(r.edge().state)==std::string("RE_TRAI"),"Hai mask mới liên tiếp mới rẽ");
    } else if (name=="arm_distance") {
        for (double sign:{-1.0,1.0}) {
            Rig r; r.in.odo_cm=100; r.arm();
            r.in.odo_cm=100+sign*r.p.turn_arm_cm; ++r.now;
            Check(r.step().reason!="HUY_RE","Đúng biên chưa hủy");
            r.in.odo_cm+=sign*0.01; ++r.now;
            auto plan=r.step(); Check(plan.state==control::PlannerState::BAM_LAN && plan.reason=="HUY_RE" && plan.debug.reason==plan.reason,"Quá quãng phải hủy");
            r.edge(); Check(r.edge().state==control::PlannerState::BAM_LAN,"Hủy không được rẽ lại");
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
            r.arm(left?"left":"right"); auto before=r.edge(left); auto plan=r.edge(left);
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
            auto x=a.edge(), y=b.edge();
            Check(x.state==y.state && x.reason==y.reason && x.cmd.cms==y.cmd.cms && x.cmd.deg==y.cmd.deg && x.state==control::PlannerState::BAM_LAN,"Tắt phải bỏ qua cảm biến/biển rẽ");
        }
    } else if (name=="red") {
        Rig r; r.turn(); control::Plan plan;
        for (int i=0;i<r.p.vote_min;++i) plan=r.send({"red"});
        Check(plan.state==control::PlannerState::DUNG_DEN_DO && plan.cmd.cms==0 && plan.cmd.deg==0,"Đỏ khi rẽ phải dừng đèn");
    } else {
        // T57_design:8,19,21,22; notes:3: mỗi lỗi giữ DUNG tới Space rồi G.
        Rig r; r.turn(); const auto start=r.now;
        std::string reason=name;
        if (name=="fault_red") { for (int i=1;i<r.p.vote_min;++i) r.send({"red"}); reason="LOI_CAM_BIEN"; }
        if (reason=="LOI_CAM_BIEN") r.in.sensor_ok=false;
        else if (reason=="CAM_BIEN_CU") {
            r.now+=200; Check(r.step(true,false).state!=control::PlannerState::DUNG,"200ms vẫn hợp lệ"); ++r.now;
        } else if (reason=="sensor_invalid") { r.in.sensor_ms=-1; reason="CAM_BIEN_CU"; }
        else if (reason=="RE_QUA_CM") {
            r.in.odo_cm=-r.p.turn_max_cm; ++r.now;
            Check(r.step().state!=control::PlannerState::DUNG,"Đúng biên cm chưa lỗi"); r.in.odo_cm-=0.01;
        } else if (reason=="RE_QUA_MS") {
            for (int64_t t=start+100;t<=start+r.p.turn_max_ms;t+=100) { r.now=t; r.in.odo_cm+=0.1; r.step(); }
            Check(r.step().state!=control::PlannerState::DUNG,"Đúng biên ms chưa lỗi"); ++r.now;
        } else if (reason=="KET_RE") {
            Rig progress; progress.turn(); progress.now+=1500; progress.in.odo_cm=1;
            Check(progress.step().state!=control::PlannerState::DUNG,"Đúng1cm/1500ms không phải kẹt");
            r.now=start+1499; Check(r.step().state!=control::PlannerState::DUNG,"Chưa1500ms không kẹt"); ++r.now;
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
        Stopped(plan,reason);
        r.in.sensor_ok=true; ++r.now; r.step(); r.in.go=true; Stopped(r.step(),reason); r.in.go=false;
        r.in.stop=true; ++r.now; r.step(); r.in.stop=false; ++r.now;
        Check(r.step().cmd.cms==0,"Space chưa G phải giữ dừng");
        r.in.go=true; ++r.now;
        if (name!="fault_red") Check(r.step().cmd.cms>0,"Space rồi G mới chạy lại");
    }
}
}
int main(int argc, char** argv) {
    try {
        Check(argc>=2,"Thiếu tên ca");
        if (std::string(argv[1])=="params") { Check(argc==3,"Thiếu YAML tạm"); Params(argv[2]); }
        else Case(argv[1]);
        std::cout<<"ĐẠT: "<<argv[1]<<'\n'; return 0;
    } catch (const std::exception& e) { std::cerr<<"KHÔNG ĐẠT: "<<e.what()<<'\n'; return 1; }
}
