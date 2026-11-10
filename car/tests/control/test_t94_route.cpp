#include "control/Planner.h"
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
void Check(bool ok, const std::string &s) {
    if (!ok) throw std::runtime_error(s);
}
// Hợp đồng T94: chỉ dùng giao diện công khai, đồng hồ giả 40 ms.
struct Rig {
    control::Planner planner;
    control::ControlParams p;
    control::PlannerInput in;
    int64_t now=1000;
    uint64_t frame=0;
    bool stale_yolo=false;
    control::Plan last;
    Rig() {
        p.start_wait_green=0; p.turn_enable=1; p.kd=0;
        p.lane1_width_frac={0.4,0.4};
        in.yolo_enabled=true; in.sensor_ok=in.encoder_ok=true;
        mask(false); in.go=true; tick(); in.go=false;
    }
    void mask(bool tee) {
        in.mask=cv::Mat(100,200,CV_8UC1,cv::Scalar(0));
        for(int y=0;y<100;++y) {
            if(tee && y<72) continue;
            const int l=tee && y<79 ? 0:60;
            const int r=tee && y<79 ? 199:140;
            in.mask.row(y).colRange(l,r+1).setTo(1);
        }
    }
    control::Plan tick(bool island=false) {
        now+=40; in.odo_cm+=1;
        in.image_ms=in.sensor_ms=in.mask_ms=now;
        if(!stale_yolo) in.detections={{},now,++frame};
        if(island) in.detections.items.push_back(
            {"round_barrier",cv::Rect2f(490,100,300,80),0.9f,now});
        last=planner.Step(in,now,p);
        return last;
    }
    void hold(double yaw,int n=20,bool island=false,bool no_turn=false) {
        in.yaw_deg=in.yaw=yaw;
        for(int i=0;i<n;++i) {
            const auto a=tick(island);
            if(no_turn) Check(a.state!=control::PlannerState::RE_TRAI &&
                a.state!=control::PlannerState::RE_PHAI,"Góc bám làn không được RE_*");
        }
    }
    void expect(int step,const std::string &note="") {
        Check(planner.RouteStep()==step,"Chặng mong "+std::to_string(step)+
              ", thực "+std::to_string(planner.RouteStep())+
              ", note="+planner.RouteNote());
        if(!note.empty()) Check(planner.RouteNote()==note,
            "Note mong "+note+", thực "+planner.RouteNote());
    }
    void tee(bool left) {
        mask(true); bool seen=false;
        for(int i=0;i<20;++i) {
            const auto a=tick();
            seen=seen || a.state==control::PlannerState::RE_TRAI;
            if(!left) Check(a.state!=control::PlannerState::RE_TRAI &&
                a.state!=control::PlannerState::RE_PHAI,"Không ý định nhưng rẽ: "+a.reason);
        }
        if(left) Check(seen,"Có ý định nhưng không RE_TRAI tại T");
        mask(false);
    }
    void open() { for(int i=0;i<p.vote_window+4;++i) tick(true); }
    void close(double yaw) {
        in.yaw_deg=in.yaw=yaw;
        // Mất hộp và đi xa hơn rb_lost_cm, cộng cửa sổ phiếu.
        const int n=static_cast<int>(p.rb_lost_cm)+p.vote_window+30;
        for(int i=0;i<n;++i) tick();
        hold(yaw);
    }
    void anchor() { open(); expect(0,"NEO_VX1"); close(-90); expect(1,"BUOC"); }
    void reach(int c) {
        anchor();
        if(c>=2) { hold(0); expect(2,"BUOC"); }
        if(c>=3) { hold(90); expect(3,"BUOC"); }
        if(c>=4) { hold(180); expect(4,"BUOC"); }
        if(c>=5) { open(); close(90); expect(5,"BUOC"); }
        if(c>=6) { hold(0); expect(6,"BUOC"); }
        if(c>=7) { hold(-90); expect(7,"BUOC"); }
    }
    // Hộp biển và mặt nạ theo Rig T79, đầu vào công khai.
    control::Plan sample(const std::string &label="", float width=80, bool moving=true) {
        now+=40; if(moving) in.odo_cm+=1;
        in.image_ms=in.sensor_ms=in.mask_ms=now;
        in.detections={{},now,++frame};
        if(!label.empty()) in.detections.items.push_back({label,
            cv::Rect2f(640-width/2,label=="work_barrier"?400:100,width,80),0.9f,now});
        last=planner.Step(in,now,p); return last;
    }
    void opening(bool right=false) {
        mask(false);
        for(int y=79;y<100;++y)
            in.mask.row(y).colRange(right?140:0,right?200:61).setTo(1);
    }
    void votes(const std::string &label,float width) {
        for(int i=0;i<p.vote_window+4;++i) sample(label,width);
    }
    void sign_right() {
        votes("right",80); expect(7,"HET_BIEN_RAO"); opening(true);
        bool seen=false;
        for(int i=0;i<20;++i) {
            const auto a=sample("right");
            Check(a.state!=control::PlannerState::RE_TRAI,"Biển phải đủ phiếu phải còn điều khiển lượt phải");
            seen=seen || a.state==control::PlannerState::RE_PHAI;
        }
        Check(seen,"Biển phải đủ phiếu phải còn điều khiển lượt phải");
    }
    void restart(double yaw) {
        in.stop=true; tick(); in.stop=false;
        planner.Reset(true);
        in.yaw_deg=in.yaw=yaw; in.go=true; tick(); in.go=false;
    }
};
void Run(const std::string &name) {
    Rig r;
    if(name=="pre_anchor") {
        r.hold(90); r.expect(0,""); Check(r.planner.RouteNote().empty(),"Chưa neo note phải rỗng");
        r.tee(false);
    } else if(name=="sequence") {
        // Kiểm ý định bằng Rig riêng để lượt RE_* không gây nhiễu chuỗi bám làn.
        r.reach(7);
        Rig a; a.reach(2); a.tee(true);
        Rig b; b.reach(3); b.tee(true);
        for(int c: {1,4,5,6,7}) { Rig x; x.reach(c); x.tee(false); }
    } else if(name=="lane_l") {
        r.anchor(); r.hold(0,20,false,true); r.expect(2,"BUOC");
        Rig b; b.reach(5); b.hold(0,20,false,true); b.expect(6,"BUOC");
        b.hold(-90,20,false,true); b.expect(7,"BUOC");
    } else if(name=="noise") {
        r.anchor();
        for(int i=0;i<6;++i) { r.hold(0,8); r.expect(1); r.hold(-90,8); r.expect(1); }
        r.hold(-45,40); r.expect(1);
        r.hold(0); r.expect(2,"BUOC"); r.hold(0,80); r.expect(2);
    } else if(name=="space_corner") {
        r.anchor(); r.hold(-45,8); r.restart(-45);
        r.hold(-45); r.expect(1); r.tee(false);
        r.hold(-90); r.expect(1,"SYNC_R2");
    } else if(name=="space_island") {
        r.open(); r.expect(0,"NEO_VX1");
        r.hold(-45,8,true); r.expect(0);
        r.restart(-45); r.hold(-45); r.expect(0); r.tee(false);
        r.hold(0); r.expect(0,"SYNC_R2");
    } else if(name=="imu") {
        r.reach(2); r.in.sensor_ok=false; r.hold(0,8); r.expect(2); r.tee(false);
        r.in.sensor_ok=true; r.hold(0,40); r.expect(2);
    } else if(name=="r1") {
        r.reach(2); r.in.sensor_ok=false; r.restart(0);
        r.hold(0); r.expect(2); r.tee(false);
        r.in.sensor_ok=true; r.hold(45); r.expect(2); r.tee(false);
        r.hold(0,40); r.expect(2,"SYNC_R2");
    } else if(name=="r2") {
        r.reach(2); r.restart(0); r.hold(0); r.expect(2,"SYNC_R2"); r.tee(true);
    } else if(name=="r3_prev") {
        r.reach(2); r.restart(-90); r.hold(-90); r.expect(1,"SYNC_R3"); r.tee(false);
    } else if(name=="r3_next") {
        r.reach(2); r.restart(90); r.hold(90); r.expect(3,"SYNC_R3"); r.tee(true);
    } else if(name=="r4") {
        r.anchor(); r.restart(180); r.hold(180); r.expect(4,"SYNC_R4"); r.tee(false);
    } else if(name=="r5") {
        r.reach(6); r.restart(0); r.hold(0); r.expect(6,"SYNC_R2");
        r.open(); r.expect(0,"SYNC_R5"); r.close(-90); r.expect(1,"BUOC");
    } else if(name=="r5_south") {
        r.anchor(); r.restart(180); r.hold(180); r.open(); r.expect(4,"SYNC_R4");
    } else if(name=="r6") {
        r.reach(4); r.restart(90); r.hold(90); r.expect(4,"SYNC_AMBIG"); r.tee(false);
        Rig b; b.anchor(); b.restart(0); b.hold(0); b.expect(1,"SYNC_AMBIG"); b.tee(false);
    } else if(name=="sync_corner") {
        r.reach(4); r.restart(90); r.hold(90); r.expect(4,"SYNC_AMBIG");
        r.hold(0); r.expect(6,"SYNC_GOC"); r.tee(false);
    } else if(name=="sync_east_plain" || name=="sync_east_island") {
        r.anchor(); r.restart(0); r.hold(0); r.expect(1,"SYNC_AMBIG");
        if(name=="sync_east_island") {
            r.open(); r.expect(0,"SYNC_R5"); r.close(-90); r.expect(1);
        } else { r.hold(-90); r.expect(7,"SYNC_GOC"); }
        r.tee(false);
    } else if(name=="final_ts") {
        r.reach(7);
        for(int i=0;i<2;++i) { r.tee(false); r.hold(-90); r.expect(7); }
        r.hold(0); r.expect(7,"HUONG_SAI");
    } else if(name=="wrong_sign") {
        r.anchor(); r.hold(-180); r.expect(1,"HUONG_SAI"); r.tee(false);
        Rig b; b.anchor(); b.hold(90); b.expect(1,"HUONG_SAI"); b.tee(false);
    } else if(name=="wrong_then_back") {
        // Hợp đồng bổ sung: HUONG_SAI giữ mơ hồ, quay lại không chốt ý định.
        r.reach(2); r.hold(-90,20,false,true); r.expect(2,"HUONG_SAI");
        r.hold(0,20,false,true); r.expect(2); r.tee(false); r.expect(2);
        r.hold(90,20,false,true); r.expect(2,"SYNC_AMBIG"); r.tee(false);
    } else if(name=="stale_yolo_no_arm") {
        // Đến c2 với lô cũ từ trước góc; giữ cả captured_ms và frame_id.
        r.anchor(); r.stale_yolo=true;
        r.in.detections.captured_ms=r.now-r.p.det_max_age_ms-1;
        const auto captured=r.in.detections.captured_ms;
        const auto frame_id=r.in.detections.frame_id;
        r.hold(0); r.expect(2,"BUOC"); r.tee(false);
        Check(r.in.detections.captured_ms==captured &&
              r.in.detections.frame_id==frame_id,"Lô YOLO cũ phải giữ mốc và frame_id");
        r.stale_yolo=false; r.hold(0); r.expect(2); r.tee(true);
    } else if(name=="stale_intent") {
        r.reach(2); r.hold(90); r.expect(3,"BUOC");
        r.hold(180); r.expect(4,"BUOC"); r.tee(false);
    } else if(name=="island_freeze_axis_swing") {
        // Giữ chặng trong lần qua đảo, dù hướng đổi và ổn định nhiều lần.
        r.open(); r.expect(0,"NEO_VX1");
        for(double yaw: {0.0,-90.0,90.0}) {
            r.in.yaw_deg=r.in.yaw=yaw;
            for(int i=0;i<20;++i) { r.tick(true); r.expect(0,"NEO_VX1"); }
        }
        r.close(-90); r.expect(1,"BUOC"); r.hold(-90,40); r.expect(1);
        Rig b; b.reach(4); b.open();
        for(double yaw: {180.0,90.0,0.0}) {
            b.in.yaw_deg=b.in.yaw=yaw;
            for(int i=0;i<20;++i) { b.tick(true); b.expect(4,"BUOC"); }
        }
        b.close(90); b.expect(5,"BUOC"); b.hold(90,40); b.expect(5);
    } else if(name=="island_freeze_dwell") {
        // Chỉ đổi sang trục mới sau khi lần qua đảo đã đóng.
        r.open();
        for(int i=0;i<25;++i) { r.tick(true); r.expect(0,"NEO_VX1"); }
        r.close(0); r.expect(0,"NEO_VX1");
        r.hold(-90); r.expect(1,"BUOC"); r.hold(-90,40); r.expect(1);
        Rig b; b.reach(4); b.open();
        for(int i=0;i<25;++i) { b.tick(true); b.expect(4,"BUOC"); }
        b.close(180); b.expect(4); b.hold(90); b.expect(5,"BUOC");
        b.hold(90,40); b.expect(5);
    } else if(name=="space_mid_turn_c2") {
        // Space giữa lượt trái phải xóa ý định và đồng bộ đúng một lần.
        r.reach(2); r.mask(true); bool turning=false;
        for(int i=0;i<20 && !turning;++i)
            turning=r.tick().state==control::PlannerState::RE_TRAI;
        Check(turning,"Thiết lập c2 chưa vào RE_TRAI");
        r.in.yaw_deg=r.in.yaw=45; r.restart(45);
        r.hold(45,20,false,true); r.expect(2); r.tee(false);
        r.hold(90); r.expect(3,"SYNC_R3"); r.hold(90,40); r.expect(3,"SYNC_R3");
    } else if(name=="imu_fault_long_c2") {
        // Lỗi 600 ms giữ chặng và cấm rẽ; hồi tươi hơn 1 s rồi thử chữ T.
        r.reach(2); r.in.sensor_ok=false; r.mask(true);
        for(int i=0;i<15;++i) {
            const auto a=r.tick(); r.expect(2);
            Check(a.state!=control::PlannerState::RE_TRAI,"Lỗi IMU vẫn RE_TRAI");
        }
        r.mask(false); r.in.sensor_ok=true; r.hold(0,40); r.expect(2); r.tee(true);
        Rig b; b.reach(2); b.in.sensor_ok=false; b.mask(true);
        b.in.yaw_deg=b.in.yaw=90;
        for(int i=0;i<15;++i) {
            const auto a=b.tick(); b.expect(2);
            Check(a.state!=control::PlannerState::RE_TRAI,"Đổi yaw lúc lỗi vẫn RE_TRAI");
        }
        b.mask(false); b.in.sensor_ok=true; b.hold(90,40); b.expect(3,"BUOC");
        b.hold(90,40); b.expect(3,"BUOC");
    } else if(name=="first_g_no_sync") {
        // G đầu và Space/G trước neo không tạo đồng bộ hay ý định.
        r.hold(0); r.expect(0);
        Check(r.planner.RouteNote().empty(),"G đầu không được SYNC");
        r.restart(0); r.hold(0); r.expect(0);
        Check(r.planner.RouteNote().empty(),"Space/G trước neo note phải rỗng");
        r.tee(false); r.expect(0);
        Check(r.planner.RouteNote().empty(),"Chữ T trước neo note phải rỗng");
        r.open(); r.expect(0,"NEO_VX1");
    } else if(name=="intent_cleared_in_island") {
        // R5 xóa ý định c2, không mang qua đảo tới c1.
        r.reach(2); r.restart(0); r.hold(0); r.expect(2,"SYNC_R2");
        r.open(); r.expect(0,"SYNC_R5"); r.mask(true);
        for(int i=0;i<20;++i) {
            const auto a=r.tick(true); r.expect(0,"SYNC_R5");
            Check(a.state!=control::PlannerState::RE_TRAI,"Ý định c2 còn trong đảo");
        }
        r.close(-90); r.expect(1,"BUOC"); r.tee(false);
    } else if(name=="h1_sign") {
        r.reach(2); r.sign_right(); r.mask(false); r.hold(-90,40);
        Check(r.last.state==control::PlannerState::BAM_LAN,"Lượt phải phải kết thúc"); r.expect(7); r.tee(false);
    } else if(name=="h1_barrier") {
        r.reach(3); r.votes("work_barrier",130); r.expect(7,"HET_BIEN_RAO"); r.opening(); bool seen=false;
        for(int i=0;i<20;++i) seen=r.sample("work_barrier",130).state==control::PlannerState::RE_TRAI || seen;
        Check(seen,"Rào đủ phiếu tại giao lộ phải RE_TRAI");
    } else if(name=="h1_then_r5") {
        r.reach(2); r.votes("right",80); r.expect(7,"HET_BIEN_RAO");
        r.restart(0); r.hold(0); r.open(); r.expect(0,"SYNC_R5"); r.close(-90); r.expect(1,"BUOC");
    } else if(name=="m2_stopped") {
        r.anchor(); r.in.yaw_deg=r.in.yaw=0; const double odo=r.in.odo_cm;
        for(int i=0;i<40;++i) { r.sample("",80,false); Check(r.in.odo_cm==odo,"Odo phải giữ nguyên"); r.expect(1); }
        r.hold(0,40); r.expect(2,"BUOC");
    } else if(name=="m2_space_carry") {
        r.anchor(); r.in.stop=true;
        for(double yaw:{0.0,90.0}) {
            r.in.yaw_deg=r.in.yaw=yaw;
            for(int i=0;i<40;++i) { r.sample("",80,false); r.expect(1); }
        }
        r.in.stop=false; r.planner.Reset(true); r.in.go=true; r.tick(); r.in.go=false;
        r.hold(90,40); r.expect(1,"SYNC_AMBIG"); r.tee(false);
    } else if(name=="m2_space_wheels" || name=="m2_space_released_wheels") {
        // M2: bánh quay khi mang xe không được đếm lúc Space hoặc DUNG chưa G.
        r.anchor(); r.in.stop=true; r.tick(); r.expect(1);
        if(name=="m2_space_released_wheels") {
            r.in.stop=false; r.planner.Reset(true);
        }
        for(double yaw:{0.0,90.0}) {
            r.in.yaw_deg=r.in.yaw=yaw;
            for(int i=0;i<40;++i) {
                const double odo=r.in.odo_cm;
                const auto a=r.sample("",80,true);
                Check(r.in.odo_cm>odo,"Odo phải tăng mỗi bước khi bánh quay");
                Check(a.state==control::PlannerState::DUNG || a.state==control::PlannerState::CHO_SAN_SANG,"Chưa G phải ở DUNG hoặc CHO_SAN_SANG");
                Check(a.cmd.cms==0,"Chưa G phải giữ tốc độ bằng 0");
                r.expect(1);
            }
        }
        r.in.stop=false; r.planner.Reset(true); r.in.go=true; r.tick(); r.in.go=false;
        r.hold(90,40); r.expect(1,"SYNC_AMBIG"); r.tee(false);
    } else if(name=="m3_no_double_left") {
        r.reach(2); r.tee(true); r.opening(); r.hold(90,20); r.expect(3,"BUOC");
        Check(r.last.state==control::PlannerState::BAM_LAN,"Lượt c2 phải kết thúc");
        for(int i=0;i<20;++i) Check(r.tick().state!=control::PlannerState::RE_TRAI,"Chưa thấy lại cạnh thật vẫn rẽ lần hai");
        r.mask(false); r.hold(90,8); r.opening(); bool seen=false;
        for(int i=0;i<20;++i) seen=r.tick().state==control::PlannerState::RE_TRAI || seen;
        Check(seen,"Có cạnh thật mới rồi mở trái phải RE_TRAI ở c3");
    } else if(name=="m1_far_island") {
        r.reach(3); r.votes("round_barrier",150); r.mask(true);
        for(int i=0;i<20;++i) Check(r.sample("round_barrier",150).state!=control::PlannerState::RE_TRAI,"Đảo nhỏ phải bỏ ý định");
        r.mask(false); r.hold(90,40); r.expect(3); r.tee(true);
    } else if(name=="m1_reset_c0_small_island") {
        r.reach(3); r.restart(0); r.hold(0); r.expect(2,"SYNC_R3");
        r.votes("round_barrier",150); r.expect(0,"SYNC_R5"); r.mask(true);
        for(int i=0;i<20;++i) {
            Check(r.sample("round_barrier",150).state!=control::PlannerState::RE_TRAI,"R5 đảo nhỏ không RE_TRAI"); r.expect(0,"SYNC_R5");
        }
    } else if(name=="ambig_no_c2c3") {
        r.anchor(); r.restart(0); r.hold(0); r.expect(1,"SYNC_AMBIG");
        r.hold(90); r.expect(1,"SYNC_AMBIG"); r.tee(false); r.hold(180); r.expect(4,"SYNC_GOC");
    } else if(name=="skip_same_heading" || name=="skip_same_heading_sign") {
        // Rủi ro bỏ chặng cùng hướng: R2 vẫn giữ ý định; H1 chỉ cứu khi có biển/rào.
        r.reach(2); r.restart(0); r.hold(0); r.expect(2,"SYNC_R2");
        if(name=="skip_same_heading") r.tee(true); else r.sign_right();
    } else if(name=="yaw_40_50") {
        r.reach(2);
        for(int i=0;i<200;++i) {
            r.in.yaw_deg=r.in.yaw=(i/10)%2?50:40;
            Check(r.tick().state!=control::PlannerState::RE_TRAI,"Yaw nhiễu không được rẽ"); r.expect(2);
        }
        r.hold(90); r.expect(3,"BUOC"); r.hold(90,40); r.expect(3,"BUOC");
        for(int i=0;i<200;++i) {
            r.in.yaw_deg=r.in.yaw=(i/10)%2?140:130;
            Check(r.tick().state!=control::PlannerState::RE_TRAI,"Yaw nhiễu c3 không được rẽ"); r.expect(3,"BUOC");
        }
    } else throw std::runtime_error("Tên ca lạ");
}
}
int main(int argc,char **argv) {
    try {
        Check(argc==2,"Cần đúng một tên ca"); Run(argv[1]);
        std::cout<<"PASS "<<argv[1]<<'\n'; return 0;
    } catch(const std::exception &e) { std::cerr<<"FAIL: "<<e.what()<<'\n'; return 1; }
}
