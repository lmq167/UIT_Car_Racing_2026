// Ca kiểm 13/11: lối ra vòng xuyến 1, đỗ xe (mẫu hỏng dài lúc tiến, chuỗi lùi vòng kín, nhiễu ngẫu nhiên có seed).
// Chỉ dùng giao diện công khai của Planner; đồng hồ giả, xe giả động học (dyaw = d·tan(lái)/L, L = 31 cm như Planner dùng cho cua IMU).
#include "control/Planner.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>

namespace {
using control::Plan;
using control::PlannerState;
const double kPi=3.14159265358979323846;
void Check(bool ok, const std::string &s) { if(!ok) throw std::runtime_error(s); }
std::string Seen(const Plan &p) {
    return std::string(control::StateName(p.state))+"/"+p.reason+" cms="+std::to_string(p.cmd.cms)+" deg="+std::to_string(p.cmd.deg)+" lane="+p.debug.state;
}
struct Rig {
    control::Planner planner;
    control::ControlParams p;
    control::PlannerInput in;
    int64_t now=1000;
    uint64_t frame=0;
    Plan last;
    Rig() {
        p.start_wait_green=0; p.turn_enable=1; p.kd=0; p.steer_rate_dps=1000;
        p.lane1_width_frac={0.4,0.4};
        in.yolo_enabled=true; in.sensor_ok=in.encoder_ok=true;
        lane(60,140); in.go=true; tick(); in.go=false;
        Check(last.state==PlannerState::BAM_LAN && last.cmd.cms>0,"Thiết lập: G với làn tốt phải bám làn; "+Seen(last));
    }
    void lane(int l, int r) {
        in.mask=cv::Mat(100,200,CV_8UC1,cv::Scalar(0));
        in.mask.colRange(l,r+1).setTo(1);
    }
    // Một nhịp 40 ms, xe tiến cm; hộp (lớp, tâm theo bề rộng ảnh, bề rộng px) nếu có.
    Plan tick(const std::string &label="", double cx=0.5, float w=300, double cm=1) {
        now+=40; if(std::isfinite(in.odo_cm)) in.odo_cm+=cm;
        in.image_ms=in.sensor_ms=in.mask_ms=now;
        in.detections={{},now,++frame};
        if(!label.empty()) in.detections.items.push_back({label,cv::Rect2f(float(cx*1280-w/2),100,w,80),0.9f,now});
        last=planner.Step(in,now,p);
        return last;
    }
    // Space rồi G như main.cpp: stop mỗi nhịp, Reset(true) khi đổi epoch, rồi go.
    void space_go() {
        in.stop=true; tick("",0.5,300,0); tick("",0.5,300,0);
        planner.Reset(true);
        in.stop=false; in.go=true; tick("",0.5,300,0); in.go=false;
        Check(last.state==PlannerState::BAM_LAN,"G sau Space phải bám làn; "+Seen(last));
    }
};

// 1) Log l27 12/11 (16925, dòng 1088-1096): cung phải lối ra vòng xuyến mới quay 23° trong 57 cm thì hộp đảo rời khung (VX_THOAT);
//    cung phải bị bỏ, bộ bám làn lái TRÁI +16° về phía đảo. Cung phải phải giữ tới khi đã quay đủ hoặc hết lần qua.
void ArcThroughExit() {
    Rig r; r.in.yaw_deg=0;
    for(int i=0;i<r.p.vote_window+4;++i) r.tick("round_barrier");
    Check(r.last.reason=="VX_QUANH_DAO","Thiết lập: đảo lớn mở lần qua; "+Seen(r.last));
    for(int i=0;i<20;++i) r.tick("round_barrier");
    Check(std::abs(r.last.cmd.deg+15)<1e-6,"Thiết lập: sau 15 cm cung phải -15°; "+Seen(r.last));
    // Xe đã quay phải 20°; hành lang trôi dần sang trái ảnh (bám làn muốn lái trái), hộp đảo còn thấy.
    r.in.yaw_deg=-20;
    for(int s=4;s<=40;s+=4) { r.lane(60-s,140-s); r.tick("round_barrier"); }
    Check(r.last.debug.state=="BAM_LAN" && r.last.debug.e<0,"Thiết lập: làn hợp lệ, mục tiêu bên trái; "+Seen(r.last));
    // Hộp đảo rời khung: còn trong lần qua (chưa đi hết rb_lost_cm) thì cung phải vẫn giữ.
    int exit_ticks=0;
    for(int i=0;i<20;++i) {
        r.tick();
        if(r.last.reason!="VX_THOAT") continue;
        ++exit_ticks;
        Check(r.last.state==PlannerState::BAM_LAN && r.last.cmd.deg<=-15+1e-6,
              "VX_THOAT khi mới quay 20°: phải giữ cung phải >= 15°, không lái trái về đảo; "+Seen(r.last));
        Check(r.last.cmd.cms<=r.p.max_speed_cms && r.last.cmd.cms>=0,"Tốc độ trong trần; "+Seen(r.last));
    }
    Check(exit_ticks>=5,"Thiết lập: phải có pha VX_THOAT; số nhịp="+std::to_string(exit_ticks));
    // Đối chứng: đã quay đủ 40° thì trả quyền cho bám làn (không ép phải nữa).
    Rig q; q.in.yaw_deg=0;
    for(int i=0;i<q.p.vote_window+24;++i) q.tick("round_barrier");
    q.in.yaw_deg=-41;
    for(int s=4;s<=40;s+=4) { q.lane(60-s,140-s); q.tick("round_barrier"); }
    for(int i=0;i<12;++i) q.tick();
    Check(q.last.reason=="VX_THOAT" && q.last.cmd.deg>-15,"Đã quay 41°: bám làn tự lái, không ép cung phải; "+Seen(q.last));
}

// Xe giả cho pha đỗ: nhịp 50 ms, quãng = lệnh tốc độ, yaw theo góc lái đã gửi; ToF theo ô (50 cm) kể từ lần quét đầu.
struct ParkSim {
    Rig r;
    double pos=0;            // vị trí dọc bãi (cm), chỉ để tra ô cho ToF
    double s0=0; bool lot=false;  // mép đầu ô 0 (đặt khi quét lần đầu: lần quét đầu ở giữa ô 0)
    bool occ[3]={false,true,true};
    bool odo_nan=false, sensor_off=false;
    double reverse_cm=0;
    int ticks=0;
    bool xong_seen=false, xong_ever=false;
    explicit ParkSim(int side=-1) {
        r.p.steer_rate_dps=200; r.p.park_enable=1; r.p.park_tof_map=1234; r.p.park_side=side;
        r.in.yaw_deg=0; r.in.odo_cm=0;
    }
    int cell_mm(double x) const {
        if(!lot || x<s0) return 150;  // trước bãi: coi như có vật (không nhận nhầm ô trống)
        const int i=int((x-s0)/50.0);
        return i<3 && occ[i] ? 150 : 700;
    }
    Plan step(bool sign=false) {
        const double dt=0.05;
        const Plan prev=r.last;
        // Động học theo lệnh nhịp trước.
        const double d=prev.cmd.cms*dt;
        pos+=d; if(d<0) reverse_cm-=d;
        true_odo+=d;
        r.in.yaw_deg+=d*std::tan(prev.cmd.deg*kPi/180.0)/31.0*180.0/kPi;
        r.now+=50; ++ticks;
        r.in.odo_cm=odo_nan ? std::numeric_limits<double>::quiet_NaN() : true_odo;
        r.in.sensor_ok=!sensor_off;
        r.in.image_ms=r.in.sensor_ms=r.in.mask_ms=r.now;
        r.in.detections={{},r.now,++r.frame};
        if(sign) r.in.detections.items.push_back({"parking",cv::Rect2f(576,200,128,128),0.9f,r.now});
        const bool left=r.p.park_side==1;
        for(int i=0;i<4;++i) {
            auto &t=r.in.tof[i];
            const bool front=i==1 || i==2, mine=left ? i<2 : i>=2;
            const int mm=mine ? cell_mm(pos+(front ? 8 : -8)) : 700;
            t.initialized=t.has_sample=true; t.api_error=0; t.valid=true; t.range_status=0; t.mm=mm; t.sample_ms=r.now;
        }
        r.last=r.planner.Step(r.in,r.now,r.p);
        if(!lot && r.last.reason=="DO_XE_QUET") { lot=true; s0=pos-25; }
        invariants(prev);
        return r.last;
    }
    double true_odo=0;
    void invariants(const Plan &prev) {
        const Plan &a=r.last;
        const bool park=a.state==PlannerState::DO_XE || a.state==PlannerState::DO_XE_XONG;
        const std::string at=" [nhịp "+std::to_string(ticks)+"] "+Seen(a);
        Check(std::isfinite(a.cmd.cms) && std::isfinite(a.cmd.deg),"Lệnh phải hữu hạn"+at);
        Check(a.cmd.cms<=r.p.max_speed_cms+1e-9,"Tốc độ vượt max_speed_cms"+at);
        Check(a.cmd.cms>=-r.p.park_rev_cms-1e-9,"Lùi nhanh hơn park_rev_cms"+at);
        Check(a.cmd.cms>=0 || a.state==PlannerState::DO_XE,"Chỉ pha đỗ được lùi"+at);
        Check(std::abs(a.cmd.deg)<=(park ? 45.0 : r.p.steer_max_deg)+1e-9,"Góc lái vượt trần (45° chỉ trong pha đỗ)"+at);
        if(a.state==PlannerState::DO_XE && (odo_nan || sensor_off)) Check(a.cmd.cms==0,"Mẫu cảm biến hỏng trong pha đỗ: không được chuyển động"+at);
        if(a.state==PlannerState::DO_XE_XONG) { xong_seen=xong_ever=true; Check(a.cmd.cms==0,"DO_XE_XONG phải đứng yên"+at); }
        if(a.state==PlannerState::DUNG) Check(a.cmd.cms==0,"DUNG phải đứng yên"+at);
        if(xong_seen && !r.in.stop && !r.in.go) Check(a.state==PlannerState::DO_XE_XONG || a.state==PlannerState::DUNG,"Đã đỗ xong thì không tự chạy lại"+at);
        Check(reverse_cm<=10+80+80+5,"Tổng quãng lùi vượt hạn các pha"+at);
        (void)prev;
    }
    // Biển P đủ phiếu rồi chạy tới khi kết thúc (đỗ xong/dừng) hoặc hết nhịp.
    void arm() { for(int i=0;i<r.p.vote_min+1;++i) step(true); }
    bool done() const { return r.last.state==PlannerState::DO_XE_XONG || r.last.state==PlannerState::DUNG; }
};

// 3) Chuỗi đỗ vòng kín đủ pha: ô 0 trống, ô 1 có xe -> phải kết thúc DO_XE_XONG, motor 0 (hai phía).
void ParkClosedLoop() {
    for(int side : {-1,1}) {
        ParkSim s(side);
        s.arm();
        std::string trail;
        std::string note;
        for(int i=0;i<4000 && !s.done();++i) {
            s.step();
            if(trail.empty() || trail.substr(trail.rfind(' ')+1)!=s.r.last.reason) trail+=" "+s.r.last.reason;
            if(note!=s.r.planner.ParkNote()) { note=s.r.planner.ParkNote(); trail+=" {"+note+"}"; }
        }
        Check(s.r.last.state==PlannerState::DO_XE_XONG && s.r.last.cmd.cms==0,
              "Ô trống rồi ô có xe: phải đỗ xong (DO_XE_XONG, motor 0); phía="+std::to_string(side)+" kết thúc "+Seen(s.r.last)+" note="+s.r.planner.ParkNote()+" chuỗi:"+trail);
        Check(std::abs(s.r.in.yaw_deg)<=3+1e-9,"Đỗ xong hướng lệch <= 3°; yaw="+std::to_string(s.r.in.yaw_deg));
        for(int i=0;i<100;++i) s.step();
    }
}

// 4) Log l27 12/11 (16656): pha TIẾN bị hủy ngầm rồi xe chạy thẳng 15 cm/s trong bãi. Mẫu hỏng DÀI (>= 250 ms) lúc tiến quét/dò mép
//    phải DỪNG có lý do (DO_XE_CAM_BIEN), không quay về bám làn tốc độ thường; mẫu hỏng ngắn vẫn giữ pha.
void ParkTienLongFault() {
    for(int kind=0;kind<2;++kind) {
        ParkSim s; s.arm();
        for(int i=0;i<400 && s.r.last.reason!="DO_XE_QUET";++i) s.step();
        for(int i=0;i<60 && s.r.last.reason=="DO_XE_QUET";++i) s.step();
        Check(s.r.last.state==PlannerState::BAM_LAN && s.r.last.cmd.cms>0 && s.r.last.cmd.cms<=5+1e-9,"Thiết lập: đang TIẾN dò mép <= 5 cm/s; "+Seen(s.r.last));
        (kind ? s.sensor_off : s.odo_nan)=true;
        s.step();
        Check(s.r.last.state==PlannerState::BAM_LAN && s.r.last.cmd.cms<=5+1e-9,"Mẫu hỏng ngắn: giữ pha TIẾN, không tăng tốc; "+Seen(s.r.last));
        for(int i=0;i<6;++i) {
            s.step();
            Check(s.r.last.cmd.cms<=5+1e-9,"Mẫu hỏng kéo dài lúc TIẾN: không được về tốc độ bám làn thường; "+Seen(s.r.last));
        }
        Check(s.r.last.state==PlannerState::DUNG && s.r.last.reason=="DO_XE_CAM_BIEN" && s.r.last.cmd.cms==0,
              "Mẫu hỏng >= 250 ms lúc TIẾN: phải dừng DO_XE_CAM_BIEN; "+Seen(s.r.last));
        s.odo_nan=s.sensor_off=false;
        for(int i=0;i<40;++i) { s.step(); Check(s.r.last.state==PlannerState::DUNG && s.r.last.cmd.cms==0,"Hồi cảm biến: vẫn dừng tới khi bấm G; "+Seen(s.r.last)); }
    }
}

// 5) Nhiễu ngẫu nhiên có seed: mẫu odo/IMU hỏng ngắn và dài ở mọi pha, Space/G, mẫu ô ngẫu nhiên; mọi nhịp kiểm ranh giới cứng.
void ParkFuzz() {
    int xong=0, total=0;
    for(unsigned seed=1;seed<=300;++seed) {
        std::mt19937 rng(seed);
        auto chance=[&](double p) { return std::uniform_real_distribution<double>(0,1)(rng)<p; };
        ParkSim s(seed%2 ? -1 : 1);
        for(bool &o:s.occ) o=chance(0.6);
        const bool faults=seed%3!=0;
        const bool solvable=(!s.occ[0] && s.occ[1]) || (s.occ[0] && !s.occ[1] && s.occ[2]);
        try {
            s.arm();
            int bad_left=0, space_left=0; bool faulted=false, spaced=false;
            for(int i=0;i<3000;++i) {
                if(faults && bad_left==0 && chance(0.01)) { bad_left=chance(0.25) ? 8 : 1+int(chance(0.5)); faulted=true; (chance(0.5) ? s.odo_nan : s.sensor_off)=true; }
                else if(bad_left>0 && --bad_left==0) s.odo_nan=s.sensor_off=false;
                if(faults && !s.r.in.stop && chance(0.0007)) { s.r.in.stop=true; space_left=6; spaced=true; s.xong_seen=false; }  // người bấm Space rồi G: được chạy lại
                else if(space_left>0 && --space_left==0) { s.r.planner.Reset(true); s.r.in.stop=false; s.r.in.go=true; }
                s.step();
                s.r.in.go=false;
            }
            ++total; xong+=s.xong_ever;
            if(!faulted && !spaced && solvable)
                Check(s.r.last.state==PlannerState::DO_XE_XONG,"Không nhiễu, có ô trống trước ô có xe: phải đỗ xong; "+Seen(s.r.last)+" note="+s.r.planner.ParkNote());
            if(!spaced) Check(s.r.last.cmd.cms==0 || s.r.last.state==PlannerState::BAM_LAN,"Cuối lượt: đứng yên hoặc còn bám làn; "+Seen(s.r.last));
        } catch(const std::exception &e) {
            throw std::runtime_error("seed "+std::to_string(seed)+": "+e.what());
        }
    }
    std::cout << "park_fuzz: " << xong << "/" << total << " lượt tới DO_XE_XONG\n";
    Check(xong>=20,"Phải có đủ lượt đỗ xong để phép kiểm có nghĩa");
}

// 6) Vòng xuyến + Space/G ngẫu nhiên có seed: ranh giới cứng ở mọi nhịp (không lùi, <= max_speed_cms, <= steer_max_deg), chặng luôn trong 0..7.
void RoundaboutFuzz() {
    for(unsigned seed=1;seed<=200;++seed) {
        std::mt19937 rng(seed);
        auto uni=[&](double a, double b) { return std::uniform_real_distribution<double>(a,b)(rng); };
        Rig r; r.p.steer_rate_dps=200; r.in.yaw_deg=uni(-20,20);
        int island_left=0, bad_left=0, shift=0;
        try {
            for(int i=0;i<1500;++i) {
                if(island_left==0 && uni(0,1)<0.01) island_left=int(uni(10,80));
                if(uni(0,1)<0.02) shift=int(uni(-40,40));
                if(uni(0,1)<0.01) r.in.mask.setTo(0); else r.lane(60+shift,140+shift);
                if(bad_left==0 && uni(0,1)<0.01) { bad_left=uni(0,1)<0.3 ? 8 : 1; r.in.odo_cm=std::numeric_limits<double>::quiet_NaN(); }
                const double keep=r.in.odo_cm;
                if(uni(0,1)<0.004) { const double y=r.in.yaw_deg; r.in.odo_cm=std::isfinite(keep) ? keep : 0; r.space_go(); r.in.yaw_deg=y+uni(-60,60); if(bad_left) r.in.odo_cm=std::numeric_limits<double>::quiet_NaN(); }
                const bool isl=island_left>0; if(isl) --island_left;
                const double before=r.in.yaw_deg;
                r.tick(isl ? "round_barrier" : "",uni(0.1,0.9),float(uni(100,500)),1);
                if(bad_left>0 && --bad_left==0) r.in.odo_cm=double(i);
                const Plan &a=r.last;
                const std::string at=" [nhịp "+std::to_string(i)+"] "+Seen(a);
                Check(std::isfinite(a.cmd.cms) && std::isfinite(a.cmd.deg),"Lệnh hữu hạn"+at);
                Check(a.cmd.cms>=0 && a.cmd.cms<=r.p.max_speed_cms+1e-9,"Tốc độ trong [0, max]"+at);
                Check(std::abs(a.cmd.deg)<=r.p.steer_max_deg+1e-9,"Góc lái trong trần"+at);
                Check(r.planner.RouteStep()>=0 && r.planner.RouteStep()<=7,"Chặng trong 0..7"+at);
                r.in.yaw_deg=before+a.cmd.cms*0.04*std::tan(a.cmd.deg*kPi/180.0)/31.0*180.0/kPi;
            }
        } catch(const std::exception &e) {
            throw std::runtime_error("seed "+std::to_string(seed)+": "+e.what());
        }
    }
}
}

int main(int argc, char **argv) {
    const std::string name=argc>1 ? argv[1] : "";
    try {
        if(name=="arc_through_exit") ArcThroughExit();
        else if(name=="park_closed_loop") ParkClosedLoop();
        else if(name=="park_tien_long_fault") ParkTienLongFault();
        else if(name=="park_fuzz") ParkFuzz();
        else if(name=="roundabout_fuzz") RoundaboutFuzz();
        else throw std::runtime_error("Không có kịch bản: "+name);
    } catch(const std::exception &e) {
        std::cerr << "HỎNG " << name << ": " << e.what() << "\n";
        return 1;
    }
    std::cout << "ĐẠT " << name << "\n";
    return 0;
}
