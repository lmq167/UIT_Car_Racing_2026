#include "control/Planner.h"
#include <iostream>
#include <stdexcept>
#include <cmath>

namespace {
void Check(bool ok, const std::string &s) { if(!ok) throw std::runtime_error(s); }
struct Rig {
    control::Planner planner;
    control::ControlParams p;
    control::PlannerInput in;
    int64_t now=1000;
    uint64_t frame=0;
    double anchor=0;
    Rig() {
        p.start_wait_green=0; p.turn_enable=1; p.kd=0;
        p.lane1_width_frac={0.4,0.4};
        in.yolo_enabled=true; in.sensor_ok=true;
        mask(0); in.go=true; step(0); in.go=false;
    }
    // 0: thẳng; 1: mở phía rẽ; 2: chữ T; 3: cong hai biên; 4: mở phía rẽ chỉ ở xa; 5: đảo trên đường rộng.
    void mask(int shape, bool right=false) {
        in.mask=cv::Mat(100,200,CV_8UC1,cv::Scalar(0));
        for(int y=0;y<100;++y) {
            int l=60,r=140;
            if(shape==1 && y>=79) { if(right) r=199; else l=0; }
            if(shape==2 && y<72) continue;
            if(shape==2 && y<79) { l=0; r=199; }
            if(shape==3 && y<79) { const int d=((95-y)/5)%2 ? 20 : 0; l+=d; r+=d; }
            if(shape==4 && y<79) { if(right) r=199; else l=0; }
            if(shape==5) { l=0; r=199; }
            in.mask.row(y).colRange(l,r+1).setTo(1);
            if(shape==5 && y>=80 && y<=90) in.mask.row(y).colRange(20,51).setTo(0);
        }
    }
    control::Plan step(double odo, const std::string &sign="", bool new_mask=true) {
        now+=40; in.odo_cm=odo; in.image_ms=in.sensor_ms=now;
        if(new_mask) in.mask_ms=now;
        in.detections={{},now,++frame};
        if(!sign.empty()) {
            const float width=sign=="round_barrier" ? 300.f : (sign=="work_barrier" ? 130.f : 80.f);
            in.detections.items.push_back({sign,cv::Rect2f(640-width/2,100,width,80),0.9f,now});
        }
        return planner.Step(in,now,p);
    }
    void arm(bool right=false) {
        for(int i=1;i<=p.vote_min;++i) {
            const auto plan=step(i,right?"right":"left");
            Check(plan.state==control::PlannerState::BAM_LAN,"Chốt biển phải chờ giao lộ");
        }
        anchor=in.odo_cm;
    }
};
void Turn(const control::Plan &plan, const std::string &reason, bool right=false) {
    Check(plan.state==(right?control::PlannerState::RE_PHAI:control::PlannerState::RE_TRAI),"Sai trạng thái rẽ: "+plan.reason);
    Check(plan.debug.reason==reason && plan.reason==reason,"Sai lý do khung đầu: "+plan.reason);
}
void Run(const std::string &name) {
    // Kiểm tốc độ rẽ và tiếp quản theo yêu cầu qua Step công khai.
    if(name.find("handover_")==0 || name=="imu_speed" || name=="imu_timeout") {
        const bool right=name=="handover_right";
        for(double ceiling:(name=="imu_speed"?std::vector<double>{20,6}:std::vector<double>{20})) {
            Rig r; r.p.max_speed_cms=ceiling; r.p.min_cms=4;
            r.p.cruise_cms=ceiling; r.p.approach_cms=4;
            r.arm(right); r.mask(1,right); control::Plan plan;
            for(int i=0;i<r.p.vote_min;++i) plan=r.step(r.in.odo_cm);
            Turn(plan,"RE_GIAO_LO",right);
            const auto state=right?control::PlannerState::RE_PHAI:control::PlannerState::RE_TRAI;
            if(name=="imu_speed") {
                Check(plan.cmd.cms==(ceiling==20?8:6),"Tốc độ đầu lượt phải là 8 có chặn trần");
                plan=r.step(r.in.odo_cm);
                Check(plan.state==state && plan.cmd.cms==(ceiling==20?8:6),"Tốc độ đang rẽ phải là 8 có chặn trần");
                continue;
            }
            r.in.yaw_deg=right?-42:(name=="handover_low_yaw"?30:50);
            if(name!="handover_no_corridor" && name!="imu_timeout") r.mask(0);
            else {
                r.in.mask.setTo(0); r.in.mask.colRange(0,141).setTo(1);
                // Cạnh trái chạm mép ảnh: chưa có hai cạnh thật, vẫn còn làn để hồi về.
            }
            if(name=="handover_offset") {
                r.in.mask.setTo(0); r.in.mask.colRange(120,199).setTo(1);
            }
            const auto corridor=control::ExtractCorridor(r.in.mask,r.p,control::TrajMode::LANE);
            int real=0; double offset=0;
            for(const auto &row:corridor.rows) if(row.valid && row.conf>=r.p.conf_min && row.flag_left=="THAT" && row.flag_right=="THAT") {
                ++real; offset+=((row.left+row.right)/2-r.in.mask.cols/2.0)/r.in.mask.cols;
            }
            if(name=="handover_no_corridor" || name=="imu_timeout") Check(real<3,"Thiết lập chưa đủ 3 hàng hai cạnh thật");
            else {
                Check(real>=3,"Thiết lập có ít nhất 3 hàng hai cạnh thật");
                Check(name=="handover_offset"?std::abs(offset/real-0.3)<0.01:std::abs(offset/real)<=0.15,"Thiết lập độ lệch tâm");
            }
            if(name=="imu_timeout") {
                r.now+=r.p.turn_max_ms+1; plan=r.step(r.in.odo_cm);
                Check(plan.state==control::PlannerState::BAM_LAN && plan.reason=="HUY_RE_CAM_BIEN" &&
                      plan.cmd.cms>0 && plan.cmd.cms<=r.p.max_speed_cms && std::abs(plan.cmd.deg)<=r.p.steer_max_deg,
                      "Hết hạn phải hủy rẽ, lệnh trong giới hạn như cũ: "+plan.reason);
                continue;
            }
            const bool fault=name=="handover_sensor_fault";
            if(fault) r.in.sensor_ok=false;
            const bool takeover=name=="handover_center" || right;
            plan=r.step(r.in.odo_cm);
            Check(plan.state==state,"Một mặt nạ chưa đủ phiếu");
            if(!fault) for(int i=0;i<r.p.vote_min+1;++i) {
                plan=r.step(r.in.odo_cm,"",false);
                Check(plan.state==state,"Mặt nạ lặp không cộng phiếu");
            }
            for(int i=1;i<r.p.vote_min;++i) {
                plan=r.step(r.in.odo_cm);
                if(i<r.p.vote_min-1 || !takeover) Check(plan.state==state,"Chưa đủ điều kiện phải giữ rẽ: "+plan.reason);
            }
            if(takeover) Check(plan.state==control::PlannerState::BAM_LAN && plan.reason=="BAM_LAN","Đủ điều kiện phải bám làn bình thường: "+plan.reason);
            if(fault) {
                Check(plan.cmd.cms==0 && plan.reason=="CAM_BIEN_CHO","Lỗi thoáng qua phải dừng giữ rẽ");
                r.now+=251; plan=r.step(r.in.odo_cm);
                Check(plan.state==control::PlannerState::BAM_LAN && plan.reason=="HUY_RE_CAM_BIEN","Lỗi kéo dài phải hủy: "+plan.reason);
            }
        }
        return;
    }

    // Các ca C kiểm luật giao lộ bằng đầu vào công khai.
    if(name=="real_stays" || name=="edge_always" || name=="real_opens" || name=="t_end" || name=="visible_anchor" || name=="round_shrinks") {
        if(name=="round_shrinks") {
            Rig r;
            for(int i=0;i<r.p.vote_window+2;++i) r.step(0,"round_barrier");
            for(int i=0;i<r.p.vote_window+8;++i) {
                r.now+=40; r.in.image_ms=r.in.mask_ms=r.in.sensor_ms=r.now;
                r.in.detections={{{"round_barrier",cv::Rect2f(540,100,200,80),.5f,r.now}},r.now,++r.frame};
                const auto plan=r.planner.Step(r.in,r.now,r.p);
                Check(plan.mode==control::TrajMode::ROUNDABOUT && plan.reason!="VX_THOAT","Hộp nhỏ vẫn giữ lần qua đã mở");
            }
            return;
        }
        for(int end:(name=="t_end"?std::vector<int>{72,60}:std::vector<int>{72})) {
            Rig r;
            if(name=="edge_always") {
                r.planner.Reset(false); r.mask(0); r.in.mask.colRange(0,61).setTo(1);
                r.in.go=true; r.step(0); r.in.go=false;
            }
            r.arm();
            if(name=="visible_anchor") {
                for(int cm=1;cm<=2*r.p.turn_arm_cm;++cm) {
                    const auto plan=r.step(cm,"left");
                    Check(plan.state==control::PlannerState::BAM_LAN && plan.reason!="HUY_RE","Biển còn thấy, chưa giao lộ không hủy hoặc rẽ");
                }
                const double last=r.in.odo_cm;
                bool cancelled=false;
                for(int cm=1;cm<=r.p.turn_arm_cm+2;++cm) {
                    const auto plan=r.step(last+cm);
                    cancelled=cancelled || plan.reason=="HUY_RE";
                    Check(plan.state==control::PlannerState::BAM_LAN,"Mất biển không giao lộ không rẽ");
                }
                Check(cancelled,"Mất biển quá turn_arm_cm phải hủy");
                r.mask(2);
                for(int i=0;i<r.p.vote_window;++i) Check(r.step(r.in.odo_cm).state==control::PlannerState::BAM_LAN,"Đã hủy không rẽ muộn");
                return;
            }
            if(name=="real_opens") r.mask(1);
            if(name=="t_end") { r.mask(0); r.in.mask.rowRange(0,end).setTo(0); r.in.mask.rowRange(end,end+7).setTo(1); }
            const bool turns=name=="real_opens" || (name=="t_end" && end==72);
            for(int i=1;i<=r.p.vote_window;++i) {
                const auto plan=r.step(r.anchor+((name=="real_stays" || name=="edge_always")?r.p.turn_start_cm+1:0)+i);
                if(turns && i==r.p.vote_min) Turn(plan,"RE_GIAO_LO");
                if(!turns || i<r.p.vote_min) Check(plan.state==control::PlannerState::BAM_LAN,"Chưa đủ bằng chứng giao lộ: "+plan.reason);
            }
        }
        return;
    }
    // T80t C(1–4): đầu vào và kỳ vọng lấy từ luật được duyệt, chỉ dùng Step công khai.
    if(name=="barrier_width" || name=="round_width") {
        const bool round=name=="round_width";
        for(const int width:(round?std::vector<int>{200,300}:std::vector<int>{60,130})) {
            Rig r; bool activated=false;
            for(int i=0;i<r.p.vote_window+12;++i) {
                r.now+=40; r.in.image_ms=r.in.mask_ms=r.in.sensor_ms=r.now;
                r.in.detections={{{round?"round_barrier":"work_barrier",
                    cv::Rect2f(640-width/2.f,432-60,width,60),.9f,r.now}},r.now,++r.frame};
                const auto plan=r.planner.Step(r.in,r.now,r.p);
                const bool active=round ? plan.reason.find("VX_")!=std::string::npos :
                    plan.state==control::PlannerState::RE_TRAI;
                if(width==(round?200:60)) Check(!active,"Hộp hẹp không được kích hoạt: "+plan.reason);
                activated=activated || active;
            }
            if(!round) {
                Check(!activated,"Rào chưa giao lộ không rẽ");
                r.mask(2);
                for(int i=0;i<r.p.vote_min;++i) activated=activated || r.step(0).state==control::PlannerState::RE_TRAI;
            }
            if(width==(round?300:130)) Check(activated,"Hộp đủ rộng phải kích hoạt");
        }
        return;
    }
    if(name=="delayed_batches") {
        for(const int age:{1000,500}) {
            Rig r; r.p.det_max_age_ms=age;
            for(int i=0;i<3;++i) {
                r.now+=230; r.in.image_ms=r.in.mask_ms=r.in.sensor_ms=r.now;
                const auto captured=r.now-360;
                r.in.detections={{{"left",cv::Rect2f(600,100,80,80),.9f,captured}},captured,++r.frame};
                const auto plan=r.planner.Step(r.in,r.now,r.p);
                Check(plan.state==control::PlannerState::BAM_LAN,"Chưa đi quãng không rẽ");
            }
            r.mask(2);
            control::Plan plan; for(int i=0;i<r.p.vote_min;++i) plan=r.step(r.in.odo_cm);
            const bool left=plan.state==control::PlannerState::RE_TRAI;
            std::cout<<"det_max_age_ms="<<age<<" state="<<control::StateName(plan.state)
                     <<" reason="<<plan.reason<<'\n';
            if(age==1000) Check(left,"Ba lô trễ còn tươi phải chốt ý định trái");
        }
        return;
    }
    if(name=="space_abandons_pass") {
        Rig r;
        for(int i=0;i<r.p.vote_window;++i) r.step(0,"round_barrier");
        Check(r.step(0,"round_barrier").mode==control::TrajMode::ROUNDABOUT,
              "Thiết lập phải đang trong lượt vòng xuyến trước Space");
        r.in.stop=true; const auto stopped=r.step(0,"round_barrier"); r.in.stop=false;
        Check(stopped.cmd.cms==0 && stopped.cmd.deg==0,"Space phải dừng");
        // G nhận lô rỗng: không bắt đầu lượt mới sau G.
        r.in.go=true; r.step(0); r.in.go=false;
        for(int cm=0;cm<=r.p.rb_lost_cm+r.p.rb_turn1_cm+20;++cm) {
            const auto plan=r.step(cm);
            Check(plan.state!=control::PlannerState::RE_TRAI,"Lượt dở không được thành rẽ chờ tại cm="+
                  std::to_string(cm)+": "+plan.reason);
        }
        return;
    }
    for(int variant=0;variant<(name=="R2" || name=="R6" || name=="R7"?4:2);++variant) {
        const bool right=variant%2!=0;
        Rig r; r.arm(right);
        if(name=="R1" || name=="R2") {
            for(int cm=1;cm<=r.p.turn_start_cm;++cm) {
                const bool visible=name=="R1" || cm<5 || cm>12;
                // Biển ngược xuất hiện lại cũng không đổi hướng đã chốt.
                const std::string sign=visible ? (name=="R2" && variant>=2 && cm>12 ? (right?"left":"right") : (right?"right":"left")) : "";
                const auto plan=r.step(r.anchor+cm,sign);
                if(cm<r.p.turn_start_cm) Check(plan.state==control::PlannerState::BAM_LAN,"Rẽ sớm");
                else Check(plan.state==control::PlannerState::BAM_LAN,"Chưa giao lộ");
            }
            r.mask(1,right);
            control::Plan first; for(int i=0;i<r.p.vote_min;++i) first=r.step(r.in.odo_cm);
            Turn(first,"RE_GIAO_LO",right);
            const auto after=r.step(r.in.odo_cm+1);
            Check(after.reason==(right?"RE_PHAI":"RE_TRAI"),"Khung sau giữ lý do máy rẽ");
        } else if(name=="R6" && variant>=2) {
            // Có phiếu giao lộ từ trước; cả phiếu cũ và mask đảo rộng đều không được làm rẽ sớm trong vòng xuyến.
            for(int i=0;i<r.p.vote_window;++i) r.step(r.anchor,"round_barrier");
            r.mask(2);
            for(int cm=1;cm<=r.p.vote_min;++cm) r.step(r.anchor+cm,"round_barrier");
            r.mask(5);
            for(int cm=r.p.vote_min+1;cm<r.p.turn_start_cm;++cm) {
                const auto plan=r.step(r.anchor+cm,cm<10?"round_barrier":"");
                Check(plan.state==control::PlannerState::BAM_LAN && plan.reason!="RE_GIAO_LO",
                      "Đảo rộng trong vòng xuyến không được kích hoạt RE_GIAO_LO");
                if(cm>=2*r.p.vote_min)
                    Check(plan.mode==control::TrajMode::ROUNDABOUT,"Phải còn trong lần qua vòng xuyến");
                if(cm>=15) Check(plan.reason=="VX_THOAT","Phải kiểm cả lúc mất phiếu nhưng chưa hết lần qua");
            }
        } else if(name=="R3" || name=="R4" || name=="R6") {
            if(name=="R3") for(int cm=1;cm<=15;++cm)
                Check(r.step(r.anchor+cm).state==control::PlannerState::BAM_LAN,"Chưa có giao lộ không rẽ sớm");
            r.mask(name=="R6"?2:1,right);
            const int start=name=="R3"?16:1;
            const int end=start+r.p.vote_min-1;
            for(int cm=start;cm<=end;++cm) {
                const auto plan=r.step(r.anchor+cm);
                if(cm<end) Check(plan.state==control::PlannerState::BAM_LAN,"Chưa đủ phiếu/nửa quãng không rẽ");
                else Turn(plan,"RE_GIAO_LO",right);
            }
        } else if(name=="R5") {
            r.mask(1,right); r.in.sensor_ok=false;
            for(int cm=1;cm<=35;++cm) Check(r.step(r.anchor+cm).state==control::PlannerState::BAM_LAN,"Cảm biến hỏng không được cua");
            r.in.sensor_ok=true;
            Check(r.step(r.anchor+36).state==control::PlannerState::BAM_LAN,"Lỗi chưa hồi không rẽ lại");
        } else if(name=="R7") {
            r.mask(variant>=2?4:3,right);
            const auto corridor=control::ExtractCorridor(r.in.mask,r.p,control::TrajMode::LANE);
            int jumps=0, near_open=0, far_open=0;
            for(size_t i=0;i<corridor.rows.size();++i) {
                const auto &row=corridor.rows[i];
                if(row.valid && row.flag_left=="NHAY" && row.flag_right=="NHAY") ++jumps;
                if(row.valid && (right ? row.flag_right!="THAT" && row.flag_left=="THAT" : row.flag_left!="THAT" && row.flag_right=="THAT"))
                    ++(i<size_t(r.p.corridor_rows/2) ? near_open : far_open);
            }
            if(variant<2) Check(jumps>=2,"Fixture cong phải có hai hàng hai biên NHAY");
            else Check(near_open==0 && far_open>=2,"Cua cùng chiều biển phải chỉ mở phía rẽ ở nửa xa");
            for(int cm=1;cm<=r.p.turn_start_cm;++cm) {
                const auto plan=r.step(r.anchor+cm);
                if(cm<r.p.turn_start_cm) Check(plan.state==control::PlannerState::BAM_LAN,"Cong không phải giao lộ");
                else Check(plan.state==control::PlannerState::BAM_LAN,"Chưa giao lộ");
            }
            r.mask(0); r.step(r.in.odo_cm); r.mask(1,right);
            control::Plan first; for(int i=0;i<r.p.vote_min;++i) first=r.step(r.in.odo_cm);
            Turn(first,"RE_GIAO_LO",right);
        } else throw std::runtime_error("Ca lạ");
    }
}
}
int main(int argc,char **argv) {
    try { Check(argc==2,"Cần tên ca"); Run(argv[1]); std::cout<<"PASS "<<argv[1]<<'\n'; return 0; }
    catch(const std::exception &e) { std::cerr<<"FAIL: "<<e.what()<<'\n'; return 1; }
}
