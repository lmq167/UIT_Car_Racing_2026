#include "control/Planner.h"
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void Check(bool ok,const char* message) { if(!ok) throw std::runtime_error(message); }
struct Rig {
    control::Planner planner;
    control::ControlParams p;
    control::PlannerInput in;
    int64_t now=0;
    uint64_t id=0;
    Rig() {
        in.yolo_enabled=true;
        in.mask=cv::Mat(60,200,CV_8UC1,cv::Scalar(0));
        for(int y=0;y<60;++y) { in.mask.at<unsigned char>(y,60)=2; in.mask.at<unsigned char>(y,140)=2; }
        p.lane_width_frac={0.4,0.4,0.4}; p.kd=0;
    }
    control::Plan step() {
        in.mask_ms=now; in.image_ms=now;
        return planner.Step(in,now,p);
    }
    control::Plan go() { in.go=true; auto result=step(); in.go=false; return result; }
    control::Detection light(const std::string& name,float right=1152,float confidence=0.5f) {
        return {name,cv::Rect2f(right-128,100,128,72),confidence,now+40};
    }
    control::Plan send(std::vector<control::Detection> items) {
        now+=40;
        for(auto& d:items) d.captured_ms=now;
        in.detections={std::move(items),now,++id};
        return step();
    }
    control::Plan green() { return send({light("green")}); }
    control::Plan red(float right=1152) { return send({light("red",right)}); }
    void moving() { p.start_wait_green=0; Check(go().cmd.cms>0,"start_wait_green=0 phải chạy"); }
    void latch() { moving(); red(); red(); Check(red().reason=="DUNG_DEN_DO","Đỏ đủ phiếu phải chốt"); }
};
void Unit() {
    {
        Rig r; Check(r.go().reason=="CHO_DEN_XUAT_PHAT","G phải chờ xanh");
        for(int i=0;i<4;++i) Check(r.send({}).cmd.cms==0,"G không đèn phải đứng");
        Check(r.green().cmd.cms==0,"Xanh 1/3");
        for(int i=0;i<10;++i) { ++r.now; Check(r.step().cmd.cms==0,"Đọc cùng lô không thêm phiếu/chuỗi"); }
        Check(r.green().cmd.cms==0,"Xanh N-1");
        const auto plan=r.green(); Check(plan.cmd.cms>0 && plan.reason=="DEN_XANH","Xanh N mới chạy");
    }
    {
        Rig r; r.p.vote_min=1; r.go(); r.green(); r.green();
        r.now+=r.p.det_max_age_ms+1;
        Check(r.green().cmd.cms==0,"Khoảng trống quá tuổi ngắt chuỗi ngay cả khi chưa đọc lô cũ");
        Check(r.green().cmd.cms==0 && r.green().cmd.cms>0,"Sau khoảng trống đếm lại ba lô");
    }
    {
        Rig r; r.now=100; r.p.vote_min=1; r.go();
        for(int i=0;i<3;++i) {
            r.now+=40; auto d=r.light("green"); d.captured_ms=90+i;
            r.in.detections={{d},d.captured_ms,++r.id};
            Check(r.step().cmd.cms==0,"Ảnh xanh chụp trước G, Infer trả sau G không được tính chuỗi");
        }
        Check(r.green().cmd.cms==0 && r.green().cmd.cms==0,"Chỉ đếm lô chụp sau khi chờ");
        Check(r.green().cmd.cms>0,"Ba lô xanh chụp sau G mới mở");
    }
    for(int interrupt=0;interrupt<4;++interrupt) {
        Rig r; r.go(); r.green(); r.green();
        if(interrupt==0) r.send({r.light("off")});
        if(interrupt==1) r.send({});
        if(interrupt==2) { r.now+=r.p.det_max_age_ms+1; Check(r.step().cmd.cms==0,"Lô cũ phải đứng"); }
        if(interrupt==3) r.red(800);
        Check(r.green().cmd.cms==0 && r.green().cmd.cms==0,"Ngắt chuỗi phải đếm lại N lô");
        Check(r.green().cmd.cms>0,"Sau ngắt đủ N mới chạy");
    }
    {
        Rig r; r.moving();
        Check(r.red(800).cmd.cms>5,"Đỏ 1 phiếu chưa giảm"); r.red(900);
        auto plan=r.red(1000); Check(plan.reason=="TIEP_CAN_DEN_DO" && plan.cmd.cms==5,"Đỏ giữa ảnh đủ phiếu giảm tốc");
        Check(r.send({}).cmd.cms==5,"Lô rỗng không xoá phiếu/hộp đỏ còn tươi");
        Check(r.red(1151).cmd.cms==5,"Ngay dưới 0.90 chưa dừng");
        plan=r.red(1152); Check(plan.cmd.cms==0 && plan.cmd.deg==0 && plan.reason=="DUNG_DEN_DO","Đúng 0.90 dừng ngay");
    }
    {
        Rig r; r.p.start_wait_green=0;
        r.p.max_speed_cms=r.p.cruise_cms=r.p.min_cms=r.p.lost_cms=0; r.p.approach_cms=5;
        Check(r.go().state==control::PlannerState::BAM_LAN,"Trần tốc độ 0 vẫn vào bám làn");
        r.red(1000); r.red(1000);
        const auto plan=r.red(1000);
        Check(plan.reason=="TIEP_CAN_DEN_DO" && plan.cmd.cms==0 && plan.debug.speed_cms==0,
              "Tiếp cận đỏ phải giữ tốc độ 0 khi max_speed_cms=0 và approach_cms=5");
    }
    {
        Rig r; r.latch();
        Check(r.send({}).cmd.cms==0,"Mất đèn giữ chốt đỏ");
        Check(r.send({r.light("off")}).cmd.cms==0,"Off giữ chốt đỏ");
        r.now+=r.p.det_max_age_ms+1; Check(r.step().cmd.cms==0,"Cũ giữ chốt đỏ");
        r.in.stop=true; Check(r.step().cmd.cms==0,"Space dừng"); r.in.stop=false;
        Check(r.go().reason=="DUNG_DEN_DO","Space/G không xoá chốt đỏ");
        Check(r.go().cmd.cms==0,"G lặp không vượt đèn");
        Check(r.green().cmd.cms==0 && r.green().cmd.cms==0,"Chốt đỏ chờ N-1 xanh");
        Check(r.green().cmd.cms>0,"Chốt đỏ đủ N xanh mới mở");
    }
    {
        Rig r; r.moving();
        for(int i=0;i<3;++i) Check(r.send({r.light("red"),r.light("green")}).cmd.cms==0 || i<2,"Đỏ thắng xanh đồng thời");
        Check(r.step().reason=="DUNG_DEN_DO","Xanh đồng thời không mở đỏ");
    }
    {
        Rig r; r.p.vote_window=3; r.moving();
        r.red(); r.red(); r.send({});
        Check(r.red().cmd.cms>0 && r.red().cmd.cms>0,"Phiếu ngoài cửa sổ không được cộng");
        Check(r.red().cmd.cms==0,"Đủ vote_min trong cửa sổ mới dừng");
    }
    {
        Rig r; r.p.vote_window=6; r.latch();
        r.green(); r.green(); Check(r.green().cmd.cms>0,"Xanh mở với cửa sổ 6");
        Check(r.step().cmd.cms>0,"Phiếu đỏ cũ không tái chốt sau xanh");
    }
    {
        Rig r; r.latch(); r.planner.Reset(true);
        Check(r.go().reason=="DUNG_DEN_DO","Epoch nhanh Space/G giữ chốt");
        r.green(); r.green(); Check(r.green().cmd.cms>0,"Epoch giữ đèn vẫn mở khi đủ xanh mới");
    }
    {
        Rig r; r.moving();
        Check(r.send({r.light("red"),r.light("red"),r.light("red")}).cmd.cms>0,"Mỗi lớp tối đa một phiếu mỗi lô");
        Check(r.red().cmd.cms>0,"Hai lô đỏ chưa đủ ba phiếu");
        Check(r.red().cmd.cms==0,"Ba lô đỏ mới đủ phiếu");
    }
    for(const std::string name:{"red","green","off"}) {
        Rig r; r.go();
        r.p.red_box_min_area_frac=r.p.green_box_min_area_frac=r.p.off_box_min_area_frac=0.01;
        for(int i=0;i<3;++i) r.send({r.light(name,1152,0.499f)});
        Check(r.step().cmd.cms==0,"Dưới conf_min không được mở xuất phát");
        auto tooSmall=r.light(name); tooSmall.box.height=71;
        for(int i=0;i<3;++i) r.send({tooSmall});
        Check(r.step().cmd.cms==0,"Dưới diện tích không được mở xuất phát");
        auto plan=r.send({r.light(name)}); plan=r.send({r.light(name)}); plan=r.send({r.light(name)});
        Check(name=="green" ? plan.cmd.cms>0 : plan.cmd.cms==0,"Conf/diện tích đúng biên");
        if(name=="red") Check(plan.reason=="DUNG_DEN_DO","Đỏ đúng biên phải chốt");
        if(name=="off") Check(plan.reason=="CHO_DEN_XUAT_PHAT","Off đúng biên vẫn chờ");
    }
    {
        Rig r; r.go();
        auto future=r.light("green");
        for(int i=0;i<3;++i) {
            r.now+=40; future.captured_ms=r.now+1;
            r.in.detections={{future},r.now+1,++r.id}; r.step();
        }
        // Mỗi lô tương lai đã ngắt chuỗi; chỉ lô xanh tươi mới được đếm lại.
        Check(r.green().cmd.cms==0,"Tương lai ngắt chuỗi");
    }
    {
        Rig r; r.moving();
        auto small=r.light("red"); small.box.height=0.01f;
        for(int i=0;i<3;++i) r.send({small,r.light("red",1152,0.499f)});
        Check(r.step().cmd.cms>5,"Đỏ dưới conf/diện tích không có phiếu");
        r.red(); r.red(); Check(r.red().cmd.cms==0,"Đỏ đúng conf và diện tích mặc định");
    }
    {
        Rig r; r.in.yolo_enabled=false;
        Check(r.go().cmd.cms>0,"no-yolo không chờ xanh dù mặc định bằng 1");
        for(int i=0;i<4;++i) Check(r.red().cmd.cms>0,"no-yolo bỏ luật đỏ");
        r.in.stop=true; r.step(); r.in.stop=false; r.p.start_wait_green=1;
        Check(r.go().cmd.cms>0,"no-yolo sau nạp tham số vẫn bỏ luật đèn");
    }
}
}
int main() {
    try { Unit(); std::cout<<"ĐẠT: Planner P2\n"; return 0; }
    catch(const std::exception& e) { std::cerr<<"KHÔNG ĐẠT: "<<e.what()<<'\n'; return 1; }
}
