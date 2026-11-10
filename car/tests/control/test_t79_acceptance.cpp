#include "control/Planner.h"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>

// Nguồn kỳ vọng: T79_design.md, T79_corridor_lane.md (Test/Giữ nguyên tuyệt đối).
// Không đọc mã triển khai. Các giả định giao diện nằm trong T79_acc_cases.md.
namespace {
using namespace control;
constexpr int kH=192, kW=320, kTick=50;
constexpr double kEps=1e-8;

void Check(bool ok, const char *why) {
    if(!ok) throw std::runtime_error(why);
}

ControlParams Params() {
    ControlParams p;
    p.corridor_rows=8;
    p.corridor_bottom_frac=0.95;
    p.corridor_top_frac=0.62;
    p.lane1_width_frac={0.57,0.17};
    p.two_lane_ratio=1.5;
    p.max_jump_frac=0.08;
    p.conf_min=0.4;
    p.lookahead_frac=0.75;
    p.avoid_min_width_ratio=0.75;
    p.obstacle_hole_ratio=0.25;
    p.center_offset_frac=0;
    return p;
}

void Limits(const Command &cmd, const ControlParams &p) {
    Check(std::isfinite(cmd.cms) && std::isfinite(cmd.deg),"Lệnh chứa NaN/vô cực");
    Check(cmd.cms>=0 && cmd.cms<=std::min(p.max_speed_cms,20.0)+kEps,
          "Lệnh vượt trần tốc độ hoặc chạy lùi");
    Check(std::abs(cmd.deg)<=std::min(p.steer_max_deg,40.0)+kEps,
          "Lệnh vượt trần góc lái");
}

void Zero(const Command &cmd) {
    Check(std::abs(cmd.cms)<=kEps && std::abs(cmd.deg)<=kEps,"Phải dừng cả tốc độ và góc");
}

int Pixel(double value) { return static_cast<int>(std::lround(value)); }
int Bottom() { return Pixel(0.95*kH); }
int Top() { return Pixel(0.62*kH); }
double Width(int y) {
    return kW*(0.17+(0.57-0.17)*(y-Top())/(Bottom()-Top()));
}

cv::Mat Road(double scale=1.0, double shift=0, int value=1) {
    cv::Mat mask(kH,kW,CV_8UC1,cv::Scalar(0));
    const double axis=kW*(0.5+shift);
    const int left=Pixel(axis-scale*Width(Bottom())/2);
    const int right=Pixel(axis+scale*Width(Bottom())/2);
    const cv::Point points[]={
        {Pixel(axis-scale*Width(Top())/2),Top()},
        {Pixel(axis+scale*Width(Top())/2),Top()},
        {right,Bottom()},{left,Bottom()}};
    cv::fillConvexPoly(mask,points,4,cv::Scalar(value),cv::LINE_8);
    cv::rectangle(mask,cv::Point(left,Bottom()),cv::Point(right,kH-1),cv::Scalar(value),cv::FILLED);
    return mask;
}

// Dịch ngang mask (pixel), phần lộ ra là 0: tạo hành lang nhảy trái-phải.
cv::Mat Shift(const cv::Mat &mask, double dx) {
    cv::Mat out;
    const cv::Mat move=(cv::Mat_<double>(2,3) << 1,0,dx,0,1,0);
    cv::warpAffine(mask,out,move,mask.size(),cv::INTER_NEAREST,cv::BORDER_CONSTANT,cv::Scalar(0));
    return out;
}

cv::Mat Obstacle(double road_scale=1.7, double hole_scale=0.4) {
    cv::Mat mask=Road(road_scale);
    // Hàng gần nhất liền để nối hai nhánh; lỗ cắt ba hàng còn lại của nửa gần.
    const int end=Bottom()-7;
    const cv::Point hole[]={
        {Pixel(kW/2.0-hole_scale*Width(Top())/2),Top()},
        {Pixel(kW/2.0+hole_scale*Width(Top())/2),Top()},
        {Pixel(kW/2.0+hole_scale*Width(end)/2),end},
        {Pixel(kW/2.0-hole_scale*Width(end)/2),end}};
    cv::fillConvexPoly(mask,hole,4,cv::Scalar(0),cv::LINE_8);
    return mask;
}

cv::Mat Island() {
    cv::Mat mask(kH,kW,CV_8UC1,cv::Scalar(0));
    const cv::Point road[]={{40,Top()},{280,Top()},{315,Bottom()},{5,Bottom()}};
    cv::fillConvexPoly(mask,road,4,cv::Scalar(1),cv::LINE_8);
    cv::rectangle(mask,cv::Point(5,Bottom()),cv::Point(315,kH-1),cv::Scalar(1),cv::FILLED);
    cv::rectangle(mask,cv::Point(130,Top()),cv::Point(190,Bottom()-7),cv::Scalar(0),cv::FILLED);
    return mask;
}

bool OnRoad(const cv::Mat &mask, double x, double y) {
    if(!std::isfinite(x) || !std::isfinite(y) || x<0 || x>kW-1 || y<0 || y>kH-1)
        return false;
    return mask.at<unsigned char>(Pixel(y),Pixel(x))==1;
}

void NoIsland(const ControlDebug &debug, const cv::Mat &mask, const ControlParams &p) {
    Check(static_cast<int>(debug.rows.size())==p.corridor_rows,"Thiếu hàng debug vòng xuyến");
    int valid=0, split=0;
    for(const auto &row:debug.rows) {
        if(!row.valid) continue;
        ++valid;
        Check(OnRoad(mask,row.target,row.y),"Target hàng đi qua đảo hoặc ra ngoài đường");
        if(row.y<=Bottom()-7) {
            ++split;
            Check(row.target>190,"Vòng xuyến không chọn nhánh phải của đảo");
        }
    }
    Check(valid>=std::max(2,p.min_valid_rows) && split>=2,"Fixture vòng xuyến không đủ hàng tách");
    Check(std::abs(debug.look_y-p.lookahead_frac*kH)<=1.0,"Sai tọa độ y lookahead");
    Check(OnRoad(mask,debug.look_x,debug.look_y) && debug.look_x>190,
          "Đường khớp tại lookahead cắt đảo");
}

struct LaneRig {
    ControlParams p=Params();
    LaneController lane;
    int64_t now=1000, mask_ms=1000;
    ControlResult Step(const cv::Mat &mask, TrajMode mode=TrajMode::LANE, bool fresh=true) {
        now+=kTick;
        if(fresh) mask_ms=now;
        const auto result=lane.Step(mask,mask_ms,now,p,mode);
        Limits(result.cmd,p);
        Check(std::isfinite(result.debug.e),"Sai số làn không hữu hạn");
        return result;
    }
    ControlResult Steady(const cv::Mat &mask, TrajMode mode=TrajMode::LANE) {
        ControlResult result;
        for(int n=0;n<p.regain_frames+12;++n) result=Step(mask,mode);
        Check(result.debug.state=="BAM_LAN","Fixture không vào BAM_LAN");
        return result;
    }
};

// A01 — §3–4: một làn thẳng cho sai số và góc gần 0.
void Straight() {
    LaneRig r;
    const auto result=r.Steady(Road());
    Check(std::abs(result.debug.e)<=0.03 && std::abs(result.cmd.deg)<=1.0,"Làn thẳng bị lái lệch");
    Check(result.cmd.cms>0,"Làn thẳng không chạy tiến");
}

// A02 — §4: đích bên phải -> e dương -> góc âm.
void RightOffset() {
    LaneRig r;
    const auto result=r.Steady(Road(1.0,0.10));
    Check(result.debug.e>0 && result.cmd.deg<0,"Sai dấu e/góc khi làn lệch phải");
}

// A03 — §3: hai làn luôn nhắm nửa phải hành lang.
void TwoLanes() {
    LaneRig r;
    const auto result=r.Steady(Road(1.7));
    Check(!result.debug.rows.empty(),"Không có debug hành lang");
    const auto *nearest=&result.debug.rows.front();
    double distance=1e9;
    int two=0;
    for(const auto &row:result.debug.rows) {
        if(!row.valid) continue;
        if(std::abs(row.y-result.debug.look_y)<distance) {
            nearest=&row;
            distance=std::abs(row.y-result.debug.look_y);
        }
        if(row.lanes==2) {
            ++two;
            Check(row.target>(row.left+row.right)/2,"Hai làn chọn target nửa trái");
        }
    }
    Check(two>=2 && nearest->lanes==2,"Fixture không được nhận là hai làn");
    Check(result.debug.look_x>(nearest->left+nearest->right)/2,"Lookahead không ở làn phải");
}

// A04 — §3: phần loe xa không kéo đích khỏi hành lang/trục gần.
void Junction() {
    LaneRig r;
    r.p.center_offset_frac=-0.10;
    auto mask=Road(1.0,-0.10);
    cv::rectangle(mask,cv::Point(0,Top()),cv::Point(kW-1,138),cv::Scalar(1),cv::FILLED);
    const auto result=r.Steady(mask);
    Check(std::abs(result.debug.e)<=0.10,"Ngã T kéo đích về tâm phần loe");
}

// A05/A26 — §3: một biên bị cắt vẫn có hành lang, cờ MEP_ANH và dấu e đúng.
void ImageEdge(bool right) {
    LaneRig r;
    const auto result=r.Steady(Road(1.0,right ? 0.30 : -0.30));
    bool edge=false;
    int valid=0;
    for(const auto &row:result.debug.rows) {
        if(!row.valid) continue;
        ++valid;
        // §4: flag_* là std::string "THAT"|"MEP_ANH"|"NHAY".
        if((right ? row.flag_right : row.flag_left)=="MEP_ANH") edge=true;
    }
    Check(edge && valid>=std::max(2,r.p.min_valid_rows),"Mất cờ MEP_ANH hoặc mất hành lang");
    const double e=right ? result.debug.e : -result.debug.e;
    Check(e>0 && e<=0.75,"Sai số biên chạm mép không hợp lý");
}

// A06 — §1/3/4: không lấp lỗ nhưng vẫn bám làn qua lỗ nhỏ.
void SmallHole() {
    LaneRig r;
    auto mask=Road();
    cv::rectangle(mask,cv::Point(156,130),cv::Point(164,170),cv::Scalar(0),cv::FILLED);
    const auto result=r.Steady(mask);
    bool hole=false;
    for(const auto &row:result.debug.rows) if(row.valid && row.hole) hole=true;
    Check(hole && result.cmd.cms>0,"Lỗ nhỏ làm mất hành lang hoặc bị xóa khỏi debug");
}

// A07 — §3: vùng trắng xa trục không được thay hành lang nối từ hàng gần.
void DetachedRoad() {
    LaneRig r;
    const auto road=Road();
    auto mask=road.clone();
    cv::rectangle(mask,cv::Point(280,Top()),cv::Point(kW-1,kH-1),cv::Scalar(1),cv::FILLED);
    const auto result=r.Steady(mask);
    Check(std::abs(result.debug.e)<=0.05,"Chọn vùng trắng xa trục");
    Check(OnRoad(road,result.debug.look_x,result.debug.look_y),"Lookahead nhảy sang vùng tách rời");
    for(const auto &row:result.debug.rows)
        if(row.valid) Check(OnRoad(road,row.target,row.y),"Target thuộc vùng tách rời");
}

// A08 — §4: tuổi < lost_hold_ms: GIU_GOC/MAT_LAN_GIU giữ góc; lost_hold_ms..lost_stop_ms:
// GIU_GOC/MAT_LAN_DUNG cms 0; từ lost_stop_ms: MAT_LAN/MAT_LAN 0,0. Mỗi mốc cho trễ tối đa một nhịp 50 ms.
void LostLane() {
    LaneRig r;
    const auto before=r.Steady(Road(1.0,0.10));
    const int64_t valid_ms=r.now, hold=r.p.lost_hold_ms, stop=r.p.lost_stop_ms;
    Check(hold>=2*kTick && stop-hold>=3*kTick,"Không đủ cửa sổ giữ/dừng để kiểm từng mốc");
    const cv::Mat black(kH,kW,CV_8UC1,cv::Scalar(0));
    bool held=false, braking=false, lost=false;
    for(int n=0;n<stop/kTick+4;++n) {
        const auto result=r.Step(black);
        const auto age=r.now-valid_ms;
        const auto &state=result.debug.state, &reason=result.debug.reason;
        const bool is_hold=state=="GIU_GOC" && reason=="MAT_LAN_GIU";
        const bool is_brake=state=="GIU_GOC" && reason=="MAT_LAN_DUNG";
        const bool is_lost=state=="MAT_LAN" && reason=="MAT_LAN";
        if(is_hold) Check(std::abs(result.cmd.deg-before.cmd.deg)<=kEps,"Mất làn ngắn làm đổi góc giữ");
        if(is_brake) Check(std::abs(result.cmd.cms)<=kEps,"MAT_LAN_DUNG vẫn có tốc độ");
        if(is_lost) Zero(result.cmd);
        if(age<hold) { held=true; Check(is_hold,"Trước lost_hold_ms phải GIU_GOC/MAT_LAN_GIU"); }
        else if(age<=hold+kTick) Check(is_hold || is_brake,"Quanh lost_hold_ms sai trạng thái mất làn");
        else if(age<stop) { braking=true; Check(is_brake,"lost_hold_ms..lost_stop_ms phải GIU_GOC/MAT_LAN_DUNG"); }
        else if(age<=stop+kTick) Check(is_brake || is_lost,"Quanh lost_stop_ms sai trạng thái mất làn");
        else { lost=true; Check(is_lost,"Quá lost_stop_ms phải MAT_LAN/MAT_LAN"); }
    }
    Check(held && braking && lost,"Chưa đi qua đủ mốc mất làn");
}

// A09 — §4: mask quá tuổi phải MAT_NAN_CU và lệnh 0,0.
void StaleMask() {
    LaneRig r;
    const auto mask=Road();
    r.Steady(mask);
    ControlResult result;
    do { result=r.Step(mask,TrajMode::LANE,false); }
    while(r.now-r.mask_ms<=r.p.mask_max_age_ms);
    Check(result.debug.reason=="MAT_NAN_CU","Không nhận mask quá tuổi");
    Zero(result.cmd);
}

// A10 — §4: khung hợp lệ, cùng chế độ, giới hạn nhảy tính theo W.
void JumpLimit() {
    LaneRig r;
    auto previous=r.Steady(Road(1.0,-0.10));
    const double first=previous.debug.look_x;
    for(int n=0;n<12;++n) {
        const auto next=r.Step(Road(1.0,0.10));
        Check(next.debug.state=="BAM_LAN","Fixture nhảy tạo mất làn");
        Check(std::abs(next.debug.look_x-previous.debug.look_x)<=r.p.max_jump_frac*kW+kEps,
              "Lookahead nhảy vượt max_jump_frac*W");
        previous=next;
    }
    Check(previous.debug.look_x>first+0.10*kW,"Giới hạn nhảy làm đông cứng đích");
}

// A11 — §1: giá trị > 2 (ở đây 3), kể cả ngoài hàng quét, phải ném runtime_error.
void InvalidMask() {
    LaneRig r;
    auto mask=Road();
    cv::rectangle(mask,cv::Point(0,0),cv::Point(0,0),cv::Scalar(3),cv::FILLED);
    bool rejected=false;
    ControlResult result;
    r.now+=kTick;
    try { result=r.lane.Step(mask,r.now,r.now,r.p); }
    catch(const std::runtime_error &) { rejected=true; }
    if(!rejected) Limits(result.cmd,r.p);
    Check(rejected,"Mask có 3 không ném runtime_error");
}

// A11b — §1: giá trị 2 cũng là mặt đường; hành lang vẽ bằng 2 bám như A01.
void RoadValueTwo() {
    LaneRig r;
    const auto result=r.Steady(Road(1.0,0,2));
    Check(std::abs(result.debug.e)<=0.03 && std::abs(result.cmd.deg)<=1.0,"Hành lang giá trị 2 bị lái lệch");
    Check(result.cmd.cms>0,"Hành lang giá trị 2 không chạy tiến");
}

struct Rig {
    ControlParams p=Params();
    Planner planner;
    PlannerInput in;
    Plan last;
    int64_t now=1000;
    uint64_t frame=0;
    bool round_seen=false, obstacle_seen=false;
    Rig() {
        p.start_wait_green=0;
        p.obstacle_action=2;
        in.yolo_enabled=true;
        in.sensor_ok=false;
        in.mask=Road(1.7);
        in.go=true;
        Step();
        in.go=false;
        for(int n=0;n<p.regain_frames+2;++n) Step();
        Check(last.state==PlannerState::BAM_LAN && last.cmd.cms>0,"Planner chưa sẵn sàng bám làn");
    }
    Plan Step(bool fresh_image=true, bool fresh_mask=true) {
        now+=kTick;
        if(fresh_mask) in.mask_ms=now;
        if(fresh_image) in.image_ms=now;
        if(in.sensor_ok) in.sensor_ms=now;
        if(in.yolo_enabled) {
            in.detections.items.clear();
            in.detections.captured_ms=now;
            in.detections.frame_id=++frame;
            if(obstacle_seen) in.detections.items.push_back({"obstacle",cv::Rect2f(600,100,128,100),0.9f,now});
            if(round_seen)
                in.detections.items.push_back({"round_barrier",cv::Rect2f(10,40,300,128),0.9f,now});
        }
        last=planner.Step(in,now,p);
        Limits(last.cmd,p);
        Check(std::isfinite(last.debug.e),"Planner trả sai số không hữu hạn");
        return last;
    }
    Plan Left() {
        in.mask=Obstacle(); obstacle_seen=true;
        for(int n=1;n<=p.vote_min;++n) {
            const auto plan=Step();
            Check((plan.reason=="NE_TRAI")== (n==p.vote_min),"NE_TRAI sai số phiếu YOLO mới");
        }
        Check(p.steer_rate_dps>0,"Cấu hình không cho góc tiến tới đích né");
        const int settle=static_cast<int>(std::ceil(2*p.steer_max_deg/(p.steer_rate_dps*0.05)))+4;
        Check(settle<2000,"Thời gian ổn định góc vượt giới hạn fixture");
        for(int n=0;n<settle;++n) {
            const auto plan=Step();
            Check(plan.state==PlannerState::BAM_LAN && plan.reason=="NE_TRAI",
                  "Không giữ NE_TRAI khi còn vật cản");
            Check(plan.cmd.cms>0 && plan.cmd.cms<=8.0+kEps,"Sai tốc độ né trái");
        }
        Check(last.cmd.deg>0,"Né trái không tạo góc dương sau khi ổn định");
        return last;
    }
    Plan Pass() {
        Left();
        in.mask=Road(1.7); obstacle_seen=false;
        for(int n=1;n<=p.vote_window;++n) {
            const auto plan=Step();
            Check(plan.reason==(n==p.vote_window ? "NE_QUA" : "NE_TRAI"),
                  "NE_QUA không theo đúng chuỗi lô YOLO vắng");
        }
        return last;
    }
    Plan Return() {
        Pass();
        for(int n=0;n<10000;++n) {
            const auto plan=Step();
            if(plan.reason=="VE_PHAI") return plan;
            Check(plan.reason=="NE_QUA","Không tới VE_PHAI theo chuỗi né");
        }
        throw std::runtime_error("Hết hạn giả lập chờ VE_PHAI");
    }
    Plan Round() {
        in.yolo_enabled=true;
        in.sensor_ok=true;
        round_seen=true;
        // Xác nhận vào vòng trước khi đưa lỗ đảo, tránh dùng trạng thái private.
        for(int n=0;n<p.vote_window+p.vote_min;++n) Step();
        Check(last.state==PlannerState::BAM_LAN && last.reason=="VX_QUANH_DAO",
              "YOLO giả chưa đưa Planner vào vòng xuyến");
        in.mask=Island();
        for(int n=0;n<p.vote_window+p.regain_frames+4;++n) {
            const auto plan=Step();
            Check(plan.state==PlannerState::BAM_LAN && plan.reason=="VX_QUANH_DAO",
                  "Planner né/dừng lòng đảo hoặc mất chế độ vòng xuyến");
            NoIsland(plan.debug,in.mask,p);
        }
        return last;
    }
};

// A12 — §5: đủ phiếu hộp giữa hành lang -> NE_TRAI, góc dương.
void AvoidLeft() { Rig r; r.Left(); }

// A13 — §5: sensor_ok=false nên quãng NE_QUA/VE_PHAI tính bằng tích phân lệnh gửi (cms·dt);
// qua đủ quãng rồi về, kết thúc theo e hoặc 5x quãng.
void AvoidSequence() {
    Rig r;
    Check(!r.in.sensor_ok,"A13 phải chạy với sensor_ok=false (quãng theo lệnh gửi)");
    auto plan=r.Pass();
    double distance=0;
    bool returning=false;
    for(int n=0;n<10000;++n) {
        distance+=plan.cmd.cms*kTick/1000.0;
        plan=r.Step();
        if(plan.reason=="VE_PHAI") {
            Check(distance+kEps>=r.p.obstacle_pass_cm,"Về phải trước đủ quãng NE_QUA");
            returning=true;
            break;
        }
        Check(plan.reason=="NE_QUA","Bỏ qua pha VE_PHAI");
        Check(plan.cmd.cms>0 && plan.cmd.cms<=8.0+kEps,"Sai tốc độ NE_QUA");
        Check(distance<=r.p.obstacle_pass_cm+2*r.p.max_speed_cms*0.05+kEps,
              "NE_QUA không kết thúc sau đủ quãng");
    }
    Check(returning,"Hết hạn chờ VE_PHAI");
    distance=0;
    for(int n=0;n<10000;++n) {
        Check(plan.cmd.cms>0 && plan.cmd.cms<=8.0+kEps,"Sai tốc độ VE_PHAI");
        distance+=plan.cmd.cms*kTick/1000.0;
        const double previous_e=plan.debug.e;
        plan=r.Step();
        if(plan.reason=="BAM_LAN") {
            Check(plan.state==PlannerState::BAM_LAN,"Kết thúc né sai trạng thái");
            Check(distance+kEps>=r.p.obstacle_pass_cm,"Kết thúc VE_PHAI trước quãng tối thiểu");
            Check(std::abs(previous_e)<0.12 || distance+kEps>=5*r.p.obstacle_pass_cm,
                  "Kết thúc VE_PHAI khi chưa hội tụ hoặc chưa đủ 5x quãng");
            return;
        }
        Check(plan.reason=="VE_PHAI","Pha về phải bị mất");
        Check(distance<=5*r.p.obstacle_pass_cm+2*r.p.max_speed_cms*0.05+kEps,
              "Không kết thúc VE_PHAI ở cận 5x quãng");
    }
    throw std::runtime_error("Hết hạn chu trình né và về");
}

// A14/A15 — §5: không đủ đường hai phía hoặc nằm ngoài hành lang -> không né/dừng.
void NotObstacle(bool outside) {
    Rig r;
    r.in.mask=Road();
    if(outside) {
        cv::rectangle(r.in.mask,cv::Point(280,Top()),cv::Point(kW-1,kH-1),cv::Scalar(1),cv::FILLED);
        cv::rectangle(r.in.mask,cv::Point(292,132),cv::Point(305,175),cv::Scalar(0),cv::FILLED);
    } else {
        cv::rectangle(r.in.mask,cv::Point(0,Top()),cv::Point(155,175),cv::Scalar(0),cv::FILLED);
    }
    for(int n=0;n<2*r.p.vote_window+r.p.regain_frames+4;++n) {
        const auto plan=r.Step();
        Check(plan.state==PlannerState::BAM_LAN && plan.reason=="BAM_LAN" && plan.cmd.cms>0,
              "Lỗ sát mép/ngoài hành lang làm né hoặc dừng");
    }
}

// A16 — §5: có hàng hole ở nửa gần nhưng không hàng hole nào avoid_ok -> DUNG_VAT_CAN,
// KHONG_DUONG_NE, 0,0 (hàng không lỗ không cứu quyết định).
void NoAvoidSpace() {
    Rig r;
    r.in.mask=Obstacle(1.0,0.6); r.obstacle_seen=true;
    for(int n=1;n<=r.p.vote_min;++n) {
        const auto plan=r.Step();
        if(n<r.p.vote_min)
            Check(plan.state!=PlannerState::DUNG_VAT_CAN,"Dừng không đủ phiếu vật cản");
    }
    Check(r.last.state==PlannerState::DUNG_VAT_CAN && r.last.reason=="KHONG_DUONG_NE",
          "Không đủ dải trái nhưng chưa DUNG_VAT_CAN/KHONG_DUONG_NE");
    Zero(r.last.cmd);
    for(int n=0;n<r.p.vote_window;++n) {
        const auto plan=r.Step();
        Check(plan.state==PlannerState::DUNG_VAT_CAN && plan.reason=="KHONG_DUONG_NE",
              "Còn vật cản không đường né nhưng tự chạy lại");
        Zero(plan.cmd);
    }
}

// A17 — §5: obstacle_action=1 giữ chức năng dừng vật cản.
void StopObstacle() {
    Rig r;
    r.p.obstacle_action=1;
    r.in.mask=Obstacle(); r.obstacle_seen=true;
    for(int n=1;n<=r.p.vote_min;++n) {
        const auto plan=r.Step();
        Check((plan.state==PlannerState::DUNG_VAT_CAN)==(n==r.p.vote_min),"Action 1 dừng sai số phiếu");
    }
    Zero(r.last.cmd);
}

// A18 — §3/4/7: target từng hàng và đường khớp tại y_l đều nằm ngoài đảo.
void RoundLane() {
    LaneRig r;
    const auto mask=Island();
    for(int n=0;n<r.p.regain_frames+12;++n) {
        const auto result=r.Step(mask,TrajMode::ROUNDABOUT);
        if(n>=r.p.regain_frames) {
            Check(result.debug.state=="BAM_LAN","ROUNDABOUT mất hành lang");
            NoIsland(result.debug,mask,r.p);
        }
    }
}

// A19 — §5/7: round_barrier đủ phiếu -> quanh đảo, không nhận đảo làm vật cản.
void RoundPlanner() { Rig r; r.Round(); }

enum class Phase { LEFT, PASS, RETURN, ROUND, LANE_ONLY };  // LANE_ONLY: chỉ A30, không qua Prepare
void Prepare(Rig &r, Phase phase) {
    if(phase==Phase::LEFT) r.Left();
    else if(phase==Phase::PASS) r.Pass();
    else if(phase==Phase::RETURN) r.Return();
    else r.Round();
    const char *reason=phase==Phase::LEFT ? "NE_TRAI" : phase==Phase::PASS ? "NE_QUA" :
                       phase==Phase::RETURN ? "VE_PHAI" : "VX_QUANH_DAO";
    Check(r.last.reason==reason,"Chưa dựng đúng pha cần dừng");
}

// A20–A25 — §6: Space/mất ảnh dừng ngay và không tự mở chốt khi dữ liệu về.
void Interrupt(Phase phase, bool stale_image) {
    Rig r;
    Prepare(r,phase);
    Plan plan;
    if(stale_image) {
        const int64_t image_ms=r.in.image_ms;
        do { plan=r.Step(false); } while(r.now-image_ms<=kStaleImageMs);
        Check(plan.reason=="MAT_ANH","Mất ảnh trong pha mới không nhận MAT_ANH");
    } else {
        r.in.stop=true;
        plan=r.Step();
        Check(plan.reason=="SPACE","Space trong pha mới không nhận SPACE");
        r.in.stop=false;
    }
    Check(plan.state==PlannerState::DUNG,"Ngắt an toàn chưa về DUNG");
    Zero(plan.cmd);
    for(int n=0;n<r.p.regain_frames+r.p.vote_window+3;++n) {
        plan=r.Step();
        Check(plan.state==PlannerState::DUNG,"Tự chạy lại khi chưa có G mới");
        Zero(plan.cmd);
    }
}

// A27 — §5: mất làn trong NE_TRAI phải chốt DUNG/MAT_LAN_NE.
void LostWhileAvoiding() {
    Rig r;
    r.Left();
    r.in.mask=cv::Mat(kH,kW,CV_8UC1,cv::Scalar(0));
    const int64_t begin=r.now;
    do { r.Step(); } while(r.now-begin<=r.p.lost_stop_ms);
    Check(r.last.state==PlannerState::DUNG && r.last.reason=="MAT_LAN_NE",
          "Mất làn khi né không chốt DUNG/MAT_LAN_NE");
    Zero(r.last.cmd);
    r.in.mask=Road(1.7);
    for(int n=0;n<r.p.regain_frames+r.p.vote_window+3;++n) {
        const auto plan=r.Step();
        Check(plan.state==PlannerState::DUNG,"Làn trở lại tự xóa chốt MAT_LAN_NE");
        Zero(plan.cmd);
    }
}

// Dựng Planner bật rẽ, YOLO và cảm biến tươi; odo +1 cm mỗi khung, yaw 0.
// 30 khung (1,5 s) cảm biến tươi để hết lỗi rẽ tạm do Rig khởi động với sensor_ok=false.
void SensorRig(Rig &r) {
    r.p.turn_enable=1;
    r.in.yolo_enabled=true;
    r.in.sensor_ok=true;
    r.in.yaw_deg=0;
    r.in.odo_cm=0;
    for(int n=0;n<30;++n) {
        r.in.odo_cm+=1;
        Check(r.Step().state==PlannerState::BAM_LAN,"Planner cảm biến tươi chưa bám làn");
    }
}
Plan Drive(Rig &r) { r.in.odo_cm+=1; return r.Step(); }
void EnterRound(Rig &r) {
    r.round_seen=true;
    for(int n=0;n<4*r.p.vote_window;++n)
        if(Drive(r).reason=="VX_QUANH_DAO") return;
    throw std::runtime_error("round_barrier đủ phiếu nhưng chưa VX_QUANH_DAO");
}

// A28 — §6 + ý định Quân: lượt 1 thoát rồi đi rb_lost_cm + rb_turn1_cm -> RE_TRAI;
// lượt 2 tương tự với rb_turn2_cm -> RE_PHAI. Không Reset giữa hai lượt.
void RoundCount() {
    Rig r;
    SensorRig(r);
    for(int pass=1;pass<=2;++pass) {
        const double turn_cm=pass==1 ? r.p.rb_turn1_cm : r.p.rb_turn2_cm;
        const auto want=pass==1 ? PlannerState::RE_TRAI : PlannerState::RE_PHAI;
        EnterRound(r);
        r.round_seen=false;
        double exit_odo=-1;
        bool turned=false;
        const int limit=static_cast<int>(r.p.rb_lost_cm+turn_cm)+4*r.p.vote_window+10;
        for(int n=0;n<limit && !turned;++n) {
            if(exit_odo>=0 && r.in.odo_cm-exit_odo>=r.p.rb_lost_cm+turn_cm) {
                r.in.mask=Road(1.7);
                r.in.mask.rowRange(0,static_cast<int>(0.72*kH)).setTo(0);
                r.in.mask.rowRange(static_cast<int>(0.72*kH),static_cast<int>(0.79*kH)).setTo(1);
            }
            const auto plan=Drive(r);
            if(exit_odo<0 && plan.reason=="VX_THOAT") exit_odo=r.in.odo_cm;
            if(plan.state==PlannerState::RE_TRAI || plan.state==PlannerState::RE_PHAI) {
                Check(plan.state==want,"Sai hướng rẽ sau lượt vòng xuyến (lượt 1 trái, lượt 2 phải)");
                Check(exit_odo>=0 && r.in.odo_cm-exit_odo+kEps>=r.p.rb_lost_cm+turn_cm,
                      "Rẽ trước khi đi đủ rb_lost_cm + rb_turnN_cm từ lúc mất phiếu");
                Check(r.in.odo_cm-exit_odo<=r.p.rb_lost_cm+turn_cm+r.p.vote_min+3,"Rẽ muộn hơn quãng cấu hình");
                turned=true;
            } else {
                Check(plan.state==PlannerState::BAM_LAN && plan.cmd.cms>0,"Ra khỏi vòng xuyến không bám làn");
            }
        }
        Check(turned,"Không rẽ sau lượt vòng xuyến");
        r.in.yaw_deg+=(pass==1 ? 1 : -1)*r.p.turn_yaw_deg;
        Check(Drive(r).state==PlannerState::BAM_LAN,"Quay đủ turn_yaw_deg mà chưa kết thúc rẽ");
        for(int n=0;n<r.p.vote_window+2;++n)
            Check(Drive(r).state==PlannerState::BAM_LAN,"Rẽ lại khi chưa có lượt vòng xuyến mới");
    }
}

// A29 — §6: trong vòng xuyến mất làn + cảm biến hỏng -> không GIU_HUONG, theo logic mất làn
// (cms <= lost_cms, 0,0 sau lost_stop_ms). Đối chứng sensor_ok=true thì có GIU_HUONG.
void RoundLostSensor(bool sensor) {
    Rig r;
    SensorRig(r);
    EnterRound(r);
    r.in.sensor_ok=sensor;
    r.in.mask=cv::Mat(kH,kW,CV_8UC1,cv::Scalar(0));
    const int64_t begin=r.now;
    bool hold=false;
    for(int n=0;n<r.p.lost_stop_ms/kTick+6;++n) {
        const auto plan=Drive(r);
        if(plan.state==PlannerState::GIU_HUONG || plan.reason=="GIU_HUONG") hold=true;
        if(sensor) continue;
        Check(!hold,"Cảm biến hỏng vẫn GIU_HUONG trong vòng xuyến");
        Check(plan.cmd.cms<=r.p.lost_cms+kEps,"Mất làn trong vòng xuyến chạy nhanh hơn lost_cms");
        if(r.now-begin>r.p.lost_stop_ms+kTick) Zero(plan.cmd);
    }
    if(sensor) Check(hold,"Đối chứng: cảm biến tốt phải cho GIU_HUONG khi mất làn trong vòng xuyến");
}

// A30 — bất biến dưới mask nhiễu/nhảy ở mỗi pha: trần tốc độ/góc (Rig::Step) và giới hạn
// đổi góc mỗi nhịp giữa hai khung liên tiếp cùng pha, cùng đang bám làn (bỏ khung chuyển dừng).
void NoisyInvariants() {
    for(const auto phase:{Phase::LANE_ONLY, Phase::LEFT, Phase::RETURN, Phase::ROUND}) {
        Rig r;
        if(phase!=Phase::LANE_ONLY) Prepare(r,phase);
        const cv::Mat base=phase==Phase::LEFT ? Obstacle() : phase==Phase::ROUND ? Island() : Road(1.7);
        cv::RNG rng(79);
        Plan previous=r.last;
        int pairs=0;
        for(int k=0;k<64;++k) {
            cv::Mat mask;
            if(k<4) mask=base.clone();
            else if(k<16) mask=Shift(base,(k%2 ? 0.25 : -0.25)*kW);
            else if(k%3==0) mask=Shift(base,rng.uniform(-0.3,0.3)*kW);
            else if(k%3==1) { mask.create(kH,kW,CV_8UC1); rng.fill(mask,cv::RNG::UNIFORM,0,2); }
            else {
                mask=base.clone();
                const cv::Point corner(rng.uniform(0,kW),rng.uniform(Top(),kH));
                cv::rectangle(mask,corner,corner+cv::Point(rng.uniform(5,80),rng.uniform(5,40)),
                              cv::Scalar(0),cv::FILLED);
            }
            r.in.mask=mask;
            const auto plan=r.Step();
            const bool both=previous.reason==plan.reason &&
                previous.state==PlannerState::BAM_LAN && plan.state==PlannerState::BAM_LAN &&
                previous.debug.state=="BAM_LAN" && plan.debug.state=="BAM_LAN" &&
                previous.cmd.cms>0 && plan.cmd.cms>0;
            if(both) {
                ++pairs;
                Check(std::abs(plan.cmd.deg-previous.cmd.deg)<=r.p.steer_rate_dps*0.05+1e-6,
                      "Góc đổi nhanh hơn steer_rate_dps dưới mask nhiễu");
            }
            previous=plan;
        }
        Check(pairs>=3,"Pha nhiễu không có đủ cặp khung bám làn để kiểm");
    }
}

// A31 — §4/§5: mask quá mask_max_age_ms (ảnh tươi) khi NE_TRAI -> cms 0, không né bằng mask cũ.
void StaleWhileAvoiding() {
    Rig r;
    r.Left();
    const int64_t mask_ms=r.in.mask_ms;
    Plan plan;
    do { plan=r.Step(true,false); } while(r.now-mask_ms<=r.p.mask_max_age_ms);
    Check(std::abs(plan.cmd.cms)<=kEps,"Mask cũ khi NE_TRAI vẫn có tốc độ");
    for(int n=0;n<3;++n)
        Check(std::abs(r.Step(true,false).cmd.cms)<=kEps,"Mask cũ kéo dài khi NE_TRAI vẫn chạy");
}

struct Case { const char *name; void (*run)(); };
const Case kCases[]={
    {"A01",Straight},{"A02",RightOffset},{"A03",TwoLanes},{"A04",Junction},
    {"A05",[]{ ImageEdge(false); }},{"A06",SmallHole},{"A07",DetachedRoad},
    {"A08",LostLane},{"A09",StaleMask},{"A10",JumpLimit},{"A11",InvalidMask},{"A11b",RoadValueTwo},
    {"A12",AvoidLeft},{"A13",AvoidSequence},{"A14",[]{ NotObstacle(false); }},
    {"A15",[]{ NotObstacle(true); }},{"A16",NoAvoidSpace},{"A17",StopObstacle},
    {"A18",RoundLane},{"A19",RoundPlanner},
    {"A20",[]{ Interrupt(Phase::LEFT,false); Interrupt(Phase::PASS,false); }},
    {"A21",[]{ Interrupt(Phase::LEFT,true); Interrupt(Phase::PASS,true); }},
    {"A22",[]{ Interrupt(Phase::RETURN,false); }},{"A23",[]{ Interrupt(Phase::RETURN,true); }},
    {"A24",[]{ Interrupt(Phase::ROUND,false); }},{"A25",[]{ Interrupt(Phase::ROUND,true); }},
    {"A26",[]{ ImageEdge(true); }},{"A27",LostWhileAvoiding},{"A28",RoundCount},
    {"A29",[]{ RoundLostSensor(false); RoundLostSensor(true); }},{"A30",NoisyInvariants},
    {"A31",StaleWhileAvoiding}
};
}

int main(int argc, char **argv) {
    try {
        Check(argc==2,"Cần đúng một mã ca A01..A31 (gồm A11b)");
        const std::string name=argv[1];
        for(const auto &test:kCases) {
            if(name!=test.name) continue;
            test.run();
            std::cout << "PASS " << name << '\n';
            return 0;
        }
        throw std::runtime_error("Mã ca không tồn tại");
    } catch(const std::exception &error) {
        std::cerr << "FAIL " << (argc>1 ? argv[1] : "T79") << ": " << error.what() << '\n';
        return 1;
    }
}
