// Unit DEV cho ExtractCorridor (T79 0c): mask tổng hợp 192x320, tham số mặc định control.yaml.
#include "control/LaneController.h"
#include "control/Planner.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
using namespace control;
constexpr int kH=192, kW=320;
void Check(bool ok, const std::string &why) { if(!ok) throw std::runtime_error(why); }
void Near(double a, double b, double tol, const std::string &why) {
    Check(std::abs(a-b)<=tol,why+": "+std::to_string(a)+" != "+std::to_string(b));
}
// Bề rộng một làn kỳ vọng tại y theo lane1_width_frac (ngoại suy tuyến tính ngoài [top, bottom]).
double Lane1(const ControlParams &p, double y) {
    const double yb=std::lround(p.corridor_bottom_frac*kH), yt=std::lround(p.corridor_top_frac*kH);
    const double t=(yb-y)/(yb-yt);
    return kW*(p.lane1_width_frac[0]+(p.lane1_width_frac[1]-p.lane1_width_frac[0])*t);
}
// Đường hình thang: tâm cx (pixel), bề rộng scale·w(y), giá trị value.
cv::Mat Road(const ControlParams &p, double cx, double scale, unsigned char value=1) {
    cv::Mat mask(kH,kW,CV_8UC1,cv::Scalar(0));
    for(int y=0;y<kH;++y) {
        const double half=std::max(0.0,scale*Lane1(p,y)/2);
        const int a=std::max(0,int(std::lround(cx-half))), b=std::min(kW-1,int(std::lround(cx+half)));
        for(int x=a;x<=b;++x) mask.at<unsigned char>(y,x)=value;
    }
    return mask;
}
int ValidRows(const Corridor &c) { int n=0; for(const auto &r:c.rows) n+=r.valid; return n; }

// Kiểm Planner bằng mask một làn cố định: hàng gần nguyên vẹn, các hàng trên có khe.
struct ObstacleRig {
    Planner planner;
    PlannerInput in;
    ControlParams p;
    int64_t now=1000;
    uint64_t frame=0;
    bool obstacle_seen=false;
    ObstacleRig() {
        p.start_wait_green=0; p.lane1_width_frac={0.8,0.8}; p.avoid_min_width_ratio=0.4;
        p.kd=0; p.obstacle_pass_cm=10;
        in.yolo_enabled=true; Mask(false); in.go=true; Step(); in.go=false;
    }
    void Mask(bool hole) {
        in.mask=cv::Mat(100,200,CV_8UC1,cv::Scalar(0));
        in.mask.colRange(20,181).setTo(1);
        if(hole) in.mask(cv::Rect(90,60,40,35)).setTo(0);
    }
    Plan Step(bool sensor=true) {
        now+=50; in.image_ms=in.mask_ms=now; in.sensor_ms=now; in.sensor_ok=sensor;
        in.detections={{},now,++frame};
        if(obstacle_seen) in.detections.items.push_back({"obstacle",cv::Rect2f(600,100,128,100),0.9f,now});
        return planner.Step(in,now,p);
    }
    Plan Left() {
        Mask(true); obstacle_seen=true; Plan plan;
        for(int i=0;i<p.vote_min;++i) plan=Step();
        Check(plan.reason=="NE_TRAI","Đủ phiếu phải né");
        return plan;
    }
    Plan Pass() {
        Left(); Mask(false); obstacle_seen=false; Plan plan;
        for(int i=0;i<p.vote_window;++i) plan=Step();
        Check(plan.reason=="NE_QUA","Đủ mask sạch phải qua");
        return plan;
    }
};
void PlannerObstacle() {
    // Một hàng gần có khe chưa đủ phiếu true; hai hàng gần mới đủ.
    for(int count:{1,2}) {
        ObstacleRig r;
        r.Mask(false);
        const auto rows=ExtractCorridor(r.in.mask,r.p,TrajMode::LANE).rows;
        for(int i=1;i<=count;++i) r.in.mask(cv::Rect(90,rows[i].y,40,1)).setTo(0);
        Plan plan;
        for(int i=0;i<r.p.vote_window;++i) plan=r.Step();
        Check(plan.mode==TrajMode::LANE,"Lỗ một/hai hàng không có hộp đều không kích né");
    }
    { ObstacleRig r; r.Left(); r.p.avoid_min_width_ratio=0.75;
      const auto plan=r.Step();
      Check(plan.state==PlannerState::DUNG_VAT_CAN && plan.reason=="KHONG_DUONG_NE" &&
            plan.cmd.cms==0 && plan.cmd.deg==0,"Mất đường né giữa NE_TRAI phải dừng 0,0");
      r.Mask(false); r.obstacle_seen=false;
      for(int n=1;n<=r.p.vote_window;++n) {
          const auto cleared=r.Step();
          Check((cleared.state==PlannerState::BAM_LAN)==(n==r.p.vote_window),
                "Dừng thiếu đường né chỉ nhả sau đủ cửa sổ mask sạch");
          // Lặp chính mask không được tạo thêm lượt sạch hoặc làm mất số lượt đang đếm.
          r.now+=1; r.in.image_ms=r.in.sensor_ms=r.now;
          const auto repeated=r.planner.Step(r.in,r.now,r.p);
          Check(repeated.state==cleared.state,"Mask trùng làm nhả dừng thiếu đường né");
      } }
    for(bool active:{false,true}) {
        ObstacleRig r;
        if(active) r.Left();
        r.obstacle_seen=true;
        r.Mask(false);
        const auto rows=ExtractCorridor(r.in.mask,r.p,TrajMode::LANE).rows;
        r.in.mask(cv::Rect(50,rows[1].y,40,1)).setTo(0);
        r.in.mask(cv::Rect(90,rows[2].y,40,1)).setTo(0);
        const auto mixed=ExtractCorridor(r.in.mask,r.p,TrajMode::LANE).rows;
        Check(mixed[1].hole && !mixed[1].avoid_ok && mixed[2].hole && mixed[2].avoid_ok,
              "Ca trộn phải có một hàng thiếu đường né và một hàng đủ đường né");
        Plan plan;
        for(int i=0;i<(active ? 1 : r.p.vote_min);++i) plan=r.Step();
        Check(plan.state==PlannerState::DUNG_VAT_CAN && plan.reason=="KHONG_DUONG_NE" &&
              plan.cmd.cms==0 && plan.cmd.deg==0,"Một hàng thiếu đường né phải dừng cả khi hàng khác đủ");
    }
    { ObstacleRig r; r.Pass();
      const double cap=2*r.p.max_speed_cms*0.05;
      for(const std::string phase:{"NE_QUA","VE_PHAI"}) {
          double distance=0;
          do {
              r.in.odo_cm+=1000; distance+=cap;
              const auto plan=r.Step();
              const std::string next=phase=="NE_QUA" ? "VE_PHAI" : "BAM_LAN";
              Check(plan.reason==(distance>=r.p.obstacle_pass_cm ? next : phase),
                    "Encoder nhảy lớn chỉ được cộng tối đa 2*max_speed_cms*dt mỗi bước");
          } while(distance<r.p.obstacle_pass_cm);
      } }
    { ObstacleRig r; auto plan=r.Pass();
      for(const std::string phase:{"NE_QUA","VE_PHAI"}) {
          r.in.odo_cm+=1; plan=r.Step();
          Check(plan.reason==phase,"Một cm chưa đủ kết thúc pha");
          double distance=1; bool finished=false;
          for(int n=1;n<=100;++n) {
              if(n>=30) distance+=plan.cmd.cms*0.05;
              plan=r.Step();
              const std::string next=phase=="NE_QUA" ? "VE_PHAI" : "BAM_LAN";
              Check(plan.reason==(distance>=r.p.obstacle_pass_cm ? next : phase),
                    "Encoder đứng 1500 ms phải dùng tích phân từng bước, không cộng bù quãng cũ");
              if(distance>=r.p.obstacle_pass_cm) { finished=true; break; }
          }
          Check(finished,"Encoder đứng yên phải kết thúc pha nhờ tích phân lệnh");
      } }
    { ObstacleRig r; auto plan=r.Pass(); double distance=0;
      for(int i=0;i<1000;++i) {
          distance+=plan.cmd.cms*0.05; r.in.odo_cm+=1000;
          plan=r.Step(false);
          if(plan.reason=="VE_PHAI") {
              Check(distance>=r.p.obstacle_pass_cm,"Cảm biến hỏng không được dùng bước nhảy odo");
              break;
          }
          Check(plan.reason=="NE_QUA" && i<999,"Tích phân lệnh khi cảm biến hỏng phải kết thúc pha");
      }
      Check(r.Step().reason=="VE_PHAI","Hồi cảm biến phải lấy mốc mới, không cộng nhảy odo"); }
}

void OneLane() {
    ControlParams p;
    const auto c=ExtractCorridor(Road(p,160,1.0),p,TrajMode::LANE);
    Check(int(c.rows.size())==p.corridor_rows && ValidRows(c)==p.corridor_rows,"Một làn phải đủ hàng hợp lệ");
    for(const auto &r:c.rows) {
        Check(r.lanes==1 && !r.hole && r.flag_left=="THAT" && r.flag_right=="THAT","Một làn: cờ/số làn sai");
        Near(r.target,160,1.0,"Một làn: target là tâm");
        Near(r.conf,1,1e-9,"Một làn: conf 1");
    }
    Check(c.rows.front().y>c.rows.back().y,"Hàng 0 phải là hàng gần (y lớn)");
    // Giá trị 2 cũng là mặt đường: cùng kết quả.
    const auto c2=ExtractCorridor(Road(p,160,1.0,2),p,TrajMode::LANE);
    for(size_t i=0;i<c.rows.size();++i) Near(c2.rows[i].target,c.rows[i].target,1e-9,"Mask 2 phải như mask 1");
}
void TwoLanes() {
    ControlParams p;
    const auto mask=Road(p,160,1.6);
    const auto lane=ExtractCorridor(mask,p,TrajMode::LANE), left=ExtractCorridor(mask,p,TrajMode::AVOID_LEFT);
    for(size_t i=0;i<lane.rows.size();++i) {
        const auto &r=lane.rows[i];
        Check(r.valid && r.lanes==2,"Hai làn phải nhận 2 làn");
        Near(r.target,r.right-0.25*(r.right-r.left),1e-9,"Hai làn LANE: tâm làn phải");
        Near(left.rows[i].target,r.right-0.75*(r.right-r.left),1e-9,"Hai làn AVOID_LEFT: tâm làn trái");
        Near(r.conf,1,1e-9,"Hai làn bề rộng bình thường: conf 1");
    }
}
void ImageEdge() {
    ControlParams p;
    const auto c=ExtractCorridor(Road(p,40,1.0),p,TrajMode::LANE);
    bool edge=false;
    for(const auto &r:c.rows) if(r.valid && r.flag_left=="MEP_ANH") {
        edge=true;
        Check(r.flag_right=="THAT" && r.left<0,"Biên trái mép ảnh phải được thay ra ngoài ảnh");
        Near(r.conf,0.7,1e-9,"Một biên không thật: conf 0,7");
        Near(r.target,40,3.0,"Thay biên giữ tâm đường thật");
    }
    Check(edge,"Thiếu cờ MEP_ANH");
    const cv::Mat white(kH,kW,CV_8UC1,cv::Scalar(1));
    for(auto mode:{TrajMode::LANE,TrajMode::AVOID_LEFT,TrajMode::RETURN_RIGHT,TrajMode::ROUNDABOUT}) {
        for(const auto &r:ExtractCorridor(white,p,mode).rows) {
            Check(r.valid && r.flag_left=="MEP_ANH" && r.flag_right=="MEP_ANH",
                  "Mask toàn đường phải ghi hai biên mép ảnh");
            Check(r.conf<p.conf_min,"Hai biên không thật phải dưới ngưỡng tin cậy mặc định");
        }
        ControlParams low=p; low.conf_min=0.35;  // cận dưới cho phép
        LaneController controller;
        const auto initial=controller.Step(Road(low,160,1),1000,1000,low,mode);
        Check(initial.debug.state=="BAM_LAN" && initial.cmd.cms>0,"Phải có làn thật trước khi thử mất biên");
        ControlResult result;
        for(int elapsed=50;elapsed<=low.lost_stop_ms+50;elapsed+=50)
            result=controller.Step(white,1000+elapsed,1000+elapsed,low,mode);
        // Cận dưới 0,35 > conf 0,3 của hàng hai biên giả: mọi chế độ, kể cả ROUNDABOUT, đều dừng.
        Check(result.debug.state=="MAT_LAN" && result.debug.reason=="MAT_LAN" &&
              result.cmd.cms==0 && result.cmd.deg==0,
              "Không có biên thật phải dừng 0,0 sau lost_stop_ms ở conf_min thấp nhất cho phép");
    }
}
// Lỗi 3 (sa bàn 10/11): đường hai làn, cạnh trái thật, cạnh phải ra khỏi ảnh (cua phải/lối ra vòng xuyến).
// Bề rộng thô đã >= two_lane_ratio·w thì đường thật ít nhất rộng như vậy: phải nhận hai làn, mục tiêu ở làn PHẢI.
void TwoLaneRightCut() {
    ControlParams p;
    const double cx=230;  // đường 2,2 làn: biên trái thật ~30..170 px, biên phải ra khỏi ảnh ở các hàng gần
    const auto c=ExtractCorridor(Road(p,cx,2.2),p,TrajMode::ROUNDABOUT);
    int cut=0;
    for(const auto &r:c.rows) {
        if(!r.valid || r.flag_left!="THAT" || r.flag_right!="MEP_ANH") continue;
        ++cut;
        const double w=Lane1(p,r.y);
        Check(r.lanes==2,"Biên phải bị cắt, bề rộng thô hai làn: phải nhận 2 làn");
        Check(r.target>=r.left+1.1*w,"Mục tiêu phải ở làn phải (>= trái + 1,1 làn; ROUNDABOUT kẹp biên phải về đoạn thô), không ở làn trái");
        Check(r.target>=r.raw_l && r.target<=kW+2*w,"Mục tiêu hợp lý");
    }
    Check(cut>=3,"Thiếu hàng trái thật/phải mép ảnh để kiểm");
    // Ngoài vòng xuyến (LANE) không áp: hàng gần giống hệt đường một làn có lối mở phải khi xe lệch phải.
    for(const auto &r:ExtractCorridor(Road(p,cx,2.2),p,TrajMode::LANE).rows)
        if(r.valid && r.flag_left=="THAT" && r.flag_right=="MEP_ANH")
            Check(r.lanes==1,"LANE: biên phải bị cắt không được dựng hai làn");
    // Một làn bị cắt biên phải (bề rộng thô < 1,5 làn): giữ như cũ, tâm một làn tính từ biên trái.
    const auto one=ExtractCorridor(Road(p,kW-20,1.0),p,TrajMode::LANE);
    int one_cut=0;
    for(const auto &r:one.rows) if(r.valid && r.flag_left=="THAT" && r.flag_right=="MEP_ANH") {
        ++one_cut;
        Check(r.lanes==1,"Một làn bị cắt biên phải vẫn là 1 làn");
        Near(r.target,r.left+Lane1(p,r.y)/2,1e-6,"Một làn bị cắt: tâm làn từ biên trái");
    }
    Check(one_cut>=1,"Thiếu hàng một làn bị cắt để kiểm");
    // Ca âm (safety-reviewer 11/11): đường MỘT làn giữa ảnh, các hàng xa mở sang phải tới mép ảnh (ngã T/miệng nhánh).
    // Hàng gần là một làn thật nên hàng xa không được dựng thành hai làn; mục tiêu giữ ở tâm làn tính từ biên trái.
    cv::Mat open=Road(p,160,1.0);
    const int y_open=std::lround(0.80*kH);
    for(int y=0;y<y_open;++y) {
        const int a=std::max(0,int(std::lround(160-Lane1(p,y)/2)));
        for(int x=a;x<kW;++x) open.at<unsigned char>(y,x)=1;
    }
    const auto branch=ExtractCorridor(open,p,TrajMode::LANE);
    int opened=0;
    for(const auto &r:branch.rows) {
        if(!r.valid) continue;
        if(r.y>=y_open) { Check(r.lanes==1,"Hàng gần đường một làn phải là 1 làn"); continue; }
        if(r.flag_left!="THAT" || r.flag_right!="MEP_ANH") continue;
        ++opened;
        Check(r.lanes==1,"Đường một làn mở phải: hàng xa không được thành 2 làn");
        Check(r.target<=r.left+0.75*Lane1(p,r.y),"Đường một làn mở phải: mục tiêu không bị kéo sang phải");
    }
    Check(opened>=2,"Thiếu hàng mở phải để kiểm ca âm");
}
// Mặt nạ thật lượt 18:01 10/11 (VX_THOAT 287,3 s và 288,2 s, lối ra vòng xuyến bên phải):
// mã cũ dựng một làn từ biên trái -> mục tiêu 0,41–0,43 W (lệch trái), xe lái trái 13–14°.
void RealRoundaboutExit(const std::string &dir) {
    ControlParams p;
    for(const char *name:{"vx_thoat_7374_15489.png","vx_thoat_7374_15537.png"}) {
        const cv::Mat mask=cv::imread(dir+"/"+name,cv::IMREAD_GRAYSCALE);
        Check(!mask.empty(),std::string("Không đọc được ")+name);
        for(auto mode:{TrajMode::ROUNDABOUT}) {  // VX_THOAT chạy chế độ ROUNDABOUT (rb_in_pass_ còn đúng)
            const auto c=ExtractCorridor(mask,p,mode);
            const auto &near=c.rows.front();
            Check(near.valid && near.flag_left=="THAT" && near.flag_right=="MEP_ANH",
                  std::string("Mặt nạ thật phải có biên trái thật, phải mép ảnh: ")+name);
            Check(near.lanes==2,std::string("Mặt nạ thật lối ra hai làn phải nhận 2 làn: ")+name);
            Check(near.target>0.5*mask.cols,std::string("Mục tiêu hàng gần phải ở nửa phải ảnh: ")+name);
        }
    }
}
void Jump() {
    ControlParams p;
    auto mask=Road(p,160,1.0);
    // Từ y<=150 mặt đường loe sang phải 0,15·W (bãi trắng nối liền): biên phải nhảy.
    for(int y=0;y<=150;++y) for(int x=160;x<std::min(kW-1,int(160+Lane1(p,y)/2+0.15*kW));++x) mask.at<unsigned char>(y,x)=1;
    const auto c=ExtractCorridor(mask,p,TrajMode::LANE);
    bool jump=false;
    for(const auto &r:c.rows) if(r.valid && r.flag_right=="NHAY") {
        jump=true;
        Check(r.right<160+Lane1(p,r.y)/2+0.10*kW,"Biên nhảy phải được thay theo bề rộng tham chiếu");
        Check(r.conf<1,"Biên nhảy phải giảm conf");
    }
    Check(jump,"Thiếu cờ NHAY");
}
void Hole() {
    PlannerObstacle();
    ControlParams p;
    auto mask=Road(p,160,1.0);
    cv::rectangle(mask,cv::Point(170,100),cv::Point(185,175),cv::Scalar(0),cv::FILLED);
    const auto c=ExtractCorridor(mask,p,TrajMode::LANE);
    const auto a=ExtractCorridor(mask,p,TrajMode::AVOID_LEFT);
    bool hole=false;
    for(size_t i=0;i<c.rows.size();++i) {
        const auto &r=c.rows[i];
        if(!r.hole) continue;
        hole=true;
        Near(r.hole_l,170,1e-9,"Đầu khe"); Near(r.hole_r,185,1e-9,"Cuối khe");
        Near(r.target,(r.left+r.right)/2,1e-9,"LANE có lỗ vẫn nhắm tâm");
        Near(r.conf,0.7,1e-9,"Lỗ trừ conf 0,3");
        const bool wide=r.hole_l-r.left>=p.avoid_min_width_ratio*Lane1(p,r.y);
        Check(r.avoid_ok==wide && a.rows[i].avoid_ok==wide,"avoid_ok theo bề rộng phần trái");
        if(wide) Near(a.rows[i].target,(r.left+r.hole_l)/2,1e-9,"AVOID_LEFT nhắm phần trống trái lỗ");
    }
    Check(hole,"Lỗ trong hành lang phải được ghi");
    // Lỗ sát trái: phần trái quá hẹp -> avoid_ok=false.
    auto narrow=Road(p,160,1.0);
    for(int y=100;y<=175;++y) {
        const int a0=int(std::lround(160-Lane1(p,y)/2));
        for(int x=a0+30;x<=200;++x) narrow.at<unsigned char>(y,x)=0;
    }
    bool blocked=false;
    for(const auto &r:ExtractCorridor(narrow,p,TrajMode::AVOID_LEFT).rows) if(r.hole && !r.avoid_ok) blocked=true;
    Check(blocked,"Phần trái hẹp phải avoid_ok=false");
}
void Split() {
    ControlParams p;
    auto mask=Road(p,150,1.0);
    cv::rectangle(mask,cv::Point(290,0),cv::Point(kW-1,kH-1),cv::Scalar(1),cv::FILLED);  // vùng rời xa trục
    for(const auto &r:ExtractCorridor(mask,p,TrajMode::LANE).rows)
        Check(r.valid && r.right<280,"Vùng rời xa trục không được chọn");
    // Đường bị cắt ở y<140, phía trên có mảng lệch hẳn sang trái: không chồng lấn -> các hàng trên không hợp lệ.
    auto cut=Road(p,160,1.0);
    cut(cv::Rect(0,0,kW,140)).setTo(0);
    cv::rectangle(cut,cv::Point(0,0),cv::Point(60,139),cv::Scalar(1),cv::FILLED);
    const auto c=ExtractCorridor(cut,p,TrajMode::LANE);
    for(const auto &r:c.rows) Check(r.valid==(r.y>=140),"Hàng không chồng lấn và mọi hàng trên phải không hợp lệ");
    // Trục ngoài mọi đoạn quá w(y): hàng gần không hợp lệ.
    p.lane1_width_frac={0.3,0.17};
    const auto far=ExtractCorridor(Road(p,20,0.5),p,TrajMode::LANE);
    Check(ValidRows(far)==0,"Đoạn cách trục quá w(y) không được nhận");
}
void Roundabout() {
    ControlParams p;
    cv::Mat mask(kH,kW,CV_8UC1,cv::Scalar(1));
    cv::rectangle(mask,cv::Point(130,0),cv::Point(190,170),cv::Scalar(0),cv::FILLED);  // đảo
    const auto lane=ExtractCorridor(mask,p,TrajMode::LANE), round=ExtractCorridor(mask,p,TrajMode::ROUNDABOUT);
    bool merged=false, right=false;
    for(size_t i=0;i<round.rows.size();++i) {
        const auto &r=round.rows[i];
        Check(r.valid,"Vòng xuyến phải có hành lang");
        if(r.y<=170) {
            Check(!r.hole && r.target>190,"ROUNDABOUT phải chọn nhánh phải đảo");
            right=true;
            merged=merged || lane.rows[i].hole;
        }
    }
    Check(right && merged,"LANE gộp quanh đảo, ROUNDABOUT tách nhánh phải");
}
// Vật cản chạm đáy vẫn phải tạo khe ngay hàng gần.
void BottomHole() {
    ControlParams p;
    auto mask=Road(p,160,1.6);
    mask(cv::Rect(145,100,31,kH-100)).setTo(0);
    for(auto mode:{TrajMode::LANE,TrajMode::AVOID_LEFT,TrajMode::RETURN_RIGHT}) {
        const auto c=ExtractCorridor(mask,p,mode);
        const auto &r=c.rows.front();
        Check(r.valid && r.hole && r.lanes==2,"Vật cản chạm đáy: hàng gần phải có khe hai làn");
        Near(r.hole_l,145,0,"Đầu khe chạm đáy");
        Near(r.hole_r,175,0,"Cuối khe chạm đáy");
    }
    Check(!ExtractCorridor(mask,p,TrajMode::ROUNDABOUT).rows.front().hole,
          "ROUNDABOUT không gộp khe hàng gần");
}
void RoundaboutJump() {
    ControlParams p;
    cv::Mat mask(kH,kW,CV_8UC1,cv::Scalar(0));
    mask.colRange(40,301).setTo(1);
    mask(cv::Rect(130,0,111,175)).setTo(0);
    bool jump=false;
    for(const auto &r:ExtractCorridor(mask,p,TrajMode::ROUNDABOUT).rows) if(r.valid) {
        jump=jump || r.flag_left=="NHAY";
        const int x=int(std::lround(r.target));
        Check(x>=0 && x<kW && mask.at<unsigned char>(r.y,x)!=0,
              "Target ROUNDABOUT phải nằm trên pixel đường");
        if(r.y<=175) Check(r.left>=241 && r.right<=300,"Biên phải nằm trong nhánh thô");
    }
    Check(jump,"Ca vòng xuyến phải có biên trái NHAY");
}
void BottomUnchanged() {
    ControlParams p;
    p.lane1_width_frac={0.5,0.5};
    for(auto mode:{TrajMode::LANE,TrajMode::AVOID_LEFT,TrajMode::RETURN_RIGHT}) {
        for(bool contains:{false,true}) {
            cv::Mat mask(kH,kW,CV_8UC1,cv::Scalar(0));
            mask.colRange(contains ? 100 : 180,contains ? 221 : 241).setTo(1);
            mask.colRange(270,301).setTo(1);
            const auto c=ExtractCorridor(mask,p,mode);
            const auto &r=c.rows.front();
            Check(r.valid && !r.hole && r.lanes==1 && r.flag_left=="THAT" && r.flag_right=="THAT",
                  "Không có khe quanh trục: giữ cờ và số làn cũ");
            Near(r.left,contains ? 100 : 180,0,"Biên trái cũ");
            Near(r.right,contains ? 220 : 240,0,"Biên phải cũ");
            Near(r.target,contains ? 160 : 210,0,"Target cũ");
            Near(r.conf,contains ? 1 : 0.7,1e-9,"Độ tin cậy cũ");
        }
    }
}
void Empty() {
    ControlParams p;
    const auto c=ExtractCorridor(cv::Mat(kH,kW,CV_8UC1,cv::Scalar(0)),p,TrajMode::LANE);
    Check(int(c.rows.size())==p.corridor_rows && ValidRows(c)==0,"Mask rỗng: đủ hàng debug, không hàng hợp lệ");
    bool thrown=false;
    try { ExtractCorridor(cv::Mat(kH,kW,CV_8UC1,cv::Scalar(3)),p,TrajMode::LANE); } catch(const std::runtime_error &) { thrown=true; }
    Check(thrown,"Giá trị > 2 phải ném runtime_error");
    thrown=false;
    try { ExtractCorridor(cv::Mat(kH,kW,CV_32FC1,cv::Scalar(1)),p,TrajMode::LANE); } catch(const std::runtime_error &) { thrown=true; }
    Check(thrown,"Sai kiểu mask phải ném runtime_error");
}
}
// Nhớ số làn (mô phỏng vòng kín 11/11): ra cua phải xe lệch trái, biên phải chạm mép ảnh. Có nhớ hai làn thì mục tiêu
// phải ở làn PHẢI; không nhớ / nhớ một làn giữ như cũ; chế độ khác LANE không đổi; đường một làn không nhớ hai.
void LaneMemory() {
    ControlParams p;
    const cv::Mat cut=Road(p,kW/2.0+50,2.0);  // hai làn, xe lệch trái: biên phải chạm mép ảnh
    int cut_rows=0;
    const auto base=ExtractCorridor(cut,p,TrajMode::LANE,0);
    const auto zero=ExtractCorridor(cut,p,TrajMode::LANE);
    const auto one=ExtractCorridor(cut,p,TrajMode::LANE,1);
    const auto two=ExtractCorridor(cut,p,TrajMode::LANE,2);
    Check(base.rows.size()==two.rows.size(),"Số hàng");
    for(size_t i=0;i<base.rows.size();++i) {
        const auto &a=base.rows[i], &z=zero.rows[i], &o=one.rows[i], &t=two.rows[i];
        Near(a.target,z.target,1e-9,"prior mặc định = 0");
        Near(a.target,o.target,1e-9,"prior 1 giữ như cũ");
        if(!t.valid || t.flag_left!="THAT" || t.flag_right!="MEP_ANH") continue;
        ++cut_rows;
        Check(t.lanes==2,"prior 2: biên phải bị cắt phải dựng hai làn");
        Check(t.target>(t.left+t.right)/2,"prior 2: mục tiêu ở làn phải (phải hơn tâm hàng)");
        Check(t.right-t.left>=2*Lane1(p,t.y)-1e-6,"prior 2: bề rộng >= hai làn");
    }
    Check(cut_rows>=2,"Thiếu hàng biên phải bị cắt để kiểm");
    int changed=0;
    for(size_t i=0;i<base.rows.size();++i) if(std::abs(base.rows[i].target-two.rows[i].target)>1) ++changed;
    Check(changed>=1,"prior 2 phải đổi mục tiêu so với prior 0 (kiểm ngược)");
    // Chế độ khác LANE không đổi theo prior.
    for(auto mode:{TrajMode::ROUNDABOUT,TrajMode::TURN_LEFT,TrajMode::TURN_RIGHT,TrajMode::AVOID_LEFT}) {
        const auto m0=ExtractCorridor(cut,p,mode,0), m2=ExtractCorridor(cut,p,mode,2);
        for(size_t i=0;i<m0.rows.size();++i) {
            if(mode==TrajMode::ROUNDABOUT) continue;  // ROUNDABOUT có luật riêng (không dùng prior)
            Near(m0.rows[i].target,m2.rows[i].target,1e-9,"Chế độ khác LANE không đổi theo prior");
        }
    }
    // Hàng ngay dưới là đường MỘT làn quan sát thật (hai biên thật): không dựng hai làn dù prior 2.
    cv::Mat open=Road(p,160,1.0);
    const int y_open=std::lround(0.80*kH);
    for(int y=0;y<y_open;++y) {
        const int a=std::max(0,int(std::lround(160-Lane1(p,y)/2)));
        for(int x=a;x<kW;++x) open.at<unsigned char>(y,x)=1;
    }
    const auto t_open=ExtractCorridor(open,p,TrajMode::LANE,2);
    int opened=0;
    for(const auto &r:t_open.rows) {
        if(!r.valid || r.y>=y_open || r.flag_left!="THAT" || r.flag_right!="MEP_ANH") continue;
        ++opened;
        Check(r.lanes==1,"Ngã T một làn sau đoạn hai làn: không dựng hai làn dù prior 2");
    }
    Check(opened>=2,"Thiếu hàng mở phải để kiểm");
    // LaneController: nhớ qua khung, Reset xóa, đường một làn không nhớ hai.
    ControlParams q;
    const cv::Mat full=Road(q,kW/2.0,2.0), narrow=Road(q,kW/2.0,1.0);
    int64_t now=1000;
    auto step=[&](LaneController &lc,const cv::Mat &mask) { now+=50; return lc.Step(mask,now,now,q,TrajMode::LANE,false); };
    LaneController fresh, remembered;
    for(int i=0;i<3;++i) step(remembered,full);
    const auto fresh_cut=step(fresh,cut), remembered_cut=step(remembered,cut);
    Check(remembered_cut.debug.valid_rows>0 && fresh_cut.debug.valid_rows>0,"Cần hàng hợp lệ để so e");
    Check(remembered_cut.debug.e>fresh_cut.debug.e+0.05,"Nhớ hai làn: e kéo xe sang phải so với bộ mới");
    remembered.Reset();
    const auto after_reset=step(remembered,cut);
    Near(after_reset.debug.e,fresh_cut.debug.e,1e-9,"Reset xóa bộ nhớ số làn");
    LaneController single;
    for(int i=0;i<3;++i) step(single,narrow);
    const cv::Mat clipped=Road(q,kW-20,1.0);
    const auto single_cut=step(single,clipped);
    LaneController single_fresh; const auto single_fresh_cut=step(single_fresh,clipped);
    Check(single_cut.debug.rows.size()==single_fresh_cut.debug.rows.size(),"Số hàng");
    for(size_t i=0;i<single_cut.debug.rows.size();++i)
        Near(single_cut.debug.rows[i].target,single_fresh_cut.debug.rows[i].target,1e-9,"Đường một làn không nhớ hai làn");
    // Mặt nạ cũ xóa bộ nhớ.
    LaneController stale;
    for(int i=0;i<3;++i) step(stale,full);
    const auto stale_res=stale.Step(cut,now-q.mask_max_age_ms-100,now+50,q,TrajMode::LANE,false);
    Check(stale_res.debug.state=="MAT_NAN_CU","Mặt nạ cũ -> MAT_NAN_CU");
    now+=100;
    const auto stale_cut=step(stale,cut);
    for(size_t i=0;i<stale_cut.debug.rows.size();++i)
        Near(stale_cut.debug.rows[i].target,fresh_cut.debug.rows[i].target,1e-9,"Mặt nạ cũ xóa bộ nhớ số làn");
}
int main(int argc, char **argv) {
    try {
        Check(argc==2 || argc==3,"Cần một tên ca (và thư mục dữ liệu)");
        const std::string name=argv[1];
        if(name=="one_lane") OneLane();
        else if(name=="two_lanes") TwoLanes();
        else if(name=="image_edge") ImageEdge();
        else if(name=="jump") Jump();
        else if(name=="hole") Hole();
        else if(name=="split") Split();
        else if(name=="roundabout") Roundabout();
        else if(name=="bottom_hole") BottomHole();
        else if(name=="roundabout_jump") RoundaboutJump();
        else if(name=="bottom_unchanged") BottomUnchanged();
        else if(name=="empty") Empty();
        else if(name=="two_lane_right_cut") TwoLaneRightCut();
        else if(name=="lane_memory") LaneMemory();
        else if(name=="real_roundabout_exit") RealRoundaboutExit(argc==3 ? argv[2] : "fixtures");
        else throw std::runtime_error("Ca lạ: "+name);
        std::cout << "PASS " << name << '\n';
        return 0;
    } catch(const std::exception &error) { std::cerr << "FAIL: " << error.what() << '\n'; return 1; }
}
