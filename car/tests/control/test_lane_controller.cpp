#include "control/Planner.h"
#include <opencv2/imgcodecs.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace {
void Check(bool ok, const std::string &message) { if(!ok) throw std::runtime_error(message); }
void Near(double actual, double expected, const std::string &message) {
    Check(std::abs(actual-expected)<1e-8,message+": "+std::to_string(actual));
}
// Mặt đường (lớp 1) cột a..b, mọi hàng.
cv::Mat Strip(int h, int w, int a, int b) {
    cv::Mat mask(h,w,CV_8UC1,cv::Scalar(0));
    mask.colRange(a,b+1).setTo(1);
    return mask;
}
control::ControlParams Params() {
    control::ControlParams p;
    p.max_speed_cms=10; p.cruise_cms=8; p.slow_per_deg=0.1; p.obstacle_action=0;
    // 624c8bd đổi kp mặc định 30→35; ca PD đặt hệ số tường minh để kiểm phép tính.
    // Cùng commit đổi tuổi mặt nạ mặc định 300→400 ms; ca biên dưới đây kiểm mốc 300 ms.
    p.kp=30; p.mask_max_age_ms=300;
    p.lane1_width_frac={0.4,0.4}; p.kd=0; p.steer_rate_dps=1000;
    p.max_jump_frac=0.5;  // ca PD/rate đổi làn đột ngột; giới hạn nhảy có ca riêng (acc_t79_A10)
    return p;
}
void MakeRun(const std::string &run) {
    { std::ofstream session(run+"/session.json"); session << "{\"synthetic\":true}"; Check(bool(session),"Không ghi được session"); }
    std::ofstream log(run+"/control_log.csv");
    log << "frame_id,t_ms,mask_ms,image_ms,mask_file,cmd_cms,cmd_deg,go,stop,lane_ref\n";
    for(int i=0;i<3;++i) {
        const std::string frame=std::to_string(i+1);
        Check(cv::imwrite(run+"/masks/"+frame+".png",Strip(60,200,50,150+i*5)),"Không ghi được mặt nạ giả");
    }
    log << "1,0,0,0,\"masks/1.png\",0,0,1,0,RIGHT\n"
        << "2,600,600,0,\"masks/2.png\",0,0,0,0,RIGHT\n"
        << "2,650,650,650,\"masks/2.png\",0,0,0,0,RIGHT\n"
        << "2,700,0,700,,0,0,1,0,RIGHT\n"
        << "3,750,750,750,\"masks/3.png\",0,0,0,0,RIGHT\n"
        << "3,800,800,800,\"masks/3.png\",0,0,1,0,RIGHT\n";
    Check(bool(log),"Không ghi được control_log");
    std::ofstream legacy(run+"/control_log_legacy.csv");
    legacy << "frame_id,t_ms,mask_ms,image_ms,mask_file,cmd_cms,cmd_deg\n"
           << "0,0,,,,0,0\n"
           << "1,0,0,0,masks/1.png,0,0\n2,600,600,0,masks/2.png,0,0\n"
           << "2,650,650,650,masks/2.png,0,0\n2,700,0,700,,0,0\n"
           << "3,750,750,750,masks/3.png,0,0\n3,800,800,800,masks/3.png,0,0\n";
    Check(bool(legacy),"Không ghi được log thiếu sự kiện");
}
void CheckReplay(const std::string &out, bool legacy=false) {
    std::ifstream log(out+"/control_log.csv"); std::string line;
    Check(bool(std::getline(log,line)) && line.find("row_target")!=std::string::npos && line.find("old_cmd_deg")!=std::string::npos,"CSV thiếu debug hoặc so sánh");
    const std::vector<std::string> names=legacy ? std::vector<std::string>{"1_2","2_3","2_4","3_6","3_7"} :
        std::vector<std::string>{"1_1","2_2","2_3","3_5","3_6"};
    for(size_t i=0;i<names.size();++i) {
        Check(bool(std::getline(log,line)),"Replay thiếu dòng");
        const bool moving=i==0 || (!legacy && i==4);
        Check(line.find(moving ? "BAM_LAN" : "DUNG")!=std::string::npos,"Replay không giữ đúng chốt MAT_ANH/G bị bỏ");
        std::istringstream row(line); std::string field;
        for(int j=0;j<5;++j) Check(bool(std::getline(row,field,',')),"CSV thiếu cột lệnh");
        const double speed=std::stod(field);
        Check(moving ? speed>0 : speed==0,"Replay sai lệnh khi mất ảnh/phục hồi/G bị bỏ");
        const auto image=cv::imread(out+"/overlays/"+names[i]+".png");
        Check(!image.empty() && image.rows==180 && image.cols==600,"Ảnh phủ thiếu hoặc sai kích thước");
        Check(image.at<cv::Vec3b>(50,300)==cv::Vec3b(0,255,255),"Ảnh phủ thiếu trục giữa");
    }
    Check(!std::getline(log,line),"Replay thừa dòng");
}
void WideCornerUnit() {
    auto p=Params(); control::LaneController c;
    // Cua L: nửa gần kín đường, nửa xa chỉ còn một cạnh thật.
    for(int sign:{-1,1}) {
        auto bend=Strip(100,200,0,199);
        if(sign<0) bend.rowRange(0,75).colRange(120,200).setTo(0);
        else bend.rowRange(0,75).colRange(0,80).setTo(0);
        c.Reset(); const auto turn=c.Step(bend,0,0,p);
        Check(turn.debug.state=="BAM_LAN" && turn.cmd.cms>0,
              "Cua L hai cạnh ngoài khung phải còn mục tiêu");
        Check(turn.debug.e*sign>0 && turn.cmd.deg*sign<0,
              "Tâm vùng đường và hướng xa phải lái đúng chiều");
        Check(turn.debug.valid_rows==p.corridor_rows,
              "Cua L phải dùng cả tâm đường gần, không chỉ cạnh xa");
        control::LaneController off;
        const auto held=off.Step(bend,0,0,p,control::TrajMode::LANE,false);
        Check(held.debug.valid_rows<p.corridor_rows,"Tắt nhánh tâm vùng đường (có ý định rẽ) giữ cách tính cũ");
        const cv::Mat gone(100,200,CV_8UC1,cv::Scalar(0));
        const auto stop=c.Step(gone,p.lost_stop_ms,p.lost_stop_ms,p);
        Check(stop.cmd.cms==0 && stop.cmd.deg==0 && stop.debug.state=="MAT_LAN",
              "Đường thật hết vẫn dừng đúng hạn");
    }
    for(int sign:{-1,1}) {
        c.Reset(); const auto edge=c.Step(Strip(100,200,sign<0?0:80,sign<0?120:199),0,0,p);
        Near(edge.debug.rows.front().target,sign<0?80:120,"Một cạnh ngoài giữ cách dựng cũ");
        Near(edge.debug.e,sign<0?-0.2:0.2,"Một cạnh ngoài giữ sai số cũ");
    }
    c.Reset(); const auto all_road=c.Step(Strip(100,200,0,199),0,0,p);
    Check(all_road.cmd.cms==0,"Không có hướng xa vẫn giữ mất làn cũ");
}
void Unit() {
    WideCornerUnit();
    auto p=Params(); control::LaneController c;
    const auto center=Strip(60,200,60,140), right=Strip(60,200,80,160), left=Strip(60,200,40,120);
    auto r=c.Step(center,0,0,p); Near(r.cmd.deg,0,"Thẳng giữa làn"); Near(r.cmd.cms,8,"Tốc độ thẳng");
    r=c.Step(right,100,100,p); Near(r.debug.e,0.2,"Sai số chuẩn hóa"); Near(r.cmd.deg,-6,"Tâm lệch phải phải lái âm");
    Near(r.debug.p,6,"Phần P"); Near(r.debug.d,0,"kd bằng 0"); Near(r.cmd.cms,7.4,"Tốc độ giảm theo góc");
    Check(int(r.debug.rows.size())==p.corridor_rows && r.debug.rows[0].valid,"Thiếu debug từng hàng");
    Near(r.debug.rows[0].left,80,"Biên trái"); Near(r.debug.rows[0].right,160,"Biên phải"); Near(r.debug.rows[0].target,120,"Tâm hàng");
    Near(r.debug.look_x,120,"Lookahead"); Check(r.debug.mode=="BAM_LAN" && r.debug.fit.size()==3,"Thiếu mode/đường khớp");
    r=c.Step(left,200,200,p); Near(r.cmd.deg,6,"Tâm lệch trái phải lái dương");
    p.kp=200; p.steer_max_deg=12; p.slow_per_deg=1; p.turn_enable=0; // góc 12 < turn_steer_deg chỉ hợp lệ khi tắt rẽ
    r=c.Step(right,300,300,p); Near(r.cmd.deg,-12,"Kẹp steer_max_deg"); Near(r.cmd.cms,p.min_cms,"Tốc độ không dưới min_cms");
    c.Reset(); p=Params(); p.steer_rate_dps=10;
    r=c.Step(right,0,0,p); Near(r.cmd.deg,-0.5,"Rate vòng đầu theo control_period_ms");
    r=c.Step(right,100,100,p); Near(r.cmd.deg,-1.5,"steer_rate_dps nhân dt");
    r=c.Step(left,100,100,p); Near(r.cmd.deg,-1.5,"Cùng thời điểm không đổi góc");
    c.Reset(); p=Params(); p.kd=2;
    c.Step(center,0,0,p); r=c.Step(right,100,100,p);
    Near(r.debug.d,4,"D theo thời gian thật giữa mẫu"); Near(r.cmd.deg,-10,"P cộng D có dấu âm");
    c.Reset(); p=Params();
    c.Step(right,0,0,p);
    const cv::Mat empty(60,200,CV_8UC1,cv::Scalar(0));
    r=c.Step(empty,299,299,p); Near(r.cmd.cms,4,"Mất làn trước lost_hold_ms"); Near(r.cmd.deg,-6,"Giữ góc cũ");
    r=c.Step(empty,300,300,p); Near(r.cmd.cms,0,"Đúng lost_hold_ms phải dừng tốc độ"); Near(r.cmd.deg,-6,"Giai đoạn hai còn giữ góc");
    r=c.Step(empty,799,799,p); Near(r.cmd.deg,-6,"Trước lost_stop_ms giữ góc");
    r=c.Step(empty,800,800,p); Near(r.cmd.cms,0,"Mất làn dừng"); Near(r.cmd.deg,0,"Đúng lost_stop_ms trả góc 0");
    Check(r.debug.state=="MAT_LAN","Thiếu trạng thái MAT_LAN");
    r=c.Step(right,850,850,p); Near(r.cmd.cms,0,"Chưa đủ khung phục hồi");
    r=c.Step(right,850,875,p); Near(r.cmd.cms,0,"Không đếm lại cùng mặt nạ");
    c.Step(right,900,900,p);
    r=c.Step(right,850,910,p); Near(r.cmd.cms,0,"Timestamp quay lùi không được tính thêm khung phục hồi");
    r=c.Step(empty,925,925,p); Near(r.cmd.cms,0,"Khung hỏng ngắt chuỗi phục hồi");
    r=c.Step(right,900,940,p); Check(r.debug.state=="MAT_NAN_CU","Không dùng lại mask trước khung hình học hỏng");
    c.Step(right,950,950,p); c.Step(right,1000,1000,p); r=c.Step(right,1050,1050,p);
    Check(r.debug.state=="BAM_LAN" && r.cmd.cms>0,"Đủ regain_frames tự chạy lại"); Near(r.debug.d,0,"Phục hồi không xung D");
    c.Reset(); r=c.Step(empty,0,0,p); Near(r.cmd.cms,0,"Chưa từng có làn không chạy mò");
    c.Reset(); c.Step(right,0,0,p);
    r=c.Step(right,0,300,p); Check(r.debug.state=="BAM_LAN","Tuổi bằng hạn vẫn hợp lệ");
    r=c.Step(right,0,301,p); Check(r.debug.state=="MAT_NAN_CU","Quá hạn phải MAT_NAN_CU");
    Near(r.cmd.cms,0,"Mask cũ tốc độ 0"); Near(r.cmd.deg,0,"Mask cũ góc 0");
    r=c.Step(right,350,350,p); Check(r.cmd.cms>0,"Mặt nạ tươi tự phục hồi sau MAT_NAN_CU");
    c.Reset(); r=c.Step(right,101,100,p); Check(r.debug.state=="MAT_NAN_CU","Mặt nạ từ tương lai phải dừng");
    cv::Mat surface(60,200,CV_8UC1,cv::Scalar(0));
    for(int y=0;y<60;++y) for(int x=80;x<160;++x) surface.at<unsigned char>(y,x)=1;
    c.Reset(); r=c.Step(surface,0,0,p); Near(r.debug.rows[0].target,119.5,"Mặt đường cột 80..159: tâm 119,5");
    for(int y=0;y<60;++y) for(int x=185;x<195;++x) surface.at<unsigned char>(y,x)=1;
    c.Reset(); r=c.Step(surface,0,0,p); Near(r.debug.rows[0].target,119.5,"Bỏ mảng đường rời xa trục xe");
    c.Reset(); const auto small=c.Step(right,0,0,p);
    c.Reset(); const auto large=c.Step(Strip(120,400,160,320),0,0,p);
    Near(small.debug.e,large.debug.e,"Không viết cứng 96x160"); Near(small.cmd.deg,large.cmd.deg,"Góc bất biến theo tỉ lệ ảnh");
    for(int sign:{-1,1}) {
        // Cua: tâm đường dịch theo y, phía xa lệch về sign -> e cùng dấu sign, góc ngược dấu.
        cv::Mat curve(60,200,CV_8UC1,cv::Scalar(0));
        for(int y=0;y<60;++y) { const int mid=100+sign*(60-y)/2; curve.row(y).colRange(mid-40,mid+41).setTo(1); }
        c.Reset(); r=c.Step(curve,0,0,p);
        Check(r.debug.e*sign>0 && r.cmd.deg*sign<0,"Dấu e/góc khi cua");
    }
    c.Reset(); p.center_offset_frac=0.1; r=c.Step(right,0,0,p); Near(r.cmd.deg,0,"Dịch tâm theo tỉ lệ W");
    p=Params(); p.min_valid_rows=3; cv::Mat partial=right.clone(); partial.rowRange(0,52).setTo(0);  // chỉ còn 2 hàng hành lang
    c.Reset(); r=c.Step(partial,0,0,p); Near(r.cmd.cms,0,"Chưa đủ min_valid_rows không chạy");
    c.Reset(); p=Params(); c.Step(right,0,0,p); c.Reset(); r=c.Step(center,100,100,p); Near(r.cmd.deg,0,"Reset xóa góc và lịch sử");
    bool rejected=false; try { c.Step(cv::Mat(2,2,CV_32FC1),100,100,p); } catch(const std::exception &) { rejected=true; }
    Check(rejected,"Phải từ chối mask sai kiểu");
    control::Planner planner; control::PlannerInput in; in.mask=center; in.image_ms=0;
    auto plan=planner.Step(in,0,p); Check(plan.state==control::PlannerState::CHO_SAN_SANG && plan.cmd.cms==0,"Planner khởi động chờ");
    in.go=true; in.mask_ms=50; plan=planner.Step(in,50,p); Check(plan.state==control::PlannerState::BAM_LAN && plan.cmd.cms>0,"G bắt đầu bám làn");
    in.stop=true; plan=planner.Step(in,100,p); Check(plan.state==control::PlannerState::DUNG && plan.cmd.cms==0 && plan.cmd.deg==0,"Space thắng G");
    in.go=false; in.stop=false; plan=planner.Step(in,150,p); Near(plan.cmd.cms,0,"Dừng chờ G mới");
    in.go=true; in.mask_ms=200; plan=planner.Step(in,200,p); Check(plan.cmd.cms>0,"G chạy lại");
    planner.Reset(); in.go=false; plan=planner.Step(in,250,p); Check(plan.state==control::PlannerState::CHO_SAN_SANG,"Planner Reset");
    in.go=true; in.image_ms=1000; in.mask_ms=0; plan=planner.Step(in,1000,p);
    Check(plan.state==control::PlannerState::CHO_SAN_SANG && plan.reason=="go ignored MAT_NAN_CU","G với mặt nạ cũ phải bị bỏ");
    in.go=false; in.image_ms=1050; in.mask_ms=1050; plan=planner.Step(in,1050,p);
    Near(plan.cmd.cms,0,"Dữ liệu tươi không được giữ lại G đã bỏ");
    in.go=true; in.image_ms=0; in.mask_ms=1100; plan=planner.Step(in,1100,p);
    Check(plan.reason=="go ignored MAT_ANH" && plan.cmd.cms==0,"G với ảnh cũ phải bị bỏ");
    in.image_ms=1100; plan=planner.Step(in,1100,p); Check(plan.cmd.cms>0,"G mới với dữ liệu tươi được nhận");
    in.go=false; in.mask_ms=1700; plan=planner.Step(in,1700,p);
    Check(plan.state==control::PlannerState::DUNG && plan.reason=="MAT_ANH" && plan.cmd.cms==0,"Mất ảnh chốt dừng");
    in.image_ms=1750; in.mask_ms=1750; plan=planner.Step(in,1750,p);
    Check(plan.state==control::PlannerState::DUNG && plan.reason=="MAT_ANH","Ảnh về vẫn đứng tới G");
    in.go=true; plan=planner.Step(in,1750,p); Check(plan.cmd.cms>0,"G mới mở chốt mất ảnh");
}
}
int main(int argc, char **argv) {
    try {
        if(argc==3 && std::string(argv[1])=="--make-run") MakeRun(argv[2]);
        else if(argc==3 && std::string(argv[1])=="--check-replay") CheckReplay(argv[2]);
        else if(argc==3 && std::string(argv[1])=="--check-replay-legacy") CheckReplay(argv[2],true);
        else { Check(argc==1,"Tham số test lạ"); Unit(); }
        std::cout << "ĐẠT: điều khiển, thời gian giả, Planner hoặc run replay\n";
        return 0;
    } catch(const std::exception &error) { std::cerr << "KHÔNG ĐẠT: " << error.what() << '\n'; return 1; }
}
