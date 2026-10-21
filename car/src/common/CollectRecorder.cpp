#include "common/CollectRecorder.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace common {
namespace {
const char *header="frame,session,pass,luot,target,t,cmd_speed,cmd_steer,meas_speed,odom_cm,yaw_rel,pitch,roll,tof0,tof1,tof2,tof3\n";
std::string Json(const std::string &s) {
    std::ostringstream out; out << '"';
    for(unsigned char ch:s) {
        if(ch=='"' || ch=='\\') out << '\\' << ch;
        else if(ch<32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(ch);
        else out << ch;
    }
    out << '"'; return out.str();
}
std::string Ask(std::istream &in,std::ostream &out,const char *prompt) {
    out << prompt << std::flush; std::string value;
    if(!std::getline(in,value) || value.empty()) throw std::runtime_error("Thiếu thông tin phiên thu; chạy trình bao ở terminal có đầu vào");
    if(!value.empty()&&value.back()=='\r') value.pop_back();
    if(value.empty()) throw std::runtime_error("Thông tin phiên không được trống");
    return value;
}
std::string Choice(std::istream &in,std::ostream &out,const char *prompt,const std::vector<std::string> &values) {
    for(;;) {
        const auto answer=Ask(in,out,prompt);
        if(answer.size()==1 && answer[0]>='1' && answer[0]<'1'+int(values.size())) return values[answer[0]-'1'];
        out << "Chọn số tương ứng trong danh sách.\n";
    }
}
}
CollectRecorder::CollectRecorder(const Config &c,const std::string &directory,std::istream &input,std::ostream &output)
    :c_(c),directory_(directory),output_(output),fps_(c.collect_fps) {
    output << "Thông tin phiên thu (xe phải đứng yên; chưa khởi tạo thiết bị):\n";
    std::string who;
    do { who=Ask(input,output,"Mã người thu, một chữ thường a–z: "); }
    while(who.size()!=1 || who[0]<'a' || who[0]>'z');
    const auto name=Ask(input,output,"Tên người thu: ");
    const auto weather=Choice(input,output,"Thời tiết 1=nắng, 2=râm, 3=mưa: ",{"nắng","râm","mưa"});
    const auto curtain=Choice(input,output,"Rèm 1=mở, 2=đóng: ",{"mở","đóng"});
    const auto lights=Choice(input,output,"Đèn phòng 1=bật, 2=tắt: ",{"bật","tắt"});
    const auto props=Ask(input,output,"Đạo cụ đang có (ghi 'không' nếu không có): ");
    const auto stamp=std::time(nullptr); std::tm tm{}; localtime_r(&stamp,&tm);
    std::ostringstream id,time; id << who << std::put_time(&tm,"%m%d%H%M"); time << std::put_time(&tm,"%Y-%m-%dT%H:%M:%S%z");
    session_=id.str();
    // Thư mục phiên do bộ ghi tạo độc quyền, không nối dữ liệu vào phiên cũ.
    std::ofstream json(directory_+"/session.json");
    json << "{\n  \"session\": " << Json(session_) << ",\n  \"collector_code\": " << Json(who)
         << ",\n  \"collector_name\": " << Json(name) << ",\n  \"time\": " << Json(time.str())
         << ",\n  \"weather\": " << Json(weather) << ",\n  \"curtain\": " << Json(curtain)
         << ",\n  \"room_lights\": " << Json(lights) << ",\n  \"props\": " << Json(props) << "\n}\n";
    json.close(); if(!json) throw std::runtime_error("Không ghi được session.json; kiểm tra dung lượng/quyền ghi");
    RewriteLog();
    output << "Phiên " << session_ << ", lưu tại " << directory_ << "\nBảng phím đang dùng:\n";
    const char *labels[]={"tăng tốc","giảm tốc","trái","phải","dừng xe","bắt đầu/dừng đợt ghi",
        "xóa đợt cuối (hai lần)","thoát (hai lần)","giảm mục tiêu","tăng mục tiêu","tăng ảnh/giây","giảm ảnh/giây"};
    for(std::size_t i=0;i<c_.collect_keys.size();++i) output << '[' << c_.collect_keys[i] << "] " << labels[i] << '\n';
    output << "Phím chọn lượt 1–9 theo thứ tự: " << c_.collect_luot_keys << '\n';
}
void CollectRecorder::RewriteLog() const {
    const auto temp=directory_+"/capture_log.csv.tmp";
    std::ofstream out(temp); out << header;
    for(const auto &row:rows_) out << row.csv << '\n';
    out.close();
    if(!out || std::rename(temp.c_str(),(directory_+"/capture_log.csv").c_str())!=0)
        throw std::runtime_error("Không cập nhật được nhật ký thu; kiểm tra đĩa và quyền ghi");
}
void CollectRecorder::Undo() {
    if(recording_) { output_ << "Dừng ghi trước khi xóa đợt cuối.\n"; return; }
    if(!last_pass_) { output_ << "Không có đợt vừa ghi để xóa.\n"; return; }
    for(const auto &row:rows_) if(row.pass==last_pass_) {
        if(std::remove((directory_+"/"+row.frame).c_str())!=0)
            throw std::runtime_error("Không xóa được ảnh của đợt cuối: "+row.frame);
    }
    rows_.erase(std::remove_if(rows_.begin(),rows_.end(),[this](const Row &r){return r.pass==last_pass_;}),rows_.end());
    RewriteLog(); output_ << "Đã xóa đợt " << last_pass_ << "; giữ nguyên các đợt trước.\n";
    last_pass_=0; count_=0; previous_.release();
}
bool CollectRecorder::Key(int key,ControlCommand &cmd,Clock::time_point now) {
    if(key<0) return false;
    if(key>='A'&&key<='Z') key+='a'-'A';
    auto is=[&](int i){return key==(c_.collect_keys[i]=="esc" ? 27 : int(c_.collect_keys[i][0]));};
    const bool confirm_undo=is(6)&&undo_armed_&&now-undo_at_<=std::chrono::milliseconds(c_.collect_confirm_ms);
    const bool confirm_quit=is(7)&&quit_armed_&&now-quit_at_<=std::chrono::milliseconds(c_.collect_confirm_ms);
    undo_armed_=quit_armed_=false;
    if(is(7)) {
        cmd={};
        if(confirm_quit) return true;
        quit_at_=now; quit_armed_=true; output_ << "Đã yêu cầu dừng xe. Bấm lại phím thoát để kết thúc.\n";
    } else if(is(6)) {
        if(confirm_undo) Undo();
        else { undo_at_=now; undo_armed_=true; output_ << "Bấm lại phím xóa để xóa đợt vừa ghi.\n"; }
    } else if(is(0)) cmd.speedCmPerSec+=c_.keyboard_speed_step_cms;
    else if(is(1)) cmd.speedCmPerSec-=c_.keyboard_speed_step_cms;
    else if(is(2)) cmd.steeringDeg-=c_.keyboard_steering_step_deg;
    else if(is(3)) cmd.steeringDeg+=c_.keyboard_steering_step_deg;
    else if(is(4)) cmd={};
    else if(is(5)) {
        recording_=!recording_;
        if(recording_) { ++pass_; count_=0; previous_.release(); last_capture_={}; }
        else last_pass_=pass_;
        output_ << (recording_ ? "Bắt đầu đợt " : "Dừng đợt ") << pass_ << ", " << count_ << " ảnh.\n";
    } else if(is(10)) fps_=std::min(10,fps_+1);
    else if(is(11)) fps_=std::max(1,fps_-1);
    else if(!recording_) {
        const auto pos=key<128 ? c_.collect_luot_keys.find(char(key)) : std::string::npos;
        if(pos!=std::string::npos) luot_=int(pos)+1;
        else if(is(8)) target_=std::max(1,target_-1);
        else if(is(9)) ++target_;
    } else output_ << "Đang ghi: dừng đợt trước khi đổi lượt/mục tiêu.\n";
    cmd.speedCmPerSec=std::clamp(cmd.speedCmPerSec,-c_.max_speed_cms,c_.max_speed_cms);
    cmd.steeringDeg=std::clamp(cmd.steeringDeg,-c_.max_steering_deg,c_.max_steering_deg);
    return false;
}
void CollectRecorder::Capture(const Snapshot &s,Clock::time_point now) {
    const auto &o=s.observation;
    if(!recording_ || !o.frame_id || o.frame_id==last_frame_ || o.frame.empty()) return;
    if(now-last_capture_<std::chrono::duration<double>(1.0/fps_)) return;
    last_frame_=o.frame_id; last_capture_=now;
    if(now-o.captured_at>std::chrono::milliseconds(c_.image_timeout_ms) || !o.odom_valid ||
       now-o.odom_at>std::chrono::milliseconds(c_.image_timeout_ms) ||
       !std::isfinite(o.meas_speed) || std::abs(o.meas_speed)<c_.collect_min_speed_cms) return;
    // So sánh toàn ảnh gốc; ảnh trước là ảnh được lưu thành công gần nhất trong đợt.
    if(!previous_.empty() && previous_.size()==o.frame.size() && previous_.type()==o.frame.type()) {
        cv::Mat diff; cv::absdiff(o.frame,previous_,diff); const auto mean=cv::mean(diff);
        double sum=0; for(int i=0;i<diff.channels();++i) sum+=mean[i];
        if(sum/diff.channels()<c_.collect_duplicate_threshold) return;
    }
    std::ostringstream name; name << session_ << "_p" << std::setfill('0') << std::setw(3) << pass_
        << "_l" << luot_ << "_t" << std::setw(2) << target_ << '_' << std::setw(4) << count_+1 << ".jpg";
    const auto frame=name.str(), path=directory_+"/"+frame, temp=path+".tmp.jpg";
    if(!cv::imwrite(temp,o.frame,{cv::IMWRITE_JPEG_QUALITY,c_.jpeg_quality}) || std::rename(temp.c_str(),path.c_str())!=0)
        throw std::runtime_error("Không lưu được ảnh thu; kiểm tra dung lượng và quyền ghi");
    std::ostringstream row; row << std::setprecision(10) << frame << ',' << session_ << ',' << pass_ << ',' << luot_ << ',' << target_
        << ',' << std::chrono::duration<double>(o.captured_at-started_).count() << ',' << s.applied.speedCmPerSec
        << ',' << s.applied.steeringDeg << ',' << o.meas_speed << ',' << o.odom_cm << ',' << o.yaw_rel_deg << ',' << o.pitch_deg << ',' << o.roll_deg;
    for(const auto &t:o.tof) row << ',' << (t.valid&&!t.timedOut && now-t.updatedAt<=std::chrono::milliseconds(c_.tof_timeout_ms) ? t.distanceMm : -1);
    std::ofstream out(directory_+"/capture_log.csv",std::ios::app); out << row.str() << '\n'; out.close();
    if(!out) throw std::runtime_error("Không ghi được nhật ký thu");
    rows_.push_back({pass_,frame,row.str()}); ++count_; previous_=o.frame.clone();
}
cv::Mat CollectRecorder::View(const Snapshot &s) const {
    if(s.observation.frame.empty()) return {};
    auto view=s.observation.frame.clone();
    std::ostringstream line1,line2,line3;
    line1 << (recording_ ? "DANG GHI" : "SAN SANG") << " | dot " << pass_ << " luot " << luot_ << " muc tieu " << target_;
    line2 << count_ << " anh | " << fps_ << " anh/s | do " << s.observation.meas_speed << " cm/s | " << s.state;
    line3 << "ToF mm:"; for(const auto &t:s.observation.tof) line3 << ' ' << (t.valid&&!t.timedOut ? t.distanceMm : -1);
    // Phông Hershey không có dấu tiếng Việt; chỉ bản xem dùng chữ không dấu.
    const std::vector<std::string> lines{line1.str(),line2.str(),line3.str()};
    const double scale=std::max(0.2,view.cols/1300.0);
    for(std::size_t i=0;i<lines.size();++i) {
        const cv::Point p(int(view.cols*0.015),int(view.rows*(0.06+0.065*i)));
        cv::putText(view,lines[i],p,cv::FONT_HERSHEY_SIMPLEX,scale,cv::Scalar(0,0,0),3);
        cv::putText(view,lines[i],p,cv::FONT_HERSHEY_SIMPLEX,scale,cv::Scalar(0,255,255),1);
    }
    return view;
}
}
