#include "common/RunLogger.h"
#include <opencv2/imgcodecs.hpp>
#include <cerrno>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>
#include <unistd.h>
namespace common {
namespace {
void MakeDirs(const std::string &path) {
    if(path.empty()) throw std::runtime_error("Thu muc log rong");
    for(std::size_t i=1;i<=path.size();++i) {
        if(i!=path.size() && path[i]!='/') continue;
        const std::string part=path.substr(0,i);
        if(::mkdir(part.c_str(),0755)!=0 && errno!=EEXIST) throw std::runtime_error("Khong tao duoc thu muc: "+part);
    }
}
}
RunLogger::RunLogger(const Config &c,const std::string &source,const std::string &mode):c_(c),collect_(mode=="collect") {
    std::time_t now=std::time(nullptr); std::tm tm{}; localtime_r(&now,&tm);
    std::ostringstream name; name << std::put_time(&tm,"%Y%m%d_%H%M%S") << '_' << mode;
    MakeDirs(c.log_dir); directory_=c.log_dir+"/"+name.str();
    // Tạo độc quyền để không ghi đè lượt khác khởi động trong cùng giây.
    if(::mkdir(directory_.c_str(),0755)!=0) {
        directory_+="_"+std::to_string(::getpid());
        if(::mkdir(directory_.c_str(),0755)!=0) throw std::runtime_error("Trung thu muc luot chay");
    }
    std::ifstream in(source,std::ios::binary); std::ofstream out(directory_+"/config.yaml",std::ios::binary);
    if(!in || !out) throw std::runtime_error("Khong sao chep duoc config");
    out << in.rdbuf(); out.close(); if(!out) throw std::runtime_error("Loi ghi ban sao config");
    csv_.open(directory_+"/log.csv");
    csv_ << "t,frame_id,image_age_ms,cmd_speed,cmd_steer,meas_speed,odom_cm,yaw_rel,pitch,roll,tof0,tof1,tof2,tof3,state,fps\n";
    if(!csv_) throw std::runtime_error("Khong mo duoc CSV");
    std::cout << "Log: " << directory_ << '\n';
    writer_=std::thread(&RunLogger::ImageThread,this);
}
RunLogger::~RunLogger() { Close(); }
void RunLogger::Close() {
    { std::lock_guard<std::mutex> lock(mutex_); done_=true; } cv_.notify_all();
    if(writer_.joinable()) writer_.join();
    if(csv_.is_open()) { csv_.flush(); csv_.close(); }
}
void RunLogger::Record(const Snapshot &s) {
    const auto &o=s.observation; auto now=Clock::now();
    const double age=o.frame_id ? std::chrono::duration<double,std::milli>(now-o.captured_at).count() : -1;
    csv_ << std::chrono::duration<double>(now-started_).count() << ',' << o.frame_id << ',' << age << ','
         << s.applied.speedCmPerSec << ',' << s.applied.steeringDeg << ',' << o.meas_speed << ',' << o.odom_cm << ','
         << o.yaw_rel_deg << ',' << o.pitch_deg << ',' << o.roll_deg;
    for(const auto &t:o.tof) csv_ << ',' << (t.valid&&!t.timedOut ? t.distanceMm : -1);
    csv_ << ',' << s.state << ',' << s.fps << '\n'; csv_.flush();
    if(!csv_) throw std::runtime_error("Loi ghi CSV");
    if(!collect_ && o.frame_id && o.frame_id!=last_saved_ && o.frame_id%c_.image_every_n==0 && !o.frame.empty()) {
        std::lock_guard<std::mutex> lock(mutex_);
        if(queue_.size()<static_cast<std::size_t>(c_.image_queue_size)) queue_.emplace_back(o.frame_id,o.frame);
        last_saved_=o.frame_id;
    }
    cv_.notify_one();
}
void RunLogger::ImageThread() {
    try {
        for(;;) {
            std::pair<std::uint64_t,cv::Mat> item;
            { std::unique_lock<std::mutex> lock(mutex_); cv_.wait(lock,[this]{return done_||!queue_.empty();});
              if(queue_.empty()&&done_) return; item=std::move(queue_.front()); queue_.pop_front(); }
            if(!cv::imwrite(directory_+"/frame_"+std::to_string(item.first)+".jpg",item.second,
                    {cv::IMWRITE_JPEG_QUALITY,c_.jpeg_quality})) throw std::runtime_error("Loi ghi JPEG");
        }
    } catch(const std::exception &e) { std::cerr << e.what() << '\n'; failed_=true; }
}
}
