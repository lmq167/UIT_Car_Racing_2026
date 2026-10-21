#include "common/CollectRecorder.h"
#include "app/Backend.h"
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace {
void Check(bool ok,const char *text) { if(!ok) throw std::runtime_error(text); }
std::string Read(const std::string &path) { std::ifstream in(path); return {std::istreambuf_iterator<char>(in),std::istreambuf_iterator<char>()}; }
}
int main() {
    char pattern[]="/tmp/ucr collect.XXXXXX";
    const char *created=mkdtemp(pattern); Check(created,"không tạo được thư mục tạm");
    const std::string dir=created;
    // Chỉ dọn đúng các tệp được tạo trong thư mục kiểm tra độc quyền.
    struct Cleanup {
        std::string dir; std::vector<std::string> names;
        ~Cleanup() { for(const auto &name:names) std::remove((dir+"/"+name).c_str()); rmdir(dir.c_str()); }
    } cleanup{dir,{"capture_log.csv","capture_log.csv.tmp","session.json","config.yaml"}};
    auto load=[&](const std::string &body) {
        std::ofstream out(dir+"/config.yaml"); out << "%YAML:1.0\n---\n" << body << '\n'; out.close();
        common::Config value; value.Load(dir+"/config.yaml"); return value;
    };
    for(const auto &body:std::vector<std::string>{"collect_fps: 0","collect_fps: 11","collect_min_speed_cms: 0",
            "collect_duplicate_threshold: -1","collect_confirm_ms: 0","collect_luot_keys: \"12345678w\"",
            "collect_keys: [\"w\"]"}) {
        bool rejected=false; try { load(body); } catch(const std::exception &) { rejected=true; }
        Check(rejected,"từ chối cấu hình thu sai giới hạn hoặc trùng phím");
    }
    Check(load("collect_fps: 10").collect_fps==10,"chấp nhận nhịp biên 10 ảnh/giây");
    common::Config c; c.collect_fps=10;
    std::istringstream input("a\nNguyễn \"An\"\n2\n1\n1\nbiển, vật cản\n"); std::ostringstream output;
    common::CollectRecorder recorder(c,dir,input,output);
    Check(Read(dir+"/session.json").find("Nguyễn \\\"An\\\"")!=std::string::npos,"thoát ký tự JSON");
#ifdef UCR_FAKE_HW
    int source_index=0;
    for(int value:{40,40,40,80,120}) {
        const auto name="input_0"+std::to_string(source_index++)+".jpg";
        cleanup.names.push_back(name);
        Check(cv::imwrite(dir+"/"+name,cv::Mat(80,120,CV_8UC3,cv::Scalar(value,value,value))),"tạo ảnh nguồn giả");
    }
    c.fake_source=dir+"/input_%02d.jpg";
    app::Backend backend(c); backend.Init();
#endif
    auto now=common::Clock::now(); common::Snapshot s; common::ControlCommand cmd;
    auto capture=[&](int value,double speed) {
        now+=std::chrono::milliseconds(150);
#ifdef UCR_FAKE_HW
        Check(backend.Frame(s.observation.frame),"camera đọc thư mục ảnh có dấu cách");
#else
        s.observation.frame=cv::Mat(80,120,CV_8UC3,cv::Scalar(value,value,value));
#endif
        s.observation.captured_at=s.observation.odom_at=now;
        ++s.observation.frame_id; s.observation.odom_valid=true; s.observation.meas_speed=speed;
        recorder.Capture(s,now);
    };
    recorder.Key('c',cmd,now); capture(40,0);
    Check(recorder.Count()==0,"không lưu khi đứng yên");
    capture(40,5); capture(40,5);
    Check(recorder.Count()==1,"bỏ ảnh trùng");
    recorder.Capture(s,now+std::chrono::milliseconds(200));
    Check(recorder.Count()==1,"không lưu lại cùng mã khung");
    // Bản xem không thay đổi ảnh dùng để lưu.
    const auto original=s.observation.frame.clone(); recorder.View(s);
    Check(cv::norm(original,s.observation.frame,cv::NORM_INF)==0,"bản xem không sửa ảnh gốc");
    recorder.Key('c',cmd,now);
    recorder.Key('4',cmd,now); recorder.Key(']',cmd,now); recorder.Key(']',cmd,now);
    recorder.Key('c',cmd,now); capture(80,5); capture(120,-5);
    Check(recorder.Count()==2&&recorder.Pass()==2,"hai đợt ghi");
    recorder.Key('u',cmd,now); recorder.Key('u',cmd,now);
    Check(recorder.Count()==2,"không xóa khi đang ghi");
    recorder.Key('c',cmd,now);
    const auto before=Read(dir+"/capture_log.csv");
    std::istringstream lines(before); std::string line; std::getline(lines,line);
    Check(line=="frame,session,pass,luot,target,t,cmd_speed,cmd_steer,meas_speed,odom_cm,yaw_rel,pitch,roll,tof0,tof1,tof2,tof3","17 cột nhật ký");
    std::vector<std::string> names;
    while(std::getline(lines,line)) {
        const auto name=line.substr(0,line.find(',')); names.push_back(name); cleanup.names.push_back(name);
        Check(std::count(line.begin(),line.end(),',')==16,"mỗi dòng ảnh có 17 giá trị");
        Check(std::regex_match(name,std::regex("a[0-9]{8}_p[0-9]{3}_l[1-9]_t[0-9]{2}_[0-9]{4}\\.jpg")),"tên ảnh đúng quy ước");
        Check(!cv::imread(dir+"/"+name).empty(),"ảnh ghi đọc được ở đường dẫn có dấu cách");
    }
    Check(names.size()==3,"một ảnh đợt đầu, hai ảnh đợt sau");
    Check(names[1].find("_p002_l4_t03_0001")!=std::string::npos,"lượt và mục tiêu gắn trong tên ảnh");
    recorder.Key('u',cmd,now);
    Check(Read(dir+"/capture_log.csv")==before,"bấm một lần chưa xóa");
    recorder.Key('u',cmd,now);
    const auto after=Read(dir+"/capture_log.csv");
    Check(after.find(names[0])!=std::string::npos && after.find(names[1])==std::string::npos,"chỉ xóa dòng đợt cuối");
    Check(std::ifstream(dir+"/"+names[0]).good()&&!std::ifstream(dir+"/"+names[1]).good(),"chỉ xóa ảnh đợt cuối");
    recorder.Key('u',cmd,now); recorder.Key('u',cmd,now);
    Check(Read(dir+"/capture_log.csv")==after,"không xóa lùi đợt cũ");
    cmd.speedCmPerSec=10;
    Check(!recorder.Key(27,cmd,now)&&cmd.speedCmPerSec==0,"thoát lần đầu yêu cầu dừng xe");
    Check(!recorder.Key(27,cmd,now+std::chrono::milliseconds(c.collect_confirm_ms+1)),"xác nhận hết hạn");
    Check(recorder.Key(27,cmd,now+std::chrono::milliseconds(c.collect_confirm_ms+2)),"thoát khi xác nhận đúng hạn");
#ifdef UCR_FAKE_HW
    backend.Apply({5,0}); usleep(30000); float position=0;
    Check(backend.Position(position)&&position>0,"encoder giả theo lệnh chấp hành");
    backend.Stop(); backend.Position(position); const auto stopped=position;
    usleep(10000); backend.Position(position);
    Check(position==stopped,"encoder giả không tăng sau dừng");
#endif
    std::cout << "Đạt kiểm tra hai đợt ghi, nhật ký, xóa đợt cuối và nguồn ảnh giả\n";
}
