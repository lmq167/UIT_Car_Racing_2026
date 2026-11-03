/* Hợp đồng runs_auto P1 (UTF-8, CSV có tiêu đề, dấu phẩy, chuỗi có thể đặt trong ").
   Bắt buộc: session.json (object JSON), control_log.csv, masks/<frame_id>.png CV_8UC1 lớp 0/1/2.
   Cột bắt buộc: frame_id,t_ms,mask_file,cmd_cms,cmd_deg (lệnh đã gửi, cm/s và độ, + trái).
   t_ms là ms đơn điệu cùng gốc với mask_ms; frame_id chỉ gồm chữ số.
   mask_file là đường tương đối masks/<frame_id>.png; để trống khi không lưu mặt nạ.
   Cột tùy chọn: mask_ms (mặc định t_ms ở dòng có mask, thời điểm ảnh gốc),
   image_ms (mặc định t_ms nếu không có cột; thiếu cột thì không tái hiện mất ảnh), go,stop (0/1),
   lane_ref (RIGHT/LEFT), planner_state (CHO_SAN_SANG/BAM_LAN/DUNG).
   Có go/stop thì ghi cả hai ở MỌI dòng; stop thắng go. G ở dòng không lưu mask
   cần mask_ms để biết mặt nạ có tồn tại và tươi; G cũ bị bỏ ngay theo L7.
   Không có sự kiện thì
   chuyển planner_state sang BAM_LAN xác định G; thiếu cả hai thì chỉ thử G tại mẫu mask đầu.
   Sau MAT_ANH phải có G hoặc chuyển trạng thái mới, không tự mở chốt.
   Khi thiếu --control, nạp bản chép run/control.yaml. Cột bổ sung được bỏ qua.
   Dòng không có mask vẫn xử lý sự kiện G/Space, không gọi thuật toán trên mask giả.
   Kết quả: control_log.csv và overlays/<frame_id>_<số dòng>.png, bảng sai khác lệnh.
   shortcut: replay chỉ tính ở mẫu có mask, D theo t_ms giữa mẫu; cần log mọi mask
   nếu cần tái hiện chính xác mọi vòng điều khiển. P1 chưa đọc luật phát hiện P2.
*/
#include "control/Planner.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>
#ifdef _WIN32
#include <direct.h>
#else
#include <unistd.h>
#endif

namespace {
void Require(bool ok, const std::string &message) { if(!ok) throw std::runtime_error(message); }
bool Exists(const std::string &path) { struct stat value; return stat(path.c_str(),&value)==0; }
void MakeDir(const std::string &path) {
#ifdef _WIN32
    const int result=_mkdir(path.c_str());
#else
    const int result=mkdir(path.c_str(),0700);
#endif
    Require(result==0,"Không tạo được thư mục mới: "+path);
}
std::string Absolute(const std::string &path) {
#ifdef _WIN32
    char *resolved=_fullpath(nullptr,path.c_str(),0);
#else
    char *resolved=realpath(path.c_str(),nullptr);
#endif
    Require(resolved!=nullptr && Exists(path),"Không tìm thấy: "+path);
    std::string value(resolved); std::free(resolved);
    std::replace(value.begin(),value.end(),'\\','/');
    while(value.size()>1 && value.back()=='/' && !(value.size()==3 && value[1]==':')) value.pop_back();
    return value;
}
std::string Comparable(std::string path) {
#ifdef _WIN32
    for(char &c:path) c=char(std::tolower(static_cast<unsigned char>(c)));
#endif
    return path;
}
std::string NewOutput(std::string path, const std::string &run) {
    std::replace(path.begin(),path.end(),'\\','/');
    while(path.size()>1 && path.back()=='/') path.pop_back();
    const auto slash=path.find_last_of('/');
    const std::string name=slash==std::string::npos ? path : path.substr(slash+1);
    Require(!name.empty() && name!="." && name!=".." && name.find(':')==std::string::npos,"Tên thư mục --out không hợp lệ");
    const std::string parent=Absolute(slash==std::string::npos ? "." :
        (slash==0 ? "/" : path.substr(0,slash==2 && path[1]==':' ? 3 : slash)));
    const std::string output=parent+"/"+name;
    const auto a=Comparable(output), b=Comparable(run);
    Require(a!=b && a.compare(0,b.size()+1,b+"/")!=0,"--out phải nằm ngoài thư mục run");
    Require(!Exists(output),"--out phải là thư mục mới: "+output);
    return output;
}
std::vector<std::string> Csv(const std::string &line) {
    std::vector<std::string> fields;
    size_t i=0;
    while(true) {
        std::string field;
        if(i<line.size() && line[i]=='"') {
            ++i; bool closed=false;
            while(i<line.size()) {
                const char c=line[i++];
                if(c!='"') field+=c;
                else if(i<line.size() && line[i]=='"') { field+='"'; ++i; }
                else { closed=true; break; }
            }
            Require(closed,"CSV thiếu dấu đóng chuỗi");
            Require(i==line.size() || line[i]==',',"CSV có ký tự sau dấu đóng chuỗi");
        } else {
            while(i<line.size() && line[i]!=',') {
                Require(line[i]!='"',"CSV có dấu nháy sai vị trí");
                field+=line[i++];
            }
        }
        fields.push_back(field);
        if(i==line.size()) break;
        ++i;
    }
    return fields;
}
double Number(const std::string &value, const std::string &key) {
    size_t used=0;
    double result=0;
    try { result=std::stod(value,&used); }
    catch(const std::exception &) { throw std::runtime_error("CSV sai số: "+key); }
    Require(used==value.size() && std::isfinite(result),"CSV sai số: "+key);
    return result;
}
int64_t Milliseconds(const std::string &value, const std::string &key) {
    Require(!value.empty() && value.find_first_not_of("0123456789")==std::string::npos,"CSV sai thời gian: "+key);
    size_t used=0;
    try {
        const auto result=std::stoll(value,&used);
        Require(used==value.size(),"CSV sai thời gian: "+key);
        return result;
    } catch(const std::exception &) { throw std::runtime_error("CSV sai thời gian: "+key); }
}
bool Flag(const std::string &value, const std::string &key) {
    Require(value=="0" || value=="1","CSV cờ phải là 0/1: "+key);
    return value=="1";
}
std::string Quote(const std::string &value) {
    std::string result="\"";
    for(char c:value) { if(c=='"') result+='"'; result+=c; }
    return result+'"';
}
std::string RowField(const control::ControlDebug &debug, int field) {
    std::ostringstream out; out << std::setprecision(12);
    for(size_t i=0;i<debug.rows.size();++i) {
        if(i) out << ';';
        const auto &row=debug.rows[i];
        switch(field) {
            case 0: out << row.y; break;
            case 1: for(size_t j=0;j<row.lines.size();++j) { if(j) out << '|'; out << row.lines[j]; } break;
            case 2: out << row.left; break;
            case 3: out << row.right; break;
            case 4: out << row.center; break;
            case 5: out << int(row.valid); break;
        }
    }
    return Quote(out.str());
}
cv::Mat Overlay(const cv::Mat &mask, const control::Plan &plan) {
    cv::Mat image(mask.size(),CV_8UC3);
    for(int y=0;y<mask.rows;++y) for(int x=0;x<mask.cols;++x) {
        const auto label=mask.at<unsigned char>(y,x);
        image.at<cv::Vec3b>(y,x)=label==1 ? cv::Vec3b(40,100,40) : (label==2 ? cv::Vec3b(240,240,240) : cv::Vec3b(20,20,20));
    }
    for(const auto &row:plan.debug.rows) {
        cv::line(image,{0,row.y},{mask.cols-1,row.y},cv::Scalar(90,90,90));
        if(row.left>=0) cv::circle(image,{cvRound(row.left),row.y},1,cv::Scalar(255,100,0),-1);
        if(row.right>=0) cv::circle(image,{cvRound(row.right),row.y},1,cv::Scalar(255,100,0),-1);
        if(row.valid) cv::circle(image,{cvRound(row.center),row.y},2,cv::Scalar(0,0,255),-1);
    }
    cv::line(image,{mask.cols/2,0},{mask.cols/2,mask.rows-1},cv::Scalar(0,255,255));
    cv::resize(image,image,cv::Size(),3,3,cv::INTER_NEAREST);
    std::ostringstream label; label << plan.debug.state << " " << plan.cmd.cms << "cm/s " << plan.cmd.deg << "deg";
    cv::putText(image,label.str(),{3,15},cv::FONT_HERSHEY_SIMPLEX,0.35,cv::Scalar(255,255,255));
    return image;
}
}
int main(int argc, char **argv) {
    try {
        std::map<std::string,std::string> options;
        for(int i=1;i<argc;++i) {
            const std::string key=argv[i];
            Require(key=="--run" || key=="--control" || key=="--out","Tham số lạ: "+key);
            Require(options.count(key)==0 && i+1<argc,"Tham số trùng hoặc thiếu giá trị: "+key);
            options[key]=argv[++i];
        }
        Require(options.count("--run") && options.count("--out"),"Dùng: ucr_replay_control --run <dir> [--control <yaml>] --out <dir>");
        const auto run=Absolute(options.at("--run"));
        const auto output=NewOutput(options.at("--out"),run);
        cv::FileStorage session(run+"/session.json",cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
        Require(session.isOpened() && session.root().isMap(),"session.json phải là object JSON");
        const std::string config=options.count("--control") ? options.at("--control") : run+"/control.yaml";
        const auto params=control::ControlParams::Load(config,std::cout);
        const auto hash=control::ControlParams::FileHash(config);
        std::ifstream input(run+"/control_log.csv");
        Require(bool(input),"Không đọc được control_log.csv");
        std::string line;
        Require(bool(std::getline(input,line)),"control_log.csv rỗng");
        if(!line.empty() && line.back()=='\r') line.pop_back();
        const auto columns=Csv(line);
        std::map<std::string,size_t> indexes;
        for(size_t i=0;i<columns.size();++i) Require(indexes.emplace(columns[i],i).second,"CSV trùng cột: "+columns[i]);
        for(const char *key:{"frame_id","t_ms","mask_file","cmd_cms","cmd_deg"}) Require(indexes.count(key)>0,std::string("CSV thiếu cột: ")+key);
        Require(indexes.count("go")==indexes.count("stop"),"CSV phải có cả go và stop");
        MakeDir(output); MakeDir(output+"/overlays");
        std::ofstream out(output+"/control_log.csv");
        Require(bool(out),"Không ghi được CSV kết quả");
        out << "frame_id,t_ms,mask_ms,mask_file,cmd_cms,cmd_deg,old_cmd_cms,old_cmd_deg,planner_state,state,reason,lane_ref,e,p,d,valid_rows,row_y,row_lines,row_left,row_right,row_center,row_valid,control_hash\n" << std::setprecision(12);
        control::Planner planner;
        bool running=false;
        bool legacy_start_attempted=false;
        std::string previous_logged_state;
        int64_t previous=-1;
        size_t samples=0, rows=0;
        double sum_speed=0,sum_angle=0,max_speed=0,max_angle=0;
        while(std::getline(input,line)) {
            ++rows;
            if(!line.empty() && line.back()=='\r') line.pop_back();
            const auto values=Csv(line);
            Require(values.size()==columns.size(),"CSV sai số cột tại dòng "+std::to_string(rows+1));
            auto get=[&](const char *key, const std::string &fallback="") {
                const auto found=indexes.find(key); return found==indexes.end() ? fallback : values[found->second];
            };
            const auto now=Milliseconds(get("t_ms"),"t_ms");
            Require(now>=previous,"CSV t_ms không tăng đơn điệu"); previous=now;
            const auto mask_file=get("mask_file");
            const auto mask_text=get("mask_ms",mask_file.empty() ? "" : std::to_string(now));
            const int64_t mask_ms=mask_text.empty() ? -1 : Milliseconds(mask_text,"mask_ms");
            const auto image_text=get("image_ms",std::to_string(now));
            const int64_t image_ms=image_text.empty() ? -1 : Milliseconds(image_text,"image_ms");
            const bool image_fresh=image_ms>=0 && image_ms<=now && now-image_ms<=500;
            const bool mask_fresh=mask_ms>=0 && mask_ms<=now && now-mask_ms<=params.mask_max_age_ms;
            bool stop=false;
            if(indexes.count("go")) {
                stop=Flag(get("stop"),"stop");
                const bool go=Flag(get("go"),"go");
                if(stop || !image_fresh) running=false;
                else if(go && mask_fresh) running=true;
            } else if(indexes.count("planner_state")) {
                const auto state=get("planner_state");
                Require(state=="BAM_LAN" || state=="CHO_SAN_SANG" || state=="DUNG","CSV planner_state không hợp lệ");
                stop=state!="BAM_LAN";
                if(stop) running=false;
                else if(state!=previous_logged_state && mask_fresh) running=true;
                previous_logged_state=state;
            } else if(!legacy_start_attempted && !mask_file.empty()) {
                legacy_start_attempted=true; running=mask_fresh;
            }
            if(!image_fresh) running=false;
            control::PlannerInput data;
            data.stop=stop;
            data.image_ms=image_ms;
            // Dừng vẫn đi qua Planner kể cả dòng không lưu mặt nạ.
            if(stop || !image_fresh) planner.Step(data,now,params);
            if(mask_file.empty()) continue;
            const auto frame=get("frame_id");
            Require(!frame.empty() && frame.find_first_not_of("0123456789")==std::string::npos,"CSV frame_id phải gồm chữ số");
            Require(mask_file=="masks/"+frame+".png","CSV mask_file phải là masks/<frame_id>.png");
            data.mask=cv::imread(run+"/"+mask_file,cv::IMREAD_UNCHANGED);
            Require(!data.mask.empty() && data.mask.type()==CV_8UC1 && data.mask.cols>=2,"Không đọc được mặt nạ chỉ số lớp: "+mask_file);
            double maximum=0; cv::minMaxLoc(data.mask,nullptr,&maximum);
            Require(maximum<=2,"Mặt nạ có lớp ngoài 0/1/2: "+mask_file);
            data.mask_ms=mask_ms; data.image_ms=image_ms;
            const auto reference=get("lane_ref","RIGHT");
            Require(reference=="RIGHT" || reference=="LEFT","CSV lane_ref phải là RIGHT/LEFT");
            data.reference=reference=="RIGHT" ? control::LaneRef::RIGHT : control::LaneRef::LEFT;
            data.go=running;
            const double old_speed=Number(get("cmd_cms"),"cmd_cms"),old_angle=Number(get("cmd_deg"),"cmd_deg");
            const auto plan=planner.Step(data,now,params);
            out << frame << ',' << now << ',' << data.mask_ms << ',' << Quote(mask_file) << ',' << plan.cmd.cms << ',' << plan.cmd.deg << ','
                << old_speed << ',' << old_angle << ',' << control::StateName(plan.state) << ',' << plan.debug.state << ',' << Quote(plan.reason) << ','
                << reference << ',' << plan.debug.e << ',' << plan.debug.p << ',' << plan.debug.d << ',' << plan.debug.valid_rows;
            for(int field=0;field<6;++field) out << ',' << RowField(plan.debug,field);
            out << ',' << hash << '\n';
            Require(bool(out),"Lỗi ghi CSV kết quả");
            Require(cv::imwrite(output+"/overlays/"+frame+"_"+std::to_string(rows)+".png",Overlay(data.mask,plan)),"Lỗi ghi ảnh phủ");
            const double ds=std::abs(plan.cmd.cms-old_speed), da=std::abs(plan.cmd.deg-old_angle);
            sum_speed+=ds; sum_angle+=da; max_speed=std::max(max_speed,ds); max_angle=std::max(max_angle,da); ++samples;
        }
        Require(input.eof() && !input.bad(),"Lỗi đọc control_log.csv");
        Require(samples>0,"Run không có mẫu mặt nạ để replay");
        out.close(); Require(bool(out),"Lỗi đóng CSV kết quả");
        std::cout << "So lệnh mới với lệnh đã ghi (trị tuyệt đối)\nMẫu | TB cm/s | Max cm/s | TB độ | Max độ\n"
                  << samples << " | " << sum_speed/samples << " | " << max_speed << " | " << sum_angle/samples << " | " << max_angle << '\n';
        return 0;
    } catch(const std::exception &error) { std::cerr << "Replay lỗi: " << error.what() << '\n'; return 1; }
}
