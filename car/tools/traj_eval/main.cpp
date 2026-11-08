#include "control/Planner.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>

// Dùng cv::glob và tiện ích thư mục OpenCV để tương thích GCC 7, không cần filesystem.
#include <opencv2/core/utils/filesystem.hpp>
namespace {
std::string number(int n) { std::ostringstream s; s<<std::setw(4)<<std::setfill('0')<<n; return s.str(); }
int road(const cv::Mat &m,double x,double y) {
    if(!std::isfinite(x)||!std::isfinite(y)||x<0||y<0||x>=m.cols||y>=m.rows) return 0;
    return m.at<unsigned char>(int(std::floor(y+0.5))<m.rows ? int(std::floor(y+0.5)):m.rows-1,
                               int(std::floor(x+0.5))<m.cols ? int(std::floor(x+0.5)):m.cols-1)==1;
}
cv::Scalar color(const std::string &f) {
    return f=="THAT" ? cv::Scalar(0,255,0) : f=="MEP_ANH" ? cv::Scalar(0,255,255):cv::Scalar(0,0,255);
}
cv::Point point(double x,double y) { return cv::Point(cvRound(std::max(-10000.,std::min(10000.,x))),cvRound(y)); }
}
int main(int argc,char **argv) {
    try {
        std::map<std::string,std::string> a;
        const std::vector<std::string> keys={"--masks","--images","--pass","--from","--to","--control","--out","--step-ms","--mode"};
        for(int i=1;i<argc;i+=2) {
            const std::string k=argv[i];
            if(i+1>=argc||std::find(keys.begin(),keys.end(),k)==keys.end()||a.count(k)) throw std::runtime_error("Doi so sai: "+k);
            a[k]=argv[i+1];
        }
        for(const auto &k:keys) if(k!="--step-ms"&&k!="--mode"&&!a.count(k)) throw std::runtime_error("Thieu "+k);
        auto integer=[](const std::string &s) { size_t end=0; int n=std::stoi(s,&end); if(end!=s.size()) throw std::runtime_error("So nguyen sai"); return n; };
        const int pass=integer(a["--pass"]), first=integer(a["--from"]), last=integer(a["--to"]), step=a.count("--step-ms")?integer(a["--step-ms"]):100;
        if(pass<0||first<0||last<first||step<=0) throw std::runtime_error("Khoang/nhịp sai");
        const bool direct=a.count("--mode");
        control::TrajMode mode=control::TrajMode::LANE;
        if(direct) {
            if(a["--mode"]=="roundabout") mode=control::TrajMode::ROUNDABOUT;
            else if(a["--mode"]=="avoid") mode=control::TrajMode::AVOID_LEFT;
            else if(a["--mode"]!="lane") throw std::runtime_error("Mode sai");
        }
        auto params=control::ControlParams::Load(a["--control"],std::cerr); params.Validate();
        const std::string out=a["--out"];
        if(cv::utils::fs::exists(out)) throw std::runtime_error("Thu muc out phai moi");
        if(!cv::utils::fs::createDirectories(out+"/overlay")) throw std::runtime_error("Khong tao duoc out");
        std::ofstream csv(out+"/eval.csv"); if(!csv) throw std::runtime_error("Khong ghi duoc CSV");
        csv<<"idx,t_ms,state,reason,mode,valid_rows,look_x,look_y,look_on_road,targets_on_road,conf_mean,lanes_near,e,deg,cms\n"<<std::setprecision(12);
        control::Planner planner; control::LaneController lane;
        std::vector<cv::Mat> thumbs;
        int count=0,missing=0;
        // Điểm đầu 1000 ms tránh mốc âm/sentinel; idx khuyết vẫn làm tăng đồng hồ.
        for(int idx=first;idx<=last;++idx) {
            std::string stem="car6_p"+number(pass)+"_"+number(idx);
            std::string mask_path=a["--masks"]+"/"+stem+".png";
            if(!cv::utils::fs::exists(mask_path)) { ++missing; continue; }
            cv::Mat raw=cv::imread(mask_path,cv::IMREAD_GRAYSCALE);
            if(raw.empty()||raw.size()!=cv::Size(320,192)) throw std::runtime_error("Mask sai: "+mask_path);
            cv::Mat bad=(raw!=0)&(raw!=255); if(cv::countNonZero(bad)) throw std::runtime_error("Mask khong phai 0/255");
            cv::Mat mask; raw.convertTo(mask,CV_8U,1./255);
            cv::Mat img=cv::imread(a["--images"]+"/"+stem+".jpg");
            if(img.empty()) throw std::runtime_error("Thieu anh: "+stem);
            cv::resize(img,img,cv::Size(320,192));
            const int64_t ms=1000+int64_t(idx-first)*step;
            control::ControlDebug d; control::Command cmd; std::string state,reason;
            if(direct) { auto r=lane.Step(mask,ms,ms,params,mode); d=r.debug; cmd=r.cmd; state=d.state; reason=d.reason; }
            else {
                control::PlannerInput in; in.mask=mask; in.mask_ms=ms; in.image_ms=ms;
                in.yolo_enabled=false; in.sensor_ok=false; in.go=count==0;
                auto p=planner.Step(in,ms,params); d=p.debug; cmd=p.cmd; state=control::StateName(p.state); reason=p.reason;
            }
            int valid=0,on=0,lanes=0; double conf=0;
            for(const auto &r:d.rows) if(r.valid) { ++valid; on+=road(mask,r.target,r.y); conf+=r.conf; if(!lanes) lanes=r.lanes; }
            const int look=d.look_x<0 ? -1:road(mask,d.look_x,d.look_y);
            csv<<idx<<','<<ms<<','<<state<<','<<reason<<','<<d.mode<<','<<d.valid_rows<<','<<d.look_x<<','<<d.look_y<<','<<look<<','<<(valid?double(on)/valid:-1)<<','<<(valid?conf/valid:0)<<','<<lanes<<','<<d.e<<','<<cmd.deg<<','<<cmd.cms<<'\n';
            cv::Mat tint=img.clone(); tint.setTo(cv::Scalar(160,100,0),mask); cv::addWeighted(img,.65,tint,.35,0,img);
            for(const auto &r:d.rows) if(r.valid) {
                cv::circle(img,point(r.left,r.y),2,color(r.flag_left),-1);
                cv::circle(img,point(r.right,r.y),2,color(r.flag_right),-1);
                cv::drawMarker(img,point(r.target,r.y),cv::Scalar(255,0,255),cv::MARKER_CROSS,5,1);
            }
            if(d.fit.size()==3 && !d.rows.empty()) {
                int lo=191,hi=0; for(const auto &r:d.rows) if(r.valid) { lo=std::min(lo,r.y); hi=std::max(hi,r.y); }
                for(int y=lo+1;y<=hi;++y) {
                    auto x=[&](int yy) {return d.fit[0]+d.fit[1]*yy+d.fit[2]*yy*yy;};
                    cv::line(img,point(x(y-1),y-1),point(x(y),y),cv::Scalar(255,255,0),1);
                }
            }
            if(look>=0) cv::circle(img,point(d.look_x,d.look_y),5,cv::Scalar(255,255,255),1);
            cv::putText(img,number(idx)+" "+state,cv::Point(3,13),cv::FONT_HERSHEY_SIMPLEX,.32,cv::Scalar(255,255,255),1);
            cv::putText(img,reason+" "+d.mode,cv::Point(3,26),cv::FONT_HERSHEY_SIMPLEX,.30,cv::Scalar(255,255,255),1);
            if(look<0) cv::putText(img,"KHONG TIN CAY",cv::Point(3,42),cv::FONT_HERSHEY_SIMPLEX,.45,cv::Scalar(0,0,255),1);
            if(!cv::imwrite(out+"/overlay/"+number(idx)+".png",img)) throw std::runtime_error("Loi ghi overlay");
            // Lấy đều tối đa 24 khung theo vị trí idx, luôn giữ khung đầu có dữ liệu.
            if(thumbs.size()<24 && (thumbs.empty()||int64_t(idx-first)*24/int64_t(last-first+1)>int64_t(idx-first-1)*24/int64_t(last-first+1))) thumbs.push_back(img.clone());
            ++count;
        }
        if(!count) throw std::runtime_error("Khong co khung nao");
        cv::Mat sheet(192*((int(thumbs.size())+3)/4),320*4,CV_8UC3,cv::Scalar(0,0,0));
        for(size_t i=0;i<thumbs.size();++i) thumbs[i].copyTo(sheet(cv::Rect(int(i%4)*320,int(i/4)*192,320,192)));
        if(!cv::imwrite(out+"/sheet.jpg",sheet)||!csv) throw std::runtime_error("Loi ghi ket qua");
        std::cout<<"p"<<number(pass)<<": "<<count<<" khung, bo qua "<<missing<<" idx\n";
    } catch(const std::exception &e) { std::cerr<<e.what()<<'\n'; return 1; }
}
