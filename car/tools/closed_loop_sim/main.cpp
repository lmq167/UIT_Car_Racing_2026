/* Mô phỏng vòng kín cho bộ bám làn (LaneController): xe lái theo lệnh, mặt nạ được dựng lại từ vị trí xe.
   Công cụ laptop, KHÔNG chạy trên xe, KHÔNG chứng minh an toàn trên xe: chỉ so sánh hai bản bộ điều khiển
   trên cùng kịch bản (quỹ đạo, dao động lái, tỷ lệ chạm trần lái, giữ xe trong hành lang).

   Mô hình đã định danh từ log xe thật (phiên m3 11/11, 2 phiên, tương quan 0,97):
     dyaw = d * tan(steer) / L_eff, L_eff ~ 31 cm; lệnh lái tới bánh trễ ~0,3-0,4 s; tuổi mặt nạ khi ra lệnh p50 ~ 160 ms.
   Hình học camera suy từ lane1_width_frac (bề rộng làn theo hàng): chân trời, chiều cao camera ~14,9 cm;
   góc nhìn ngang (hfov) KHÔNG đo được: mặc định 62 độ, thử thêm giá trị khác.
   Giới hạn: mặt nạ sạch (không nhiễu/vạch giả), không có biển/vật cản, đường phẳng, xe một khối.

   Dịch: g++ -std=c++17 -O2 -Iinclude tools/closed_loop_sim/main.cpp src/control/ControlParams.cpp
         src/control/LaneController.cpp $(pkg-config --cflags opencv4) -lopencv_core -lopencv_imgproc -o sim
   Chạy: sim --control control.yaml [--hfov 62] [--act-delay-ms 350] [--mask-delay-ms 160] [--csv out.csv] */
#include "control/ControlParams.h"
#include "control/LaneController.h"
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace {
constexpr double kPi=3.14159265358979323846;
double Rad(double d) { return d*kPi/180.0; }

struct Pt { double x, y, h; };  // h: hướng (rad) của đường tâm
struct Segment { double len_or_angle_deg; double radius; };  // radius=0: thẳng dài len; radius>0: cung, góc có dấu (+ trái)

struct Track {
    std::vector<Pt> pts;      // đường tâm, bước 2 cm
    double half_width=30;     // nửa bề rộng mặt đường (cm)
    double lane_offset=-15;   // tâm làn lái so với đường tâm (âm = bên phải)
    int end_index=0;          // hết đoạn đánh giá; phía sau là đoạn thẳng 250 cm để xe còn thấy đường
    double Length() const { return (pts.size()-1)*2.0; }
};
Track Build(const std::vector<Segment>& segments,double half_width,double lane_offset) {
    Track t; t.half_width=half_width; t.lane_offset=lane_offset;
    Pt p{0,0,0}; t.pts.push_back(p);
    for (const auto& s : segments) {
        if (s.radius<=0) {
            const int n=int(std::lround(s.len_or_angle_deg/2.0));
            for (int i=0;i<n;++i) { p.x+=2.0*std::cos(p.h); p.y+=2.0*std::sin(p.h); t.pts.push_back(p); }
        } else {
            const double angle=Rad(s.len_or_angle_deg), arc=std::abs(angle)*s.radius;
            const int n=std::max(1,int(std::lround(arc/2.0)));
            const double dh=angle/n, step=arc/n;
            for (int i=0;i<n;++i) {
                const double mid=p.h+dh/2;
                p.x+=step*std::cos(mid); p.y+=step*std::sin(mid); p.h+=dh; t.pts.push_back(p);
            }
        }
    }
    t.end_index=int(t.pts.size())-1;
    for (int i=0;i<125;++i) { p.x+=2.0*std::cos(p.h); p.y+=2.0*std::sin(p.h); t.pts.push_back(p); }
    return t;
}
// Độ lệch ngang có dấu (+ trái đường tâm) và chỉ số điểm gần nhất, tìm quanh `hint`.
double Lateral(const Track& t,double x,double y,int& hint,int window=250) {
    const int lo=std::max(0,hint-window), hi=std::min(int(t.pts.size())-1,hint+window);
    double best=1e18; int bi=lo;
    for (int i=lo;i<=hi;++i) {
        const double dx=x-t.pts[i].x, dy=y-t.pts[i].y, d2=dx*dx+dy*dy;
        if (d2<best) { best=d2; bi=i; }
    }
    hint=bi;
    const Pt& p=t.pts[bi];
    // chiếu lên pháp tuyến trái; ngoài hai đầu đường coi như không có mặt đường
    const double along=(x-p.x)*std::cos(p.h)+(y-p.y)*std::sin(p.h);
    if ((bi==0 && along<0) || (bi==int(t.pts.size())-1 && along>0)) return 1e9;
    return -(x-p.x)*std::sin(p.h)+(y-p.y)*std::cos(p.h);
}
struct Camera { double y_horizon_px, f_px, height_cm, forward_cm; int w, h; };
cv::Mat Render(const Track& t,const Camera& cam,double rx,double ry,double yaw,int& hint) {
    cv::Mat mask=cv::Mat::zeros(cam.h,cam.w,CV_8UC1);
    Lateral(t,rx,ry,hint,400);  // chỉ để cập nhật chỉ số đường tâm gần xe nhất; cửa sổ tìm theo hint
    const double cx=rx+cam.forward_cm*std::cos(yaw), cy=ry+cam.forward_cm*std::sin(yaw);
    for (int y=int(std::ceil(cam.y_horizon_px))+1;y<cam.h;++y) {
        const double d=cam.height_cm*cam.f_px/(y+0.5-cam.y_horizon_px);
        for (int x=0;x<cam.w;++x) {
            const double lat=(x+0.5-cam.w/2.0)*d/cam.f_px;  // + phải
            const double wx=cx+d*std::cos(yaw)+lat*std::sin(yaw), wy=cy+d*std::sin(yaw)-lat*std::cos(yaw);
            int h=hint; const double off=Lateral(t,wx,wy,h,120);
            if (std::abs(off)<=t.half_width) mask.at<uint8_t>(y,x)=1;
        }
    }
    return mask;
}
struct Result {
    std::string name; double rms_off=0, max_off=0, rms_steer=0, sat_frac=0, flips_per_10s=0, out_frac=0, mean_speed=0;
    double lane_out_frac=0; double duration=0; bool finished=false, left_road=false;
};
struct Config { double hfov=62, act_delay_ms=350, mask_delay_ms=160, init_off=0, init_yaw=0, leff=31, cam_forward=12; std::string csv; double dump_t=-1; std::string dump_path; };

Result Run(const std::string& name,const Track& track,const control::ControlParams& params,const Config& cfg) {
    Camera cam; cam.w=160; cam.h=96;
    // chân trời theo bề rộng làn tuyến tính: w(y)=a*(y_frac-y0)
    const double yb=params.corridor_bottom_frac, yt=params.corridor_top_frac;
    const double wb=params.lane1_width_frac[0], wt=params.lane1_width_frac[1];
    const double a=(wb-wt)/(yb-yt), y0=yb-wb/a;
    cam.y_horizon_px=y0*cam.h;
    cam.height_cm=30.0*cam.h/(a*cam.w);
    cam.f_px=0.5*cam.w/std::tan(Rad(cfg.hfov/2));
    cam.forward_cm=cfg.cam_forward;

    control::LaneController lc;
    // xe bắt đầu trên tâm làn lái + độ lệch ban đầu
    const Pt& p0=track.pts[0];
    const double lane=track.lane_offset+cfg.init_off;
    double x=p0.x-lane*std::sin(p0.h), y=p0.y+lane*std::cos(p0.h), yaw=p0.h+Rad(cfg.init_yaw);
    const double dt=0.01, period=params.control_period_ms/1000.0;
    struct Pose { double x,y,yaw; };
    std::deque<Pose> history;  // 10 ms một mẫu
    std::deque<double> pending;  // hàng đợi trễ lệnh lái (độ), mỗi mẫu 10 ms
    const int actDelay=std::max(0,int(std::lround(cfg.act_delay_ms/10.0)));
    const int maskDelay=std::max(0,int(std::lround(cfg.mask_delay_ms/10.0)));
    for (int i=0;i<actDelay;++i) pending.push_back(0);
    double steerApplied=0, steerCmd=0, speed=0;
    int hint=0, hintBody=0, hintMask=0;
    std::vector<double> offs, steers, speeds;
    std::ofstream csv; if (!cfg.csv.empty()) { csv.open(cfg.csv); csv<<"t,x,y,yaw_deg,off_cm,steer_cmd,steer_applied,speed,e,mode\n"; }
    Result r; r.name=name;
    int64_t now=1000; double t=0, nextControl=0; int signFlips=0; double lastSign=0;
    int outFrames=0, satFrames=0, frames=0, laneOutFrames=0;
    const double maxT=120;
    while (t<maxT) {
        history.push_back({x,y,yaw}); if (history.size()>size_t(maskDelay+2)) history.pop_front();
        if (t>=nextControl) {
            nextControl+=period;
            const Pose& seen=history.size()>size_t(maskDelay) ? history[history.size()-1-maskDelay] : history.front();
            const cv::Mat mask=Render(track,cam,seen.x,seen.y,seen.yaw,hintMask);
            if (cfg.dump_t>=0 && t>=cfg.dump_t && t<cfg.dump_t+period) cv::imwrite(cfg.dump_path,mask*120);
            const auto res=lc.Step(mask,now-int64_t(cfg.mask_delay_ms),now,params,control::TrajMode::LANE,true);
            steerCmd=res.cmd.deg; speed=res.cmd.cms;
            if (cfg.dump_t>=0 && t>=cfg.dump_t && t<cfg.dump_t+period) {
                std::fprintf(stderr,"t=%.2f e=%.3f look_x=%.1f steer=%.2f\n",t,res.debug.e,res.debug.look_x,steerCmd);
                for (const auto& r : res.debug.rows) std::fprintf(stderr," y=%d valid=%d L=%.1f R=%.1f tgt=%.1f lanes=%d conf=%.2f fl=%s fr=%s hole=%d\n",r.y,int(r.valid),r.left,r.right,r.target,r.lanes,r.conf,r.flag_left.c_str(),r.flag_right.c_str(),int(r.hole));
            }
            const int hb=hint; (void)hb;
            if (csv) {
                int h2=hintBody; const double off=Lateral(track,x,y,h2)-track.lane_offset;
                csv<<t<<','<<x<<','<<y<<','<<yaw*180/kPi<<','<<off<<','<<steerCmd<<','<<steerApplied<<','<<speed<<','<<res.debug.e<<','<<res.debug.mode<<'\n';
            }
            // chỉ tính khi xe đã chạy (lệnh tốc độ > 0)
            if (speed>0.5) {
                int h2=hintBody; const double off=Lateral(track,x,y,h2)-track.lane_offset; hintBody=h2;
                if (std::abs(off)<1e8) { offs.push_back(off); }
                steers.push_back(steerApplied); speeds.push_back(speed);
                // thân xe (rộng 22 cm, mũi tại điểm nhìn phía trước 27 cm tính từ trục sau): hai góc trước ngoài mặt đường?
                bool out=false;
                for (double sx : {-11.0,11.0}) for (double fx : {0.0,27.0}) {
                    const double wx=x+fx*std::cos(yaw)+sx*std::sin(yaw), wy=y+fx*std::sin(yaw)-sx*std::cos(yaw);
                    int h3=hintBody; const double o=Lateral(track,wx,wy,h3);
                    if (std::abs(o)>track.half_width) out=true;
                }
                if (out) ++outFrames;
                if (std::abs(off)>track.half_width*0.5+(track.lane_offset!=0 ? 0 : 0)) ++laneOutFrames;
                ++frames;
                if (std::abs(steerApplied)>=params.steer_max_deg-0.5) ++satFrames;
                const double sign=steerApplied>2 ? 1 : steerApplied<-2 ? -1 : 0;
                if (sign!=0 && lastSign!=0 && sign!=lastSign) ++signFlips;
                if (sign!=0) lastSign=sign;
            }
            now+=int64_t(params.control_period_ms);
        }
        // trễ lệnh lái: lệnh hiện tại vào hàng đợi, bánh lái nhận lệnh cũ
        pending.push_back(steerCmd); steerApplied=pending.front(); pending.pop_front();
        // động học xe đạp: tâm trục sau
        const double v=speed;
        x+=v*std::cos(yaw)*dt; y+=v*std::sin(yaw)*dt;
        yaw+=v*std::tan(Rad(steerApplied))/cfg.leff*dt;
        t+=dt;
        // kết thúc khi tới cuối đường (còn 40 cm) hoặc rời xa mặt đường
        int h4=hint; const double o=Lateral(track,x,y,h4); hint=h4;
        if (hint>=track.end_index) { r.finished=true; break; }
        if (std::abs(o)>track.half_width+40 && std::abs(o)<1e8) { r.left_road=true; break; }
    }
    r.duration=t;
    auto rms=[](const std::vector<double>& v) { double s=0; for (double a : v) s+=a*a; return v.empty() ? 0.0 : std::sqrt(s/v.size()); };
    r.rms_off=rms(offs); for (double a : offs) r.max_off=std::max(r.max_off,std::abs(a));
    r.rms_steer=rms(steers);
    r.sat_frac=frames ? double(satFrames)/frames : 0; r.out_frac=frames ? double(outFrames)/frames : 0; r.lane_out_frac=frames ? double(laneOutFrames)/frames : 0;
    r.flips_per_10s=frames ? signFlips/(frames*period/10.0) : 0;
    double sp=0; for (double s : speeds) sp+=s; r.mean_speed=speeds.empty() ? 0 : sp/speeds.size();
    return r;
}
}  // namespace

int main(int argc,char** argv) {
    std::map<std::string,std::string> o;
    for (int i=1;i+1<argc;i+=2) o[argv[i]]=argv[i+1];
    if (!o.count("--control")) { std::cerr<<"Dùng: sim --control control.yaml [--hfov 62] [--act-delay-ms 350] [--mask-delay-ms 160] [--csv out.csv] [--only tên]\n"; return 2; }
    Config cfg;
    if (o.count("--hfov")) cfg.hfov=std::stod(o["--hfov"]);
    if (o.count("--act-delay-ms")) cfg.act_delay_ms=std::stod(o["--act-delay-ms"]);
    if (o.count("--mask-delay-ms")) cfg.mask_delay_ms=std::stod(o["--mask-delay-ms"]);
    if (o.count("--leff")) cfg.leff=std::stod(o["--leff"]);
    if (o.count("--csv")) cfg.csv=o["--csv"];
    if (o.count("--dump-mask-t")) { cfg.dump_t=std::stod(o["--dump-mask-t"]); cfg.dump_path=o["--dump-path"]; }
    std::streambuf* keep=std::cout.rdbuf(); std::ofstream sink("NUL"); std::cout.rdbuf(sink.rdbuf());
    const auto params=control::ControlParams::Load(o["--control"],std::cout);
    std::cout.rdbuf(keep);
    struct Scn { std::string name; Track track; double off, yaw; };
    std::vector<Scn> scenarios;
    const double hw2=30, lo2=-15, hw1=15, lo1=0;
    auto add=[&](const std::string& n,const std::vector<Segment>& s,double hw,double lo,double off=0,double yaw=0) { scenarios.push_back({n,Build(s,hw,lo),off,yaw}); };
    add("thang_2lan_lech+8",{{400,0}},hw2,lo2,8,3);
    add("thang_1lan_lech-6",{{400,0}},hw1,lo1,-6,-3);
    for (double R : {50.0,80.0,120.0}) {
        const std::string r=std::to_string(int(R));
        add("cong_trai_R"+r+"_2lan",{{150,0},{90,R},{150,0}},hw2,lo2);
        add("cong_phai_R"+r+"_2lan",{{150,0},{-90,R},{150,0}},hw2,lo2);
        add("cong_trai_R"+r+"_1lan",{{150,0},{90,R},{150,0}},hw1,lo1);
        add("cong_phai_R"+r+"_1lan",{{150,0},{-90,R},{150,0}},hw1,lo1);
    }
    add("chuoi_S_R70_2lan",{{100,0},{90,70},{-90,70},{90,70},{-90,70},{100,0}},hw2,lo2);
    add("chuoi_S_R70_1lan",{{100,0},{90,70},{-90,70},{90,70},{-90,70},{100,0}},hw1,lo1);
    std::printf("%-26s %6s %7s %7s %7s %6s %7s %7s %7s %7s %s\n","kich_ban","s","rmsOff","maxOff","rmsLai","chamTran","lat/10s","ngoai%","ngLan%","v_tb","ket_thuc");
    for (const auto& s : scenarios) {
        if (o.count("--only") && s.name.find(o["--only"])==std::string::npos) continue;
        Config c=cfg; c.init_off=s.off; c.init_yaw=s.yaw;
        const auto r=Run(s.name,s.track,params,c);
        std::printf("%-26s %6.1f %7.2f %7.2f %7.2f %6.1f%% %7.1f %6.1f%% %6.1f%% %7.1f %s\n",r.name.c_str(),r.duration,r.rms_off,r.max_off,r.rms_steer,
                    100*r.sat_frac,r.flips_per_10s,100*r.out_frac,100*r.lane_out_frac,r.mean_speed,r.finished ? "xong" : r.left_road ? "ROI_DUONG" : "het_gio");
    }
    return 0;
}
