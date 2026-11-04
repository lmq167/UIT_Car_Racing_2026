#include "control/LaneController.h"
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <random>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace {
using control::ControlParams;
using control::LaneController;
using control::LaneRef;

void Check(bool ok, const std::string &message) {
    if(!ok) throw std::runtime_error(message);
}
void Near(double actual, double expected, const std::string &message) {
    Check(std::isfinite(actual) && std::abs(actual-expected)<=1e-9,
          message+": nhận "+std::to_string(actual)+", cần "+std::to_string(expected));
}
template<class F> void Reject(F action) {
    try { action(); }
    catch(const std::exception &error) {
        Check(std::string(error.what()).find("kd_filter_ms")!=std::string::npos,
              "Lỗi phải nêu kd_filter_ms: "+std::string(error.what()));
        return;
    }
    throw std::runtime_error("Phải từ chối kd_filter_ms sai kiểu/miền");
}
ControlParams Load(const std::string &path, const std::string &body, bool header=true) {
    {
        std::ofstream out(path);
        if(header) out << "%YAML:1.0\n---\n";
        out << body;
        out.close();
        Check(bool(out),"Không ghi được YAML tạm: "+path);
    }
    std::ostringstream report;
    return ControlParams::Load(path,report);
}
ControlParams Params() {
    ControlParams p;
    p.scan_rows={0.5}; p.row_weights={1}; p.lane_width_frac={0.4};
    p.min_valid_rows=1; p.regain_frames=1; p.right_lost_ms=0; p.pair_mode=0;
    p.kd=2; p.kd_filter_ms=100;
    p.Validate();
    return p;
}
cv::Mat Mask(int center) {
    cv::Mat mask(60,200,CV_8UC1,cv::Scalar(0));
    for(int y=0;y<mask.rows;++y) {
        mask.at<unsigned char>(y,center-40)=2;
        mask.at<unsigned char>(y,center+40)=2;
    }
    return mask;
}
double D(LaneController &c, int center, int64_t now, const ControlParams &p,
         LaneRef ref=LaneRef::RIGHT) {
    const auto r=c.Step(Mask(center),now,now,p,ref);
    Check(r.debug.state=="BAM_LAN","Mặt nạ tổng hợp phải bám làn");
    Near(r.debug.e,(center-100)/100.0,"Sai số từ tâm làn tổng hợp");
    return r.debug.d;
}

// Cùng cách đọc CSV và dựng mặt nạ của test_t55_lane_rules, không sinh lại golden.
std::vector<std::string> Csv(const std::string &line) {
    std::vector<std::string> out;
    std::string field;
    bool quoted=false;
    for(size_t i=0;i<line.size();++i) {
        const char c=line[i];
        if(c=='"') {
            if(quoted && i+1<line.size() && line[i+1]=='"') { field+='"'; ++i; }
            else quoted=!quoted;
        } else if(c==',' && !quoted) { out.push_back(field); field.clear(); }
        else if(c!='\r') field+=c;
    }
    Check(!quoted,"CSV chưa đóng dấu nháy");
    out.push_back(field);
    return out;
}
cv::Mat FixtureMask(const std::string &json, const ControlParams &p) {
    static const std::regex row(R"re(\{"y_frac":([-+0-9.eE]+),"left_frac":(null|[-+0-9.eE]+),"right_frac":(null|[-+0-9.eE]+),"center_frac":(null|[-+0-9.eE]+),"valid":(true|false)\})re");
    cv::Mat mask(200,320,CV_8UC1,cv::Scalar(0));
    size_t i=0;
    for(std::sregex_iterator it(json.begin(),json.end(),row), end; it!=end; ++it,++i) {
        Check(i<p.scan_rows.size(),"Fixture thừa hàng quét");
        Near(std::stod((*it)[1].str()),p.scan_rows[i],"Vị trí hàng fixture");
        if((*it)[5].str()=="false") continue;
        Check((*it)[3].str()!="null","Hàng hợp lệ thiếu mép phải");
        const int y=static_cast<int>(p.scan_rows[i]*mask.rows);
        const int right=static_cast<int>(std::lround(std::stod((*it)[3].str())*mask.cols));
        Check(right>0 && right<=mask.cols,"Mép phải fixture ngoài ảnh");
        if((*it)[2].str()!="null") {
            const int left=static_cast<int>(std::lround(std::stod((*it)[2].str())*mask.cols));
            Check(left>=0 && left<right,"Mép trái fixture sai");
            for(int x=left;x<right;++x) mask.at<unsigned char>(y,x)=1;
            if(left>0) mask.at<unsigned char>(y,left)=2;
        } else if(right==mask.cols) mask.at<unsigned char>(y,mask.cols-1)=1;
        if(right<mask.cols) mask.at<unsigned char>(y,right)=2;
    }
    Check(i==p.scan_rows.size(),"Fixture thiếu/sai hàng quét");
    return mask;
}
void Golden(const std::string &path) {
    const auto p=Load(path,"kd_filter_ms: 0\nright_lost_ms: 0\npair_mode: 0\n");
    Check(p.kd_filter_ms==0,"YAML tắt lọc phải nạp đúng 0");
    std::ifstream frames(std::string(T56_FIXTURE_DIR)+"/lane_run_7675_31p5_39p9.csv");
    std::ifstream golden(std::string(T56_FIXTURE_DIR)+"/lane_golden_fab24d1.csv");
    std::string line, expected;
    Check(bool(std::getline(frames,line)) && Csv(line)==std::vector<std::string>{
        "t_s","t_ms","scan_rows","e","p","sent_steer_deg","go"},"Sai đầu đề fixture");
    Check(bool(std::getline(golden,expected)) && Csv(expected)==std::vector<std::string>{
        "t_s","cmd.cms","cmd.deg","e","p","state","reason"},"Sai đầu đề golden");
    LaneController c;
    int count=0;
    int64_t previous=-1;
    while(std::getline(frames,line)) {
        const auto f=Csv(line);
        Check(f.size()==7 && bool(std::getline(golden,expected)),"Thiếu cột/khung golden");
        const auto g=Csv(expected);
        Check(g.size()==7,"Golden sai số cột");
        const int64_t now=std::stoll(f[1]);
        Check(now>previous,"Thời gian fixture không tăng"); previous=now;
        const auto r=c.Step(FixtureMask(f[2],p),now,now,p);
        const std::string at=" tại khung "+std::to_string(++count);
        Near(std::stod(f[0]),std::stod(g[0]),"Thời gian golden"+at);
        Near(r.cmd.cms,std::stod(g[1]),"Tốc độ golden"+at);
        Near(r.cmd.deg,std::stod(g[2]),"Góc golden"+at);
        Near(r.debug.e,std::stod(g[3]),"e golden"+at);
        Near(r.debug.p,std::stod(g[4]),"p golden"+at);
        Check(r.debug.state==g[5] && r.debug.reason==g[6],"Trạng thái/lý do golden"+at);
    }
    Check(frames.eof() && count==164,"Phải kiểm đủ 164 khung");
    Check(!std::getline(golden,expected) && golden.eof(),"Golden thừa khung/lỗi đọc");
}
void YamlDefault() {
    std::ostringstream report;
    const auto p=ControlParams::Load(T56_CONTROL_YAML,report);
    Check(p.kd_filter_ms==0,"control.yaml thật phải tắt lọc");
    std::ifstream yaml(T56_CONTROL_YAML);
    Check(bool(yaml),"Không mở được control.yaml thật");
    std::string line;
    int count=0;
    while(std::getline(yaml,line))
        if(line.compare(0,13,"kd_filter_ms:")==0) ++count;
    Check(yaml.eof(),"Lỗi đọc control.yaml thật");
    Check(count==1,"control.yaml thật phải có đúng một dòng bắt đầu kd_filter_ms:, nhận "+std::to_string(count));
}
void StepAndNoise() {
    const auto p=Params();
    LaneController c;
    // e: 0 -> 0,2 -> 0,2; raw: 4 -> 0; a=0,05/(0,1+0,05)=1/3.
    Near(D(c,100,0,p),0,"Chưa sẵn sàng phải d=0");
    Near(D(c,120,50,p),8,"Mẫu đầu: kd*raw = 2*4");
    Near(D(c,120,100,p),16.0/3,"Mẫu sau: 2*(4+(0-4)/3)");
    // dt=100 ms: a=1/2, raw=-2, filt=(8/3)/2-1=1/3.
    Near(D(c,100,200,p),2.0/3,"Hệ số lọc phải theo dt thực");

    for(int tau:{500,1}) {
        auto boundary=p; boundary.kd_filter_ms=tau;
        LaneController endpoint;
        const std::string at=" với kd_filter_ms="+std::to_string(tau);
        Near(D(endpoint,100,0,boundary),0,"Lấy mốc"+at);
        Near(D(endpoint,120,50,boundary),8,"Mẫu đầu: 2*4"+at);
        // dt=50 ms: a=1/11 khi tau=500; a=50/51 khi tau=1.
        Near(D(endpoint,120,100,boundary),tau==500 ? 80.0/11 : 8.0/51,
             "Mẫu sau: 2*(4+a*(0-4))"+at);
    }

    auto off=p; off.kd_filter_ms=0;
    LaneController filtered, unfiltered;
    bool reduced=false;
    for(int i=0;i<50;++i) {
        const int center=i%2 ? 120 : 100;
        const double a=D(filtered,center,i*50,p), b=D(unfiltered,center,i*50,off);
        Check(std::abs(a)<=std::abs(b)+1e-9,"Lọc làm tăng |d| trên nhiễu xen kẽ");
        if(std::abs(a)+1e-9<std::abs(b)) reduced=true;
    }
    Check(reduced,"Chuỗi nhiễu phải có mẫu giảm |d|");
}
void ClearState(const std::string &scenario) {
    auto p=Params();
    if(scenario=="recovery") p.regain_frames=3;
    LaneController c;
    Near(D(c,100,0,p),0,"Khởi động");
    Near(D(c,120,50,p),8,"Nạp trạng thái lọc khác 0");
    Near(D(c,120,100,p),16.0/3,"Trạng thái lọc trước ngắt");
    int64_t now=200;
    LaneRef ref=LaneRef::RIGHT;
    if(scenario=="reset") c.Reset();
    else if(scenario=="ref") ref=LaneRef::LEFT;
    else if(scenario=="stale") {
        const int64_t stale=100+p.mask_max_age_ms+1;
        const auto r=c.Step(Mask(120),100,stale,p);
        Check(r.debug.state=="MAT_NAN_CU","Phải đi qua nhánh mặt nạ cũ");
        Near(r.debug.d,0,"Mặt nạ cũ xóa đạo hàm");
        now=stale+50;
    } else {
        const cv::Mat empty(60,200,CV_8UC1,cv::Scalar(0));
        const auto r=c.Step(empty,150,150,p);
        Check(r.debug.state=="MAT_LAN" || r.debug.state=="GIU_GOC","Phải đi qua mất làn");
        Near(r.debug.d,0,"Mất làn xóa đạo hàm");
        for(int i=1;i<p.regain_frames;++i,now+=50) {
            const auto waiting=c.Step(Mask(100),now,now,p);
            Check(waiting.debug.state!="BAM_LAN","Chưa đủ khung phục hồi");
            Near(waiting.debug.d,0,"Chưa phục hồi phải d=0");
        }
    }
    Near(D(c,100,now,p,ref),0,"Sau ngắt: lấy mốc e, chưa có đạo hàm");
    Near(D(c,120,now+50,p,ref),8,"Sau ngắt: mẫu đầu phải khởi tạo lại filt=raw");
    Near(D(c,120,now+100,p,ref),16.0/3,"Sau ngắt: mẫu tiếp mới lọc với a=1/3");
}
void OffExact() {
    auto p=Params(); p.kd_filter_ms=0;
    LaneController c;
    std::mt19937 rng(56);
    int64_t now=0, previousTime=0;
    double previousError=0, mutant=0;
    bool cancellationSeen=false;
    for(int i=0;i<50;++i) {
        const int center=65+static_cast<int>(rng()%71);
        if(i) now+=20+static_cast<int>(rng()%81);
        const auto r=c.Step(Mask(center),now,now,p);
        Check(r.debug.state=="BAM_LAN","Chuỗi ngẫu nhiên phải bám làn");
        Near(r.debug.e,(center-100)/100.0,"e của chuỗi ngẫu nhiên");
        double expected=0;
        if(i) {
            const double dt=(now-previousTime)/1000.0;
            const double raw=(r.debug.e-previousError)/dt;
            expected=p.kd*raw;
            if(i==1) mutant=raw;
            else mutant+=1.0*(raw-mutant);
            if(p.kd*mutant!=expected) cancellationSeen=true;
        }
        Check(r.debug.d==expected,"Tắt lọc phải == biểu thức cũ ở khung "+std::to_string(i));
        previousError=r.debug.e; previousTime=now;
    }
    Check(cancellationSeen,"Hạt cố định phải phân biệt được lọc a=1 do sai số triệt tiêu");
}
void Parameters(const std::string &path) {
    Check(ControlParams{}.kd_filter_ms==0,"Mặc định phải tắt lọc");
    for(int value:{0,1,500}) {
        ControlParams p; p.kd_filter_ms=value; p.Validate();
        for(bool header:{false,true})
            Check(Load(path,"kd_filter_ms: "+std::to_string(value)+"\n",header).kd_filter_ms==value,
                  "Load phải nhận biên hợp lệ");
    }
    for(int value:{-1,501}) {
        ControlParams p; p.kd_filter_ms=value;
        Reject([&] { p.Validate(); });
    }
    for(bool header:{false,true}) {
        Check(Load(path,"kp: 30\n",header).kd_filter_ms==0,"Thiếu khóa phải dùng 0");
        auto held=Load(path,"kp: 42\nkd_filter_ms: 123\n",header);
        std::ostringstream before; held.PrintTable(before);
        for(const std::string value:{"-1","501","12.5"}) {
            Reject([&] { held=Load(path,"kp: 31\nkd_filter_ms: "+value+"\n",header); });
            std::ostringstream after; held.PrintTable(after);
            Check(after.str()==before.str() && held.kd_filter_ms==123,
                  "Load bị từ chối phải giữ nguyên tham số đã nạp: "+value);
        }
        Check(Load(path,"kd_filter_ms: 12.0\n",header).kd_filter_ms==12,
              "Load phải nhận số thực nguyên 12.0");
    }
}
}
int main(int argc, char **argv) {
    try {
        Check(argc>=2,"Cần tên ca kiểm thử");
        const std::string scenario=argv[1];
        if(scenario=="golden" || scenario=="params") {
            Check(argc==3,"Cần đường dẫn YAML tạm riêng của ca");
            struct Cleanup { std::string path; ~Cleanup() { std::remove(path.c_str()); } } cleanup{argv[2]};
            if(scenario=="golden") Golden(argv[2]); else Parameters(argv[2]);
        } else {
            Check(argc==2,"Thừa đối số");
            if(scenario=="yaml_default") YamlDefault();
            else if(scenario=="step_noise") StepAndNoise();
            else if(scenario=="off_exact") OffExact();
            else if(scenario=="lost" || scenario=="stale" || scenario=="reset" ||
                    scenario=="ref" || scenario=="recovery") ClearState(scenario);
            else throw std::runtime_error("Ca không biết: "+scenario);
        }
        std::cout << "ĐẠT: " << scenario << '\n';
        return 0;
    } catch(const std::exception &error) {
        std::cerr << "KHÔNG ĐẠT: " << error.what() << '\n';
        return 1;
    }
}
