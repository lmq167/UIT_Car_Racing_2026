#include "control/LaneController.h"
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace {
using control::ControlParams;
using control::ControlResult;
using control::LaneController;
void Check(bool ok, const std::string &why) { if(!ok) throw std::runtime_error(why); }
void Near(double actual, double expected, const std::string &why, double tolerance=1e-9) {
    Check(std::isfinite(actual) && std::abs(actual-expected)<=tolerance,
          why+": nhận "+std::to_string(actual)+", cần "+std::to_string(expected));
}
double Number(const std::string &s) {
    size_t used=0;
    const double n=std::stod(s,&used);
    Check(used==s.size() && std::isfinite(n),"Số không hợp lệ: "+s);
    return n;
}
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
ControlParams UnitParams() {
    ControlParams p;
    p.pair_mode=1; p.right_lost_ms=0; p.kd=0; p.steer_rate_dps=1000;
    p.lane_width_frac={0.4,0.4,0.4};
    p.lane_width_min_ratio=0.5; p.lane_width_max_ratio=1.5;
    p.Validate();
    return p;
}
// T55c-spec B3: mỗi hàng có cùng đoạn mặt đường [begin,end), vạch thật ở x cho trước.
cv::Mat Mask(const ControlParams &p, const std::vector<std::vector<double>> &lines,
             double begin=0, double end=1) {
    Check(lines.size()==p.scan_rows.size(),"Sai số hàng mặt nạ tổng hợp");
    cv::Mat mask(100,1000,CV_8UC1,cv::Scalar(0));
    for(size_t i=0;i<lines.size();++i) {
        const int y=static_cast<int>(p.scan_rows[i]*mask.rows);
        for(int x=static_cast<int>(std::lround(begin*mask.cols));x<std::lround(end*mask.cols);++x)
            mask.at<unsigned char>(y,x)=1;
        for(double f:lines[i]) {
            Check(f>=begin && f<end,"Vạch nằm ngoài đoạn mặt đường");
            mask.at<unsigned char>(y,static_cast<int>(std::lround(f*mask.cols)))=2;
        }
    }
    return mask;
}
cv::Mat Uniform(const ControlParams &p, const std::vector<double> &lines) {
    return Mask(p,std::vector<std::vector<double>>(p.scan_rows.size(),lines));
}
ControlResult Step(LaneController &c, const cv::Mat &mask, const ControlParams &p, int64_t ms=1000) {
    return c.Step(mask,ms,ms,p);
}
void Pair(const ControlResult &r, size_t i, double left, double right, double center,
          int cols=1000, double tolerance=1e-9) {
    Check(r.debug.rows.size()==3,"Thiếu ba hàng debug");
    const auto &row=r.debug.rows.at(i);
    Check(row.valid,"Hàng "+std::to_string(i)+" phải hợp lệ");
    Near(row.left<0 ? -1 : row.left/cols,left,"Mép trái hàng "+std::to_string(i),tolerance);
    Near(row.right<0 ? -1 : row.right/cols,right,"Mép phải hàng "+std::to_string(i),tolerance);
    Near(row.center/cols,center,"Tâm hàng "+std::to_string(i),tolerance);
}
void AllPairs(const ControlResult &r, double left, double right, double center) {
    for(size_t i=0;i<3;++i) Pair(r,i,left,right,center);
}
void Invalid(const ControlResult &r, size_t i) {
    Check(r.debug.rows.size()==3 && !r.debug.rows.at(i).valid,
          "Hàng "+std::to_string(i)+" phải không hợp lệ");
}
cv::Mat ReadMask(const std::string &path) {
    const auto mask=cv::imread(path,cv::IMREAD_UNCHANGED);
    Check(!mask.empty() && mask.type()==CV_8UC1,"Không nạp được mask 8 bit một kênh: "+path);
    double lo=0,hi=0; cv::minMaxLoc(mask,&lo,&hi);
    Check(lo>=0 && hi<=2,"Mask phải giữ nhãn 0/1/2: "+path);
    return mask;
}
void PrintRows(const ControlResult &r, const cv::Mat &mask, const ControlParams &p) {
    Check(r.debug.rows.size()==p.scan_rows.size(),"Thiếu hàng để in");
    for(size_t i=0;i<r.debug.rows.size();++i) {
        const auto &row=r.debug.rows[i];
        auto frac=[&](double x) { return x<0 ? -1.0 : x/mask.cols; };
        std::cout << p.scan_rows[i] << ',' << frac(row.left) << ',' << frac(row.right)
                  << ',' << frac(row.center) << ',' << row.valid << '\n';
    }
    std::cout << "e=" << r.debug.e << ", cmd.deg=" << r.cmd.deg << '\n';
}

// T55c-test §1: dựng mask đúng như T55a, so toàn bộ đầu ra golden fab24d1 tới 1e-9.
void Off() {
    ControlParams p; p.right_lost_ms=0;
    Check(p.pair_mode==0,"pair_mode mặc định phải là 0");
    std::ifstream fixture(std::string(T55_FIXTURE_DIR)+"/lane_run_7675_31p5_39p9.csv");
    std::ifstream golden(std::string(T55_FIXTURE_DIR)+"/lane_golden_fab24d1.csv");
    std::string line,gline;
    Check(bool(std::getline(fixture,line)) && Csv(line)==std::vector<std::string>{
        "t_s","t_ms","scan_rows","e","p","sent_steer_deg","go"},"Sai đầu đề fixture");
    Check(bool(std::getline(golden,gline)) && Csv(gline)==std::vector<std::string>{
        "t_s","cmd.cms","cmd.deg","e","p","state","reason"},"Sai đầu đề golden");
    const std::regex object(R"(\{[^{}]*\})");
    const std::regex field(R"re("([a-z_]+)"\s*:\s*(null|true|false|[-+0-9.eE]+))re");
    LaneController c;
    size_t frames=0;
    int64_t previous=-1;
    while(std::getline(fixture,line)) {
        const auto f=Csv(line);
        Check(f.size()==7 && bool(std::getline(golden,gline)),"Thiếu dòng/cột golden hoặc fixture");
        const auto g=Csv(gline);
        Check(g.size()==7,"Golden phải có bảy cột");
        cv::Mat mask(200,320,CV_8UC1,cv::Scalar(0));
        size_t i=0;
        for(std::sregex_iterator it(f[2].begin(),f[2].end(),object),end;it!=end;++it,++i) {
            const std::string body=it->str();
            std::vector<std::pair<std::string,std::string>> values;
            for(std::sregex_iterator v(body.begin(),body.end(),field);v!=end;++v)
                values.emplace_back((*v)[1].str(),(*v)[2].str());
            const std::vector<std::string> keys={"y_frac","left_frac","right_frac","center_frac","valid"};
            Check(i<p.scan_rows.size() && values.size()==keys.size(),"Sai lược đồ scan_rows");
            for(size_t k=0;k<keys.size();++k) Check(values[k].first==keys[k],"Sai khóa scan_rows");
            Near(Number(values[0].second),p.scan_rows[i],"Vị trí hàng fixture",1e-6);
            Check(values[4].second=="true" || values[4].second=="false","valid phải là bool");
            if(values[4].second=="false") continue;
            const int y=static_cast<int>(p.scan_rows[i]*mask.rows);
            const int right=static_cast<int>(std::lround(Number(values[2].second)*mask.cols));
            Check(right>0 && right<=mask.cols,"Mép phải fixture ngoài ảnh");
            if(values[1].second!="null") {
                const int left=static_cast<int>(std::lround(Number(values[1].second)*mask.cols));
                Check(left>=0 && left<right,"Mép trái fixture ngoài ảnh");
                for(int x=left;x<right;++x) mask.at<unsigned char>(y,x)=1;
                if(left>0) mask.at<unsigned char>(y,left)=2;
            } else if(right==mask.cols) mask.at<unsigned char>(y,mask.cols-1)=1;
            if(right<mask.cols) mask.at<unsigned char>(y,right)=2;
        }
        Check(i==3,"Fixture phải có ba hàng");
        const double ms=Number(f[1]);
        Check(ms>=0 && ms<1e12 && std::floor(ms)==ms && ms>previous,"Thời gian fixture không hợp lệ");
        previous=static_cast<int64_t>(ms);
        const auto r=Step(c,mask,p,previous);
        const std::string at=" tại t_s="+f[0];
        Near(Number(f[0]),Number(g[0]),"Thời điểm golden"+at);
        Near(r.cmd.cms,Number(g[1]),"Tốc độ golden"+at);
        Near(r.cmd.deg,Number(g[2]),"Góc golden"+at);
        Near(r.debug.e,Number(g[3]),"e golden"+at);
        Near(r.debug.p,Number(g[4]),"p golden"+at);
        Check(r.debug.state==g[5] && r.debug.reason==g[6],"Trạng thái/lý do golden"+at);
        ++frames;
    }
    Check(fixture.eof() && frames==164 && !std::getline(golden,gline) && golden.eof(),"Golden/fixture thiếu hoặc thừa khung");
    p.lane_width_frac={0.5,0.5,0.5}; p.center_offset_frac=0; p.Validate();
    const auto straight=Uniform(p,{0.25,0.75});
    LaneController a,b;
    const auto old=Step(a,straight,p);
    p.pair_mode=1;
    const auto pair=Step(b,straight,p);
    Check(old.debug.valid_rows==3 && pair.debug.valid_rows==3,"Hai vạch thẳng phải hợp lệ");
    Near(old.debug.e,0,"Xe giữa làn"); Near(pair.debug.e,old.debug.e,"e hai chế độ với hai vạch thẳng");
    std::cout << "Golden: " << frames << " khung, dung sai 1e-9\n";
}

// T55c-test §2; spec số kỳ vọng: PNG 6010 do người dùng xác nhận, chế độ cũ phải khác.
void Frame6010() {
    auto p=ControlParams{}; p.pair_mode=1; p.right_lost_ms=0; p.steer_rate_dps=1000;
    const auto mask=ReadMask(std::string(T55_FIXTURE_DIR)+"/mask_8218_6010.png");
    Check(mask.cols==160 && mask.rows==96,"Sai kích thước PNG 6010");
    LaneController a,b;
    const auto r=Step(a,mask,p);
    p.pair_mode=0;
    const auto old=Step(b,mask,p);
    PrintRows(r,mask,p);
    const double expected[3][2]={{0.059,0.631},{0.103,0.534},{0.181,0.481}};
    for(size_t i=0;i<3;++i)
        Pair(r,i,expected[i][0],expected[i][1],(expected[i][0]+expected[i][1])/2,mask.cols,0.03);
    Near(r.debug.e,-0.33,"e khung 6010",0.05);
    Check(r.cmd.deg>0,"6010 phải lái trái (góc dương)");
    bool differs=false;
    Check(old.debug.rows.size()==3,"Chế độ cũ thiếu hàng");
    for(size_t i=1;i<3;++i)
        differs=differs || old.debug.rows[i].valid!=r.debug.rows[i].valid ||
            std::abs(old.debug.rows[i].left-r.debug.rows[i].left)>0.03*mask.cols ||
            std::abs(old.debug.rows[i].right-r.debug.rows[i].right)>0.03*mask.cols;
    Check(differs,"6010 không phân biệt được hai chế độ ở hàng giữa/xa");
}

// T55c-test §4; spec B3: kỳ vọng hình học suy từ cặp kề nhau, không đọc mã Scan.
void Unit(const std::string &name) {
    auto p=UnitParams();
    LaneController c;
    if(name=="single_right") AllPairs(Step(c,Uniform(p,{0.55}),p),0,0.55,0.275);
    else if(name=="single_left") AllPairs(Step(c,Uniform(p,{0.45}),p),0.45,1,0.725);
    else if(name=="empty") AllPairs(Step(c,Uniform(p,{}),p),-1,1,0.8);
    else if(name=="exact") AllPairs(Step(c,Uniform(p,{0.25,0.5,0.9}),p),0.25,0.5,0.375);
    else if(name=="near_previous" || name=="near_other" || name=="near_none" ||
            name=="reset" || name=="stale" || name=="invalid_reset" || name=="outside") {
        if(name!="near_none") {
            const bool other=name=="near_other";
            const double l=other ? 0.21 : 0.12, r=other ? 0.79 : 0.54;
            AllPairs(Step(c,Uniform(p,{l,r}),p),l,r,(l+r)/2);
        }
        if(name=="reset") c.Reset();
        if(name=="stale") {
            const auto stale=c.Step(Uniform(p,{0.12,0.54}),1000,1001+p.mask_max_age_ms,p);
            Check(stale.debug.reason=="MAT_NAN_CU","Mặt nạ cũ phải báo MAT_NAN_CU");
        }
        if(name=="invalid_reset") {
            const auto bad=Step(c,Uniform(p,{0.45,0.55}),p,1050);
            for(size_t i=0;i<3;++i) Invalid(bad,i);
        }
        const bool outside=name=="outside";
        const double middle=outside ? 0.45 : 0.49;
        const auto r=Step(c,Uniform(p,{0.1,middle,0.85}),p,2000);
        if(name=="near_previous") AllPairs(r,0.1,0.49,0.295);
        else AllPairs(r,middle,0.85,(middle+0.85)/2);
    } else if(name=="near_tie") {
        p.lane_width_frac={0.3,0.3,0.3}; p.Validate();
        AllPairs(Step(c,Uniform(p,{0.44,0.62}),p),0.44,0.62,0.53);
        // Cả hai cặp khớp trong 0,15; tổng lệch phải thắng tâm gần xe hơn.
        AllPairs(Step(c,Uniform(p,{0.35,0.51,0.66}),p,1050),0.51,0.66,0.585);
    } else if(name=="upper_gap") {
        const auto r=Step(c,Mask(p,{{0.3,0.7},{0.01,0.1,0.9,0.99},{0.32,0.72}}),p);
        Pair(r,0,0.3,0.7,0.5); Invalid(r,1); Pair(r,2,0.32,0.72,0.52);
    } else if(name=="upper_tie") {
        const auto r=Step(c,Mask(p,{{0.3,0.7},{0.1,0.5,0.9},{0.1,0.5,0.9}}),p);
        Pair(r,0,0.3,0.7,0.5); Pair(r,1,0.1,0.5,0.3); Pair(r,2,0.1,0.5,0.3);
    } else if(name=="shift_limit") {
        p.pair_max_shift_frac=0.05; p.Validate();
        auto r=Step(c,Mask(p,{{0.3,0.7},{0.35,0.75},{0.35,0.75}}),p);
        Pair(r,1,0.35,0.75,0.55); Pair(r,2,0.35,0.75,0.55);
        c.Reset();
        r=Step(c,Mask(p,{{0.3,0.7},{0.351,0.751},{0.351,0.751}}),p);
        Pair(r,0,0.3,0.7,0.5); Invalid(r,1); Invalid(r,2);
    } else if(name=="near_invalid") {
        const auto r=Step(c,Mask(p,{{0.45,0.55},{0.3,0.7},{0.3,0.7}}),p);
        for(size_t i=0;i<3;++i) Invalid(r,i);
        Check(r.debug.valid_rows==0,"Không có mốc near thì không hàng nào hợp lệ");
    } else if(name=="width") {
        // Cặp mép đường rộng 0,1 vẫn hợp lệ, nhưng phép ra tâm cũ phải bỏ other.
        const auto mask=Mask(p,{{},{},{}},0.45,0.55);
        AllPairs(Step(c,mask,p),-1,0.55,0.35);
        LaneController left;
        const auto r=left.Step(mask,1000,1000,p,control::LaneRef::LEFT);
        AllPairs(r,0.45,-1,0.65);
        c.Reset();
        const auto bad=Step(c,Uniform(p,{0.45,0.55}),p);
        for(size_t i=0;i<3;++i) Invalid(bad,i);
    } else if(name=="offset") {
        const auto mask=Uniform(p,{0.2,0.6,0.9});
        AllPairs(Step(c,mask,p),0.2,0.6,0.4);
        c.Reset(); p.center_offset_frac=0.2; p.Validate();
        AllPairs(Step(c,mask,p),0.6,0.9,0.75);
    } else Check(false,"Không biết ca đơn vị: "+name);
}

// T55c-test §5: kiểm Load ngắn, cả YAML thường và tiêu đề OpenCV; không sửa test khóa cũ.
void Params(const std::string &path) {
    struct Cleanup { std::string path; ~Cleanup() { std::remove(path.c_str()); } } cleanup{path};
    for(bool header:{false,true}) {
        auto load=[&](const std::string &body) {
            std::ofstream out(path); Check(bool(out),"Không mở được YAML tạm");
            if(header) out << "%YAML:1.0\n---\n";
            out << body; out.close(); Check(bool(out),"Không ghi được YAML tạm");
            std::ostringstream report;
            return ControlParams::Load(path,report);
        };
        const auto d=load("{}\n");
        Check(d.pair_mode==0,"Mặc định pair_mode qua Load");
        Near(d.pair_max_shift_frac,0.25,"Mặc định pair_max_shift_frac qua Load");
        for(const auto &key:std::vector<std::string>{"pair_mode","pair_max_shift_frac"}) {
            const std::vector<std::string> values=key=="pair_mode" ?
                std::vector<std::string>{"-1","0","1","2","0.5"} :
                std::vector<std::string>{"0.04","0.05","0.6","0.61"};
            for(const auto &v:values) {
                const bool accept=key=="pair_mode" ? (v=="0" || v=="1") : (v=="0.05" || v=="0.6");
                bool rejected=false;
                ControlParams q;
                try { q=load(key+": "+v+"\n"); }
                catch(const std::exception &e) {
                    rejected=true;
                    Check(!accept && std::string(e.what()).find(key)!=std::string::npos,
                          "Từ chối sai hoặc lỗi thiếu tên khóa: "+key+"="+v+"; "+e.what());
                }
                Check(rejected!=accept,"Load nhận/từ chối sai: "+key+"="+v);
                if(accept) Near(key=="pair_mode" ? q.pair_mode : q.pair_max_shift_frac,Number(v),"Giá trị sau Load");
            }
        }
    }
}

// T55c-test §6: giữ lịch sử trước cửa sổ so sánh, thời gian tương đối từ dòng log đầu.
int Compare(const std::string &dir, double from, double to) {
    Check(from>=0 && to>=from,"Khoảng thời gian phải có 0 <= từ <= đến");
    std::ifstream in(dir+"/control_log.csv");
    std::string line;
    Check(bool(std::getline(in,line)),"Không đọc được control_log.csv");
    const auto header=Csv(line);
    auto column=[&](const std::string &key) {
        const auto it=std::find(header.begin(),header.end(),key);
        Check(it!=header.end() && std::count(header.begin(),header.end(),key)==1,"Thiếu/trùng cột "+key);
        return static_cast<size_t>(it-header.begin());
    };
    const size_t time=column("t_ms"), file=column("mask_file");
    ControlParams p; p.right_lost_ms=0;
    LaneController a,b;
    double first=-1,previous=-1;
    size_t frames=0,different=0,sameRows=0,rows=0;
    while(std::getline(in,line)) {
        const auto f=Csv(line);
        Check(f.size()==header.size(),"Log sai số cột");
        const double ms=Number(f[time]);
        Check(ms>=0 && ms<1e12 && std::floor(ms)==ms && ms>=previous,"t_ms log không hợp lệ/đi lùi");
        previous=ms;
        if(first<0) first=ms;
        const double seconds=(ms-first)/1000;
        if(seconds>to) break;
        if(f[file].empty()) continue;
        const auto mask=ReadMask(dir+"/"+f[file]);
        p.pair_mode=0; const auto old=Step(a,mask,p,static_cast<int64_t>(ms));
        p.pair_mode=1; const auto pair=Step(b,mask,p,static_cast<int64_t>(ms));
        if(seconds<from) continue;
        Check(old.debug.rows.size()==3 && pair.debug.rows.size()==3,"Thiếu hàng log so sánh");
        bool differs=false;
        for(size_t i=0;i<3;++i) {
            const auto &x=old.debug.rows[i], &y=pair.debug.rows[i];
            const bool same=x.valid==y.valid && std::abs(x.left-y.left)<=0.01*mask.cols &&
                std::abs(x.right-y.right)<=0.01*mask.cols;
            sameRows+=same; ++rows; differs=differs || !same;
        }
        ++frames;
        if(differs) {
            ++different;
            std::cout << "t_s=" << seconds << ", t_ms=" << ms << ", mask=" << f[file]
                      << ", e=" << old.debug.e << " -> " << pair.debug.e << '\n';
            for(size_t i=0;i<3;++i) {
                const auto &x=old.debug.rows[i], &y=pair.debug.rows[i];
                auto frac=[&](double v) { return v<0 ? -1.0 : v/mask.cols; };
                std::cout << "  y_frac=" << p.scan_rows[i] << ": (" << frac(x.left) << ',' << frac(x.right)
                          << ", valid=" << x.valid << ") -> (" << frac(y.left) << ',' << frac(y.right)
                          << ", valid=" << y.valid << ")\n";
            }
        }
    }
    Check(!in.bad() && frames>0,"Không có khung mask trong khoảng hoặc lỗi đọc log");
    std::cout << "So " << frames << " khung; khác " << different << "; hàng giống " << sameRows
              << '/' << rows << " = " << 100.0*sameRows/rows << "%\n";
    return sameRows*100>=rows*95 ? 0 : 1;
}
}

int main(int argc, char **argv) {
    try {
        std::locale::global(std::locale::classic());
        std::cout << std::setprecision(12);
        Check(argc>=2,"Cần tên ca, --print-pairs <mask.png> [pair_mode] hoặc --compare-sessions <run_dir> <từ_s> <đến_s>");
        const std::string name=argv[1];
        if(name=="--print-pairs") {
            Check(argc==3 || argc==4,"Cần mask.png và tùy chọn pair_mode");
            ControlParams p; p.right_lost_ms=0;
            const double mode=argc==4 ? Number(argv[3]) : 1;
            Check(mode==0 || mode==1,"pair_mode chỉ nhận 0/1"); p.pair_mode=static_cast<int>(mode);
            const auto mask=ReadMask(argv[2]); LaneController c;
            std::cout << "y_frac,left_frac,right_frac,center_frac,valid\n";
            PrintRows(Step(c,mask,p),mask,p);
        } else if(name=="--compare-sessions") {
            Check(argc==5,"Cần run_dir, t_từ_s, t_đến_s");
            return Compare(argv[2],Number(argv[3]),Number(argv[4]));
        } else if(name=="params") {
            Check(argc==3,"Cần đường dẫn YAML tạm riêng");
            // Tự kiểm bộ đọc CSV dùng cho công cụ tay, gồm nháy kép và cột mask trống.
            Check(Csv("10,\"masks/a,\"\"b\"\".png\",\r")==std::vector<std::string>{"10","masks/a,\"b\".png",""},"Bộ đọc CSV làm mất tên mask");
            bool rejected=false;
            try { Csv("10,\"chưa đóng"); } catch(const std::exception &) { rejected=true; }
            Check(rejected,"CSV thiếu dấu nháy phải bị từ chối");
            Params(argv[2]);
        } else {
            Check(argc==2,"Thừa tham số");
            if(name=="off") Off();
            else if(name=="6010") Frame6010();
            else if(name=="pair_6100") {
                // T55c-test §3: khung 6100 CHƯA được người dùng xác nhận, chỉ in, không ctest.
                const auto mask=ReadMask(std::string(T55_FIXTURE_DIR)+"/mask_8218_6100.png");
                for(int mode:{0,1}) {
                    ControlParams p; p.pair_mode=mode; p.right_lost_ms=0; p.steer_rate_dps=1000;
                    LaneController c;
                    std::cout << "pair_mode=" << mode << " (6100 chưa xác nhận)\ny_frac,left_frac,right_frac,center_frac,valid\n";
                    PrintRows(Step(c,mask,p),mask,p);
                }
            } else Unit(name);
        }
        std::cout << "ĐẠT: " << name << '\n';
        return 0;
    } catch(const std::exception &e) {
        std::cerr << "KHÔNG ĐẠT: " << e.what() << '\n';
        return 1;
    }
}
