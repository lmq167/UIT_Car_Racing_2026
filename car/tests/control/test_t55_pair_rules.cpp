#include "control/LaneController.h"
#include "control/Planner.h"
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
    else if(name=="empty") AllPairs(Step(c,Uniform(p,{}),p),-1,1,1-0.4/4);
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

// T55d §1: tọa độ nguyên trên 1000 cột, kiểm riêng hai phía của xe.
void NearBoundary(bool leftSide) {
    auto p=UnitParams();
    for(int distance:{29,30,31}) {
        const int middle=500+(leftSide ? -distance : distance);
        const double x=middle/1000.0;
        const std::vector<double> lines=leftSide ? std::vector<double>{x-0.2,x,0.95} :
                                                       std::vector<double>{0.05,x,x+0.2};
        LaneController c;
        const auto r=Step(c,Uniform(p,lines),p);
        const bool near=distance<30;
        const double l=leftSide ? (near ? x-0.2 : x) : (near ? x : 0.05);
        const double right=leftSide ? (near ? x : 0.95) : (near ? x+0.2 : x);
        std::cout << "sat_vach phia=" << (leftSide ? "trai" : "phai") << ", px=" << distance << '\n';
        AllPairs(r,l,right,(l+right)/2);
    }
}

// T55d §1: chỉ một mép ở biên khớp 0,15; mép kia lệch 0,05.
void MatchBoundary(bool leftEdge) {
    auto p=UnitParams();
    for(int distance:{149,150,151}) {
        LaneController c;
        const double l=leftEdge ? (100+distance)/1000.0 : 0.46;
        const double right=leftEdge ? 0.54 : (900-distance)/1000.0;
        AllPairs(Step(c,Uniform(p,{l,right}),p),l,right,(l+right)/2);
        const auto r=Step(c,Uniform(p,leftEdge ? std::vector<double>{0.1,0.49,0.85} :
                                                            std::vector<double>{0.15,0.51,0.9}),p,1050);
        const bool match=distance<=150;
        const double a=leftEdge ? (match ? 0.1 : 0.49) : (match ? 0.51 : 0.15);
        const double b=leftEdge ? (match ? 0.49 : 0.85) : (match ? 0.9 : 0.51);
        std::cout << "khop_mep=" << (leftEdge ? "trai" : "phai") << ", px=" << distance << '\n';
        AllPairs(r,a,b,(a+b)/2);
    }
}

// T55d §3, spec B3: một vạch vẫn ghép được với mép ảnh; chưa có làn thì không chạy mò.
void LostCommands(const std::string &name) {
    auto p=UnitParams();
    cv::Mat mask;
    if(name=="single_commands") mask=Uniform(p,{0.55});
    else if(name=="near_invalid_commands") mask=Mask(p,{{0.45,0.55},{0.3,0.7},{0.3,0.7}});
    else Check(name=="empty_mat_commands","Sai ca mất làn");
    LaneController c;
    const auto r=Step(c,mask,p);
    control::Planner planner;
    control::PlannerInput input;
    input.mask=mask; input.mask_ms=1000; input.image_ms=1000; input.go=true;
    const auto plan=planner.Step(input,1000,p);
    std::cout << "Lane state=" << r.debug.state << ", reason=" << r.debug.reason << ", cms=" << r.cmd.cms
              << "; Planner state=" << control::StateName(plan.state) << ", reason=" << plan.reason
              << ", debug.state=" << plan.debug.state << ", rows=" << plan.debug.valid_rows << '\n';
    if(name=="single_commands") {
        AllPairs(r,0,0.55,0.275);
        Check(r.debug.valid_rows==3 && r.debug.state=="BAM_LAN" && r.cmd.cms>0,
              "Một vạch + mép ảnh đủ ba hàng: phải bám làn");
        Check(plan.state==control::PlannerState::BAM_LAN && plan.cmd.cms>0,"G với một vạch hợp lệ phải bám làn");
    } else {
        Check(r.debug.valid_rows==0,"Khung đầu không có hàng hợp lệ");
        for(const auto &row:r.debug.rows) Check(!row.valid,"Không được bịa hàng hợp lệ");
        Check(r.debug.state=="MAT_LAN","Khung đầu không có làn phải MAT_LAN");
        Near(r.cmd.cms,0,"Chưa từng có làn: tốc độ 0"); Near(r.cmd.deg,0,"Chưa từng có làn: góc 0");
        if(mask.empty()) {
            control::Planner old;
            auto legacy=p; legacy.pair_mode=0;
            const auto before=old.Step(input,1000,legacy);
            Check(plan.state==before.state && plan.reason==before.reason && plan.debug.state==before.debug.state &&
                  plan.debug.valid_rows==before.debug.valid_rows,"Mat rỗng phải giữ đường xử lý trước B3");
        } else Check(plan.debug.valid_rows==0 && plan.debug.state=="MAT_LAN","G không được bỏ qua đường mất làn");
        Near(plan.cmd.cms,0,"G không có làn: tốc độ 0"); Near(plan.cmd.deg,0,"G không có làn: góc 0");
        if(name=="near_invalid_commands") for(size_t i=0;i<3;++i) Invalid(r,i);
    }
}

// T55d §4: kiểm độc lập Validate và Load, cả YAML thường lẫn OpenCV.
void NewParams(const std::string &path, bool order) {
    struct Cleanup { std::string path; ~Cleanup() { std::remove(path.c_str()); } } cleanup{path};
    auto reject=[&](const auto &action, const std::string &key) {
        bool rejected=false;
        try { action(); }
        catch(const std::exception &e) {
            rejected=true;
            Check(std::string(e.what()).find(key)!=std::string::npos,"Lỗi phải nêu "+key+": "+e.what());
        }
        Check(rejected,"Phải từ chối "+key);
    };
    auto load=[&](const std::string &body, bool header) {
        std::ofstream out(path);
        Check(bool(out),"Không mở được YAML tạm");
        if(header) out << "%YAML:1.0\n---\n";
        out << body; out.close(); Check(bool(out),"Không ghi được YAML tạm");
        std::ostringstream report;
        return ControlParams::Load(path,report);
    };
    if(order) {
        for(const auto &rows:std::vector<std::vector<double>>{{0.45,0.65,0.85},{0.85,0.85,0.45}}) {
            for(int mode:{0,1}) {
                ControlParams p; p.pair_mode=mode; p.scan_rows=rows;
                if(mode) reject([&] { p.Validate(); },"scan_rows"); else p.Validate();
                std::ostringstream body;
                body << "pair_mode: " << mode << "\nscan_rows: [" << rows[0] << ',' << rows[1] << ',' << rows[2] << "]\n";
                for(bool header:{false,true}) {
                    if(mode) reject([&] { load(body.str(),header); },"scan_rows");
                    else Check(load(body.str(),header).scan_rows==rows,"pair_mode=0 phải nhận thứ tự cũ");
                }
            }
        }
        for(bool header:{false,true}) Check(load("pair_mode: 1\n",header).pair_mode==1,"Thứ tự mặc định phải nhận");
    } else {
        Near(ControlParams{}.pair_merge_frac,0.08,"Mặc định gộp vạch");
        for(bool header:{false,true}) Near(load("{}\n",header).pair_merge_frac,0.08,"Mặc định gộp qua Load");
        for(const auto &v:std::vector<std::string>{"-0.01","0","0.019","0.02","0.12","0.121"}) {
            const bool accept=v=="0" || v=="0.02" || v=="0.12";
            for(int mode:{0,1}) {
                ControlParams p; p.pair_mode=mode; p.pair_merge_frac=Number(v);
                if(accept) p.Validate(); else reject([&] { p.Validate(); },"pair_merge_frac");
                for(bool header:{false,true}) {
                    const auto body="pair_mode: "+std::to_string(mode)+"\npair_merge_frac: "+v+"\n";
                    if(accept) Near(load(body,header).pair_merge_frac,Number(v),"Giá trị gộp qua Load");
                    else reject([&] { load(body,header); },"pair_merge_frac");
                }
            }
        }
    }
}

// T55d §5 và spec bổ sung: chỉ gộp danh sách ghép cặp, không sửa debug.lines thô.
void Merge(const std::string &name) {
    auto p=UnitParams();
    if(name=="merge_near_outer") {
        LaneController c;
        AllPairs(Step(c,Uniform(p,{0.25,0.29,0.7,0.75}),p),0.25,0.75,0.5);
        c.Reset(); p.center_offset_frac=0.15; p.Validate();
        AllPairs(Step(c,Uniform(p,{0.56,0.6,0.63,0.9}),p),0.56,0.9,0.73);
        return;
    }
    if(name=="merge_upper_roles") {
        // Cùng cụm [0,45;0,50;0,55]: mép phải chọn 0,55, mép trái chọn 0,45.
        for(bool rightRole:{true,false}) {
            LaneController c;
            const std::vector<double> lower=rightRole ? std::vector<double>{0.2,0.6} :
                                                       std::vector<double>{0.4,0.8};
            const auto r=Step(c,Mask(p,{lower,{0.2,0.45,0.5,0.55,0.8},{0.2,0.45,0.5,0.55,0.8}}),p);
            Pair(r,0,lower[0],lower[1],(lower[0]+lower[1])/2);
            const double left=rightRole ? 0.2 : 0.45, right=rightRole ? 0.55 : 0.8;
            for(size_t i=1;i<3;++i) Pair(r,i,left,right,(left+right)/2);
        }
        return;
    }
    if(name=="merge_7675_930") {
        ControlParams real; real.pair_mode=1; real.right_lost_ms=0;
        const auto mask=ReadMask(std::string(T55_FIXTURE_DIR)+"/masks_t55/7675_930.png");
        Check(mask.cols==160 && mask.rows==96,"Sai kích thước PNG 7675_930");
        LaneController c;
        const auto r=Step(c,mask,real);
        PrintRows(r,mask,real);
        Pair(r,2,0.64375,0.903125,(0.64375+0.903125)/2,mask.cols,0.005);
        return;
    }
    if(name=="merge_upper_anchor_row1") {
        LaneController c;
        const auto r=Step(c,Mask(p,{{0.2,0.6},{0.2,0.45,0.5,0.55,0.8},
                                  {0.2,0.45,0.5,0.55,0.6,0.65,0.8}}),p);
        Pair(r,0,0.2,0.6,0.4);
        Pair(r,1,0.2,0.55,0.375);
        Pair(r,2,0.2,0.55,0.375);
        return;
    }
    if(name=="merge_noise") {
        p.lane_width_min_ratio=0.75; p.Validate();
        LaneController c;
        const auto r=Step(c,Mask(p,{{0.3,0.7},{0.3,0.58,0.62},{0.3,0.58,0.62}}),p);
        Pair(r,0,0.3,0.7,0.5);
        for(size_t i=1;i<3;++i) Pair(r,i,0.3,0.62,0.46);
        return;
    }
    const std::vector<double> lines=name=="merge_chain" ? std::vector<double>{0.3,0.65,0.7,0.75} :
        name=="merge_default_gap_0p07" ? std::vector<double>{0.3,0.65,0.72} :
        name=="merge_exact" ? std::vector<double>{0.3,0.65,0.73} : std::vector<double>{0.3,0.68,0.72};
    const auto mask=Uniform(p,lines);
    if(name=="merge_disabled") p.pair_merge_frac=0;
    if(name=="merge_off") {
        p.pair_mode=0;
        LaneController a,b;
        const auto enabled=Step(a,mask,p);
        p.pair_merge_frac=0;
        const auto disabled=Step(b,mask,p);
        Near(enabled.cmd.cms,disabled.cmd.cms,"mode=0: tốc độ không đổi");
        Near(enabled.cmd.deg,disabled.cmd.deg,"mode=0: góc không đổi");
        Near(enabled.debug.e,disabled.debug.e,"mode=0: e không đổi");
        Check(enabled.debug.state==disabled.debug.state && enabled.debug.reason==disabled.debug.reason &&
              enabled.debug.valid_rows==disabled.debug.valid_rows,"mode=0: trạng thái không đổi");
        for(size_t i=0;i<3;++i) {
            const auto &x=enabled.debug.rows[i], &y=disabled.debug.rows[i];
            Check(x.valid==y.valid && x.lines==y.lines,"mode=0: valid/lines không đổi");
            Near(x.left,y.left,"mode=0: trái"); Near(x.right,y.right,"mode=0: phải"); Near(x.center,y.center,"mode=0: tâm");
        }
        return;
    }
    LaneController c;
    const auto r=Step(c,mask,p);
    for(const auto &row:r.debug.rows) {
        Check(row.lines.size()==lines.size(),"Gộp không được xóa vạch thô");
        for(size_t j=0;j<lines.size();++j) Near(row.lines[j],lines[j]*mask.cols,"Giữ vị trí vạch thô");
    }
    // Giữ tên CTest merge_mean cũ; nay kiểm thành viên ngoài cùng, không lấy trung bình.
    const double right=name=="merge_exact" ? 0.65 : name=="merge_disabled" ? 0.68 :
        name=="merge_chain" ? 0.75 : 0.72;
    AllPairs(r,0.3,right,(0.3+right)/2);
}

// T55d §2: dựng cùng đoạn đường/vạch như test B1 cũ; không suy ra vạch không có trong log.
void PairReplayB1() {
    std::ifstream in(std::string(T55_FIXTURE_DIR)+"/lane_run_7675_31p5_39p9.csv");
    std::string line;
    Check(bool(std::getline(in,line)) && Csv(line)==std::vector<std::string>{
        "t_s","t_ms","scan_rows","e","p","sent_steer_deg","go"},"Sai đầu đề fixture B1");
    const std::regex object(R"(\{[^{}]*\})");
    const std::regex field(R"re("([a-z_]+)"\s*:\s*(null|true|false|[-+0-9.eE]+))re");
    ControlParams p; p.pair_mode=1; p.right_lost_ms=500;
    LaneController a,b;
    double first=-1;
    size_t frames=0,different=0;
    int64_t previous=-1;
    while(std::getline(in,line)) {
        const auto f=Csv(line); Check(f.size()==7,"Sai số cột B1");
        cv::Mat mask(200,320,CV_8UC1,cv::Scalar(0));
        std::vector<std::pair<double,double>> edges;
        size_t i=0;
        for(std::sregex_iterator it(f[2].begin(),f[2].end(),object),end;it!=end;++it,++i) {
            std::vector<std::pair<std::string,std::string>> v;
            const auto body=it->str();
            for(std::sregex_iterator j(body.begin(),body.end(),field);j!=end;++j) v.emplace_back((*j)[1].str(),(*j)[2].str());
            const std::vector<std::string> keys={"y_frac","left_frac","right_frac","center_frac","valid"};
            Check(i<3 && v.size()==5,"Sai hàng fixture B1");
            for(size_t k=0;k<5;++k) Check(v[k].first==keys[k],"Sai khóa fixture B1");
            Near(Number(v[0].second),p.scan_rows[i],"Hàng B1",1e-6);
            Check(v[4].second=="true","Fixture B1 cần hàng hợp lệ");
            const int y=static_cast<int>(p.scan_rows[i]*mask.rows);
            const int right=static_cast<int>(std::lround(Number(v[2].second)*mask.cols));
            const int left=v[1].second=="null" ? -1 : static_cast<int>(std::lround(Number(v[1].second)*mask.cols));
            Check(right>0 && right<=mask.cols && left<right && left>=-1,"Mép fixture B1 ngoài ảnh");
            if(left>=0) {
                for(int x=left;x<right;++x) mask.at<unsigned char>(y,x)=1;
                if(left>0) mask.at<unsigned char>(y,left)=2;
            } else {
                // B3 cần đoạn mặt đường để có mép bị cắt khi fixture ghi left=null.
                for(int x=0;x<right;++x) mask.at<unsigned char>(y,x)=1;
            }
            if(right<mask.cols) mask.at<unsigned char>(y,right)=2;
            edges.emplace_back(left,right);
        }
        Check(i==3,"Thiếu hàng B1");
        const double ms=Number(f[1]), seconds=Number(f[0]);
        Check(ms>=0 && ms<1e12 && std::floor(ms)==ms && ms>previous,"Thời gian B1 không tăng");
        previous=static_cast<int64_t>(ms);
        const auto active=Step(a,mask,p,previous);
        auto off=p; off.right_lost_ms=0;
        const auto disabled=Step(b,mask,off,previous);
        Check(disabled.debug.state=="BAM_LAN","B3+B1=0 không được mất làn tại "+f[0]);
        bool differs=false;
        for(size_t j=0;j<3;++j) {
            const auto &row=disabled.debug.rows.at(j);
            differs=differs || !row.valid || std::abs(row.left-edges[j].first)>0.015*mask.cols ||
                std::abs(row.right-edges[j].second)>0.015*mask.cols;
        }
        if(differs) { ++different; std::cout << "B1 cap khac fixture t_s=" << seconds << '\n'; }
        if(active.debug.state!="BAM_LAN" && first<0) first=seconds;
        if(seconds<37.7) Check(active.debug.state=="BAM_LAN","B1+B3 kích hoạt trước 37,7 s");
        ++frames;
    }
    Check(in.eof() && frames==164,"Thiếu/thừa fixture B1");
    std::cout << "B1+B3: frames=" << frames << ", cap_khac_fixture=" << different << ", first_s=" << first << '\n';
    Check(first>=38.2 && first<=38.6,"B1+B3 phải kích hoạt trong 38,2–38,6 s");
}

std::pair<double,double> SelectedPair(const cv::Mat &mask, const control::RowDebug &row, const ControlParams &p,
                                     size_t j, double lowerLeft);
bool CarpetClear(const cv::Mat &mask, const ControlResult &r, const ControlParams &p, double carpet) {
    if(!r.debug.rows.at(2).valid) return true;
    std::pair<double,double> pair{};
    for(size_t j=0;j<3;++j)
        if(r.debug.rows[j].valid) pair=SelectedPair(mask,r.debug.rows[j],p,j,pair.first);
    const double leftDistance=std::abs(pair.first/mask.cols-carpet);
    const double rightDistance=std::abs(pair.second/mask.cols-carpet);
    std::cout << " cap_thuc=(" << pair.first/mask.cols << ',' << pair.second/mask.cols
              << "), distance_left=" << leftDistance << ", distance_right=" << rightDistance << '\n';
    return leftDistance>=0.03 && rightDistance>=0.03;
}

// T55d §6: mỗi khung controller mới; D chỉ in số liệu, không tham gia tiêu chí đạt/hỏng.
void Regression(const std::string &category) {
    std::ifstream in(std::string(T55_FIXTURE_DIR)+"/labels_b3.csv");
    std::string line;
    Check(bool(std::getline(in,line)) && Csv(line)==std::vector<std::string>{"run","frame","row","category","status",
        "old_left","old_right","old_valid","b3_left","b3_right","b3_valid","e_old","e_b3"},"Sai đầu đề nhãn B3");
    size_t frames=0,failures=0,less=0,selected=0,carpets=0;
    auto expect=[&](bool ok, const std::string &why) { if(!ok) { ++failures; std::cout << "LOI: " << why << '\n'; } };
    while(std::getline(in,line)) {
        std::vector<std::vector<std::string>> f{Csv(line)};
        for(int j=1;j<3;++j) { Check(bool(std::getline(in,line)),"Nhãn thiếu hàng"); f.push_back(Csv(line)); }
        for(size_t j=0;j<3;++j) Check(f[j].size()==13 && f[j][0]==f[0][0] && f[j][1]==f[0][1] &&
            f[j][2]==std::to_string(j) && f[j][3]==f[0][3] && f[j][4]==f[0][4],"Nhãn khung sai lược đồ");
        Check((f[0][3]=="D" && f[0][4]=="cho_Quan") ||
              ((f[0][3]=="A" || f[0][3]=="B" || f[0][3]=="C") && f[0][4]=="xet"),"Sai hạng mục/trạng thái nhãn");
        ++frames;
        if(category!="total" && f[0][3]!=category) continue;
        if(category=="total" && f[0][4]=="cho_Quan") continue;
        ++selected;
        ControlParams p; p.pair_mode=1; p.right_lost_ms=0;
        const auto mask=ReadMask(std::string(T55_FIXTURE_DIR)+"/masks_t55/"+f[0][0]+"_"+f[0][1]+".png");
        Check(mask.cols==160 && mask.rows==96,"Sai kích thước mặt nạ hồi quy");
        LaneController c;
        const auto r=Step(c,mask,p);
        Check(r.debug.rows.size()==3,"Thiếu hàng hồi quy");
        int oldValid=0;
        for(size_t j=0;j<3;++j) {
            const auto &v=f[j]; const auto &row=r.debug.rows[j];
            Check((v[7]=="0" || v[7]=="1") && (v[10]=="0" || v[10]=="1"),"valid nhãn phải 0/1");
            oldValid+=v[7]=="1";
            const std::string at=v[0]+"_"+v[1]+" row="+std::to_string(j);
            auto same=[&](size_t left, size_t right) {
                return std::abs(row.left/mask.cols-Number(v[left]))<=0.03 &&
                       std::abs(row.right/mask.cols-Number(v[right]))<=0.03;
            };
            if(category=="D") {
                std::cout << "CHO " << at << " old=(" << v[5] << ',' << v[6] << ',' << v[7]
                          << ") b3_truoc=(" << v[8] << ',' << v[9] << ',' << v[10] << ") moi=("
                          << row.left/mask.cols << ',' << row.right/mask.cols << ',' << row.valid
                          << ") e_old=" << v[11] << " e_b3=" << v[12] << " e_moi=" << r.debug.e << '\n';
            } else if(v[3]=="C" && j==2) {
                const double expectedCarpet=v[1]=="1520" ? 0.9625 : v[1]=="1530" ? 0.95625 :
                    v[1]=="1540" ? 0.94375 : v[1]=="1580" ? 0.921875 : -1;
                const double carpet=Number(v[6]);
                Check(v[0]=="7675" && expectedCarpet>=0,"Khung C không có nhãn biên thảm đã xác nhận");
                Near(carpet,expectedCarpet,"Biên thảm "+at,1e-6); ++carpets;
                std::cout << at << " moi=(" << row.left/mask.cols << ',' << row.right/mask.cols << ',' << row.valid
                          << ") carpet=" << carpet << '\n';
                expect(CarpetClear(mask,r,p,carpet),"C "+at);
            } else if(category!="total") {
                const bool ok=j<2 || category=="A" ? row.valid==(v[10]=="1") && same(8,9) :
                    row.valid && same(5,6);
                std::cout << at << " moi=(" << row.left/mask.cols << ',' << row.right/mask.cols << ',' << row.valid << ")\n";
                expect(ok,category+" "+at);
            }
        }
        if(category=="A") expect(std::isfinite(r.debug.e) && r.debug.e*Number(f[0][12])>0,"A: dấu e "+f[0][0]+"_"+f[0][1]);
        if(r.debug.valid_rows<oldValid && f[0][3]!="C") {
            ++less; std::cout << "IT_HANG " << f[0][0] << '_' << f[0][1] << " old=" << oldValid << " moi=" << r.debug.valid_rows << '\n';
        }
    }
    const size_t expectedSelected=category=="A" ? 11 : category=="B" ? 8 : category=="total" ? 23 : 4;
    Check(in.eof() && frames==27 && selected==expectedSelected,"Nhãn thiếu/thừa khung hoặc sai số khung hạng mục");
    if(category=="C" || category=="total") Check(carpets==4,"Phải kiểm đủ bốn nhãn biên thảm C");
    if(category=="D") std::cout << "CHO_QUAN: " << selected << " khung, không tính đạt/hỏng\n";
    else std::cout << "Hoi_quy " << category << ": selected=" << selected << ", loi=" << failures << ", it_hang_ngoai_C=" << less << '\n';
    Check(failures==0 && (category!="total" || less==0),"Hồi quy "+category+" chưa đạt; không nới sai số");
}

// T55d §7: nội suy tuyến tính trên mẫu đã sắp; gộp dịch trái/phải vào cùng phân bố.
double Percentile(const std::vector<double> &sorted, double q) {
    Check(!sorted.empty() && q>=0 && q<=1,"Mẫu percentile rỗng hoặc q ngoài miền");
    const double index=q*(sorted.size()-1);
    const auto low=static_cast<size_t>(std::floor(index)), high=static_cast<size_t>(std::ceil(index));
    return sorted[low]+(sorted[high]-sorted[low])*(index-low);
}
// Spec "từ cặp ra hàng": left=-1 che mép đối diện; khôi phục qua tính kề nhau, không dùng tâm chiếu.
std::pair<double,double> SelectedPair(const cv::Mat &mask, const control::RowDebug &row, const ControlParams &p,
                                     size_t j, double lowerLeft) {
    Check(row.valid && row.right>=0 && row.right<=mask.cols && row.y>=0 && row.y<mask.rows,"Hàng đo dịch thiếu mép phải/y");
    if(row.left>=0) return {row.left,row.right};
    double left=-1;
    for(size_t i=0;i<row.lines.size();) {
        size_t end=i+1;
        while(end<row.lines.size() && row.lines[end]-row.lines[end-1]<p.pair_merge_frac*mask.cols)
            ++end;
        double edge=row.lines[i];
        const double anchor=j==0 ? (0.5+p.center_offset_frac)*mask.cols : lowerLeft;
        for(size_t k=i+1;k<end;++k)
            if(j==0 ? std::abs(row.lines[k]-anchor)>std::abs(edge-anchor) :
                      std::abs(row.lines[k]-anchor)<std::abs(edge-anchor)) edge=row.lines[k];
        // Bỏ cả cụm chứa mép phải, kể cả khi mép phải không phải thành viên đầu cụm.
        if(row.lines[end-1]<row.right-1e-9) left=edge;
        i=end;
    }
    if(left<0) {
        int x=std::min(mask.cols-1,static_cast<int>(std::floor(row.right)));
        if(mask.at<unsigned char>(row.y,x)==0) --x;
        while(x>=0 && mask.at<unsigned char>(row.y,x)!=0) --x;
        left=x+1;
    }
    Check(left>=0 && left<row.right,"Không khôi phục được cặp kề từ debug/mặt nạ");
    return {left,row.right};
}
void ShiftStats(const std::string &dir, double merge, double maxShift) {
    Near(Percentile({0,0.1,0.2,0.3},0.5),0.15,"Tự kiểm p50");
    Near(Percentile({0,0.1,0.2,0.3},0.9),0.27,"Tự kiểm p90");
    Near(Percentile({0,0.1,0.2,0.3},0.99),0.297,"Tự kiểm p99");
    Near(Percentile({0.2},0.99),0.2,"Tự kiểm một mẫu");
    auto checkParams=UnitParams();
    const auto checkMask=Uniform(checkParams,{0.3,0.65,0.7,0.75});
    control::RowDebug hidden;
    hidden.y=static_cast<int>(checkParams.scan_rows[0]*checkMask.rows); hidden.valid=true;
    hidden.right=1000; hidden.lines={300,650,700,750};
    Near(SelectedPair(checkMask,hidden,checkParams,0,0).first,750,"Tự kiểm mép bị che: ngoài cùng");
    checkParams.center_offset_frac=0.4;
    Near(SelectedPair(checkMask,hidden,checkParams,0,0).first,650,"Tự kiểm mép bị che: vị trí xe lệch");
    checkParams.center_offset_frac=0;
    hidden.y=static_cast<int>(checkParams.scan_rows[1]*checkMask.rows);
    Near(SelectedPair(checkMask,hidden,checkParams,1,680).first,700,"Tự kiểm mép bị che: gần mép hàng dưới");
    hidden.right=750;
    Near(SelectedPair(checkMask,hidden,checkParams,1,680).first,300,"Tự kiểm bỏ cụm chứa mép phải");
    hidden.right=1000;
    checkParams.pair_merge_frac=0;
    Near(SelectedPair(checkMask,hidden,checkParams,1,680).first,750,"Tự kiểm mép bị che: không gộp");
    hidden.right=300;
    Near(SelectedPair(checkMask,hidden,checkParams,1,680).first,0,"Tự kiểm mép bị cắt");
    ControlParams p; p.pair_mode=1; p.right_lost_ms=0; p.pair_max_shift_frac=maxShift; p.pair_merge_frac=merge; p.Validate();
    std::ifstream in(dir+"/control_log.csv"); std::string line;
    Check(bool(std::getline(in,line)),"Không đọc được control_log.csv");
    const auto header=Csv(line);
    auto column=[&](const std::string &key) {
        const auto it=std::find(header.begin(),header.end(),key);
        Check(it!=header.end() && std::count(header.begin(),header.end(),key)==1,"Thiếu/trùng cột "+key);
        return static_cast<size_t>(it-header.begin());
    };
    const auto time=column("t_ms"), file=column("mask_file");
    LaneController c; size_t frames=0,pairs=0; double previous=-1;
    size_t samples25=0,samples35=0,pairs25=0,pairs35=0;
    std::vector<double> shifts;
    while(std::getline(in,line)) {
        const auto f=Csv(line); Check(f.size()==header.size(),"Log sai số cột");
        const double ms=Number(f[time]);
        Check(ms>=0 && ms<1e12 && std::floor(ms)==ms && ms>=previous,"t_ms sai/đi lùi"); previous=ms;
        if(f[file].empty()) continue;
        const auto mask=ReadMask(dir+"/"+f[file]);
        const auto r=Step(c,mask,p,static_cast<int64_t>(ms)); ++frames;
        Check(r.debug.rows.size()==p.scan_rows.size(),"Thiếu hàng khi đo dịch");
        std::pair<double,double> lower{};
        for(size_t j=0;j<r.debug.rows.size();++j) {
            const auto &row=r.debug.rows[j];
            if(!row.valid) continue;
            const auto current=SelectedPair(mask,row,p,j,lower.first);
            if(j>0 && r.debug.rows[j-1].valid) {
                const double left=std::abs(lower.first-current.first)/mask.cols;
                const double right=std::abs(lower.second-current.second)/mask.cols;
                shifts.push_back(left); shifts.push_back(right); ++pairs;
                samples25+=(left>0.25)+(right>0.25); samples35+=(left>0.35)+(right>0.35);
                pairs25+=std::max(left,right)>0.25; pairs35+=std::max(left,right)>0.35;
            }
            lower=current;
        }
    }
    Check(in.eof() && frames>0,"Không có mặt nạ hoặc lỗi đọc log");
    std::cout << "frames=" << frames << ", adjacent_pairs=" << pairs << ", samples=" << shifts.size()
              << ", pair_merge_frac=" << merge << ", pair_max_shift_frac=" << maxShift << '\n';
    Check(!shifts.empty(),"Không có hai hàng liền nhau đủ mép để đo");
    std::sort(shifts.begin(),shifts.end());
    std::cout << "p50=" << Percentile(shifts,0.5) << ", p90=" << Percentile(shifts,0.9)
              << ", p99=" << Percentile(shifts,0.99) << ", max=" << shifts.back() << '\n';
    std::cout << "samples_gt_0.25=" << samples25 << ", samples_gt_0.35=" << samples35
              << ", adjacent_pairs_gt_0.25=" << pairs25 << ", adjacent_pairs_gt_0.35=" << pairs35 << '\n';
}
}

int main(int argc, char **argv) {
    try {
        std::locale::global(std::locale::classic());
        std::cout << std::setprecision(12);
        Check(argc>=2,"Cần tên ca, --print-pairs <mask.png> [pair_mode] hoặc --compare-sessions <run_dir> <từ_s> <đến_s>");
        const std::string name=argv[1];
        if(name=="--shift-stats") {
            Check(argc>=3 && argc<=5,"Cần --shift-stats <run_dir> [pair_merge_frac] [pair_max_shift_frac]");
            ShiftStats(argv[2],argc>=4 ? Number(argv[3]) : ControlParams{}.pair_merge_frac,
                       argc==5 ? Number(argv[4]) : 0.6);
        } else if(name=="params_order" || name=="params_merge") {
            Check(argc==3,"Cần đường dẫn YAML tạm riêng");
            NewParams(argv[2],name=="params_order");
        } else if(name=="--print-pairs") {
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
            else if(name=="near_boundary_left" || name=="near_boundary_right") NearBoundary(name=="near_boundary_left");
            else if(name=="match_boundary_left" || name=="match_boundary_right") MatchBoundary(name=="match_boundary_left");
            else if(name=="empty_mat_commands" || name=="single_commands" || name=="near_invalid_commands") LostCommands(name);
            else if(name=="replay_b1_pair") PairReplayB1();
            else if(name.compare(0,6,"merge_")==0) Merge(name);
            else if(name.compare(0,8,"regress_")==0) {
                Regression(name.substr(8));
                if(name=="regress_D") return 0;
            }
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
