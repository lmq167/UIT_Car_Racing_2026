#include "control/LaneController.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace {
using control::ControlParams;
using control::ControlResult;
using control::LaneController;

void Check(bool ok, const std::string &message) {
    if(!ok) throw std::runtime_error(message);
}
void Near(double actual, double expected, const std::string &message, double tolerance=1e-9) {
    Check(std::isfinite(actual) && std::isfinite(expected) && std::abs(actual-expected)<=tolerance,
          message+": nhận "+std::to_string(actual)+", cần "+std::to_string(expected));
}
double Number(const std::string &text) {
    size_t used=0;
    const double value=std::stod(text,&used);
    Check(used==text.size() && std::isfinite(value),"Số không hợp lệ: "+text);
    return value;
}
std::vector<std::string> Csv(const std::string &line) {
    std::vector<std::string> fields;
    std::string field;
    bool quoted=false;
    for(size_t i=0;i<line.size();++i) {
        const char c=line[i];
        if(c=='"') {
            if(quoted && i+1<line.size() && line[i+1]=='"') { field+='"'; ++i; }
            else quoted=!quoted;
        } else if(c==',' && !quoted) { fields.push_back(field); field.clear(); }
        else if(c!='\r') field+=c;
    }
    Check(!quoted,"CSV chưa đóng dấu nháy");
    fields.push_back(field);
    return fields;
}
std::string QuoteCsv(const std::string &value) {
    std::string out="\"";
    for(char c:value) { if(c=='"') out+='"'; out+=c; }
    return out+'"';
}
struct Row { double y, left, right, center; bool valid; };
struct Frame { double seconds; int64_t ms; std::vector<Row> rows; double e; };

// Fixture có lược đồ cố định; đọc null riêng để không nhầm mép trái 0 với mép bị thiếu.
std::vector<Row> Rows(const std::string &text) {
    static const std::regex object(R"(\{[^{}]*\})");
    static const std::regex field(R"re("([a-z_]+)"\s*:\s*(null|true|false|[-+0-9.eE]+))re");
    std::vector<Row> rows;
    for(std::sregex_iterator it(text.begin(),text.end(),object), end; it!=end; ++it) {
        const std::string body=it->str();
        std::vector<std::pair<std::string,std::string>> values;
        for(std::sregex_iterator f(body.begin(),body.end(),field); f!=end; ++f)
            values.emplace_back((*f)[1].str(),(*f)[2].str());
        const std::vector<std::string> keys={"y_frac","left_frac","right_frac","center_frac","valid"};
        Check(values.size()==keys.size(),"scan_rows sai lược đồ");
        for(size_t i=0;i<keys.size();++i) Check(values[i].first==keys[i],"scan_rows sai khóa/thứ tự");
        auto coordinate=[&](size_t i) {
            if(values[i].second=="null") return -1.0;
            const double v=Number(values[i].second);
            Check(v>=0 && v<=1,"Tọa độ fixture ngoài [0,1]");
            return v;
        };
        Check(values[4].second=="true" || values[4].second=="false","valid phải là bool");
        rows.push_back({coordinate(0),coordinate(1),coordinate(2),coordinate(3),values[4].second=="true"});
        const auto &r=rows.back();
        Check(r.y>0 && r.y<1 && (!r.valid || (r.right>=0 && r.center>=0)),"Hàng fixture thiếu tọa độ");
    }
    Check(!rows.empty(),"scan_rows rỗng");
    return rows;
}
std::vector<Frame> Fixture() {
    std::ifstream in(std::string(T55_FIXTURE_DIR)+"/lane_run_7675_31p5_39p9.csv");
    std::string line;
    Check(bool(std::getline(in,line)) && Csv(line)==std::vector<std::string>{
        "t_s","t_ms","scan_rows","e","p","sent_steer_deg","go"},"Thiếu/sai đầu đề fixture");
    std::vector<Frame> frames;
    while(std::getline(in,line)) {
        const auto f=Csv(line);
        Check(f.size()==7,"Fixture phải có 7 cột");
        const double ms=Number(f[1]);
        Check(ms>=0 && ms<1e12 && std::floor(ms)==ms,"t_ms phải là số nguyên");
        frames.push_back({Number(f[0]),static_cast<int64_t>(ms),Rows(f[2]),Number(f[3])});
        if(frames.size()>1) {
            const auto &previous=frames[frames.size()-2];
            Check(frames.back().ms>previous.ms && frames.back().seconds>previous.seconds,"Fixture không tăng thời gian");
        }
    }
    Check(in.eof() && frames.size()==164,"Fixture thiếu dòng (cần 164)");
    Near(frames.front().seconds,31.529,"Mốc đầu fixture");
    Near(frames.back().seconds,39.888,"Mốc cuối fixture");
    return frames;
}

cv::Mat Mask(const std::vector<Row> &rows, const ControlParams &p, int cols=320, int height=200) {
    Check(rows.size()==p.scan_rows.size(),"Số hàng fixture khác tham số mặc định");
    cv::Mat mask(height,cols,CV_8UC1,cv::Scalar(0));
    for(size_t i=0;i<rows.size();++i) {
        const auto &r=rows[i];
        Near(r.y,p.scan_rows[i],"Vị trí hàng quét",1e-6);
        if(!r.valid) continue;
        const int y=static_cast<int>(p.scan_rows[i]*height);
        const int right=static_cast<int>(std::lround(r.right*cols));
        Check(right>0 && right<=cols,"Mép phải không dựng được");
        if(r.left>=0) {
            const int left=static_cast<int>(std::lround(r.left*cols));
            Check(left>=0 && left<right,"Hai mép không có bề rộng");
            for(int x=left;x<right;++x) mask.at<unsigned char>(y,x)=1;
            if(left>0) mask.at<unsigned char>(y,left)=2;
        } else if(right==cols) {
            // Chỉ mép cuối ảnh: không dựng thêm biên trái mà log ghi null.
            mask.at<unsigned char>(y,cols-1)=1;
        }
        if(right<cols) mask.at<unsigned char>(y,right)=2;
    }
    return mask;
}
ControlResult Step(LaneController &c, const cv::Mat &mask, int64_t now, const ControlParams &p) {
    return c.Step(mask,now,now,p,control::LaneRef::RIGHT);
}
std::vector<ControlResult> Replay(const std::vector<Frame> &frames, const ControlParams &p) {
    LaneController c;
    std::vector<ControlResult> out;
    for(const auto &frame:frames) out.push_back(Step(c,Mask(frame.rows,p),frame.ms,p));
    return out;
}
bool RowMatches(const control::RowDebug &actual, const Row &expected, int cols, double tolerance=0.015) {
    auto match=[&](double a, double b) { return b<0 ? a<0 : a>=0 && std::abs(a/cols-b)<=tolerance; };
    return actual.valid==expected.valid && match(actual.left,expected.left) &&
           match(actual.right,expected.right) && match(actual.center,expected.center);
}

// T55a §1: ≥95% khung phải đồng thời khớp mọi L/R/C và sai số e trong log.
void CheckMask(const std::vector<Frame> &frames) {
    ControlParams p;
    size_t matched=0, rowMatched=0, totalRows=0, mismatches=0;
    for(const auto &frame:frames) {
        LaneController c; // Kiểm hình học độc lập, không tích lũy thời gian B1.
        const auto mask=Mask(frame.rows,p);
        const auto r=Step(c,mask,frame.ms,p);
        Check(r.debug.rows.size()==frame.rows.size(),"Thiếu debug hàng quét");
        bool ok=std::isfinite(r.debug.e) && std::abs(r.debug.e-frame.e)<=0.02;
        for(size_t i=0;i<frame.rows.size();++i) {
            const bool rowOk=RowMatches(r.debug.rows[i],frame.rows[i],mask.cols);
            rowMatched+=rowOk; ++totalRows; ok=ok && rowOk;
        }
        if(ok) ++matched;
        else if(mismatches++<10)
            std::cout << "Lệch mặt nạ tại t_s=" << frame.seconds << '\n';
    }
    std::cout << "Khớp mặt nạ/e: " << matched << '/' << frames.size()
              << " khung; L/R/C: " << rowMatched << '/' << totalRows << " hàng\n";
    Check(matched*100>=frames.size()*95,"Tự kiểm mặt nạ dưới 95%");
}

// T55a §1: chỉ leader gọi trên cây khung; CTest tuyệt đối không sinh golden.
void WriteGolden(const std::vector<Frame> &frames, const std::string &path) {
    CheckMask(frames);
    const auto results=Replay(frames,ControlParams{});
    for(const auto &r:results)
        Check(r.debug.state=="BAM_LAN","Từ chối sinh golden: cây đã có B1 hoặc mặt nạ sai");
    std::ofstream out(path);
    Check(bool(out),"Không mở được golden: "+path);
    out << "t_s,cmd.cms,cmd.deg,e,p,state,reason\n" << std::setprecision(17);
    for(size_t i=0;i<frames.size();++i) {
        const auto &r=results[i];
        out << frames[i].seconds << ',' << r.cmd.cms << ',' << r.cmd.deg << ',' << r.debug.e << ','
            << r.debug.p << ',' << QuoteCsv(r.debug.state) << ',' << QuoteCsv(r.debug.reason) << '\n';
    }
    out.close();
    Check(bool(out),"Ghi golden thất bại");
}

// T55 §4.1: tắt cả hai luật phải khớp bản fab24d1 tới 1e-9 từng khung.
void Disabled(const std::vector<Frame> &frames) {
    ControlParams p; p.right_lost_ms=0; p.kh=0;
    const auto results=Replay(frames,p);
    std::ifstream in(std::string(T55_FIXTURE_DIR)+"/lane_golden_fab24d1.csv");
    std::string line;
    Check(bool(std::getline(in,line)),"Thiếu golden: leader phải chạy --write-golden trên cây khung trước DEV");
    Check(Csv(line)==std::vector<std::string>{"t_s","cmd.cms","cmd.deg","e","p","state","reason"},"Sai đầu đề golden");
    for(size_t i=0;i<frames.size();++i) {
        Check(bool(std::getline(in,line)),"Golden thiếu dòng");
        const auto f=Csv(line);
        Check(f.size()==7,"Golden sai số cột");
        const auto &r=results[i];
        const std::string at=" tại t_s="+std::to_string(frames[i].seconds);
        Near(frames[i].seconds,Number(f[0]),"Thời gian golden"+at);
        Near(r.cmd.cms,Number(f[1]),"Tốc độ golden"+at);
        Near(r.cmd.deg,Number(f[2]),"Góc golden"+at);
        Near(r.debug.e,Number(f[3]),"e golden"+at);
        Near(r.debug.p,Number(f[4]),"p golden"+at);
        Check(r.debug.state==f[5] && r.debug.reason==f[6],"Trạng thái/lý do khác golden"+at);
    }
    Check(!std::getline(in,line) && in.eof(),"Golden thừa dòng/lỗi đọc");
}

// T55 §4.2: đoạn clip trong fixture bắt đầu 37,786 s; B1=500 kích hoạt khoảng 38,29 s.
void ReplayB1(const std::vector<Frame> &frames) {
    ControlParams p; p.kh=0; p.right_lost_ms=500;
    const auto active=Replay(frames,p);
    p.right_lost_ms=0;
    const auto disabled=Replay(frames,p);
    double first=-1, start=-1, previous=-1;
    for(size_t i=0;i<frames.size();++i) {
        bool any=false, all=true;
        for(const auto &row:frames[i].rows) if(row.valid) { any=true; all=all && row.right>=p.right_clip_frac; }
        if(any && all) { if(start<0) start=frames[i].seconds; }
        else if(start>=0) {
            if(previous-start>=0.2) std::cout << "Đoạn mọi R clip: " << start << " -> " << previous << '\n';
            Check(previous-start<0.2 || start>=37.7,"Có đoạn clip sớm: báo leader, không nới kỳ vọng");
            start=-1;
        }
        previous=frames[i].seconds;
        Check(disabled[i].debug.state=="BAM_LAN","B1=0 vẫn kích hoạt mất làn");
        if(active[i].debug.state=="MAT_LAN" && first<0) first=frames[i].seconds;
        if(frames[i].seconds<37.7) Check(active[i].debug.state=="BAM_LAN","B1 kích hoạt trước 37,7 s");
    }
    if(start>=0 && previous-start>=0.2) {
        std::cout << "Đoạn mọi R clip: " << start << " -> " << previous << '\n';
        Check(start>=37.7,"Có đoạn clip sớm: báo leader");
    }
    std::cout << "B1 kích hoạt lần đầu: " << first << " s\n";
    Check(first>=38.2 && first<=38.6,"B1 phải kích hoạt trong 38,2–38,6 s");
}

// T55 §4.3: cua trái được thêm ≥4 độ; cổng chặn hàng xa sai quanh 36,7 s.
void ReplayB2(const std::vector<Frame> &frames) {
    ControlParams p; p.right_lost_ms=0; p.kh=0;
    const auto baseline=Replay(frames,p);
    p.kh=12; p.kh_gate_frac=0.35;
    const auto active=Replay(frames,p);
    size_t turning=0, gated=0, rejected=0;
    double minimum=std::numeric_limits<double>::infinity();
    for(size_t i=0;i<frames.size();++i) {
        const double t=frames[i].seconds;
        if(t>=33 && t<=36.4) {
            // Khung mà hàng xa lệch hàng giữa quá kh_gate_frac (vd. 33,069 s: 0,405) bị cổng loại: h=0.
            const auto &r=frames[i].rows;
            const bool pass=r.size()>=3 && r.front().valid && r[r.size()/2].valid && r.back().valid &&
                            std::abs(r.back().center-r[r.size()/2].center)<=p.kh_gate_frac;
            if(!pass) {
                ++rejected;
                Near(active[i].debug.p-p.kp*active[i].debug.e,0,"Khung bị cổng loại phải có h=0 tại "+std::to_string(t));
                continue;
            }
            ++turning;
            Check(baseline[i].cmd.deg>0 && active[i].cmd.deg>0,"B2 phải giữ dấu lái trái dương");
            const double gain=active[i].cmd.deg-baseline[i].cmd.deg;
            minimum=std::min(minimum,gain);
            Check(gain>=4-1e-9,"B2 tăng lái dưới 4 độ tại "+std::to_string(t));
        }
        if(std::abs(t-36.7)<=0.05) {
            ++gated;
            Near(active[i].debug.p-p.kp*active[i].debug.e,0,"Cổng B2 không chặn tại "+std::to_string(t));
        }
    }
    Check(turning>0 && gated>0,"Thiếu cửa sổ replay B2");
    std::cout << "B2: " << turning << " khung cua; tăng nhỏ nhất " << minimum << " độ; " << rejected << " khung cổng loại trong 33-36,4 s; " << gated << " khung cổng\n";
}

ControlParams UnitParams() {
    ControlParams p;
    p.scan_rows={0.8,0.5,0.2}; p.row_weights={0.5,0.3,0.2}; p.lane_width_frac={0.4,0.4,0.4};
    p.min_valid_rows=1; p.kd=0; p.steer_rate_dps=1000; p.right_lost_ms=500; p.kh=0;
    p.Validate();
    return p;
}
std::vector<Row> Edges(const ControlParams &p, const std::vector<double> &right) {
    Check(right.size()==p.scan_rows.size(),"Sai số hàng dựng ca đơn vị");
    std::vector<Row> rows;
    for(size_t i=0;i<right.size();++i) {
        const double left=right[i]-p.lane_width_frac[i];
        rows.push_back({p.scan_rows[i],left,right[i],(left+right[i])/2,true});
    }
    return rows;
}
void Geometry(const ControlResult &r, const std::vector<Row> &rows, int cols=1000) {
    Check(r.debug.rows.size()==rows.size(),"Không đủ hàng kiểm tiền điều kiện");
    for(size_t i=0;i<rows.size();++i)
        Check(RowMatches(r.debug.rows[i],rows[i],cols,1e-9),"Mặt nạ ca đơn vị không tái tạo hàng "+std::to_string(i));
}
void Tracking(const ControlResult &r, const std::string &where) {
    Check(r.debug.state=="BAM_LAN" && r.cmd.cms>0,where+": phải còn bám làn");
}
void Lost(const ControlResult &r, const std::string &where) {
    Check(r.debug.state=="MAT_LAN",where+": phải MAT_LAN");
}

// T55 §2: >= nhận cả biên bằng; hơi dưới ngưỡng hoặc chỉ một hàng clip đều không đủ.
void ClipCase(const std::string &scenario) {
    auto p=UnitParams();
    std::vector<double> right(3,p.right_clip_frac);
    bool trigger=true;
    if(scenario=="below") { right.assign(3,p.right_clip_frac-0.001); trigger=false; }
    if(scenario=="all_valid") { right[1]=0.8; trigger=false; }
    auto rows=Edges(p,right);
    if(scenario=="invalid_rows") { rows[1]={p.scan_rows[1],-1,-1,-1,false}; }
    const auto mask=Mask(rows,p,1000);
    LaneController c;
    auto r=Step(c,mask,1000,p); Geometry(r,rows); Tracking(r,"Bắt đầu đếm clip");
    for(int64_t now=1100;now<=1400;now+=100) Tracking(Step(c,mask,now,p),"Trước ngưỡng");
    Tracking(Step(c,mask,1499,p),"499 ms");
    r=Step(c,mask,1500,p);
    if(trigger) Lost(r,"Đúng 500 ms");
    else {
        Tracking(r,"Không thỏa mọi hàng");
        Tracking(Step(c,mask,2100,p),"Không được tích lũy khi thiếu điều kiện");
    }
}

// T55 §2: một khung R<clip xóa bộ đếm; 400 ms + khung ngắt + 400 ms không được kích hoạt.
void CounterReset() {
    auto p=UnitParams();
    const auto rows=Edges(p,{0.99,0.99,0.99});
    const auto clipped=Mask(rows,p,1000);
    const auto clearRows=Edges(p,{0.99,0.989,0.99});
    const auto clear=Mask(clearRows,p,1000);
    LaneController c;
    Geometry(Step(c,clipped,1000,p),rows);
    Tracking(Step(c,clipped,1400,p),"Đợt clip 400 ms đầu");
    auto r=Step(c,clear,1450,p); Geometry(r,clearRows); Tracking(r,"Khung ngắt");
    Tracking(Step(c,clipped,1500,p),"Đếm lại");
    Tracking(Step(c,clipped,1900,p),"Đợt clip 400 ms sau");
    Tracking(Step(c,clipped,1999,p),"499 ms sau khi đếm lại");
    Lost(Step(c,clipped,2000,p),"Đợt mới đủ 500 ms");
}

// Notes §1: bộ đếm theo now_ms; mask_ms khác now_ms nhưng vẫn trong tuổi cho phép.
void ClockAndReset() {
    auto p=UnitParams(); const auto rows=Edges(p,{0.99,0.99,0.99}); const auto mask=Mask(rows,p,1000);
    LaneController c;
    auto r=c.Step(mask,900,1000,p); Geometry(r,rows); Tracking(r,"Mask trễ 100 ms");
    Tracking(c.Step(mask,1499,1499,p),"now mới 499 ms dù mask đã 599 ms");
    Lost(c.Step(mask,1500,1500,p),"now đủ 500 ms");
    c.Reset();
    Tracking(Step(c,mask,1600,p),"Reset xóa mất làn");
    Tracking(Step(c,mask,2000,p),"Tích lũy 400 ms");
    c.Reset();
    Tracking(Step(c,mask,2001,p),"Reset xóa bộ đếm");
    Tracking(Step(c,mask,2401,p),"400 ms sau Reset không đủ");
    Lost(Step(c,mask,2501,p),"500 ms sau Reset");
}

// T55 §2/notes §5: đi cùng đường mất làn cũ; còn clip không phục hồi, làn tốt đủ regain_frames mới chạy.
void Recovery() {
    auto p=UnitParams(); p.regain_frames=3;
    auto old=p; old.right_lost_ms=0;
    const auto rows=Edges(p,{0.99,0.99,0.99}); const auto mask=Mask(rows,p,1000);
    const auto clear=Mask(Edges(p,{0.8,0.8,0.8}),p,1000);
    const cv::Mat empty(mask.rows,mask.cols,CV_8UC1,cv::Scalar(0));
    LaneController active, legacy;
    const std::vector<int64_t> times={1000,1100,1200,1300,1400,1499,1500,1550,1800,2298,2299,2300,2500};
    for(int64_t now:times) {
        const auto a=Step(active,mask,now,p);
        const auto b=Step(legacy,now<1500 ? mask : empty,now,old);
        if(now<1500) Tracking(a,"B1 chưa đủ hạn");
        else {
            Lost(a,"Clip không được phục hồi");
            Check(a.debug.state==b.debug.state && a.debug.reason==b.debug.reason,"B1 đổi trạng thái/lý do đường mất làn cũ");
            Near(a.cmd.cms,b.cmd.cms,"Tốc độ đường mất làn cũ");
            Near(a.cmd.deg,b.cmd.deg,"Góc đường mất làn cũ");
        }
        if(now==1500) Near(a.cmd.cms,p.lost_cms,"B1 hạ xuống lost_cms");
        if(now==1800) Near(a.cmd.cms,0,"Qua lost_hold_ms phải dừng tốc độ");
        if(now==2300) { Near(a.cmd.cms,0,"Qua lost_stop_ms dừng tốc độ"); Near(a.cmd.deg,0,"Qua lost_stop_ms trả góc 0"); }
    }
    for(int frame=1;frame<=p.regain_frames;++frame) {
        const int64_t now=2500+frame*50;
        const auto r=Step(active,clear,now,p);
        if(frame<p.regain_frames) {
            Lost(r,"Chưa đủ khung phục hồi"); Near(r.cmd.cms,0,"Chưa đủ khung phải đứng");
            Lost(active.Step(clear,now,now+1,p),"Cùng mask không được đếm hai lần");
        } else Tracking(r,"Đủ regain_frames");
    }
}

// T55 §2: không hàng hợp lệ dùng nguyên đường mất làn cũ; 0 tắt B1 cả khi clip kéo dài.
void DisabledAndEmpty() {
    auto p=UnitParams(); p.right_lost_ms=0;
    const auto rows=Edges(p,{1,1,1}); const auto mask=Mask(rows,p,1000);
    LaneController c;
    for(int64_t now=1000;now<=3500;now+=100) Tracking(Step(c,mask,now,p),"B1=0 với R=1");
    const cv::Mat empty(mask.rows,mask.cols,CV_8UC1,cv::Scalar(0));
    LaneController a,b;
    auto enabled=p; enabled.right_lost_ms=500;
    for(int64_t now:{1000,1100,1600,1900}) {
        const auto x=Step(a,empty,now,enabled), y=Step(b,empty,now,p);
        Lost(x,"Không có hàng hợp lệ");
        Check(x.debug.valid_rows==0 && x.debug.state==y.debug.state && x.debug.reason==y.debug.reason,"B1 thay đường không có hàng hợp lệ");
        Near(x.cmd.cms,y.cmd.cms,"Không hàng: tốc độ cũ"); Near(x.cmd.deg,y.cmd.deg,"Không hàng: góc cũ");
    }
}

// T55 §3/notes §2–3: suy h qua debug.p, không thêm trường vào ControlDebug.
void Heading() {
    auto p=UnitParams(); p.right_lost_ms=0; p.kh=12; p.lane_width_frac={0.1,0.1,0.1};
    auto run=[&](const std::vector<double> &centers, int missing, double gate) {
        auto q=p; q.kh_gate_frac=gate;
        q.scan_rows.resize(centers.size()); q.row_weights.resize(centers.size()); q.lane_width_frac.resize(centers.size());
        std::vector<double> right; for(double center:centers) right.push_back(center+0.05);
        auto rows=Edges(q,right);
        if(missing>=0) rows[static_cast<size_t>(missing)]={q.scan_rows[static_cast<size_t>(missing)],-1,-1,-1,false};
        q.Validate();
        const auto mask=Mask(rows,q,1000);
        LaneController c,baseline;
        const auto r=Step(c,mask,1000,q);
        Geometry(r,rows); Tracking(r,"Tiền điều kiện B2");
        double h=0;
        const size_t n=r.debug.rows.size();
        if(n>=3 && missing<0) {
            const double near=r.debug.rows.front().center/1000;
            const double middle=r.debug.rows[n/2].center/1000;
            const double far=r.debug.rows.back().center/1000;
            if(std::abs(far-middle)<=gate) h=std::clamp((far-near)/0.5,-1.0,1.0);
        }
        Near((r.debug.p-q.kp*r.debug.e)/q.kh,h,"Công thức/dấu/cổng h");
        q.kh=0;
        const auto b=Step(baseline,mask,1000,q);
        if(h==0) Near(r.cmd.deg,b.cmd.deg,"h=0 không đổi góc");
        else Check((r.cmd.deg-b.cmd.deg)*h<0,"Số hạng nhìn trước sai dấu lái");
        return h;
    };
    Near(run({0.5,0.5,0.5},-1,0.35),0,"Làn thẳng");
    Check(run({0.5,0.425,0.35},-1,0.35)<0,"Làn xa lệch trái phải h âm");
    Check(run({0.5,0.575,0.65},-1,0.35)>0,"Làn xa lệch phải phải h dương");
    for(int missing:{0,1,2}) Near(run({0.5,0.575,0.65},missing,0.35),0,"Thiếu gần/giữa/xa");
    Near(run({0.5},-1,0.35),0,"n=1"); Near(run({0.5,0.65},-1,0.35),0,"n=2");
    Near(run({0.5,0.5,0.75},-1,0.25),0.5,"Đúng biên cổng được nhận");
    Near(run({0.5,0.5,0.751},-1,0.25),0,"Vượt cổng một pixel phải chặn");
    Near(run({0.1,0.5,0.9},-1,1),1,"Kẹp h dương");
    Near(run({0.9,0.5,0.1},-1,1),-1,"Kẹp h âm");
}

template<class F> void Reject(F action, const std::string &key) {
    bool rejected=false;
    try { action(); }
    catch(const std::exception &error) {
        rejected=true;
        Check(std::string(error.what()).find(key)!=std::string::npos,"Lỗi phải nêu khóa: "+key);
    }
    Check(rejected,"Phải từ chối "+key);
}

// Notes §4: mặc định/miền 4 khóa, int YAML; độc lập với test_control_params.cpp được bảo vệ.
void Params(const std::string &path) {
    struct Cleanup { std::string path; ~Cleanup() { std::remove(path.c_str()); } } cleanup{path};
    auto load=[&](const std::string &body, bool header) {
        { std::ofstream out(path); Check(bool(out),"Không tạo được YAML tạm");
          if(header) out << "%YAML:1.0\n---\n";
          out << body; out.close(); Check(bool(out),"Không ghi được YAML tạm"); }
        std::ostringstream report;
        return ControlParams::Load(path,report);
    };
    const ControlParams defaults;
    defaults.Validate();
    Near(defaults.right_clip_frac,0.99,"Mặc định clip"); Check(defaults.right_lost_ms==500,"Mặc định hạn clip");
    Near(defaults.kh,0,"Mặc định tắt B2"); Near(defaults.kh_gate_frac,0.35,"Mặc định cổng B2");
    for(bool header:{false,true}) {
        const auto d=load("{}\n",header);
        Near(d.right_clip_frac,0.99,"Thiếu khóa dùng mặc định clip"); Check(d.right_lost_ms==500,"Thiếu khóa dùng mặc định ms");
        Near(d.kh,0,"Thiếu khóa dùng mặc định kh"); Near(d.kh_gate_frac,0.35,"Thiếu khóa dùng mặc định cổng");
        for(int value:{0,200,500,2000}) {
            const auto q=load("right_lost_ms: "+std::to_string(value)+"\n",header);
            Check(q.right_lost_ms==value,"Nạp sai hạn clip hợp lệ"); q.Validate();
        }
        for(const auto &value:{"199","2001","-1","500.5"})
            Reject([&] { load(std::string("right_lost_ms: ")+value+"\n",header); },"right_lost_ms");
        for(double value:{0.90,1.00}) Near(load("right_clip_frac: "+std::to_string(value)+"\n",header).right_clip_frac,value,"Biên clip");
        for(double value:{0.0,30.0}) Near(load("kh: "+std::to_string(value)+"\n",header).kh,value,"Biên kh");
        for(double value:{0.1,1.0}) Near(load("kh_gate_frac: "+std::to_string(value)+"\n",header).kh_gate_frac,value,"Biên cổng");
        for(const auto &item:std::vector<std::pair<std::string,std::string>>{
            {"right_clip_frac","0.899"},{"right_clip_frac","1.001"},{"kh","-0.1"},{"kh","30.1"},
            {"kh_gate_frac","0.099"},{"kh_gate_frac","1.001"},{"right_lost_ms","\"500\""}})
            Reject([&] { load(item.first+": "+item.second+"\n",header); },item.first);
    }
    for(int value:{-1,199,2001}) { auto q=defaults; q.right_lost_ms=value; Reject([&] { q.Validate(); },"right_lost_ms"); }
    for(int value:{0,200,2000}) { auto q=defaults; q.right_lost_ms=value; q.Validate(); }
}
}

int main(int argc, char **argv) {
    try {
        std::locale::global(std::locale::classic());
        Check(argc>=2,"Cần tên ca hoặc --check-mask / --write-golden <file>");
        const std::string scenario=argv[1];
        if(scenario=="--write-golden") { Check(argc==3,"Cần đường dẫn golden"); WriteGolden(Fixture(),argv[2]); }
        else if(scenario=="params") { Check(argc==3,"Cần đường dẫn YAML tạm riêng"); Params(argv[2]); }
        else {
            Check(argc==2,"Thừa tham số");
            if(scenario=="--check-mask") CheckMask(Fixture());
            else if(scenario=="disabled") Disabled(Fixture());
            else if(scenario=="replay_b1") ReplayB1(Fixture());
            else if(scenario=="replay_b2") ReplayB2(Fixture());
            else if(scenario=="heading") Heading();
            else if(scenario=="threshold" || scenario=="below" || scenario=="all_valid" || scenario=="invalid_rows") ClipCase(scenario);
            else if(scenario=="counter_reset") CounterReset();
            else if(scenario=="clock_reset") ClockAndReset();
            else if(scenario=="recovery") Recovery();
            else if(scenario=="disabled_empty") DisabledAndEmpty();
            else Check(false,"Ca không biết: "+scenario);
        }
        std::cout << "ĐẠT: " << scenario << '\n';
        return 0;
    } catch(const std::exception &error) {
        std::cerr << "KHÔNG ĐẠT: " << error.what() << '\n';
        return 1;
    }
}
