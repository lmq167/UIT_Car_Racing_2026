#include "control/LaneController.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace control {
namespace {
double Clamp(double value, double low, double high) { return std::max(low,std::min(value,high)); }
}
RowDebug ScanLines(const cv::Mat &mask, int y, int &begin, int &end, bool &surface) {
    RowDebug row; row.y=y;
    const auto *pixels=mask.ptr<unsigned char>(y);
    begin=0; end=mask.cols; surface=false;
    for(int x=0;x<mask.cols;++x) if(pixels[x]==1) surface=true;
    // Chọn mảng đường gần trục xe, bỏ mảng rời bên ngoài đường.
    if(surface) {
        double best=std::numeric_limits<double>::infinity();
        for(int x=0;x<mask.cols;) {
            if(!pixels[x]) { ++x; continue; }
            const int a=x;
            bool road=false;
            while(x<mask.cols && pixels[x]) { road=road || pixels[x]==1; ++x; }
            const double distance=std::max(0.0,std::max(a-mask.cols/2.0,mask.cols/2.0-x));
            if(road && distance<best) { best=distance; begin=a; end=x; }
        }
    }
    for(int x=begin;x<end;) {
        if(pixels[x]!=2) { ++x; continue; }
        const int a=x;
        while(x<end && pixels[x]==2) ++x;
        row.lines.push_back((a+x-1)/2.0);
    }
    return row;
}
namespace {
RowDebug Scan(const cv::Mat &mask, int y, double width, const ControlParams &params, LaneRef ref) {
    int begin, end; bool surface;
    auto row=ScanLines(mask,y,begin,end,surface);
    if(row.lines.empty() && !surface) return row;
    std::vector<double> candidates=row.lines;
    double boundary=row.lines.empty() ? (ref==LaneRef::RIGHT ? end : begin) :
        (ref==LaneRef::RIGHT ? row.lines.back() : row.lines.front());
    // Còn đủ một làn phía ngoài vạch: vạch đó là phân làn, dùng mép đường.
    if(surface && !row.lines.empty()) {
        const double outer=ref==LaneRef::RIGHT ? end : begin;
        if(std::abs(outer-boundary)>=width*params.lane_width_min_ratio) boundary=outer;
    }
    if(surface) { candidates.push_back(begin); candidates.push_back(end); }
    if(ref==LaneRef::RIGHT) { row.right=boundary; row.center=boundary-width/2; }
    else { row.left=boundary; row.center=boundary+width/2; }
    double best=std::numeric_limits<double>::infinity();
    for(double other:candidates) {
        const double gap=ref==LaneRef::RIGHT ? boundary-other : other-boundary;
        if(gap<=0 || gap<width*params.lane_width_min_ratio || gap>width*params.lane_width_max_ratio) continue;
        if(std::abs(gap-width)<best) {
            best=std::abs(gap-width);
            if(ref==LaneRef::RIGHT) row.left=other; else row.right=other;
            row.center=(boundary+other)/2;
        }
    }
    row.valid=row.center>=0 && row.center<=mask.cols;
    return row;
}
struct Pair { double left=0, right=0; bool valid=false, left_line=false, right_line=false; };
RowDebug ScanPair(const cv::Mat &mask, int y, double width, const ControlParams &params,
                  LaneRef ref, bool near, const Pair &anchor, Pair &selected) {
    int begin, end; bool surface;
    auto row=ScanLines(mask,y,begin,end,surface);
    if(!near && !anchor.valid) return row;
    const double car=(0.5+params.center_offset_frac)*mask.cols;
    struct Edge { double left, right; bool line; };
    std::vector<Edge> edges;
    if(surface) edges.push_back({double(begin),double(begin),false});
    for(size_t i=0;i<row.lines.size();) {
        double left=row.lines[i], right=left;
        size_t next=i+1;
        // So với vạch thô liền trước để giữ trọn cụm nối theo chuỗi.
        while(next<row.lines.size() &&
              row.lines[next]-row.lines[next-1]<params.pair_merge_frac*mask.cols) {
            const double x=row.lines[next++];
            if(near) {
                if(std::abs(x-car)>std::abs(left-car)) left=right=x;
            } else {
                // Cùng cụm chọn thành viên riêng theo vai trò mép của từng cặp.
                if(std::abs(x-anchor.left)<std::abs(left-anchor.left)) left=x;
                if(std::abs(x-anchor.right)<std::abs(right-anchor.right)) right=x;
            }
        }
        edges.push_back({left,right,true});
        i=next;
    }
    if(surface) edges.push_back({double(end),double(end),false});
    int close=-1;
    double distance=0.03*mask.cols;
    if(near) for(size_t i=0;i<edges.size();++i) {
        const double delta=std::abs(edges[i].left-car);
        if(edges[i].line && delta<distance) { close=int(i); distance=delta; }
    }
    bool best_match=false;
    double best=std::numeric_limits<double>::infinity();
    for(size_t i=1;i<edges.size();++i) {
        const double left=edges[i-1].left, right=edges[i].right, gap=right-left;
        if(edges[i-1].line && edges[i].line &&
           (gap<width*params.lane_width_min_ratio || gap>width*params.lane_width_max_ratio)) continue;
        bool match=false;
        double score=0;
        if(near) {
            if(close<0) {
                if(!(left<car && car<=right)) continue;
            } else {
                if(int(i)!=close && int(i)-1!=close) continue;
                const double dl=std::abs(left-anchor.left), dr=std::abs(right-anchor.right);
                match=anchor.valid && dl<=0.15*mask.cols && dr<=0.15*mask.cols;
                score=match ? dl+dr : std::abs((left+right)/2-car);
            }
        } else {
            const double dl=std::abs(left-anchor.left), dr=std::abs(right-anchor.right);
            if(dl>params.pair_max_shift_frac*mask.cols || dr>params.pair_max_shift_frac*mask.cols) continue;
            score=dl+dr;
        }
        // Duyệt từ trái sang phải, giữ cặp trái khi điểm bằng nhau.
        if(!selected.valid || (match && !best_match) || (match==best_match && score<best)) {
            selected={left,right,true,edges[i-1].line,edges[i].line}; best=score; best_match=match;
        }
    }
    if(!selected.valid) return row;
    const double boundary=ref==LaneRef::RIGHT ? selected.right : selected.left;
    if(ref==LaneRef::RIGHT) { row.right=boundary; row.center=boundary-width/2; }
    else { row.left=boundary; row.center=boundary+width/2; }
    const double gap=selected.right-selected.left;
    if(gap>=width*params.lane_width_min_ratio && gap<=width*params.lane_width_max_ratio) {
        row.left=selected.left; row.right=selected.right; row.center=(selected.left+selected.right)/2;
    }
    row.valid=row.center>=0 && row.center<=mask.cols;
    return row;
}
}
void LaneController::Reset() {
    // Chuyển làn đang dở thì giữ vạch phân làn đang theo dõi (Planner đặt lại bằng SetSwerve).
    const int dir=swerve_dir_; const auto x=d_x_; const auto v=d_valid_; const auto m=d_miss_;
    *this=LaneController{}; swerve_dir_=dir; d_x_=x; d_valid_=v; d_miss_=m;
}
void LaneController::SetSwerve(int dir) {
    if(dir==swerve_dir_) return;
    if(dir==0 || swerve_dir_==0) { d_x_.clear(); d_valid_.clear(); d_miss_.clear(); }
    swerve_dir_=dir;
}
// Hàng quét khi chuyển làn: theo vạch phân làn D (khởi tạo từ cặp bao quanh xe, rồi bám vạch gần nhất ở khung sau).
// Mục tiêu = D - nửa làn (sang trái) hoặc D + nửa làn (về phải); mất D quá lâu thì hàng hỏng.
RowDebug LaneController::SwerveRow(const cv::Mat &mask, int y, size_t i, double left, double right, bool edge_is_line,
                                   const ControlParams &params) {
    constexpr int kMissSteps=40;      // ~2 s ở chu kỳ 50 ms: một nhịp vạch đứt.
    const double tolerance=0.10*mask.cols;
    if(d_x_.size()!=params.scan_rows.size()) {
        d_x_.assign(params.scan_rows.size(),0); d_valid_.assign(params.scan_rows.size(),0); d_miss_.assign(params.scan_rows.size(),0);
    }
    int begin, end; bool surface;
    auto row=ScanLines(mask,y,begin,end,surface);
    if(!d_valid_[i] && edge_is_line) { d_x_[i]=swerve_dir_>0 ? left : right; d_valid_[i]=1; d_miss_[i]=0; }
    if(d_valid_[i]) {
        double best=tolerance; bool found=false; double x_new=0;
        for(double x:row.lines) if(std::abs(x-d_x_[i])<=best) { best=std::abs(x-d_x_[i]); x_new=x; found=true; }
        if(found) { d_x_[i]=x_new; d_miss_[i]=0; }
        else if(++d_miss_[i]>kMissSteps) d_valid_[i]=0;
    }
    const double width=params.lane_width_frac[i]*mask.cols;
    row.left=row.right=row.center=-1; row.valid=false;
    if(!d_valid_[i]) return row;
    if(swerve_dir_>0) { row.right=d_x_[i]; row.left=row.right-width; }
    else { row.left=d_x_[i]; row.right=row.left+width; }
    row.center=(row.left+row.right)/2; row.valid=true;
    return row;
}
ControlResult LaneController::Step(const cv::Mat &mask, int64_t mask_ms, int64_t now_ms,
                                   const ControlParams &params, LaneRef ref) {
    params.Validate();
    if(now_ms<0 || (last_now_ms_>=0 && now_ms<last_now_ms_))
        throw std::runtime_error("Thời gian điều khiển now_ms phải tăng đơn điệu");
    const double dt=(last_now_ms_<0 ? params.control_period_ms : now_ms-last_now_ms_)/1000.0;
    last_now_ms_=now_ms;
    ControlResult result;
    auto finish=[&](double cms, double deg, const char *state, const char *reason) {
        result.cmd={cms,deg};
        result.debug.angle_deg=deg; result.debug.speed_cms=cms;
        result.debug.state=state; result.debug.reason=reason;
        angle_=deg;
        return result;
    };
    if(mask_ms<0 || mask_ms>now_ms || now_ms-mask_ms>params.mask_max_age_ms ||
       (last_mask_ms_>=0 && mask_ms<last_mask_ms_)) {
        regain_=0; derivative_ready_=false; right_clip_since_ms_=-1;
        filtered_derivative_=0; filter_ready_=false;
        have_near_=false;
        return finish(0,0,"MAT_NAN_CU","MAT_NAN_CU");
    }
    last_mask_ms_=mask_ms;
    if(!mask.empty()) {
        if(mask.dims!=2 || mask.type()!=CV_8UC1 || mask.cols<2)
            throw std::runtime_error("Mặt nạ phải là CV_8UC1 H×W, W >= 2");
        double maximum=0; cv::minMaxLoc(mask,nullptr,&maximum);
        if(maximum>2) throw std::runtime_error("Mặt nạ có chỉ số lớp ngoài 0/1/2");
        double weighted=0, total=0;
        Pair anchor;
        std::vector<Pair> pairs;
        std::vector<double> width_ratios;
        for(size_t i=0;i<params.scan_rows.size();++i) {
            const int y=std::min(mask.rows-1,int(params.scan_rows[i]*mask.rows));
            RowDebug row;
            if(params.pair_mode==0) row=Scan(mask,y,params.lane_width_frac[i]*mask.cols,params,ref);
            else {
                Pair selected;
                const Pair previous{near_left_,near_right_,have_near_};
                row=ScanPair(mask,y,params.lane_width_frac[i]*mask.cols,params,ref,
                             i==0,i==0 ? previous : anchor,selected);
                if(selected.valid && row.valid) anchor=selected;
                if(i==0) {
                    have_near_=selected.valid && row.valid;
                    if(have_near_) { near_left_=selected.left; near_right_=selected.right; }
                }
                // Chuỗi cặp chưa có neo (hàng gần hỏng) thì dùng luật pair_mode 0 thay vì bỏ cả ba hàng;
                // hàng xa hỏng khi neo còn thì vẫn bỏ, vì Scan dễ gán vạch sai bên ở hàng xa.
                if(!selected.valid && !anchor.valid) row=Scan(mask,y,params.lane_width_frac[i]*mask.cols,params,ref);
                pairs.push_back(selected);
                const double width=params.lane_width_frac[i]*mask.cols;
                const double gap=selected.right-selected.left;
                width_ratios.push_back(0);
                if(row.valid && selected.left_line && selected.right_line &&
                   gap>=params.lane_width_min_ratio*width && gap<=params.lane_width_max_ratio*width) {
                    width_ratios.back()=gap/width;
                }
                if(swerve_dir_!=0) row=SwerveRow(mask,y,i,selected.left,selected.right,
                                       selected.valid && (swerve_dir_>0 ? selected.left_line : selected.right_line),params);
            }
            if(swerve_dir_==0 && row.valid && row.left<0 && row.right/mask.cols>=params.right_clip_frac)
                row.center=row.right-params.lane_width_frac[i]*mask.cols/4;
            if(row.valid) {
                ++result.debug.valid_rows;
                weighted+=params.row_weights[i]*(row.center-(mask.cols/2.0+params.center_offset_frac*mask.cols))/(mask.cols/2.0);
                total+=params.row_weights[i];
            }
            result.debug.rows.push_back(row);
        }
        if(params.pair_mode==1) {
            weighted=0; total=0;
            for(size_t i=0;i<result.debug.rows.size();++i) {
                auto &row=result.debug.rows[i];
                if(!row.valid) continue;
                if(swerve_dir_!=0) {  // chuyển làn: hàng đã tính theo vạch phân làn, không thay mép giả
                    weighted+=params.row_weights[i]*(row.center-(mask.cols/2.0+params.center_offset_frac*mask.cols))/(mask.cols/2.0);
                    total+=params.row_weights[i];
                    continue;
                }
                const auto &pair=pairs[i];
                const double nominal=params.lane_width_frac[i]*mask.cols, pair_gap=pair.right-pair.left;
                // Mép giả chỉ bị thay khi cặp trông như một làn đủ bề rộng; ngoài miền đó giữ luật B3 (mép thảm ẩn).
                if(pair.left_line!=pair.right_line && pair_gap>=params.lane_width_min_ratio*nominal &&
                   pair_gap<=params.lane_width_max_ratio*nominal) {
                    double k=0;
                    size_t nearest=width_ratios.size();
                    for(size_t j=0;j<width_ratios.size();++j) {
                        const size_t distance=i>j ? i-j : j-i;
                        if(width_ratios[j]>0 && distance<nearest) {
                            k=width_ratios[j]; nearest=distance;
                        }
                    }
                    if(k>0) {
                        const double width=k*params.lane_width_frac[i]*mask.cols;
                        const double replacement=pair.left_line ? pair.left+width : pair.right-width;
                        if(replacement>=0 && replacement<=mask.cols) {
                            if(pair.left_line) { row.left=pair.left; row.right=replacement; }
                            else { row.right=pair.right; row.left=replacement; }
                            row.center=(row.left+row.right)/2;
                            if(i==0 && have_near_) { near_left_=row.left; near_right_=row.right; }
                        }
                    }
                }
                weighted+=params.row_weights[i]*(row.center-(mask.cols/2.0+params.center_offset_frac*mask.cols))/(mask.cols/2.0);
                total+=params.row_weights[i];
            }
        }
        if(total>0) result.debug.e=Clamp(weighted/total,-1,1);
        // Hàng trọng số 0 không được tạo lệnh khi các hàng có trọng số đều hỏng.
        if(total<=0) result.debug.valid_rows=0;
    } else if(params.pair_mode==1) have_near_=false;
    bool valid=result.debug.valid_rows>=params.min_valid_rows;
    if(params.right_lost_ms>0 && swerve_dir_==0) {
        bool any=false, all=true;
        for(const auto &row:result.debug.rows) if(row.valid) {
            any=true;
            all=all && row.right/mask.cols>=params.right_clip_frac;
        }
        if(any && all) {
            if(right_clip_since_ms_<0) right_clip_since_ms_=now_ms;
            if(right_lost_ || now_ms-right_clip_since_ms_>=params.right_lost_ms) {
                valid=false;
                right_lost_=true;
            }
        } else right_clip_since_ms_=-1;
    } else { right_clip_since_ms_=-1; right_lost_=false; }
    if(!valid) { lost_=true; regain_=0; regain_mask_ms_=-1; derivative_ready_=false; filtered_derivative_=0; filter_ready_=false; }
    else if(lost_) {
        if(mask_ms>regain_mask_ms_) { ++regain_; regain_mask_ms_=mask_ms; }
        if(regain_>=params.regain_frames) { lost_=false; derivative_ready_=false; filtered_derivative_=0; filter_ready_=false; right_lost_=false; }
    }
    if(lost_) {
        const int64_t age=last_valid_ms_<0 ? params.lost_stop_ms : now_ms-last_valid_ms_;
        if(age>=params.lost_stop_ms) return finish(0,0,"MAT_LAN",valid ? "CHO_HOI_LAN" : "MAT_LAN");
        if(age>=params.lost_hold_ms) return finish(0,angle_,"GIU_GOC",valid ? "CHO_HOI_LAN" : "MAT_LAN_DUNG");
        return finish(params.lost_cms,angle_,"GIU_GOC",valid ? "CHO_HOI_LAN" : "MAT_LAN_GIU");
    }
    if(ref!=reference_) { derivative_ready_=false; filtered_derivative_=0; filter_ready_=false; }
    result.debug.p=params.kp*result.debug.e;
    result.debug.d=derivative_ready_ && dt>0 ? params.kd*(result.debug.e-error_)/dt : 0;
    if(params.kd_filter_ms>0 && derivative_ready_ && dt>0) {
        const double raw=(result.debug.e-error_)/dt;
        const double a=dt/(params.kd_filter_ms/1000.0+dt);
        if(!filter_ready_) { filtered_derivative_=raw; filter_ready_=true; }
        else filtered_derivative_+=a*(raw-filtered_derivative_);
        result.debug.d=params.kd*filtered_derivative_;
    }
    const double desired=Clamp(-(result.debug.p+result.debug.d),-params.steer_max_deg,params.steer_max_deg);
    const double step=params.steer_rate_dps*dt;
    const double angle=Clamp(Clamp(desired,angle_-step,angle_+step),-params.steer_max_deg,params.steer_max_deg);
    error_=result.debug.e; derivative_ready_=true; reference_=ref; last_valid_ms_=mask_ms;
    return finish(std::max(params.min_cms,params.cruise_cms-params.slow_per_deg*std::abs(angle)),
                  angle,"BAM_LAN","BAM_LAN");
}
}
