#include "control/LaneController.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace control {
namespace {
double Clamp(double value, double low, double high) { return std::max(low,std::min(value,high)); }
RowDebug Scan(const cv::Mat &mask, int y, double width, const ControlParams &params, LaneRef ref) {
    RowDebug row; row.y=y;
    const auto *pixels=mask.ptr<unsigned char>(y);
    int begin=0, end=mask.cols;
    bool surface=false;
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
}
void LaneController::Reset() { *this=LaneController{}; }
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
        regain_=0; derivative_ready_=false;
        return finish(0,0,"MAT_NAN_CU","MAT_NAN_CU");
    }
    last_mask_ms_=mask_ms;
    if(!mask.empty()) {
        if(mask.dims!=2 || mask.type()!=CV_8UC1 || mask.cols<2)
            throw std::runtime_error("Mặt nạ phải là CV_8UC1 H×W, W >= 2");
        double maximum=0; cv::minMaxLoc(mask,nullptr,&maximum);
        if(maximum>2) throw std::runtime_error("Mặt nạ có chỉ số lớp ngoài 0/1/2");
        double weighted=0, total=0;
        for(size_t i=0;i<params.scan_rows.size();++i) {
            const int y=std::min(mask.rows-1,int(params.scan_rows[i]*mask.rows));
            auto row=Scan(mask,y,params.lane_width_frac[i]*mask.cols,params,ref);
            if(row.valid) {
                ++result.debug.valid_rows;
                weighted+=params.row_weights[i]*(row.center-(mask.cols/2.0+params.center_offset_frac*mask.cols))/(mask.cols/2.0);
                total+=params.row_weights[i];
            }
            result.debug.rows.push_back(row);
        }
        if(total>0) result.debug.e=Clamp(weighted/total,-1,1);
        // Hàng trọng số 0 không được tạo lệnh khi các hàng có trọng số đều hỏng.
        if(total<=0) result.debug.valid_rows=0;
    }
    bool valid=result.debug.valid_rows>=params.min_valid_rows;
    if(params.right_lost_ms>0) {
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
    if(!valid) { lost_=true; regain_=0; regain_mask_ms_=-1; derivative_ready_=false; }
    else if(lost_) {
        if(mask_ms>regain_mask_ms_) { ++regain_; regain_mask_ms_=mask_ms; }
        if(regain_>=params.regain_frames) { lost_=false; derivative_ready_=false; right_lost_=false; }
    }
    if(lost_) {
        const int64_t age=last_valid_ms_<0 ? params.lost_stop_ms : now_ms-last_valid_ms_;
        if(age>=params.lost_stop_ms) return finish(0,0,"MAT_LAN",valid ? "CHO_HOI_LAN" : "MAT_LAN");
        if(age>=params.lost_hold_ms) return finish(0,angle_,"GIU_GOC",valid ? "CHO_HOI_LAN" : "MAT_LAN_DUNG");
        return finish(params.lost_cms,angle_,"GIU_GOC",valid ? "CHO_HOI_LAN" : "MAT_LAN_GIU");
    }
    if(ref!=reference_) derivative_ready_=false;
    result.debug.p=params.kp*result.debug.e;
    if(params.kh>0 && result.debug.rows.size()>=3) {
        const auto &near=result.debug.rows.front();
        const auto &middle=result.debug.rows[result.debug.rows.size()/2];
        const auto &far=result.debug.rows.back();
        if(near.valid && middle.valid && far.valid) {
            const double c_near=near.center/mask.cols, c_middle=middle.center/mask.cols, c_far=far.center/mask.cols;
            if(std::abs(c_far-c_middle)<=params.kh_gate_frac)
                result.debug.p+=params.kh*Clamp((c_far-c_near)/0.5,-1,1);
        }
    }
    result.debug.d=derivative_ready_ && dt>0 ? params.kd*(result.debug.e-error_)/dt : 0;
    const double desired=Clamp(-(result.debug.p+result.debug.d),-params.steer_max_deg,params.steer_max_deg);
    const double step=params.steer_rate_dps*dt;
    const double angle=Clamp(Clamp(desired,angle_-step,angle_+step),-params.steer_max_deg,params.steer_max_deg);
    error_=result.debug.e; derivative_ready_=true; reference_=ref; last_valid_ms_=mask_ms;
    return finish(std::max(params.min_cms,params.cruise_cms-params.slow_per_deg*std::abs(angle)),
                  angle,"BAM_LAN","BAM_LAN");
}
}
