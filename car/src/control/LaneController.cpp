#include "control/LaneController.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace control {
namespace {
constexpr double kNoiseFrac=0.15;    // TẠM, CHƯA ĐO: đoạn hẹp hơn 0,15·w(y) là nhiễu
constexpr double kConfPenalty=0.3;   // TẠM, CHƯA ĐO: mức trừ độ tin cậy mỗi dấu hiệu xấu
constexpr double kMergeFactor=1.5;   // TẠM, CHƯA ĐO: gộp đoạn khi bề rộng gộp ≤ two_lane_ratio·w·1,5
constexpr double kMinWidthRatio=0.6; // TẠM, CHƯA ĐO: bề rộng dưới 0,6·w(y) là bất thường
constexpr double kRightLaneFrac=0.25, kLeftLaneFrac=0.75;  // tâm làn phải/trái của đường hai làn
const char *const kThat="THAT";
const char *const kEdge="MEP_ANH";
const char *const kJump="NHAY";
double Clamp(double value, double low, double high) { return std::max(low,std::min(value,high)); }
struct Seg { int l, r; };  // pixel đường đầu/cuối, tính cả hai đầu
void CheckMask(const cv::Mat &mask) {
    if(mask.dims!=2 || mask.type()!=CV_8UC1 || mask.cols<2)
        throw std::runtime_error("Mặt nạ phải là CV_8UC1 H×W, W >= 2");
    double maximum=0; cv::minMaxLoc(mask,nullptr,&maximum);
    if(maximum>2) throw std::runtime_error("Mặt nạ có chỉ số lớp ngoài 0/1/2");
}
}
const char *TrajModeName(TrajMode mode) {
    switch(mode) {
        case TrajMode::AVOID_LEFT: return "NE_TRAI";
        case TrajMode::RETURN_RIGHT: return "VE_PHAI";
        case TrajMode::ROUNDABOUT: return "VONG_XUYEN";
        default: return "BAM_LAN";
    }
}
Corridor ExtractCorridor(const cv::Mat &mask, const ControlParams &params, TrajMode mode) {
    CheckMask(mask);
    const int W=mask.cols, H=mask.rows, n=params.corridor_rows;
    const double axis=W/2.0+params.center_offset_frac*W;
    const double y_near=std::lround(params.corridor_bottom_frac*H), y_far=std::lround(params.corridor_top_frac*H);
    Corridor corridor;
    corridor.rows.reserve(n);
    int lower=-1;          // chỉ số hàng hợp lệ gần nhất phía dưới
    Seg lower_raw{0,0};    // đoạn thô của hàng đó (trước khi thay biên)
    bool stopped=false;
    for(int i=0;i<n;++i) {
        const double frac=params.corridor_bottom_frac+(params.corridor_top_frac-params.corridor_bottom_frac)*i/std::max(1,n-1);
        RowDebug row;
        row.y=int(Clamp(std::lround(frac*H),0,H-1));
        const double t=y_near>y_far ? Clamp((y_near-row.y)/(y_near-y_far),0,1) : 0;
        const double w=W*(params.lane1_width_frac[0]+(params.lane1_width_frac[1]-params.lane1_width_frac[0])*t);
        if(stopped) { corridor.rows.push_back(row); continue; }
        std::vector<Seg> segs;
        const auto *pixels=mask.ptr<unsigned char>(row.y);
        for(int x=0;x<W;) {
            if(!pixels[x]) { ++x; continue; }
            const int a=x;
            while(x<W && pixels[x]) ++x;
            if(x-a>=kNoiseFrac*w) segs.push_back({a,x-1});
        }
        std::vector<Seg> pick;
        if(lower<0) {
            // Hàng gần (hoặc chưa có hàng hợp lệ nào bên dưới): đoạn chứa trục, không có thì đoạn gần trục nhất.
            double best=w;
            for(const auto &s:segs) {
                const double distance=std::max(0.0,std::max(s.l-axis,axis-s.r));
                if(distance<=best) { best=distance; pick={s}; if(distance==0) break; }
            }
            // Khe chứa trục ở hàng gần: giữ hai đoạn đã qua lọc nhiễu để truy vết vật cản.
            if(mode!=TrajMode::ROUNDABOUT) for(size_t k=1;k<segs.size();++k) {
                if(segs[k-1].r<axis && segs[k].l>axis &&
                   segs[k].r-segs[k-1].l<=params.two_lane_ratio*w*kMergeFactor) {
                    pick={segs[k-1],segs[k]};
                    break;
                }
            }
        } else {
            std::vector<Seg> overlap;
            for(const auto &s:segs) if(s.r>=lower_raw.l && s.l<=lower_raw.r) overlap.push_back(s);
            if(overlap.empty()) { stopped=true; corridor.rows.push_back(row); continue; }
            if(overlap.size()>=2 && mode!=TrajMode::ROUNDABOUT &&
               overlap.back().r-overlap.front().l<=params.two_lane_ratio*w*kMergeFactor) pick=overlap;
            else if(mode==TrajMode::ROUNDABOUT) pick={overlap.back()};
            else {
                int best=-1;
                for(const auto &s:overlap) {
                    const int amount=std::min(s.r,lower_raw.r)-std::max(s.l,lower_raw.l);
                    if(amount>best) { best=amount; pick={s}; }
                }
            }
        }
        if(pick.empty()) { corridor.rows.push_back(row); continue; }
        const Seg raw{pick.front().l,pick.back().r};
        if(pick.size()>=2) {
            row.hole=true;
            int gap=-1;
            for(size_t k=1;k<pick.size();++k) if(pick[k].l-pick[k-1].r>gap) {
                gap=pick[k].l-pick[k-1].r; row.hole_l=pick[k-1].r+1; row.hole_r=pick[k].l-1;
            }
        }
        const RowDebug *below=lower>=0 ? &corridor.rows[lower] : nullptr;
        const double jump=params.max_jump_frac*W;
        row.flag_left=raw.l<=0 ? kEdge : below && below->flag_left==kThat && std::abs(raw.l-lower_raw.l)>jump ? kJump : kThat;
        row.flag_right=raw.r>=W-1 ? kEdge : below && below->flag_right==kThat && std::abs(raw.r-lower_raw.r)>jump ? kJump : kThat;
        const bool left_ok=row.flag_left==kThat, right_ok=row.flag_right==kThat;
        const double reference=below && below->flag_left==kThat && below->flag_right==kThat ? below->right-below->left : w;
        row.left=raw.l; row.right=raw.r;
        if(!left_ok && right_ok) row.left=row.right-reference;
        else if(left_ok && !right_ok) row.right=row.left+reference;
        else if(!left_ok && !right_ok) {
            const double center=below ? (below->left+below->right)/2 : (raw.l+raw.r)/2.0;
            row.left=center-reference/2; row.right=center+reference/2;
        }
        if(mode==TrajMode::ROUNDABOUT) {
            row.left=std::max(row.left,double(raw.l));
            row.right=std::min(row.right,double(raw.r));
            if(row.left>row.right) { row.left=raw.l; row.right=raw.r; }  // tâm hàng dưới nằm ngoài đoạn thô
        }
        const double width=row.right-row.left;
        row.lanes=width>=params.two_lane_ratio*w ? 2 : 1;
        row.avoid_ok=!row.hole || row.lanes==2 || row.hole_l-row.left>=params.avoid_min_width_ratio*w;
        row.target=row.lanes==2 ? row.right-kRightLaneFrac*width : (row.left+row.right)/2;
        if(mode==TrajMode::AVOID_LEFT) {
            if(row.lanes==2) row.target=row.right-kLeftLaneFrac*width;
            else if(row.hole && row.avoid_ok) row.target=(row.left+row.hole_l)/2;
        }
        const int bad_edges=int(!left_ok)+int(!right_ok);
        const bool odd_width=width<kMinWidthRatio*w || width>params.two_lane_ratio*kMergeFactor*w;
        row.conf=Clamp(1-kConfPenalty*(bad_edges+int(row.hole)+int(odd_width)),0,1);
        row.valid=true;
        corridor.rows.push_back(row);
        lower=int(corridor.rows.size())-1; lower_raw=raw;
    }
    return corridor;
}
void LaneController::Reset() { *this=LaneController{}; }
ControlResult LaneController::Step(const cv::Mat &mask, int64_t mask_ms, int64_t now_ms,
                                   const ControlParams &params, TrajMode mode) {
    params.Validate();
    if(now_ms<0 || (last_now_ms_>=0 && now_ms<last_now_ms_))
        throw std::runtime_error("Thời gian điều khiển now_ms phải tăng đơn điệu");
    const double dt=(last_now_ms_<0 ? params.control_period_ms : now_ms-last_now_ms_)/1000.0;
    last_now_ms_=now_ms;
    ControlResult result;
    result.debug.mode=TrajModeName(mode);
    auto finish=[&](double cms, double deg, const char *state, const char *reason) {
        result.cmd={cms,deg};
        result.debug.angle_deg=deg; result.debug.speed_cms=cms;
        result.debug.state=state; result.debug.reason=reason;
        angle_=deg;
        return result;
    };
    if(mask_ms<0 || mask_ms>now_ms || now_ms-mask_ms>params.mask_max_age_ms ||
       (last_mask_ms_>=0 && mask_ms<last_mask_ms_)) {
        regain_=0; derivative_ready_=false; have_look_=false;
        return finish(0,0,"MAT_NAN_CU","MAT_NAN_CU");
    }
    last_mask_ms_=mask_ms;
    bool valid=false;
    if(!mask.empty()) {
        auto corridor=ExtractCorridor(mask,params,mode);
        const double W=mask.cols, axis=W/2.0+params.center_offset_frac*W;
        std::vector<const RowDebug*> usable;
        for(const auto &row:corridor.rows) if(row.valid && row.conf>=params.conf_min) usable.push_back(&row);
        result.debug.valid_rows=int(usable.size());
        if(int(usable.size())>=std::max(2,params.min_valid_rows)) {
            // Bình phương tối thiểu trọng số conf, biến y dịch tâm c cho ổn định số rồi đổi về y pixel.
            const int degree=usable.size()>=3 ? 2 : 1;
            double c=0, total=0;
            for(const auto *row:usable) { c+=row->conf*row->y; total+=row->conf; }
            c=total>0 ? c/total : usable.front()->y;
            cv::Mat a(int(usable.size()),degree+1,CV_64F), b(int(usable.size()),1,CV_64F), k;
            for(int i=0;i<a.rows;++i) {
                const double s=std::sqrt(usable[i]->conf), u=usable[i]->y-c;
                for(int j=0;j<=degree;++j) a.at<double>(i,j)=s*std::pow(u,j);
                b.at<double>(i)=s*usable[i]->target;
            }
            if(cv::solve(a,b,k,cv::DECOMP_SVD)) {
                const double k0=k.at<double>(0), k1=k.at<double>(1), k2=degree==2 ? k.at<double>(2) : 0;
                result.debug.fit={k0-k1*c+k2*c*c,k1-2*k2*c,k2};
                const double y_l=params.lookahead_frac*mask.rows, u=y_l-c;
                // Làm tròn 1e-6 px: bỏ nhiễu số của SVD để đường thẳng giữa trục cho e đúng 0.
                double x=std::round((k0+k1*u+k2*u*u)*1e6)/1e6;
                const RowDebug *nearest=usable.front();
                for(const auto *row:usable) if(std::abs(row->y-y_l)<std::abs(nearest->y-y_l)) nearest=row;
                x=Clamp(x,nearest->left,nearest->right);
                if(have_look_ && look_mode_==mode)
                    x=Clamp(x,look_x_-params.max_jump_frac*W,look_x_+params.max_jump_frac*W);
                if(std::isfinite(x)) {
                    look_x_=x; have_look_=true; look_mode_=mode;
                    result.debug.look_x=x; result.debug.look_y=y_l;
                    result.debug.e=Clamp((x-axis)/(W/2.0),-1,1);
                    valid=true;
                } else result.debug.fit.clear();
            }
        }
        result.debug.rows=std::move(corridor.rows);
    }
    if(!valid) { have_look_=false; result.debug.look_x=-1; }
    // Mất làn/giữ góc/phục hồi: giữ nguyên logic bộ bám làn trước T79.
    if(!valid) { lost_=true; regain_=0; regain_mask_ms_=-1; derivative_ready_=false; }
    else if(lost_) {
        if(mask_ms>regain_mask_ms_) { ++regain_; regain_mask_ms_=mask_ms; }
        if(regain_>=params.regain_frames) { lost_=false; derivative_ready_=false; }
    }
    if(lost_) {
        const int64_t age=last_valid_ms_<0 ? params.lost_stop_ms : now_ms-last_valid_ms_;
        if(age>=params.lost_stop_ms) return finish(0,0,"MAT_LAN",valid ? "CHO_HOI_LAN" : "MAT_LAN");
        if(age>=params.lost_hold_ms) return finish(0,angle_,"GIU_GOC",valid ? "CHO_HOI_LAN" : "MAT_LAN_DUNG");
        return finish(params.lost_cms,angle_,"GIU_GOC",valid ? "CHO_HOI_LAN" : "MAT_LAN_GIU");
    }
    if(mode!=mode_) derivative_ready_=false;
    result.debug.p=params.kp*result.debug.e;
    result.debug.d=derivative_ready_ && dt>0 ? params.kd*(result.debug.e-error_)/dt : 0;
    const double desired=Clamp(-(result.debug.p+result.debug.d),-params.steer_max_deg,params.steer_max_deg);
    const double step=params.steer_rate_dps*dt;
    const double angle=Clamp(Clamp(desired,angle_-step,angle_+step),-params.steer_max_deg,params.steer_max_deg);
    error_=result.debug.e; derivative_ready_=true; mode_=mode; last_valid_ms_=mask_ms;
    return finish(std::max(params.min_cms,params.cruise_cms-params.slow_per_deg*std::abs(angle)),
                  angle,"BAM_LAN","BAM_LAN");
}
}
