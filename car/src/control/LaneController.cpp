#include "control/LaneController.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace control {
namespace {
constexpr double kNoiseFrac=0.15;    // TẠM, CHƯA ĐO: đoạn hẹp hơn 0,15·w(y) là nhiễu
constexpr double kConfPenalty=0.3;   // TẠM, CHƯA ĐO: mức trừ độ tin cậy mỗi dấu hiệu xấu
constexpr double kNoRealEdgePenalty=0.1; // TẠM, CHƯA ĐO: hai biên không thật phải thấp hơn conf_min mặc định 0,4
constexpr double kMergeFactor=1.5;   // TẠM, CHƯA ĐO: gộp đoạn khi bề rộng gộp ≤ two_lane_ratio·w·1,5
constexpr double kMinWidthRatio=0.6; // TẠM, CHƯA ĐO: bề rộng dưới 0,6·w(y) là bất thường
constexpr double kRightLaneFrac=0.25, kLeftLaneFrac=0.75;  // tâm làn phải/trái của đường hai làn
constexpr double kReturnKpScale=2.0;  // TẠM, CHƯA ĐO: độ lợi P khi về làn phải sau né (mô hình xe: dịch ngang 30 cm cần >= 59 cm quãng ở lái 40°). Quân 12/11: 1,5 -> 2,0 vì lượt chạy sa bàn trả làn chưa đủ mạnh (lái chỉ -10..-16° ở e 0,2-0,3)
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
        case TrajMode::TURN_LEFT: return "RE_HANH_LANG_TRAI";
        case TrajMode::TURN_RIGHT: return "RE_HANH_LANG_PHAI";
        default: return "BAM_LAN";
    }
}
Corridor ExtractCorridor(const cv::Mat &mask, const ControlParams &params, TrajMode mode, int prior_lanes) {
    CheckMask(mask);
    const bool branch=mode==TrajMode::ROUNDABOUT || mode==TrajMode::TURN_LEFT || mode==TrajMode::TURN_RIGHT;
    const int W=mask.cols, H=mask.rows, n=params.corridor_rows;
    const double axis=W/2.0+params.center_offset_frac*W;
    const double y_near=std::lround(params.corridor_bottom_frac*H), y_far=std::lround(params.corridor_top_frac*H);
    Corridor corridor;
    corridor.rows.reserve(n);
    int lower=-1;          // chỉ số hàng hợp lệ gần nhất phía dưới
    Seg lower_raw{0,0};    // đoạn thô của hàng đó (trước khi thay biên)
    bool stopped=false;
    bool real_one_lane_below=false;  // đã thấy hàng một làn có hai biên thật ở dưới trong khung này
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
            if(!branch) for(size_t k=1;k<segs.size();++k) {
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
            if(overlap.size()>=2 && !branch &&
               overlap.back().r-overlap.front().l<=params.two_lane_ratio*w*kMergeFactor) pick=overlap;
            else if(branch) pick={mode==TrajMode::TURN_LEFT ? overlap.front() : overlap.back()};
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
        const bool right_edge_cut=row.flag_right==kEdge;
        const double reference=below && below->flag_left==kThat && below->flag_right==kThat ? below->right-below->left : w;
        row.raw_l=raw.l; row.raw_r=raw.r;
        row.left=raw.l; row.right=raw.r;
        if(!left_ok && right_ok) row.left=row.right-reference;
        else if(left_ok && !right_ok) {
            row.right=row.left+reference;
            // Lỗi 3 (sa bàn 10/11): biên phải chạm mép ảnh nên bề rộng thật >= bề rộng thô. Thô đã đủ
            // hai làn thì dựng đường hai làn (2·w) từ biên trái thật: mục tiêu ở làn PHẢI, không ở làn trái.
            // Biên phải thật (biên trái mép ảnh) không cần: tâm làn phải = phải − w/2 cho cả 1 và 2 làn.
            // Chỉ trong lần qua vòng xuyến (cả chặng là đường hai làn, Quân 10/11) và khi hàng gần nhất đã rộng hai làn
            // hoặc hàng ngay dưới đã là 2 làn. Ở LANE, đường một làn có lối mở phải (ngã T) cho đoạn thô giống hệt
            // nên không áp (safety-reviewer 11/11, MEDIUM-1).
            if(mode==TrajMode::ROUNDABOUT && right_edge_cut && raw.r-raw.l>=params.two_lane_ratio*w && (!below || below->lanes==2))
                row.right=row.left+std::max(reference,2*w);
            // Khung trước đã thấy rõ đường hai làn mà biên phải nay chạm mép ảnh (xe lệch sang làn trái khi ra cua phải):
            // giữ bề rộng hai làn để mục tiêu vẫn ở làn phải, không trượt về làn trái. Không áp khi hàng ngay dưới
            // CHÍNH LÀ đường một làn quan sát thật (hai biên thật): đó là ngã T/lối mở phải sau đoạn hai làn.
            else if(mode==TrajMode::LANE && right_edge_cut && prior_lanes==2 && !real_one_lane_below)
                row.right=row.left+std::max(reference,2*w);
        }
        else if(!left_ok && !right_ok) {
            const double center=below ? (below->left+below->right)/2 : (raw.l+raw.r)/2.0;
            row.left=center-reference/2; row.right=center+reference/2;
        }
        if(branch) {
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
        row.conf=Clamp(1-kConfPenalty*(bad_edges+int(row.hole)+int(odd_width))-(bad_edges==2 ? kNoRealEdgePenalty : 0),0,1);
        row.valid=true;
        if(row.flag_left==kThat && row.flag_right==kThat && row.lanes==1 && !row.hole) real_one_lane_below=true;
        corridor.rows.push_back(row);
        lower=int(corridor.rows.size())-1; lower_raw=raw;
    }
    return corridor;
}
bool BackupLookahead(const cv::Mat &mask, const ControlParams &params, double &look_x, int &rows_used) {
    look_x=-1; rows_used=0;
    if(mask.empty()) return false;
    CheckMask(mask);
    const int W=mask.cols, H=mask.rows;
    const double axis=W/2.0+params.center_offset_frac*W;
    const double near=std::lround(params.corridor_bottom_frac*H), far=std::lround(params.corridor_top_frac*H);
    Seg lower{0,0};
    std::vector<double> ys, targets;
    for(int i=0;i<params.corridor_rows;++i) {
        const double frac=params.corridor_bottom_frac+(params.corridor_top_frac-params.corridor_bottom_frac)*i/std::max(1,params.corridor_rows-1);
        const int y=int(Clamp(std::lround(frac*H),0,H-1));
        const double t=near>far ? Clamp((near-y)/(near-far),0,1) : 0;
        const double w=W*(params.lane1_width_frac[0]+(params.lane1_width_frac[1]-params.lane1_width_frac[0])*t);
        const auto *pixels=mask.ptr<unsigned char>(y);
        Seg pick{0,-1}; int best=-1;
        for(int x=0;x<W;) {
            if(!pixels[x]) { ++x; continue; }
            const int l=x; while(x<W && pixels[x]) ++x;
            const Seg s{l,x-1};
            if(x-l<kNoiseFrac*w) continue;
            if(ys.empty()) {
                if(s.l<=axis && axis<=s.r) { pick=s; break; }
            } else {
                const int overlap=std::min(s.r,lower.r)-std::max(s.l,lower.l);
                if(overlap>=0 && overlap>best) { best=overlap; pick=s; }
            }
        }
        if(pick.r<pick.l) break;
        ys.push_back(y);
        targets.push_back((pick.l==0 && pick.r==W-1) || pick.r-pick.l<w ?
            (pick.l+pick.r)/2.0 : pick.r-params.lane_backup_right_frac*w);
        lower=pick;
    }
    rows_used=int(ys.size());
    if(rows_used<std::max(2,params.min_valid_rows)) return false;
    const int degree=rows_used>=3 ? 2 : 1;
    double c=0; for(double y:ys) c+=y; c/=rows_used;
    cv::Mat a(rows_used,degree+1,CV_64F), b(rows_used,1,CV_64F), k;
    for(int i=0;i<rows_used;++i) {
        for(int j=0;j<=degree;++j) a.at<double>(i,j)=std::pow(ys[i]-c,j);
        b.at<double>(i)=targets[i];
    }
    if(!cv::solve(a,b,k,cv::DECOMP_SVD)) return false;
    const double u=params.lookahead_frac*H-c;
    const double x=k.at<double>(0)+k.at<double>(1)*u+(degree==2 ? k.at<double>(2)*u*u : 0);
    if(!std::isfinite(x)) return false;
    look_x=std::round(x*1e6)/1e6;
    return true;
}
void LaneController::Reset() {
    *this=LaneController{}; }
ControlResult LaneController::Step(const cv::Mat &mask, int64_t mask_ms, int64_t now_ms,
                                   const ControlParams &params, TrajMode mode, bool wide_center) {
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
        regain_=0; derivative_ready_=false; have_look_=false; lanes_memory_=0;
        return finish(0,0,"MAT_NAN_CU","MAT_NAN_CU");
    }
    last_mask_ms_=mask_ms;
    bool valid=false;
    if(!mask.empty()) {
        auto corridor=ExtractCorridor(mask,params,mode,lanes_memory_);
        const double W=mask.cols, axis=W/2.0+params.center_offset_frac*W;
        // TẠM CHƯA ĐO: chỉ dùng tâm vùng đường khi hàng gần nhất phủ ít nhất 90%.
        // Giữ cờ/độ tin cậy gốc trong debug; không biến mép ảnh thành cạnh thật.
        auto centered=corridor.rows;
        const size_t near_count=1;
        // Đầu ngã T và cua L cho mặt nạ giống nhau: Planner tắt nhánh này khi đang có ý định rẽ (ngã T chờ M6).
        bool wide_near=wide_center && mode==TrajMode::LANE && !centered.empty();
        for(size_t i=0;i<near_count && wide_near;++i) {
            const auto &row=centered[i];
            wide_near=row.valid && !row.hole && row.flag_left==kEdge && row.flag_right==kEdge &&
                cv::countNonZero(mask.row(row.y))>=0.9*W;
        }
        const bool far_direction=std::any_of(centered.begin()+std::min(near_count,centered.size()),centered.end(),
            [&](const RowDebug &row) {
                return row.valid && !row.hole && row.conf>=params.conf_min &&
                    (row.flag_left==kThat || row.flag_right==kThat);
            });
        if(wide_near && far_direction) for(auto &row:centered) {
            if(!row.valid || row.hole) continue;
            // Tâm đoạn thô cho hướng ở xa, không dựng bề rộng từ một cạnh duy nhất.
            row.left=row.raw_l; row.right=row.raw_r;
            row.target=(row.raw_l+row.raw_r)/2;
            if(row.flag_left==kEdge && row.flag_right==kEdge &&
               cv::countNonZero(mask.row(row.y))>=0.9*W) row.conf=1;
        }
        const auto &fit_rows=wide_near && far_direction ? centered : corridor.rows;
        std::vector<const RowDebug*> usable;
        for(const auto &row:fit_rows) if(row.valid && row.conf>=params.conf_min) usable.push_back(&row);
        result.debug.valid_rows=int(usable.size());
        const bool real_edge=std::any_of(usable.begin(),usable.end(),[](const RowDebug *row) {
            return row->flag_left==kThat || row->flag_right==kThat;
        });
        double x=-1;
        const double y_l=params.lookahead_frac*mask.rows;
        if((mode==TrajMode::ROUNDABOUT || real_edge) && int(usable.size())>=std::max(2,params.min_valid_rows)) {
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
                const double u=y_l-c;
                // Làm tròn 1e-6 px: bỏ nhiễu số của SVD để đường thẳng giữa trục cho e đúng 0.
                x=std::round((k0+k1*u+k2*u*u)*1e6)/1e6;
                const RowDebug *nearest=usable.front();
                for(const auto *row:usable) if(std::abs(row->y-y_l)<std::abs(nearest->y-y_l)) nearest=row;
                x=Clamp(x,nearest->left,nearest->right);
                valid=std::isfinite(x);
            }
        }
        if(params.lane_backup && mode==TrajMode::LANE) {
            valid=BackupLookahead(mask,params,x,result.debug.valid_rows);
            result.debug.fit.clear();
        }
        if(valid) {
                if(have_look_ && look_mode_==mode)
                    x=Clamp(x,look_x_-params.max_jump_frac*W,look_x_+params.max_jump_frac*W);
                if(std::isfinite(x)) {
                    look_x_=x; have_look_=true; look_mode_=mode;
                    result.debug.look_x=x; result.debug.look_y=y_l;
                    result.debug.e=Clamp((x-axis)/(W/2.0),-1,1);
                    valid=true;
                } else result.debug.fit.clear();
        }
        {   // Nhớ số làn chỉ từ các hàng có cả hai biên thật (không lấy từ hàng dựng lại).
            int two=0, one=0;
            for(const auto &row:corridor.rows) if(row.valid && !row.hole && row.flag_left==kThat && row.flag_right==kThat) ++(row.lanes==2 ? two : one);
            // Chỉ học ở LANE; chế độ khác (vòng xuyến, rẽ, né) xóa để vùng đất rộng không thành "hai làn".
            if(mode!=TrajMode::LANE) lanes_memory_=0;
            else if(valid && two>=2 && two>one) lanes_memory_=2;
            else if(valid && one>=2 && one>two) lanes_memory_=1;
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
        if(age>=params.lost_hold_ms) lanes_memory_=0;
        if(age>=params.lost_hold_ms) return finish(0,angle_,"GIU_GOC",valid ? "CHO_HOI_LAN" : "MAT_LAN_DUNG");
        return finish(params.lost_cms,angle_,"GIU_GOC",valid ? "CHO_HOI_LAN" : "MAT_LAN_GIU");
    }
    if(mode!=mode_) derivative_ready_=false;
    // Về làn phải sau né cần dịch ngang ~30 cm trong quãng ngắn (log 11/11 m4: lái chỉ tới 23° với e 0,67): tăng độ lợi khi RETURN_RIGHT.
    result.debug.p=params.kp*(mode==TrajMode::RETURN_RIGHT ? kReturnKpScale : 1.0)*result.debug.e;
    result.debug.d=derivative_ready_ && dt>0 ? params.kd*(result.debug.e-error_)/dt : 0;
    const double desired=Clamp(-(result.debug.p+result.debug.d),-params.steer_max_deg,params.steer_max_deg);
    const double step=params.steer_rate_dps*dt;
    const double angle=Clamp(Clamp(desired,angle_-step,angle_+step),-params.steer_max_deg,params.steer_max_deg);
    error_=result.debug.e; derivative_ready_=true; mode_=mode; last_valid_ms_=mask_ms;
    return finish(std::max(params.min_cms,params.cruise_cms-params.slow_per_deg*std::abs(angle)),
                  angle,"BAM_LAN","BAM_LAN");
}
}
