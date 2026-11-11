#include "control/ParkScan.h"
#include <algorithm>
#include <cmath>

namespace control {
ParkScan::ParkScan(const ParkScanConfig& cfg): cfg_(cfg) { Reset(); }
void ParkScan::Reset() {
    for (int i=0; i<4; ++i) { runs_[i]=Run{}; finished_[i]=Run{}; }
    current_[0]=current_[1]=ScanCell::UNKNOWN;
    have_odo_=false;
    odo_=0;
    result_=ParkSlotResult{};
    why_="chua_thay_mep_xa";
}
ScanCell ClassifyTofSample(const TofSlot& s, int64_t now, double occ_mm, int max_age_ms) {
    if (!s.initialized || !s.has_sample || s.api_error!=0 || !s.valid ||
        s.mm<0 || s.sample_ms<0 || s.sample_ms>now ||
        static_cast<long double>(now)-s.sample_ms>max_age_ms)
        return ScanCell::UNKNOWN;
    return s.mm<=occ_mm ? ScanCell::OCCUPIED : ScanCell::EMPTY;
}
ScanCell ParkScan::Classify(int64_t now, const TofSlot& s) const {
    return ClassifyTofSample(s,now,cfg_.occ_mm,cfg_.max_age_ms);
}
void ParkScan::Sample(int i, double x, ScanCell cell) {
    Run& r=runs_[i];
    const bool adjacent=r.started && x-r.last<=cfg_.max_gap_cm;
    if (cell==r.cell && cell!=ScanCell::UNKNOWN && adjacent) {
        r.last=x;
        ++r.count;
        return;
    }
    if (r.cell==ScanCell::EMPTY && cell==ScanCell::OCCUPIED && adjacent) {
        finished_[i]=r;
        finished_[i].closed=true;
        finished_[i].end=x;
    } else {
        // Không giữ kết quả cũ khi bắt đầu ô mới hoặc gặp dữ liệu không chắc.
        finished_[i]=Run{};
    }
    Run next;
    next.cell=cell;
    next.first=next.last=x;
    next.count=cell==ScanCell::UNKNOWN ? 0 : 1;
    next.near=cell==ScanCell::EMPTY && r.cell==ScanCell::OCCUPIED && adjacent;
    next.open=cell==ScanCell::EMPTY && !r.started;
    next.started=true;
    r=next;
}
ParkSlotResult ParkScan::SideResult(int side, std::string& why) const {
    ParkSlotResult out;
    const Run& f=finished_[side*2];
    const Run& r=finished_[side*2+1];
    if (!f.closed || !r.closed) { why="chua_thay_mep_xa"; return out; }
    if (f.count<cfg_.min_samples || r.count<cfg_.min_samples ||
        (!f.near && !f.open) || (!r.near && !r.open)) {
        why="chuoi_khong_du"; return out;
    }
    if (std::abs(f.end-r.end)>cfg_.edge_tol_cm) {
        why="mep_khong_khop"; return out;
    }
    const double near=std::max(f.first,r.first), far=std::min(f.end,r.end);
    // Độ dài xác nhận là phần giao, không phải hợp hai chuỗi.
    const double length=far-near;
    if (length<cfg_.slot_cm-cfg_.slot_tol_cm) { why="o_qua_ngan"; return out; }
    out.open_near=f.open || r.open;
    if (!out.open_near && length>cfg_.slot_cm+cfg_.slot_tol_cm) {
        why="o_qua_dai"; return ParkSlotResult{};
    }
    out.found=true;
    out.side=side==0 ? 1 : -1;
    out.far_x_cm=far;
    out.near_x_cm=out.open_near ? far-cfg_.slot_cm : near;
    return out;
}
void ParkScan::Update(int64_t now, double odo, const TofSlot slots[4]) {
    if (!std::isfinite(odo) || (have_odo_ && odo<=odo_)) return;
    have_odo_=true;
    odo_=odo;
    for (int i=0; i<4; ++i) {
        const int mapped=cfg_.map[i];
        const ScanCell cell=mapped>=0 && mapped<4 ? Classify(now,slots[mapped]) : ScanCell::UNKNOWN;
        Sample(i,odo+(i%2==0 ? cfg_.front_off_cm : cfg_.rear_off_cm),cell);
    }
    for (int s=0; s<2; ++s)
        current_[s]=runs_[2*s].cell==runs_[2*s+1].cell ? runs_[2*s].cell : ScanCell::UNKNOWN;
    std::string left_why, right_why;
    const auto left=SideResult(0,left_why), right=SideResult(1,right_why);
    result_=ParkSlotResult{};
    if (left.found && right.found) {
        if (std::abs(left.far_x_cm-right.far_x_cm)<=1e-9) why_="hai_ben";
        else result_=left.far_x_cm<right.far_x_cm ? left : right;
    } else if (left.found) result_=left;
    else if (right.found) result_=right;
    else why_=left_why;
    if (result_.found) why_.clear();
}
ParkSlotResult ParkScan::Result() const { return result_; }
ScanCell ParkScan::Current(int side) const {
    return side==1 ? current_[0] : side==-1 ? current_[1] : ScanCell::UNKNOWN;
}
const std::string& ParkScan::Why() const { return why_; }
}
