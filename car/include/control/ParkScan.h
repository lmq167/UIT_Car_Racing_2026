#pragma once
#include <cstdint>
#include <string>

namespace control {
struct TofSlot {
    bool initialized=false, valid=false, has_sample=false;
    int mm=-1, api_error=0, range_status=255;
    int64_t sample_ms=-1;
};
struct ParkScanConfig {
    int map[4]={-1,-1,-1,-1}; // Trái trước/sau, phải trước/sau; chưa xác nhận.
    // TẠM: toàn bộ số đo và ngưỡng này chưa xác nhận trên xe.
    double occ_mm=400, slot_cm=50, slot_tol_cm=10, front_off_cm=19,
           rear_off_cm=0, max_gap_cm=3, edge_tol_cm=6;
    int max_age_ms=300, min_samples=3;
};
enum class ScanCell { UNKNOWN, EMPTY, OCCUPIED };
struct ParkSlotResult {
    bool found=false;
    int side=0; // -1 phải, +1 trái.
    double near_x_cm=0, far_x_cm=0;
    bool open_near=false;
};
class ParkScan {
public:
    explicit ParkScan(const ParkScanConfig&);
    void Reset();
    void Update(int64_t now_ms, double odo_cm, const TofSlot slots[4]);
    ParkSlotResult Result() const;
    ScanCell Current(int side) const;
    const std::string& Why() const;
private:
    struct Run {
        ScanCell cell=ScanCell::UNKNOWN;
        double first=0, last=0;
        int count=0;
        bool started=false, near=false, open=false, closed=false;
        double end=0;
    };
    ParkScanConfig cfg_;
    Run runs_[4], finished_[4];
    ScanCell current_[2]={ScanCell::UNKNOWN, ScanCell::UNKNOWN};
    bool have_odo_=false;
    double odo_=0;
    ParkSlotResult result_;
    std::string why_;
    ScanCell Classify(int64_t now, const TofSlot&) const;
    void Sample(int index, double x, ScanCell cell);
    ParkSlotResult SideResult(int side_index, std::string& why) const;
};
}
