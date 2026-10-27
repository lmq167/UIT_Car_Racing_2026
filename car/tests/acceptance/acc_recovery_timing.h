// Tiện ích thời gian của test hiệu chuẩn và an toàn; không đổi ngưỡng an toàn trong cấu hình xe.
#pragma once
#include <chrono>
#include <cstdint>
#include <thread>
#include <algorithm>
#include <utility>
#include <vector>

namespace acc_fixture {
constexpr int ScheduleSlackMs=100; // Dung sai lịch theo danh sách ca đã duyệt; không cộng vào tuổi ảnh hay trần tăng mềm.
template<class Predicate>
bool WaitUntil(std::chrono::steady_clock::time_point deadline,Predicate ready) {
    do {
        if(ready()) return true;
        if(std::chrono::steady_clock::now()>=deadline) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } while(true);
}
struct Frame { std::uint64_t id; long long ms; };
inline long long ObservedPeriodMs(const std::vector<Frame> &frames,
                                 const std::vector<std::pair<int,int>> &gaps) {
    long long period=0;
    for(size_t i=1;i<frames.size();++i) {
        const auto before=frames[i-1],after=frames[i];
        bool interrupted=false;
        for(const auto &gap:gaps)
            if(before.ms<gap.second && after.ms>=gap.first) interrupted=true;
        // Đo nhịp ảnh Controller thật sự thấy, kể cả khi nó bỏ khung; không tính thời gian ngắt.
        if(!interrupted && after.id>before.id && after.ms>before.ms)
            period=std::max(period,after.ms-before.ms);
    }
    return period;
}
inline long long RecoveryCheckMs(Frame after,long long ramp_ms,int cycle_ms,long long period_ms) {
    return after.ms+ramp_ms+std::max(static_cast<long long>(cycle_ms),period_ms)+ScheduleSlackMs;
}
struct RecoveryWindow {
    long long last_latest,first_earliest,duration_min,duration_max;
};
inline RecoveryWindow FrameWindow(Frame before,Frame after,int gap_begin,int gap_end) {
    const auto duration=after.ms-before.ms;
    if(after.id==before.id+1) return {before.ms,after.ms,duration,duration};
    // Controller đọc ảnh mới nhất, có thể bỏ khung; khoảng ngắt chỉ cho cận, không cho D thật.
    return {gap_begin,gap_end,gap_end-gap_begin,duration};
}
}
