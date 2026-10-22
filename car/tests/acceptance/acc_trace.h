// Tiện ích đọc vết phần cứng giả (UCR_FAKE_HW_TRACE) cho các test chấp nhận.
// Định dạng (docs/ucr_car.md, mục "Điểm cắm kiểm thử bản giả lập"): mỗi dòng "<ms>,<loại>,<giá_trị>",
// loại thuộc motor_speed | servo_angle | motor_cfg | motor_init. Mốc ms tính từ lúc khởi động của MỖI tiến trình
// (main và ucr_stop có mốc riêng) nên so thứ tự giữa hai tiến trình phải dựa vào thứ tự dòng, không dựa vào ms.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>
#include "acc_common.h"

namespace acc {

struct TraceLine {
    long long ms = 0;
    std::string kind, value;
    double num = 0;  // giá trị số (motor_speed, servo_angle); vô nghĩa với motor_cfg
};
using Trace = std::vector<TraceLine>;

// Đọc cả file vết. Dòng sai định dạng (thiếu dấu phẩy) bị bỏ qua nhưng được đếm vào *bad (nếu cho).
inline Trace ReadTrace(const std::string &path, int *bad = nullptr) {
    Trace out;
    if (bad) *bad = 0;
    std::ifstream f(path.c_str());
    std::string line;
    while (std::getline(f, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;
        const auto a = line.find(',');
        const auto b = a == std::string::npos ? std::string::npos : line.find(',', a + 1);
        if (b == std::string::npos) { if (bad) ++*bad; continue; }
        TraceLine t;
        t.ms = std::atoll(line.substr(0, a).c_str());
        t.kind = line.substr(a + 1, b - a - 1);
        t.value = line.substr(b + 1);
        t.num = std::strtod(t.value.c_str(), nullptr);
        out.push_back(t);
    }
    return out;
}

inline Trace Of(const Trace &t, const std::string &kind) {
    Trace out;
    for (const auto &l : t) if (l.kind == kind) out.push_back(l);
    return out;
}
inline size_t Count(const Trace &t, const std::string &kind) { return Of(t, kind).size(); }
// Giá trị số của dòng cuối cùng loại kind; false nếu không có dòng nào.
inline bool LastNum(const Trace &t, const std::string &kind, double &v) {
    bool found = false;
    for (const auto &l : t) if (l.kind == kind) { v = l.num; found = true; }
    return found;
}
inline bool Near(double a, double b, double eps = 1e-3) { return std::fabs(a - b) <= eps; }
// Có lệnh tốc độ khác 0 (đã di chuyển) hay không.
inline bool Moved(const Trace &t) {
    for (const auto &l : t) if (l.kind == "motor_speed" && std::fabs(l.num) > 0.5) return true;
    return false;
}

}  // namespace acc
