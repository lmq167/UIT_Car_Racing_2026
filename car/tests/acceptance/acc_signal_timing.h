// Đo tín hiệu và quan sát vết trên CÙNG đồng hồ; không trừ mốc riêng của main/ucr_stop.
#pragma once
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

struct SignalTiming {
    using Clock = std::chrono::steady_clock;
    const Clock::time_point origin = Clock::now();
    std::string path;
    size_t rows = 0;
    double previous = 0, signal = 0, first_zero = -1, last_nonzero = -1;
    double zero_trace = -1, nonzero_trace = -1, value = 0, tolerance = 0;
    double zero_error = 0, nonzero_error = 0;
    double settled_zero = -1, settled_error = 0;
    bool bad = false, resumed = false, final_zero = false;

    double Now() const {
        return std::chrono::duration<double, std::milli>(Clock::now() - origin).count();
    }

    // Khoảng [lo, hi] chứa thời điểm dòng được ghi; chỉ sai số quan sát thực mới được cộng.
    void Record(double lo, double hi, double trace_ms, double speed) {
        if (hi < signal) return;
        final_zero = std::fabs(speed) <= 1e-3;
        if (std::fabs(speed) > 1e-3) {
            tolerance = std::fmax(tolerance, hi - lo);
            nonzero_error = hi - lo;
            last_nonzero = hi; nonzero_trace = trace_ms; value = speed;
            settled_zero = -1;
            if (first_zero >= 0) resumed = true;
        } else {
            if (first_zero < 0) {
                zero_error = hi - lo;
                first_zero = hi; zero_trace = trace_ms;
            }
            if (settled_zero < 0) {
                tolerance = std::fmax(tolerance, hi - lo);
                settled_zero = hi; settled_error = hi - lo;
            }
        }
    }

    void Scan(bool armed) {
        const double lo = previous;
        const double start = Now();
        std::ifstream in(path);
        std::string line;
        size_t count = 0;
        while (std::getline(in, line)) {
            if (in.eof()) break;  // Không lấy dòng đang ghi dở.
            if (++count <= rows || !armed) continue;
            std::istringstream fields(line);
            std::string ms, type, number;
            std::getline(fields, ms, ','); std::getline(fields, type, ',');
            std::getline(fields, number);
            if (type != "motor_speed") continue;
            char *end = nullptr;
            const double speed = std::strtod(number.c_str(), &end);
            if (end == number.c_str() || *end || !std::isfinite(speed)) { bad = true; continue; }
            const double trace_ms = std::strtod(ms.c_str(), &end);
            if (end == ms.c_str() || *end || !std::isfinite(trace_ms)) { bad = true; continue; }
            Record(lo, Now(), trace_ms, speed);
        }
        rows = count;
        previous = start;
    }

    bool Passed(double budget, double period, bool killed) const {
        // Máy đo bị trễ hơn một chu kỳ thì HỎNG tiền đề, không dùng nó để nới hạn.
        return budget > 0 && tolerance <= period && !bad && final_zero && first_zero >= signal &&
            first_zero - signal <= budget + (killed ? 0 : zero_error) &&
            settled_zero - signal <= budget + (killed ? 0 : settled_error) &&
            (last_nonzero < 0 || last_nonzero - signal <= budget + (killed ? 0 : nonzero_error));
    }

    void Print(double budget, double period, bool killed) const {
        std::fprintf(stderr,
            "[DO_DUNG] nguon=%s signal=%.3f ms; motor_khac_0_cuoi=%.6g tai %.3f ms (vet=%.3f ms, tre=%.3f ms); "
            "motor_0_dau=%.3f ms (vet=%.3f ms, tre=%.3f ms); han=%.3f ms; dung_sai_do=%.3f ms "
            "(lenh_khac_0=%.3f, lenh_0=%.3f); dung_on_dinh=%.3f ms (tre=%.3f ms, sai_so=%.3f ms); "
            "chu_ky=%.3f ms; co_lenh_khac_0_sau_0=%d; vet_loi=%d; phan_loai=%s; ket_qua=%s (-1=khong_co)\n",
            killed ? "ucr_stop" : "main", signal, value, last_nonzero, nonzero_trace,
            last_nonzero < 0 ? -1 : last_nonzero - signal, first_zero, zero_trace,
            first_zero < 0 ? -1 : first_zero - signal, budget, tolerance, nonzero_error, zero_error,
            settled_zero, settled_zero < 0 ? -1 : settled_zero - signal, settled_error, period,
            resumed ? 1 : 0, bad ? 1 : 0,
            bad ? "VET_LOI" : tolerance > period ? "PHEP_DO_TRE" : first_zero < 0 ? "THIEU_LENH_DUNG" :
            !final_zero ? "LENH_CUOI_KHAC_0" : Passed(budget, period, killed) ? "TRONG_HAN" :
            settled_zero - settled_error - signal <= budget &&
                (last_nonzero < 0 || last_nonzero - nonzero_error - signal <= budget) ?
                "PHEP_DO_GIAO_HAN" : "NGHI_DUNG_TRE",
            Passed(budget, period, killed) ? "DAT" : "HONG");
    }
};

// Ca biên độc lập, không chạy mã xe; được gọi bởi acc_shell_harness.
inline int SignalTimingSelfTest(const char *path = nullptr) {
    SignalTiming t;
    t.signal = 100;
    t.Record(100, 102, 701, 20);
    t.Record(118, 120, 719, 0);
    if (!t.Passed(40, 20, false)) return 1;
    t.Record(120, 121, 720, 20);
    if (t.Passed(40, 20, false)) return 2;
    t.Record(121, 122, 721, 0);  // Lệnh đang bay trong cửa sổ phản ứng vẫn phải có dừng cuối.
    if (!t.Passed(40, 20, false)) return 7;
    t.Record(150, 152, 751, 20);
    t.Record(152, 153, 752, 0);
    if (t.Passed(40, 20, false)) return 8;
    SignalTiming late;
    late.signal = 100;
    late.Record(150, 152, 751, 20);
    late.Record(153, 154, 753, 0);
    if (late.Passed(40, 20, false)) return 3;
    SignalTiming edge;
    edge.signal = 100;
    edge.Record(139, 141, 740, 0);
    if (!edge.Passed(40, 20, false) || edge.Passed(40, 20, true)) return 4;
    SignalTiming stalled;
    stalled.signal = 100;
    stalled.Record(100, 160, 750, 0);
    if (stalled.Passed(40, 20, false)) return 5;
    SignalTiming missing;
    missing.signal = 100;
    if (missing.Passed(40, 20, false)) return 6;
    SignalTiming separate;
    separate.signal = 100;
    separate.Record(100, 119, 710, 20);
    separate.Record(150, 151, 750, 0);
    if (separate.Passed(40, 20, false)) return 12;  // Không mượn sai số mẫu khác để nới hạn.
    SignalTiming held;
    held.signal = 100;
    held.Record(100, 101, 700, 0);
    held.Record(101, 102, 701, 20);
    held.Record(199, 200, 799, 0);
    if (held.Passed(40, 20, false)) return 13;  // Lệnh tiến sớm nhưng giữ lâu vẫn phải hỏng.
    if (path) {
        { std::ofstream out(path); out << "0,motor_speed,20\n"; }
        SignalTiming parser;
        parser.path = path; parser.Scan(false); parser.signal = parser.Now();
        { std::ofstream out(path, std::ios::app); out << "999,motor_speed,0"; }
        parser.Scan(true);
        if (parser.first_zero >= 0) return 9;
        { std::ofstream out(path, std::ios::app); out << '\n'; }
        parser.Scan(true);
        if (parser.first_zero < 0 || parser.zero_trace != 999) return 10;
        { std::ofstream out(path, std::ios::app); out << "1000,motor_speed,sai\n"; }
        parser.Scan(true);
        if (!parser.bad || parser.Passed(40, 20, false)) return 11;
    }
    std::puts("[DO_DUNG] 10 ca bien thoi gian DAT; 3 ca doc vet khi co duong dan");
    return 0;
}
