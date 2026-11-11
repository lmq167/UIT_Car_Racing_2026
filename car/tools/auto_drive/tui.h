#pragma once
// Giao diện terminal (TUI) của ucr_auto: hàm THUẦN dựng một khung từ trạng thái (P5b, Quân 11/11 19:24).
// Chỉ dùng mã ANSI: về đầu màn (ESC[H), xóa tới cuối dòng (ESC[K), xóa phần còn lại (ESC[J), màu (ESC[..m).
// Không đụng terminal, tín hiệu hay phím; luồng vẽ và kiểm isatty nằm ở main.cpp.
#include <algorithm>
#include <cstdio>
#include <deque>
#include <string>
#include <vector>

namespace auto_drive {
constexpr int kTuiWidth = 100, kTuiEvents = 8;
struct TuiEvent { std::string text; bool warn = false; };
struct TuiState {
    std::string git, model, mode = "CHO G", state, reason;
    double run_s = 0, rec_s = -1;          // rec_s < 0: chưa có dòng log
    bool recording = false;
    double cms = 0, deg = 0, e = 0;
    bool sensor_ok = false;
    double yaw = 0, odo = 0;
    int sensor_age_ms = -1;
    bool turning = false;
    double yaw_tgt = 0, yaw_err = 0;
    int progress_pct = 0, route_step = 0, turn_dir = 0;
    int frame_age_ms = -1;
    double camera_fps = 0, road_ms = 0, loop_p95_ms = 0;
    std::string tof;                       // dòng ToF đã định dạng (chỉ khi xe đứng); rỗng = không có
    std::deque<TuiEvent> events;           // mới nhất ở cuối
};
// Đếm cột hiển thị: bỏ qua dãy ESC[...chữ, đếm mỗi ký tự UTF-8 là một cột.
inline int tuiVisibleWidth(const std::string& s) {
    int w = 0;
    for (size_t i = 0; i < s.size(); ++i) {
        const unsigned char c = s[i];
        if (c == 0x1b && i + 1 < s.size() && s[i + 1] == '[') {
            i += 2;
            while (i < s.size() && !((s[i] >= 'A' && s[i] <= 'Z') || (s[i] >= 'a' && s[i] <= 'z'))) ++i;
            continue;
        }
        if ((c & 0xC0) != 0x80) ++w;
    }
    return w;
}
// Cắt chuỗi thuần (không mã ANSI) còn tối đa `cols` cột, không cắt giữa ký tự UTF-8.
inline std::string tuiClip(const std::string& s, int cols) {
    int w = 0;
    for (size_t i = 0; i < s.size(); ++i) {
        const unsigned char c = s[i];
        if ((c & 0xC0) != 0x80) { if (w == cols) return s.substr(0, i); ++w; }
    }
    return s;
}
inline std::string renderTui(const TuiState& t, bool color) {
    const std::string red = color ? "\033[31m" : "", yel = color ? "\033[33m" : "", grn = color ? "\033[32m" : "",
                      bold = color ? "\033[1m" : "", off = color ? "\033[0m" : "";
    std::vector<std::pair<std::string, std::string>> rows;  // (màu, nội dung thuần)
    char b[256];
    auto add = [&](const std::string& c) { rows.emplace_back(c, b); };
    char rec[32];
    if (t.rec_s >= 0) std::snprintf(rec, sizeof(rec), "s=%.1f", t.rec_s);
    else std::snprintf(rec, sizeof(rec), "s=--");
    std::snprintf(b, sizeof(b), "ucr_auto %s | model %s | chay %.0fs | %s | %s", t.git.c_str(), t.model.c_str(),
                  t.run_s, rec, t.recording ? "DANG GHI" : "khong ghi anh");
    add(bold);
    std::snprintf(b, sizeof(b), "%-9s FSM %s | %s", t.mode.c_str(), t.state.c_str(), t.reason.c_str());
    add(t.mode == "DANG CHAY" ? grn : t.mode == "DUNG" ? red : yel);
    std::snprintf(b, sizeof(b), "Lenh: toc do %.1f cm/s  lai %.0f do  e=%.2f", t.cms, t.deg, t.e);
    add("");
    if (t.sensor_ok) std::snprintf(b, sizeof(b), "IMU yaw %.1f do  odo %.1f cm  tuoi %d ms", t.yaw, t.odo, t.sensor_age_ms);
    else std::snprintf(b, sizeof(b), "IMU/encoder LOI hoac tat (tuoi %d ms)", t.sensor_age_ms);
    add(!t.sensor_ok || t.sensor_age_ms > 200 ? red : "");
    if (t.turning) std::snprintf(b, sizeof(b), "Re: %s dich %.0f con %.0f do  tien do %d%%  | lo trinh k=%d",
                                 t.turn_dir > 0 ? "TRAI" : "PHAI", t.yaw_tgt, t.yaw_err, t.progress_pct, t.route_step);
    else std::snprintf(b, sizeof(b), "Re: -  | lo trinh k=%d y_dinh=%s", t.route_step,
                       t.turn_dir > 0 ? "TRAI" : t.turn_dir < 0 ? "PHAI" : "-");
    add("");
    std::snprintf(b, sizeof(b), "Anh: tuoi %d ms  camera %.1f fps  model %.0f ms  loop_p95 %.1f ms", t.frame_age_ms,
                  t.camera_fps, t.road_ms, t.loop_p95_ms);
    add(t.frame_age_ms > 300 ? red : "");
    std::snprintf(b, sizeof(b), "ToF: %s", t.tof.empty() ? "- (chi doc khi xe dung)" : t.tof.c_str());
    add("");
    std::snprintf(b, sizeof(b), "---- su kien / canh bao ----");
    add("");
    const int n = int(t.events.size()), start = std::max(0, n - kTuiEvents);
    for (int i = start; i < n; ++i) {
        std::snprintf(b, sizeof(b), "%s", t.events[size_t(i)].text.c_str());
        add(t.events[size_t(i)].warn ? red : "");
    }
    for (int i = n - start; i < kTuiEvents; ++i) { b[0] = 0; add(""); }
    std::snprintf(b, sizeof(b), "G chay | Space dung | R nap yaml | Esc thoat");
    add(bold);
    std::string out = "\033[H";
    for (const auto& r : rows) {
        const std::string text = tuiClip(r.second, kTuiWidth);
        out += r.first.empty() ? text : r.first + text + off;
        out += "\033[K\n";
    }
    out += "\033[J";
    return out;
}
} // namespace auto_drive
