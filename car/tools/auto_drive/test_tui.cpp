// Kiểm hàm thuần dựng khung TUI (P5b): độ rộng, mã điều khiển cho phép, số dòng cố định, vùng sự kiện, mẫu.
#include "tui.h"
#include <fstream>
#include <iostream>
#include <regex>

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { std::cerr << __LINE__ << ": " #cond "\n"; ++failures; } } while (0)

namespace {
auto_drive::TuiState sample() {
    auto_drive::TuiState t;
    t.git = "5ef5b75"; t.model = "ucr_models_t94"; t.mode = "DANG CHAY"; t.state = "RE_TRAI"; t.reason = "RE_TRAI";
    t.run_s = 125.4; t.rec_s = 64.5; t.recording = true; t.cms = 8; t.deg = 31; t.e = -0.12;
    t.sensor_ok = true; t.yaw = 47.2; t.odo = 738.9; t.sensor_age_ms = 30;
    t.turning = true; t.yaw_tgt = 90; t.yaw_err = 42.8; t.progress_pct = 52; t.route_step = 3; t.turn_dir = 1;
    t.frame_age_ms = 140; t.camera_fps = 29.8; t.road_ms = 38; t.loop_p95_ms = 12.5;
    for (int i = 0; i < 12; ++i) t.events.push_back({"su kien " + std::to_string(i), i == 11});
    t.events.push_back({std::string(150, 'x'), false});  // dòng quá dài phải bị cắt
    return t;
}
std::vector<std::string> lines(const std::string& s) {
    std::vector<std::string> out; size_t a = 0;
    for (size_t b; (b = s.find('\n', a)) != std::string::npos; a = b + 1) out.push_back(s.substr(a, b - a));
    out.push_back(s.substr(a));
    return out;
}
}

int main(int argc, char** argv) {
    const auto t = sample();
    const std::string color = auto_drive::renderTui(t, true), plain = auto_drive::renderTui(t, false);
    CHECK(color.rfind("\033[H", 0) == 0 && plain.rfind("\033[H", 0) == 0);
    CHECK(color.size() >= 3 && color.substr(color.size() - 3) == "\033[J");
    // Chỉ các mã cho phép: ESC[H, ESC[K, ESC[J, ESC[<số>m.
    const std::regex esc("\033\\[[^A-Za-z]*[A-Za-z]"), ok("\033\\[(H|K|J|[0-9;]*m)");
    for (auto it = std::sregex_iterator(color.begin(), color.end(), esc); it != std::sregex_iterator(); ++it)
        CHECK(std::regex_match(it->str(), ok));
    CHECK(plain.find("\033[3") == std::string::npos && plain.find("m") != std::string::npos);  // không màu khi tắt
    const auto ls = lines(color);
    CHECK(ls.size() == 18);  // 9 dòng cố định + 8 sự kiện + ESC[J
    for (const auto& l : ls) CHECK(auto_drive::tuiVisibleWidth(l) <= auto_drive::kTuiWidth);
    CHECK(color.find("su kien 4") == std::string::npos && color.find("su kien 5") != std::string::npos);  // chỉ 8 sự kiện cuối
    CHECK(color.find("\033[31msu kien 11") != std::string::npos);  // cảnh báo tô đỏ
    CHECK(color.find("s=64.5") != std::string::npos && color.find("tien do 52%") != std::string::npos);
    CHECK(color.find("G chay | Space dung | R nap yaml | Esc thoat") != std::string::npos);
    auto empty = auto_drive::TuiState();
    CHECK(lines(auto_drive::renderTui(empty, false)).size() == 18 &&
          auto_drive::renderTui(empty, false).find("s=--") != std::string::npos);
    CHECK(auto_drive::tuiClip("aếb", 2) == "aế");
    if (argc > 1) { std::ofstream(argv[1], std::ios::binary) << plain; }  // mẫu cho tasks/tui_1112/sample.txt
    if (failures) { std::cerr << "test_tui HONG " << failures << "\n"; return 1; }
    std::cout << "test_tui OK\n";
    return 0;
}
