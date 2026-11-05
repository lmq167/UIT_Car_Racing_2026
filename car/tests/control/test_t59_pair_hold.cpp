#include "control/LaneController.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <map>
#include <sstream>
#include <stdexcept>

namespace {
using control::ControlParams;
using control::ControlResult;
using control::LaneController;
using Masks = std::map<std::string, cv::Mat>;

void Check(bool ok, const std::string &why) {
    if (!ok) throw std::runtime_error(why);
}

bool DataLine(std::istream &in, std::string &line) {
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (!line.empty() && line.front() != '#') return true;
    }
    Check(in.eof(), "Lỗi đọc fixture");
    return false;
}

Masks ReadMasks() {
    std::ifstream in(std::string(T55_FIXTURE_DIR) + "/t59_masks.txt");
    Check(bool(in), "Không mở được fixture mặt nạ");
    Masks masks;
    std::string line;
    const int ys[] = {81, 62, 43};
    while (DataLine(in, line)) {
        std::istringstream header(line);
        std::string tag, name;
        int run = 0, frame = 0;
        Check(bool(header >> tag >> name >> run >> frame) && tag == "CASE" &&
              run > 0 && frame >= 0 && name == "m" + std::to_string(run) + "_" + std::to_string(frame),
              "Sai đầu mục mặt nạ: " + line);
        Check((header >> std::ws).eof(), "Thừa trường đầu mục: " + name);
        cv::Mat mask(96, 160, CV_8UC1, cv::Scalar(0));
        for (int y : ys) {
            Check(DataLine(in, line) && line.size() == 160, "Thiếu/sai độ dài hàng: " + name);
            for (int x = 0; x < mask.cols; ++x) {
                Check(line[x] >= '0' && line[x] <= '2', "Sai nhãn 0/1/2: " + name);
                mask.at<unsigned char>(y, x) = static_cast<unsigned char>(line[x] - '0');
            }
        }
        Check(masks.emplace(name, mask).second, "Trùng mặt nạ: " + name);
    }
    Check(masks.size() == 50, "Fixture phải có đúng 50 mặt nạ");
    return masks;
}

ControlParams Params(int mode) {
    ControlParams p;
    p.pair_mode = mode;
    p.right_lost_ms = 0;
    p.Validate();
    Check(p.scan_rows.size() == 3, "Cần đúng ba hàng quét");
    const int ys[] = {81, 62, 43};
    for (size_t i = 0; i < 3; ++i)
        Check(static_cast<int>(p.scan_rows[i] * 96) == ys[i], "Hàng quét mặc định lệch fixture");
    return p;
}

ControlResult Step(LaneController &c, const cv::Mat &mask, const ControlParams &p, int64_t ms = 1000) {
    return c.Step(mask, ms, ms, p, control::LaneRef::RIGHT);
}

ControlResult Fresh(const cv::Mat &mask, int mode) {
    LaneController c;
    return Step(c, mask, Params(mode));
}

void Expect(bool condition, const std::string &why, bool &ok) {
    if (!condition) {
        std::cout << "KHÔNG ĐẠT: " << why << '\n';
        ok = false;
    }
}

// T59_test.md:8-9: kiểm cả khoảng e và biên độ, độc lập hoặc chuỗi cách 50 ms.
bool Straight(const Masks &masks, bool sequence) {
    const int frames[] = {260, 400, 440, 460, 520, 540, 580, 600, 760, 820,
                          840, 880, 900, 920, 980, 1000, 1040, 1060, 1080, 1140};
    LaneController c;
    const auto p = Params(1);
    double low = 1, high = -1;
    int64_t ms = 1000;
    bool ok = true;
    for (int frame : frames) {
        const std::string name = "m8887_" + std::to_string(frame);
        const auto &mask = masks.at(name);
        const double e = (sequence ? Step(c, mask, p, ms) : Fresh(mask, 1)).debug.e;
        std::cout << name << " e=" << e << '\n';
        Expect(std::isfinite(e) && e >= -0.12 && e <= 0.0, name + ": e ngoài [-0.12, 0.0]", ok);
        low = std::min(low, e);
        high = std::max(high, e);
        ms += 50;
    }
    std::cout << "max_e-min_e=" << high - low << '\n';
    Expect(high - low <= 0.03, "Biên độ e vượt 0.03", ok);
    return ok;
}

// T59_test.md:13: golden độc lập giữ nguyên, so e và cả ba hàng theo px, sai số 1e-6.
bool Golden(const Masks &masks) {
    std::ifstream in(std::string(T55_FIXTURE_DIR) + "/t59_pair0_golden.txt");
    Check(bool(in), "Không mở được golden pair_mode=0");
    std::map<std::string, bool> seen;
    std::string line, name;
    bool ok = true;
    while (DataLine(in, line)) {
        std::istringstream row(line);
        double e = 0;
        Check(bool(row >> name >> e) && std::isfinite(e), "Sai tên/e trong golden");
        Check(seen.emplace(name, true).second, "Trùng golden: " + name);
        const auto r = Fresh(masks.at(name), 0);
        Expect(std::isfinite(r.debug.e) && std::abs(r.debug.e - e) <= 1e-6, name + ": e khác golden", ok);
        Check(r.debug.rows.size() == 3, name + ": thiếu hàng kết quả");
        for (size_t i = 0; i < 3; ++i) {
            int valid = -1;
            double left = 0, right = 0, center = 0;
            char a = 0, b = 0, c = 0;
            Check(bool(row >> valid >> a >> left >> b >> right >> c >> center) &&
                  (valid == 0 || valid == 1) && a == ',' && b == ',' && c == ',' &&
                  std::isfinite(left) && std::isfinite(right) && std::isfinite(center),
                  name + ": sai hàng golden");
            const auto &actual = r.debug.rows[i];
            Expect(actual.valid == (valid == 1) && std::isfinite(actual.left) &&
                   std::isfinite(actual.right) && std::isfinite(actual.center) &&
                   std::abs(actual.left - left) <= 1e-6 && std::abs(actual.right - right) <= 1e-6 &&
                   std::abs(actual.center - center) <= 1e-6,
                   name + ": hàng " + std::to_string(i) + " khác golden", ok);
        }
        Check((row >> std::ws).eof(), name + ": thừa trường golden");
    }
    Check(seen.size() == masks.size(), "Golden không phủ đủ 50 mặt nạ");
    std::cout << "Đã so golden " << seen.size() << " mặt nạ, 150 hàng\n";
    return ok;
}

bool Run(const std::string &name, const Masks &masks) {
    bool ok = true;
    // T59_test.md:7,12: bộ điều khiển mới cho từng ảnh, kể cả ảnh lật, ref RIGHT.
    if (name == "jump_left" || name == "flip") {
        for (const std::string id : {"m8887_1620", "m8887_1700"}) {
            cv::Mat mask = masks.at(id);
            if (name == "flip") cv::flip(mask, mask, 1);
            const double e = Fresh(mask, 1).debug.e;
            std::cout << id << " e=" << e << '\n';
            Expect(std::isfinite(e) && (name == "flip" ? e > 0 : e <= 0.02),
                   id + (name == "flip" ? ": e phải > 0" : ": e phải <= 0.02"), ok);
        }
    } else if (name == "straight_stable" || name == "straight_seq") {
        return Straight(masks, name == "straight_seq");
    } else if (name == "far_not_worse") {
        // T59_test.md:10: mode 1 không tăng độ lớn sai số so mode 0 tại khung 1980.
        const auto &mask = masks.at("m8887_1980");
        const double e0 = Fresh(mask, 0).debug.e, e1 = Fresh(mask, 1).debug.e;
        std::cout << "m8887_1980 e0=" << e0 << " e1=" << e1 << '\n';
        Expect(std::isfinite(e0) && std::isfinite(e1) && std::abs(e1) <= std::abs(e0) + 1e-9,
               "m8887_1980: |e1| vượt |e0| + 1e-9", ok);
    } else if (name == "no_row_loss") {
        // Quân 05/11 (đổi tiêu chí d): luật B3 cũ giữ nguyên, hàng không ghép được cặp thì hỏng.
        // Mặt nạ có làn thật 8887 260-1780: còn >= 2 hàng, e cùng dấu và lệch <= 0,02 so với mã 28e2930.
        // Mất cả 3 hàng chỉ chấp nhận ở m8677_220 và m8677_320 (cảnh không có làn).
        std::ifstream in(std::string(T55_FIXTURE_DIR) + "/t59_cur_golden.txt");
        Check(bool(in), "Không mở được golden mã hiện tại");
        std::map<std::string, bool> real;
        LaneController c;  // e đo theo chuỗi khung như xe thật: mốc near của khung trước giữ cặp ổn định
        const auto p1 = Params(1);
        int64_t ms = 1000;
        std::string line, name2;
        while (DataLine(in, line)) {
            std::istringstream row(line);
            double e = 0;
            int rows = 0;
            Check(bool(row >> name2 >> e >> rows) && std::isfinite(e), "Sai dòng golden mã hiện tại");
            real[name2] = true;
            const auto r = Step(c, masks.at(name2), p1, ms);
            ms += 50;
            const double e1 = r.debug.e;
            std::cout << name2 << " e=" << e1 << " e_hien_tai=" << e << " hang=" << r.debug.valid_rows << '\n';
            Expect(r.debug.valid_rows >= 2, name2 + ": còn dưới 2 hàng hợp lệ", ok);
            Expect(std::isfinite(e1) && e1 * e > 0 && std::abs(e1 - e) <= 0.02, name2 + ": e khác dấu hoặc lệch > 0,02", ok);
        }
        Check(real.size() == 30, "Golden mã hiện tại phải có 30 mặt nạ 8887 260-1780");
        for (const auto &item : masks) {
            if (item.first == "m8677_220" || item.first == "m8677_320") continue;
            Expect(Fresh(item.second, 1).debug.valid_rows >= 1, item.first + ": mất cả 3 hàng ngoài hai cảnh không có làn", ok);
        }
    } else if (name == "pair0_golden") {
        return Golden(masks);
    } else {
        throw std::runtime_error("Ca không hợp lệ: " + name);
    }
    return ok;
}
}

int main(int argc, char **argv) {
    try {
        std::locale::global(std::locale::classic());
        std::cout << std::setprecision(12);
        Check(argc == 2, "Cần đúng một tên ca");
        const bool ok = Run(argv[1], ReadMasks());
        std::cout << (ok ? "ĐẠT: " : "KHÔNG ĐẠT: ") << argv[1] << '\n';
        return ok ? 0 : 1;
    } catch (const std::exception &e) {
        std::cerr << "KHÔNG ĐẠT: " << e.what() << '\n';
        return 1;
    }
}
