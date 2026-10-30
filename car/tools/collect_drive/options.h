#pragma once
#include "limits.h"
#include "../collect_common/pass_store.h"
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace collect_drive {
namespace fs = drive_collect::pass_fs;
struct Options {
    fs::path outRoot, out;
    std::string label = "drive", who, weather, curtain, lights, props, note;
    int interval = kDefaultIntervalMs;
    bool reverseMotor = kReverseMotor, help = false;
};
inline Options parse(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        if (key == "--help") { o.help = true; continue; }
        if (key == "--reverse-motor") { o.reverseMotor = true; continue; }
        if (key != "--label" && key != "--out-root" && key != "--interval-ms" &&
            key != "--who" && key != "--weather" && key != "--curtain" &&
            key != "--lights" && key != "--props" && key != "--note")
            throw std::runtime_error("Tham số không biết: " + key);
        if (++i >= argc || std::string(argv[i]).compare(0, 2, "--") == 0)
            throw std::runtime_error("Thiếu giá trị: " + key);
        const std::string value = argv[i];
        if (key == "--interval-ms") {
            if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos)
                throw std::runtime_error("Nhịp ghi phải là số nguyên dương");
            const auto n = std::stoull(value);
            if (!n || n > static_cast<unsigned long long>(std::numeric_limits<int>::max()))
                throw std::runtime_error("Nhịp ghi ngoài khoảng cho phép");
            o.interval = static_cast<int>(n);
        } else if (key == "--out-root") {
            if (value.empty()) throw std::runtime_error("Thư mục ra rỗng");
            o.outRoot = value;
        } else if (key == "--label") o.label = value;
        else if (key == "--who") o.who = value;
        else if (key == "--weather") o.weather = value;
        else if (key == "--curtain") o.curtain = value;
        else if (key == "--lights") o.lights = value;
        else if (key == "--props") o.props = value;
        else o.note = value;
    }
    return o;
}
inline std::string jsonString(const std::string& value) {
    std::ostringstream out;
    out << '"';
    for (unsigned char c : value) {
        if (c == '"' || c == '\\') out << '\\' << static_cast<char>(c);
        else if (c < 32) out << "\\u00" << std::hex << std::setw(2) << std::setfill('0') << int(c);
        else out << static_cast<char>(c);
    }
    return out.str() + '"';
}
inline std::string safeLabel(const std::string& label) {
    std::string result;
    for (unsigned char c : label) {
        if ((c & 0xc0) == 0x80) continue;
        result += ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                   (c >= '0' && c <= '9') || c == '_' || c == '-') ? char(c) : '_';
    }
    return result.empty() ? "drive" : result;
}
} // namespace collect_drive
