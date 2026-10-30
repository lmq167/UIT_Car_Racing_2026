#include "pass_store.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <utility>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace drive_collect {
namespace {
const char* const passHeader = "pass,event,ts_ms,frames,end_reason";
const char* const frameHeader = "frame,monotonic_ns,wall_time_unix_ms,filename,width,height,mean_luma,lap_var,dropped_since_last,cmd_speed,cmd_steer,state,pass,file";
const char* const sessionHeader = "ts_ms,git,argv,label,pipeline";

int64_t wallMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string csv(const std::string& value) {
    if (value.find_first_of(",\"\r\n") == std::string::npos) return value;
    std::string result = "\"";
    for (char ch : value) { if (ch == '"') result += '"'; result += ch; }
    return result + '"';
}

bool number(const std::string& value, uint64_t& result) {
    if (value.empty()) return false;
    result = 0;
    for (char ch : value) {
        if (ch < '0' || ch > '9' || result >
            (std::numeric_limits<uint64_t>::max() - (ch - '0')) / 10) return false;
        result = result * 10 + (ch - '0');
    }
    return true;
}

std::string readFile(const pass_fs::path& path) {
    std::ifstream input;
    input.exceptions(std::ios::badbit | std::ios::failbit);
    input.open(path.string(), std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

// Phân biệt đối số bằng độ dài, kể cả cờ có dấu cách hoặc xuống dòng.
std::string arguments(const std::vector<std::string>& argv) {
    std::string result;
    for (const auto& arg : argv) result += std::to_string(arg.size()) + ':' + arg;
    return result;
}
} // namespace

bool frameDue(bool stopped, int64_t elapsed_ms) {
    return !stopped || elapsed_ms >= 2000;
}

PassStore::PassStore(pass_fs::path outRoot, const SessionInfo& session)
    : root_(std::move(outRoot)) {
    try { initialize(session); }
    catch (const std::exception& error) { ioError(error.what()); }
}

StoreStatus PassStore::ioError(const std::string& message) {
    status_ = {StoreError::Io, message};
    return status_;
}

void PassStore::replace(const pass_fs::path& target, const std::string& contents) {
    const pass_fs::path temp = target.string() + ".tmp";
    std::ofstream output;
    output.exceptions(std::ios::badbit | std::ios::failbit);
    output.open(temp.string(), std::ios::binary | std::ios::trunc);
    output.write(contents.data(), static_cast<std::streamsize>(contents.size()));
    output.flush();
    output.close();
#ifdef _WIN32
    // rename của Windows không thay được tệp cũ; không xóa đích trước khi đổi tên.
    if (!MoveFileExW(temp.c_str(), target.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
        throw std::runtime_error("Không thay tệp " + target.string() +
                                 ": " + std::to_string(GetLastError()));
#else
    pass_fs::rename(temp, target);
#endif
}

void PassStore::append(const char* name, const char* header, const std::string& row) {
    const auto path = root_ / name;
    bool empty = true, torn = false;
    if (pass_fs::exists(path)) {
        std::ifstream input;
        input.exceptions(std::ios::badbit | std::ios::failbit);
        input.open(path.string(), std::ios::binary);
        input.seekg(0, std::ios::end);
        empty = input.tellg() == 0;
        if (!empty) {
            input.seekg(0);
            std::string first;
            std::getline(input, first);
            if (!first.empty() && first.back() == '\r') first.pop_back();
            if (first != header) throw std::runtime_error("Sai tiêu đề " + path.string());
            input.clear();
            input.seekg(-1, std::ios::end);
            char last = 0;
            input.get(last);
            torn = last != '\n';
        }
    }
    std::ofstream output;
    output.exceptions(std::ios::badbit | std::ios::failbit);
    output.open(path.string(), std::ios::binary | std::ios::app);
    if (empty) output << header << '\n';
    // Giữ nguyên mảnh cuối do kill, nhưng không nối dòng mới vào mảnh đó.
    else if (torn) output << '\n';
    output << row << '\n' << std::flush;
    output.close();
}

void PassStore::initialize(const SessionInfo& session) {
    pass_fs::create_directories(root_);
    uint64_t counter = 0, csvMax = 0, imageMax = 0, value = 0;
    if (pass_fs::exists(root_ / ".pass_counter")) {
        std::istringstream input(readFile(root_ / ".pass_counter"));
        std::string text;
        input >> text;
        if (number(text, value)) counter = value;
    }
    if (pass_fs::exists(root_ / "passes.csv")) {
        const auto text = readFile(root_ / "passes.csv");
        std::string first;
        bool quoted = false, firstColumn = true;
        for (size_t i = 0; i < text.size(); ++i) {
            const char ch = text[i];
            if (ch == '"') {
                if (quoted && i + 1 < text.size() && text[i + 1] == '"') ++i;
                else quoted = !quoted;
            } else if (!quoted && ch == ',') {
                if (firstColumn && number(first, value)) csvMax = std::max(csvMax, value);
                firstColumn = false;
            } else if (!quoted && ch == '\n') {
                first.clear(); firstColumn = true;
            } else if (firstColumn) first += ch;
        }
    }
    for (const auto& entry : pass_fs::directory_iterator(root_)) {
        const std::string name = entry.path().filename().string();
        const auto underscore = name.find('_');
        if (name.size() > 5 && name[0] == 'p' && underscore != std::string::npos &&
            name.compare(name.size() - 4, 4, ".jpg") == 0 &&
            pass_fs::is_regular_file(entry.path()) &&
            number(name.substr(1, underscore - 1), value)) imageMax = std::max(imageMax, value);
    }
    const uint64_t maximum = std::max({counter, csvMax, imageMax});
    if (maximum == std::numeric_limits<uint64_t>::max()) {
        status_ = {StoreError::Exhausted, "Đã hết số lượt"};
        return;
    }
    next_ = maximum + 1;
    append("sessions.csv", sessionHeader,
           std::to_string(session.ts_ms < 0 ? wallMs() : session.ts_ms) + ',' +
           csv(session.git) + ',' + csv(arguments(session.argv)) + ',' +
           csv(session.label) + ',' + csv(session.pipeline));
}

void PassStore::event(const char* kind, int64_t ts_ms, const std::string& reason) {
    append("passes.csv", passHeader, std::to_string(pass_) + ',' + kind + ',' +
           std::to_string(ts_ms < 0 ? wallMs() : ts_ms) + ',' + std::to_string(frames_) + ',' + csv(reason));
}

StoreStatus PassStore::beginPass(int64_t ts_ms) {
    if (!status_) return status_;
    if (active_) return {StoreError::InvalidState, "Lượt trước chưa kết thúc"};
    if (next_ == 0) return {StoreError::Exhausted, "Đã hết số lượt"};
    try {
        // Giữ số trước khi ghi start; kill giữa các bước chỉ tạo khoảng trống số.
        replace(root_ / ".pass_counter", std::to_string(next_) + '\n');
        pass_ = next_;
        next_ = next_ == std::numeric_limits<uint64_t>::max() ? 0 : next_ + 1;
        frames_ = 0;
        lastFile_.clear();
        event("start", ts_ms, "");
        active_ = true;
        return {};
    } catch (const std::exception& error) { return ioError("Mở lượt: " + std::string(error.what())); }
}

StoreStatus PassStore::saveFrame(const std::vector<unsigned char>& jpeg, int64_t ts_ms,
                                const CaptureColumns& capture) {
    if (!status_) return status_;
    if (!active_) return {StoreError::InvalidState, "Chưa mở lượt"};
    if (jpeg.empty() || capture.width <= 0 || capture.height <= 0 ||
        !std::isfinite(capture.mean_luma) || !std::isfinite(capture.lap_var) ||
        !std::isfinite(capture.cmd_speed))
        return {StoreError::InvalidFrame, "JPEG rỗng hoặc cột capture không hợp lệ"};
    if (frames_ == std::numeric_limits<uint64_t>::max())
        return {StoreError::Exhausted, "Đã hết số ảnh trong lượt"};
    try {
        std::ostringstream name;
        name.imbue(std::locale::classic());
        name << 'p' << std::setw(4) << std::setfill('0') << pass_ << '_'
             << std::setw(4) << frames_ + 1 << ".jpg";
        const auto target = root_ / name.str();
        if (pass_fs::exists(target)) throw std::runtime_error("Ảnh đã tồn tại: " + target.string());
        replace(target, std::string(reinterpret_cast<const char*>(jpeg.data()), jpeg.size()));
        ++frames_;
        lastFile_ = name.str();
        std::ostringstream row;
        row.imbue(std::locale::classic());
        row << std::setprecision(12) << capture.frame << ',' << capture.monotonic_ns << ','
            << ts_ms << ',' << lastFile_ << ',' << capture.width << ',' << capture.height << ','
            << capture.mean_luma << ',' << capture.lap_var << ',' << capture.dropped_since_last
            << ',' << capture.cmd_speed << ',' << capture.cmd_steer << ',' << csv(capture.state)
            << ',' << pass_ << ',' << lastFile_;
        append("frames.csv", frameHeader, row.str());
        return {};
    } catch (const std::exception& error) { return ioError("Ghi ảnh/frames.csv: " + std::string(error.what())); }
}

StoreStatus PassStore::endPass(const std::string& reason, int64_t ts_ms) {
    if (!status_) return status_;
    if (!active_) return {StoreError::InvalidState, "Không có lượt đang mở"};
    try {
        event("end", ts_ms, reason);
        active_ = false;
        return {};
    } catch (const std::exception& error) { return ioError("Kết thúc lượt: " + std::string(error.what())); }
}

} // namespace drive_collect
