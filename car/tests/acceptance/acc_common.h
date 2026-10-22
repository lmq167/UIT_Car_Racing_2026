// Tiện ích dùng chung cho các test chấp nhận (acceptance test) viết bằng C++.
// Không phụ thuộc cách cài đặt của mã nguồn xe; chỉ dùng thư viện chuẩn + POSIX.
#pragma once
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

namespace acc {

// Bộ đếm lỗi: mỗi CHECK sai in một dòng "HONG" rồi tiếp tục để thấy hết lỗi.
inline int &Failures() { static int n = 0; return n; }
inline int &Checks() { static int n = 0; return n; }

#define ACC_CHECK(cond, msg)                                                         \
    do {                                                                             \
        ++acc::Checks();                                                             \
        if (!(cond)) {                                                               \
            ++acc::Failures();                                                       \
            std::fprintf(stderr, "[HONG] %s:%d  %s  (dieu kien: %s)\n", __FILE__, __LINE__, \
                         std::string(msg).c_str(), #cond);                           \
        }                                                                            \
    } while (0)

inline int Finish(const char *name) {
    std::printf("[%s] %d phep kiem, %d loi\n", name, Checks(), Failures());
    return Failures() == 0 ? 0 : 1;
}

inline std::string MakeTempDir(const char *prefix) {
    const char *base = std::getenv("TMPDIR");
    std::string tmpl = std::string(base && *base ? base : "/tmp") + "/" + prefix + "XXXXXX";
    std::vector<char> buf(tmpl.begin(), tmpl.end());
    buf.push_back('\0');
    if (!mkdtemp(buf.data())) return "";
    return std::string(buf.data());
}

inline bool WriteFile(const std::string &path, const std::string &text) {
    std::ofstream f(path.c_str(), std::ios::binary);
    if (!f) return false;
    f << text;
    return static_cast<bool>(f);
}

inline std::string ReadFile(const std::string &path) {
    std::ifstream f(path.c_str(), std::ios::binary);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

inline bool Exists(const std::string &path) {
    struct stat st;
    return ::stat(path.c_str(), &st) == 0;
}

inline std::vector<std::string> ListDir(const std::string &dir) {
    std::vector<std::string> out;
    DIR *d = opendir(dir.c_str());
    if (!d) return out;
    while (dirent *e = readdir(d)) {
        std::string n = e->d_name;
        if (n != "." && n != "..") out.push_back(n);
    }
    closedir(d);
    return out;
}

// Tìm thư mục con có tên kết thúc bằng hậu tố (ví dụ "_auto") trong log_dir.
inline std::string FindRunDir(const std::string &log_dir, const std::string &suffix) {
    std::string found;
    for (const auto &n : ListDir(log_dir)) {
        if (n.size() >= suffix.size() && n.compare(n.size() - suffix.size(), suffix.size(), suffix) == 0)
            found = log_dir + "/" + n;  // lấy cái cuối cùng (tên có thời gian nên cái sau mới hơn)
    }
    return found;
}

inline std::vector<std::string> Split(const std::string &s, char sep) {
    std::vector<std::string> out;
    std::string cur;
    for (char ch : s) {
        if (ch == sep) { out.push_back(cur); cur.clear(); }
        else if (ch != '\r') cur.push_back(ch);
    }
    out.push_back(cur);
    return out;
}

// Bảng CSV đọc theo tên cột.
struct Csv {
    std::vector<std::string> header;
    std::vector<std::vector<std::string>> rows;
    int Col(const std::string &name) const {
        for (size_t i = 0; i < header.size(); ++i) if (header[i] == name) return static_cast<int>(i);
        return -1;
    }
    double Num(size_t row, const std::string &name) const {
        int c = Col(name);
        if (c < 0 || row >= rows.size() || static_cast<size_t>(c) >= rows[row].size()) return 0.0 / 0.0;
        return std::atof(rows[row][c].c_str());
    }
};

inline Csv ReadCsv(const std::string &path) {
    Csv csv;
    std::ifstream f(path.c_str());
    std::string line;
    bool first = true;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        auto cells = Split(line, ',');
        if (first) { csv.header = cells; first = false; }
        else if (cells.size() == csv.header.size()) csv.rows.push_back(cells);  // bỏ dòng cụt (ghi dở)
    }
    return csv;
}

inline void SleepMs(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

}  // namespace acc
