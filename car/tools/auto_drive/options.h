#pragma once
#include "limits.h"
#include <cerrno>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

namespace auto_drive {
struct Options {
    std::string control, modelDir, engine, info, outRoot, yoloEngine, yoloInfo;
    std::string unetOnnxSha256, yoloOnnxSha256;
    bool noYolo=false, help=false, debugWindows=false;
};
inline Options parse(int argc, char** argv) {
    Options o;
    for (int i=1; i<argc; ++i) {
        const std::string key=argv[i];
        if (key=="--help") { o.help=true; continue; }
        if (key=="--no-yolo") { o.noYolo=true; continue; }
        if (key=="--debug-windows") { o.debugWindows=true; continue; }
        if (key!="--control" && key!="--model-dir" && key!="--out-root")
            throw std::runtime_error("Tham số không biết: "+key);
        if (++i>=argc || !*argv[i] || std::string(argv[i]).compare(0,2,"--")==0)
            throw std::runtime_error("Thiếu giá trị: "+key);
        if (key=="--control") o.control=argv[i];
        else if (key=="--model-dir") o.modelDir=argv[i];
        else o.outRoot=argv[i];
    }
    return o;
}
inline std::string parentPath(const std::string& p) {
    const auto n=p.find_last_of('/');
    return n==std::string::npos ? "." : n==0 ? "/" : p.substr(0,n);
}
inline bool regularFile(const std::string& p) {
    struct stat s{}; return stat(p.c_str(),&s)==0 && S_ISREG(s.st_mode);
}
inline std::string absolutePath(const std::string& p) {
    if (!p.empty() && p[0]=='/') return p;
    char cwd[4096];
    if (!getcwd(cwd,sizeof(cwd))) throw std::runtime_error("Không đọc được thư mục làm việc");
    return std::string(cwd)+"/"+p;
}
inline void makeDirs(const std::string& p) {
    if (p.empty() || p=="/" || p==".") return;
    struct stat s{};
    if (stat(p.c_str(),&s)==0) {
        if (!S_ISDIR(s.st_mode)) throw std::runtime_error("Không phải thư mục: "+p);
        return;
    }
    makeDirs(parentPath(p));
    if (mkdir(p.c_str(),0700)!=0 && errno!=EEXIST) throw std::runtime_error("Không tạo được thư mục: "+p);
}
inline std::string readBytes(const std::string& p) {
    std::ifstream f(p,std::ios::binary);
    if (!f) throw std::runtime_error("Không đọc được: "+p);
    std::ostringstream s; s<<f.rdbuf();
    if (f.bad()) throw std::runtime_error("Lỗi đọc: "+p);
    return s.str();
}
inline std::string jsonString(const std::string& value) {
    std::ostringstream out; out << '"';
    for (unsigned char c:value) {
        if (c=='"' || c=='\\') out << '\\' << char(c);
        else if (c<32) out << "\\u00" << std::hex << std::setw(2) << std::setfill('0') << int(c);
        else out << char(c);
    }
    return out.str()+'"';
}
inline std::string csvString(const std::string& value) {
    std::string out="\"";
    for (char c:value) { if (c=='"') out+='"'; out+=c; }
    return out+'"';
}
}
