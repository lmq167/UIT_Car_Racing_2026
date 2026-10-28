#ifndef UCR_CAM_COLLECT_OPTIONS_H
#define UCR_CAM_COLLECT_OPTIONS_H
#include <chrono>
#include <cstdint>
#include <ctime>
#if defined(_GLIBCXX_RELEASE) && _GLIBCXX_RELEASE < 8
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace fs = std::filesystem;
#endif
#include <iomanip>
#include <limits>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace {
// Chuỗi camera_pipeline trong config.yaml, giữ đúng ảnh dùng khi tự lái.
const char* defaultPipeline =
    "nvarguscamerasrc sensor-mode=5 wbmode=0 ! "
    "video/x-raw(memory:NVMM),width=1080,height=720,framerate=60/1 ! "
    "nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! "
    "video/x-raw,width=640,height=384,format=BGR ! appsink drop=true max-buffers=1";

struct Options {
    fs::path out, source;
    std::string pipeline = defaultPipeline;
    uint64_t interval = 200, maxFrames = 0, minFree = 1024;
    int quality = 90;
};

std::string trim(const std::string& s) {
    const auto first = s.find_first_not_of(" \t\r\n");
    return first == std::string::npos ? "" : s.substr(first, s.find_last_not_of(" \t\r\n") - first + 1);
}

std::string pipelineForCapture(std::string pipeline) {
    pipeline = trim(pipeline);
    pipeline = std::regex_replace(pipeline, std::regex("^gst-launch(-1\\.0)?\\s+(-[evq]\\s+)*"), "");
    if (pipeline.empty()) throw std::runtime_error("Chuỗi pipeline rỗng");
    const auto bang = pipeline.find_last_of('!');
    const auto tail = trim(pipeline.substr(bang == std::string::npos ? 0 : bang + 1));
    if (std::regex_match(tail, std::regex("appsink(\\s+.*)?"))) return pipeline;
    if (pipeline.find("appsink") != std::string::npos)
        throw std::runtime_error("appsink phải nằm ở cuối pipeline: " + pipeline);
    if (std::regex_match(tail, std::regex("(nv3dsink|nvoverlaysink|autovideosink|xvimagesink|ximagesink|fakesink)(\\s+.*)?"))) {
        if (bang == std::string::npos) throw std::runtime_error("Pipeline thiếu nguồn: " + pipeline);
        pipeline = trim(pipeline.substr(0, bang));
    }
    if (pipeline.find("nvarguscamerasrc") != std::string::npos && pipeline.find("videoconvert") == std::string::npos) {
        if (pipeline.find("nvvidconv") == std::string::npos) pipeline += " ! nvvidconv";
        pipeline += " ! video/x-raw,format=BGRx";
    }
    pipeline += " ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false";
    return pipeline;
}

uint64_t number(const std::string& text, const std::string& name) {
    if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos)
        throw std::runtime_error("Giá trị phải là số nguyên không âm: " + name);
    try { return std::stoull(text); }
    catch (const std::exception&) { throw std::runtime_error("Giá trị quá lớn: " + name); }
}

Options parse(int argc, char** argv) {
    Options o;
    bool customPipeline = false;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        if (key != "--out-dir" && key != "--source" && key != "--pipeline" && key != "--interval-ms" &&
            key != "--max-frames" && key != "--jpeg-quality" && key != "--min-free-mb")
            throw std::runtime_error("Tham số không biết: " + key);
        if (++i == argc || std::string(argv[i]).compare(0, 2, "--") == 0)
            throw std::runtime_error("Thiếu giá trị tham số: " + key);
        const std::string value = argv[i];
        if (value.empty()) throw std::runtime_error("Giá trị rỗng: " + key);
        if (key == "--out-dir") o.out = value;
        else if (key == "--source") o.source = value;
        else if (key == "--pipeline") { o.pipeline = value; customPipeline = true; }
        else if (key == "--interval-ms") o.interval = number(value, key);
        else if (key == "--max-frames") o.maxFrames = number(value, key);
        else if (key == "--min-free-mb") o.minFree = number(value, key);
        else {
            const auto q = number(value, key);
            if (q < 1 || q > 100) throw std::runtime_error("--jpeg-quality phải từ 1 đến 100");
            o.quality = static_cast<int>(q);
        }
    }
    if (o.interval > static_cast<uint64_t>(std::numeric_limits<int>::max()))
        throw std::runtime_error("--interval-ms quá lớn");
    if (customPipeline && !o.source.empty()) throw std::runtime_error("Chỉ chọn --source hoặc --pipeline");
    if (!o.source.empty() && !fs::exists(o.source)) throw std::runtime_error("Nguồn không tồn tại: " + o.source.string());
    o.pipeline = pipelineForCapture(o.pipeline);
    if (o.out.empty()) {
        const auto now = std::chrono::system_clock::now();
        const auto t = std::chrono::system_clock::to_time_t(now);
        std::ostringstream name;
        name << "cam_" << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S") << '_' << getpid();
        o.out = fs::path("runs") / name.str();
    }
    return o;
}

}
#endif
