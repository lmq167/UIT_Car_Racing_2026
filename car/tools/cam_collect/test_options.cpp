#include "options.h"
#include <iostream>
#include <vector>

void require(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}

Options arguments(std::vector<std::string> values) {
    values.insert(values.begin(), "ucr_collect_cam");
    std::vector<char*> argv;
    for (auto& value : values) argv.push_back(value.data());
    return parse(static_cast<int>(argv.size()), argv.data());
}

int main() {
    try {
        const auto defaults = arguments({});
        require(defaults.interval == 200 && defaults.maxFrames == 0 && defaults.quality == 90 && defaults.minFree == 1024,
                "Sai giá trị mặc định");
        require(defaults.pipeline.find("width=640,height=384,format=BGR") != std::string::npos, "Sai cỡ mặc định");
        require(defaults.out.string().find("cam_") != std::string::npos, "Thiếu tên theo giờ");
        require(pipelineForCapture(" videotestsrc ! appsink sync=false ") == "videotestsrc ! appsink sync=false",
                "Pipeline hoàn chỉnh bị đổi");
        require(pipelineForCapture("videotestsrc ! video/x-raw,width=80,height=60").find("width=80,height=60") != std::string::npos,
                "Mất cỡ nguồn");
        const auto rebuilt = pipelineForCapture("gst-launch-1.0 -e -v nvarguscamerasrc sensor-mode=5 ! video/x-raw(memory:NVMM),width=1080,height=720 ! nv3dsink sync=false");
        require(rebuilt.find("gst-launch") == std::string::npos && rebuilt.find("nv3dsink") == std::string::npos &&
                rebuilt.find("nvvidconv") != std::string::npos && rebuilt.find("appsink") != std::string::npos,
                "Không dựng được đuôi appsink");
        const auto converted = pipelineForCapture("nvarguscamerasrc ! nvvidconv ! nvoverlaysink");
        require(converted.find("nvvidconv", converted.find("nvvidconv") + 1) == std::string::npos, "Lặp bộ đổi NVMM");
        const auto custom = arguments({"--max-frames", "12", "--interval-ms", "0", "--jpeg-quality", "100", "--min-free-mb", "0"});
        require(custom.maxFrames == 12 && custom.interval == 0 && custom.quality == 100 && custom.minFree == 0,
                "Không nhận tham số hợp lệ");
        const std::vector<std::vector<std::string>> invalid = {
            {"--out-dir"}, {"--width", "640"}, {"--height", "384"}, {"--unknown"},
            {"--jpeg-quality", "0"}, {"--jpeg-quality", "101"}, {"--interval-ms", "2147483648"},
            {"--max-frames", "-1"}, {"--max-frames", "1x"}, {"--min-free-mb", "18446744073709551616"},
            {"--pipeline", ""}, {"--pipeline", "appsink ! videoconvert"}, {"--pipeline", "fakesink"},
            {"--source", "__ucr_missing_source_4a33__"}
        };
        for (const auto& values : invalid) {
            bool rejected = false;
            try { arguments(values); } catch (const std::exception&) { rejected = true; }
            require(rejected, "Chấp nhận tham số sai");
        }
        std::cout << "PASS: mặc định, dựng pipeline, tham số hợp lệ và 14 đầu vào sai\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
