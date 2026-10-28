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
        require(defaults.interval == 1000 && defaults.maxFrames == 0 && defaults.quality == 90 && defaults.minFree == 1024 &&
                defaults.warmup == 15 && defaults.startDelay == 0 && defaults.maxSeconds == 0,
                "Sai giá trị mặc định");
        require(defaults.pipeline.find("width=640,height=384,format=BGR") != std::string::npos, "Sai cỡ mặc định");
        require(defaults.out.parent_path() == fs::path("runs_cam") &&
                std::regex_match(defaults.out.filename().string(), std::regex("[0-9]{8}_[0-9]{6}_cam(_[0-9]+)?")),
                "Thiếu tên theo giờ");
        require(safeLabel("L1 a/..\\x") == "L1_a____x", "Tên đợt chưa được làm sạch");
        require(safeLabel("đợt L1") == "__t_L1", "Tên UTF-8 sai");
        require(jsonString("Việt\"\\\n\t\r\001") == "\"Việt\\\"\\\\\\u000a\\u0009\\u000d\\u0001\"", "Thoát JSON sai");
        require(jsonString(std::string("\0\b\f", 3)) == "\"\\u0000\\u0008\\u000c\"", "Thoát ký tự điều khiển sai");
        const auto metadata = arguments({"--out-root", "cam test", "--label", "L1/a", "--who", "Người thu",
            "--weather", "nắng", "--curtain", "mở", "--lights", "tắt", "--props", "biển", "--note", "",
            "--start-delay-s", "2", "--max-seconds", "7", "--warmup-frames", "3"});
        require(metadata.out.parent_path() == fs::path("cam test") && metadata.label == "L1/a" &&
                metadata.who == "Người thu" && metadata.weather == "nắng" && metadata.curtain == "mở" &&
                metadata.lights == "tắt" && metadata.props == "biển" && metadata.note.empty() &&
                metadata.startDelay == 2 && metadata.maxSeconds == 7 && metadata.warmup == 3,
                "Sai tham số phiên");
        const auto explicitOut = arguments({"--out-root", "ignored", "--out-dir", "__ucr_new_output_4a33__"});
        require(explicitOut.explicitOut && explicitOut.out == fs::path("__ucr_new_output_4a33__"), "Sai ưu tiên out-dir");
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
            {"--source", "__ucr_missing_source_4a33__"},
            {"--max-seconds", "2147483648"}, {"--start-delay-s", "2147483648"},
            {"--source", ".", "--pipeline", "videotestsrc ! appsink"},
            {"--warmup-frames", "-1"}, {"--out-dir", "."}, {"--note"}
        };
        for (const auto& values : invalid) {
            bool rejected = false;
            try { arguments(values); } catch (const std::exception&) { rejected = true; }
            require(rejected, "Chấp nhận tham số sai");
        }
        std::cout << "PASS: mặc định, pipeline, siêu dữ liệu, JSON và đầu vào sai\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
