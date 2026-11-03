#pragma once
#include "hooks.h"
#include "options.h"
#include "perception/RoadSegmenter.h"
#include <algorithm>
#include <cstdint>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <vector>

namespace auto_drive {
#if defined(UCR_AUTO_TEST_HOOKS) && UCR_AUTO_TEST_HOOKS
// Dùng cùng Preprocess/Decode với xe, chỉ thay engine bằng logits từ PNG chỉ số lớp.
class FakeRoadEngine : public perception::IInferEngine {
    std::vector<std::pair<uint64_t,cv::Mat>> masks_;
    cv::Mat current_;
public:
    FakeRoadEngine(const std::string& path, const perception::InfoJson& info) {
        std::ifstream f(path); std::string line;
        if (!std::getline(f,line)) throw std::runtime_error("Thiếu tiêu đề ROAD_MASKS");
        if (!line.empty() && line.back()=='\r') line.pop_back();
        if (line!="frame_id,mask") throw std::runtime_error("Sai tiêu đề ROAD_MASKS");
        uint64_t last=0;
        while (std::getline(f,line)) {
            if (!line.empty() && line.back()=='\r') line.pop_back();
            const auto comma=line.find(',');
            if (comma==std::string::npos || comma==0 || line.find_first_not_of("0123456789",0)<comma)
                throw std::runtime_error("Sai frame_id ROAD_MASKS");
            const auto id=std::stoull(line.substr(0,comma));
            const auto name=line.substr(comma+1);
            if (id<=last || (!last && id!=1) || name.empty() || name[0]=='/' || name.find("..")!=std::string::npos)
                throw std::runtime_error("Sai thứ tự/đường dẫn ROAD_MASKS");
            auto mask=cv::imread(parentPath(path)+"/"+name,cv::IMREAD_UNCHANGED);
            if (mask.type()!=CV_8UC1 || mask.size()!=info.input) throw std::runtime_error("Sai PNG ROAD_MASKS");
            double min=0,max=0; cv::minMaxLoc(mask,&min,&max);
            if (min<0 || max>2) throw std::runtime_error("Lớp mask ngoài 0/1/2");
            masks_.emplace_back(id,mask); last=id;
        }
        if (f.bad() || masks_.empty()) throw std::runtime_error("ROAD_MASKS rỗng/lỗi đọc");
    }
    void select(uint64_t id) {
        for (const auto& sample:masks_) {
            if (sample.first>id) break;
            current_=sample.second;
        }
    }
    void Infer(const float*, float* out) override {
        const auto n=current_.total();
        std::fill(out,out+3*n,0.0f);
        for (size_t i=0;i<n;++i) out[current_.ptr<unsigned char>()[i]*n+i]=1.0f;
    }
};
#endif
struct Frame {
    cv::Mat bgr;
    uint64_t id=0;
    int64_t captured_ms=-1;
};
struct ModelResult {
    Frame frame;
    cv::Mat mask;
    double road_ms=0;
};
// Một hộp thư mới nhất, bên đọc/ghi thử đúng một lần; Mat đã công bố không được sửa.
template<class T> class Mailbox {
    std::mutex mutex_;
    T value_;
public:
    bool offer(const T& value) {
        std::unique_lock<std::mutex> lock(mutex_,std::try_to_lock);
        if (!lock.owns_lock()) return false;
        value_=value; return true;
    }
    bool latest(T& value) {
        std::unique_lock<std::mutex> lock(mutex_,std::try_to_lock);
        if (!lock.owns_lock()) return false;
        value=value_; return true;
    }
};
}
