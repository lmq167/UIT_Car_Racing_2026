#pragma once
#include "hooks.h"
#include "options.h"
#include "perception/RoadSegmenter.h"
#include "perception/YoloDetector.h"
#include "control/Detections.h"
#include <cstring>
#include <regex>
#if !defined(UCR_AUTO_TEST_HOOKS) || !UCR_AUTO_TEST_HOOKS
#include <NvInfer.h>
#endif
#include <algorithm>
#include <cstdint>
#include <mutex>
#include <memory>
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
// Tensor giả vẫn qua toàn bộ Preprocess/Decode/NMS thật.
class FakeYoloEngine : public perception::IInferEngine {
    std::vector<std::pair<uint64_t,std::vector<float>>> samples_;
    size_t selected_=0;
public:
    FakeYoloEngine(const std::string& path,const perception::YoloInfo& info) {
        std::ifstream f(path); std::string line;
        if (!std::getline(f,line)) throw std::runtime_error("Thiếu tiêu đề YOLO_TENSORS");
        if (!line.empty() && line.back()=='\r') line.pop_back();
        if (line!="frame_id,tensor") throw std::runtime_error("Sai tiêu đề YOLO_TENSORS");
        uint64_t last=0;
        while (std::getline(f,line)) {
            if (!line.empty() && line.back()=='\r') line.pop_back();
            const auto comma=line.find(',');
            if (comma==std::string::npos || comma==0 || line.substr(0,comma).find_first_not_of("0123456789")!=std::string::npos)
                throw std::runtime_error("Sai frame_id YOLO_TENSORS");
            const auto id=std::stoull(line.substr(0,comma));
            const auto name=line.substr(comma+1);
            if (id<=last || (!last && id!=1) || name.empty() || name[0]=='/' || name.find("..")!=std::string::npos ||
                name.find(':')!=std::string::npos || name.find('\\')!=std::string::npos)
                throw std::runtime_error("Sai thứ tự/đường dẫn YOLO_TENSORS");
            const auto bytes=readBytes(parentPath(path)+"/"+name);
            if (bytes.size()<10 || bytes.compare(0,8,std::string("\x93NUMPY\x01\x00",8))!=0)
                throw std::runtime_error("Tensor phải là .npy v1");
            const size_t header=static_cast<unsigned char>(bytes[8])+256*static_cast<unsigned char>(bytes[9]);
            if (!header || header>bytes.size()-10) throw std::runtime_error("Header .npy bị cắt");
            const auto meta=bytes.substr(10,header);
            const auto has=[&](const std::string& pattern) { return std::regex_search(meta,std::regex(pattern)); };
            if (meta.back()!='\n' || !has("['\"]descr['\"]\\s*:\\s*['\"]<f4['\"]") ||
                !has("['\"]fortran_order['\"]\\s*:\\s*False") ||
                !has("['\"]shape['\"]\\s*:\\s*\\(\\s*1\\s*,\\s*"+std::to_string(4+info.num_classes)+"\\s*,\\s*"+
                     std::to_string(info.candidates)+"\\s*,?\\s*\\)"))
                throw std::runtime_error("Tensor phải là <f4 C-order [1,4+nc,N] khớp metadata");
            if (bytes.size()-10-header!=info.OutputElements()*sizeof(float)) throw std::runtime_error("Sai số byte tensor");
            std::vector<float> values(info.OutputElements());
            std::memcpy(values.data(),bytes.data()+10+header,values.size()*sizeof(float));
            perception::YoloDetector(info).Decode(values.data());
            samples_.emplace_back(id,std::move(values)); last=id;
        }
        if (f.bad() || samples_.empty()) throw std::runtime_error("YOLO_TENSORS rỗng/lỗi đọc");
    }
    void select(uint64_t id) {
        while (selected_+1<samples_.size() && samples_[selected_+1].first<=id) ++selected_;
    }
    void Infer(const float*,float* output) override {
        const auto& values=samples_[selected_].second;
        std::copy(values.begin(),values.end(),output);
    }
};
#else
// TrtEngine dùng kích thước binding khi copy: kiểm khớp YOLO trước khi giao buffer.
inline void validateYoloEngine(const std::string& path,const perception::YoloInfo& info) {
    struct Logger : nvinfer1::ILogger { void log(Severity,const char*) noexcept override {} } logger;
    const auto destroyRuntime=[](nvinfer1::IRuntime* p) { if(p) p->destroy(); };
    const auto destroyEngine=[](nvinfer1::ICudaEngine* p) { if(p) p->destroy(); };
    std::unique_ptr<nvinfer1::IRuntime,decltype(destroyRuntime)> runtime(nvinfer1::createInferRuntime(logger),destroyRuntime);
    const auto bytes=readBytes(path);
    if (!runtime || bytes.empty()) throw std::runtime_error("Không nạp được engine YOLO");
    std::unique_ptr<nvinfer1::ICudaEngine,decltype(destroyEngine)> engine(
        runtime->deserializeCudaEngine(bytes.data(),bytes.size()),destroyEngine);
    if (!engine || engine->hasImplicitBatchDimension() || engine->getNbBindings()!=2)
        throw std::runtime_error("Engine YOLO phải có explicit batch, hai binding");
    const int in=engine->getBindingIndex(info.input_name.c_str()),out=engine->getBindingIndex(info.output_name.c_str());
    if (in<0 || out<0 || in==out || !engine->bindingIsInput(in) || engine->bindingIsInput(out))
        throw std::runtime_error("Binding YOLO không khớp metadata");
    for (int i=0;i<2;++i) {
        const auto d=engine->getBindingDimensions(i);
        const bool input=i==in;
        if (engine->getBindingDataType(i)!=nvinfer1::DataType::kFLOAT ||
            engine->getBindingFormat(i)!=nvinfer1::TensorFormat::kLINEAR || engine->isShapeBinding(i) ||
            !engine->isExecutionBinding(i) || d.nbDims!=(input ? 4 : 3) || d.d[0]!=1 ||
            d.d[1]!=(input ? 3 : 4+info.num_classes) || d.d[2]!=(input ? info.input.height : info.candidates) ||
            (input && d.d[3]!=info.input.width)) throw std::runtime_error("Shape/kiểu engine YOLO không khớp metadata");
    }
}
#endif
struct YoloResult {
    control::Detections detections;
    double yolo_ms=0;
};
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
