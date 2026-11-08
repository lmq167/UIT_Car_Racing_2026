#include "perception/TrtEngine.h"
#include "perception/InfoJson.h"
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <vector>

namespace perception {
namespace {
void Cuda(cudaError_t result) {
    if(result!=cudaSuccess) throw std::runtime_error(std::string("CUDA: ")+cudaGetErrorString(result));
}
class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char *message) noexcept override {
        if(severity<=Severity::kWARNING) std::cerr << "TensorRT: " << message << '\n';
    }
};
}

struct TrtEngine::Impl {
    Logger logger;
    nvinfer1::IRuntime *runtime=nullptr;
    nvinfer1::ICudaEngine *engine=nullptr;
    nvinfer1::IExecutionContext *context=nullptr;
    cudaStream_t stream=nullptr;
    cudaEvent_t events[4]{};
    void *bindings[2]{};
    size_t bytes[2]{};
    int input=-1, output=-1;
    float copy_ms=0, infer_ms=0;
    ~Impl() {
        if(stream) cudaStreamSynchronize(stream);
        for(auto buffer:bindings) if(buffer) cudaFree(buffer);
        for(auto event:events) if(event) cudaEventDestroy(event);
        if(stream) cudaStreamDestroy(stream);
        if(context) context->destroy();
        if(engine) engine->destroy();
        if(runtime) runtime->destroy();
    }
};

TrtEngine::TrtEngine(const std::string &path, const InfoJson *info) : impl_(new Impl) {
    auto &p=*impl_;
    std::ifstream file(path,std::ios::binary);
    if(!file) throw std::runtime_error("Không đọc được engine: "+path);
    const std::vector<char> data((std::istreambuf_iterator<char>(file)),std::istreambuf_iterator<char>());
    if(file.bad() || data.empty()) throw std::runtime_error("Engine rỗng hoặc đọc lỗi: "+path);
    p.runtime=nvinfer1::createInferRuntime(p.logger);
    if(!p.runtime) throw std::runtime_error("Không tạo được TensorRT runtime");
    p.engine=p.runtime->deserializeCudaEngine(data.data(),data.size());
    if(!p.engine) throw std::runtime_error("Không giải tuần tự được engine");
    if(p.engine->hasImplicitBatchDimension() || p.engine->getNbBindings()!=2)
        throw std::runtime_error("Engine phải có explicit batch, một input và một output");
    for(int i=0;i<2;++i) {
        if(p.engine->bindingIsInput(i)) {
            if(p.input!=-1) throw std::runtime_error("Engine có nhiều input");
            p.input=i;
        } else {
            if(p.output!=-1) throw std::runtime_error("Engine có nhiều output");
            p.output=i;
        }
        if(p.engine->getBindingDataType(i)!=nvinfer1::DataType::kFLOAT ||
           p.engine->getBindingFormat(i)!=nvinfer1::TensorFormat::kLINEAR ||
           !p.engine->isExecutionBinding(i) || p.engine->isShapeBinding(i))
            throw std::runtime_error("Binding phải là tensor FP32 tuyến tính");
        const auto dims=p.engine->getBindingDimensions(i);
        if(dims.nbDims<1 || dims.d[0]!=1) throw std::runtime_error("Binding phải có batch cố định bằng 1");
        size_t count=1;
        for(int j=0;j<dims.nbDims;++j) {
            if(dims.d[j]<=0 || count>std::numeric_limits<size_t>::max()/size_t(dims.d[j])/sizeof(float))
                throw std::runtime_error("Binding động hoặc kích thước quá lớn");
            count*=size_t(dims.d[j]);
        }
        p.bytes[i]=count*sizeof(float);
        if(info && (dims.nbDims!=4 || dims.d[1]!=(i==p.input ? 3 : info->num_classes) || dims.d[2]!=info->input.height || dims.d[3]!=info->input.width))
            throw std::runtime_error("Shape engine không khớp metadata");
    }
    if(p.input<0 || p.output<0) throw std::runtime_error("Thiếu binding input/output");
    if(info && (p.engine->getBindingIndex(info->input_name.c_str())!=p.input ||
                p.engine->getBindingIndex(info->output_name.c_str())!=p.output))
        throw std::runtime_error("Tên binding không khớp metadata");
    p.context=p.engine->createExecutionContext();
    if(!p.context) throw std::runtime_error("Không tạo được execution context");
    Cuda(cudaStreamCreate(&p.stream));
    for(auto &event:p.events) Cuda(cudaEventCreate(&event));
    for(int i=0;i<2;++i) Cuda(cudaMalloc(&p.bindings[i],p.bytes[i]));
}

TrtEngine::~TrtEngine() = default;

void TrtEngine::Infer(const float *input, float *output) {
    if(!input || !output) throw std::runtime_error("Thiếu buffer input/output");
    auto &p=*impl_;
    Cuda(cudaEventRecord(p.events[0],p.stream));
    Cuda(cudaMemcpyAsync(p.bindings[p.input],input,p.bytes[p.input],cudaMemcpyHostToDevice,p.stream));
    Cuda(cudaEventRecord(p.events[1],p.stream));
    if(!p.context->enqueueV2(p.bindings,p.stream,nullptr)) throw std::runtime_error("enqueueV2 thất bại");
    Cuda(cudaEventRecord(p.events[2],p.stream));
    Cuda(cudaMemcpyAsync(output,p.bindings[p.output],p.bytes[p.output],cudaMemcpyDeviceToHost,p.stream));
    Cuda(cudaEventRecord(p.events[3],p.stream));
    Cuda(cudaStreamSynchronize(p.stream));
    float h2d=0, d2h=0;
    Cuda(cudaEventElapsedTime(&h2d,p.events[0],p.events[1]));
    Cuda(cudaEventElapsedTime(&p.infer_ms,p.events[1],p.events[2]));
    Cuda(cudaEventElapsedTime(&d2h,p.events[2],p.events[3]));
    p.copy_ms=h2d+d2h;
}
float TrtEngine::CopyMs() const { return impl_->copy_ms; }
float TrtEngine::InferMs() const { return impl_->infer_ms; }
}
