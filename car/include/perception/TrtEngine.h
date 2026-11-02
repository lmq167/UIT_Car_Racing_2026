#pragma once
#include <memory>
#include <string>

namespace perception {
struct IInferEngine {
    virtual ~IInferEngine() = default;
    virtual void Infer(const float *input, float *output) = 0;
};

#ifdef UCR_WITH_TRT
struct InfoJson;
class TrtEngine : public IInferEngine {
public:
    explicit TrtEngine(const std::string &path, const InfoJson *info = nullptr);
    ~TrtEngine();
    void Infer(const float *input, float *output) override;
    float CopyMs() const;
    float InferMs() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
#endif
}
