// Tiền điều kiện cho các test auto khác: một kết quả smoke giả đầy đủ, đúng nguồn/config hiện tại.
// Chỉ dùng trong test; các ca kiểm chính gate phải tự dựng kết quả lỗi của mình.
#pragma once
#include "common/Smoke.h"
#include <stdexcept>
namespace acc {
inline void SeedGate(const common::Config &c,const std::string &config) {
#ifndef UCR_FAKE_HW
    (void)c; (void)config;
    throw std::runtime_error("Chỉ được dựng kết quả smoke giả ở UCR_FAKE_HW=ON");
#else
    if(c.tof_mapping_confirmed!=1) throw std::runtime_error("Mẫu smoke ĐẠT cần ánh xạ nguyên 1");
    common::SmokeResult(c.log_dir+"/smoke_gate.json",c.loaded_text.empty() ? common::ConfigText(config) : c.loaded_text,
                        true,true,true,true,true,true,true,true);
#endif
}
}
