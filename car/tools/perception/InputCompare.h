#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace perception {
struct InputComparison {
    double max_abs=0;
    int max_levels=0;
    std::size_t different=0, over_one=0;
    bool passed=true;
};

inline InputComparison CompareInput(const float *actual, std::size_t actual_size,
                                    const float *reference, std::size_t reference_size) {
    if(actual_size!=reference_size || (actual_size && (!actual || !reference)))
        throw std::invalid_argument("Hai mảng đầu vào phải cùng kích thước và có dữ liệu");
    InputComparison result;
    for(std::size_t i=0;i<actual_size;++i) {
        const double a=actual[i], b=reference[i];
        if(!std::isfinite(a) || !std::isfinite(b) || a < -1e-6 || a > 1+1e-6 ||
           b < -1e-6 || b > 1+1e-6)
            throw std::invalid_argument("Đầu vào phải hữu hạn và chuẩn hoá trong [0,1]");
        const int levels=static_cast<int>(std::abs(std::round(a*255)-std::round(b*255)));
        result.max_abs=std::max(result.max_abs,std::abs(a-b));
        result.max_levels=std::max(result.max_levels,levels);
        result.different+=(levels>0);
        result.over_one+=(levels>1);
    }
    // Khôi phục mức uint8; dung sai chỉ bù sai số float32 ở biên 1/255.
    result.passed=result.max_levels<=1 && result.max_abs<=1.0/255+1e-6;
    return result;
}
}
