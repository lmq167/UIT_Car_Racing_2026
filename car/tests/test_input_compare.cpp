#include "../tools/perception/InputCompare.h"
#include <iostream>
#include <limits>
#include <vector>

namespace {
void Check(bool ok, const char *message) {
    if(!ok) throw std::runtime_error(message);
}
template<class F> void Reject(F action) {
    bool rejected=false;
    try { action(); } catch(const std::invalid_argument &) { rejected=true; }
    Check(rejected,"Phải từ chối mảng sai");
}
perception::InputComparison Compare(const std::vector<float> &a, const std::vector<float> &b) {
    return perception::CompareInput(a.data(),a.size(),b.data(),b.size());
}
}

int main() {
    try {
        const float scale=1.0f/255;
        const auto equal=Compare({0,0.5f,1},{0,0.5f,1});
        Check(equal.passed && equal.max_abs==0 && equal.max_levels==0 &&
              equal.different==0 && equal.over_one==0,"Mảng giống nhau phải đạt");
        const auto boundary=Compare({0},{scale});
        Check(boundary.max_abs>1.0/255 && boundary.passed && boundary.max_levels==1 &&
              boundary.different==1 && boundary.over_one==0,"Biên float32 một mức phải đạt");
        Check(Compare({0},{scale+1e-7f}).passed,"Một mức cộng sai số float phải đạt");
        for(int level=0;level<255;++level) {
            const float a=static_cast<float>(level/255.0), b=(level+1)*scale;
            Check(Compare({a},{b}).passed && Compare({b},{a}).passed,
                  "Mọi cặp mức liền kề phải đạt theo cả hai chiều");
        }
        const auto two=Compare({0,scale,1},{0,3*scale,1-scale});
        Check(!two.passed && two.max_levels==2 && two.different==2 && two.over_one==1,
              "Hai mức phải hỏng và đếm đúng phần tử lệch");
        Check(!Compare({0},{scale+2e-6f}).passed,"Không vượt dung sai 1e-6 dù làm tròn còn một mức");
        const auto noise=Compare({0.5f},{0.5f+1e-7f});
        Check(noise.passed && noise.different==0,"Nhiễu cùng mức không tính là lệch mức");
        Reject([] { Compare({0},{0,1}); });
        Reject([] { perception::CompareInput(nullptr,1,nullptr,1); });
        for(float bad:{std::numeric_limits<float>::quiet_NaN(),
                       std::numeric_limits<float>::infinity(),-0.1f,1.1f}) {
            Reject([&] { Compare({bad},{0}); });
            Reject([&] { Compare({0},{bad}); });
        }
        std::cout<<"ĐẠT: so mức đầu vào, biên float32, bộ đếm, kích thước và dữ liệu sai\n";
        return 0;
    } catch(const std::exception &error) {
        std::cerr<<"KHÔNG ĐẠT: "<<error.what()<<'\n';
        return 1;
    }
}
