#include "perception/RoadSegmenter.h"
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace {
using Clock=std::chrono::steady_clock;
double Ms(Clock::time_point start) {
    return std::chrono::duration<double,std::milli>(Clock::now()-start).count();
}
double Report(const char *name, std::vector<double> samples) {
    const double mean=std::accumulate(samples.begin(),samples.end(),0.0)/samples.size();
    std::sort(samples.begin(),samples.end());
    const size_t p95=(95*samples.size()+99)/100-1;
    std::cout<<name<<": mean_ms="<<mean<<" p95_ms="<<samples[p95]<<'\n';
    return mean;
}
void Ram(const char *stage) {
    std::cout<<"RAM "<<stage<<" (KiB):"<<std::endl;
    if(std::system("free -k")!=0) throw std::runtime_error("Không chạy được free -k");
}
}

int main(int argc, char **argv) {
    try {
        std::map<std::string,std::string> options;
        for(int i=1;i<argc;i+=2) {
            const std::string key=argv[i];
            if((key!="--engine" && key!="--info" && key!="--iterations" && key!="--engine2") ||
               i+1==argc || std::string(argv[i+1]).empty() || std::string(argv[i+1]).find("--")==0 ||
               !options.emplace(key,argv[i+1]).second)
                throw std::runtime_error("Tham số không hợp lệ: "+key);
        }
        for(const char *key:{"--engine","--info"})
            if(!options.count(key)) throw std::runtime_error(std::string("Thiếu tham số ")+key);
        const auto count=options.count("--iterations") ? options.at("--iterations") : "200";
        if(count.find_first_not_of("0123456789")!=std::string::npos)
            throw std::runtime_error("iterations phải là số nguyên dương");
        const unsigned long n=std::stoul(count);
        if(n==0 || n>1000000) throw std::runtime_error("iterations phải trong [1,1000000]");
        const perception::InfoJson info(options.at("--info"));
        perception::RoadSegmenter road(info);
        const cv::Mat image(info.source,CV_8UC3,cv::Scalar(0,0,0));
        std::cout<<"Ảnh tổng hợp theo source_image_hw chỉ dùng đo thời gian; không đánh giá nhận dạng.\n";
        Ram("trước khi nạp");
        perception::TrtEngine engine(options.at("--engine"),&info);
        Ram("sau engine đường");
        std::unique_ptr<perception::TrtEngine> second;
        if(options.count("--engine2")) {
            second.reset(new perception::TrtEngine(options.at("--engine2")));
            Ram("sau hai engine");
        }
        std::vector<float> logits(info.Elements());
        std::vector<double> copy, pre, infer, post;
        for(unsigned long i=0;i<n+20;++i) {
            auto start=Clock::now();
            const auto blob=road.Preprocess(image);
            const double pre_ms=Ms(start);
            engine.Infer(blob.ptr<float>(),logits.data());
            start=Clock::now();
            const auto mask=road.Restore(road.Decode(logits.data()));
            const double post_ms=Ms(start);
            if(mask.empty()) throw std::runtime_error("Mask rỗng");
            if(i>=20) {
                pre.push_back(pre_ms); post.push_back(post_ms);
                copy.push_back(engine.CopyMs()); infer.push_back(engine.InferMs());
            }
        }
        std::cout<<"Số lượt đo="<<n<<", bỏ 20 lượt đầu; CUDA event tách H2D+D2H và enqueueV2\n";
        Report("chép dữ liệu",copy); Report("tiền xử lý",pre);
        const double mean=Report("suy luận",infer);
        Report("hậu xử lý",post);
        std::cout<<"Mốc nanounet_96x160 trtexec=6.765 ms; lệch_ms="<<mean-6.765
                 <<" lệch_%="<<100*(mean/6.765-1)<<'\n';
        std::cout<<"Mốc chỉ phù hợp nanounet_96x160; cần đối chiếu tải máy, xung, nhiệt và cách đo nếu lệch lớn.\n";
        return 0;
    } catch(const std::exception &error) {
        std::cerr<<"Lỗi: "<<error.what()<<"\nCách dùng: trt_bench --engine FILE --info FILE "
                   "[--iterations 200] [--engine2 FILE]\n";
        return 2;
    }
}
