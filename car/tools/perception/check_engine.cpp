#include "perception/RoadSegmenter.h"
#include "InputCompare.h"
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

#if __has_include(<filesystem>)
#include <filesystem>
namespace fs=std::filesystem;
#else
#include <experimental/filesystem>
namespace fs=std::experimental::filesystem;
#endif
namespace {
std::vector<float> ReadNpy(const fs::path &path, const perception::InfoJson &info, int channels) {
    std::ifstream file(path.string(),std::ios::binary);
    auto fail=[&]() { throw std::runtime_error("NPY v1 <f4 NCHW không hợp lệ: "+path.string()); };
    unsigned char prefix[10]{};
    if(!file.read(reinterpret_cast<char*>(prefix),10) || std::memcmp(prefix,"\x93NUMPY",6)!=0 ||
       prefix[6]!=1 || prefix[7]!=0) fail();
    const size_t header_len=size_t(prefix[8])+size_t(prefix[9])*256;
    if(header_len==0 || (10+header_len)%16!=0) fail();
    std::string header(header_len,'\0');
    if(!file.read(&header[0],header_len) || header.back()!='\n') fail();
    std::smatch dims;
    if(!std::regex_match(header,dims,std::regex(
           "\\s*\\{\\s*'descr'\\s*:\\s*'<f4'\\s*,\\s*"
           "'fortran_order'\\s*:\\s*False\\s*,\\s*"
           "'shape'\\s*:\\s*\\(\\s*1\\s*,\\s*([1-9][0-9]*)\\s*,\\s*([1-9][0-9]*)\\s*,\\s*([1-9][0-9]*)\\s*,?\\s*\\)"
           "\\s*,?\\s*\\}\\s*")) ||
       std::stoull(dims[1].str())!=size_t(channels) ||
       std::stoull(dims[2].str())!=size_t(info.input.height) ||
       std::stoull(dims[3].str())!=size_t(info.input.width)) fail();
    const auto payload_pos=file.tellg();
    file.seekg(0,std::ios::end);
    const auto end=file.tellg();
    const size_t elements=size_t(info.input.height)*info.input.width*channels;
    const size_t bytes=elements*sizeof(float);
    if(bytes>size_t(std::numeric_limits<std::streamsize>::max())) fail();
    if(end<payload_pos || static_cast<uint64_t>(end-payload_pos)!=bytes) fail();
    file.seekg(payload_pos);
    const uint16_t endian=1;
    if(*reinterpret_cast<const unsigned char*>(&endian)!=1 || sizeof(float)!=4) fail();
    std::vector<float> data(elements);
    if(!file.read(reinterpret_cast<char*>(data.data()),bytes)) fail();
    for(float value:data) if(!std::isfinite(value)) fail();
    return data;
}

std::map<std::string,std::string> Options(int argc, char **argv) {
    std::map<std::string,std::string> options;
    for(int i=1;i<argc;) {
        const std::string key=argv[i];
        if(key=="--raw-input") {
            if(!options.emplace(key,"").second)
                throw std::runtime_error("Tham số lặp: "+key);
            ++i;
            continue;
        }
        if((key!="--engine" && key!="--info" && key!="--ref") || i+1==argc ||
           std::string(argv[i+1]).empty() || std::string(argv[i+1]).find("--")==0 ||
           !options.emplace(key,argv[i+1]).second)
            throw std::runtime_error("Tham số không hợp lệ: "+key);
        i+=2;
    }
    for(const char *key:{"--engine","--info","--ref"})
        if(!options.count(key)) throw std::runtime_error(std::string("Thiếu tham số ")+key);
    return options;
}
}

int main(int argc, char **argv) {
    try {
        const auto options=Options(argc,argv);
        const bool raw_input=options.count("--raw-input")!=0;
        const perception::InfoJson info(options.at("--info"));
        perception::RoadSegmenter road(info);
        perception::TrtEngine engine(options.at("--engine"),&info);
        const fs::path ref(options.at("--ref"));
        std::ifstream list((ref/"list.txt").string());
        if(!list) throw std::runtime_error("Thiếu list.txt trong thư mục tham chiếu");
        std::vector<std::string> images;
        std::set<std::string> seen;
        std::string line;
        while(std::getline(list,line)) {
            std::istringstream row(line);
            std::string image, pass, labels, extra;
            if(!(row>>image>>pass>>labels) || row>>extra || fs::path(image).filename()!=fs::path(image) ||
               image=="." || image==".." || image.find('\\')!=std::string::npos ||
               pass.find_first_not_of("0123456789")!=std::string::npos || !seen.insert(image).second)
                throw std::runtime_error("Dòng list.txt không hợp lệ: "+line);
            if(labels!="-") {
                std::istringstream tags(labels);
                std::string tag;
                std::set<std::string> unique;
                if(labels.back()==',') throw std::runtime_error("Nhãn rỗng: "+line);
                while(std::getline(tags,tag,','))
                    if((tag!="ham" && tag!="cau" && tag!="giao_lo" && tag!="val") || !unique.insert(tag).second)
                        throw std::runtime_error("Nhãn không hợp lệ: "+line);
            }
            images.push_back(image);
        }
        if(list.bad() || images.size()<20) throw std::runtime_error("Cần ít nhất 20 ảnh trong list.txt");
        const auto masks=ref/"engine_masks";
        fs::create_directories(masks);
        bool passed=true;
        std::cout << "anh\tinput_max_abs\targmax_match\tline_iou\tlogits_max_abs\tket_qua"
                     "\tinput_max_levels\tinput_diff_gt0\tinput_diff_gt1\n";
        for(size_t index=0;index<images.size();++index) {
            std::ostringstream stem;
            stem<<std::setw(3)<<std::setfill('0')<<index;
            const auto input_ref=ReadNpy(ref/(stem.str()+"_input.npy"),info,3);
            const auto logits_ref=ReadNpy(ref/(stem.str()+"_logits.npy"),info,info.num_classes);
            cv::Mat blob;
            const float *input=input_ref.data();
            size_t input_size=input_ref.size();
            if(!raw_input) {
                blob=road.Preprocess(cv::imread((ref/images[index]).string(),cv::IMREAD_COLOR));
                input=blob.ptr<float>();
                input_size=blob.total();
            }
            const auto comparison=perception::CompareInput(input,input_size,input_ref.data(),input_ref.size());
            double logits_error=0;
            std::vector<float> logits(info.OutputElements());
            engine.Infer(input,logits.data());
            const auto mask=road.Decode(logits.data()), reference=road.Decode(logits_ref.data());
            size_t same=0, intersection=0, union_count=0;
            for(size_t i=0;i<mask.total();++i) {
                const auto a=mask.ptr<unsigned char>()[i], b=reference.ptr<unsigned char>()[i];
                same+=(a==b);
                if(info.num_classes==3) {
                    int predicted=0, expected=0;
                    for(int c=1;c<info.num_classes;++c) {
                        if(logits[c*mask.total()+i]>logits[predicted*mask.total()+i]) predicted=c;
                        if(logits_ref[c*mask.total()+i]>logits_ref[expected*mask.total()+i]) expected=c;
                    }
                    intersection+=(predicted==2 && expected==2);
                    union_count+=(predicted==2 || expected==2);
                }
            }
            for(size_t i=0;i<logits.size();++i)
                logits_error=std::max(logits_error,std::abs(double(logits[i])-logits_ref[i]));
            const double match=double(same)/mask.total();
            const bool ok=match>=0.995 && comparison.passed;
            passed=passed && ok;
            if(!cv::imwrite((masks/(stem.str()+"_mask.png")).string(),road.Restore(mask)))
                throw std::runtime_error("Không ghi được mask PNG");
            std::cout<<images[index]<<'\t'<<comparison.max_abs<<'\t'<<match<<'\t';
            if(info.num_classes==3) std::cout<<(union_count ? double(intersection)/union_count : 1);
            else std::cout<<"NA";
            std::cout<<'\t'<<logits_error<<'\t'<<(ok ? "ĐẠT" : "KHÔNG ĐẠT")<<'\t'
                     <<comparison.max_levels<<'\t'<<comparison.different<<'\t'<<comparison.over_one<<'\n';
        }
        std::cout<<(passed ? "ĐẠT" : "KHÔNG ĐẠT")<<": "<<images.size()<<" ảnh\n";
        return passed ? 0 : 1;
    } catch(const std::exception &error) {
        std::cerr<<"Lỗi: "<<error.what()<<"\nCách dùng: check_engine --engine FILE --info FILE --ref DIR [--raw-input]\n";
        return 2;
    }
}
