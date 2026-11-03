#include "perception/RoadSegmenter.h"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <vector>

namespace {
void Check(bool ok, const std::string &message) {
    if(!ok) throw std::runtime_error(message);
}
template<class F> void Reject(F action) {
    bool rejected=false;
    try { action(); } catch(const std::exception &) { rejected=true; }
    Check(rejected,"Phải từ chối đầu vào sai");
}
const std::string json=R"json({
"task":"segmentation","class_names":["background","track_surface","line"],"num_classes":3,
"source_image_hw":[7,11],"crop_y":[2,7],"crop_x":[1,10],"input_hw":[3,4],
"input_shape":[1,3,3,4],"output_shape":[1,3,3,4],"input_name":"images","output_name":"logits",
"input_dtype":"float32","color_order":"RGB","layout":"NCHW","output_type":"logits",
"activation":"softmax_over_channels_argmax","resize":"cv2.resize INTER_AREA (crop -> input)",
"normalization":{"scale":0.00392156862745098,"mean":[0,0,0],"std":[1,1,1]},
"preprocessing_in_model":false,"dynamic":false,"fake_data":false})json";
struct FakeEngine : perception::IInferEngine {
    void Infer(const float *input, float *output) override {
        for(int i=0;i<36;++i) output[i]=input[i];
    }
};
}

int main() {
    try {
        char path[]="/tmp/ucr_perception_XXXXXX";
        const int fd=mkstemp(path);
        Check(fd>=0,"Không tạo được file tạm"); close(fd);
        struct Cleanup { const char *path; ~Cleanup() { std::remove(path); } } cleanup{path};
        auto load=[&](const std::string &text, bool allowFakeData=false) {
            { std::ofstream out(path); out<<text; Check(bool(out),"Không ghi được metadata"); }
            return perception::InfoJson(path,allowFakeData);
        };
        const auto info=load(json);
        perception::RoadSegmenter road(info);
        cv::Mat image(7,11,CV_8UC3);
        for(int y=0;y<7;++y) for(int x=0;x<11;++x)
            for(int c=0;c<3;++c) image.at<cv::Vec3b>(y,x)[c]=static_cast<unsigned char>((y*11+x)*3+c);
        const auto blob=road.Preprocess(image);
        Check(blob.dims==4 && blob.size[0]==1 && blob.size[1]==3 && blob.size[2]==3 && blob.size[3]==4 &&
              blob.type()==CV_32F && blob.isContinuous(),"Blob phải là float32 NCHW");
        // Giá trị uint8 INTER_AREA đối chiếu độc lập bằng trung bình diện tích rồi làm tròn.
        for(int y=0;y<3;++y) for(int x=0;x<4;++x) for(int c=0;c<3;++c) {
            double sum=0;
            const double top=2+y*5.0/3, bottom=2+(y+1)*5.0/3;
            const double left=1+x*9.0/4, right=1+(x+1)*9.0/4;
            for(int sy=2;sy<7;++sy) for(int sx=1;sx<10;++sx) {
                const double dy=std::max(0.0,std::min(bottom,double(sy+1))-std::max(top,double(sy)));
                const double dx=std::max(0.0,std::min(right,double(sx+1))-std::max(left,double(sx)));
                sum+=dy*dx*image.at<cv::Vec3b>(sy,sx)[2-c];
            }
            const double expected=std::round(sum/((bottom-top)*(right-left)))/255;
            Check(std::abs(blob.ptr<float>()[c*12+y*4+x]-expected)<1e-6,"Crop/RGB/uint8/255 sai");
        }
        cv::Mat constant(7,11,CV_8UC3,cv::Scalar(3,27,251));
        const auto flat=road.Preprocess(constant);
        Check(std::abs(flat.ptr<float>()[0]-251.0/255)<1e-6 &&
              std::abs(flat.ptr<float>()[12]-27.0/255)<1e-6 &&
              std::abs(flat.ptr<float>()[24]-3.0/255)<1e-6,"Thứ tự RGB sai");
        std::vector<float> logits(36,-2);
        logits[0]=5; logits[12+1]=5; logits[24+2]=5;
        const auto mask=road.Decode(logits.data());
        Check(mask.at<unsigned char>(0,0)==0 && mask.at<unsigned char>(0,1)==1 &&
              mask.at<unsigned char>(0,2)==2 && mask.at<unsigned char>(0,3)==0,"Argmax hoặc hòa điểm sai");
        cv::Mat all_line(info.input,CV_8U,cv::Scalar(2));
        const auto full=road.Restore(all_line);
        Check(full.size()==info.source && full.type()==CV_8U,"Cỡ mask gốc sai");
        for(int y=0;y<full.rows;++y) for(int x=0;x<full.cols;++x)
            Check(full.at<unsigned char>(y,x)==(info.crop.contains(cv::Point(x,y)) ? 2 : 0),"Ngoài crop phải bằng 0");
        FakeEngine fake;
        Check(cv::countNonZero(road.Infer(constant,fake))==0,"Engine giả phải chọn kênh đỏ ở lớp 0");
        Reject([&] { road.Preprocess(cv::Mat(6,11,CV_8UC3)); });
        Reject([&] { road.Preprocess(cv::Mat(7,11,CV_32FC3)); });
        Reject([&] { road.Decode(nullptr); });
        Reject([&] { road.Restore(cv::Mat(1,1,CV_8U)); });
        logits[0]=std::numeric_limits<float>::infinity();
        Reject([&] { road.Decode(logits.data()); });
        for(const auto &change:std::vector<std::pair<std::string,std::string>>{
            {"\"fake_data\":false","\"fake_data\":true"}, {"\"fake_data\":false","\"other\":false"},
            {"\"fake_data\":false","\"fake_data\":2"}, {"\"fake_data\":false","\"fake_data\":\"true\""},
            {"\"dynamic\":false","\"dynamic\":true"}, {"\"preprocessing_in_model\":false","\"preprocessing_in_model\":true"},
            {"\"input_hw\":[3,4]","\"input_hw\":[0,4]"}, {"\"crop_y\":[2,7]","\"crop_y\":[2,8]"},
            {"\"crop_x\":[1,10]","\"crop_x\":[10,1]"}, {"\"color_order\":\"RGB\"","\"color_order\":\"BGR\""},
            {"\"output_shape\":[1,3,3,4]","\"output_shape\":[1,1,3,4]"},
            {"\"scale\":0.00392156862745098","\"scale\":1"},
            {"\"input_name\":\"images\"","\"input_name\":42"}}) {
            auto broken=json;
            const auto pos=broken.find(change.first);
            Check(pos!=std::string::npos,"Không tìm thấy trường thử");
            broken.replace(pos,change.first.size(),change.second);
            Reject([&] { load(broken); });
            if(change.second=="\"fake_data\":true")
                Check(load(broken,true).input==info.input,"Hooks phải nhận fake_data=true qua cùng schema");
            else Reject([&] { load(broken,true); });
        }
        Reject([&] { load("{}"); });
        Reject([&] { load("{}",true); });
        std::cout<<"ĐẠT: metadata, crop, RGB, uint8/255, NCHW, argmax, mask gốc, engine giả\n";
        return 0;
    } catch(const std::exception &error) {
        std::cerr<<"KHÔNG ĐẠT: "<<error.what()<<'\n'; return 1;
    }
}
