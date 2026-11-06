#include "perception/YoloDetector.h"
#define UCR_AUTO_TEST_HOOKS 1
#include "../tools/auto_drive/model_worker.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unistd.h>

namespace {
void Check(bool ok, const std::string &message) {
    if(!ok) throw std::runtime_error(message);
}
template<class F> void Reject(F action) {
    bool rejected=false;
    try { action(); } catch(const std::exception &) { rejected=true; }
    Check(rejected,"Phải từ chối dữ liệu sai");
}
std::string Metadata(int h=320,int w=320) {
    std::ostringstream out;
    out<<"{\"input_hw\":["<<h<<','<<w<<"],\"input_shape\":[1,3,"<<h<<','<<w<<"],";
    out<<R"json("task":"detection","class_names":["red","green","off","left","right","no_left","no_right","round_barrier","work_barrier"],
"num_classes":9,"input_name":"images","output_name":"output0","output_shape":[1,13,4],
"input_dtype":"float32","color_order":"RGB","layout":"NCHW","resize":"letterbox",
"normalization":{"scale":0.00392156862745098,"mean":[0,0,0],"std":[1,1,1]},
"letterbox":{"auto":false,"center":true,"padding_value":114,"scaleup":true,"stride":32},
"preprocessing_in_model":false,"dynamic":false,"opset":12,
"output_type":"cxcywh_pixels_and_class_probabilities","box_format":"cxcywh","box_units":"input_pixels",
"scores_are_probabilities":true,"objectness":false,"nms_in_model":false,"nms_class_aware":true,
"conf_threshold":0.25,"nms_iou":0.5})json";
    return out.str();
}
std::string Changed(std::string text,const std::string &before,const std::string &after) {
    const auto pos=text.find(before);
    Check(pos!=std::string::npos,"Không tìm thấy trường thử");
    text.replace(pos,before.size(),after);
    return text;
}
void Box(std::vector<float> &out,int i,float cx,float cy,float w,float h,int cls,float score) {
    out[i]=cx; out[4+i]=cy; out[8+i]=w; out[12+i]=h;
    out[(4+cls)*4+i]=score;
}
void Near(double a,double b) { Check(std::abs(a-b)<1e-5,"Giá trị số lệch tham chiếu"); }
struct FakeEngine : perception::IInferEngine {
    std::vector<float> result;
    bool called=false;
    void Infer(const float *input,float *output) override {
        Near(input[0],114.0/255);
        std::copy(result.begin(),result.end(),output);
        called=true;
    }
};
void WorkerShape(const perception::YoloInfo &info,int channels,bool accepted) {
    char path[]="ucr_yolo_worker_XXXXXX";
    const int fd=mkstemp(path);
    Check(fd>=0,"Không tạo được CSV tạm"); close(fd);
    const std::string tensor=std::string(path)+".npy";
    struct Cleanup {
        std::string csv,tensor;
        ~Cleanup() { std::remove(csv.c_str()); std::remove(tensor.c_str()); }
    } cleanup{path,tensor};
    { std::ofstream csv(path); csv<<"frame_id,tensor\n1,"<<tensor<<'\n';
      Check(bool(csv),"Không ghi được CSV tạm"); }
    std::string header="{'descr': '<f4', 'fortran_order': False, 'shape': (1, "+
        std::to_string(channels)+", "+std::to_string(info.candidates)+"), }";
    header.append((16-(10+header.size()+1)%16)%16,' '); header+='\n';
    const char length[]={static_cast<char>(header.size()%256),static_cast<char>(header.size()/256)};
    const std::vector<float> values(size_t(channels)*info.candidates,0);
    { std::ofstream npy(tensor,std::ios::binary);
      npy.write("\x93NUMPY\x01\x00",8); npy.write(length,2); npy<<header;
      npy.write(reinterpret_cast<const char*>(values.data()),values.size()*sizeof(float));
      Check(bool(npy),"Không ghi được tensor tạm"); }
    bool loaded=false;
    try {
        auto_drive::FakeYoloEngine engine(path,info);
        std::vector<float> output(info.OutputElements(),-1);
        engine.Infer(nullptr,output.data());
        Check(output==values,"Engine giả phải trả đúng tensor");
        loaded=true;
    } catch(const std::runtime_error &error) {
        if (accepted) throw;
        Check(std::string(error.what())=="Tensor phải là <f4 C-order [1,4+nc,N] khớp metadata",
              "Phải từ chối ngay tại kiểm tra shape");
    }
    Check(loaded==accepted,"Kết quả nạp tensor không khớp số lớp metadata");
}
}

int main() {
    try {
        char path[]="ucr_yolo_XXXXXX";
        const int fd=mkstemp(path);
        Check(fd>=0,"Không tạo được file tạm"); close(fd);
        struct Cleanup { const char *path; ~Cleanup() { std::remove(path); } } cleanup{path};
        auto load=[&](const std::string &text) {
            { std::ofstream file(path); file<<text; Check(bool(file),"Không ghi được metadata"); }
            return perception::YoloInfo(path);
        };
        const auto json=Metadata();
        const auto info=load(json);
        const perception::YoloDetector yolo(info);

        Check(info.num_classes==9 && info.OutputElements()==52,"Kích thước 9 lớp sai");
        auto json10=Changed(json,"\"work_barrier\"]","\"work_barrier\",\"parking\"]");
        json10=Changed(json10,"\"num_classes\":9","\"num_classes\":10");
        json10=Changed(json10,"[1,13,4]","[1,14,4]");
        const auto info10=load(json10);
        Check(info10.num_classes==10 && info10.OutputElements()==56,"Kích thước 10 lớp sai");
        WorkerShape(info,13,true);
        WorkerShape(info10,14,true);
        WorkerShape(info,14,false);
        WorkerShape(info10,13,false);
        const perception::YoloDetector yolo10(info10);
        std::vector<float> out10(info10.OutputElements(),0);
        Box(out10,0,50,105,50,30,9,0.9f);
        Box(out10,1,50,105,50,30,9,0.8f);
        Box(out10,2,50,105,50,30,8,0.7f);
        auto decoded10=yolo10.Decode(out10.data());
        Check(decoded10.size()==2 && decoded10[0].class_name=="parking" &&
              decoded10[1].class_name=="work_barrier","Giải mã/NMS lớp thứ 10 sai");
        auto swapped10=Changed(json10,"\"red\"","\"parking\"");
        swapped10=Changed(swapped10,"\"parking\"]","\"red\"]");
        const perception::YoloDetector reordered10(load(swapped10));
        Check(reordered10.Decode(out10.data())[0].class_name=="red","10 lớp phải ánh xạ theo tên");
        out10[(4+9)*4]=1.01f;
        Reject([&] { yolo10.Decode(out10.data()); });
        out10[(4+9)*4]=0.9f;
        FakeEngine fake10; fake10.result=out10;
        Check(yolo10.Infer(cv::Mat(720,1280,CV_8UC3,cv::Scalar(0,0,0)),fake10).size()==2 &&
              fake10.called,"Engine giả 10 lớp sai");
        auto d2=Changed(json10,"\"resize\":\"letterbox\"",
            "\"resize\":\"letterbox: cv2.resize INTER_AREA giữ tỉ lệ, đệm 114, căn giữa\"");
        d2=Changed(d2,"\"box_format\":\"cxcywh\",\"box_units\":\"input_pixels\",","");
        d2=Changed(d2,"\"scores_are_probabilities\":true,","");
        d2=Changed(d2,"\"letterbox\":{\"auto\":false,\"center\":true,\"padding_value\":114,\"scaleup\":true,\"stride\":32},",
            "\"letterbox_1280x720\":{\"scale\":0.325,\"resized_wh\":[416,234],\"pad_left\":0,\"pad_top\":11},");
        d2=Changed(d2,"[320,320]","[256,416]");
        d2=Changed(d2,"[1,3,320,320]","[1,3,256,416]");
        d2=Changed(d2,"[1,14,4]","[1,14,2184]");
        const auto d2info=load(d2);
        Check(d2info.OutputElements()==30576 && d2info.pad_top==11 &&
              d2info.resized==cv::Size(416,234),"Metadata dạng D2 sai");
        Reject([&] { load(Changed(d2,"\"pad_top\":11","\"pad_top\":12")); });
        for(const auto &metadata:{json,json10}) {
            for(const auto &bad:{std::string("0"),std::string("-1"),std::string("9.5"),
                                 std::string("\"10\""),std::string("11"),std::string("2147483647")})
                Reject([&] { load(Changed(metadata,metadata==json ? "\"num_classes\":9" : "\"num_classes\":10",
                    "\"num_classes\":"+bad)); });
            Reject([&] { load(Changed(metadata,"\"num_classes\":","\"missing_num_classes\":")); });
        }
        Reject([&] { load(Changed(json10,"[1,14,4]","[1,13,4]")); });
        Reject([&] { load(Changed(json10,"\"num_classes\":10","\"num_classes\":9")); });
        Reject([&] { load(Changed(json10,"\"parking\"","\"red\"")); });

        Near(info.letterbox_scale,0.25);
        Check(info.resized==cv::Size(320,180) && info.pad_left==0 && info.pad_right==0 &&
              info.pad_top==70 && info.pad_bottom==70,"Scale/pad 320 sai");
        // Sinh lại bằng Python/cv2: python round3_final/tools/test_make_fake_yolo.py LetterboxReferenceTest
        cv::Mat image(720,1280,CV_8UC3);
        for(int y=0;y<720;++y) for(int x=0;x<1280;++x)
            image.at<cv::Vec3b>(y,x)=cv::Vec3b((3*x+5*y)%251,(7*x+11*y)%253,(13*x+17*y)%255);
        const auto blob=yolo.Preprocess(image);
        Check(blob.dims==4 && blob.size[0]==1 && blob.size[1]==3 && blob.size[2]==320 &&
              blob.size[3]==320 && blob.type()==CV_32F && blob.isContinuous(),"Blob NCHW sai");
        struct Pixel { int y,x,r,g,b; };
        for(const auto &p:std::vector<Pixel>{{0,0,114,114,114},{69,319,114,114,114},
            {70,0,45,27,12},{70,1,97,55,24},{100,17,164,52,63},
            {160,160,205,118,218},{249,319,149,137,141},{319,319,114,114,114}}) {
            const size_t offset=size_t(p.y)*320+p.x;
            Near(blob.ptr<float>()[offset],p.r/255.0);
            Near(blob.ptr<float>()[320*320+offset],p.g/255.0);
            Near(blob.ptr<float>()[2*320*320+offset],p.b/255.0);
        }
        const auto odd=load(Metadata(320,321));
        Check(odd.resized==cv::Size(321,181) && odd.pad_top==69 && odd.pad_bottom==70,"Đệm lẻ sai");
        const auto tie_down=load(Metadata(320,328)), tie_up=load(Metadata(320,344));
        Check(tie_down.resized.height==184 && tie_up.resized.height==194,"Round nửa về chẵn sai");
        const auto horizontal=load(Metadata(180,321));
        Check(horizontal.pad_left==0 && horizontal.pad_right==1,"Đệm ngang lẻ sai");
        std::vector<float> out(info.OutputElements(),0);
        Box(out,0,50,105,50,30,0,0.9f);
        Box(out,1,50,105,50,30,0,0.8f);
        auto detections=yolo.Decode(out.data());
        Check(detections.size()==1 && detections[0].class_name=="red","NMS cùng lớp sai");
        const auto &d=detections[0];
        Near(d.box.x,100); Near(d.box.y,80); Near(d.box.width,200); Near(d.box.height,120); Near(d.confidence,0.9f);
        out[(4+0)*4+1]=0; out[(4+1)*4+1]=0.8f;
        detections=yolo.Decode(out.data());
        Check(detections.size()==2 && detections[1].class_name=="green","NMS khác lớp sai");
        const perception::YoloDetector reordered(load(Changed(json,"\"red\",\"green\"","\"green\",\"red\"")));
        Check(reordered.Decode(out.data())[0].class_name=="green","Phải ánh xạ theo tên/thứ tự file");
        FakeEngine fake; fake.result=out;
        Check(yolo.Infer(image,fake).size()==2 && fake.called,"Không gọi IInferEngine");
        out.assign(info.OutputElements(),0);
        Box(out,0,50,105,50,30,0,0.24f);
        Check(yolo.Decode(out.data()).empty(),"Dưới ngưỡng phải rỗng");
        out[16]=0.25f; out[20]=0.25f;
        detections=yolo.Decode(out.data());
        Check(detections.size()==1 && detections[0].class_name=="red","Ngưỡng bằng/hoà lớp sai");
        Box(out,1,51,105,50,30,0,0.25f);
        detections=yolo.Decode(out.data());
        Check(detections.size()==1,"NMS hoà điểm sai"); Near(detections[0].box.x,100);
        out.assign(info.OutputElements(),0);
        Box(out,0,50,105,50,30,0,0);
        const perception::YoloDetector zero_threshold(load(Changed(json,"\"conf_threshold\":0.25","\"conf_threshold\":0")));
        Check(zero_threshold.Decode(out.data()).size()==1,"Ngưỡng 0 phải giữ điểm 0");
        const perception::YoloDetector one_threshold(load(Changed(json,"\"conf_threshold\":0.25","\"conf_threshold\":1")));
        out[16]=1;
        Check(one_threshold.Decode(out.data()).size()==1,"Ngưỡng 1 phải giữ điểm 1");
        out.assign(info.OutputElements(),0);
        Box(out,0,0,82.5f,50,25,0,0.9f); Box(out,1,12.5f,82.5f,25,25,0,0.8f);
        detections=yolo.Decode(out.data());
        Check(detections.size()==2,"Phải NMS trước khi kẹp biên");
        Near(detections[0].box.x,0); Near(detections[0].box.width,100);
        out.assign(info.OutputElements(),0);
        Box(out,0,162.5f,162.5f,375,225,0,0.9f);
        detections=yolo.Decode(out.data());
        Check(detections.size()==1,"Mất hộp vượt biên");
        Near(detections[0].box.x,0); Near(detections[0].box.y,0);
        Near(detections[0].box.width,1280); Near(detections[0].box.height,720);
        for(float bad:{std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity()}) {
            out.back()=bad; Reject([&] { yolo.Decode(out.data()); }); out.back()=0;
            out[0]=bad; Reject([&] { yolo.Decode(out.data()); }); out[0]=0;
        }
        Reject([&] { yolo.Decode(nullptr); });
        Reject([&] { yolo.Preprocess(cv::Mat(719,1280,CV_8UC3)); });
        Reject([&] { yolo.Preprocess(cv::Mat(720,1280,CV_32FC3)); });
        out.assign(info.OutputElements(),0); out[16]=1.01f;
        Reject([&] { yolo.Decode(out.data()); });
        out[16]=0; out[8]=-1;
        Reject([&] { yolo.Decode(out.data()); });
        const auto with_transform=Changed(json,"\"task\":",R"json("letterbox_1280x720":{"scale":0.25,"resized_wh":[320,180],"pad_left":0,"pad_top":70},"task":)json");
        load(with_transform);
        Reject([&] { load(Changed(with_transform,"\"pad_top\":70","\"pad_top\":71")); });
        for(const auto &change:std::vector<std::pair<std::string,std::string>>{
            {"\"red\"","\"RED\""},{"\"green\"","\"red\""},
            {"\"detection\"","\"segmentation\""},{"[1,13,4]","[1,14,4]"},
            {"[1,13,4]","[1,13,0]"},{"[320,320]","[0,320]"},
            {"\"conf_threshold\":0.25","\"conf_threshold\":1.1"},
            {"\"nms_iou\":0.5","\"nms_iou\":-0.1"},
            {"\"padding_value\":114","\"padding_value\":0"},
            {"\"auto\":false","\"auto\":true"},{"\"center\":true","\"center\":false"},
            {"\"scaleup\":true","\"scaleup\":false"},{"\"stride\":32","\"stride\":16"},
            {"\"opset\":12","\"opset\":13"},
            {"\"objectness\":false","\"objectness\":true"},
            {"\"input_name\":\"images\"","\"input_name\":\"output0\""},
            {"\"input_name\":\"images\"","\"input_name\":42"},
            {"\"scale\":0.00392156862745098","\"scale\":1"}})
            Reject([&] { load(Changed(json,change.first,change.second)); });
        Reject([&] { load("{}"); });
        std::cout<<"ĐẠT: YOLO metadata, letterbox, decode, NMS, ảnh gốc, engine giả, NaN/Inf\n";
        return 0;
    } catch(const std::exception &error) {
        std::cerr<<"KHÔNG ĐẠT: "<<error.what()<<'\n'; return 1;
    }
}
