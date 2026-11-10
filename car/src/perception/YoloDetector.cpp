#include "perception/YoloDetector.h"
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace perception {
namespace {
void Require(bool ok, const std::string &key) {
    if(!ok) throw std::runtime_error("YOLO không hợp lệ: "+key);
}
std::string Text(const cv::FileNode &root, const char *key) {
    const auto node=root[key];
    Require(node.isString() && !std::string(node).empty(),key);
    return std::string(node);
}
double Number(const cv::FileNode &root, const char *key) {
    const auto node=root[key];
    Require(node.isInt() || node.isReal(),key);
    const double value=double(node);
    Require(std::isfinite(value),key);
    return value;
}
std::vector<int> Integers(const cv::FileNode &root, const char *key, size_t count) {
    const auto node=root[key];
    Require(node.isSeq() && node.size()==count,key);
    std::vector<int> values;
    for(const auto &item:node) {
        Require(item.isInt(),key);
        values.push_back(int(item));
    }
    return values;
}
void Flag(const cv::FileNode &root, const char *key, bool value) {
    const auto node=root[key];
    Require(node.isInt() && int(node)==int(value),key);
}
int PythonRound(double value) {
    const double lower=std::floor(value), fraction=value-lower;
    return int(lower)+(fraction>0.5 || (fraction==0.5 && int(lower)%2!=0));
}
}

YoloInfo::YoloInfo(const std::string &path) {
    cv::FileStorage file(path,cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
    Require(file.isOpened(),path);
    const auto root=file.root();
    for(const auto &entry:std::vector<std::pair<const char*,const char*>>{
        {"task","detection"},{"color_order","RGB"},{"layout","NCHW"},
        {"input_dtype","float32"},
        {"output_type","cxcywh_pixels_and_class_probabilities"}})
        Require(Text(root,entry.first)==entry.second,entry.first);
    // D2 ghi hợp đồng tiền xử lý trong resize và letterbox_1280x720.
    const bool export_d2=Text(root,"resize")=="letterbox: cv2.resize INTER_AREA giữ tỉ lệ, đệm 114, căn giữa";
    Require(export_d2 || Text(root,"resize")=="letterbox","resize");
    for(const auto &entry:std::vector<std::pair<const char*,const char*>>{
        {"box_format","cxcywh"},{"box_units","input_pixels"}})
        if(!export_d2 || !root[entry.first].empty())
            Require(Text(root,entry.first)==entry.second,entry.first);
    for(const char *key:{"preprocessing_in_model","dynamic","objectness","nms_in_model"})
        Flag(root,key,false);
    Flag(root,"nms_class_aware",true);
    if(!export_d2 || !root["scores_are_probabilities"].empty()) Flag(root,"scores_are_probabilities",true);
    Require(root["num_classes"].isInt(),"num_classes");
    num_classes=int(root["num_classes"]);
    Require(num_classes==9 || num_classes==10 || num_classes==12,"num_classes phải là 9, 10 hoặc 12");
    std::set<std::string> expected{
        "red","green","off","left","right","no_left","no_right","round_barrier","work_barrier"};
    if(num_classes>=10) expected.insert("parking");
    if(num_classes==12) {
        expected.insert("people");
        expected.insert("obstacle");
    }
    const auto classes=root["class_names"];
    Require(classes.isSeq() && classes.size()==size_t(num_classes),"class_names");
    std::set<std::string> found;
    for(const auto &item:classes) {
        Require(item.isString(),"class_names");
        class_names.push_back(std::string(item));
        found.insert(class_names.back());
    }
    Require(found==expected,"class_names phải đủ tên lớp theo num_classes, không trùng");
    const auto hw=Integers(root,"input_hw",2);
    if(!root["source_image_hw"].empty())
        Require(Integers(root,"source_image_hw",2)==std::vector<int>({720,1280}),"source_image_hw");
    Require(hw[0]>0 && hw[1]>0,"input_hw");
    Require(size_t(hw[0])<=std::numeric_limits<size_t>::max()/size_t(hw[1])/3/sizeof(float),"input_hw quá lớn");
    input=cv::Size(hw[1],hw[0]);
    Require(Integers(root,"input_shape",4)==std::vector<int>({1,3,hw[0],hw[1]}),"input_shape");
    const auto shape=Integers(root,"output_shape",3);
    Require(shape[0]==1 && shape[1]==4+num_classes && shape[2]>0,"output_shape phải là [1,4+num_classes,N], N > 0");
    candidates=shape[2];
    Require(size_t(candidates)<=std::numeric_limits<size_t>::max()/size_t(4+num_classes)/sizeof(float),"output_shape quá lớn");
    input_name=Text(root,"input_name"); output_name=Text(root,"output_name");
    Require(input_name!=output_name,"tên binding trùng nhau");
    Require(root["opset"].isInt() && int(root["opset"])==12,"opset");
    const auto norm=root["normalization"];
    Require(norm.isMap() && std::abs(Number(norm,"scale")-1.0/255)<1e-15,"normalization.scale");
    Require(Integers(norm,"mean",3)==std::vector<int>({0,0,0}),"normalization.mean");
    Require(Integers(norm,"std",3)==std::vector<int>({1,1,1}),"normalization.std");
    const double conf=Number(root,"conf_threshold"), iou=Number(root,"nms_iou");
    Require(conf>=0 && conf<=1 && iou>=0 && iou<=1,"conf_threshold/nms_iou");
    conf_threshold=float(conf); nms_iou=float(iou);
    const auto lb=root["letterbox"];
    if(!export_d2 || !lb.empty()) {
        Require(lb.isMap(),"letterbox");
        Flag(lb,"auto",false); Flag(lb,"center",true); Flag(lb,"scaleup",true);
        Require(lb["padding_value"].isInt() && int(lb["padding_value"])==114,"letterbox.padding_value");
        Require(lb["stride"].isInt() && int(lb["stride"])==32,"letterbox.stride");
    }
    letterbox_scale=std::min(input.height/720.0,input.width/1280.0);
    resized=cv::Size(PythonRound(1280*letterbox_scale),PythonRound(720*letterbox_scale));
    Require(resized.width>0 && resized.height>0,"letterbox resized_wh");
    const double dx=(input.width-resized.width)/2.0, dy=(input.height-resized.height)/2.0;
    pad_left=PythonRound(dx-0.1); pad_right=PythonRound(dx+0.1);
    pad_top=PythonRound(dy-0.1); pad_bottom=PythonRound(dy+0.1);
    // Notebook không ghi phép đổi này; nếu file có ghi thì phải khớp phép tính chuẩn.
    const auto transform=root["letterbox_1280x720"];
    if(export_d2) Require(transform.isMap(),"letterbox_1280x720");
    if(!transform.empty()) {
        Require(transform.isMap(),"letterbox_1280x720");
        Require(std::abs(Number(transform,"scale")-letterbox_scale)<1e-12,"letterbox_1280x720.scale");
        Require(Integers(transform,"resized_wh",2)==std::vector<int>({resized.width,resized.height}),"letterbox_1280x720.resized_wh");
        Require(Number(transform,"pad_left")==pad_left && Number(transform,"pad_top")==pad_top,"letterbox_1280x720.pad");
    }
}

cv::Mat YoloDetector::Preprocess(const cv::Mat &image) const {
    Require(image.type()==CV_8UC3 && image.size()==cv::Size(1280,720),"ảnh phải là BGR uint8 1280×720");
    cv::Mat resized, padded, rgb, scaled;
    cv::resize(image,resized,info_.resized,0,0,cv::INTER_AREA);
    cv::copyMakeBorder(resized,padded,info_.pad_top,info_.pad_bottom,info_.pad_left,info_.pad_right,
                       cv::BORDER_CONSTANT,cv::Scalar(114,114,114));
    cv::cvtColor(padded,rgb,cv::COLOR_BGR2RGB);
    rgb.convertTo(scaled,CV_32F,1.0/255);
    const int dims[]={1,3,info_.input.height,info_.input.width};
    cv::Mat blob(4,dims,CV_32F);
    const size_t plane=blob.total()/3;
    std::vector<cv::Mat> channels;
    for(int c=0;c<3;++c) channels.emplace_back(info_.input,CV_32F,blob.ptr<float>()+c*plane);
    cv::split(scaled,channels);
    return blob;
}

std::vector<Detection> YoloDetector::Decode(const float *out) const {
    Require(out!=nullptr,"thiếu output");
    const size_t n=info_.candidates;
    for(size_t i=0;i<info_.OutputElements();++i) Require(std::isfinite(out[i]),"output chứa NaN/Inf");
    struct Candidate { int cls; float score; cv::Rect2d box; };
    std::vector<Candidate> candidates;
    for(size_t i=0;i<n;++i) {
        int best=0;
        for(int c=0;c<info_.num_classes;++c) {
            const float score=out[(4+c)*n+i];
            Require(score>=0 && score<=1,"xác suất ngoài [0,1]");
            if(score>out[(4+best)*n+i]) best=c;
        }
        const double w=out[2*n+i], h=out[3*n+i], r=info_.letterbox_scale;
        Require(w>=0 && h>=0,"kích thước hộp âm");
        const float score=out[(4+best)*n+i];
        if(score<info_.conf_threshold || w==0 || h==0) continue;
        candidates.push_back({best,score,cv::Rect2d((out[i]-w/2-info_.pad_left)/r,
            (out[n+i]-h/2-info_.pad_top)/r,w/r,h/r)});
    }
    std::stable_sort(candidates.begin(),candidates.end(),[](const Candidate &a,const Candidate &b) {
        return a.score>b.score;
    });
    std::vector<bool> kept(candidates.size(),false);
    for(int cls=0;cls<info_.num_classes;++cls) {
        std::vector<cv::Rect2d> boxes;
        std::vector<float> scores;
        std::vector<size_t> positions;
        for(size_t i=0;i<candidates.size();++i) if(candidates[i].cls==cls) {
            boxes.push_back(candidates[i].box);
            // NMSBoxes dùng > ngưỡng; giữ cả điểm 0 khi conf_threshold=0.
            scores.push_back(std::max(candidates[i].score,std::numeric_limits<float>::denorm_min()));
            positions.push_back(i);
        }
        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes,scores,0.0f,info_.nms_iou,indices);
        for(int index:indices) kept[positions[index]]=true;
    }
    std::vector<Detection> detections;
    for(size_t i=0;i<candidates.size();++i) if(kept[i]) {
        const auto &candidate=candidates[i];
        const auto &b=candidate.box;
        const double x1=std::max(0.0,std::min(1280.0,b.x)), y1=std::max(0.0,std::min(720.0,b.y));
        const double x2=std::max(0.0,std::min(1280.0,b.x+b.width)), y2=std::max(0.0,std::min(720.0,b.y+b.height));
        if(x2>x1 && y2>y1) detections.push_back({info_.class_names[candidate.cls],
            cv::Rect2f(float(x1),float(y1),float(x2-x1),float(y2-y1)),candidate.score});
    }
    return detections;
}

std::vector<Detection> YoloDetector::Infer(const cv::Mat &image, IInferEngine &engine) const {
    const auto blob=Preprocess(image);
    std::vector<float> output(info_.OutputElements());
    engine.Infer(blob.ptr<float>(),output.data());
    return Decode(output.data());
}
}
