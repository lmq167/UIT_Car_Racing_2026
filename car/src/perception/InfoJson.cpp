#include "perception/InfoJson.h"
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace perception {
namespace {
void Require(bool ok, const std::string &key) {
    if(!ok) throw std::runtime_error("Metadata không hợp lệ: "+key);
}
std::string Text(const cv::FileNode &root, const char *key) {
    const auto node=root[key];
    Require(node.isString(),key);
    const auto value=std::string(node);
    Require(!value.empty(),key);
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
void FlagFalse(const cv::FileNode &root, const char *key) {
    const auto node=root[key];
    Require(node.isInt() && int(node)==0,key);
}
}

InfoJson::InfoJson(const std::string &path) {
    cv::FileStorage file(path,cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
    Require(file.isOpened(),path);
    const auto root=file.root();
    for(const auto &entry:std::vector<std::pair<const char*,const char*>>{
        {"task","segmentation"},{"color_order","RGB"},{"layout","NCHW"},
        {"input_dtype","float32"},{"output_type","logits"},
        {"activation","softmax_over_channels_argmax"},
        {"resize","cv2.resize INTER_AREA (crop -> input)"}})
        Require(Text(root,entry.first)==entry.second,entry.first);
    for(const char *key:{"fake_data","preprocessing_in_model","dynamic"}) FlagFalse(root,key);
    Require(root["num_classes"].isInt() && int(root["num_classes"])==3,"num_classes");
    const auto classes=root["class_names"];
    Require(classes.isSeq() && classes.size()==3,"class_names");
    int index=0;
    for(const char *name:{"background","track_surface","line"}) {
        Require(classes[index].isString() && std::string(classes[index])==name,"class_names");
        ++index;
    }
    const auto source_hw=Integers(root,"source_image_hw",2), hw=Integers(root,"input_hw",2);
    const auto y=Integers(root,"crop_y",2), x=Integers(root,"crop_x",2);
    Require(source_hw[0]>0 && source_hw[1]>0 && hw[0]>0 && hw[1]>0,"image_hw");
    Require(0<=y[0] && y[0]<y[1] && y[1]<=source_hw[0],"crop_y");
    Require(0<=x[0] && x[0]<x[1] && x[1]<=source_hw[1],"crop_x");
    source=cv::Size(source_hw[1],source_hw[0]); input=cv::Size(hw[1],hw[0]);
    crop=cv::Rect(x[0],y[0],x[1]-x[0],y[1]-y[0]);
    Require(size_t(hw[0])<=std::numeric_limits<size_t>::max()/size_t(hw[1])/3/sizeof(float),"input_hw quá lớn");
    const std::vector<int> shape{1,3,hw[0],hw[1]};
    Require(Integers(root,"input_shape",4)==shape,"input_shape");
    Require(Integers(root,"output_shape",4)==shape,"output_shape");
    input_name=Text(root,"input_name"); output_name=Text(root,"output_name");
    Require(input_name!=output_name,"tên input/output trùng nhau");
    const auto norm=root["normalization"];
    Require(norm.isMap(),"normalization");
    Require(norm["scale"].isReal() || norm["scale"].isInt(),"normalization.scale");
    scale=double(norm["scale"]);
    Require(std::isfinite(scale) && std::abs(scale-1.0/255)<1e-15,"normalization.scale");
    Require(Integers(norm,"mean",3)==std::vector<int>({0,0,0}),"normalization.mean");
    Require(Integers(norm,"std",3)==std::vector<int>({1,1,1}),"normalization.std");
}
}
