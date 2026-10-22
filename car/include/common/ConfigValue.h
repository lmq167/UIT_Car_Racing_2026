#pragma once
#include <opencv2/core.hpp>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>
namespace common {
inline bool HasConfigKey(const cv::FileNode &root,const char *name) {
    // Xét sự có mặt của tên khóa, không coi nút có giá trị rỗng là khóa thiếu.
    for(auto it=root.begin(); it!=root.end(); ++it)
        if((*it).name()==name) return true;
    return false;
}
inline std::string ConfigValueText(const cv::FileNode &node) {
    if(node.isString()) return "\""+static_cast<std::string>(node)+"\"";
    if(node.isInt() || node.isReal()) {
        std::ostringstream out; out.precision(17); out << static_cast<double>(node); return out.str();
    }
    if(node.isSeq() || node.isMap()) {
        std::string out=node.isSeq() ? "[" : "{";
        for(auto it=node.begin(); it!=node.end(); ++it) {
            if(it!=node.begin()) out+=", ";
            if(node.isMap()) out+=(*it).name()+": ";
            out+=ConfigValueText(*it);
        }
        return out+(node.isSeq() ? "]" : "}");
    }
    return "<rỗng>";
}
inline void BadConfigType(const char *name,const cv::FileNode &node) {
    throw std::runtime_error(std::string("Sai kiểu hoặc giá trị cấu hình: ")+name+" = "+ConfigValueText(node));
}
template<class T>
typename std::enable_if<std::is_arithmetic<T>::value>::type
ReadConfigValue(const char *name,const cv::FileNode &node,T &value) {
    if(!node.isInt() && !node.isReal()) BadConfigType(name,node);
    const double number=static_cast<double>(node);
    // Không cho phép OpenCV ép chữ, làm tròn số nguyên hoặc tràn kiểu đích.
    if(!std::isfinite(number) || number<std::numeric_limits<T>::lowest() ||
       number>std::numeric_limits<T>::max() || (std::is_integral<T>::value && std::trunc(number)!=number))
        BadConfigType(name,node);
    value=static_cast<T>(number);
}
inline void ReadConfigValue(const char *name,const cv::FileNode &node,std::string &value) {
    if(!node.isString()) BadConfigType(name,node);
    value=static_cast<std::string>(node);
}
inline void ReadConfigValue(const char *name,const cv::FileNode &node,std::vector<std::string> &value) {
    if(!node.isSeq()) BadConfigType(name,node);
    std::vector<std::string> parsed;
    for(auto it=node.begin(); it!=node.end(); ++it) {
        if(!(*it).isString()) BadConfigType(name,node);
        parsed.push_back(static_cast<std::string>(*it));
    }
    // Chỉ ghi kết quả sau khi mọi phần tử đã hợp lệ.
    value=std::move(parsed);
}
inline void ReadConfigFlag(const char *name,const cv::FileNode &node,int &value) {
    if(node.isString()) {
        const auto text=static_cast<std::string>(node);
        if(text!="true" && text!="false") BadConfigType(name,node);
        value=text=="true";
    } else {
        ReadConfigValue(name,node,value);
        if(value!=0 && value!=1) BadConfigType(name,node);
    }
}
}
