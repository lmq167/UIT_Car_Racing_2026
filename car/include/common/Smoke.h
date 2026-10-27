#pragma once
#include "common/Config.h"
#include <opencv2/core.hpp>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cstdio>
namespace common {
inline std::string ConfigText(const std::string &path) {
    std::ifstream in(path,std::ios::binary);
    if(!in) throw std::runtime_error("Không đọc được cấu hình smoke");
    std::ostringstream out; out<<in.rdbuf(); return out.str();
}
inline std::string SourceId() { return UCR_SOURCE_ID; }
inline int Simulated() {
#ifdef UCR_FAKE_HW
    return 1;
#else
    return 0;
#endif
}
inline void SmokeResult(const std::string &path,const std::string &config,bool camera,bool sensors,
        bool servo,bool tof,bool drop,bool moving,bool killed,bool stopped) {
    const auto tmp=path+".tmp";
    cv::FileStorage out(tmp,cv::FileStorage::WRITE|cv::FileStorage::FORMAT_JSON);
    if(!out.isOpened()) throw std::runtime_error("Không ghi được smoke.json");
    out<<"source_id"<<SourceId()<<"config_text"<<config<<"simulated"<<Simulated()
       <<"camera"<<int(camera)<<"sensors"<<int(sensors)<<"servo"<<int(servo)
       <<"tof_brake"<<int(tof)<<"camera_brake"<<int(drop)<<"encoder_moving"<<int(moving)
       <<"kill9"<<int(killed)<<"encoder_stopped"<<int(stopped)
       <<"passed"<<int(camera&&sensors&&servo&&tof&&drop&&moving&&killed&&stopped);
    out.release();
    if(std::rename(tmp.c_str(),path.c_str())!=0) throw std::runtime_error("Không hoàn tất smoke.json");
}
inline bool SmokePassed(const Config &c,const std::string &config) {
    try {
        cv::FileStorage in(c.log_dir+"/smoke_gate.json",cv::FileStorage::READ);
        if(!in.isOpened() || !in["simulated"].isInt() || int(in["simulated"])!=Simulated() ||
           !in["source_id"].isString() || std::string(in["source_id"])!=SourceId() ||
           !in["config_text"].isString() ||
           std::string(in["config_text"])!=(c.loaded_text.empty() ? ConfigText(config) : c.loaded_text)) return false;
        // Cờ tổng không thay thế bằng chứng của từng phép thử.
        for(const char *key:{"camera","sensors","servo","tof_brake","camera_brake",
                "encoder_moving","kill9","encoder_stopped","passed"})
            if(!in[key].isInt() || int(in[key])!=1) return false;
        return true;
    } catch(const std::exception &) { return false; }
}
}
