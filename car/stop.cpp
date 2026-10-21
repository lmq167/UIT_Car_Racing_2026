#include "app/Backend.h"
#include <opencv2/core.hpp>
#include <iostream>
#include <stdexcept>
#include <string>
int main(int argc,char **argv) {
    std::string path="../config.yaml";
    for(int i=1;i<argc;++i) {
        if(std::string(argv[i])!="--config" || i+1>=argc) return 2;
        path=argv[++i];
    }
    common::Config config; bool loaded=true;
    try {
        cv::FileStorage fs(path,cv::FileStorage::READ);
        if(!fs.isOpened()) throw std::runtime_error("Khong mo duoc config dung khan");
        if(fs["servo_trim_deg"].empty()) std::cerr << "Thieu servo_trim_deg, dung 0\n";
        else fs["servo_trim_deg"] >> config.servo_trim_deg;
        if(config.servo_trim_deg < -60 || config.servo_trim_deg > 60) {
            config.servo_trim_deg=0;
            throw std::runtime_error("Trim ngoai gioi han, dung 0");
        }
    }
    catch(const std::exception &e) { std::cerr << e.what() << "; dung trim mac dinh de thu dung motor\n"; loaded=false; }
    try {
        app::Backend backend(config);
        const bool stopped=backend.EmergencyOpenAndStop();
        return stopped&&loaded ? 0 : 1;
    } catch(const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
