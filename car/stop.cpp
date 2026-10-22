#include "app/Backend.h"
#include "common/ConfigValue.h"
#include <opencv2/core.hpp>
#include <iostream>
#include <stdexcept>
#include <string>
int main(int argc,char **argv) {
    std::string path="../config.yaml";
    bool invalid_args=false;
    for(int i=1;i<argc;++i) {
        if(std::string(argv[i])!="--config" || i+1>=argc ||
           std::string(argv[i+1]).empty() || argv[i+1][0]=='-') {
            invalid_args=true;
            break;
        }
        path=argv[++i];
    }
    if(invalid_args) {
        // Bỏ cả đường dẫn đã đọc trước lỗi, vẫn tiếp tục đường dừng khẩn.
        path="../config.yaml";
        std::cerr << "Tham số không hợp lệ. Cách dùng: ucr_stop [--config <file>]\n"
                  << "Dùng cấu hình mặc định: " << path << " để thử dừng xe\n";
    }
    common::Config config; bool loaded=true;
    try {
        cv::FileStorage fs(path,cv::FileStorage::READ);
        if(!fs.isOpened()) throw std::runtime_error("Không mở được cấu hình để đọc servo_trim_deg: "+path);
        if(!common::HasConfigKey(fs.root(),"servo_trim_deg"))
            throw std::runtime_error("Thiếu khóa servo_trim_deg");
        common::ReadConfigValue("servo_trim_deg",fs["servo_trim_deg"],config.servo_trim_deg);
        if(config.servo_trim_deg < -60 || config.servo_trim_deg > 60) {
            throw std::runtime_error("servo_trim_deg ngoài giới hạn [-60, 60]: "+std::to_string(config.servo_trim_deg));
        }
    }
    catch(const std::exception &e) {
        // Lỗi cấu hình không được ngăn đường dừng khẩn mở thiết bị.
        config.servo_trim_deg=0;
        std::cerr << "Lỗi cấu hình servo_trim_deg: " << e.what()
                  << "; dùng trim mặc định 0 để thử dừng motor và servo\n";
        loaded=false;
    }
    try {
        app::Backend backend(config);
        const bool stopped=backend.EmergencyOpenAndStop();
        if(invalid_args) return 2;
        return stopped&&loaded ? 0 : 1;
    } catch(const std::exception &e) { std::cerr << e.what() << '\n'; return invalid_args ? 2 : 1; }
}
