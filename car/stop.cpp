#include "app/Backend.h"
#include "common/ConfigValue.h"
#include "common/Smoke.h"
#include <opencv2/core.hpp>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <cmath>
#include <utility>
namespace {
double Uptime() { double value=0; std::ifstream in("/proc/uptime"); if(!(in>>value)) return -1; return value; }
}
int main(int argc,char **argv) {
    std::string path="../config.yaml";
    std::string smoke_report; double since=-1;
    bool invalid_args=false;
    for(int i=1;i<argc;++i) {
        const std::string arg=argv[i];
        if((arg!="--config" && arg!="--smoke-report" && arg!="--stop-since") || i+1>=argc ||
           std::string(argv[i+1]).empty() || argv[i+1][0]=='-') {
            invalid_args=true;
            break;
        }
        if(arg=="--config") path=argv[++i];
        else if(arg=="--smoke-report") smoke_report=argv[++i];
        else { try { std::size_t used=0; const std::string text=argv[++i]; since=std::stod(text,&used);
                     if(used!=text.size() || !std::isfinite(since) || since<0) invalid_args=true;
               } catch(...) { invalid_args=true; } }
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
        if(!smoke_report.empty() && !invalid_args) {
            // Mốc /proc/uptime được run.sh lấy trước kill, gồm cả thời gian mở UART/I²C.
            common::Config full; full.Load(path);
            bool zero=false; int consecutive=0; float previous=0;
            auto previous_at=common::Clock::now(); bool have=backend.Position(previous);
            while(stopped && loaded && since>=0 && Uptime()>=since && Uptime()-since<=1) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                float current=0; const bool valid=backend.Position(current); const auto now=common::Clock::now();
                const double dt=std::chrono::duration<double>(now-previous_at).count();
                const bool at_rest=have&&valid&&dt>0&&std::isfinite(current)&&
                    std::abs((current-previous)/dt)<=full.smoke_stop_speed_cms;
                consecutive=at_rest ? consecutive+1 : 0;
                previous=current; previous_at=now; have=valid;
                if(consecutive>=3 && Uptime()-since<=1) { zero=true; break; }
            }
            cv::FileStorage before(smoke_report,cv::FileStorage::READ);
            if(!before.isOpened() || std::string(before["source_id"])!=common::SourceId() ||
               std::string(before["config_text"])!=full.loaded_text || int(before["simulated"])!=common::Simulated())
                throw std::runtime_error("Bản ghi smoke không khớp");
            auto flag=[&](const char *key){return int(before[key])==1;};
            const bool camera=flag("camera"),sensors=flag("sensors"),servo=flag("servo"),tof=flag("tof_brake"),
                drop=flag("camera_brake"),moving=flag("encoder_moving");
            before.release();
            const bool pass=camera&&sensors&&servo&&tof&&drop&&moving&&zero;
            for(const auto &item: {std::make_pair("camera",camera),{"IMU/ToF",sensors},{"servo",servo},
                    {"phanh ToF",tof},{"phanh mất ảnh",drop},{"encoder trước kill > 0",moving},{"kill -9 / encoder về 0 <= 1 s",zero}})
                std::cout<<item.first<<": "<<(item.second ? "ĐẠT" : "KHÔNG ĐẠT")<<'\n';
            common::SmokeResult(smoke_report,full.loaded_text,camera,sensors,servo,tof,drop,moving,true,zero);
            common::SmokeResult(full.log_dir+"/smoke_gate.json",full.loaded_text,camera,sensors,servo,tof,drop,moving,true,zero);
            return pass ? 0 : 1;
        }
        if(invalid_args) return 2;
        return stopped&&loaded ? 0 : 1;
    } catch(const std::exception &e) { std::cerr << e.what() << '\n'; return invalid_args ? 2 : 1; }
}
