// Tiện ích kiểm thử: chỉ dùng giao diện công khai và vết lệnh đã công bố trong docs/ucr_car.md.
#pragma once
#ifndef UCR_FAKE_HW
#error "Test hiệu chuẩn và an toàn chi duoc bien dich voi UCR_FAKE_HW=ON"
#endif
#include "acc_trace.h"
#include "acc_seed_gate.h"
#include "app/Backend.h"
#include "app/Safety.h"
#include "common/Smoke.h"
#include <stdexcept>

namespace acc_fixture {
using common::Clock;
inline void RemoveTemp(const std::string &path) {
    for (const auto &name : acc::ListDir(path)) {
        const auto child=path+"/"+name;
        struct stat st{};
        if (lstat(child.c_str(),&st)==0 && S_ISDIR(st.st_mode)) RemoveTemp(child);
        else unlink(child.c_str());
    }
    rmdir(path.c_str());
}
struct Fixture {
    std::string dir=acc::MakeTempDir("acc_fixture_");
    std::string config=dir+"/config.yaml", trace=dir+"/trace.csv";
    common::Config c;
    Fixture() {
        if(dir.empty()) throw std::runtime_error("Không tạo được thư mục tạm");
        c.log_dir=dir+"/runs";
        if(mkdir(c.log_dir.c_str(),0700)!=0) throw std::runtime_error("Không tạo được runs tạm");
        c.enable_display=0; c.enable_oled=0; c.servo_trim_deg=7;
        c.tof_mapping_confirmed=1; c.image_every_n=1000000;
        c.fake_source="";
        WriteConfig();
        if(setenv("UCR_FAKE_HW_TRACE",trace.c_str(),1)!=0) throw std::runtime_error("Không đặt được vết giả");
    }
    ~Fixture() { unsetenv("UCR_FAKE_HW_TRACE"); RemoveTemp(dir); }
    void WriteConfig(const std::string &extra="") {
        if(!acc::WriteFile(config,"%YAML:1.0\n---\nlog_dir: \""+c.log_dir+
            "\"\nenable_display: 0\nenable_oled: 0\n"+extra)) throw std::runtime_error("Không ghi được config tạm");
    }
    void Gate() {
        acc::SeedGate(c,config);
    }
};
inline common::Observation Good(const common::Config &c,Clock::time_point now) {
    common::Observation o;
    o.frame_id=1; o.captured_at=now; o.imu_valid=true;
    o.sensor_at=now; o.odom_at=now; o.odom_valid=true;
    o.frame=cv::Mat(c.camera_height,c.camera_width,CV_8UC3,cv::Scalar(60,100,140));
    o.imu.calibration={3,3,3,3};
    for(auto &r:o.tof) {
        r.valid=true; r.hasSample=true; r.initialized=true; r.rangeStatus=0;
        r.timedOut=false; r.distanceMm=2*c.tof_brake_mm; r.updatedAt=now;
    }
    return o;
}
inline void Tail(const Fixture &f) {
    const auto trace=acc::ReadTrace(f.trace);
    double v=999;
    ACC_CHECK(acc::LastNum(trace,"motor_speed",v) && acc::Near(v,0),"Lệnh motor cuối phải bằng 0");
    ACC_CHECK(acc::LastNum(trace,"servo_angle",v) && acc::Near(v,f.c.servo_trim_deg),"Servo cuối phải về trim vật lý");
}
inline void Applied(const Fixture &f,app::Backend &backend,common::ControlCommand cmd,
                    float speed,int steer,const std::string &label) {
    backend.Apply(cmd);
    const auto trace=acc::ReadTrace(f.trace);
    double v=999;
    ACC_CHECK(acc::LastNum(trace,"motor_speed",v) && acc::Near(v,speed,0.001),label+": sai tốc độ cuối ra phần cứng");
    const int physical=std::max(-60,std::min(60,(f.c.servo_invert ? -steer : steer)+f.c.servo_trim_deg));
    ACC_CHECK(acc::LastNum(trace,"servo_angle",v) && acc::Near(v,physical),label+": sai góc vật lý sau chiều/trim");
}
}
