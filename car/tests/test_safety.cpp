#include "app/Safety.h"
#include "app/IController.h"
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
namespace {
void Check(bool value,const char *name) { if(!value) throw std::runtime_error(name); }
void CheckConfigRates() {
    // Tệp chỉ dùng trong thư mục chạy kiểm tra; không ghi đè tệp có sẵn.
    const char *path="test_safety_rates.yaml";
    Check(!std::ifstream(path).good(),"tep kiem tra tan so da ton tai");
    struct Cleanup {
        const char *path;
        ~Cleanup() { std::remove(path); }
    } cleanup{path};
    auto load=[&](int control,int sensor,int odometry,int tof) {
        {
            std::ofstream out(path);
            out << "%YAML:1.0\n---\ncontrol_hz: " << control
                << "\nsensor_hz: " << sensor << "\nodometry_hz: " << odometry
                << "\ntof_safety_enabled: " << tof << '\n';
            out.close();
            Check(bool(out),"khong ghi duoc cau hinh kiem tra");
        }
        common::Config c;
        c.Load(path);
    };
    auto reject=[&](int control,int sensor,int tof,const char *key) {
        bool rejected=false;
        try { load(control,sensor,1,tof); }
        catch(const std::runtime_error &e) {
            rejected=std::string(e.what()).find(key)!=std::string::npos;
        }
        Check(rejected,"phai tu choi tan so thap va neu dung khoa");
    };
    for(int rate : {-1,0,1,9}) {
        reject(rate,50,1,"control_hz");
        reject(50,rate,1,"sensor_hz");
    }
    reject(9,50,0,"control_hz");
    reject(50,9,0,"sensor_hz");
    load(10,10,1,1);
    load(50,50,20,1);
}
}
int main() {
    CheckConfigRates();
    common::Config c; common::Observation o; const auto now=common::Clock::now();
    o.frame_id=1; o.captured_at=now;
    auto &t=o.tof[c.FrontSlot()]; t.valid=true; t.distanceMm=2000; t.updatedAt=now;
    std::string state;
    auto apply=[&](float speed,int angle) { return app::SafeCommand(c,o,{speed,angle},now,state); };
    Check(apply(100,100).speedCmPerSec==c.max_speed_cms,"chan toc do tien");
    Check(apply(-100,-100).steeringDeg==-c.max_steering_deg,"chan goc lai");
    Check(apply(-100,0).speedCmPerSec==-c.max_speed_cms,"chan toc do lui");
    t.distanceMm=c.tof_brake_mm-1;
    Check(apply(10,0).speedCmPerSec==0,"phanh vat can");
    Check(apply(-10,0).speedCmPerSec==-10,"cho lui khi vat can phia truoc");
    t.distanceMm=2000; t.valid=false;
    Check(apply(10,0).speedCmPerSec==0,"phanh ToF loi");
    t.valid=true; t.updatedAt=now-std::chrono::seconds(2);
    Check(apply(10,0).speedCmPerSec==0,"phanh ToF cu");
    t.updatedAt=now; o.captured_at=now-std::chrono::milliseconds(501);
    auto stopped=apply(10,10);
    Check(stopped.speedCmPerSec==0&&stopped.steeringDeg==0,"mat anh tra tam");
    o.captured_at=now; o.frame_id=0;
    Check(apply(-10,10).speedCmPerSec==0,"chua co anh");
    o.frame_id=1;
    Check(apply(std::numeric_limits<float>::quiet_NaN(),10).speedCmPerSec==0,"chan NaN");
    Check(app::StopController().step(o).speedCmPerSec==0,"bo dieu khien dung");
    std::cout << "Dat cac kiem tra an toan\n";
}
