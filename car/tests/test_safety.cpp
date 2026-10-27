#include "app/Safety.h"
#include "app/IController.h"
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <unistd.h>
namespace {
void Check(bool value,const char *name) { if(!value) throw std::runtime_error(name); }
void CheckRecoveryAtLowFps() {
    common::Config c;
    c.image_timeout_ms=500; c.camera_resume_max_s=2; c.soft_start_s=1;
    const auto origin=common::Clock::now();
    // Khung mới cách 83 ms (~12 FPS); kiểm cả khi luồng chấp hành cũng chậm.
    for(int tick_ms:{20,83}) for(float limit:{0.1f,25.0f}) for(float target:{20.0f,40.0f,-20.0f}) {
        c.max_speed_cms=limit;
        app::CameraRecovery recovery;
        common::Observation o;
        bool reached=false;
        for(int ms=0;ms<=2200;ms+=tick_ms) {
            const auto now=origin+std::chrono::milliseconds(ms);
            int frame_ms=ms/83*83;
            if(frame_ms>=332 && frame_ms<996) frame_ms=249;
            o.frame_id=1+frame_ms/83;
            o.captured_at=origin+std::chrono::milliseconds(frame_ms);
            for(auto &tof:o.tof) { tof.valid=true; tof.distanceMm=2000; tof.updatedAt=now; }
            const float full=std::clamp(target,-limit,limit);
            std::string state;
            auto cmd=recovery.Filter(c,o,{full,4},now,state);
            cmd=app::SafeCommand(c,o,cmd,now,state);
            if(ms>=760 && ms<1000) Check(cmd.speedCmPerSec==0,"mất ảnh ở FPS thấp phải dừng");
            if(ms>=1000 && ms<1996) {
                const double cap=std::abs(full)*(ms-996)/1000.0;
                Check(std::abs(cmd.speedCmPerSec)<=cap+1e-3,"FPS thấp vẫn giữ trần tăng mềm theo thời gian");
            }
            if(ms>=2000) {
                Check(std::abs(cmd.speedCmPerSec-full)<=1e-3,"sau soft_start_s phải đạt lệnh đầy ở FPS thấp");
                reached=true;
            }
        }
        Check(reached,"phải quan sát lệnh đầy sau tăng mềm");
    }
}
void CheckConfigRates() {
    // Mỗi lượt có thư mục riêng, kể cả khi nhiều bản build chạy đồng thời.
    char folder[]="/tmp/ucr_safety_rates_XXXXXX";
    Check(mkdtemp(folder)!=nullptr,"khong tao duoc thu muc kiem tra tan so");
    const std::string config=std::string(folder)+"/test_safety_rates.yaml";
    const char *path=config.c_str();
    Check(!std::ifstream(path).good(),"tep kiem tra tan so da ton tai");
    struct Cleanup {
        const char *path, *folder;
        ~Cleanup() { std::remove(path); rmdir(folder); }
    } cleanup{path,folder};
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
void CheckLongRecovery() {
    common::Config c;
    const auto base=common::Clock::now();
    auto at=[&](int ms){return base+std::chrono::milliseconds(ms);};
    for(float speed:{20.0f,-20.0f}) {
        app::CameraRecovery recovery; common::Observation o; std::string state;
        auto run=[&](int ms,bool image) {
            if(image) { ++o.frame_id; o.captured_at=at(ms); }
            return recovery.Filter(c,o,{speed,4},at(ms),state);
        };
        Check(run(0,false).speedCmPerSec==0,"chưa từng có ảnh phải đứng yên");
        Check(run(100,true).speedCmPerSec==0,"ảnh đầu tiên cũng bắt đầu tăng mềm từ 0");
        Check(run(2201,false).speedCmPerSec==0 && state=="MAT_ANH","ngắt dài mà ảnh còn cũ vẫn là MAT_ANH");
        Check(run(2300,true).steeringDeg==0 && state=="DUNG_CHO_ANH_ON_DINH","ảnh về: chờ ổn định và trả góc về trim");
        for(int ms=2400;ms<=3300;ms+=100) Check(run(ms,true).speedCmPerSec==0,"chưa đủ thời gian phải đứng yên");
        Check(run(3801,false).speedCmPerSec==0 && state=="MAT_ANH","ảnh lại cũ phải tính lại khoảng ổn định");
        for(int ms=3900;ms<=5800;ms+=100) Check(run(ms,true).speedCmPerSec==0,"sau ngắt phải chờ đủ khoảng mới");
        Check(run(5900,true).speedCmPerSec==0 && state=="TANG_TOC_MEM","đủ ổn định bắt đầu tăng từ 0");
        for(int ms=6000;ms<=6900;ms+=100) {
            const auto cmd=run(ms,true);
            Check(std::abs(cmd.speedCmPerSec-speed*(ms-5900)/1000.0f)<1e-3,"tiến/lùi tăng mềm đúng trần");
        }
        Check(run(7401,false).speedCmPerSec==0,"mất ảnh trong/sau tăng mềm vẫn dừng");
        Check(run(7500,true).speedCmPerSec==0 && state=="TANG_TOC_MEM","ngắt ngắn tiếp theo vẫn tăng mềm");
    }
    // Chỉ nhận một khung rồi giữ nguyên không thể hoàn thành khoảng ổn định.
    app::CameraRecovery recovery; common::Observation o; std::string state;
    o.frame_id=1; o.captured_at=at(0); recovery.Filter(c,o,{20,0},at(0),state);
    recovery.Filter(c,o,{20,0},at(2200),state);
    o.frame_id=2; o.captured_at=at(2300);
    for(int ms=2300;ms<=5000;ms+=100)
        Check(recovery.Filter(c,o,{20,0},at(ms),state).speedCmPerSec==0,"khung đứng yên không chứng minh ảnh liên tục");
}
void CheckFirstImageRamp() {
    common::Config c;
    const auto base=common::Clock::now();
    for(float speed:{25.0f,-25.0f}) {
        app::CameraRecovery recovery; common::Observation o; std::string state;
        Check(recovery.Filter(c,o,{speed,0},base,state).speedCmPerSec==0,"đợi ảnh đầu phải dừng");
        for(int ms=3000;ms<=4200;ms+=100) {
            const auto now=base+std::chrono::milliseconds(ms);
            o.frame_id++; o.captured_at=now;
            const auto cmd=recovery.Filter(c,o,{speed,0},now,state);
            const float expected=speed*std::min(1.0f,(ms-3000)/1000.0f);
            Check(std::abs(cmd.speedCmPerSec-expected)<1e-3,"ảnh đầu trễ 3 s vẫn tăng từ 0 đủ soft_start_s");
            Check(state!="DUNG_CHO_ANH_ON_DINH","ảnh đầu không áp thời gian chờ phục hồi dài");
        }
    }
}
}
int main() {
    CheckRecoveryAtLowFps();
    CheckLongRecovery();
    CheckFirstImageRamp();
    CheckConfigRates();
    common::Config c; common::Observation o; const auto now=common::Clock::now();
    o.frame_id=1; o.captured_at=now;
    auto &t=o.tof[c.FrontSlot()]; t.valid=true; t.distanceMm=2000; t.updatedAt=now;
    auto &rear=o.tof[c.RearSlot()]; rear.valid=true; rear.distanceMm=2000; rear.updatedAt=now;
    std::string state;
    for(const char *mode:{"auto","manual","collect","smoke","calib_servo","calib_motor","calib_position"}) {
        const bool allowed=std::string(mode)=="calib_position";
        const auto cmd=app::SafeCommand(c,o,{10,5,true,50},now,state,allowed);
        Check(cmd.position_control==allowed && cmd.speedCmPerSec==(allowed ? 10 : 0),
            "chỉ calib bước position được giữ lệnh vị trí");
        if(!allowed) Check(cmd.steeringDeg==0 && cmd.position_cm==0,"lệnh vị trí bị cấm phải xoá toàn bộ");
    }
    Check(app::SafeCommand(c,o,{10,5,true,50},now,state).speedCmPerSec==0,
        "đường gọi không cấp quyền vị trí mặc định giữ dừng");
    auto apply=[&](float speed,int angle) { return app::SafeCommand(c,o,{speed,angle},now,state); };
    for(int fault:{0,1,2,3}) {
        rear.valid=fault!=0; rear.timedOut=fault==1;
        rear.distanceMm=fault==3 ? c.tof_brake_mm/2 : 2000;
        rear.updatedAt=fault==2 ? now-std::chrono::seconds(2) : now;
        Check(apply(10,0).speedCmPerSec==10,"ToF sau hỏng/hết giờ không chặn tiến");
        Check(apply(-10,0).speedCmPerSec==0 && state=="PHANH_TOF_SAU","ToF sau hỏng/hết giờ phải chặn lùi");
        common::ControlCommand position{10,0,true,50};
        state="VI_TRI";
        Check(app::SafeCommand(c,o,position,now,state,true).speedCmPerSec==10 && state=="VI_TRI",
            "ToF sau không chặn lệnh vị trí có tốc độ tiến");
        position.speedCmPerSec=-10;
        Check(app::SafeCommand(c,o,position,now,state,true).speedCmPerSec==0 && state=="PHANH_TOF_SAU",
            "ToF sau vẫn chặn lệnh vị trí có tốc độ lùi");
    }
    rear.valid=true; rear.timedOut=false; rear.updatedAt=now; rear.distanceMm=2000;
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
    t.updatedAt=now; o.captured_at=now-std::chrono::milliseconds(c.image_timeout_ms);
    Check(!app::FreshImage(c,o,now),"tuổi ảnh đúng ngưỡng không còn mới");
    auto stopped=apply(10,10);
    Check(stopped.speedCmPerSec==0&&stopped.steeringDeg==0,"mat anh tra tam");
    o.captured_at=now; o.frame_id=0;
    Check(apply(-10,10).speedCmPerSec==0,"chua co anh");
    o.frame_id=1;
    Check(apply(std::numeric_limits<float>::quiet_NaN(),10).speedCmPerSec==0,"chan NaN");
    Check(app::StopController().step(o).speedCmPerSec==0,"bo dieu khien dung");
    std::cout << "Dat cac kiem tra an toan\n";
}
