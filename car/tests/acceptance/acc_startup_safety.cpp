// E3/E6: mặc định cấu hình và quyền gửi vị trí.
// Chỉ giao diện include/ -> Backend/App -> vết phần cứng; không đọc trường giới hạn để phán xét.
#include "acc_fixture.h"
#include "acc_recovery_timing.h"
#include "app/App.h"
#include "app/FakeHardware.h"
#include <memory>
#include <mutex>

namespace {
using namespace acc_fixture;
using Ms=std::chrono::milliseconds;

void ClearFake() {
    for(const char *name:{"UCR_FAKE_HW_TRACE","UCR_FAKE_KEYS","UCR_FAKE_CAMERA_GAPS",
                         "UCR_FAKE_TOF_SCRIPT","UCR_FAKE_UI_STALL","UCR_FAKE_FAULT"}) unsetenv(name);
}

// E3: thiếu khoá giữ đúng giá trị mặc định, không đòi cách in cảnh báo.
void Defaults() {
    Fixture f;
    ACC_CHECK(acc::Near(f.c.soft_start_s,1) && acc::Near(f.c.camera_resume_max_s,2),"Mặc định C++ 1.0/2.0");
    for(const std::string &extra:{std::string(),std::string("soft_start_s: 3\n"),std::string("camera_resume_max_s: 3\n")}) {
        f.WriteConfig(extra); common::Config c; c.Load(f.config);
        ACC_CHECK(acc::Near(c.soft_start_s,extra.find("soft_start_s")==std::string::npos ? 1 : 3),"Thiếu soft_start_s giữ 1.0");
        ACC_CHECK(acc::Near(c.camera_resume_max_s,extra.find("camera_resume_max_s")==std::string::npos ? 2 : 3),"Thiếu camera_resume_max_s giữ 2.0");
    }
}

// E6: SafeCommand không cấp ngữ cảnh calib phải từ chối vị trí; tốc độ thường không bị chặn nhầm.
void PositionDenied() {
    Fixture f; app::Backend backend(f.c); backend.Init();
    for(float speed:{-10.f,10.f}) {
        auto now=Clock::now(); auto o=Good(f.c,now); std::string state;
        auto ordinary=app::SafeCommand(f.c,o,{speed,0},now,state);
        Applied(f,backend,ordinary,speed,0,"Đối chứng: tốc độ thường qua Safety");
        for(float distance:{-50.f,50.f}) {
            common::ControlCommand cmd{speed,0}; cmd.position_control=true; cmd.position_cm=distance;
            for(bool explicit_deny:{false,true}) {
                const auto filtered=explicit_deny ? app::SafeCommand(f.c,o,cmd,now,state,false) :
                                                   app::SafeCommand(f.c,o,cmd,now,state);
                ACC_CHECK(!filtered.position_control && acc::Near(filtered.speedCmPerSec,0),"Không cấp quyền calib: vị trí phải thành lệnh dừng");
                Applied(f,backend,filtered,0,filtered.steeringDeg,"Vị trí bị loại không được chạy motor");
            }
            const auto allowed=app::SafeCommand(f.c,o,cmd,now,state,true);
            ACC_CHECK(allowed.position_control && acc::Near(allowed.position_cm,distance),"Quyền calib position: giữ lệnh vị trí hợp lệ");
            Applied(f,backend,allowed,speed,0,"Đối chứng: quyền calib không chặn tất cả vị trí");
            backend.Apply({});
        }
    }
    backend.Stop(); Tail(f);
}
}

int main(int argc,char **argv) {
    if(argc!=2) return 2;
    try {
        ClearFake();
        const std::string scenario=argv[1];
        if(scenario=="defaults") Defaults();
        else if(scenario=="position_default") PositionDenied();
        else return 2;
    } catch(const std::exception &e) { ACC_CHECK(false,e.what()); }
    return acc::Finish(argv[1]);
}
