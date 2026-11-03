#include "hooks.h"
#include <limits>

namespace auto_drive {
void pollHooks(int64_t now, const std::function<void(const char*,const std::string&)>& trace) {
#if defined(UCR_AUTO_TEST_HOOKS) && UCR_AUTO_TEST_HOOKS
    static std::streamoff consumed=0;
    static uint64_t expected=1;
    const char* path=std::getenv("UCR_AUTO_FAKE_CTRL");
    if (!path) return;
    std::ifstream f(path);
    f.seekg(consumed);
    std::string line;
    // Giới hạn công việc mỗi nhịp, kể cả khi tệp tiêm lỗi có nhiều dòng.
    for (int count=0; count<32 && std::getline(f,line); ++count) {
        if (f.eof()) break;
        consumed=f.tellg();
        std::istringstream in(line);
        uint64_t seq=0; std::string cmd, extra;
        if (!(in>>seq>>cmd) || seq!=expected++) throw std::runtime_error("hook_error: số thứ tự CTRL");
        const auto number=[&]() {
            double n;
            if (!(in>>n) || !std::isfinite(n)) throw std::runtime_error("hook_error: đối số không hữu hạn");
            return n;
        };
        if (cmd=="camera_stall") hooks.cameraStall=true;
        else if (cmd=="camera_resume") hooks.cameraStall=false;
        else if (cmd=="camera_pause_600ms") hooks.cameraPauseUntil=now+600;
        else if (cmd=="road_stall") hooks.roadStall=true;
        else if (cmd=="road_resume") hooks.roadStall=false;
        else if (cmd=="road_throw") hooks.roadThrow=true;
        else if (cmd=="yolo_stall") hooks.yoloStall=true;
        else if (cmd=="yolo_resume") hooks.yoloStall=false;
        else if (cmd=="yolo_throw") hooks.yoloThrow=true;
        else if (cmd=="yolo_throw_on") hooks.yoloThrowOn=true;
        else if (cmd=="yolo_throw_off") hooks.yoloThrowOn=false;
        else if (cmd=="planner_throw") hooks.plannerThrow=true;
        else if (cmd=="controller_stall") hooks.controllerStall=true;
        else if (cmd=="controller_resume") hooks.controllerStall=false;
        else if (cmd=="controller_throw") hooks.controllerThrow=true;
        else if (cmd=="writer_stall") hooks.writerStall=true;
        else if (cmd=="writer_resume") hooks.writerStall=false;
        else if (cmd=="jpeg_fail") hooks.jpegFail=true;
        else if (cmd=="csv_fail") hooks.csvFail=true;
        else if (cmd=="free_mb") {
            const double n=number();
            if (n<0) throw std::runtime_error("hook_error: dung lượng âm");
            hooks.freeMb=n;
        } else if (cmd=="control_delay") {
            const double n=number();
            if (n<0 || n>500 || std::floor(n)!=n) throw std::runtime_error("hook_error: control_delay ngoài miền");
            hooks.controlDelay=int(n);
        } else if (cmd=="controller_command") {
            const double cms=number(), deg=number();
            if (std::abs(cms)>1000 || std::abs(deg)>1000 || hooks.overridePending)
                throw std::runtime_error("hook_error: controller_command ngoài miền/đang bận");
            hooks.overrideCms=cms; hooks.overrideDeg=deg; hooks.overridePending=true;
        } else throw std::runtime_error("hook_error: lệnh không biết: "+cmd);
        if (in>>extra) throw std::runtime_error("hook_error: thừa đối số");
        trace("inject",line);
    }
#else
    (void)now; (void)trace;
#endif
}
}
