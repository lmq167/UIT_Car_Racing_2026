#pragma once
#include "app/Backend.h"
#include "app/IController.h"
#include "common/RunLogger.h"
#include "common/CollectRecorder.h"
#include "app/Calibration.h"
#include "common/SharedChannels.h"
#include <atomic>
#include <functional>
#include <thread>
#include <vector>
namespace app {
class App {
public:
    App(common::Config c, std::string mode, std::string config_path,std::string step="",
        std::string kill_file="",bool smoke_drop=false,double calib_drop_after=0);
    ~App();
    void Init();
    void Start();
    // Chỉ thay bộ điều khiển trước lần Start đầu tiên; không được truyền con trỏ rỗng.
    void SetController(std::unique_ptr<IController> controller);
    void RequestStop() noexcept;
    void Wait();
    bool IsRunning() const { return running_; }
    bool Failed() const { return failed_; }
private:
    void CheckAutoSmoke() const;
    void Launch(std::function<void()> work);
    void CameraThread();
    void SensorThread();
    void OdometryThread();
    void ActuatorThread();
    void AutonomyThread();
    void UIThread();
    common::Snapshot Snapshot();
    common::Config c_;
    std::string mode_, config_path_;
    Backend backend_;
    common::SharedData shared_;
    common::Snapshot actuator_snapshot_;
    std::unique_ptr<common::RunLogger> logger_;
    std::unique_ptr<common::CollectRecorder> collector_;
    std::unique_ptr<Calibration> calibration_;
    std::string step_,kill_file_;
    bool smoke_drop_=false;
    double calib_drop_after_=0;
    std::atomic<bool> drop_camera_{false};
    std::atomic<bool> cancel_held_command_{false};
    std::unique_ptr<IController> controller_;
    std::atomic<bool> running_{false}, failed_{false};
    std::vector<std::thread> threads_;
    std::atomic<bool> started_{false};
};
}
