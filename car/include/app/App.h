#pragma once
#include "app/Backend.h"
#include "app/IController.h"
#include "common/RunLogger.h"
#include "common/SharedChannels.h"
#include <atomic>
#include <functional>
#include <thread>
#include <vector>
namespace app {
class App {
public:
    App(common::Config c, std::string mode, std::string config_path);
    ~App();
    void Init();
    void Start();
    void RequestStop() noexcept;
    void Wait();
    bool IsRunning() const { return running_; }
    bool Failed() const { return failed_; }
private:
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
    std::unique_ptr<common::RunLogger> logger_;
    std::unique_ptr<IController> controller_;
    std::atomic<bool> running_{false}, failed_{false};
    std::vector<std::thread> threads_;
};
}
