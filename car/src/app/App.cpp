#include "app/App.h"
#include "app/Safety.h"
#include <opencv2/highgui.hpp>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <utility>
namespace app {
namespace {
void Sleep(int hz) { std::this_thread::sleep_for(std::chrono::microseconds(1000000/hz)); }
}
App::App(common::Config c,std::string mode,std::string path):c_(std::move(c)),mode_(std::move(mode)),
    config_path_(std::move(path)),backend_(c_),controller_(new StopController) {}
App::~App() { RequestStop(); Wait(); }
void App::Init() {
    logger_.reset(new common::RunLogger(c_,config_path_,mode_));
    if(mode_=="collect") {
        const char *display=std::getenv("DISPLAY"), *wayland=std::getenv("WAYLAND_DISPLAY");
        if(!c_.enable_display || !((display&&*display)||(wayland&&*wayland)))
            throw std::runtime_error("Thu ảnh cần cửa sổ nhận bàn phím; bật enable_display và mở phiên màn hình trước khi chạy");
        collector_.reset(new common::CollectRecorder(c_,logger_->Directory(),std::cin,std::cout));
    }
    backend_.Init();
}
void App::Launch(std::function<void()> work) {
    threads_.emplace_back([this,work]{
        try { work(); }
        catch(const std::exception &e) {
            failed_=true; RequestStop();
            std::cerr << "Lỗi luồng: " << e.what() << '\n'
                      << "Đã yêu cầu dừng xe. Kiểm tra nguồn, dây nối, quyền UART/I²C/camera và dung lượng ghi; "
                         "xử lý lỗi trước khi khởi động phiên mới. Không tự thay cấu hình phần cứng.\n";
        }
        catch(...) { failed_=true; RequestStop(); }
    });
}
void App::Start() {
    running_=true;
    try {
        Launch([this]{CameraThread();}); Launch([this]{SensorThread();}); Launch([this]{OdometryThread();});
        Launch([this]{ActuatorThread();}); Launch([this]{AutonomyThread();}); Launch([this]{UIThread();});
    } catch(...) { failed_=true; RequestStop(); throw; }
}
void App::RequestStop() noexcept { running_=false; backend_.Stop(); }
void App::Wait() {
    // Dừng trước khi chờ camera/ghi ảnh để không kéo dài chuyển động khi thoát.
    backend_.Stop();
    for(auto &t:threads_) if(t.joinable()) t.join();
    threads_.clear(); backend_.Stop(); if(logger_) logger_->Close();
}
common::Snapshot App::Snapshot() { std::lock_guard<std::mutex> lock(shared_.mutex); return shared_.value; }
void App::CameraThread() {
    while(running_) {
        cv::Mat frame;
        if(backend_.Frame(frame)) {
            const auto time=common::Clock::now();
            std::lock_guard<std::mutex> lock(shared_.mutex);
            auto &o=shared_.value.observation; o.frame=std::move(frame); o.captured_at=time; ++o.frame_id;
        }
        Sleep(c_.camera_hz);
    }
}
void App::SensorThread() {
    auto previous=common::Clock::now();
    while(running_) {
        auto now=common::Clock::now(); common::Observation o;
        backend_.Sensors(o,std::chrono::duration<double>(now-previous).count()); previous=now;
        { std::lock_guard<std::mutex> lock(shared_.mutex); auto &v=shared_.value.observation;
          v.imu=o.imu; v.imu_valid=o.imu_valid; v.tof=o.tof; v.sensor_at=o.sensor_at;
          v.yaw_rel_deg=o.yaw_rel_deg; v.pitch_deg=o.pitch_deg; v.roll_deg=o.roll_deg; }
        Sleep(c_.sensor_hz);
    }
}
void App::OdometryThread() {
    float previous_cm=0; bool have_previous=false;
    auto previous_time=common::Clock::now(); std::deque<double> speeds;
    while(running_) {
        float cm=0; const bool valid=backend_.Position(cm)&&std::isfinite(cm); auto now=common::Clock::now();
        double speed=0;
        if(valid) {
            if(have_previous) {
                const double dt=std::chrono::duration<double>(now-previous_time).count();
                if(dt>0) speeds.push_back((cm-previous_cm)/dt);
                if(speeds.size()>static_cast<std::size_t>(c_.speed_filter_size)) speeds.pop_front();
                if(!speeds.empty()) speed=std::accumulate(speeds.begin(),speeds.end(),0.0)/speeds.size();
            }
            previous_cm=cm; previous_time=now; have_previous=true;
        } else { have_previous=false; speeds.clear(); }
        { std::lock_guard<std::mutex> lock(shared_.mutex); auto &o=shared_.value.observation;
          o.odom_valid=valid; o.meas_speed=speed; if(valid) { o.odom_cm=cm; o.odom_at=now; } }
        Sleep(c_.odometry_hz);
    }
}
void App::ActuatorThread() {
    common::ControlCommand previous; auto sent=common::Clock::time_point{};
    while(running_) {
        auto s=Snapshot(); auto now=common::Clock::now(); auto cmd=s.requested;
        std::string state=mode_=="auto" ? "DUNG" : "LAI_TAY";
        cmd=SafeCommand(c_,s.observation,cmd,now,state);
        if(logger_->Failed()) throw std::runtime_error("Loi luong ghi anh");
        if(!running_) { backend_.Stop(); break; }
        const bool send_motor=cmd.speedCmPerSec!=previous.speedCmPerSec ||
            now-sent>=std::chrono::milliseconds(c_.motor_keepalive_ms);
        if(send_motor || cmd.steeringDeg!=previous.steeringDeg) {
            backend_.Apply(cmd,send_motor); previous=cmd;
            if(send_motor) sent=now;
        }
        { std::lock_guard<std::mutex> lock(shared_.mutex); shared_.value.applied=cmd; shared_.value.state=state; }
        Sleep(c_.control_hz);
    }
    backend_.Stop();
}
void App::AutonomyThread() {
    std::uint64_t last_frame=0, count=0; auto started=common::Clock::now();
    while(running_) {
        auto s=Snapshot();
        if(s.observation.frame_id!=last_frame) {
            last_frame=s.observation.frame_id; ++count;
            if(mode_=="auto") {
                auto cmd=controller_->step(s.observation);
                std::lock_guard<std::mutex> lock(shared_.mutex); shared_.value.requested=cmd;
            }
        }
        const auto now=common::Clock::now();
        const double elapsed=std::chrono::duration<double>(now-started).count();
        if(elapsed*1000>=c_.fps_report_ms) {
            const double fps=count/elapsed; count=0; started=now;
            { std::lock_guard<std::mutex> lock(shared_.mutex); shared_.value.fps=fps; }
            std::cout << "FPS: " << fps << '\n';
        }
        logger_->Record(Snapshot()); Sleep(c_.control_hz);
    }
}
void App::UIThread() {
    const char *display=std::getenv("DISPLAY"), *wayland=std::getenv("WAYLAND_DISPLAY");
    const bool show=c_.enable_display && ((display&&*display)||(wayland&&*wayland));
    if(mode_=="manual"&&!show) std::cerr << "Khong co cua so ban phim; xe giu dung, Ctrl+C de thoat\n";
    auto rendered=common::Clock::time_point{};
    while(running_) {
        auto s=Snapshot(); const auto now=common::Clock::now();
        if(now-rendered>=std::chrono::duration<double>(1.0/c_.ui_hz)) { backend_.Render(s,mode_); rendered=now; }
        if(show) {
            if(!s.observation.frame.empty()) cv::imshow("UCR",collector_ ? collector_->View(s) : s.observation.frame);
            const int raw_key=cv::waitKey(1);
            const int key=raw_key<0 ? -1 : raw_key&255;
            if(collector_) {
                auto cmd=s.requested;
                if(collector_->Key(key,cmd)) { RequestStop(); break; }
                { std::lock_guard<std::mutex> lock(shared_.mutex); shared_.value.requested=cmd; }
                collector_->Capture(s);
            } else if(key=='q'||key=='Q'||key==27) { RequestStop(); break; }
            if(mode_=="manual") {
                std::lock_guard<std::mutex> lock(shared_.mutex); auto &cmd=shared_.value.requested;
                switch(key) {
                    case 'w': case 'W': cmd.speedCmPerSec+=c_.keyboard_speed_step_cms; break;
                    case 's': case 'S': cmd.speedCmPerSec-=c_.keyboard_speed_step_cms; break;
                    case 'a': case 'A': cmd.steeringDeg-=c_.keyboard_steering_step_deg; break;
                    case 'd': case 'D': cmd.steeringDeg+=c_.keyboard_steering_step_deg; break;
                    case 'c': case 'C': cmd.steeringDeg=0; break;
                    case ' ': case 'x': case 'X': cmd={}; break;
                }
                cmd.speedCmPerSec=std::clamp(cmd.speedCmPerSec,-c_.max_speed_cms,c_.max_speed_cms);
                cmd.steeringDeg=std::clamp(cmd.steeringDeg,-c_.max_steering_deg,c_.max_steering_deg);
            }
        }
        Sleep(collector_ ? std::max(20,c_.control_hz) : c_.ui_hz);
    }
    if(show) cv::destroyAllWindows();
}
}
