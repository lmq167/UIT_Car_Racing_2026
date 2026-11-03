#include "app/App.h"
#include <sstream>
#include "app/Safety.h"
#include "common/Smoke.h"
#ifdef UCR_FAKE_HW
#include "app/FakeHardware.h"
#endif
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <poll.h>
#include <unistd.h>
namespace app {
namespace {
void Sleep(int hz) { std::this_thread::sleep_for(std::chrono::microseconds(1000000/hz)); }
}
App::App(common::Config c,std::string mode,std::string path,std::string step,double calib_drop_after):c_(std::move(c)),mode_(std::move(mode)),
    config_path_(std::move(path)),backend_(c_),step_(std::move(step)),calib_drop_after_(calib_drop_after),controller_(new StopController) {}
App::~App() { RequestStop(); Wait(); }
void App::CheckAutoSmoke() const {
    if(mode_!="auto") return;
    if(!c_.tof_mapping_confirmed || !common::SmokePassed(c_,config_path_))
        throw std::runtime_error("Chưa có smoke_gate.json ĐẠT khớp mã nguồn/cấu hình");
}
void App::Init() {
#ifdef UCR_FAKE_HW
    // Kiểm hết kịch bản trước cả tạo log và khởi tạo thiết bị.
    (void)fake::KeyScript();
    (void)fake::UIStallScript();
#endif
    CheckAutoSmoke();
    logger_.reset(new common::RunLogger(c_,config_path_,mode_));
    if(mode_=="calib") calibration_.reset(new Calibration(c_,step_,logger_->Directory(),config_path_));
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
    CheckAutoSmoke();
    started_=true;
    running_=true;
    try {
        Launch([this]{CameraThread();}); Launch([this]{SensorThread();}); Launch([this]{OdometryThread();});
        Launch([this]{ActuatorThread();}); Launch([this]{AutonomyThread();}); Launch([this]{UIThread();});
    } catch(...) { failed_=true; RequestStop(); throw; }
}
void App::SetController(std::unique_ptr<IController> controller) {
    if(started_) throw std::logic_error("Chỉ được thay bộ điều khiển trước Start");
    if(!controller) throw std::invalid_argument("Bộ điều khiển không được rỗng");
    controller_=std::move(controller);
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
    auto moving_at=common::Clock::time_point{};
    while(running_) {
        if(calib_drop_after_>0 && !drop_camera_) {
            if(moving_at==common::Clock::time_point{} && Snapshot().applied.speedCmPerSec>0) moving_at=common::Clock::now();
            if(moving_at!=common::Clock::time_point{} && common::Clock::now()-moving_at>=std::chrono::duration<double>(calib_drop_after_)) {
                drop_camera_=true; std::cout<<"Đã ngắt khung ảnh hiệu chuẩn; đánh dấu vị trí bánh lúc này.\n";
            }
        }
        cv::Mat frame;
        if(!drop_camera_ && backend_.Frame(frame)) {
            const auto time=common::Clock::now();
            if(mode_=="calib" && step_=="latency") {
                const auto s=Snapshot();
                cv::putText(frame,"M="+std::to_string(s.applied.marker)+" A="+std::to_string(s.applied.steeringDeg),
                    cv::Point(frame.cols/20,frame.rows/10),cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(0,255,255),1);
            }
            std::lock_guard<std::mutex> lock(shared_.mutex);
            auto &o=shared_.value.observation;
            if(ImageGapExceeded(c_,o,time)) image_gap_seen_=true;
            o.frame=std::move(frame); o.captured_at=time; ++o.frame_id;
        }
        Sleep(c_.camera_hz);
    }
}
void App::SensorThread() {
    auto previous=common::Clock::now();
    while(running_) {
#ifdef UCR_FAKE_HW
        fake::Fault("sensor_throw");
#endif
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
    CameraRecovery recovery; auto previous_tick=common::Clock::now();
    while(running_) {
#ifdef UCR_FAKE_HW
        fake::Fault("actuator_throw");
#endif
        auto s=Snapshot(); auto now=common::Clock::now(); auto cmd=s.requested;
        std::string state=mode_=="auto" ? "DUNG" : s.requested_state;
        const bool held_mode=mode_=="calib" || mode_=="manual";
        const char *latch_reason=nullptr;
        const auto cancel_held=[&](const char *reason) {
            bool had_command=false;
            {
                std::lock_guard<std::mutex> lock(shared_.mutex);
                const auto &requested=shared_.value.requested;
                had_command=requested.speedCmPerSec!=0 || requested.steeringDeg!=0 ||
                    requested.position_control || requested.calib_limits.enabled ||
                    previous.speedCmPerSec!=0 || previous.steeringDeg!=0 || previous.position_control;
                shared_.value.requested={}; cancel_held_command_=true;
            }
            cmd={};
            // Lệnh đã xoá thì những vòng còn nguy hiểm không tạo thêm lần chốt.
            if(had_command) latch_reason=reason;
        };
        const bool image_gap=held_mode && image_gap_seen_.exchange(false);
        const bool calib_limit=(mode_=="calib") && CalibrationLimitReached(c_,s.observation,cmd.calib_limits,now);
        if((held_mode && (!FreshImage(c_,s.observation,now) || image_gap)) || calib_limit) {
            // Giữ yêu cầu huỷ tới khi luồng bàn phím nhận được, kể cả ảnh đã có lại.
            if(held_mode) cancel_held(calib_limit ? "gioi_han_calib" : "mat_anh");
            else {
                std::lock_guard<std::mutex> lock(shared_.mutex);
                shared_.value.requested={}; cancel_held_command_=true; cmd={};
            }
            if(calib_limit) state="DUNG_GIOI_HAN_CALIB";
            else state="MAT_ANH";
        }
        cmd.speedCmPerSec=std::clamp(cmd.speedCmPerSec,-c_.max_speed_cms,c_.max_speed_cms);
        if(mode_=="auto") cmd=recovery.Filter(c_,s.observation,cmd,now,state);
        cmd=SafeCommand(c_,s.observation,cmd,now,state,mode_=="calib" && step_=="position");
        if(held_mode && s.requested.speedCmPerSec!=0 && cmd.speedCmPerSec==0 &&
            (state=="PHANH_TOF" || state=="PHANH_TOF_SAU"))
            cancel_held(state=="PHANH_TOF_SAU" ? "phanh_tof_sau" : "phanh_tof");
        if(logger_->Failed()) throw std::runtime_error("Loi luong ghi anh");
        if(!running_) { backend_.Stop(); break; }
        const bool send_motor=cmd.speedCmPerSec!=previous.speedCmPerSec || cmd.position_control!=previous.position_control ||
            now-sent>=std::chrono::milliseconds(c_.motor_keepalive_ms);
        if(send_motor || cmd.steeringDeg!=previous.steeringDeg || cmd.marker!=previous.marker) {
            backend_.Apply(cmd,send_motor); previous=cmd;
            if(send_motor) sent=now;
        }
        // Gửi lệnh dừng trước khi ghi thông báo để đầu ra chậm không trì hoãn phanh.
        if(latch_reason) std::cout<<std::string("AN_TOAN_CHOT: ")+latch_reason+'\n'<<std::flush;
        s.applied=cmd; s.state=state; s.sampled_at=now;
        s.actuator_ms=std::chrono::duration<double,std::milli>(now-previous_tick).count();
        { std::lock_guard<std::mutex> lock(shared_.mutex); shared_.value.applied=cmd; shared_.value.state=state;
          shared_.value.actuator_ms=s.actuator_ms;
          // Ghi cùng ảnh/thời điểm đã xét an toàn, không ghép với ảnh của vòng sau.
          actuator_snapshot_=s; }
        previous_tick=now;
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
            // Một lần ghi, để không chen vào giữa dòng AN_TOAN_CHOT của luồng chấp hành.
            std::ostringstream fps_line; fps_line << "FPS: " << fps << '\n'; std::cout << fps_line.str();
        }
        { std::lock_guard<std::mutex> lock(shared_.mutex); s=actuator_snapshot_; }
        if(s.sampled_at!=common::Clock::time_point{}) logger_->Record(s);
        Sleep(c_.control_hz);
    }
}
void App::UIThread() {
    const char *display=std::getenv("DISPLAY"), *wayland=std::getenv("WAYLAND_DISPLAY");
    const bool show=c_.enable_display && ((display&&*display)||(wayland&&*wayland));
    bool keyboard_available=show;
#ifdef UCR_FAKE_HW
    keyboard_available=keyboard_available || !fake::KeyScript().empty();
#endif
    if(mode_=="manual"&&!keyboard_available) std::cerr << "Khong co cua so ban phim; xe giu dung, Ctrl+C de thoat\n";
    auto rendered=common::Clock::time_point{};
    std::uint64_t rendered_marker=0;
#ifdef UCR_FAKE_HW
    const auto &ui_stall=fake::UIStallScript();
    bool stalled=false;
#endif
    bool terminal_open=true;
    while(running_) {
#ifdef UCR_FAKE_HW
        if(!stalled && fake::Millis()>=ui_stall.first) {
            stalled=true;
            std::this_thread::sleep_for(std::chrono::milliseconds(ui_stall.second));
        }
#endif
        auto s=Snapshot(); const auto now=common::Clock::now();
        const bool marker_changed=s.applied.marker!=rendered_marker;
        if(marker_changed || now-rendered>=std::chrono::duration<double>(1.0/c_.ui_hz)) {
            auto view=s;
            if(marker_changed && s.applied.marker) view.state="DANH_DAU_"+std::to_string(s.applied.marker);
            backend_.Render(view,mode_); rendered=now; rendered_marker=s.applied.marker;
        }
        int key=-1;
        if(show) {
            if(!s.observation.frame.empty()) cv::imshow("UCR",s.observation.frame);
            const int raw_key=cv::waitKey(1);
            key=raw_key<0 ? -1 : raw_key&255;
        }
#ifdef UCR_FAKE_HW
        if(mode_=="calib" || mode_=="manual") {
            const int injected=fake::NextKey();
            if(injected>=0) key=injected;
        }
#endif
        if(key=='q'||key=='Q'||key==27) { RequestStop(); break; }
        if(mode_=="manual") {
            std::lock_guard<std::mutex> lock(shared_.mutex); auto &cmd=shared_.value.requested;
            if(cancel_held_command_.exchange(false)) cmd={};
            if(!FreshImage(c_,shared_.value.observation,common::Clock::now())) cmd={};
            else switch(key) {
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
        if(calibration_ && terminal_open && key<0) {
            pollfd input{STDIN_FILENO,POLLIN,0};
            if(::poll(&input,1,0)>0 && (input.revents&(POLLIN|POLLHUP))) {
                char ch=0; const auto count=::read(STDIN_FILENO,&ch,1);
                if(count==0) {
                    terminal_open=false;
                    // Mất đầu vào thì huỷ lượt đang lái; vẫn cho phép quan sát và nhận tín hiệu dừng.
                    if(s.requested.speedCmPerSec!=0 || s.requested.steeringDeg!=0) key='x';
                } else if(count>0 && ch!='\n' && ch!='\r') key=static_cast<unsigned char>(ch);
            }
        }
        if(calibration_) {
            if(cancel_held_command_.exchange(false)) calibration_->CancelMotion();
            const auto cmd=calibration_->Tick(s,key);
            if(calibration_->Done()) { if(calibration_->Failed()) failed_=true; RequestStop(); break; }
            std::lock_guard<std::mutex> lock(shared_.mutex);
            shared_.value.requested=cancel_held_command_ || (mode_=="calib" &&
                !FreshImage(c_,shared_.value.observation,common::Clock::now())) ? common::ControlCommand{} : cmd;
            shared_.value.requested_state=calibration_->State();
        }
        Sleep(calibration_ ? std::max(20,c_.control_hz) : c_.ui_hz);
    }
    if(show) cv::destroyAllWindows();
}
}
