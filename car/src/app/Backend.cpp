#include "app/Backend.h"
#ifndef UCR_FAKE_HW
#include "common/Logger.h"
#endif
#ifdef UCR_FAKE_HW
#include "app/FakeHardware.h"
#endif
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>
namespace app {
#ifdef UCR_FAKE_HW
void Backend::AdvanceFake() {
    const auto now=common::Clock::now();
    fake_position_+=fake_speed_.load()*std::chrono::duration<double>(now-fake_updated_).count();
    fake_updated_=now;
}
#endif
namespace {
#ifndef UCR_FAKE_HW
double Axis(const BNO055::Vector3 &v, int axis) { return axis==0 ? v.x : axis==1 ? v.y : v.z; }
#endif
void Check(bool ok, const char *message) { if (!ok) throw std::runtime_error(message); }
}
Backend::Backend(const common::Config &c) : c_(c)
#ifndef UCR_FAKE_HW
, imu_("/dev/i2c-1", c.imu_calibration_file)
#endif
{}
Backend::~Backend() { Stop(); }
void Backend::Init() {
#ifndef UCR_FAKE_HW
    // Điểm gọi duy nhất chuỗi khởi tạo motor; không thử lại khi lỗi.
    motor_ready_=true;
    Check(motor_.Init(), "Loi khoi tao motor");
    servo_ready_=true;
    Check(servo_.Init() && servo_.SetPWMFreq(50), "Loi khoi tao servo");
    Apply({});
    Check(imu_.initialize(), "Loi khoi tao IMU");
    Check(imu_.measureYawRateDrift(std::chrono::milliseconds(c_.imu_drift_ms)), "Loi do troi IMU");
    // Độ lệch trục cấu hình được đo riêng khi khác trục Z của hàm BTC.
    bias_=imu_.yawRateBiasDegPerSec();
    if (c_.yaw_axis!=2) {
        double sum=0; int count=0;
        auto end=common::Clock::now()+std::chrono::milliseconds(c_.imu_drift_ms);
        while(common::Clock::now()<end) {
            BNO055::Data d; Check(imu_.read(d), "Loi do troi truc IMU");
            sum+=Axis(d.gyroscopeDegPerSec,c_.yaw_axis); ++count;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        bias_=sum/count;
    }
    Check(imu_.address()!=0x29, "IMU 0x29 trung dia chi khoi tao ToF; can xac nhan BTC");
    Check(tof_.Init("/dev/i2c-1"), "Loi khoi tao ToF");
    if (c_.enable_oled) { Check(oled_.Init(), "Loi khoi tao OLED"); oled_ready_=true; }
    Check(camera_.open(c_.camera_pipeline,cv::CAP_GSTREAMER), "Loi mo camera");
#else
    if(motor_ready_) throw std::runtime_error("Không được lặp khởi tạo motor giả");
    fake::TraceNumber("motor_init",1);
    // Mô phỏng đúng thứ tự chuỗi khởi tạo driver; mỗi lệnh một dòng.
    for(const char *cmd:{"N1 O K1","N1 O D1","N1 $008=0","N1 $002=330","N1 O M3","N1 $101=1","N1 O U C r"})
        fake::Trace("motor_cfg",cmd);
    motor_ready_=servo_ready_=true;
    Apply({});
    // Nguồn trống tạo ảnh kiểm tra; có thể dùng video hoặc mẫu tên ảnh %06d.jpg.
    if (!c_.fake_source.empty()) Check(camera_.open(c_.fake_source), "Loi mo nguon anh gia lap");
#endif
}
bool Backend::Frame(cv::Mat &frame) {
#ifdef UCR_FAKE_HW
    if(fake::CameraGap()) return false;
    if(c_.fake_source.empty()) { frame=cv::Mat(c_.camera_height,c_.camera_width,CV_8UC3,cv::Scalar(40,60,40)); return true; }
#endif
    return camera_.read(frame) && !frame.empty();
}
bool Backend::Sensors(common::Observation &o, double dt) {
#ifndef UCR_FAKE_HW
    common::DeferredDeviceLog messages;
#endif
    std::lock_guard<std::timed_mutex> lock(i2c_mutex_);
#ifdef UCR_FAKE_HW
    (void)dt;
    o.imu_valid=true;
    std::array<int,4> distances{{2000,2000,2000,2000}};
    const auto now=fake::Millis();
    const auto &rows=fake::TofScript();
    // Trước dòng đầu tiên của kịch bản: chưa có mẫu đo, không coi là đường trống.
    if(!fake::Env("UCR_FAKE_TOF_SCRIPT").empty()) distances.fill(-1);
    for(const auto &row:rows) if(row.ms<=now) distances=row.mm;
    for(std::size_t i=0;i<o.tof.size();++i) {
        auto &t=o.tof[i]; t.distanceMm=distances[i]; t.initialized=true;
        t.valid=t.hasSample=distances[i]>=0; t.rangeStatus=t.valid ? 0 : 255;
        t.updatedAt=common::Clock::now();
    }
#else
    o.imu_valid=imu_.read(o.imu);
    Check(tof_.Read(o.tof,std::chrono::milliseconds(c_.tof_timeout_ms)), "Loi doc ToF");
    Check(o.imu_valid, "Loi doc IMU");
    yaw_+=(Axis(o.imu.gyroscopeDegPerSec,c_.yaw_axis)-bias_)*dt*c_.yaw_sign;
    o.pitch_deg=Axis(o.imu.eulerDeg,c_.pitch_axis)*c_.pitch_sign;
    o.roll_deg=Axis(o.imu.eulerDeg,c_.roll_axis)*c_.roll_sign;
#endif
    o.yaw_rel_deg=yaw_; o.sensor_at=common::Clock::now(); return true;
}
bool Backend::Position(float &cm) {
#ifndef UCR_FAKE_HW
    common::DeferredDeviceLog messages;
#endif
    std::lock_guard<std::timed_mutex> lock(motor_mutex_);
#ifdef UCR_FAKE_HW
    AdvanceFake(); cm=static_cast<float>(fake_position_); return true;
#else
    return motor_.GetPositionCm(&cm);
#endif
}
void Backend::Apply(common::ControlCommand c, bool send_motor) {
#ifndef UCR_FAKE_HW
    common::DeferredDeviceLog messages;
    const auto deadline=common::Clock::now()+std::chrono::milliseconds(100);
    try {
    if(send_motor || c.speedCmPerSec==0) {
      std::unique_lock<std::timed_mutex> lock(motor_mutex_,std::defer_lock);
      Check(lock.try_lock_until(deadline),"Hết hạn chờ khoá motor; buộc dừng phiên");
      if(stopping_) return;
      if(c.position_control && c.speedCmPerSec>0) {
          if(!position_active_) {
              motor_.PositionControl(true); Check(motor_.LastWriteOk(),"Lỗi chuyển chế độ vị trí");
              position_active_=true;
              motor_.SetSpeedPositionCms(c.position_cm,c.speedCmPerSec);
          }
      } else {
          if(position_active_) { motor_.SetSpeedCms(0); motor_.PositionControl(false); position_active_=false; }
          motor_.SetSpeedCms(c.speedCmPerSec);
      }
      Check(motor_.LastWriteOk(),"Loi gui motor"); }
    { std::unique_lock<std::timed_mutex> lock(i2c_mutex_,std::defer_lock);
      Check(lock.try_lock_until(deadline),"Hết hạn chờ khoá I²C; buộc dừng phiên");
      if(stopping_) return;
      Check(servo_.SetAngle(c_.servo_trim_deg+(c_.servo_invert ? -c.steeringDeg : c.steeringDeg)),"Loi gui servo"); }
    } catch(...) {
        // Gửi v0 ngay, trước mọi lần chờ khoá tiếp theo hoặc xuất lỗi.
        stopping_=true;
        const bool sent=motor_.EmergencyStop();
        StopUntil(deadline);
        common::Logger::Error("Backend","apply",sent ? "Đã gửi v0 dự phòng; buộc dừng phiên" : "Không gửi được v0 dự phòng");
        throw;
    }
#else
    const auto deadline=common::Clock::now()+std::chrono::milliseconds(100);
    {
        std::unique_lock<std::timed_mutex> lock(motor_mutex_,std::defer_lock);
        if(!lock.try_lock_until(deadline)) {
            StopUntil(deadline);
            throw std::runtime_error("Hết hạn chờ khoá motor giả; buộc dừng phiên");
        }
        if(stopping_) return;
        AdvanceFake();
        if(send_motor || c.speedCmPerSec==0) {
            const bool position_sent=c.position_control && c.speedCmPerSec>0 && !position_active_;
            position_active_=c.position_control && c.speedCmPerSec>0;
            fake_speed_=c.speedCmPerSec;
            // Ghi vết trong khoá: Stop() không chen được giữa đặt tốc độ và dòng vết.
            if(position_sent) fake::TraceNumber("motor_position",c.position_cm);
            fake::TraceNumber("motor_speed",c.speedCmPerSec);
        }
    }
    const int angle=c_.servo_trim_deg+(c_.servo_invert ? -c.steeringDeg : c.steeringDeg);
    fake::TraceNumber("servo_angle",std::max(-60,std::min(60,angle)));
#endif
}
void Backend::Stop() noexcept {
    StopUntil(common::Clock::now()+std::chrono::milliseconds(100));
}
void Backend::StopUntil(common::Clock::time_point deadline) noexcept {
    stopping_=true;
#ifdef UCR_FAKE_HW
    try {
        {
            std::unique_lock<std::timed_mutex> lock(motor_mutex_,std::defer_lock);
            if(lock.try_lock_until(deadline)) AdvanceFake();
            fake_speed_=0;
        }
        if(motor_ready_) fake::TraceNumber("motor_speed",0);
        if(servo_ready_) fake::TraceNumber("servo_angle",c_.servo_trim_deg);
    } catch(...) {}
#endif
#ifndef UCR_FAKE_HW
    try {
        common::DeferredDeviceLog messages;
        bool motor_ok=true, servo_ok=true, motor_timeout=false, servo_timeout=false;
        {
            std::unique_lock<std::timed_mutex> lock(motor_mutex_,std::defer_lock);
            if(lock.try_lock_until(deadline)) {
                if(motor_ready_) {
                    motor_.SetSpeedCms(0); motor_ok=motor_.LastWriteOk();
                    if(position_active_) {
                        motor_.PositionControl(false); motor_ok=motor_.LastWriteOk() && motor_ok;
                        position_active_=false; motor_.SetSpeedCms(0); motor_ok=motor_.LastWriteOk() && motor_ok;
                    }
                }
            } else {
                motor_timeout=true;
                motor_ok=motor_.EmergencyStop();
            }
        }
        {
            std::unique_lock<std::timed_mutex> lock(i2c_mutex_,std::defer_lock);
            if(lock.try_lock_until(deadline)) {
                if(servo_ready_) servo_ok=servo_.SetAngle(c_.servo_trim_deg);
            } else {
                servo_timeout=true;
                // Đối tượng riêng tránh truy cập trạng thái Servo đang do luồng khác giữ.
                Servo emergency_servo;
                servo_ok=emergency_servo.Init() && emergency_servo.SetPWMFreq(50) && emergency_servo.SetAngle(c_.servo_trim_deg);
            }
        }
        if(motor_timeout || servo_timeout)
            common::Logger::Error("Backend","stop","Hết hạn chờ khoá thiết bị; đã thử đường dừng độc lập");
        if(!motor_ok || !servo_ok)
            common::Logger::Error("Backend","stop","Không xác nhận được motor 0 hoặc servo về trim");
    } catch(...) { motor_.EmergencyStop(); }
#endif
}
void Backend::Render(const common::Snapshot &s, const std::string &mode) {
#ifndef UCR_FAKE_HW
    common::DeferredDeviceLog messages;
    std::lock_guard<std::timed_mutex> lock(i2c_mutex_);
    if(!oled_ready_) return;
    oled_.clear();
    if(s.state=="DUNG_CHO_ANH_ON_DINH" || s.state=="MAT_ANH") {
        oled_.drawString(0,0,s.state);
        oled_.drawString(0,15,s.state=="MAT_ANH" ? "CHO ANH MOI" : "CHO ANH ON DINH");
    } else {
        oled_.drawString(0,0,mode+" "+s.state);
        oled_.drawString(0,15,"FPS "+std::to_string(static_cast<int>(s.fps)));
    }
    oled_.drawString(0,30,"cm/s "+std::to_string(s.observation.meas_speed));
    oled_.drawString(0,45,"ToF "+std::to_string(s.observation.tof[c_.FrontSlot()].distanceMm)); oled_.display();
#else
    (void)s; (void)mode;
#endif
}
}
