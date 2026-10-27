#include "app/Backend.h"
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
    fake_position_+=fake_speed_*std::chrono::duration<double>(now-fake_updated_).count();
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
bool Backend::EmergencyOpenAndStop() {
#ifdef UCR_FAKE_HW
    fake::TraceNumber("motor_speed",0);
    fake::TraceNumber("servo_angle",c_.servo_trim_deg);
    std::cout << "Gia lap: motor 0, servo " << c_.servo_trim_deg << '\n'; return true;
#else
    // Thử cả hai thiết bị ngay cả khi một cổng lỗi; không khởi tạo driver motor.
    const bool m=motor_.OpenPortOnly();
    bool sent=false;
    if(m) { motor_.SetSpeedCms(0); sent=motor_.LastWriteOk(); }
    const bool s=servo_.Init() && servo_.SetPWMFreq(50);
    const bool centered=s && servo_.SetAngle(c_.servo_trim_deg);
    return m && sent && centered;
#endif
}
bool Backend::Frame(cv::Mat &frame) {
#ifdef UCR_FAKE_HW
    if(fake::CameraGap()) return false;
    if(c_.fake_source.empty()) { frame=cv::Mat(384,640,CV_8UC3,cv::Scalar(40,60,40)); return true; }
#endif
    return camera_.read(frame) && !frame.empty();
}
bool Backend::Sensors(common::Observation &o, double dt) {
    std::lock_guard<std::mutex> lock(i2c_mutex_);
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
    std::lock_guard<std::mutex> lock(motor_mutex_);
#ifdef UCR_FAKE_HW
    AdvanceFake(); cm=static_cast<float>(fake_position_); return true;
#else
    return motor_.GetPositionCm(&cm);
#endif
}
void Backend::Apply(common::ControlCommand c, bool send_motor) {
#ifndef UCR_FAKE_HW
    if(send_motor) { std::lock_guard<std::mutex> lock(motor_mutex_);
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
    { std::lock_guard<std::mutex> lock(i2c_mutex_);
      if(stopping_) return;
      Check(servo_.SetAngle(c_.servo_trim_deg+(c_.servo_invert ? -c.steeringDeg : c.steeringDeg)),"Loi gui servo"); }
#else
    std::lock_guard<std::mutex> lock(motor_mutex_);
    if(stopping_) return;
    AdvanceFake();
    if(send_motor) {
        if(c.position_control && c.speedCmPerSec>0 && !position_active_) fake::TraceNumber("motor_position",c.position_cm);
        position_active_=c.position_control && c.speedCmPerSec>0;
        fake_speed_=c.speedCmPerSec; fake::TraceNumber("motor_speed",fake_speed_);
    }
    const int angle=c_.servo_trim_deg+(c_.servo_invert ? -c.steeringDeg : c.steeringDeg);
    fake::TraceNumber("servo_angle",std::max(-60,std::min(60,angle)));
#endif
}
void Backend::Stop() noexcept {
    stopping_=true;
#ifdef UCR_FAKE_HW
    try { std::lock_guard<std::mutex> lock(motor_mutex_); AdvanceFake(); fake_speed_=0;
          if(motor_ready_) fake::TraceNumber("motor_speed",0);
          if(servo_ready_) fake::TraceNumber("servo_angle",c_.servo_trim_deg); } catch(...) {}
#endif
#ifndef UCR_FAKE_HW
    try { std::lock_guard<std::mutex> lock(motor_mutex_); if(motor_ready_) {
        motor_.SetSpeedCms(0);
        if(position_active_) { motor_.PositionControl(false); position_active_=false; motor_.SetSpeedCms(0); }
    } } catch(...) {}
    try { std::lock_guard<std::mutex> lock(i2c_mutex_); if(servo_ready_) servo_.SetAngle(c_.servo_trim_deg); } catch(...) {}
#endif
}
void Backend::Render(const common::Snapshot &s, const std::string &mode) {
#ifndef UCR_FAKE_HW
    std::lock_guard<std::mutex> lock(i2c_mutex_);
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
