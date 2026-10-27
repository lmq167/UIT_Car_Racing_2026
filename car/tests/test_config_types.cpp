#include "common/Config.h"
#include "common/ConfigValue.h"
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unistd.h>
namespace {
void Check(bool ok,const std::string &message) { if(!ok) throw std::runtime_error(message); }
}
int main() {
    char path[]="/tmp/ucr_config_types_XXXXXX";
    const int fd=mkstemp(path); Check(fd>=0,"Không tạo được tệp tạm"); close(fd);
    struct Cleanup { const char *path; ~Cleanup() { std::remove(path); } } cleanup{path};
    auto load=[&](const std::string &key,const std::string &value) {
        { std::ofstream out(path); out << "%YAML:1.0\n---\n" << key << ": " << value << '\n'; }
        common::Config c; c.Load(path); return c;
    };
    auto reject=[&](const std::string &key,const std::string &value) {
        bool rejected=false;
        try { load(key,value); }
        catch(const std::runtime_error &e) {
            const std::string message=e.what();
            rejected=message.find(key)!=std::string::npos && message.find('=')!=std::string::npos;
        }
        Check(rejected,"Phải báo tên khóa và giá trị sai: "+key+" = "+value);
    };
    for(const char *key:{"camera_hz","sensor_hz","control_hz","odometry_hz","ui_hz","max_speed_cms",
        "max_steering_deg","servo_trim_deg","tof_brake_mm","tof_timeout_ms","image_timeout_ms",
        "camera_resume_max_s","resume_stable_s","soft_start_s","camera_width","camera_height","smoke_min_fps",
        "smoke_stop_speed_cms","smoke_servo_deg","calib_speed_cms","calib_servo_step_deg",
        "calib_servo_limit_deg","calib_distance_cm","calib_position_cm","calib_max_duration_s",
        "imu_drift_ms","yaw_axis","pitch_axis","roll_axis","yaw_sign","pitch_sign","roll_sign",
        "image_every_n","jpeg_quality","image_queue_size","speed_filter_size","keyboard_speed_step_cms",
        "keyboard_steering_step_deg","motor_keepalive_ms","fps_report_ms","collect_fps",
        "collect_min_speed_cms","collect_duplicate_threshold","collect_confirm_ms"})
        for(const char *value:{"abc","\"\"","\"50\"","[]","[1, 2]","{}","{ x: 1 }",".Nan",".Inf"}) reject(key,value);
    for(const char *key:{"servo_invert","tof_safety_enabled","enable_oled","enable_display","tof_mapping_confirmed"}) {
        for(const char *value:{"abc","\"\"","2","-1","0.5","1.0","yes","true","false","\"true\"","\"false\"",
            "'true'","'false'","[]","{}","[1]","{ x: 1 }"}) reject(key,value);
        for(const char *value:{"0","1","1 # xác nhận bằng mắt"}) load(key,value);
    }
    Check(load("tof_mapping_confirmed","1").tof_mapping_confirmed==1,"Ánh xạ đã xác nhận");
    Check(load("tof_mapping_confirmed","0").tof_mapping_confirmed==0,"Ánh xạ chưa xác nhận");
    reject("control_hz","50.5"); reject("servo_trim_deg","2147483648.");
    reject("max_speed_cms","1.e100");
    for(const char *value:{"-1","0","100.0000001","101"}) reject("max_speed_cms",value);
    Check(load("resume_stable_s","2.0").resume_stable_s==2.0f,"Thời gian ảnh ổn định hợp lệ");
    Check(load("resume_stable_s","0.5").resume_stable_s==0.5f,"Biên dưới thời gian ổn định");
    Check(load("resume_stable_s","30").resume_stable_s==30.0f,"Biên trên thời gian ổn định");
    for(const char *value:{"\"2\"","0.5","0"}) {
        bool rejected=false;
        try { load("camera_resume_max_s",value); }
        catch(const std::runtime_error &e) {
            const std::string message=e.what();
            rejected=message.find("camera_resume_max_s = ")!=std::string::npos &&
                message.find("ngoài khoảng hợp lệ > image_timeout_ms/1000 s")!=std::string::npos;
        }
        Check(rejected,"Lỗi phục hồi phải nêu khóa, khoảng hợp lệ và bất đẳng thức chặt theo timeout");
    }
    for(const char *value:{"-1","0","0.499999999","30.0000001","31","1.e100","1.e-100","\"2\"","[]",".Nan",".Inf"}) {
        bool rejected=false;
        try { load("resume_stable_s",value); }
        catch(const std::runtime_error &e) {
            const std::string message=e.what();
            rejected=message.find("resume_stable_s")!=std::string::npos && message.find("[0.5, 30]")!=std::string::npos;
        }
        Check(rejected,"Thời gian ổn định sai phải có tên khóa và khoảng hợp lệ");
    }
    for(const char *key:{"resume_key","khoa_khong_co"}) {
        bool rejected=false;
        try { load(key,"\"r\""); }
        catch(const std::runtime_error &e) { rejected=e.what()==std::string("khoá lạ: ")+key; }
        Check(rejected,"Khoá bỏ hoặc không hỗ trợ phải bị báo khoá lạ");
    }
    // Kiểm tra hai biên hợp lệ và các giá trị vượt khoảng tuổi mẫu ToF.
    for(const char *key:{"tof_timeout_ms","image_timeout_ms"}) for(const char *value:{"-1","0","501","10000"}) {
        bool rejected=false;
        try { load(key,value); }
        catch(const std::runtime_error &e) {
            const std::string message=e.what();
            rejected=message.find(key)!=std::string::npos &&
                     message.find(value)!=std::string::npos &&
                     message.find("[1, 500]")!=std::string::npos;
        }
        Check(rejected,"Phải báo khóa, giá trị và khoảng tuổi mẫu hợp lệ: "+std::string(key)+" = "+value);
    }
    Check(load("tof_timeout_ms","1").tof_timeout_ms==1,"Tuổi mẫu ToF tối thiểu hợp lệ");
    Check(load("tof_timeout_ms","500").tof_timeout_ms==500,"Tuổi mẫu ToF tối đa hợp lệ");
    // Các nhánh kiểm khoảng phải chỉ ra đúng khóa, giá trị và miền hợp lệ.
    for(const auto &item:std::vector<std::pair<std::string,std::string>>{
        {"camera_hz","0"},{"sensor_hz","9"},{"control_hz","9"},{"odometry_hz","0"},{"ui_hz","0"},
        {"max_steering_deg","61"},{"servo_trim_deg","61"},{"tof_brake_mm","0"},
        {"camera_resume_max_s","0.5"},{"resume_stable_s","0"},{"soft_start_s","0"},{"camera_width","0"},{"camera_height","0"},
        {"smoke_min_fps","0"},{"smoke_stop_speed_cms","0"},{"smoke_servo_deg","61"},
        {"calib_speed_cms","0"},{"calib_servo_step_deg","0"},{"calib_servo_limit_deg","61"},
        {"calib_distance_cm","0"},{"calib_position_cm","0"},{"calib_max_duration_s","0"},
        {"imu_drift_ms","0"},{"image_every_n","0"},{"jpeg_quality","101"},{"image_queue_size","0"},
        {"speed_filter_size","0"},{"motor_keepalive_ms","201"},{"fps_report_ms","0"},
        {"yaw_axis","3"},{"pitch_axis","-1"},{"roll_axis","3"},{"yaw_sign","0"},
        {"pitch_sign","0"},{"roll_sign","0"},{"keyboard_speed_step_cms","0"},
        {"keyboard_steering_step_deg","0"},{"collect_fps","11"},{"collect_min_speed_cms","0"},
        {"collect_duplicate_threshold","256"},{"collect_confirm_ms","0"}}) {
        bool rejected=false;
        try { load(item.first,item.second); }
        catch(const std::runtime_error &e) {
            const std::string message=e.what();
            rejected=message.find(item.first)!=std::string::npos && message.find(item.second)!=std::string::npos &&
                message.find("khoảng hợp lệ")!=std::string::npos;
        }
        Check(rejected,"Thiếu thông báo khoảng hợp lệ: "+item.first+" = "+item.second);
    }
    for(const char *key:{"camera_pipeline","fake_source","imu_calibration_file","log_dir","collect_luot_keys"})
        for(const char *value:{"5","1.5","[a]","[]","{}","{ x: a }"}) reject(key,value);
    for(const char *key:{"collect_keys","tof_names"})
        for(const char *value:{"5","abc","\"\"","{}","[front, 5, left, right]",
            "[front, [], left, right]","[front, {}, left, right]"}) reject(key,value);
    // In rõ kiểu nút hoặc lỗi cú pháp để đối chiếu OpenCV 4.1.1 với bản mới.
    for(const char *value:{"","~","null"}) {
        { std::ofstream out(path); out << "%YAML:1.0\n---\ncontrol_hz: " << value << '\n'; }
        bool rejected=false;
        try {
            cv::FileStorage fs(path,cv::FileStorage::READ);
            std::cout << "Giá trị [" << value << "]: có tên khóa="
                      << common::HasConfigKey(fs.root(),"control_hz")
                      << ", kiểu nút=" << fs["control_hz"].type()
                      << ", giá trị=" << common::ConfigValueText(fs["control_hz"]) << '\n';
            try { common::Config c; c.Load(path); }
            catch(const std::runtime_error &e) {
                const std::string message=e.what();
                rejected=message.find("control_hz")!=std::string::npos && message.find('=')!=std::string::npos;
            }
        } catch(const cv::Exception &e) {
            // YAML không hợp lệ cũng phải bị từ chối, không được dùng mặc định.
            std::cout << "Giá trị [" << value << "]: OpenCV từ chối cú pháp: " << e.what() << '\n';
            rejected=true;
        }
        Check(rejected,"Không được coi khóa có giá trị rỗng là khóa thiếu: control_hz: "+std::string(value));
    }
    Check(load("control_hz","50").control_hz==50,"Số nguyên hợp lệ");
    Check(load("max_speed_cms","12.5").max_speed_cms==12.5f,"Số thực hợp lệ");
    Check(load("servo_trim_deg","-7").servo_trim_deg==-7,"Trim âm hợp lệ");
    Check(load("control_hz","50").tof_safety_enabled==1,"Khóa thiếu vẫn dùng mặc định");
    Check(load("log_dir","\"5\"").log_dir=="5","Chuỗi số có dấu nháy hợp lệ");
    Check(load("fake_source","\"\"").fake_source.empty(),"Chuỗi rỗng hợp lệ cho nguồn giả");
    Check(load("collect_luot_keys","\"123456789\"").collect_luot_keys=="123456789","Bảng phím dạng chuỗi hợp lệ");
    Check(load("tof_names","[front, rear, left, right]").tof_names.size()==4,"Danh sách chuỗi hợp lệ");
    Check(load("collect_keys","[w, s, a, d, \" \", c, u, esc, \"[\", \"]\", \"+\", \"-\"]").collect_keys.size()==12,
          "Danh sách phím hợp lệ");
    std::cout << "Đạt kiểm tra kiểu các khóa số, cờ, chuỗi và danh sách\n";
}
