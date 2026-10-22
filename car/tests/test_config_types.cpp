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
        "imu_drift_ms","yaw_axis","pitch_axis","roll_axis","yaw_sign","pitch_sign","roll_sign",
        "image_every_n","jpeg_quality","image_queue_size","speed_filter_size","keyboard_speed_step_cms",
        "keyboard_steering_step_deg","motor_keepalive_ms","fps_report_ms","collect_fps",
        "collect_min_speed_cms","collect_duplicate_threshold","collect_confirm_ms"})
        for(const char *value:{"abc","\"\"","\"50\"","[]","[1, 2]","{}","{ x: 1 }",".Nan",".Inf"}) reject(key,value);
    for(const char *key:{"servo_invert","tof_safety_enabled","enable_oled","enable_display"}) {
        for(const char *value:{"abc","\"\"","2","-1","0.5","[]","{}","[1]","{ x: 1 }"}) reject(key,value);
        for(const char *value:{"0","1","true","false","\"true\"","\"false\""}) load(key,value);
    }
    reject("control_hz","50.5"); reject("servo_trim_deg","2147483648.");
    reject("max_speed_cms","1.e100");
    // Kiểm tra hai biên hợp lệ và các giá trị vượt khoảng tuổi mẫu ToF.
    for(const char *value:{"-1","0","501","10000"}) {
        bool rejected=false;
        try { load("tof_timeout_ms",value); }
        catch(const std::runtime_error &e) {
            const std::string message=e.what();
            rejected=message.find("tof_timeout_ms")!=std::string::npos &&
                     message.find(value)!=std::string::npos &&
                     message.find("[1, 500]")!=std::string::npos;
        }
        Check(rejected,"Phải báo khóa, giá trị và khoảng ToF hợp lệ: "+std::string(value));
    }
    Check(load("tof_timeout_ms","1").tof_timeout_ms==1,"Tuổi mẫu ToF tối thiểu hợp lệ");
    Check(load("tof_timeout_ms","500").tof_timeout_ms==500,"Tuổi mẫu ToF tối đa hợp lệ");
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
