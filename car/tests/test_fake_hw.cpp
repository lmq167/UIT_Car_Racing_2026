#include "app/App.h"
#include "app/FakeHardware.h"
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unistd.h>
namespace {
void Check(bool ok,const std::string &message) { if(!ok) throw std::runtime_error(message); }
struct Controller:app::IController {
    common::ControlCommand step(const common::Observation &) override { return {6,4}; }
};
struct Row { long long ms; std::string kind,value; };
}
int main(int argc,char **argv) {
    Check(argc==2,"Cần tên kịch bản"); const std::string scenario=argv[1];
    char folder[]="/tmp/ucr_fake_hooks_XXXXXX";
    Check(mkdtemp(folder)!=nullptr,"Không tạo được thư mục tạm");
    const std::string root=folder,trace=root+"/trace.csv",config=root+"/config.yaml",tof=root+"/tof.txt";
    unsetenv("UCR_FAKE_HW_TRACE"); unsetenv("UCR_FAKE_TOF_SCRIPT");
    unsetenv("UCR_FAKE_CAMERA_GAPS"); unsetenv("UCR_FAKE_FAULT");
    setenv("UCR_FAKE_HW_TRACE",trace.c_str(),1);
    // Các mốc tuyệt đối dùng chung với mã giả, tránh phụ thuộc thời gian chuẩn bị thư mục.
    const auto base=app::fake::Millis()+300;
    if(scenario=="tof") {
        std::ofstream out(tof); out << "0 2000 2000 2000 2000\n" << base << " 2000 100 2000 2000\n"
            << base+300 << " 2000 -1 2000 2000\n" << base+600 << " 2000 2000 2000 2000\n";
        out.close(); setenv("UCR_FAKE_TOF_SCRIPT",tof.c_str(),1);
    } else if(scenario=="camera") {
        const auto gaps=std::to_string(base)+":"+std::to_string(base+800);
        setenv("UCR_FAKE_CAMERA_GAPS",gaps.c_str(),1);
    } else if(scenario=="actuator_throw" || scenario=="sensor_throw") {
        const auto fault=scenario+"@"+std::to_string(base);
        setenv("UCR_FAKE_FAULT",fault.c_str(),1);
    } else Check(scenario=="normal","Kịch bản không hợp lệ");
    common::Config c; c.log_dir=root; c.enable_display=c.enable_oled=0;
    c.servo_trim_deg=7; c.servo_invert=1; c.image_timeout_ms=200;
    c.tof_names={"rear","front","left","right"};
    { std::ofstream out(config); out << "%YAML:1.0\n---\nservo_trim_deg: 7\n"; }
    {
        app::App application(c,"auto",config);
        bool rejected=false;
        try { application.SetController(nullptr); } catch(const std::invalid_argument &) { rejected=true; }
        Check(rejected,"Không chấp nhận bộ điều khiển rỗng");
        application.SetController(std::unique_ptr<app::IController>(new Controller));
        application.Init(); application.Start();
        rejected=false;
        try { application.SetController(std::unique_ptr<app::IController>(new Controller)); }
        catch(const std::logic_error &) { rejected=true; }
        Check(rejected,"Không thay bộ điều khiển sau Start");
        const auto end=base+1400;
        while(application.IsRunning() && app::fake::Millis()<end)
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        const bool fault=scenario=="actuator_throw" || scenario=="sensor_throw";
        Check(application.Failed()==fault,"Trạng thái lỗi phải khớp kịch bản");
        application.RequestStop(); application.Wait();
    }
    std::ifstream in(trace); std::vector<Row> rows; std::string line;
    while(std::getline(in,line)) {
        const auto first=line.find(','),second=line.find(',',first+1);
        Check(first!=std::string::npos && second!=std::string::npos,"Vết phải đủ ba trường");
        rows.push_back({std::stoll(line.substr(0,first)),line.substr(first+1,second-first-1),line.substr(second+1)});
    }
    Check(rows.size()>4,"Phải có vết phần cứng");
    Check(rows[rows.size()-2].kind=="motor_speed" && rows[rows.size()-2].value=="0","Lệnh motor cuối bằng 0");
    Check(rows.back().kind=="servo_angle" && rows.back().value=="7","Servo cuối về trim");
    int init=0,cfg=0; bool moved=false,trim=false,blocked=false,resumed=false;
    long long last_motor=-1; std::string last_value; bool stopped_in_fault=false;
    for(const auto &row:rows) {
        if(row.kind=="motor_init") ++init;
        if(row.kind=="motor_cfg") ++cfg;
        if(row.kind=="servo_angle" && row.value=="3") trim=true;
        if(row.kind!="motor_speed") continue;
        if(row.value=="6") moved=true;
        if(scenario=="normal" && last_motor>=0 && row.value=="6" && row.value==last_value && row.ms<base+1200)
            Check(row.ms-last_motor>=c.motor_keepalive_ms-1000/c.control_hz,"Không gửi tốc độ lặp sớm khi không đổi");
        last_motor=row.ms; last_value=row.value;
        if(scenario=="tof" && row.ms>=base+100 && row.ms<base+600) {
            Check(row.value=="0","ToF gần hoặc -1 phải chặn tiến"); blocked=true;
        }
        if(scenario=="camera" && row.ms>=base+350 && row.ms<base+800) {
            Check(row.value=="0","Mất ảnh phải dừng motor"); blocked=true;
        }
        if((scenario=="tof" || scenario=="camera") && row.ms>=base+900 && row.value=="6") resumed=true;
        if((scenario=="sensor_throw" || scenario=="actuator_throw") && row.ms>=base) {
            if(stopped_in_fault) Check(row.value=="0","Sau lệnh dừng lỗi không được gửi tốc độ khác 0");
            if(row.value=="0") {
                if(!stopped_in_fault) Check(row.ms<=base+150,"Lỗi luồng phải yêu cầu dừng kịp thời");
                stopped_in_fault=true;
            }
        }
    }
    Check(init==1 && cfg==7,"Chuỗi khởi tạo chỉ một lần, đủ bảy lệnh");
    Check(moved && trim,"Đã gửi tốc độ 6 cm/s và góc 7-4=3 độ");
    if(scenario=="tof" || scenario=="camera") Check(blocked && resumed,"Quan sát được dừng rồi phục hồi");
    if(scenario=="actuator_throw" || scenario=="sensor_throw") Check(stopped_in_fault,"Quan sát được dừng sau lỗi");
    std::cout << "Đạt kịch bản " << scenario << "; vết lưu tại " << trace << '\n';
}
