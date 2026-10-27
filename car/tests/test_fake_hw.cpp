#include "app/App.h"
#include "app/FakeHardware.h"
#include "app/Safety.h"
#include "common/Smoke.h"
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
    unsetenv("UCR_FAKE_KEYS");
    unsetenv("UCR_FAKE_UI_STALL");
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
    if(scenario=="normal") {
        Check(app::fake::ParseUIStall("")==std::make_pair(0LL,0LL),"biến chặn UI rỗng không chặn");
        Check(app::fake::ParseUIStall("0:0")==std::make_pair(0LL,0LL),"chấp nhận hai số 0");
        Check(app::fake::ParseUIStall("1200:4000")==std::make_pair(1200LL,4000LL),"đúng mốc và thời gian chặn UI");
        for(const char *bad:{"1",":1","1:","1:2:3","-1:2","1:-2","+1:2","1.0:2"," 1:2","1:2,3:4",
                             "9223372036854775808:1","1:9223372036854775808"}) {
            bool rejected=false;
            try { (void)app::fake::ParseUIStall(bad); }
            catch(const std::runtime_error &e) { rejected=std::string(e.what()).find("UCR_FAKE_UI_STALL")!=std::string::npos; }
            Check(rejected,std::string("Phải báo rõ kịch bản chặn UI sai: ")+bad);
        }
        const auto keys=app::fake::ParseKeys("1500:w,0:,,1500::,1500: ,1500:\x7f");
        Check(keys==std::vector<std::pair<long long,int>>{{0,','},{1500,'w'},{1500,':'},{1500,' '},{1500,127}},
              "Phím ASCII, thứ tự thời gian và thứ tự cùng mốc phải được giữ đúng");
        Check(app::fake::ParseKeys("").empty(),"Biến rỗng không tiêm phím");
        Check(app::fake::ParseKeys("9223372036854775807:w").front().first==std::numeric_limits<long long>::max(),
              "Mốc ms lớn nhất không bị tràn");
        for(const char *bad:{":w","-1:w","+1:w","1.5:w"," 1:w","1:","1:ww","1:w,",
                             "1:w,,2:s","1:w,2","9223372036854775808:w","1:\xc3\xa9"}) {
            bool rejected=false;
            try { (void)app::fake::ParseKeys(bad); }
            catch(const std::runtime_error &e) { rejected=std::string(e.what()).find("UCR_FAKE_KEYS")!=std::string::npos; }
            Check(rejected,std::string("Phải báo rõ kịch bản phím sai: ")+bad);
        }
        setenv("UCR_FAKE_KEYS","0:w,0:s,9223372036854775807:q",1);
        Check(app::fake::NextKey()=='w' && app::fake::NextKey()=='s' && app::fake::NextKey()==-1,
              "Mỗi phím chỉ phát một lần, mốc tương lai chưa phát");
    }
    common::Config c; c.log_dir=root; c.enable_display=c.enable_oled=0;
    c.servo_trim_deg=7; c.servo_invert=1; c.image_timeout_ms=200;
    c.tof_names={"rear","front","left","right"};
    c.tof_mapping_confirmed=1;
    { std::ofstream out(config); out << "%YAML:1.0\n---\nservo_trim_deg: 7\n"; }
    for(bool custom:{false,true}) {
        app::App application(c,"auto",config);
        if(custom) application.SetController(std::unique_ptr<app::IController>(new Controller));
        bool rejected=false;
        try { application.Init(); } catch(const std::runtime_error &) { rejected=true; }
        Check(rejected && !std::ifstream(trace).good(),"Thiếu smoke: chặn cả bộ điều khiển mặc định trước khởi tạo motor");
    }
    common::SmokeResult(root+"/smoke_gate.json",common::ConfigText(config),true,true,true,false,true,true,true,true);
    {
        app::App application(c,"auto",config);
        bool rejected=false;
        try { application.Init(); } catch(const std::runtime_error &) { rejected=true; }
        Check(rejected && !std::ifstream(trace).good(),"Smoke KHÔNG ĐẠT: chặn bản giả mặc định trước motor");
    }
    common::SmokeResult(root+"/smoke_gate.json",common::ConfigText(config),true,true,true,true,true,true,true,true);
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
        const auto end=base+(scenario=="camera" ? 2200 : 1400);
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
        if(scenario=="camera" && row.kind=="servo_angle" && row.ms>=base+350 && row.ms<base+800)
            Check(row.value=="7","Mất ảnh trả góc vật lý về trim, kể cả khi đảo chiều servo");
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
    if(scenario=="normal") {
        common::RunLogger logger(c,config,"sample");
        common::Snapshot s; s.sampled_at=common::Clock::now();
        s.observation.frame_id=1; s.observation.captured_at=s.sampled_at-std::chrono::milliseconds(100);
        s.applied={6,4}; s.state="DUNG";
        std::this_thread::sleep_for(std::chrono::milliseconds(c.image_timeout_ms+20));
        logger.Record(s);
        s.sampled_at+=std::chrono::milliseconds(400); s.applied={}; s.state="MAT_ANH";
        logger.Record(s); logger.Close();
        std::ifstream input(logger.Directory()+"/log.csv"); std::string line;
        std::getline(input,line);
        for(const auto &want:std::vector<std::pair<double,std::string>>{{100,"DUNG"},{500,"MAT_ANH"}}) {
            Check(bool(std::getline(input,line)),"Phải ghi đủ hai mẫu chấp hành");
            std::istringstream row(line); std::vector<std::string> cells; std::string cell;
            while(std::getline(row,cell,',')) cells.push_back(cell);
            Check(cells.size()>=15 && std::abs(std::stod(cells[2])-want.first)<=1e-3 && cells[14]==want.second,
                  "Tuổi ảnh và trạng thái phải cùng mốc chấp hành dù ghi đĩa muộn");
            Check(std::abs(std::stod(cells[3])-(want.second=="MAT_ANH" ? 0 : 6))<=1e-3,
                  "CSV phải giữ đúng lệnh đã áp dụng của từng mẫu");
        }
    }
    if(scenario=="normal") for(int invert:{0,1}) {
        auto config=c; config.servo_invert=invert; config.image_timeout_ms=500;
        const auto geometry=root+"/servo_"+std::to_string(invert)+".csv";
        setenv("UCR_FAKE_HW_TRACE",geometry.c_str(),1);
        app::Backend backend(config); backend.Init();
        app::CameraRecovery recovery; common::Observation o;
        const auto origin=common::Clock::now();
        auto apply=[&](int ms,bool image,float speed,int angle) {
            const auto now=origin+std::chrono::milliseconds(ms);
            if(image) { ++o.frame_id; o.captured_at=now; }
            for(auto &t:o.tof) { t.valid=true; t.distanceMm=2000; t.updatedAt=now; }
            std::string state;
            auto cmd=recovery.Filter(config,o,{6,4},now,state);
            cmd=app::SafeCommand(config,o,cmd,now,state); backend.Apply(cmd);
            std::ifstream input(geometry); std::string row; float motor=999,servo=999;
            while(std::getline(input,row)) {
                const auto comma=row.find(','); const auto end=row.find(',',comma+1);
                const auto kind=row.substr(comma+1,end-comma-1);
                if(kind=="servo_angle") servo=std::stof(row.substr(end+1));
                if(kind=="motor_speed") motor=std::stof(row.substr(end+1));
            }
            Check(std::abs(motor-speed)<1e-4,"tốc độ phục hồi cuối tới Backend phải đúng");
            Check(servo==angle,"góc phục hồi: đảo chiều góc lái rồi cộng trim đúng một lần");
        };
        const int turned=invert ? 3 : 11;
        apply(0,true,6,turned);
        apply(501,false,0,7);
        apply(600,true,0,turned);
        for(int ms=700;ms<1100;ms+=100) apply(ms,true,6.f*(ms-600)/1000,turned);
        apply(1100,true,3,turned);
        for(int ms=1200;ms<1600;ms+=100) apply(ms,true,6.f*(ms-600)/1000,turned);
        apply(1600,true,6,turned);
        apply(3900,false,0,7);
        for(int ms=4000;ms<6000;ms+=100) apply(ms,true,0,7);
        apply(6000,true,0,turned);
        for(int ms=6100;ms<6500;ms+=100) apply(ms,true,6.f*(ms-6000)/1000,turned);
        apply(6500,true,3,turned);
        backend.Stop();
    }
    if(scenario=="normal") for(float limit:{0.1f,25.0f}) for(int gap_ms:{600,2300}) for(float requested:{20.0f,40.0f}) {
        auto config=c; config.calib_speed_cms=requested; config.image_timeout_ms=500;
        config.max_speed_cms=limit;
        const auto file=root+"/calib_"+std::to_string(limit)+"_"+std::to_string(gap_ms)+"_"+std::to_string(int(requested))+".csv";
        setenv("UCR_FAKE_HW_TRACE",file.c_str(),1);
        app::Backend backend(config); backend.Init();
        app::Calibration calibration(config,"motor",root,root+"/config.yaml","",false);
        common::Snapshot snapshot;
        const auto origin=common::Clock::now();
        auto motor_speed=[&] {
            std::ifstream input(file); std::string row; double motor=999;
            while(std::getline(input,row)) {
                const auto comma=row.find(','),end=row.find(',',comma+1);
                if(row.substr(comma+1,end-comma-1)=="motor_speed") motor=std::stod(row.substr(end+1));
            }
            return motor;
        };
        auto apply=[&](int ms,bool image,int key) {
            const auto now=origin+std::chrono::milliseconds(ms);
            auto &o=snapshot.observation;
            if(image) { ++o.frame_id; o.captured_at=now; }
            o.odom_valid=true; o.odom_at=now;
            for(auto &t:o.tof) { t.valid=true; t.distanceMm=2000; t.updatedAt=now; }
            auto cmd=calibration.Tick(snapshot,key,now);
            auto state=calibration.State();
            cmd=app::SafeCommand(config,o,cmd,now,state);
            snapshot.state=state; backend.Apply(cmd);
            const float expected=image && key=='y' ? std::min(requested,config.max_speed_cms) : 0;
            Check(std::abs(motor_speed()-expected)<=1e-3,"calib: mất ảnh xoá lượt; chỉ bấm lại mới chạy tới Backend");
            Check(state!="DUNG_CHO_ANH_ON_DINH" && state!="TANG_TOC_MEM","calib không dùng phục hồi auto");
        };
        apply(0,true,'y'); apply(501,false,'y'); apply(gap_ms,true,-1); apply(gap_ms+50,true,'y');
        // Collect dùng lệnh người lái qua cùng SafeCommand, không qua CameraRecovery.
        for(float direction:{-1.0f,1.0f}) {
            auto o=snapshot.observation; auto now=o.captured_at;
            std::string state;
            o.captured_at=now-std::chrono::milliseconds(gap_ms);
            auto cmd=app::SafeCommand(config,o,{direction*requested,0},now,state);
            backend.Apply(cmd);
            Check(motor_speed()==0,
                  "collect mất ảnh dừng ở cả hai chiều");
            o.captured_at=now;
            cmd=app::SafeCommand(config,o,{direction*requested,0},now,state);
            backend.Apply(cmd);
            Check(std::abs(motor_speed()-direction*std::min(requested,config.max_speed_cms))<=1e-3,
                  "collect ảnh mới trả ngay lệnh trong trần, không tăng mềm/chốt");
        }
        backend.Stop();
    }
    std::cout << "Đạt kịch bản " << scenario << "; vết lưu tại " << trace << '\n';
}
