// E3/E4/E6: mặc định cấu hình, ảnh đầu tăng mềm và quyền gửi vị trí.
// Chỉ giao diện include/ -> Backend/App -> vết phần cứng; không đọc trường giới hạn để phán xét.
#include "acc_fixture.h"
#include "acc_recovery_timing.h"
#include "app/App.h"
#include "app/FakeHardware.h"
#include <memory>
#include <mutex>

namespace {
using namespace acc_fixture;
using Ms=std::chrono::milliseconds;

struct Seen {
    std::mutex mutex;
    std::vector<Frame> frames;
    Clock::time_point origin=Clock::now()-Ms(app::fake::Millis());
};
class Controller : public app::IController {
    std::shared_ptr<Seen> seen_;
    std::string scenario_;
public:
    Controller(std::shared_ptr<Seen> seen,std::string scenario):seen_(std::move(seen)),scenario_(std::move(scenario)) {}
    common::ControlCommand step(const common::Observation &o) override {
        {
            std::lock_guard<std::mutex> lock(seen_->mutex);
            if(o.frame_id && (seen_->frames.empty() || seen_->frames.back().id!=o.frame_id))
                seen_->frames.push_back({o.frame_id,std::chrono::duration_cast<Ms>(o.captured_at-seen_->origin).count()});
        }
        common::ControlCommand cmd{scenario_=="reverse" ? -20.f : 20.f,10};
        if(scenario_=="position") {
            cmd.position_control=true; cmd.position_cm=50;
            // E6: tự gắn giới hạn hợp lệ không được cấp quyền vị trí cho auto.
            cmd.calib_limits.enabled=true; cmd.calib_limits.deadline=Clock::now()+std::chrono::seconds(3);
            cmd.calib_limits.start_odom_cm=o.odom_cm; cmd.calib_limits.max_distance_cm=100;
        }
        return cmd;
    }
};

void ClearFake() {
    for(const char *name:{"UCR_FAKE_HW_TRACE","UCR_FAKE_KEYS","UCR_FAKE_CAMERA_GAPS",
                         "UCR_FAKE_TOF_SCRIPT","UCR_FAKE_UI_STALL","UCR_FAKE_FAULT"}) unsetenv(name);
}

// E3: thiếu khoá giữ đúng giá trị mặc định, không đòi cách in cảnh báo.
void Defaults() {
    Fixture f;
    ACC_CHECK(acc::Near(f.c.soft_start_s,1) && acc::Near(f.c.camera_resume_max_s,2),"Mặc định C++ 1.0/2.0");
    for(const std::string &extra:{std::string(),std::string("soft_start_s: 3\n"),std::string("camera_resume_max_s: 3\n")}) {
        f.WriteConfig(extra); common::Config c; c.Load(f.config);
        ACC_CHECK(acc::Near(c.soft_start_s,extra.find("soft_start_s")==std::string::npos ? 1 : 3),"Thiếu soft_start_s giữ 1.0");
        ACC_CHECK(acc::Near(c.camera_resume_max_s,extra.find("camera_resume_max_s")==std::string::npos ? 2 : 3),"Thiếu camera_resume_max_s giữ 2.0");
    }
}

// E6: SafeCommand không cấp ngữ cảnh calib phải từ chối vị trí; tốc độ thường không bị chặn nhầm.
void PositionDenied() {
    Fixture f; app::Backend backend(f.c); backend.Init();
    for(float speed:{-10.f,10.f}) {
        auto now=Clock::now(); auto o=Good(f.c,now); std::string state;
        auto ordinary=app::SafeCommand(f.c,o,{speed,0},now,state);
        Applied(f,backend,ordinary,speed,0,"Đối chứng: tốc độ thường qua Safety");
        for(float distance:{-50.f,50.f}) {
            common::ControlCommand cmd{speed,0}; cmd.position_control=true; cmd.position_cm=distance;
            for(bool explicit_deny:{false,true}) {
                const auto filtered=explicit_deny ? app::SafeCommand(f.c,o,cmd,now,state,false) :
                                                   app::SafeCommand(f.c,o,cmd,now,state);
                ACC_CHECK(!filtered.position_control && acc::Near(filtered.speedCmPerSec,0),"Không cấp quyền calib: vị trí phải thành lệnh dừng");
                Applied(f,backend,filtered,0,filtered.steeringDeg,"Vị trí bị loại không được chạy motor");
            }
            const auto allowed=app::SafeCommand(f.c,o,cmd,now,state,true);
            ACC_CHECK(allowed.position_control && acc::Near(allowed.position_cm,distance),"Quyền calib position: giữ lệnh vị trí hợp lệ");
            Applied(f,backend,allowed,speed,0,"Đối chứng: quyền calib không chặn tất cả vị trí");
            backend.Apply({});
        }
    }
    backend.Stop(); Tail(f);
}

double Held(const acc::Trace &rows,const std::string &kind,long long at) {
    double value=0; bool found=false;
    for(const auto &row:rows) if(row.kind==kind && row.ms<=at) { value=row.num; found=true; }
    if(!found) throw std::runtime_error("Thiếu vết "+kind+" trước mốc "+std::to_string(at));
    return value;
}

void Startup(const std::string &scenario) {
    Fixture f;
    f.c.camera_hz=25; f.c.camera_width=160; f.c.camera_height=96;
    f.c.control_hz=50; f.c.sensor_hz=50; f.c.soft_start_s=3;
    f.c.motor_keepalive_ms=100; f.c.imu_drift_ms=100;
    f.WriteConfig("tof_mapping_confirmed: 1\nsoft_start_s: 3\n"); f.Gate();
    const bool position=scenario=="position", tof=scenario=="tof";
    setenv("UCR_FAKE_CAMERA_GAPS",position ? "" : "0:3000",1);
    if(tof) {
        const auto script=f.dir+"/tof.txt";
        if(!acc::WriteFile(script,"0 125 2000 2000 2000\n4000 2000 2000 2000 2000\n"))
            throw std::runtime_error("Không tạo được kịch bản ToF");
        setenv("UCR_FAKE_TOF_SCRIPT",script.c_str(),1);
    }
    auto seen=std::make_shared<Seen>();
    acc::Trace rows;
    long long cycle=0,delta=0,first=0,end=0;
    std::vector<Frame> frames;
    {
        app::App car(f.c,"auto",f.config);
        car.SetController(std::unique_ptr<app::IController>(new Controller(seen,scenario)));
        car.Init(); car.Start();
        try {
            // Đo chu kỳ trước ảnh đầu; không dùng lệnh dọn đối tượng để chứng minh đã dừng.
            if(!WaitUntil(Clock::now()+std::chrono::seconds(5),[&] { return !car.IsRunning() || app::fake::Millis()>=2500; }))
                throw std::runtime_error("Không tới mốc đo đầu ca");
            if(!car.IsRunning() || car.Failed()) throw std::runtime_error("Tiền đề: App đã thoát trước mốc đo");
            rows=acc::ReadTrace(f.trace);
            std::vector<long long> ticks;
            for(const auto &r:rows) if(r.kind=="servo_angle" && r.ms>=1000 && r.ms<=2400) ticks.push_back(r.ms);
            if(ticks.size()<10) throw std::runtime_error("Tiền đề: thiếu mẫu chu kỳ Actuator");
            cycle=20;
            for(size_t i=1;i<ticks.size();++i) cycle=std::max(cycle,ticks[i]-ticks[i-1]);
            delta=2*cycle+100;
            if(delta>600) throw std::runtime_error("Tiền đề: máy quá chậm cho cửa sổ tăng mềm");
            if(!WaitUntil(Clock::now()+std::chrono::seconds(5),[&] {
                std::lock_guard<std::mutex> lock(seen->mutex); return !seen->frames.empty() || !car.IsRunning();
            })) throw std::runtime_error("Không nhận được ảnh đầu");
            {
                std::lock_guard<std::mutex> lock(seen->mutex);
                if(seen->frames.empty()) throw std::runtime_error("Tiền đề: không có ảnh đầu thực tế");
                first=seen->frames.front().ms;
            }
            if(!position && (first<3000 || first>3500)) throw std::runtime_error("Tiền đề: ảnh đầu không ở sau ngắt 0–3 s (first="+std::to_string(first)+" ms)");
            end=position ? 4500 : first+3000+delta+600;
            if(!WaitUntil(Clock::now()+std::chrono::seconds(8),[&] { return !car.IsRunning() || app::fake::Millis()>=end; }))
                throw std::runtime_error("Không đủ cửa sổ quan sát tăng mềm");
            if(!car.IsRunning() || car.Failed()) throw std::runtime_error("App thoát trước hết quan sát");
            { std::lock_guard<std::mutex> lock(seen->mutex); frames=seen->frames; }
            const auto stop_at=app::fake::Millis();
            car.RequestStop(); car.Wait();
            rows=acc::ReadTrace(f.trace);
            // Chỉ xét vết trước dọn tiến trình, đọc sau khi luồng đã dừng để tránh dòng ghi dở.
            rows.erase(std::remove_if(rows.begin(),rows.end(),[&](const acc::TraceLine &r) { return r.ms>=stop_at; }),rows.end());
        } catch(...) { car.RequestStop(); car.Wait(); throw; }
    }
    Tail(f);
    const auto csv=acc::ReadCsv(acc::FindRunDir(f.c.log_dir,"_auto")+"/log.csv");
    const int state_column=csv.Col("state");
    bool no_frame=false,fresh_sensors=false;
    if(state_column<0 || csv.rows.empty()) throw std::runtime_error("Thiếu CSV/state cho tiền đề ảnh/cảm biến");
    for(size_t i=0;i<csv.rows.size();++i) {
        if(csv.Num(i,"frame_id")==0) {
            no_frame=true;
            ACC_CHECK(csv.Num(i,"image_age_ms")==-1 && csv.rows[i][state_column]=="MAT_ANH" && acc::Near(csv.Num(i,"cmd_speed"),0),
                      "J: chưa từng có ảnh thì frame_id=0, tuổi=-1, MAT_ANH và lệnh 0");
        }
        if(csv.Num(i,"frame_id")>0 && csv.Num(i,"image_age_ms")>=0 && csv.Num(i,"image_age_ms")<500 &&
           csv.Num(i,"tof0")>0 && csv.Num(i,"tof1")>0) fresh_sensors=true;
    }
    ACC_CHECK(fresh_sensors,"Tiền đề: phải có ảnh và ToF hợp lệ, tránh luôn dừng vì thiếu cảm biến");
    if(position) {
        ACC_CHECK(frames.size()>10,"Tiền đề auto vị trí: đã có nhiều ảnh mới");
        for(const auto &r:rows) if(r.kind=="motor_speed") ACC_CHECK(acc::Near(r.num,0),"E6: auto không được chạy lệnh vị trí");
        return;
    }
    ACC_CHECK(no_frame,"Phải quan sát được thời gian chưa có ảnh đầu");
    const auto frame_period=ObservedPeriodMs(frames,{});
    ACC_CHECK(frame_period>0 && frame_period<=250,"Tiền đề: nhịp ảnh thực đủ cho cửa sổ tăng mềm");
    std::printf("Do dau ca: cycle=%lld delta=%lld first=%lld frame=%lld\n",cycle,delta,first,frame_period);
    bool intermediate=false,full=false;
    for(const auto &r:rows) if(r.kind=="motor_speed") {
        const double magnitude=std::abs(r.num);
        ACC_CHECK(std::isfinite(r.num),"Tốc độ hữu hạn");
        if(r.ms<first) ACC_CHECK(acc::Near(magnitude,0),"Chưa có ảnh đầu: giữ motor 0");
        const double ceiling=20*std::max(0.0,std::min(1.0,double(r.ms-first+delta)/3000));
        ACC_CHECK(magnitude<=ceiling+1e-3,"Ảnh đầu: không vượt trần tuyến tính từ 0");
        if(magnitude>1 && magnitude<18) intermediate=true;
        if(r.ms>=first+3000 && acc::Near(magnitude,20)) full=true;
        if(tof && r.ms<4000) ACC_CHECK(acc::Near(magnitude,0),"ToF gần vẫn chặn tăng mềm");
        if(magnitude>1e-3) ACC_CHECK((r.num<0)==(scenario=="reverse"),"Tăng mềm giữ đúng chiều");
    }
    ACC_CHECK(intermediate && full,"Phải có cả tốc độ trung gian và tốc độ đầy");
    for(long long offset:{1500LL,2250LL}) {
        const double value=std::abs(Held(rows,"motor_speed",first+offset));
        // ToF an toàn lại từ 4 s, các điểm đo đều sau đó ít nhất 0,5 s.
        ACC_CHECK(value>=20*double(offset-delta)/3000-1e-3 && value<=20*double(offset+delta)/3000+1e-3,
                  "Ảnh đầu: tốc độ phải tăng theo thời gian, không chỉ đứng yên rồi nhảy");
    }
    ACC_CHECK(acc::Near(std::abs(Held(rows,"motor_speed",end-100)),20),"Hết 3 s tăng mềm: đạt lệnh ±20");
}
}

int main(int argc,char **argv) {
    if(argc!=2) return 2;
    try {
        ClearFake();
        const std::string scenario=argv[1];
        if(scenario=="defaults") Defaults();
        else if(scenario=="position_default") PositionDenied();
        else if(scenario=="forward" || scenario=="reverse" || scenario=="tof" || scenario=="position") Startup(scenario);
        else return 2;
    } catch(const std::exception &e) { ACC_CHECK(false,e.what()); }
    return acc::Finish(argv[1]);
}
