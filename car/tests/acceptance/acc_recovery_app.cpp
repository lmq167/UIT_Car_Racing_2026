// Hiệu chuẩn/an toàn R01–04/R06/R08–12/R14–17, CFG10, S13: App thật với camera/ToF giả và bộ điều khiển tiêm.
// Khung quan sát đủ thì đo captured_at; bỏ khung thì kiểm cận thời gian, không đoán D/t0 chính xác.
#include "acc_fixture.h"
#include "acc_recovery_timing.h"
#include "app/App.h"
#include "app/FakeHardware.h"
#include <mutex>
#include <memory>

namespace {
using namespace acc_fixture;
struct Observed {
    std::mutex mutex;
    std::vector<Frame> frames;
    Clock::time_point origin=Clock::now()-std::chrono::milliseconds(app::fake::Millis());
};
class Controller : public app::IController {
    std::shared_ptr<Observed> seen_;
    std::string scenario_;
public:
    Controller(std::shared_ptr<Observed> seen,std::string scenario):seen_(std::move(seen)),scenario_(std::move(scenario)) {}
    common::ControlCommand step(const common::Observation &o) override {
        {
            std::lock_guard<std::mutex> lock(seen_->mutex);
            if(o.frame_id && (seen_->frames.empty() || o.frame_id!=seen_->frames.back().id))
                seen_->frames.push_back({o.frame_id,std::chrono::duration_cast<std::chrono::milliseconds>(o.captured_at-seen_->origin).count()});
        }
        const auto ms=app::fake::Millis();
        float speed=scenario_=="reverse" ? -20.f : scenario_=="clamp" ? 40.f : 20.f;
        if(scenario_=="lower" && ms>=2450) speed=4;
        if(scenario_=="lower" && ms>=2650) speed=0;
        if(scenario_=="direction" && ms>=2450) speed=-20;
        return {speed,10};
    }
};
void Run(const std::string &scenario) {
    Fixture f;
    const bool gate_case=scenario=="gate" || scenario=="gate_missing";
    f.c.camera_hz=25; f.c.control_hz=100; f.c.sensor_hz=100;
    // R khác mặc định nhưng vẫn chứa ngắt 1200 ms và biên 300 ms của ca cấu hình thay đổi.
    if(scenario=="config_changed") { f.c.max_speed_cms=10; f.c.camera_resume_max_s=1.8f; f.c.soft_start_s=0.5; f.c.image_timeout_ms=200; }
    if(scenario=="stable_changed" || scenario=="below" || scenario=="above" || scenario=="short_long") f.c.resume_stable_s=0.7f;
    // Mất ngắn 1,2 s vẫn dưới R=2 s, đủ cửa sổ thấy dừng dù sanitizer làm trễ lịch.
    std::vector<std::pair<int,int>> gaps{{1000,2200}};
    if(scenario=="below") gaps={{1000,2700}};
    if(scenario=="above" || scenario=="stable_changed" || scenario=="stable_default") gaps={{1000,3300}};
    if(scenario=="twice") gaps={{1000,2200},{2450,3650}};
    if(scenario=="long_short") gaps={{1000,3300},{3600,4800}};
    if(scenario=="short_long") gaps={{998,2198},{2448,4748}};
    if(scenario=="startup") gaps={{0,2500}};
    if(gate_case) gaps.clear();
    const int cycle_ms=1000/f.c.control_hz;
    const int settle_ms=cycle_ms+ScheduleSlackMs;
    if(scenario!="startup") {
        for(const auto &gap:gaps) {
            const auto duration=gap.second-gap.first;
            ACC_CHECK(duration>=1200,"Thiết lập sai: khoảng ngắt sau khi đã có ảnh phải >= 1200 ms");
            if(duration<1200) return;
            if(duration==1200) {
                const bool short_schedule=duration<f.c.camera_resume_max_s*1000-300;
                ACC_CHECK(short_schedule,"Thiết lập sai: ngắt 1200 ms phải < camera_resume_max_s - 300 ms theo cấu hình ca");
                if(!short_schedule) return;
            }
            // Hai phía ngắt có thể lệch một chu kỳ ảnh 60 ms ở container; không đổi dung sai lệnh.
            if(duration<f.c.camera_resume_max_s*1000) {
                const bool separated=duration+2*60<f.c.camera_resume_max_s*1000;
                ACC_CHECK(separated,"Thiết lập sai: cận mất ngắn với chu kỳ ảnh 60 ms phải dưới R");
                if(!separated) return;
            }
            // FrameWindow.last_latest <= gap.first, kể cả khi Controller bỏ khung.
            const auto stop_by=gap.first+f.c.image_timeout_ms+settle_ms;
            ACC_CHECK(stop_by<gap.second,"Thiết lập sai lịch: stop_by="+std::to_string(stop_by)+
                " ms phải < gap.second="+std::to_string(gap.second)+" ms để quan sát dừng trước ảnh có lại");
            if(stop_by>=gap.second) return;
        }
    }
    std::string spec;
    for(auto gap:gaps) spec+=(spec.empty() ? "" : ",")+std::to_string(gap.first)+":"+std::to_string(gap.second);
    setenv("UCR_FAKE_CAMERA_GAPS",spec.c_str(),1);
    if(scenario=="tof") {
        const auto script=f.dir+"/tof.txt";
        acc::WriteFile(script,"0 2000 2000 2000 2000\n2400 125 2000 2000 2000\n3000 2000 2000 2000 2000\n");
        setenv("UCR_FAKE_TOF_SCRIPT",script.c_str(),1);
    }
    auto write_config=[&] { f.WriteConfig("tof_mapping_confirmed: 1\ncamera_hz: 25\ncontrol_hz: 100\nsensor_hz: 100\nservo_trim_deg: 7\nmax_speed_cms: "+
        std::to_string(f.c.max_speed_cms)+"\ncamera_resume_max_s: "+std::to_string(f.c.camera_resume_max_s)+
        "\nsoft_start_s: "+std::to_string(f.c.soft_start_s)+"\nresume_stable_s: "+std::to_string(f.c.resume_stable_s)+
        "\nimage_timeout_ms: "+std::to_string(f.c.image_timeout_ms)+"\n"); };
    write_config();
    if(scenario!="gate_missing") f.Gate();
    if(scenario=="gate") common::SmokeResult(f.c.log_dir+"/smoke_gate.json",common::ConfigText(f.config),false,true,true,true,true,true,true,true);
    auto seen=std::make_shared<Observed>();
    long long end=0;
    acc::Trace running_trace;
    std::vector<Frame> frames;
    bool rejected=false;
    {
        app::App app(f.c,"auto",f.config);
        try {
            app.SetController(std::unique_ptr<app::IController>(new Controller(seen,scenario)));
            app.Init(); app.Start();
            const auto deadline=Clock::now()+std::chrono::seconds(12);
            const bool ready=WaitUntil(deadline,[&] {
                if(!app.IsRunning()) return true;
                if(gate_case) return app::fake::Millis()>=1200;
                std::lock_guard<std::mutex> lock(seen->mutex);
                const auto period=ObservedPeriodMs(seen->frames,gaps);
                if(!period) return false;
                for(auto fr:seen->frames) if(fr.ms>=gaps.back().second) {
                    if(seen->frames.back().ms<RecoveryCheckMs(fr,int((f.c.resume_stable_s+f.c.soft_start_s)*1000),cycle_ms,period)) return false;
                    frames=seen->frames;
                    return true;
                }
                return false;
            });
            ACC_CHECK(ready,"App phải đạt điều kiện quan sát trong hạn 12 s");
            ACC_CHECK(app.IsRunning() || gate_case,"App không được thoát trước khi hoàn tất phép đo");
            if(frames.empty()) { std::lock_guard<std::mutex> lock(seen->mutex); frames=seen->frames; }
            // Không lấy lệnh dừng do chính test dọn App làm bằng chứng đã phục hồi về yêu cầu 0.
            running_trace=acc::ReadTrace(f.trace);
            end=app::fake::Millis()+1;
            app.RequestStop(); app.Wait();
            ACC_CHECK(!app.Failed() || gate_case,"App không được lỗi trong phép phục hồi");
        } catch(const std::exception &e) {
            rejected=true;
            ACC_CHECK(gate_case,std::string("App từ chối ngoài ca gate: ")+e.what());
            if(gate_case) ACC_CHECK(std::string(e.what()).find("smoke")!=std::string::npos,"S13: phải nêu lý do smoke");
            app.RequestStop(); app.Wait();
        }
    }
    int malformed=0; const auto trace=acc::ReadTrace(f.trace,&malformed);
    ACC_CHECK(malformed==0,"Vết lệnh phải có dữ liệu hợp lệ");
    if(gate_case) {
        ACC_CHECK(rejected,"S15/16: gate thiếu/lỗi phải từ chối khởi động App");
        ACC_CHECK(trace.empty(),"S15/16: gate lỗi phải chặn trước mọi thiết bị");
        return;
    }
    ACC_CHECK(!trace.empty(),"Phải có vết khi App chạy");
    ACC_CHECK(acc::Count(trace,"motor_init")==1,"Một phiên App chỉ khởi tạo motor một lần");
    Tail(f);
    const auto speeds=acc::Of(running_trace,"motor_speed");
    for(const auto &l:acc::Of(trace,"motor_speed")) ACC_CHECK(std::isfinite(l.num) && std::fabs(l.num)<=f.c.max_speed_cms+0.001,"Không vượt trần cấu hình");
    ACC_CHECK(acc::Moved(trace),"Phải có lệnh chuyển động làm đối chứng; không được đạt với StopController");
    ACC_CHECK(!frames.empty(),"Phải quan sát được ảnh nhận thật qua IController");
    const auto period_ms=ObservedPeriodMs(frames,gaps);
    ACC_CHECK(period_ms>0,"Thiết lập phải đo được chu kỳ ảnh quan sát ngoài khoảng ngắt");
    std::printf("Chu ky anh quan sat: %lld ms; FPS can duoi: %.2f\n",period_ms,period_ms ? 1000.0/period_ms : 0);
    const auto run=acc::FindRunDir(f.c.log_dir,"_auto");
    const auto csv=acc::ReadCsv(run+"/log.csv"); const int state_col=csv.Col("state");
    ACC_CHECK(state_col>=0 && !csv.rows.empty(),"R14: phải có cột state và dữ liệu log");
    bool saw_stale=false;
    for(size_t i=0;i<csv.rows.size() && state_col>=0;++i) {
        const auto &state=csv.rows[i][state_col];
        ACC_CHECK(state!="DUNG_CHOT_CAN_XAC_NHAN","R14: cấm trạng thái đã bỏ ở mọi giai đoạn");
        if(csv.Num(i,"image_age_ms")>=f.c.image_timeout_ms) {
            saw_stale=true;
            ACC_CHECK(state=="MAT_ANH","R14: ảnh cũ phải đúng MAT_ANH");
            ACC_CHECK(acc::Near(csv.Num(i,"cmd_speed"),0,1e-3),"R14: log ảnh cũ có lệnh 0");
        }
        if(scenario=="below" || scenario=="startup")
            ACC_CHECK(state!="DUNG_CHO_ANH_ON_DINH","R03/R17: mất ngắn/chưa từng có ảnh không vào chờ ổn định");
    }
    // Chưa từng có ảnh không có tuổi ảnh cũ; R17 kiểm 0/tâm trước khung đầu ở vết bên dưới.
    ACC_CHECK(saw_stale || scenario=="startup","R14: đã có ảnh thì phải quan sát được MAT_ANH trong log");
    if(scenario=="startup") {
        if(frames.empty()) return;
        const auto first=frames.front().ms;
        bool full=false;
        for(const auto &l:speeds) {
            if(l.ms<first) ACC_CHECK(acc::Near(l.num,0),"R17: trước ảnh đầu chỉ lệnh 0");
            if(l.ms>=first && l.ms<=first+settle_ms && acc::Near(l.num,20)) full=true;
        }
        for(const auto &l:acc::Of(running_trace,"servo_angle")) if(l.ms<first)
            ACC_CHECK(acc::Near(l.num,7),"R17: chưa có ảnh phải về trim");
        ACC_CHECK(full,"R17: ảnh đầu không tạo chốt/tăng mềm");
        return;
    }
    bool waiting_for_stable=false;
    for(const auto &gap:gaps) {
        Frame before{0,-1},after{0,-1};
        for(auto fr:frames) {
            if(fr.ms<gap.first) before=fr;
            if(fr.ms>=gap.second && !after.id) after=fr;
        }
        ACC_CHECK(before.id && after.id,"Phải có khung trước/sau ngắt để đo D");
        if(!before.id || !after.id) continue;
        if(&gap==&gaps.front()) {
            double moving=0;
            for(const auto &l:speeds) if(l.ms<gap.first) moving=l.num;
            ACC_CHECK(std::fabs(moving)>0.1,"R01: phải đang yêu cầu chạy trước lần mất ảnh đầu");
        }
        const auto window=FrameWindow(before,after,gap.first,gap.second);
        const auto resume_ms=f.c.camera_resume_max_s*1000;
        std::printf("D trong [%lld,%lld] ms; khung %llu -> %llu; t0 trong [%lld,%lld] ms\n",
            window.duration_min,window.duration_max,static_cast<unsigned long long>(before.id),
            static_cast<unsigned long long>(after.id),window.first_earliest,after.ms);
        const bool short_gap=window.duration_max<=resume_ms;
        const bool long_gap=window.duration_min>resume_ms;
        ACC_CHECK(short_gap || long_gap,"Khoảng quan sát phải đủ tách hai phía R; thiếu bằng chứng là lỗi thiết lập");
        if(scenario=="below") ACC_CHECK(short_gap,"R03: toàn bộ khoảng D phải dưới R; kịch bản delta 300 ms");
        if(scenario=="above") ACC_CHECK(long_gap,"R04: toàn bộ khoảng D phải trên R; kịch bản delta 300 ms");
        if(long_gap) waiting_for_stable=true;
        const auto stop_by=window.last_latest+f.c.image_timeout_ms+settle_ms;
        ACC_CHECK(stop_by<gap.second,"Thiết lập phải để lại cửa sổ quan sát dừng trước ảnh có lại");
        bool saw_stop=false, saw_center=false;
        double held=999;
        for(const auto &l:speeds) {
            if(l.ms<=stop_by) { held=l.num; saw_stop=true; }
            if(l.ms>stop_by && l.ms<gap.second) ACC_CHECK(acc::Near(l.num,0),"R01: không có lệnh chạy muộn khi chắc chắn ảnh còn cũ");
        }
        ACC_CHECK(acc::Near(held,0),"R01: quá timeout + một chu kỳ + dung sai lịch phải dừng");
        held=999;
        for(const auto &l:acc::Of(running_trace,"servo_angle")) {
            if(l.ms<=stop_by) { held=l.num; saw_center=true; }
            if(l.ms>stop_by && l.ms<gap.second) ACC_CHECK(acc::Near(l.num,7),"R01: ảnh cũ phải giữ trim");
        }
        ACC_CHECK(acc::Near(held,7),"R01: tới hạn phải về trim");
        ACC_CHECK(saw_stop && saw_center,"R01: phải quan sát được dừng/tâm trong khoảng mất ảnh");
        const auto next_gap=(&gap==&gaps.back()) ? end : gaps[1].first;
        const auto stable_ms=waiting_for_stable ? std::llround(f.c.resume_stable_s*1000) : 0;
        const auto ramp_start=window.first_earliest+stable_ms;
        const auto check_at=RecoveryCheckMs(after,int(f.c.soft_start_s*1000)+int(stable_ms),cycle_ms,period_ms);
        bool ramp_sample=false, have_held=false;
        double final_held=999;
        const double expected=scenario=="lower" ? 0 :
            (scenario=="reverse" || scenario=="direction") ? -20 :
            std::min(double(f.c.max_speed_cms),scenario=="clamp" ? 40.0 : 20.0);
        for(const auto &l:speeds) if(l.ms>=window.first_earliest && l.ms<next_gap) {
            if(l.ms<ramp_start) ACC_CHECK(acc::Near(l.num,0,1e-3),"H/R04/06/10: chưa đủ ảnh ổn định phải giữ 0");
            else {
                double request=scenario=="clamp" ? 40 : 20;
                // Chỉ bù truyền lệnh giữa các luồng; R15 thành phần không bù lịch, tốc độ sai số 1e-3.
                if(scenario=="lower" && l.ms>=2450+settle_ms) request=4;
                if(scenario=="lower" && l.ms>=2650+settle_ms) request=0;
                // Mốc vết lượng tử theo ms; +1 ms cho cận thời điểm, sai số tốc độ chỉ 1e-3.
                const double bound=std::min(double(f.c.max_speed_cms),request*std::min(1.0,double(l.ms-ramp_start+1)/(1000*f.c.soft_start_s)));
                ACC_CHECK(std::fabs(l.num)<=bound+1e-3,"Mọi lệnh trong tăng mềm nằm dưới trần tuyến tính kể từ khung đầu");
                if(l.ms<ramp_start+f.c.soft_start_s*1000 && std::fabs(l.num)>0.1) ramp_sample=true;
                if(l.ms<=check_at) { final_held=l.num; have_held=true; }
                if(l.ms>check_at) ACC_CHECK(acc::Near(l.num,expected),"Sau hạn tăng mềm phải tiếp tục giữ đúng lệnh hiện tại");
                if(scenario=="tof" && l.ms>=2400+settle_ms+1000/f.c.sensor_hz && l.ms<3000) ACC_CHECK(acc::Near(l.num,0),"R12: ToF chặn trong tăng mềm");
                if(scenario=="reverse") ACC_CHECK(l.num<=1e-3,"R16: lùi giữ đúng dấu");
                if(scenario=="direction" && l.ms>=2450+settle_ms) ACC_CHECK(l.num<=1e-3,"R16: đổi chiều theo yêu cầu mới");
            }
        }
        for(const auto &l:acc::Of(running_trace,"servo_angle"))
            if(l.ms>=window.first_earliest && l.ms<ramp_start)
                ACC_CHECK(acc::Near(l.num,7),"H/R06/10: chờ ảnh ổn định phải giữ trim");
        if(next_gap>ramp_start+settle_ms) {
            ACC_CHECK(ramp_sample,"Phải có mẫu tăng mềm khác 0 trước S");
            if(&gap==&gaps.back()) ACC_CHECK(next_gap>check_at,"Phải quan sát qua S và một chu kỳ ảnh đo được");
            if(next_gap>check_at) {
                std::printf("Kiem lenh giu tai %lld ms: %.3f, can %.3f\n",check_at,final_held,expected);
                ACC_CHECK(have_held && acc::Near(final_held,expected),"Sau S phải đạt và giữ lệnh đã chặn trần trước hạn đo");
            }
        }
    }
    if(waiting_for_stable) {
        const int col=state_col;
        bool found=false;
        for(size_t i=0;i<csv.rows.size();++i) if(col>=0 && csv.rows[i][col]=="DUNG_CHO_ANH_ON_DINH") {
            found=true; ACC_CHECK(acc::Near(csv.Num(i,"cmd_speed"),0,1e-3),"H/R14: log chờ ổn định phải có lệnh 0");
        }
        ACC_CHECK(found,"H/R14: log phải ghi rõ DUNG_CHO_ANH_ON_DINH");
    }
    ACC_CHECK(acc::ReadFile(run+"/config.yaml")==common::ConfigText(f.config),"CFG10: bản sao config trong lượt phải khớp");
}
}
int main(int argc,char **argv) {
    if(argc!=2) return 2;
    try { Run(argv[1]); }
    catch(const std::exception &e) { ACC_CHECK(false,std::string("Ngoại lệ: ")+e.what()); }
    return acc::Finish(argv[1]);
}
