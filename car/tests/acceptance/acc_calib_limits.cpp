// Yêu cầu: giới hạn lượt, xoá góc khi huỷ và mặc định cấu hình trong phiếu hiệu chuẩn H1/H3/H4, L1/L3/L4.
// Chỉ Tick/Config công khai -> SafeCommand -> Backend -> vết phần cứng; không kiểm mô tả giới hạn nội bộ.
#include "acc_fixture.h"
#include "app/Calibration.h"
#include "acc_recovery_timing.h"
#include <cstdio>

namespace {
using namespace acc_fixture;
using Ms=std::chrono::milliseconds;

class OpenInput {
    int saved_=-1, writer_=-1;
public:
    OpenInput() {
        int channel[2];
        if(pipe(channel)!=0) throw std::runtime_error("Không tạo được stdin mở");
        saved_=dup(STDIN_FILENO);
        if(saved_<0 || dup2(channel[0],STDIN_FILENO)<0) {
            close(channel[0]); close(channel[1]);
            if(saved_>=0) close(saved_);
            throw std::runtime_error("Không thay được stdin");
        }
        close(channel[0]); writer_=channel[1]; std::clearerr(stdin);
    }
    ~OpenInput() {
        dup2(saved_,STDIN_FILENO); close(saved_); close(writer_); std::clearerr(stdin);
    }
};

struct Trial {
    Fixture f;
    app::Backend backend;
    app::Calibration calib;
    common::Snapshot s;
    Clock::time_point start;
    explicit Trial(const std::string &step):backend(f.c),calib(f.c,step,f.dir,f.config) {
        backend.Init();
        // Thời gian chuẩn bị là thời gian thật vì constructor không nhận đồng hồ.
        const bool ready=WaitUntil(Clock::now()+std::chrono::seconds(3),[&] {
            start=Clock::now();
            s.observation=Good(f.c,start);
            s.observation.odom_cm=37; s.observation.yaw_rel_deg=42;
            calib.Tick(s,'d',start);
            const auto cmd=calib.Tick(s,'y',start);
            backend.Apply(cmd);
            return std::abs(cmd.speedCmPerSec)>0.001f || calib.Done() || calib.Failed();
        });
        double speed=0;
        if(!ready || calib.Done() || calib.Failed() ||
           !acc::LastNum(acc::ReadTrace(f.trace),"motor_speed",speed) || speed<=0.001)
            throw std::runtime_error("Tiền đề: phải chạy trước khi kiểm giới hạn Tick");
    }
    common::ControlCommand Tick(Clock::time_point now,int key=-1) {
        std::string state;
        return app::SafeCommand(f.c,s.observation,calib.Tick(s,key,now),now,state);
    }
    void Fresh(Clock::time_point now,double odom=37,double yaw=42) {
        s.observation=Good(f.c,now); s.observation.odom_cm=odom; s.observation.yaw_rel_deg=yaw;
    }
};

// H1/L1: từng giới hạn độc lập, Tick vẫn kiểm lớp thứ hai; lệnh sau huỷ phải 0/trim.
void Limits() {
    OpenInput input;
    for(const std::string step:{"motor","radius"}) {
        for(const std::string cause:{"time_equal","time_after","distance_equal","distance_after","encoder_invalid","encoder_stale","yaw"}) {
            if(cause=="yaw" && step!="radius") continue;
            Trial t(step);
            auto now=t.start+Ms(100);
            t.Fresh(now);
            auto cmd=t.Tick(now);
            Applied(t.f,t.backend,cmd,10,cmd.steeringDeg,"Tiền đề: chưa tới giới hạn vẫn chạy");
            if(cause=="time_equal" || cause=="time_after") {
                now=t.start+Ms(60000+(cause=="time_after" ? 1 : 0)); t.Fresh(now);
            } else if(cause=="distance_equal" || cause=="distance_after") {
                t.Fresh(now,37+200+(cause=="distance_after" ? 0.1 : 0));
            } else if(cause=="encoder_invalid") t.s.observation.odom_valid=false;
            else if(cause=="encoder_stale") t.s.observation.odom_at=now-Ms(600);
            else t.Fresh(now,37,42+360);
            Applied(t.f,t.backend,t.Tick(now),0,0,"Chạm "+cause+": motor 0 và góc về trim");
            now+=Ms(100); t.Fresh(now);
            Applied(t.f,t.backend,t.Tick(now),0,0,"Quan sát tốt lại không phục hồi lượt cũ");
            t.backend.Stop(); Tail(t.f);
        }
        // Biên phía dưới: còn 1 ms / 0.1 cm / 0.1 độ thì không tự huỷ.
        for(const std::string below:{"time","distance","yaw"}) {
            if(below=="yaw" && step!="radius") continue;
            Trial t(step);
            auto now=t.start+Ms(below=="time" ? 59999 : 100);
            t.Fresh(now,below=="distance" ? 236.9 : 37,below=="yaw" ? 401.9 : 42);
            auto cmd=t.Tick(now);
            Applied(t.f,t.backend,cmd,10,cmd.steeringDeg,"Sát dưới "+below+" vẫn chạy");
            t.backend.Stop(); Tail(t.f);
        }
    }
}

// H3/L4: huỷ lặp không phục hồi góc; từng phím mới bắt đầu từ góc tương đối 0.
void CancelKeys() {
    OpenInput input;
    const int keys[]{-1,'a','d','+','-'};
    for(int key:keys) {
        Trial t("motor");
        auto now=t.start+Ms(100); t.Fresh(now);
        for(int i=0;i<4;++i) t.calib.Tick(t.s,'d',now);
        auto cmd=t.Tick(now);
        Applied(t.f,t.backend,cmd,10,20,"Tiền đề: góc cũ 20 độ");
        t.calib.CancelMotion(); t.calib.CancelMotion();
        now+=Ms(100); t.Fresh(now);
        Applied(t.f,t.backend,t.Tick(now),0,0,"Huỷ lặp phải 0/trim");
        now+=Ms(100); t.Fresh(now);
        Applied(t.f,t.backend,t.Tick(now,key),0,key=='a' ? -5 : key=='d' ? 5 : 0,
                "Phím mới không cộng góc trước huỷ");
        t.backend.Stop(); Tail(t.f);
    }
}

// H4/L3: thiếu riêng từng khoá/cả ba chỉ đòi mặc định 200/50/60, không đòi cảnh báo.
void Defaults() {
    Fixture f;
    const std::vector<std::string> keys{"calib_distance_cm","calib_position_cm","calib_max_duration_s"};
    for(int missing=-1;missing<3;++missing) {
        std::string text;
        for(int i=0;i<3;++i) if(missing>=0 && i!=missing) text+=keys[i]+": 1\n";
        f.WriteConfig(text);
        common::Config c; c.Load(f.config);
        const double values[]{c.calib_distance_cm,c.calib_position_cm,c.calib_max_duration_s};
        const double defaults[]{200,50,60};
        for(int i=0;i<3;++i) ACC_CHECK(acc::Near(values[i],missing<0 || missing==i ? defaults[i] : 1),
                                    "Thiếu khoá giữ đúng mặc định: "+keys[i]);
    }
}
}

int main(int argc,char **argv) {
    if(argc!=2) return 2;
    try {
        const std::string scenario=argv[1];
        if(scenario=="tick_limits") Limits();
        else if(scenario=="cancel_keys") CancelKeys();
        else if(scenario=="limit_defaults") Defaults();
        else return 2;
    } catch(const std::exception &e) { ACC_CHECK(false,e.what()); }
    return acc::Finish(argv[1]);
}
