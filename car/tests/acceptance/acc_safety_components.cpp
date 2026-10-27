// danh sách ca đã duyệt CFG01/02/05–10, B01–10, R06/07/10/13/15–17, A03/04, S12/13.
// Kỳ vọng lấy từ phiếu và quyết định leader; nối Filter/SafeCommand -> Backend -> vết lệnh.
#include "acc_fixture.h"
#include "app/Calibration.h"
#include "acc_recovery_timing.h"
#include <array>
#include <limits>

namespace {
using namespace acc_fixture;
using Ms=std::chrono::milliseconds;

// CFG01/02/05–10: Config::Load, gồm quan hệ timeout thực và việc thiếu khóa dùng mặc định.
void ConfigCases() {
    Fixture f;
    auto load=[&](const std::string &text) { f.WriteConfig(text); common::Config c; c.Load(f.config); return c; };
    auto c=load("");
    ACC_CHECK(acc::Near(c.max_speed_cms,25,1e-3) && c.camera_resume_max_s==2 && c.soft_start_s==1,
              "CFG01: mặc định phải là 25/2/1");
    ACC_CHECK(acc::Near(c.resume_stable_s,2) && c.tof_mapping_confirmed==0,"CFG07/08: mặc định ổn định 2 s/nguyên 0");
    for(const auto v:{"0.1","25","100"}) {
        c=load(std::string("max_speed_cms: ")+v+"\n");
        ACC_CHECK(acc::Near(c.max_speed_cms,std::stof(v),1e-3),"CFG02: nhận cả hai biên hợp lệ");
    }
    for(int timeout:{200,500}) {
        for(const auto r:{"0.501","2","5"}) {
            c=load("image_timeout_ms: "+std::to_string(timeout)+"\ncamera_resume_max_s: "+r+"\n");
            ACC_CHECK(acc::Near(c.camera_resume_max_s,std::stof(r)),"CFG05: nhận R hợp lệ theo timeout");
        }
    }
    for(const auto s:{"0.2","1","2","10"}) {
        c=load(std::string("soft_start_s: ")+s+"\n");
        ACC_CHECK(acc::Near(c.soft_start_s,std::stof(s)),"E3/CFG06: thời gian trong [0.2,10]");
    }
    for(int timeout:{200,500}) for(const auto stable:{"0.5","0.501","1","2.0","29.999","30.0"}) {
        c=load("image_timeout_ms: "+std::to_string(timeout)+"\nresume_stable_s: "+stable+"\n");
        ACC_CHECK(acc::Near(c.resume_stable_s,std::stof(stable)),"CFG07: W trong [0.5,30], độc lập timeout");
    }
    for(const auto flag:{"0","1"}) {
        c=load(std::string("tof_mapping_confirmed: ")+flag+"\n");
        ACC_CHECK(c.tof_mapping_confirmed==std::stoi(flag),"CFG08: chỉ cờ số nguyên 0/1");
    }
    for(const auto text:{"max_speed_cms: 10\ncamera_resume_max_s: 1\nsoft_start_s: 0.5\nresume_stable_s: 0.7\n",
                          "max_speed_cms: 25\ncamera_resume_max_s: 3\nsoft_start_s: 2\nresume_stable_s: 2\n"}) {
        c=load(text);
        ACC_CHECK(c.loaded_text==common::ConfigText(f.config),"CFG10: bản sao toàn bộ config phải khớp byte");
    }
}

// B01–08/A04: từng lỗi rear/front tách biệt; dùng hai phía 0.5x/2x, tuyệt đối không kiểm đúng biên.
void RearCases() {
    Fixture f; app::Backend b(f.c); b.Init();
    const auto now=Clock::now();
    for(bool permuted:{false,true}) {
        f.c.tof_names=permuted ? std::vector<std::string>{"left","front","right","rear"} :
                                std::vector<std::string>{"front","rear","left","right"};
        for(bool reverse:{false,true}) {
            const int slot=reverse ? f.c.RearSlot() : f.c.FrontSlot();
            const float request=reverse ? -20.f : 20.f;
            for(int fault=0;fault<7;++fault) {
                auto o=Good(f.c,now);
                auto &r=o.tof[slot];
                if(fault==0) r.distanceMm=f.c.tof_brake_mm/2;
                if(fault==1) r.distanceMm=-1;
                if(fault==2) r.valid=false;
                if(fault==3) r={};
                if(fault==4) r.timedOut=true;
                if(fault==5) r.updatedAt=now-Ms(2*f.c.tof_timeout_ms);
                if(fault==6) r.updatedAt=now-Ms(f.c.tof_timeout_ms/2);
                std::string state;
                auto cmd=app::SafeCommand(f.c,o,{request,10},now,state);
                Applied(f,b,cmd,fault==6 ? request : 0,10,"B01–04/06/A04 lỗi "+std::to_string(fault));
                if(fault!=6) ACC_CHECK(state==(reverse ? "PHANH_TOF_SAU" : "PHANH_TOF"),"Sai trạng thái phanh theo hướng");
                o=Good(f.c,now);
                auto &other=o.tof[reverse ? f.c.FrontSlot() : f.c.RearSlot()];
                if(fault==0) other.distanceMm=f.c.tof_brake_mm/2;
                if(fault==1) other.distanceMm=-1;
                if(fault==2) other.valid=false;
                if(fault==3) other={};
                if(fault==4) other.timedOut=true;
                if(fault==5) other.updatedAt=now-Ms(2*f.c.tof_timeout_ms);
                cmd=app::SafeCommand(f.c,o,{request,10},now,state);
                Applied(f,b,cmd,request,10,"B05/07: không dùng cảm biến phía đối diện");
                if(!reverse) ACC_CHECK(state!="PHANH_TOF_SAU","I/B05: tiến với rear hỏng/hết giờ không phanh sau");
            }
        }
        auto both=Good(f.c,now); std::string both_state;
        for(int slot:{f.c.FrontSlot(),f.c.RearSlot()}) {
            both.tof[slot].distanceMm=f.c.tof_brake_mm/2;
            both.tof[slot].updatedAt=now-Ms(f.c.tof_timeout_ms/2);
        }
        Applied(f,b,app::SafeCommand(f.c,both,{20,10},now,both_state),0,10,"B11: hai phía gần vẫn phanh front");
        ACC_CHECK(both_state=="PHANH_TOF" && both_state!="PHANH_TOF_SAU","B11: rear không chiếm nguyên nhân phanh tiến");
    }
    auto o=Good(f.c,now);
    o.tof[f.c.RearSlot()].distanceMm=f.c.tof_brake_mm/2;
    f.c.tof_safety_enabled=0; std::string state;
    Applied(f,b,app::SafeCommand(f.c,o,{-20,10},now,state),-20,10,"B08: tắt riêng ToF");
    o.captured_at=now-Ms(2*f.c.image_timeout_ms);
    Applied(f,b,app::SafeCommand(f.c,o,{-20,10},now,state),0,0,"B08: mất ảnh vẫn dừng/tâm");
    f.c.tof_safety_enabled=1;
    for(bool rear:{false,true}) {
        o=Good(f.c,now); o.tof[rear ? f.c.RearSlot() : f.c.FrontSlot()].distanceMm=f.c.tof_brake_mm/2;
        common::ControlCommand position{rear ? -20.f : 20.f,10}; position.position_control=true; position.position_cm=rear ? -50 : 50;
        Applied(f,b,app::SafeCommand(f.c,o,position,now,state,true),0,10,"A04/E6: vị trí calib được cấp quyền vẫn qua ToF");
    }
    for(float v:{-40.f,-30.f,30.f,40.f}) {
        o=Good(f.c,now);
        Applied(f,b,app::SafeCommand(f.c,o,{v,100},now,state),v<0 ? -25 : 25,20,"B09: trần cả hai chiều");
    }
    b.Stop(); Tail(f);
}

// H/R06/07/10/13/15–17: không phím; ảnh mới liên tục đủ W rồi mới tăng mềm.
void RecoveryCases() {
    Fixture f; app::Backend b(f.c); b.Init();
    const auto base=Clock::now();
    app::CameraRecovery recovery;
    auto tick=[&](int now_ms,int frame_ms,float request,float want,int expected_steer=10,int rear_fault=-1) {
        const auto now=base+Ms(now_ms);
        auto o=Good(f.c,now); o.captured_at=base+Ms(frame_ms); o.frame_id=frame_ms+1;
        auto &rear=o.tof[f.c.RearSlot()];
        if(rear_fault==0) rear.distanceMm=f.c.tof_brake_mm/2;
        if(rear_fault==1) rear.distanceMm=-1;
        if(rear_fault==2) rear.valid=false;
        if(rear_fault==3) rear={};
        if(rear_fault==4) rear.timedOut=true;
        if(rear_fault==5) rear.updatedAt=now-Ms(2*f.c.tof_timeout_ms);
        std::string state;
        auto cmd=recovery.Filter(f.c,o,{request,10},now,state);
        cmd=app::SafeCommand(f.c,o,cmd,now,state);
        Applied(f,b,cmd,want,now_ms-frame_ms>=f.c.image_timeout_ms ? 0 : expected_steer,"R: mốc "+std::to_string(now_ms));
        ACC_CHECK(state!="DUNG_CHOT_CAN_XAC_NHAN","R14: không xuất trạng thái đã bỏ");
        return state;
    };
    // E4: các ca dưới kiểm phục hồi sau khi giai đoạn tăng mềm ban đầu đã hoàn tất.
    auto warm=[&] {
        recovery=app::CameraRecovery();
        for(int ms=-2000;ms<0;ms+=100) {
            const auto now=base+Ms(ms); auto o=Good(f.c,now); std::string state;
            recovery.Filter(f.c,o,{20,10},now,state);
        }
    };
    warm(); tick(0,0,20,20);
    // R01: đồng hồ truyền vào giữ trần logic 500 ms, không cộng dung sai lịch của test App.
    ACC_CHECK(f.c.image_timeout_ms==500,"R01: giữ timeout ảnh mặc định 500 ms");
    tick(499,0,20,20);
    ACC_CHECK(tick(500,0,20,0)=="MAT_ANH","R19: tuổi đúng 500 ms đã là ảnh cũ");
    ACC_CHECK(tick(501,0,20,0)=="MAT_ANH","R01: tuổi ảnh 501 ms phải dừng/tâm ngay");
    tick(2300,0,20,0);
    for(int ms=2400;ms<4400;ms+=100)
        ACC_CHECK(tick(ms,ms,20,0,0)=="DUNG_CHO_ANH_ON_DINH","H/R06: chưa đủ W phải giữ 0/tâm");
    tick(4400,4400,20,0);
    for(int ms=4500;ms<=5500;ms+=100) tick(ms,ms,20,std::min(20.f,(ms-4400)*0.02f));

    // R10/R13: lặp ảnh rồi quá tuổi khi đang chờ phải đặt lại toàn bộ W.
    warm();
    tick(0,0,20,20); tick(501,0,20,0); tick(2300,0,20,0);
    for(int ms=2400;ms<=3400;ms+=100) tick(ms,ms,20,0,0);
    tick(3600,3400,20,0,0); tick(3800,3400,20,0,0); tick(4001,3400,20,0,0);
    for(int ms=4100;ms<6100;ms+=100) tick(ms,ms,20,0,0);
    tick(6100,6100,20,0);
    for(int ms=6200;ms<=7200;ms+=100) tick(ms,ms,20,std::min(20.f,(ms-6100)*0.02f));

    for(int direction:{-1,1}) {
        warm();
        tick(0,0,20.f*direction,20.f*direction);
        tick(501,0,20.f*direction,0);
        tick(600,600,20.f*direction,0); // R02: mất đúng 0,6 s.
        tick(850,850,20.f*direction,5.f*direction);
        tick(1100,1100,-20.f*direction,-10.f*direction);
        tick(1350,1350,4,3); // R15: yêu cầu mới là trần.
        tick(1400,1400,0,0);
        ACC_CHECK(tick(1450,1450,-20,0,10,0)=="PHANH_TOF_SAU","R16: rear gần phanh khi tăng mềm lùi");
        tick(1700,1700,-20,-20);
    }
    // R03: kiểm cả trạng thái, xe luôn dừng cũng không được đạt vì phải có mẫu +5 rồi +20.
    warm(); f.c.resume_stable_s=0.7f;
    tick(0,0,20,20); tick(500,0,20,0);
    ACC_CHECK(tick(1700,1700,20,0)!="DUNG_CHO_ANH_ON_DINH","R03: mất ngắn không vào chờ ổn định");
    ACC_CHECK(tick(1950,1950,20,5)!="DUNG_CHO_ANH_ON_DINH","R03: tăng ngay, không chờ W");
    // Giữ ảnh liên tục: nhảy 750 ms tới mốc cuối sẽ vô tình tạo một lần mất ảnh mới.
    tick(2200,2200,20,10); tick(2450,2450,20,15);
    tick(2700,2700,20,20);
    // R16b: cùng từng lỗi rear, tách mất ngắn và mất dài đã chờ đủ W; phải có lệnh âm trước lỗi.
    for(int gap:{600,2300}) for(int fault=1;fault<=5;++fault) {
        warm();
        tick(0,0,-20,-20); tick(500,0,-20,0);
        int ramp=gap;
        if(gap>2000) {
            ramp=gap+int(std::ceil(double(f.c.resume_stable_s)*1000));
            for(int ms=gap;ms<ramp;ms+=100)
                ACC_CHECK(tick(ms,ms,-20,0,0)=="DUNG_CHO_ANH_ON_DINH","R16b: chờ đủ W sau mất dài");
        }
        tick(ramp,ramp,-20,0);
        tick(ramp+250,ramp+250,-20,-5);
        ACC_CHECK(tick(ramp+300,ramp+300,-20,0,10,fault)=="PHANH_TOF_SAU","R16b: lỗi rear phải phanh trong tăng mềm");
        ACC_CHECK(tick(ramp+400,ramp+400,-20,0,10,fault)=="PHANH_TOF_SAU","R16b: giữ phanh khi lỗi còn hiệu lực");
    }
    // R13/R19: một khung không được tính là chuỗi; tuổi đúng ngưỡng đặt lại W.
    f.c.resume_stable_s=2;
    for(int age:{499,500,501}) {
        warm();
        tick(0,0,20,20); tick(500,0,20,0); tick(2300,0,20,0);
        tick(2400,2400,20,0,0);
        ACC_CHECK(tick(2400+age,2400,20,0,0)==(age<500 ? "DUNG_CHO_ANH_ON_DINH" : "MAT_ANH"),"R19: 499 mới, 500/501 cũ lúc chờ");
        if(age<500) ACC_CHECK(tick(4400,2400,20,0,0)=="MAT_ANH","R13: thời gian trôi với một khung không đủ W");
        const int fresh=age<500 ? 4500 : 3000;
        for(int ms=fresh;ms<fresh+2000;ms+=100) tick(ms,ms,20,0,0);
        tick(fresh+2000,fresh+2000,20,0);
        tick(fresh+2250,fresh+2250,20,5);
        tick(fresh+2500,fresh+2500,20,10); tick(fresh+2750,fresh+2750,20,15);
        tick(fresh+3000,fresh+3000,20,20);
    }
    recovery=app::CameraRecovery();
    auto never=Good(f.c,base); never.frame_id=0; std::string state;
    auto cmd=recovery.Filter(f.c,never,{20,10},base+Ms(3000),state);
    ACC_CHECK(state!="DUNG_CHO_ANH_ON_DINH","R17: chưa từng có ảnh không vào chờ ổn định");
    Applied(f,b,app::SafeCommand(f.c,never,cmd,base+Ms(3000),state),0,0,"R17: chưa từng có ảnh");
    tick(3100,3100,20,0);
    tick(3350,3350,20,5); tick(3600,3600,20,10);
    tick(3850,3850,20,15); tick(4100,4100,20,20);
    b.Stop(); Tail(f);
}

// B10: chiều áp trước trim; dừng không đảo trim, kể cả khi yêu cầu góc vượt giới hạn.
void ServoCases() {
    for(int invert:{0,1}) {
        Fixture f; f.c.servo_invert=invert; app::Backend b(f.c); b.Init();
        auto now=Clock::now(); auto o=Good(f.c,now); std::string state;
        for(int angle:{-100,100}) Applied(f,b,app::SafeCommand(f.c,o,{20,angle},now,state),20,angle<0 ? -20 : 20,"B10");
        b.Stop(); Tail(f);
    }
}

// S12/13: từng mục lỗi phải passed=0, dấu vân tay/config/bản giả phải khớp.
void SmokeResults() {
    Fixture f; const auto text=common::ConfigText(f.config);
    const std::array<const char*,8> fields{{"camera","sensors","servo","tof_brake","camera_brake","encoder_moving","kill9","encoder_stopped"}};
    auto result=[&](const std::array<bool,8> &v) {
        common::SmokeResult(f.c.log_dir+"/smoke_gate.json",text,v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7]);
    };
    for(int bad=-1;bad<8;++bad) {
        std::array<bool,8> flags{{true,true,true,true,true,true,true,true}};
        if(bad>=0) flags[bad]=false;
        result(flags);
        cv::FileStorage in(f.c.log_dir+"/smoke_gate.json",cv::FileStorage::READ);
        ACC_CHECK(in.isOpened(),"S12: phải ghi JSON đọc được");
        ACC_CHECK(std::string(in["source_id"])==common::SourceId(),"S12: source_id");
        ACC_CHECK(std::string(in["config_text"])==text && int(in["simulated"])==1,"S12: config/bản giả");
        for(size_t i=0;i<fields.size();++i) ACC_CHECK(int(in[fields[i]])==int(flags[i]),"S12: kết quả từng mục");
        ACC_CHECK(int(in["passed"])==int(bad<0),"S12: một mục lỗi không thể toàn bộ ĐẠT");
        ACC_CHECK(common::SmokePassed(f.c,f.config)==(bad<0),"S13: gate từng mục lỗi");
    }
    for(int mismatch=0;mismatch<4;++mismatch) {
        cv::FileStorage out(f.c.log_dir+"/smoke_gate.json",cv::FileStorage::WRITE|cv::FileStorage::FORMAT_JSON);
        out<<"source_id"<<(mismatch==0 ? "khac-nguon" : common::SourceId())
           <<"config_text"<<(mismatch==1 ? text+"\n" : text)
           <<"simulated"<<(mismatch==2 ? 0 : 1)<<"passed"<<1;
        // Mỗi đối chứng chỉ sai một điều kiện, tránh thiếu trường che lỗi kiểm định danh.
        for(size_t i=0;i<fields.size();++i)
            if(mismatch!=3 || i!=0) out<<fields[i]<<1;
        out.release();
        ACC_CHECK(!common::SmokePassed(f.c,f.config),"S13: không chấp nhận dấu vân tay sai hoặc thiếu mục");
    }
    unlink((f.c.log_dir+"/smoke_gate.json").c_str());
    ACC_CHECK(!common::SmokePassed(f.c,f.config),"S13: thiếu kết quả phải chặn");
}

// Giữ đầu ghi pipe mở: Tick không được bị hủy vì stdin của CTest đã EOF.
class OpenStdin {
    int saved_=-1, write_=-1;
public:
    OpenStdin() {
        int channel[2];
        if(pipe(channel)!=0) throw std::runtime_error("Không tạo được stdin cho calib");
        saved_=dup(STDIN_FILENO);
        if(saved_<0 || dup2(channel[0],STDIN_FILENO)<0) {
            close(channel[0]); close(channel[1]);
            if(saved_>=0) close(saved_);
            throw std::runtime_error("Không giữ được stdin cho calib");
        }
        close(channel[0]); write_=channel[1];
        std::clearerr(stdin);
    }
    ~OpenStdin() {
        dup2(saved_,STDIN_FILENO); close(saved_); close(write_); std::clearerr(stdin);
    }
};

// A03/A06/B09, phân xử C: mất ảnh xóa lệnh giữ; ảnh mới vẫn 0 tới phím chạy mới.
void CalibrationSafety() {
    OpenStdin input;
    Fixture f; app::Backend b(f.c); b.Init();
    for(float requested:{20.f,40.f}) for(int gap:{600,2300}) for(bool early:{false,true}) {
        f.c.calib_speed_cms=requested;
        app::Calibration calib(f.c,"motor",f.dir,f.config,"",false);
        common::Snapshot s;
        auto tick=[&](int key,Clock::time_point now,bool fresh) {
            if(fresh) s.observation=Good(f.c,now);
            const auto raw=calib.Tick(s,key,now);
            std::string state;
            const auto safe=app::SafeCommand(f.c,s.observation,raw,now,state);
            return safe;
        };
        // Nhịp chuẩn bị không có phím; y ở nhịp sau. Mỗi phím là một sự kiện riêng.
        b.Apply(tick(-1,Clock::now(),true));
        const bool moving=WaitUntil(Clock::now()+std::chrono::seconds(3),[&] {
            if(calib.Done() || calib.Failed()) return true;
            b.Apply(tick('y',Clock::now(),true));
            double v=0;
            return acc::LastNum(acc::ReadTrace(f.trace),"motor_speed",v) && v>1e-3;
        });
        double speed=0;
        const bool started=moving && !calib.Done() && !calib.Failed() &&
            acc::LastNum(acc::ReadTrace(f.trace),"motor_speed",speed) && speed>1e-3;
        ACC_CHECK(started,"A03/A06: thiết lập phải có motor khác 0 ở Backend, không Done/Failed/EOF");
        if(!started) throw std::runtime_error("Thiếu phím/chuẩn bị calib; chưa kiểm được mất ảnh");
        ACC_CHECK(acc::Near(speed,std::min(25.f,requested),1e-3),"B09: calib bị chặn đúng trần trước mất ảnh");
        const auto last=s.observation.captured_at;
        const auto stale_at=last+Ms(f.c.image_timeout_ms+1);
        auto cmd=tick(-1,stale_at,false);
        Applied(f,b,cmd,0,0,"A03: ảnh cũ xóa lệnh giữ");
        cmd=tick('y',last+Ms(gap-1),false);
        Applied(f,b,cmd,0,0,"A06: y khi ảnh cũ bị bỏ");
        const auto returned=last+Ms(gap);
        cmd=tick(-1,returned,true);
        Applied(f,b,cmd,0,0,"A06: ảnh có lại chưa bấm vẫn đứng yên");
        // Tick công khai nhận now: tiến đồng hồ xác định, không cần ngủ W+S ngoài đời.
        const int hold_ms=early ? 0 : int((f.c.resume_stable_s+f.c.soft_start_s)*1000)+200;
        for(int elapsed=0;elapsed<=hold_ms;elapsed+=100) {
            cmd=tick(-1,returned+Ms(elapsed),true);
            Applied(f,b,cmd,0,0,"A03/A06: chưa phím mới giữ 0, kể cả lâu hơn W+S");
            ACC_CHECK(!calib.Done() && !calib.Failed(),"A06: không được hủy do EOF trước khi bấm lại");
        }
        const auto resumed=returned+Ms(hold_ms+10);
        cmd=tick('y',resumed,true);
        Applied(f,b,cmd,std::min(25.f,requested),0,"A06/B09: phím mới chạy ngay, không chốt/tăng mềm");
        cmd=tick('q',resumed+Ms(10),true);
        ACC_CHECK(calib.Done(),"A01: q kết thúc bước");
        Applied(f,b,cmd,0,0,"A01: q dừng/tâm");
    }
    b.Stop(); Tail(f);
}

// S03/04: chỉ kiểm phần camera/cảm biến, không tự xác nhận servo hoặc bánh kê.
void SmokeSensors() {
    OpenStdin input;
    for(int fault=0;fault<12;++fault) {
        Fixture f;
        if(fault==11) f.c.tof_names={"left","front","right","rear"};
        app::Backend b(f.c); b.Init();
        app::Calibration smoke(f.c,"smoke",f.dir,f.config,f.kill_file,false);
        const auto base=Clock::now();
        for(int i=0;i<60 && !smoke.Done();++i) {
            const auto now=base+Ms(100*i);
            common::Snapshot s; s.observation=Good(f.c,now); s.observation.frame_id=1+i;
            s.fps=2*f.c.smoke_min_fps;
            if(fault==1) { s.observation.frame_id=0; s.observation.frame.release(); }
            if(fault==2) s.observation.frame=cv::Mat(f.c.camera_height,f.c.camera_width/2,CV_8UC3);
            if(fault==3) s.observation.frame=cv::Mat(f.c.camera_height/2,f.c.camera_width,CV_8UC3);
            if(fault==4) s.fps=f.c.smoke_min_fps/2;
            if(fault==5) s.observation.imu_valid=false;
            if(fault>=6 && fault<=9) s.observation.tof[fault-6].valid=false;
            if(fault==10) s.observation.tof[2].distanceMm=-1;
            const auto cmd=smoke.Tick(s,-1,now);
            ACC_CHECK(acc::Near(cmd.speedCmPerSec,0,1e-3),"S06: chưa xác nhận bánh kê không được yêu cầu motor");
            std::string state;
            Applied(f,b,app::SafeCommand(f.c,s.observation,cmd,now,state),0,cmd.steeringDeg,"S03/04/06");
        }
        common::Snapshot s; s.observation=Good(f.c,base+Ms(6100));
        smoke.Tick(s,'q',base+Ms(6100));
        cv::FileStorage result(f.dir+"/smoke.json",cv::FileStorage::READ);
        ACC_CHECK(result.isOpened(),"S03/04: hủy cũng phải ghi kết quả từng phần");
        if(result.isOpened()) {
            ACC_CHECK(int(result["camera"])==int(fault<1 || fault>4),"S03: camera phải đúng ảnh/kích thước/FPS");
            if(fault==0 || fault>=5) ACC_CHECK(int(result["sensors"])==int(fault==0 || fault==11),"S04: mỗi IMU/ToF lỗi làm cụm không ĐẠT");
            ACC_CHECK(int(result["passed"])==0 && int(result["servo"])==0,"S05/06: chưa xác nhận không được tự ĐẠT");
        }
        b.Stop(); Tail(f);
    }
}

// S05/06: bốn xác nhận servo theo phiếu in, chưa xác nhận bánh kê; kiểm góc cuối qua Backend.
void SmokeServo() {
    OpenStdin input;
    Fixture f; f.c.servo_invert=1;
    app::Backend b(f.c); b.Init();
    app::Calibration smoke(f.c,"smoke",f.dir,f.config,f.kill_file,false);
    const auto base=Clock::now();
    std::vector<int> angles;
    for(int i=0;i<45 && !smoke.Done();++i) {
        const auto now=base+Ms(i*100);
        common::Snapshot s; s.observation=Good(f.c,now); s.observation.frame_id=i+1;
        s.fps=2*f.c.smoke_min_fps;
        const int key=(i>=30 && i<=33) ? 'y' : -1;
        auto cmd=smoke.Tick(s,key,now);
        ACC_CHECK(acc::Near(cmd.speedCmPerSec,0,1e-3),"S06: bốn xác nhận servo chưa phải xác nhận bánh kê");
        std::string state;
        cmd=app::SafeCommand(f.c,s.observation,cmd,now,state);
        Applied(f,b,cmd,0,cmd.steeringDeg,"S05: servo qua chiều và trim");
        if(angles.empty() || angles.back()!=cmd.steeringDeg) angles.push_back(cmd.steeringDeg);
    }
    ACC_CHECK(angles==std::vector<int>({0,-10,10,0}),"S05: đúng thứ tự tâm/âm 10/dương 10/tâm, không tự chạy tiếp");
    common::Snapshot s; s.observation=Good(f.c,base+Ms(4600));
    smoke.Tick(s,'q',base+Ms(4600));
    cv::FileStorage result(f.dir+"/smoke.json",cv::FileStorage::READ);
    ACC_CHECK(result.isOpened(),"S05: phải có kết quả khi thoát");
    if(result.isOpened()) ACC_CHECK(int(result["passed"])==0 && int(result["encoder_moving"])==0,"S06: thoát trước kê bánh không được ĐẠT");
    b.Stop(); Tail(f);
}
}
int main(int argc,char **argv) {
    if(argc!=2) return 2;
    try {
        const std::string scenario=argv[1];
        if(scenario=="config") ConfigCases();
        else if(scenario=="rear") RearCases();
        else if(scenario=="recovery") RecoveryCases();
        else if(scenario=="servo") ServoCases();
        else if(scenario=="smoke_result") SmokeResults();
        else if(scenario=="calib_safety") CalibrationSafety();
        else if(scenario=="smoke_sensors") SmokeSensors();
        else if(scenario=="smoke_servo") SmokeServo();
        else return 2;
    } catch(const std::exception &e) { ACC_CHECK(false,std::string("Ngoại lệ ngoài dự kiến: ")+e.what()); }
    return acc::Finish(argv[1]);
}
