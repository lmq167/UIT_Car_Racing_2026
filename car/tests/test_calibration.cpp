#include "app/Safety.h"
#include "app/Calibration.h"
#include "common/Smoke.h"
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <cstdio>
#include <unistd.h>
namespace {
void Check(bool ok,const char *message) { if(!ok) throw std::runtime_error(message); }
}
int main() {
    common::Config c; common::Observation o; std::string state;
    const auto base=common::Clock::now();
    auto at=[&](int ms){return base+std::chrono::milliseconds(ms);};
    auto frame=[&](int ms){o.frame_id++; o.captured_at=at(ms);};
    for(auto &t:o.tof) { t.valid=true; t.distanceMm=2000; t.updatedAt=base; }
    frame(0);
    o.tof[c.RearSlot()].distanceMm=1;
    Check(app::SafeCommand(c,o,{-10,0},base,state).speedCmPerSec==0 && state=="PHANH_TOF_SAU","phanh ToF sau");
    Check(app::SafeCommand(c,o,{10,0},base,state).speedCmPerSec==10,"ToF sau không chặn tiến");
    o.tof[c.RearSlot()].valid=false;
    Check(app::SafeCommand(c,o,{-10,0},base,state).speedCmPerSec==0,"ToF sau lỗi phải dừng");
    app::CameraRecovery recovery;
    common::ControlCommand filtered;
    auto run=[&](int ms){filtered=recovery.Filter(c,o,{20,4},at(ms),state); return filtered.speedCmPerSec;};
    Check(run(0)==20,"khởi đầu có ảnh không cần tăng tốc mềm");
    Check(run(501)==0 && filtered.steeringDeg==0,"mất ảnh dừng và trả góc về trim"); frame(600);
    Check(run(600)==0,"ảnh có lại bắt đầu từ 0");
    Check(filtered.steeringDeg==4,"ảnh có lại: tăng tốc mềm vẫn giữ góc bộ điều khiển");
    for(int ms=601;ms<=1600;ms+=37) { frame(ms); Check(run(ms)<=20.0f*(ms-600)/1000+1e-5,"trần tăng tuyến tính"); }
    frame(1600); Check(run(1600)==20,"đủ soft_start_s đạt lệnh đầy");
    Check(run(2101)==0,"lần ngắt kế tiếp"); frame(2200); Check(run(2200)==0,"lần ngắt sau vẫn khởi động mềm");
    frame(2400); Check(std::abs(run(2400)-4)<1e-5,"tăng mềm lần hai");
    Check(run(4401)==0,"ngắt dài dừng"); frame(4500);
    Check(run(4500)==0 && state=="DUNG_CHO_ANH_ON_DINH","ngắt dài chờ ảnh ổn định");
    Check(filtered.steeringDeg==0,"chờ ảnh vẫn trả góc về trim");
    for(int ms=4600;ms<6500;ms+=100) { frame(ms); Check(run(ms)==0,"giữ dừng tới đủ ảnh ổn định"); }
    frame(6500); Check(run(6500)==0,"đủ ổn định bắt đầu tăng mềm từ 0");
    for(int ms=6600;ms<7000;ms+=100) { frame(ms); run(ms); }
    frame(7000); Check(std::abs(run(7000)-10)<=1e-3,"đủ ổn định rồi tăng mềm");
    for(int gap:{1999,2000,2001}) {
        app::CameraRecovery edge; frame(0);
        edge.Filter(c,o,{20,0},at(0),state);
        edge.Filter(c,o,{20,0},at(501),state);
        frame(gap); edge.Filter(c,o,{20,0},at(gap),state);
        Check((state=="DUNG_CHO_ANH_ON_DINH")== (gap>2000),"biên camera_resume_max_s");
    }
    char path[]="/tmp/ucr_calibration_config_XXXXXX"; int fd=mkstemp(path); Check(fd>=0,"tạo config tạm"); close(fd);
    auto load=[&](const std::string &line){std::ofstream out(path); out<<"%YAML:1.0\n---\n"<<line<<'\n'; out.close(); common::Config cfg; cfg.Load(path);};
    for(const char *line:{"max_speed_cms: 101","max_speed_cms: 100.0000001","max_speed_cms: 0","camera_resume_max_s: 0.5",
            "camera_resume_max_s: \"2\"","soft_start_s: 0","soft_start_s: []","resume_stable_s: 0","resume_stable_s: \"2\"",
            "tof_mapping_confirmed: 2","smoke_servo_deg: 0","smoke_servo_deg: 61","smoke_servo_deg: 2.5"}) {
        bool rejected=false; try {load(line);} catch(const std::exception &) {rejected=true;}
        Check(rejected,"khóa mới sai phải bị từ chối");
    }
    load("max_speed_cms: 100"); std::remove(path);
    Check(app::CalibrationStep("position") && !app::CalibrationStep("unknown"),"kiểm bước trước thiết bị");
    char folder[]="/tmp/ucr_calibration_flow_XXXXXX"; Check(mkdtemp(folder)!=nullptr,"tạo thư mục smoke giả");
    c.log_dir=folder; c.tof_mapping_confirmed=1;
    const std::string cfg=c.log_dir+"/config.yaml", marker=c.log_dir+"/kill.ready";
    { std::ofstream out(cfg); out<<"%YAML:1.0\n---\nservo_trim_deg: 0\n"; }
    {
        common::Config bounded=c; bounded.calib_max_duration_s=2;
        common::Snapshot s;
        auto &obs=s.observation;
        const auto start=common::Clock::now();
        obs.frame_id=1; obs.captured_at=obs.odom_at=start; obs.odom_valid=true;
        obs.odom_cm=17; obs.yaw_rel_deg=30;
        for(const char *step:{"motor","radius","position","servo"}) {
            app::Calibration cal(bounded,step,c.log_dir,cfg,"",false);
            auto cmd=cal.Tick(s,'d',start);
            Check(cmd.steeringDeg==bounded.calib_servo_step_deg,"đặt góc trước lượt");
            cmd=cal.Tick(s,'y',start);
            const auto limits=cmd.calib_limits;
            const double distance=std::string(step)=="position" ? bounded.calib_position_cm : bounded.calib_distance_cm;
            Check(limits.enabled && limits.deadline==start+std::chrono::seconds(2) &&
                limits.start_odom_cm==17 && limits.max_distance_cm==distance &&
                limits.limit_yaw==(std::string(step)=="radius") && limits.start_yaw_deg==30,
                "lệnh giữ mang đủ mốc lượt, hạn tuyệt đối và giới hạn riêng từng bước");
            auto current=obs;
            Check(!app::CalibrationLimitReached(bounded,current,limits,start),"đầu lượt hợp lệ");
            current.odom_at=start+std::chrono::seconds(2);
            Check(app::CalibrationLimitReached(bounded,current,limits,current.odom_at),"chạm hạn dù encoder mới");
            current=obs; current.odom_cm=17+distance;
            Check(app::CalibrationLimitReached(bounded,current,limits,start),"đủ quãng tiến, kể cả bán kính");
            current.odom_cm=17-distance;
            Check(app::CalibrationLimitReached(bounded,current,limits,start),"đủ quãng theo độ lớn encoder");
            current=obs; current.odom_valid=false;
            Check(app::CalibrationLimitReached(bounded,current,limits,start),"encoder lỗi dừng độc lập UI");
            current=obs; current.odom_cm=std::numeric_limits<double>::quiet_NaN();
            Check(app::CalibrationLimitReached(bounded,current,limits,start),"encoder NaN dừng");
            current=obs; current.odom_at-=std::chrono::milliseconds(bounded.tof_timeout_ms);
            Check(app::CalibrationLimitReached(bounded,current,limits,start),"encoder tới tuổi giới hạn dừng");
            if(limits.limit_yaw) {
                current=obs; current.yaw_rel_deg=30+360;
                Check(app::CalibrationLimitReached(bounded,current,limits,start),"đủ vòng yaw dương");
                current.yaw_rel_deg=30-360;
                Check(app::CalibrationLimitReached(bounded,current,limits,start),"đủ vòng yaw âm");
            }
            cal.CancelMotion();
            cmd=cal.Tick(s,-1,start);
            Check(cmd.speedCmPerSec==0 && cmd.steeringDeg==0 && !cmd.position_control && !cmd.calib_limits.enabled,
                "huỷ xoá motor, góc, chế độ vị trí và giới hạn cũ");
            cal.Tick(s,'d',start); cal.Tick(s,'y',start);
            s.observation.odom_at=s.observation.captured_at=start+std::chrono::seconds(2);
            cmd=cal.Tick(s,-1,start+std::chrono::seconds(2));
            Check(cmd.speedCmPerSec==0 && cmd.steeringDeg==0,"Tick vẫn tự kiểm hạn và xoá góc");
            obs.odom_at=obs.captured_at=start;
            cal.Tick(s,'d',start); cal.Tick(s,'y',start);
            obs.captured_at=start-std::chrono::seconds(1);
            cmd=cal.Tick(s,-1,start);
            Check(cmd.speedCmPerSec==0 && cmd.steeringDeg==0,"mất ảnh xoá cả góc calib");
            obs.captured_at=start;
            cmd=cal.Tick(s,-1,start);
            Check(cmd.speedCmPerSec==0 && cmd.steeringDeg==0,"ảnh về không khôi phục góc/lệnh");
        }
    }
    const auto config_text=common::ConfigText(cfg);
    const auto gate=c.log_dir+"/smoke_gate.json";
    auto write_gate=[&](const std::string &bad,int value) {
        cv::FileStorage out(gate,cv::FileStorage::WRITE|cv::FileStorage::FORMAT_JSON);
        out<<"source_id"<<(bad=="source_id" ? "sai" : common::SourceId())
           <<"config_text"<<(bad=="config_text" ? "sai" : config_text)
           <<"simulated"<<(bad=="simulated" ? 1-common::Simulated() : common::Simulated());
        for(const char *key:{"camera","sensors","servo","tof_brake","camera_brake",
                "encoder_moving","kill9","encoder_stopped","passed"}) {
            if(bad==key && value<0) continue;
            if(bad==key && value==2) out<<key<<"1";
            else out<<key<<(bad==key ? value : 1);
        }
    };
    write_gate("",1); Check(common::SmokePassed(c,cfg),"smoke đủ mục, đúng dấu vân tay mới đạt");
    for(const char *key:{"camera","sensors","servo","tof_brake","camera_brake",
            "encoder_moving","kill9","encoder_stopped","passed"})
        for(int value:{-1,0,2}) {
            write_gate(key,value);
            Check(!common::SmokePassed(c,cfg),"cờ tổng không được che mục thiếu, sai kiểu hoặc không đạt");
        }
    for(const char *key:{"source_id","config_text","simulated"}) {
        write_gate(key,0); Check(!common::SmokePassed(c,cfg),"khác mã/cấu hình/loại build phải bị chặn");
    }
    { std::ofstream out(gate); out<<"{\"passed\":"; }
    Check(!common::SmokePassed(c,cfg),"JSON hỏng trả không đạt, không thoát bằng ngoại lệ");
    common::Snapshot snapshot; snapshot.fps=20; snapshot.observation.imu_valid=true;
    snapshot.observation.frame=cv::Mat(c.camera_height,c.camera_width,CV_8UC3,cv::Scalar(0,0,0));
    for(auto &t:snapshot.observation.tof) {t.valid=true; t.distanceMm=2000;}
    auto tick=[&](app::Calibration &cal,int ms,int key=-1) {
        const auto now=common::Clock::now()+std::chrono::milliseconds(ms);
        auto &obs=snapshot.observation; obs.frame_id++; obs.captured_at=obs.sensor_at=obs.odom_at=now; obs.odom_valid=true;
        for(auto &t:obs.tof) t.updatedAt=now;
        return cal.Tick(snapshot,key,now);
    };
    {
        app::Calibration cal(c,"smoke",c.log_dir,cfg,"",true);
        tick(cal,3001);
        for(int i=0;i<5;++i) Check(tick(cal,3010+i*10,'y').speedCmPerSec==0,"thiếu trình bao: không được chạy motor");
        Check(cal.Done() && cal.Failed(),"smoke trực tiếp chỉ cho kiểm cảm biến/servo, không bỏ phép kill");
        Check(tick(cal,4000,'y').speedCmPerSec==0,"smoke đã kết thúc không nhận lệnh chạy lại");
    }
    {
        common::Config unconfirmed=c; unconfirmed.tof_mapping_confirmed=0;
        app::Calibration cal(unconfirmed,"smoke",c.log_dir,cfg,marker,true);
        Check(tick(cal,3001).speedCmPerSec==0 && cal.Done() && cal.Failed(),"ánh xạ chưa xác nhận: không motor");
    }
    {
        app::Calibration cal(c,"smoke",c.log_dir,cfg,marker,true);
        Check(tick(cal,3001).steeringDeg==0,"smoke bắt đầu ở trim");
        Check(tick(cal,3010,'y').steeringDeg==-10,"servo âm 10 độ cần xác nhận");
        Check(tick(cal,3020).steeringDeg==-10,"chưa xác nhận thì giữ góc smoke");
        Check(tick(cal,3020,'y').steeringDeg==10,"servo dương 10 độ cần xác nhận");
        Check(tick(cal,3030,'y').steeringDeg==0,"servo trả trim");
        Check(tick(cal,3040,'y').speedCmPerSec==0,"chưa xác nhận kê bánh: không motor");
        Check(tick(cal,3050,'y').speedCmPerSec==10,"đã xác nhận kê bánh: chạy 10");
        snapshot.observation.meas_speed=10; tick(cal,5051);
        snapshot.observation.tof[c.FrontSlot()].distanceMm=100; tick(cal,5100);
        snapshot.observation.meas_speed=0; Check(tick(cal,5200).speedCmPerSec==0,"encoder dừng sau ToF");
        snapshot.observation.tof[c.FrontSlot()].distanceMm=2000; tick(cal,5300,'y');
        snapshot.observation.meas_speed=10; tick(cal,6400); Check(cal.DropCamera(),"chỉ ngắt ảnh sau encoder đang chạy");
        snapshot.observation.meas_speed=0; tick(cal,7000); Check(!cal.DropCamera(),"encoder dừng: cấp ảnh lại");
        Check(!std::ifstream(marker).good(),"chưa chạy lại: không được kill");
        snapshot.observation.meas_speed=10; tick(cal,8101);
        Check(std::ifstream(marker).good(),"encoder chạy lại mới báo run.sh kill");
    }
    std::remove(marker.c_str());
    {
        common::Config small=c; small.smoke_servo_deg=8; small.max_steering_deg=6;
        app::Calibration cal(small,"smoke",c.log_dir,cfg,"",true);
        tick(cal,3001);
        Check(tick(cal,3010,'y').steeringDeg==-6,"góc smoke vẫn bị chặn bởi giới hạn cơ khí");
        Check(tick(cal,3020,'y').steeringDeg==6,"góc smoke dương vẫn bị chặn");
    }
    {
        app::Calibration cal(c,"motor",c.log_dir,cfg,"",false);
        Check(tick(cal,100,'y').speedCmPerSec==10,"bắt đầu lượt calib để kiểm mất ảnh");
        auto obs=snapshot.observation; obs.captured_at-=std::chrono::milliseconds(800);
        snapshot.observation=obs;
        auto cmd=cal.Tick(snapshot,-1,obs.odom_at);
        std::string status;
        Check(app::SafeCommand(c,obs,cmd,obs.odom_at,status).speedCmPerSec==0,"calib mất ảnh vẫn dừng qua an toàn");
        Check(cmd.speedCmPerSec==0 && !cmd.position_control,"mất ảnh xoá lượt calib, không chỉ che lệnh đầu ra");
        Check(cal.Tick(snapshot,'y',obs.odom_at).speedCmPerSec==0,"bấm y lúc ảnh cũ không được ghi nhớ");
        snapshot.state.clear();
        cmd=tick(cal,900);
        Check(cmd.speedCmPerSec==0,"ảnh mới: giữ dừng tới khi người bấm lại");
        Check(tick(cal,950,'y').speedCmPerSec==10,"bấm lại lúc ảnh mới: chạy ngay, không chốt/tăng mềm");
        cal.CancelMotion();
        Check(tick(cal,970).speedCmPerSec==0,"huỷ từ luồng chấp hành vẫn có hiệu lực khi ảnh đã trở lại");
        Check(tick(cal,990,'y').speedCmPerSec==10,"lượt mới chỉ bắt đầu bằng phím mới");
        Check(tick(cal,1000,'x').speedCmPerSec==0 && tick(cal,1100).speedCmPerSec==0,"x vẫn huỷ lượt calib");
    }
    {
        app::Calibration cal(c,"smoke",c.log_dir,cfg,marker,true);
        tick(cal,3001); for(int i=0;i<5;++i) tick(cal,3010+i*10,'y');
        snapshot.observation.meas_speed=0;
        tick(cal,5100); Check(cal.Done() && cal.Failed() && !std::ifstream(marker).good(),"không có encoder chạy thì không tính đạt kill");
    }
    for(const char *name:{"servo","radius","motor","position","imu","tof","camera","latency","timing"}) {
        const std::string step=name;
        app::Calibration cal(c,step,c.log_dir,cfg,"",false);
        const bool motion=step=="servo" || step=="radius" || step=="motor" || step=="position";
        const auto cmd=tick(cal,100,'y');
        Check((cmd.speedCmPerSec>0)==motion,"chỉ bước chuyển động mới nhận y chạy motor");
        Check(cmd.position_control==(step=="position"),"chỉ bước vị trí phát lệnh đích");
        if(motion) Check(tick(cal,62000).speedCmPerSec==0,"lượt hiệu chuẩn có thời hạn");
        tick(cal,63000,'a'); Check(tick(cal,63010,'x').steeringDeg==0,"x trả trim");
        Check(tick(cal,63020).steeringDeg==0,"sau x giữ trim");
    }
}
