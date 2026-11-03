#include "app/App.h"
#include "common/Smoke.h"
#include <cstdlib>
#include <csignal>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <cmath>
#include <sstream>
#include <unistd.h>
namespace {
volatile std::sig_atomic_t stopped=0;
void Signal(int value) { stopped=value; }
}
int main(int argc,char **argv) {
    std::cout<<std::unitbuf;
    // Nhánh chỉ đọc đứng trước mọi đường khởi tạo và chỉ nhận đúng cú pháp hợp đồng.
    for(int i=1;i<argc;++i) if(std::string(argv[i])=="--check-config") {
        if(argc!=4 || std::string(argv[1])!="--check-config" || std::string(argv[2])!="--config" ||
           std::string(argv[3]).empty() || std::string(argv[3]).compare(0,2,"--")==0) {
            std::cerr<<"Cú pháp: main --check-config --config <file>\n";
            return 2;
        }
        try {
            common::Config config; config.Load(argv[3]);
            std::cout<<"CAU_HINH_OK: "<<argv[3]<<'\n';
            return 0;
        } catch(const std::exception &e) { std::cerr<<e.what()<<'\n'; return 2; }
    }
    // Hàm kết thúc bất thường trả mã lỗi; trình bao bên ngoài đảm nhận dừng độc lập.
    std::set_terminate([]{ std::_Exit(134); });
    // Không tự khởi động lại đọc stdin: tín hiệu cũng ngắt được câu hỏi đầu phiên.
    struct sigaction action{}; action.sa_handler=Signal; sigemptyset(&action.sa_mask);
    sigaction(SIGINT,&action,nullptr); sigaction(SIGTERM,&action,nullptr);
    try {
        std::string path="../config.yaml",mode="manual",step;
        double calib_drop_after=0;
        for(int i=1;i<argc;++i) {
            const std::string arg=argv[i];
            if(arg=="--help") { std::cout << "--config <tep.yaml> --mode manual|auto|calib --step <1..10>\n"
                <<"--check-config --config <file>: chỉ kiểm cấu hình, không mở thiết bị\n"
                <<"Bước: 1 môi trường, 2 servo, 3 bán kính, 4 motor, 5 vị trí (đang khoá), 6 IMU, 7 ToF, 8 camera, 9 độ trễ, 10 nhịp vòng.\n"
                <<"--env-info: chỉ đọc bản build; --calib-drop-camera-after <giây>: chỉ calib motor\n"; return 0; }
            if(arg=="--env-info") {
                std::cout<<"OpenCV "<<CV_VERSION<<"; source_id="<<common::SourceId()<<"; simulated="<<common::Simulated()<<'\n';
                std::istringstream info(cv::getBuildInformation()); std::string line;
                while(std::getline(info,line)) if(line.find("GStreamer")!=std::string::npos) std::cout<<line<<'\n';
                return 0;
            }
            if(arg!="--config" && arg!="--mode" && arg!="--step" && arg!="--calib-drop-camera-after") throw std::runtime_error("tham số không biết: "+arg);
            if(i+1>=argc || std::string(argv[i+1]).compare(0,2,"--")==0)
                throw std::runtime_error("thiếu giá trị tham số: "+arg);
            if(arg=="--config") path=argv[++i];
            else if(arg=="--step") {
                if(!step.empty()) throw std::runtime_error("Bước hiệu chuẩn bị lặp");
                const std::string number=argv[++i];
                const char *steps[]={"env","servo","radius","motor","position","imu","tof","camera","latency","timing"};
                int index=number=="position" ? 4 : number=="10" ? 9 : (number.size()==1 && number[0]>='1' && number[0]<='9' ? number[0]-'1' : -1);
                if(index<0) throw std::runtime_error("Bước hiệu chuẩn không hợp lệ: "+number+"; --step phải từ 1 đến 10");
                step=steps[index];
            }
            else if(arg=="--calib-drop-camera-after") {
                const std::string text=argv[++i]; std::size_t used=0;
                calib_drop_after=std::stod(text,&used);
                if(used!=text.size() || !std::isfinite(calib_drop_after) || calib_drop_after<=0)
                    throw std::runtime_error("--calib-drop-camera-after phải là số giây > 0");
            }
            else if(arg=="--mode") {
                mode=argv[++i];
                // Giữ cách gọi theo tên để các lệnh hiệu chuẩn đã lưu vẫn dùng được.
                if(mode=="calib" && i+1<argc && std::string(argv[i+1]).compare(0,2,"--")!=0) {
                    if(!step.empty()) throw std::runtime_error("Bước hiệu chuẩn bị lặp");
                    step=argv[++i];
                    if(!app::CalibrationStep(step)) throw std::runtime_error("Bước hiệu chuẩn không hợp lệ: "+step);
                }
            }
        }
        if(mode!="manual"&&mode!="auto"&&mode!="calib") throw std::runtime_error("Chế độ không hợp lệ");
        if(mode=="calib" && step.empty()) throw std::runtime_error("thiếu bước hiệu chuẩn: dùng --step <1..10>");
        if(mode!="calib" && !step.empty()) throw std::runtime_error("--step chỉ dùng với --mode calib");
        if(mode=="calib" && step=="position") {
            std::cerr<<"Bước 5 đang khoá cho tới khi kiểm chế độ vị trí trên xe\n";
            return 2;
        }
        if(calib_drop_after>0 && (mode!="calib" || step!="motor")) throw std::runtime_error("Cờ ngắt ảnh hiệu chuẩn chỉ dùng với calib motor");
        if(step=="env") {
            // Dùng chung phép kiểm chỉ đọc; truyền đúng bản thực thi kể cả khi build ở ngoài repo.
            ::execlp("bash","bash",UCR_ENV_CHECK_SCRIPT,argv[0],static_cast<char *>(nullptr));
            throw std::runtime_error("Không chạy được scripts/env_check.sh bằng bash");
        }
        common::Config config; config.Load(path);
        app::App application(config,mode,path,step,calib_drop_after);
        application.Init();
        if(!stopped) application.Start();
        while(!stopped && application.IsRunning()) std::this_thread::sleep_for(std::chrono::milliseconds(20));
        application.RequestStop(); application.Wait();
        return application.Failed() ? 1 : stopped ? 128+stopped : 0;
    } catch(const std::exception &e) {
        std::cerr << e.what() << '\n'
                  << "Nếu lỗi thiết bị: giữ xe đứng yên, kiểm tra nguồn, dây nối và quyền truy cập UART/I²C/camera; "
                     "đối chiếu cấu hình, hỏi BTC nếu trùng địa chỉ IMU/ToF. Không tự đổi địa chỉ hoặc khởi tạo lại motor.\n";
        return 1;
    }
}
