#include "app/App.h"
#include <cstdlib>
#include <csignal>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
namespace {
volatile std::sig_atomic_t stopped=0;
void Signal(int value) { stopped=value; }
}
int main(int argc,char **argv) {
    // Hàm kết thúc bất thường trả mã lỗi; trình bao bên ngoài đảm nhận dừng độc lập.
    std::set_terminate([]{ std::_Exit(134); });
    // Không tự khởi động lại đọc stdin: tín hiệu cũng ngắt được câu hỏi đầu phiên.
    struct sigaction action{}; action.sa_handler=Signal; sigemptyset(&action.sa_mask);
    sigaction(SIGINT,&action,nullptr); sigaction(SIGTERM,&action,nullptr);
    try {
        std::string path="../config.yaml",mode="manual";
        for(int i=1;i<argc;++i) {
            const std::string arg=argv[i];
            if(arg=="--help") { std::cout << "--config <tep.yaml> --mode manual|auto|collect|calib\n"; return 0; }
            if(arg!="--config" && arg!="--mode") throw std::runtime_error("tham số không biết: "+arg);
            if(i+1>=argc || std::string(argv[i+1]).compare(0,2,"--")==0)
                throw std::runtime_error("thiếu giá trị tham số: "+arg);
            if(arg=="--config") path=argv[++i];
            else if(arg=="--mode") mode=argv[++i];
        }
        if(mode=="calib") { std::cout << "Chưa cài chế độ hiệu chuẩn\n"; return 0; }
        if(mode!="manual"&&mode!="auto"&&mode!="collect") throw std::runtime_error("Chế độ không hợp lệ");
        common::Config config; config.Load(path);
        app::App application(config,mode,path);
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
