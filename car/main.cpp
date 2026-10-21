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
    std::signal(SIGINT,Signal); std::signal(SIGTERM,Signal);
    try {
        std::string path="../config.yaml",mode="manual";
        for(int i=1;i<argc;++i) {
            const std::string arg=argv[i];
            if(arg=="--help") { std::cout << "--config <tep.yaml> --mode manual|auto|collect|calib\n"; return 0; }
            if(i+1>=argc) throw std::runtime_error("Thieu gia tri tham so");
            if(arg=="--config") path=argv[++i];
            else if(arg=="--mode") mode=argv[++i];
            else throw std::runtime_error("Tham so khong biet: "+arg);
        }
        if(mode=="collect"||mode=="calib") { std::cout << "chua cai\n"; return 0; }
        if(mode!="manual"&&mode!="auto") throw std::runtime_error("Che do khong hop le");
        common::Config config; config.Load(path);
        app::App application(config,mode,path);
        application.Init();
        if(!stopped) application.Start();
        while(!stopped && application.IsRunning()) std::this_thread::sleep_for(std::chrono::milliseconds(20));
        application.RequestStop(); application.Wait();
        return application.Failed() ? 1 : stopped ? 128+stopped : 0;
    } catch(const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
