#include "control/ParkScan.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
using namespace control;
namespace {
void Expect(bool ok, const char* expectation) {
    if (!ok) { std::cerr << "SAI: " << expectation << '\n'; std::exit(1); }
}
ParkScanConfig Config() {
    ParkScanConfig c;
    for (int i=0;i<4;++i) c.map[i]=i;
    return c;
}
// Mẫu giả theo tọa độ thực của từng cảm biến, bước encoder 1,5 cm.
void Feed(ParkScan& scan, const ParkScanConfig& c, double length=50,
          int fault=0, bool both=false, bool open=false) {
    const double start=open ? -c.front_off_cm : -30;
    for (double odo=start; odo<=length+3; odo+=1.5) {
        TofSlot slots[4];
        for (int i=0;i<4;++i) {
            auto& s=slots[i];
            s.initialized=s.valid=s.has_sample=true;
            s.sample_ms=1000;
            const double x=odo+(i%2==0 ? c.front_off_cm : c.rear_off_cm);
            s.mm=(i<2 || both) && x>=0 && x<length ? 800 : 200;
            if (fault==1 && i==0 && std::abs(odo-0)<0.1) s.api_error=1;
            if (fault==2 && i==0) s.initialized=false;
            if (fault==3 && i==0) s.sample_ms=0;
            if (fault==4 && i==1) s.mm=200;
            if (fault==5 && i==0 && std::abs(odo)<0.1) s.valid=false;
            if (fault==6 && i==0 && std::abs(odo)<0.1) s.has_sample=false;
            if (fault==7 && i==0 && std::abs(odo)<0.1) s.mm=-1;
        }
        scan.Update(1000,odo,slots);
    }
}
}
int main() {
    const auto c=Config();
    {
        ParkScan s(c); Feed(s,c);
        const auto r=s.Result();
        Expect(r.found && r.side==1 && !r.open_near,"ô 50 cm giữa hai vật: có ô trái, mép gần đóng");
        Expect(std::abs(r.near_x_cm)<=1.5 && std::abs(r.far_x_cm-50)<=1.5,"tọa độ hai mép sai không quá một bước");
        TofSlot bad[4];
        s.Update(2000,0,bad); s.Update(2000,52.5,bad);
        Expect(s.Result().found && s.Current(1)==ScanCell::OCCUPIED,"odo lùi hoặc đứng: giữ nguyên trạng thái và kết quả");
        s.Reset();
        Expect(!s.Result().found && s.Current(1)==ScanCell::UNKNOWN,"Reset xóa ô và trạng thái");
        Feed(s,c); Expect(s.Result().found,"Reset cho phép quét lại từ odo thấp");
    }
    {
        ParkScan s(c); Feed(s,c,0);
        Expect(!s.Result().found,"ô có vật: không tìm thấy ô");
    }
    for (int fault: {1,2,3,4,5,6,7}) {
        ParkScan s(c); Feed(s,c,50,fault);
        Expect(!s.Result().found,"lỗi, mất cảm biến, quá cũ hoặc trước/sau không khớp: không có ô");
    }
    {
        ParkScan s(c); Feed(s,c,100);
        Expect(!s.Result().found && s.Why()=="o_qua_dai","hai ô liền không vật ngăn: quá dài, không có ô");
    }
    {
        ParkScan s(c); Feed(s,c,35);
        Expect(!s.Result().found && s.Why()=="o_qua_ngan","ô 35 cm: quá ngắn");
    }
    {
        ParkScan s(c); Feed(s,c,50,0,false,true);
        const auto r=s.Result();
        Expect(r.found && r.open_near && std::abs(r.near_x_cm-(r.far_x_cm-50))<1e-9,"mép gần mở, đủ 50 cm: có ô, suy mép gần từ mép xa");
    }
    for (int missing: {0,1}) {
        auto unmapped=c; unmapped.map[missing]=-1;
        ParkScan s(unmapped); Feed(s,unmapped);
        Expect(!s.Result().found,"thiếu ánh xạ trước hoặc sau: không có ô");
    }
    {
        ParkScan s(c); Feed(s,c,50,0,true);
        Expect(!s.Result().found && s.Why()=="hai_ben","hai bên cùng ô cùng mép xa: không chọn bên");
    }
    {
        ParkScan s(c);
        TofSlot slots[4];
        for (auto& t: slots) { t.initialized=t.valid=t.has_sample=true; t.mm=800; t.sample_ms=1000; }
        for (double x=0;x<80;x+=1.5) s.Update(1000,x,slots);
        Expect(!s.Result().found && s.Why()=="chua_thay_mep_xa","chưa gặp vật cuối: không có ô");
    }
    std::cout << "ParkScan: tất cả kỳ vọng đạt\n";
}
