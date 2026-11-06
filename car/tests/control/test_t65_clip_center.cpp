#include "control/LaneController.h"
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void Check(bool ok, const std::string &message) {
    if(!ok) throw std::runtime_error(message);
}
void Near(double actual, double expected, const std::string &message) {
    Check(std::isfinite(actual) && std::abs(actual-expected)<1e-9,message);
}
cv::Mat Mask(const control::ControlParams &p, int cols, int right, bool both, bool surface) {
    cv::Mat mask(100,cols,CV_8UC1,cv::Scalar(0));
    for(size_t i=0;i<p.scan_rows.size();++i) {
        const int y=int(p.scan_rows[i]*mask.rows);
        const int left=both ? right-int(p.lane_width_frac[i]*cols) : 0;
        if(surface || both)
            for(int x=left;x<right;++x) mask.at<unsigned char>(y,x)=1;
        if(both) mask.at<unsigned char>(y,left)=2;
        if(right<cols) mask.at<unsigned char>(y,right)=2;
        else mask.at<unsigned char>(y,cols-1)=1;
    }
    return mask;
}
void Geometry(int mode, int cols, int right, bool both, bool surface, control::LaneRef ref) {
    control::ControlParams p;
    p.pair_mode=mode; p.lane_width_frac={0.4,0.5,0.6};
    const auto mask=Mask(p,cols,right,both,surface);
    control::LaneController c;
    const auto r=c.Step(mask,0,0,p,ref);
    Check(r.debug.valid_rows==3,"Phải giữ đủ ba hàng hợp lệ");
    double expected_error=0, total=0;
    for(size_t i=0;i<r.debug.rows.size();++i) {
        const auto &row=r.debug.rows[i];
        const double width=p.lane_width_frac[i]*cols;
        const double expected=both ? right-width/2 : right-width/(double(right)/cols>=p.right_clip_frac ? 4 : 2);
        Check(row.valid,"Hàng phải hợp lệ");
        Near(row.right,right,"Sai biên phải");
        Near(row.left,both ? right-width : -1,"Sai biên trái");
        Near(row.center,expected,"Sai tâm hàng: mode="+std::to_string(mode)+", right="+std::to_string(right));
        expected_error+=p.row_weights[i]*(expected-cols/2.0)/(cols/2.0);
        total+=p.row_weights[i];
    }
    Near(r.debug.e,expected_error/total,"Sai số e phải dùng tâm đã hiệu chỉnh");
}
void Lost(int mode) {
    control::ControlParams p; p.pair_mode=mode; p.lane_width_frac={0.4,0.5,0.6};
    p.right_lost_ms=500;
    const auto mask=Mask(p,1000,1000,false,true);
    control::LaneController c;
    for(int64_t now:{0,100,200,300,400,499})
        Check(c.Step(mask,now,now,p).debug.state=="BAM_LAN","Không được mất làn trước 500 ms");
    const auto r=c.Step(mask,500,500,p);
    Check(r.debug.state=="GIU_GOC" && r.debug.reason=="MAT_LAN_GIU","Phải vào đường valid=false đúng 500 ms");
    Check(c.Step(mask,1300,1300,p).debug.state=="MAT_LAN","Mất làn phải tiếp tục dừng theo luật cũ");
}
}
int main() {
    try {
        for(int mode:{0,1}) {
            for(int cols:{1000,2000}) for(int permille:{985,990,1000}) {
                const int right=cols*permille/1000;
                for(bool surface:{false,true}) Geometry(mode,cols,right,false,surface,control::LaneRef::RIGHT);
                for(auto ref:{control::LaneRef::RIGHT,control::LaneRef::LEFT})
                    Geometry(mode,cols,right,true,true,ref);
            }
            Lost(mode);
        }
        std::cout << "ĐẠT: 48 ca hình học và 2 ca mất vạch 500 ms\n";
        return 0;
    } catch(const std::exception &error) {
        std::cerr << "KHÔNG ĐẠT: " << error.what() << '\n';
        return 1;
    }
}
