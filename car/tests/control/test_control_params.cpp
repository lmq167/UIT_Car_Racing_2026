#include "control/ControlParams.h"
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace {
void Check(bool ok, const std::string &message) { if(!ok) throw std::runtime_error(message); }
template<class F> void Reject(F action, const std::string &key, const std::string &context="") {
    try { action(); }
    catch(const std::exception &error) {
        Check(std::string(error.what()).find(key)!=std::string::npos,"Lỗi phải nêu khóa/đường dẫn: "+key+"; nhận: "+error.what()+context);
        return;
    }
    throw std::runtime_error("Phải từ chối: "+key+context);
}
}
int main(int argc, char **argv) {
    try {
        Check(argc==3,"Cần đường dẫn control.yaml mẫu và file tạm");
        const std::string temporary=argv[2];
        struct Cleanup { std::string path; ~Cleanup() { std::remove(path.c_str()); } } cleanup{temporary};
        std::ostringstream report;
        const auto sample=control::ControlParams::Load(argv[1],report);
        const control::ControlParams defaults;
        Check(defaults.cruise_cms==15 && defaults.steer_max_deg==40 && defaults.max_speed_cms==20 &&
              defaults.obstacle_action==2,"Mặc định thi phải là 15/40/20/2");
        Check(std::abs(defaults.slow_per_deg-1.0/6)<1e-9,"Mặc định giảm tốc phải cho khoảng 10 cm/s ở góc 30 độ, cruise 15");
        Check(defaults.imu_fallback==0,"Dự phòng IMU phải mặc định tắt");
        Check(defaults.record_every_n==0,"Mặc định tắt ghi ảnh (Quân 11/11: lỗi thì tự đặt 3)");
        std::ostringstream a,b; sample.PrintTable(a); defaults.PrintTable(b);
        Check(a.str()==b.str(),"control.yaml mẫu phải đúng toàn bộ mặc định L2");
        Check(report.str().find("(mặc định)")==std::string::npos,"Mẫu đủ khóa, không dùng mặc định ngầm");
        auto load=[&](const std::string &body, bool header=true) {
            { std::ofstream out(temporary,std::ios::binary); if(header) out << "%YAML:1.0\n---\n"; out << body; Check(bool(out),"Không ghi được file tạm"); }
            report.str(""); report.clear();
            return control::ControlParams::Load(temporary,report);
        };
        auto rejectLoad=[&](const std::string &body, bool header, const std::string &key) {
            std::string context=std::string("; header=")+(header?"true":"false")+"; YAML=\"";
            for(char c:body) {
                if(c=='\0') context+="\\0";
                else if(c=='\n') context+="\\n";
                else if(c=='\r') context+="\\r";
                else if(c=='\t') context+="\\t";
                else { if(c=='\\' || c=='\"') context+='\\'; context+=c; }
            }
            Reject([&] { load(body,header); },key,context+'\"');
        };
        const auto missing=load("kp: 42.\n");
        Check(missing.kp==42 && missing.cruise_cms==15 && missing.lane1_width_frac==defaults.lane1_width_frac,"Thiếu khóa phải dùng mặc định");
        Check(missing.max_speed_cms==20 && missing.approach_cms==5 && missing.min_cms==4 && missing.lost_cms==4,
              "Thiếu khóa tốc độ phải giữ mặc định 15/20/5/4/4");
        Check(report.str().find("cruise_cms = 15 cm/s (mặc định)")!=std::string::npos,"Thiếu nhãn/đơn vị mặc định trong báo cáo");
        Check(report.str().find("kp = 42 độ (mặc định)")==std::string::npos,"Khóa đã nạp không được báo mặc định");
        for(bool header:{false,true}) {
            for(int value:{0,1})
                Check(load("imu_fallback: "+std::to_string(value)+"\n",header).imu_fallback==value,"Phải nhận imu_fallback 0/1");
            for(const auto &value:{"-1","2","0.5",".Nan",".Inf","\"1\""})
                rejectLoad(std::string("imu_fallback: ")+value+"\n",header,"imu_fallback");
            const auto race=load("cruise_cms: 15\nmax_speed_cms: 15\nsteer_max_deg: 40\n",header);
            Check(race.cruise_cms==15 && race.max_speed_cms==15 && race.steer_max_deg==40,"Phải nhận tốc độ 15 và góc 40");
            for(int value:{0,1,5,40})
                Check(load("record_every_n: "+std::to_string(value)+"\n",header).record_every_n==value,"Phải nhận tắt ghi ảnh và nhịp ghi hợp lệ");
            const auto cap20=load("max_speed_cms: 20\ncruise_cms: 20\napproach_cms: 10\nmin_cms: 10\nlost_cms: 10\n",header);
            Check(cap20.max_speed_cms==20 && cap20.cruise_cms==20 && cap20.approach_cms==10 &&
                  cap20.min_cms==10 && cap20.lost_cms==10,"Phải nhận trần 20 và giữ biên 10 của approach/min/lost");
            for(int value:{2,8,20})
                Check(load("corridor_rows: "+std::to_string(value)+"\n",header).corridor_rows==value,"Số hàng hành lang hợp lệ phải nạp được");
            for(double value:{0.35,1.0})
                Check(load("corridor_conf_min: "+std::to_string(value)+"\n",header).conf_min==value,"Phải nhận hai biên corridor_conf_min 0.35 và 1");
            const auto plain=load("kp: 42.\nlane1_width_frac: [0.57, 0.17]\n",header);
            Check(plain.kp==42 && plain.lane1_width_frac==defaults.lane1_width_frac,"YAML thường phải nạp như YAML OpenCV");
            for(const auto &body:{"", "{}\n", "# Chỉ chú thích\n  # dòng hai\n", "\n {} # map rỗng\r\n", "{ \n }\n", "{}\t# map rỗng\r\n", "{}\n...\n"}) {
                const auto empty=load(body,header);
                std::ostringstream table; empty.PrintTable(table);
                Check(table.str()==b.str(),"Tài liệu/map rỗng phải dùng toàn bộ mặc định");
                Check(report.str().find("kp = 35 độ (mặc định)")!=std::string::npos,"Map rỗng phải báo mặc định và đơn vị");
            }
            for(const auto &body:{"kp: [\n", "kp: [4\n", "lane1_width_frac: [0.5}\n", "{kp: 4\n", "[]\n", "kp: 4\n---\nkp: 5\n", "{}#sai\n", "---#sai\n", "  %YAML:1.0\n",
                                  "kp: 4\n...\nkp: 5\n", "kp: 4\n...\n---\nkp: 5\n", "{}\n---\n{}\n", "...\n...\n",
                                  "# chú thích\n%YAML:1.0\n", "{}x\n", "{ }#sai\n", "scalar\n", "- kp: 4\n", "-\n  kp: 4\n"})
                rejectLoad(body,header,temporary);
            rejectLoad(std::string("kp: 42\0",7)+"junk",header,temporary);
        }
        Check(load("\xef\xbb\xbfkp: 42\n",false).kp==42,"YAML UTF-8 có BOM phải nạp được");
        Check(load("\xef\xbb\xbf%YAML:1.0\r\n---\r\nkp: 42\r\n",false).kp==42,"BOM, tiêu đề và CRLF phải nạp được");
        const std::string raw="kp: 42\n";
        load(raw,false);
        std::ifstream original(temporary,std::ios::binary);
        std::ostringstream unchanged; unchanged<<original.rdbuf();
        Check(unchanged.str()==raw,"Load không được sửa byte file gốc");
        original.close();
        const auto rawHash=control::ControlParams::FileHash(temporary);
        load(raw,true);
        Check(control::ControlParams::FileHash(temporary)!=rawHash,"FNV phải phân biệt byte tiêu đề gốc");
        for(const auto &item:std::vector<std::pair<std::string,std::string>>{
            {"start_wait_green: 2\n","start_wait_green"},
            {"start_wait_green: 0.5\n","start_wait_green"},
            {"vote_window: 0\n","vote_window"},
            {"vote_window: 31\n","vote_window"},
            {"vote_min: 6\n","vote_min"},
            {"green_frames: 6\n","green_frames"},
            {"approach_cms: 11\n","approach_cms"},
            {"red_stop_x_frac: 1.01\n","red_stop_x_frac"},
            {"red_conf_min: -0.01\n","red_conf_min"},
            {"red_conf_min: 1.01\n","red_conf_min"},
            {"red_conf_min: \"0.5\"\n","red_conf_min"},
            {"red_box_min_area_frac: -0.1\n","red_box_min_area_frac"},
            {"red_box_min_area_frac: 1.1\n","red_box_min_area_frac"},
            {"green_conf_min: -0.01\n","green_conf_min"},
            {"green_conf_min: 1.01\n","green_conf_min"},
            {"green_conf_min: \"0.5\"\n","green_conf_min"},
            {"green_box_min_area_frac: -0.1\n","green_box_min_area_frac"},
            {"green_box_min_area_frac: 1.1\n","green_box_min_area_frac"},
            {"off_conf_min: -0.01\n","off_conf_min"},
            {"off_conf_min: 1.01\n","off_conf_min"},
            {"off_conf_min: \"0.5\"\n","off_conf_min"},
            {"off_box_min_area_frac: -0.1\n","off_box_min_area_frac"},
            {"off_box_min_area_frac: 1.1\n","off_box_min_area_frac"},
            {"unknown: 1\n","unknown"}, {"lane_width_px: [100]\n","lane_width_px"},
            {"kp: .Nan\n","kp"}, {"kd: .Inf\n","kd"}, {"kp: -1\n","kp"},
            {"steer_max_deg: 41\n","steer_max_deg"}, {"steer_max_deg: 40.1\n","steer_max_deg"},
            {"cruise_cms: 16\nmax_speed_cms: 15\n","cruise_cms"}, {"max_speed_cms: 21\n","max_speed_cms"},
            {"max_speed_cms: 20.1\n","max_speed_cms"},
            {"max_speed_cms: 20\ncruise_cms: 20.1\n","cruise_cms"},
            {"max_speed_cms: 20\ncruise_cms: 21\n","cruise_cms"},
            {"max_speed_cms: 20\ncruise_cms: 20\napproach_cms: 10.1\n","approach_cms"},
            {"max_speed_cms: 20\ncruise_cms: 20\nmin_cms: 10.1\n","min_cms"},
            {"max_speed_cms: 20\ncruise_cms: 20\nlost_cms: 10.1\n","lost_cms"},
            {"max_speed_cms: 20\ncruise_cms: 20\napproach_cms: 15\n","approach_cms"},
            {"max_speed_cms: 20\ncruise_cms: 20\nmin_cms: 15\n","min_cms"},
            {"max_speed_cms: 20\ncruise_cms: 20\nlost_cms: 15\n","lost_cms"},
            {"control_period_ms: 19\n","control_period_ms"}, {"regain_frames: 3.5\n","regain_frames"},
            {"record_every_n: -1\n","record_every_n"}, {"record_every_n: 41\n","record_every_n"},
            {"record_every_n: 0.5\n","record_every_n"},
            {"park_rev_cms: 6.1\n","park_rev_cms"}, {"park_tof_map: 1123\n","park_tof_map"}, {"park_tof_map: 1235\n","park_tof_map"}, {"park_tof_map: 123\n","park_tof_map"}, {"park_steer_deg: 46\n","park_steer_deg"},{"park_rev_cms: 1.9\n","park_rev_cms"},
            {"cruise_cms: \"8\"\n","cruise_cms"}, {"kp: []\n","kp"},
            {"kp:\n","kp"}, {"lane1_width_frac: []\n","lane1_width_frac"},
            {"lane1_width_frac: [0.1, 0.2]\n","lane1_width_frac"}, {"lane1_width_frac: [0.5]\n","lane1_width_frac"},
            {"lane1_width_frac: [0.5, 0.2, 0.1]\n","lane1_width_frac"}, {"lane1_width_frac: [0.5, 0]\n","lane1_width_frac"},
            {"lane1_width_frac: [1.1, 0.2]\n","lane1_width_frac"}, {"lane1_width_frac: [.Nan, 0.2]\n","lane1_width_frac"},
            {"corridor_rows: 1\n","corridor_rows"}, {"corridor_rows: 21\n","corridor_rows"}, {"corridor_rows: 8.5\n","corridor_rows"},
            {"corridor_bottom_frac: 1.01\n","corridor_bottom_frac"}, {"corridor_top_frac: 0.96\n","corridor_top_frac"},
            {"corridor_top_frac: 0\n","corridor_top_frac"},
            {"two_lane_ratio: 1.19\n","two_lane_ratio"}, {"two_lane_ratio: 3.01\n","two_lane_ratio"},
            {"max_jump_frac: 0\n","max_jump_frac"}, {"max_jump_frac: 0.51\n","max_jump_frac"},
            {"corridor_conf_min: 1.01\n","corridor_conf_min"}, {"corridor_conf_min: -0.01\n","corridor_conf_min"},
            {"corridor_conf_min: 0\n","corridor_conf_min"}, {"corridor_conf_min: 0.2\n","corridor_conf_min"},
            {"corridor_conf_min: 0.349\n","corridor_conf_min"},
            {"lookahead_frac: 0.5\n","lookahead_frac"}, {"lookahead_frac: 0.96\n","lookahead_frac"},
            {"avoid_min_width_ratio: 0\n","avoid_min_width_ratio"}, {"avoid_min_width_ratio: 2.01\n","avoid_min_width_ratio"},
            {"obstacle_hole_ratio: 0\n","obstacle_hole_ratio"}, {"obstacle_hole_ratio: 1.01\n","obstacle_hole_ratio"},
            {"corridor_rows: 3\nmin_valid_rows: 4\n","min_valid_rows"},
            {"lost_stop_ms: 200\n","lost_stop_ms"}, {"min_cms: 9\ncruise_cms: 8\n","min_cms"},
            {"max_speed_cms: 7\n","cruise_cms"}, {"lost_cms: 9\nmax_speed_cms: 8\ncruise_cms: 8\n","lost_cms"},
            // Khóa thuật toán làn cũ đã bỏ (T79): nay là khóa lạ.
            {"scan_rows: [0.85, 0.65, 0.45]\n","scan_rows"}, {"row_weights: [0.5, 0.3, 0.2]\n","row_weights"},
            {"lane_width_frac: [0.7, 0.55, 0.4]\n","lane_width_frac"}, {"right_lost_ms: 500\n","right_lost_ms"},
            {"right_clip_frac: 0.99\n","right_clip_frac"}, {"pair_mode: 0\n","pair_mode"},
            {"pair_max_shift_frac: 0.25\n","pair_max_shift_frac"}, {"pair_merge_frac: 0.08\n","pair_merge_frac"},
            {"kd_filter_ms: 0\n","kd_filter_ms"}, {"lane_width_min_ratio: 0.6\n","lane_width_min_ratio"},
            {"lane_width_max_ratio: 1.4\n","lane_width_max_ratio"},
        }) for(bool header:{false,true}) rejectLoad(item.first,header,item.second);
        auto invalid=defaults; invalid.kp=std::numeric_limits<double>::quiet_NaN();
        Reject([&] { invalid.Validate(); },"kp");
        for(int value:{-1,2}) { invalid=defaults; invalid.imu_fallback=value; Reject([&] { invalid.Validate(); },"imu_fallback"); }
        invalid=defaults; invalid.record_every_n=0; invalid.Validate();
        invalid.record_every_n=-1;
        Reject([&] { invalid.Validate(); },"record_every_n");
        invalid=defaults; invalid.lane1_width_frac={0.1,0.2};
        Reject([&] { invalid.Validate(); },"lane1_width_frac");
        for(double value:{0.35,1.0}) { invalid=defaults; invalid.conf_min=value; invalid.Validate(); }
        for(double value:{0.2,0.349,1.01}) {
            invalid=defaults; invalid.conf_min=value;
            Reject([&] { invalid.Validate(); },"corridor_conf_min");
        }
        const auto zeroSpeed=load("max_speed_cms: 0\ncruise_cms: 0\nmin_cms: 0\nlost_cms: 0\napproach_cms: 5\n");
        Check(zeroSpeed.max_speed_cms==0 && zeroSpeed.approach_cms==5,"Trần tốc độ 0 vẫn nạp được approach_cms 5");
        const auto two=load("corridor_rows: 2\nmin_valid_rows: 2\nlane1_width_frac: [0.3, 0.3]\n");
        Check(two.corridor_rows==2 && two.lane1_width_frac[0]==two.lane1_width_frac[1],"Bề rộng gần bằng xa và 2 hàng phải hợp lệ");
        { std::ofstream out(temporary,std::ios::binary); out << "hello"; }
        Check(control::ControlParams::FileHash(temporary)=="a430d84680aabd0b","Sai FNV-1a 64 của hello");
        { std::ofstream out(temporary,std::ios::binary|std::ios::trunc); }
        Check(control::ControlParams::FileHash(temporary)=="cbf29ce484222325","Sai FNV-1a của file rỗng");
        std::remove(temporary.c_str());
        Reject([&] { control::ControlParams::Load(temporary,report); },temporary);
        Reject([&] { control::ControlParams::FileHash(temporary); },temporary);
        std::cout << "ĐẠT: bảng L2, mẫu YAML, mặc định, kiểu, miền, ràng buộc chéo, FNV-1a\n";
        return 0;
    } catch(const std::exception &error) { std::cerr << "KHÔNG ĐẠT: " << error.what() << '\n'; return 1; }
}
