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
        Check(missing.kp==42 && missing.cruise_cms==8 && missing.scan_rows==defaults.scan_rows,"Thiếu khóa phải dùng mặc định");
        Check(report.str().find("cruise_cms = 8 cm/s (mặc định)")!=std::string::npos,"Thiếu nhãn/đơn vị mặc định trong báo cáo");
        Check(report.str().find("kp = 42 độ (mặc định)")==std::string::npos,"Khóa đã nạp không được báo mặc định");
        for(bool header:{false,true}) {
            const auto plain=load("kp: 42.\nscan_rows: [0.85, 0.65, 0.45]\n",header);
            Check(plain.kp==42 && plain.scan_rows==defaults.scan_rows,"YAML thường phải nạp như YAML OpenCV");
            for(const auto &body:{"", "{}\n", "# Chỉ chú thích\n  # dòng hai\n", "\n {} # map rỗng\r\n", "{ \n }\n", "{}\t# map rỗng\r\n", "{}\n...\n"}) {
                const auto empty=load(body,header);
                std::ostringstream table; empty.PrintTable(table);
                Check(table.str()==b.str(),"Tài liệu/map rỗng phải dùng toàn bộ mặc định");
                Check(report.str().find("kp = 30 độ (mặc định)")!=std::string::npos,"Map rỗng phải báo mặc định và đơn vị");
            }
            for(const auto &body:{"kp: [\n", "kp: [4\n", "scan_rows: [0.5}\n", "{kp: 4\n", "[]\n", "kp: 4\n---\nkp: 5\n", "{}#sai\n", "---#sai\n", "  %YAML:1.0\n",
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
            {"unknown: 1\n","unknown"}, {"lane_width_px: [100]\n","lane_width_px"},
            {"kp: .Nan\n","kp"}, {"kd: .Inf\n","kd"}, {"kp: -1\n","kp"},
            {"steer_max_deg: 41\n","steer_max_deg"}, {"max_speed_cms: 11\n","max_speed_cms"},
            {"control_period_ms: 19\n","control_period_ms"}, {"regain_frames: 3.5\n","regain_frames"},
            {"cruise_cms: \"8\"\n","cruise_cms"}, {"kp: []\n","kp"},
            {"kp:\n","kp"}, {"scan_rows: []\n","scan_rows"},
            {"scan_rows: [0, 0.6, 0.4]\n","scan_rows"}, {"scan_rows: [1, 0.6, 0.4]\n","scan_rows"},
            {"scan_rows: [0.9,0.8,0.7,0.6,0.5,0.4]\n","scan_rows"},
            {"row_weights: [0,0,0]\n","row_weights"}, {"row_weights: [-1,1,1]\n","row_weights"},
            {"row_weights: [1,1]\n","row_weights"}, {"row_weights: [.Nan,1,1]\n","row_weights"},
            {"lane_width_frac: [0,0.5,0.4]\n","lane_width_frac"},
            {"lane_width_frac: [1.1,0.5,0.4]\n","lane_width_frac"},
            {"min_valid_rows: 4\n","min_valid_rows"},
            {"lost_stop_ms: 200\n","lost_stop_ms"}, {"min_cms: 9\n","min_cms"},
            {"max_speed_cms: 7\n","cruise_cms"}, {"lost_cms: 9\nmax_speed_cms: 8\n","lost_cms"}
        }) for(bool header:{false,true}) rejectLoad(item.first,header,item.second);
        auto invalid=defaults; invalid.kp=std::numeric_limits<double>::quiet_NaN();
        Reject([&] { invalid.Validate(); },"kp");
        invalid=defaults; invalid.row_weights={1e308,1e308,0};
        Reject([&] { invalid.Validate(); },"row_weights");
        const auto one=load("scan_rows: [0.5]\nrow_weights: [2]\nlane_width_frac: [1]\nmin_valid_rows: 1\n");
        Check(one.scan_rows.size()==1 && one.row_weights[0]==2,"Trọng số không cần chuẩn hóa hoặc <= 1");
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
