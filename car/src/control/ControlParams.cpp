#include "control/ControlParams.h"
#include <opencv2/core.hpp>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <variant>

namespace control {
namespace {
using Values = std::vector<double>;
using Value = std::variant<int, double, Values>;
using Member = std::variant<int ControlParams::*, double ControlParams::*, Values ControlParams::*>;
struct Parameter {
    const char *name;
    Value initial;
    Member member;
    double low, high;
    const char *unit, *description;
};
// Nguồn duy nhất cho mặc định, miền, nạp và in bảng (L2).
const Parameter table[] = {
    {"work_barrier_action",1,&ControlParams::work_barrier_action,0,1,"1","Rào công trình (0 = tắt, 1 = rẽ trái theo IMU)"},
    {"work_barrier_y2_frac",0.65,&ControlParams::work_barrier_y2_frac,0.30,0.95,"H","Đáy hộp rào tối thiểu để bắt đầu rẽ"},
    {"obstacle_action",2,&ControlParams::obstacle_action,0,2,"1","Vật cản (0 = tắt, 1 = dừng, 2 = né)"},
    {"obstacle_hole_frac",0.10,&ControlParams::obstacle_hole_frac,0.02,0.5,"1","Tỉ lệ lỗ tối thiểu để nhận vật cản"},
    {"obstacle_pass_cm",40.0,&ControlParams::obstacle_pass_cm,10,200,"cm","Quãng đi thêm sau khi vật cản biến mất, gồm vùng mù khoảng 25 cm"},
    {"turn_enable",1,&ControlParams::turn_enable,0,1,"1","Bật rẽ theo biển"},
    {"turn_yaw_deg",80.0,&ControlParams::turn_yaw_deg,30,120,"độ","Góc quay đích"},
    {"turn_steer_deg",25.0,&ControlParams::turn_steer_deg,5,40,"độ","Góc lái khi rẽ, không vượt steer_max_deg"},
    {"turn_arm_cm",80.0,&ControlParams::turn_arm_cm,10,300,"cm","Quãng chờ ngã rẽ tối đa"},
    {"turn_start_cm",30.0,&ControlParams::turn_start_cm,0,150,"cm","Quãng từ lúc biển đủ phiếu tới lúc bắt đầu rẽ"},
    {"rb_lost_cm",30.0,&ControlParams::rb_lost_cm,10,100,"cm","Quãng mất phiếu round_barrier liên tục để kết thúc một lần qua vòng xuyến"},
    {"rb_turn1_cm",60.0,&ControlParams::rb_turn1_cm,0,400,"cm","Quãng sau lần qua vòng xuyến thứ 1 tới lúc rẽ trái"},
    {"rb_turn2_cm",60.0,&ControlParams::rb_turn2_cm,0,400,"cm","Quãng sau lần qua vòng xuyến thứ 2 tới lúc rẽ phải"},
    {"rb_yaw_kp",1.0,&ControlParams::rb_yaw_kp,0.1,5,"độ/độ","Hệ số lái giữ hướng khi mất làn trong vòng xuyến"},
    {"rb_hold_max_cm",150.0,&ControlParams::rb_hold_max_cm,20,400,"cm","Quãng giữ hướng tối đa mỗi lần qua vòng xuyến"},
    {"turn_max_cm",70.0,&ControlParams::turn_max_cm,10,200,"cm","Quãng rẽ tối đa"},
    {"turn_max_ms",8000,&ControlParams::turn_max_ms,1000,20000,"ms","Thời gian rẽ tối đa"},
    {"start_wait_green",1,&ControlParams::start_wait_green,0,1,"1","Chờ xanh khi xuất phát"},
    {"vote_window",5,&ControlParams::vote_window,1,30,"lô","Cửa sổ bỏ phiếu"},
    {"vote_min",3,&ControlParams::vote_min,1,30,"lô","Số phiếu tối thiểu"},
    {"green_frames",3,&ControlParams::green_frames,1,30,"lô","Số lô xanh liên tiếp"},
    {"red_conf_min",0.5,&ControlParams::red_conf_min,0,1,"1","Tin cậy đỏ"},
    {"green_conf_min",0.5,&ControlParams::green_conf_min,0,1,"1","Tin cậy xanh"},
    {"off_conf_min",0.5,&ControlParams::off_conf_min,0,1,"1","Tin cậy tắt"},
    {"red_box_min_area_frac",0.0001,&ControlParams::red_box_min_area_frac,0,1,"W*H","Diện tích đỏ tối thiểu"},
    {"green_box_min_area_frac",0.0001,&ControlParams::green_box_min_area_frac,0,1,"W*H","Diện tích xanh tối thiểu"},
    {"off_box_min_area_frac",0.0001,&ControlParams::off_box_min_area_frac,0,1,"W*H","Diện tích tắt tối thiểu"},
    {"approach_cms",5.0,&ControlParams::approach_cms,0,10,"cm/s","Tốc độ tiếp cận đỏ"},
    {"red_stop_x_frac",0.90,&ControlParams::red_stop_x_frac,0,1,"W","Mép phải hộp đỏ để dừng"},

    {"control_period_ms",50,&ControlParams::control_period_ms,20,100,"ms","Chu kỳ điều khiển"},
    {"mask_max_age_ms",300,&ControlParams::mask_max_age_ms,50,1000,"ms","Tuổi mặt nạ tối đa"},
    {"cmd_max_age_ms",200,&ControlParams::cmd_max_age_ms,50,500,"ms","Tuổi lệnh tối đa"},
    {"det_max_age_ms",500,&ControlParams::det_max_age_ms,50,2000,"ms","Tuổi phát hiện tối đa (P2)"},
    {"max_speed_cms",15.0,&ControlParams::max_speed_cms,0,20,"cm/s","Trần tốc độ"},
    {"cruise_cms",15.0,&ControlParams::cruise_cms,0,20,"cm/s","Tốc độ thẳng"},
    {"min_cms",4.0,&ControlParams::min_cms,0,10,"cm/s","Tốc độ thấp nhất khi bám làn"},
    {"lost_cms",4.0,&ControlParams::lost_cms,0,10,"cm/s","Tốc độ giữ khi mất làn"},
    {"slow_per_deg",0.1666666667,&ControlParams::slow_per_deg,0,1,"cm/s/độ","Giảm tốc theo góc"},
    {"steer_max_deg",40.0,&ControlParams::steer_max_deg,0,40,"độ","Trần góc lái"},
    {"steer_rate_dps",200.0,&ControlParams::steer_rate_dps,10,1000,"độ/s","Tốc độ đổi góc tối đa"},
    {"kp",30.0,&ControlParams::kp,0,200,"độ","Hệ số tỉ lệ"},
    {"kd",2.0,&ControlParams::kd,0,50,"độ.s","Hệ số vi phân"},
    {"scan_rows",Values{0.85,0.65,0.45},&ControlParams::scan_rows,0,1,"H","Hàng quét theo chiều cao"},
    {"row_weights",Values{0.5,0.3,0.2},&ControlParams::row_weights,0,1.7976931348623157e308,"1","Trọng số hàng quét"},
    {"lane_width_frac",Values{0.70,0.55,0.40},&ControlParams::lane_width_frac,0,1,"W","Bề rộng làn theo chiều rộng"},
    {"lane_width_min_ratio",0.6,&ControlParams::lane_width_min_ratio,0.1,1,"1","Tỉ lệ bề rộng nhỏ nhất"},
    {"lane_width_max_ratio",1.4,&ControlParams::lane_width_max_ratio,1,3,"1","Tỉ lệ bề rộng lớn nhất"},
    {"center_offset_frac",0.0,&ControlParams::center_offset_frac,-0.5,0.5,"W","Dịch tâm tham chiếu"},
    {"min_valid_rows",1,&ControlParams::min_valid_rows,1,5,"hàng","Số hàng hợp lệ tối thiểu"},
    {"lost_hold_ms",300,&ControlParams::lost_hold_ms,0,2000,"ms","Hạn giữ tốc độ khi mất làn"},
    {"lost_stop_ms",800,&ControlParams::lost_stop_ms,0,3000,"ms","Hạn trả góc về không"},
    {"regain_frames",3,&ControlParams::regain_frames,1,20,"khung","Số khung liên tiếp để phục hồi"},
    {"record_every_n",5,&ControlParams::record_every_n,0,40,"khung","Nhịp ghi ảnh và mặt nạ (0 = tắt ghi ảnh)"},
    {"yolo_every_n",3,&ControlParams::yolo_every_n,1,10,"khung","Nhịp YOLO (P2)"},
    {"right_clip_frac",0.99,&ControlParams::right_clip_frac,0.90,1.0,"W","Ngưỡng vạch phải sát mép"},
    {"right_lost_ms",500,&ControlParams::right_lost_ms,0,2000,"ms","Hạn mất vạch phải (0 = tắt)"},
    {"pair_mode",0,&ControlParams::pair_mode,0,1,"1","Chọn cặp vạch (0 = như cũ, 1 = bao quanh xe)"},
    {"pair_max_shift_frac",0.25,&ControlParams::pair_max_shift_frac,0.05,0.6,"W","Dịch ngang tối đa của vạch giữa hai hàng"},
    {"pair_merge_frac",0.08,&ControlParams::pair_merge_frac,0,0.12,"W","Gộp vạch gần nhau trong một hàng (0 = tắt)"},
    {"kd_filter_ms",0,&ControlParams::kd_filter_ms,0,500,"ms","Hằng thời gian lọc đạo hàm sai số (0 = tắt)"}
};
void Require(bool ok, const std::string &key) {
    if(!ok) throw std::runtime_error("Tham số điều khiển không hợp lệ: " + key);
}
template<class T> void Print(std::ostream &out, const T &value) { out << value; }
void Print(std::ostream &out, const Values &values) {
    out << '[';
    for(size_t i=0;i<values.size();++i) { if(i) out << ','; out << values[i]; }
    out << ']';
}
double Number(const cv::FileNode &node, const char *key) {
    Require(node.isInt() || node.isReal(),key);
    const double value=double(node);
    Require(std::isfinite(value),key);
    return value;
}
}
ControlParams::ControlParams() {
    for(const auto &entry:table) std::visit([&](auto member) {
        using T=typename std::decay<decltype(this->*member)>::type;
        this->*member=std::get<T>(entry.initial);
    },entry.member);
}
void ControlParams::Validate() const {
    for(const auto &entry:table) std::visit([&](auto member) {
        auto check=[&](double value) {
            Require(std::isfinite(value) && value>=entry.low && value<=entry.high,entry.name);
        };
        using T=typename std::decay<decltype(this->*member)>::type;
        if constexpr(std::is_same<T,Values>::value) {
            const auto &values=this->*member;
            Require(!values.empty() && values.size()<=5,entry.name);
            for(double value:values) check(value);
        } else check(this->*member);
    },entry.member);
    Require(row_weights.size()==scan_rows.size(),"row_weights / scan_rows");
    Require(lane_width_frac.size()==scan_rows.size(),"lane_width_frac / scan_rows");
    double sum=0;
    for(size_t i=0;i<scan_rows.size();++i) {
        Require(scan_rows[i]>0 && scan_rows[i]<1,"scan_rows");
        Require(lane_width_frac[i]>0,"lane_width_frac");
        sum+=row_weights[i];
    }
    Require(std::isfinite(sum) && sum>0,"row_weights");
    Require(min_valid_rows<=int(scan_rows.size()),"min_valid_rows / scan_rows");
    Require(min_cms<=cruise_cms && cruise_cms<=max_speed_cms,"min_cms / cruise_cms / max_speed_cms");
    Require(lost_cms<=max_speed_cms,"lost_cms / max_speed_cms");
    Require(vote_min<=vote_window,"vote_min / vote_window");
    // Chỉ ràng buộc khi bật rẽ: yaml cũ có steer_max_deg nhỏ vẫn nạp được lúc rẽ tắt.
    Require(turn_enable==0 || turn_steer_deg<=steer_max_deg,"turn_steer_deg / steer_max_deg");
    Require(turn_start_cm<turn_arm_cm,"turn_start_cm / turn_arm_cm");
    Require(green_frames<=vote_window,"green_frames / vote_window");
    Require(lost_stop_ms>=lost_hold_ms,"lost_stop_ms / lost_hold_ms");
    Require(right_lost_ms==0 || (right_lost_ms>=200 && right_lost_ms<=2000),"right_lost_ms");
    Require(pair_merge_frac==0 || pair_merge_frac>=0.02,"pair_merge_frac");
    // B3 coi hàng đầu danh sách là hàng gần nhất (y_frac lớn nhất): chỉ ép thứ tự khi bật B3.
    if(pair_mode==1) for(size_t i=1;i<scan_rows.size();++i) Require(scan_rows[i]<scan_rows[i-1],"scan_rows (pair_mode=1 cần giảm dần)");
}
ControlParams ControlParams::Load(const std::string &path, std::ostream &report) {
    ControlParams params;
    try {
        std::ifstream input(path,std::ios::binary);
        Require(bool(input),path);
        std::string yaml; std::getline(input,yaml,'\0');
        // Byte NUL làm getline dừng trước EOF, phải từ chối thay vì để OpenCV cắt chuỗi.
        Require(input.eof() && !input.bad(),path);
        if(yaml.compare(0,3,"\xef\xbb\xbf")==0) yaml.erase(0,3);
        // Kiểm tra khung văn bản trước vì các bản OpenCV chấp nhận YAML hỏng khác nhau.
        std::istringstream lines(yaml);
        std::string line,content;
        size_t lineNumber=0;
        bool started=false,ended=false;
        while(std::getline(lines,line)) {
            const auto currentLine=lineNumber++;
            const auto comment=line.find('#');
            if(comment!=std::string::npos &&
               (comment==0 || line[comment-1]==' ' || line[comment-1]=='\t')) line.erase(comment);
            const auto first=line.find_first_not_of(" \t\r");
            if(first==std::string::npos) continue;
            line.erase(line.find_last_not_of(" \t\r")+1);
            const auto text=line.substr(first);
            if(text.compare(0,5,"%YAML")==0) {
                Require(currentLine==0 && first==0 && text=="%YAML:1.0",path);
                continue;
            }
            if(text.compare(0,3,"---")==0) {
                Require(first==0 && text=="---" && !started && !ended && content.empty(),path);
                started=true;
                continue;
            }
            if(text.compare(0,3,"...")==0) {
                Require(first==0 && text=="..." && !ended,path);
                ended=true;
                continue;
            }
            Require(!ended,path);
            content+=line+'\n';
        }
        std::string compact;
        for(char c:content) if(c!=' ' && c!='\t' && c!='\r' && c!='\n') compact+=c;
        std::vector<char> brackets;
        char quote=0;
        for(size_t i=0;i<content.size();++i) {
            const char c=content[i];
            if(quote) {
                if(c=='\\' && quote=='"') ++i;
                else if(c==quote) quote=0;
            } else if(c=='"' || c=='\'') quote=c;
            else if(c=='[' || c=='{') brackets.push_back(c);
            else if(c==']' || c=='}') {
                Require(!brackets.empty() && brackets.back()==(c==']'?'[':'{'),path);
                brackets.pop_back();
            }
        }
        Require(brackets.empty() && !quote,path);
        const bool empty=compact.empty() || compact=="{}";
        Require(compact.compare(0,2,"{}")!=0 || empty,path);
        if(!empty) {
            const auto first=content.find_first_not_of(" \t\r\n");
            Require(content[first]=='{' ||
                    (content[first]!='[' && content.compare(first,2,"- ")!=0 &&
                     content.compare(first,2,"-\t")!=0 && content.compare(first,2,"-\n")!=0 &&
                     content.find(':',first)!=std::string::npos),path);
        }
        cv::FileStorage file;
        cv::FileNode root;
        if(!empty) {
            if(yaml.compare(0,5,"%YAML")!=0) yaml.insert(0,"%YAML:1.0\n");
            file.open(yaml,cv::FileStorage::READ | cv::FileStorage::MEMORY);
            Require(file.isOpened(),path);
            root=file.root();
            Require(root.isMap() && file.root(1).isNone(),path);
        }
        std::vector<std::string> seen;
        if(!empty) for(const auto &node:root) {
            const auto key=node.name();
            const Parameter *found=nullptr;
            for(const auto &entry:table) if(key==entry.name) found=&entry;
            Require(found!=nullptr,key+" (khóa lạ)");
            for(const auto &previous:seen) Require(previous!=key,key+" (khóa trùng)");
            seen.push_back(key);
            std::visit([&](auto member) {
                using T=typename std::decay<decltype(params.*member)>::type;
                if constexpr(std::is_same<T,Values>::value) {
                    Require(node.isSeq() && node.size()>=1 && node.size()<=5,found->name);
                    auto &values=params.*member; values.clear();
                    for(const auto &item:node) values.push_back(Number(item,found->name));
                } else {
                    const double value=Number(node,found->name);
                    Require(value>=found->low && value<=found->high,found->name);
                    if constexpr(std::is_same<T,int>::value) Require(std::floor(value)==value,found->name);
                    params.*member=static_cast<T>(value);
                }
            },found->member);
        }
        params.Validate();
        params.PrintTable(report);
        for(const auto &entry:table) {
            bool present=false;
            for(const auto &key:seen) if(key==entry.name) present=true;
            if(!present) {
                report << entry.name << " = ";
                std::visit([&](auto member) { Print(report,params.*member); },entry.member);
                report << ' ' << entry.unit << " (mặc định)\n";
            }
        }
    } catch(const std::exception &error) {
        throw std::runtime_error("Không đọc được tham số " + path + ": " + error.what());
    }
    return params;
}
void ControlParams::PrintTable(std::ostream &out) const {
    out << "Tên | Giá trị | Mặc định | Min | Max | Đơn vị | Mô tả\n";
    for(const auto &entry:table) {
        out << entry.name << " | ";
        std::visit([&](auto member) { Print(out,this->*member); },entry.member);
        out << " | "; std::visit([&](const auto &value) { Print(out,value); },entry.initial);
        out << " | " << entry.low << " | " << entry.high << " | " << entry.unit << " | " << entry.description << '\n';
    }
}
std::string ControlParams::FileHash(const std::string &path) {
    std::ifstream input(path,std::ios::binary);
    Require(bool(input),path);
    uint64_t hash=UINT64_C(14695981039346656037);
    char byte;
    while(input.get(byte)) { hash^=static_cast<unsigned char>(byte); hash*=UINT64_C(1099511628211); }
    Require(input.eof() && !input.bad(),path);
    std::ostringstream out;
    out << std::hex << std::setfill('0') << std::setw(16) << hash;
    return out.str();
}
}
