#pragma once
#ifdef UCR_FAKE_HW
#include "common/Types.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <locale>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
namespace app { namespace fake {
// Một mốc cho toàn tiến trình, kể cả trước khi tạo Backend và các luồng.
inline const auto started=common::Clock::now();
inline long long Millis() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(common::Clock::now()-started).count();
}
inline std::string Env(const char *name) { const char *v=std::getenv(name); return v ? v : ""; }
inline void Trace(const char *kind,const std::string &value) {
    static std::mutex mutex;
    std::lock_guard<std::mutex> lock(mutex);
    const auto path=Env("UCR_FAKE_HW_TRACE");
    if(path.empty()) return;
    std::ofstream out(path,std::ios::app);
    out.imbue(std::locale::classic());
    out << Millis() << ',' << kind << ',' << value << '\n';
    out.flush();
    if(!out) throw std::runtime_error("Không ghi được vết phần cứng giả: "+path);
}
template<class T> inline void TraceNumber(const char *kind,T value) {
    // Giữ nguyên giá trị cả khi công cụ đọc vết dùng double cho lệnh float.
    std::ostringstream out; out.imbue(std::locale::classic()); out.precision(std::numeric_limits<double>::max_digits10); out << value;
    Trace(kind,out.str());
}
inline long long Time(const std::string &text) {
    if(text.empty() || text.find_first_not_of("0123456789")!=std::string::npos)
        throw std::runtime_error("Mốc thời gian giả không hợp lệ: "+text);
    return std::stoll(text);
}
struct TofRow { long long ms; std::array<int,4> mm; };
inline const std::vector<TofRow> &TofScript() {
    static const auto rows=[] {
        std::vector<TofRow> result;
        const auto path=Env("UCR_FAKE_TOF_SCRIPT");
        if(path.empty()) return result;
        std::ifstream in(path);
        if(!in) throw std::runtime_error("Không mở được kịch bản ToF giả: "+path);
        std::string line;
        while(std::getline(in,line)) {
            if(line.find_first_not_of(" \t\r")==std::string::npos) continue;
            std::istringstream row(line); std::string time,extra; TofRow sample{};
            if(!(row>>time>>sample.mm[0]>>sample.mm[1]>>sample.mm[2]>>sample.mm[3]) || (row>>extra))
                throw std::runtime_error("Dòng ToF giả không hợp lệ: "+line);
            sample.ms=Time(time);
            for(int mm:sample.mm) if(mm < -1) throw std::runtime_error("Khoảng cách ToF giả phải >= -1: "+line);
            result.push_back(sample);
        }
        if(in.bad()) throw std::runtime_error("Lỗi đọc kịch bản ToF giả: "+path);
        return result;
    }();
    return rows;
}
inline bool CameraGap() {
    static const auto gaps=[] {
        std::vector<std::pair<long long,long long>> result;
        const auto text=Env("UCR_FAKE_CAMERA_GAPS");
        if(text.empty()) return result;
        std::istringstream in(text); std::string part;
        while(std::getline(in,part,',')) {
            const auto colon=part.find(':');
            if(colon==std::string::npos) throw std::runtime_error("Khoảng ngắt camera giả không hợp lệ: "+part);
            const auto begin=Time(part.substr(0,colon)),end=Time(part.substr(colon+1));
            if(end<=begin) throw std::runtime_error("Khoảng ngắt camera giả phải có kết thúc > bắt đầu");
            result.emplace_back(begin,end);
        }
        if(text.back()==',') throw std::runtime_error("Khoảng ngắt camera giả bị rỗng");
        return result;
    }();
    const auto now=Millis();
    for(const auto &gap:gaps) if(now>=gap.first && now<gap.second) return true;
    return false;
}
inline std::vector<std::pair<long long,int>> ParseKeys(const std::string &text) {
    std::vector<std::pair<long long,int>> result;
    try {
        std::size_t begin=0;
        while(begin<text.size()) {
            const auto colon=text.find(':',begin);
            if(colon==std::string::npos || colon+1>=text.size())
                throw std::runtime_error("thiếu mốc ms hoặc phím");
            const auto ms=Time(text.substr(begin,colon-begin));
            const auto key=static_cast<unsigned char>(text[colon+1]);
            if(key>127) throw std::runtime_error("phím phải là đúng một ký tự ASCII");
            result.emplace_back(ms,key);
            begin=colon+2;
            if(begin==text.size()) break;
            if(text[begin]!=',' || ++begin==text.size())
                throw std::runtime_error("cần dấu phẩy giữa các mục, không để mục rỗng");
        }
    } catch(const std::exception &e) {
        throw std::runtime_error(std::string("UCR_FAKE_KEYS không hợp lệ; cần <ms>:<phím>[,<ms>:<phím>...], ")+
            "ms là số nguyên trong [0, 9223372036854775807], phím là một ký tự ASCII: "+e.what());
    }
    std::stable_sort(result.begin(),result.end(),[](const auto &a,const auto &b) { return a.first<b.first; });
    return result;
}
inline const std::vector<std::pair<long long,int>> &KeyScript() {
    static const auto keys=ParseKeys(Env("UCR_FAKE_KEYS"));
    return keys;
}
inline int NextKey() {
    const auto &keys=KeyScript();
    static std::size_t next=0;
    if(next==keys.size() || Millis()<keys[next].first) return -1;
    return keys[next++].second;
}
inline void Fault(const char *thread) {
    static const auto spec=[] {
        const auto text=Env("UCR_FAKE_FAULT");
        if(text.empty()) return std::make_pair(std::string(),0LL);
        const auto at=text.find('@');
        if(at==std::string::npos) throw std::runtime_error("Lỗi giả phải có dạng loại@ms");
        const auto kind=text.substr(0,at);
        if(kind!="actuator_throw" && kind!="sensor_throw" && kind!="terminate")
            throw std::runtime_error("Loại lỗi giả không hợp lệ: "+kind);
        return std::make_pair(kind,Time(text.substr(at+1)));
    }();
    static std::atomic<bool> fired{false};
    if(spec.first.empty() || Millis()<spec.second) return;
    if(spec.first!="terminate" && spec.first!=thread) return;
    if(fired.exchange(true)) return;
    if(spec.first=="terminate") std::terminate();
    throw std::runtime_error("Lỗi giả trong luồng: "+spec.first);
}
} }
#endif
