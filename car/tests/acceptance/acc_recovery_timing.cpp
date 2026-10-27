// Tự kiểm bộ chờ và cận quan sát: bỏ khung hợp lệ, mơ hồ qua R phải bị phát hiện.
#include "acc_recovery_timing.h"
#include <cstdio>

int main() {
    int failures=0;
    auto check=[&](bool ok,const char *why) {
        if(!ok) { ++failures; std::fprintf(stderr,"[HONG] %s\n",why); }
    };
    using Clock=std::chrono::steady_clock;
    check(acc_fixture::WaitUntil(Clock::now(),[]{return true;}),"Điều kiện đã đúng phải trả ngay");
    check(!acc_fixture::WaitUntil(Clock::now(),[]{return false;}),"Hết giờ, điều kiện sai phải hỏng");
    int polls=0;
    check(acc_fixture::WaitUntil(Clock::now()+std::chrono::seconds(2),[&]{return ++polls==4;}) && polls==4,
          "Phải hỏi lại điều kiện, không chỉ ngủ rồi kiểm một lần");
    const auto start=Clock::now();
    check(!acc_fixture::WaitUntil(start+std::chrono::milliseconds(10),[]{return false;}) &&
          Clock::now()>=start+std::chrono::milliseconds(10),"Điều kiện không tới phải hết hạn");
    const auto exact=acc_fixture::FrameWindow({24,960},{25,2720},1000,2700);
    check(exact.last_latest==960 && exact.first_earliest==2720 &&
          exact.duration_min==1760 && exact.duration_max==1760,"Khung liên tiếp giữ mốc chính xác");
    const auto missed=acc_fixture::FrameWindow({22,880},{27,2800},1000,2700);
    check(missed.last_latest==1000 && missed.first_earliest==2700 &&
          missed.duration_min==1700 && missed.duration_max==1920,"Bỏ khung phải dùng khoảng bao");
    check(missed.duration_max<2000,"Khoảng hoàn toàn dưới R vẫn kiểm được phục hồi");
    const auto ambiguous=acc_fixture::FrameWindow({20,800},{30,2850},1000,2700);
    check(!(ambiguous.duration_max<2000),"Không kết luận mất ngắn nếu khoảng có thể vượt R");
    const auto above=acc_fixture::FrameWindow({22,880},{27,3400},1000,3300);
    check(above.duration_min>2000,"Mất dài phải được chứng minh bằng cận dưới");
    const std::vector<std::pair<int,int>> gaps{{1000,2200}};
    const std::vector<acc_fixture::Frame> fast{{1,880},{2,920},{3,960},{4,2208},{5,2248},{6,2288}};
    const std::vector<acc_fixture::Frame> slow{{1,800},{3,880},{5,960},{6,2208},{8,2292},{10,2376}};
    check(acc_fixture::ObservedPeriodMs(fast,gaps)==40,"Nhịp nhanh đo từ captured_at, bỏ khoảng ngắt");
    check(acc_fixture::ObservedPeriodMs(slow,gaps)==84,"Nhịp quan sát gần 12 FPS vẫn đúng khi bỏ khung");
    check(acc_fixture::ObservedPeriodMs({{1,960},{2,2208}},gaps)==0,
          "Chỉ thấy hai phía ngắt không đủ bằng chứng để đo FPS");
    check(acc_fixture::ObservedPeriodMs({{2,40},{1,80},{3,80}}, {})==0,
          "Không đo chu kỳ từ id lùi hoặc timestamp trùng");
    const auto check_at=acc_fixture::RecoveryCheckMs({6,2208},1000,10,acc_fixture::ObservedPeriodMs(slow,gaps));
    check(check_at==3392 && check_at>2208+1000+10+acc_fixture::ScheduleSlackMs,
          "Hạn sau phục hồi dùng ảnh thực nhận và chu kỳ đo, không dùng mốc ngắt dự kiến");
    check(acc_fixture::RecoveryCheckMs({6,2250},1000,10,84)>2200+1000+2*acc_fixture::ScheduleSlackMs,
          "Ảnh trở lại trễ cần cửa sổ đo vượt mốc kết thúc cố định cũ");
    check(check_at<2208+2000,"Dung sai chu kỳ không biến thành chờ thêm cả giây");
    std::printf("[recovery_timing] %d loi\n",failures);
    return failures ? 1 : 0;
}
