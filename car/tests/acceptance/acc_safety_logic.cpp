// Test chấp nhận – lớp an toàn (mức logic) và mặc định cấu hình.
//
// Hành vi mong đợi suy ra CHỈ từ yêu cầu:
//  - Quy tắc an toàn khi chạy xe: (a) dừng khi mất ảnh quá 0,5 s; (b) có giới hạn tốc độ tối đa trong config;
//    (c) lớp an toàn ToF phía trước (phanh khi vật quá gần) BẬT MẶC ĐỊNH.
//  - Yêu cầu an toàn của chương trình xe: "chặn tốc độ/góc theo config; ToF trước < ngưỡng và đang tiến -> tốc độ 0;
//    mất ảnh > 0,5 s ở chế độ auto -> dừng".
//  - btc_code_notes.md mục 5/6.1: kiểm tra số hữu hạn (NaN/vô cực); "không coi khoảng cách -1 là đường trống";
//    dữ liệu lỗi/quá cũ phải có chính sách dừng; ToF đang lùi không bị chặn bởi ToF trước.
//  - docs/ucr_car.md (bảng khoá): tof_brake_mm "so sánh nhỏ hơn ngưỡng"; tof_timeout_ms "mất/không hợp lệ/quá cũ
//    cũng chặn tiến"; image_timeout_ms "mất ảnh trả tốc độ và góc tương đối về 0"; tof_names theo slot.
// Giao diện dùng: include/app/Safety.h (SafeCommand), include/common/Config.h, include/common/Types.h.
#include "acc_common.h"
#include "app/Safety.h"
#include "common/Config.h"
#include "common/Types.h"
#include <cmath>
#include <limits>

using common::Clock;
using common::ControlCommand;
using common::Observation;

namespace {

// Quan sát "khoẻ": ảnh vừa chụp, cả 4 ToF hợp lệ ở 2000 mm và vừa cập nhật.
Observation Healthy(const common::Config &c, Clock::time_point now) {
    Observation o;
    o.frame_id = 7;
    o.captured_at = now;
    o.sensor_at = now;
    for (auto &t : o.tof) {
        t.initialized = true; t.hasSample = true; t.valid = true; t.timedOut = false;
        t.distanceMm = 2000; t.updatedAt = now;
    }
    (void)c;
    return o;
}

ControlCommand Run(const common::Config &c, const Observation &o, ControlCommand cmd, Clock::time_point now) {
    std::string state;
    return app::SafeCommand(c, o, cmd, now, state);
}

bool Near(float a, float b, float eps = 1e-3f) { return std::fabs(a - b) <= eps; }

void SpeedAndSteerLimits() {
    // Yêu cầu: "chặn tốc độ/góc theo config" => vượt giới hạn thì bị chặn ĐÚNG giới hạn.
    common::Config c;
    c.max_speed_cms = 12.5f;
    c.max_steering_deg = 10;
    const auto now = Clock::now();
    const auto o = Healthy(c, now);

    auto r = Run(c, o, {1000.f, 1000}, now);
    ACC_CHECK(Near(r.speedCmPerSec, 12.5f), "tien 1000 cm/s phai bi chan dung 12.5, nhan " + std::to_string(r.speedCmPerSec));
    ACC_CHECK(r.steeringDeg == 10, "goc 1000 phai bi chan dung 10, nhan " + std::to_string(r.steeringDeg));

    r = Run(c, o, {-1000.f, -1000}, now);
    ACC_CHECK(Near(r.speedCmPerSec, -12.5f), "lui -1000 phai bi chan dung -12.5, nhan " + std::to_string(r.speedCmPerSec));
    ACC_CHECK(r.steeringDeg == -10, "goc -1000 phai bi chan dung -10, nhan " + std::to_string(r.steeringDeg));

    // Trong giới hạn: giữ nguyên (không bị cắt oan), cả tiến lẫn lùi.
    r = Run(c, o, {7.5f, 3}, now);
    ACC_CHECK(Near(r.speedCmPerSec, 7.5f) && r.steeringDeg == 3, "lenh trong gioi han phai giu nguyen (7.5, 3)");
    r = Run(c, o, {-7.5f, -3}, now);
    ACC_CHECK(Near(r.speedCmPerSec, -7.5f) && r.steeringDeg == -3, "lenh lui trong gioi han phai giu nguyen");
    r = Run(c, o, {12.5f, 10}, now);
    ACC_CHECK(Near(r.speedCmPerSec, 12.5f) && r.steeringDeg == 10, "lenh dung bang gioi han phai giu nguyen");
    r = Run(c, o, {12.6f, 11}, now);
    ACC_CHECK(Near(r.speedCmPerSec, 12.5f) && r.steeringDeg == 10, "lenh vuot nhe gioi han (12.6, 11) phai bi chan ve (12.5, 10)");

    // Giới hạn đổi theo config chứ không viết cứng.
    common::Config c2;
    c2.max_speed_cms = 5.0f;
    c2.max_steering_deg = 4;
    r = Run(c2, Healthy(c2, now), {20.f, 20}, now);
    ACC_CHECK(Near(r.speedCmPerSec, 5.0f) && r.steeringDeg == 4, "gioi han phai doc tu config (5.0, 4), nhan " +
              std::to_string(r.speedCmPerSec) + "," + std::to_string(r.steeringDeg));

    // Số không hữu hạn: kết quả phải hữu hạn và nằm trong giới hạn (btc_code_notes mục 5: kiểm tra số hữu hạn).
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    for (float bad : {nan, inf, -inf}) {
        r = Run(c, o, {bad, 5}, now);
        ACC_CHECK(std::isfinite(r.speedCmPerSec) && std::fabs(r.speedCmPerSec) <= 12.5f + 1e-3f,
                  "toc do khong huu han phai thanh so huu han trong gioi han, nhan " + std::to_string(r.speedCmPerSec));
    }
}

void TofBrakeForward() {
    // Yêu cầu: "ToF trước < ngưỡng và đang tiến -> tốc độ 0"; docs: "so sánh nhỏ hơn ngưỡng".
    common::Config c;
    c.tof_brake_mm = 250;
    const int front = c.FrontSlot();
    ACC_CHECK(front >= 0 && front < 4, "FrontSlot phai nam trong 0..3");
    const auto now = Clock::now();

    auto with_front = [&](int mm) {
        Observation o = Healthy(c, now);
        o.tof[front].distanceMm = mm;
        return o;
    };

    ACC_CHECK(Near(Run(c, with_front(249), {10.f, 0}, now).speedCmPerSec, 0.f), "front 249 < 250 khi tien phai ve 0");
    ACC_CHECK(Near(Run(c, with_front(0), {10.f, 0}, now).speedCmPerSec, 0.f), "front 0 mm khi tien phai ve 0");
    ACC_CHECK(Near(Run(c, with_front(100), {10.f, 4}, now).speedCmPerSec, 0.f), "front 100 mm khi tien (co lai) phai ve 0");
    ACC_CHECK(Near(Run(c, with_front(250), {10.f, 0}, now).speedCmPerSec, 10.f), "front == nguong (250) KHONG phanh (so sanh nho hon)");
    ACC_CHECK(Near(Run(c, with_front(251), {10.f, 0}, now).speedCmPerSec, 10.f), "front 251 khong phanh");
    ACC_CHECK(Near(Run(c, with_front(2000), {10.f, 0}, now).speedCmPerSec, 10.f), "front 2000 khong phanh");

    // Ngưỡng lấy từ config.
    common::Config c2;
    c2.tof_brake_mm = 400;
    Observation o2 = Healthy(c2, now);
    o2.tof[c2.FrontSlot()].distanceMm = 399;
    ACC_CHECK(Near(Run(c2, o2, {10.f, 0}, now).speedCmPerSec, 0.f), "nguong 400: front 399 phai phanh");
    o2.tof[c2.FrontSlot()].distanceMm = 400;
    ACC_CHECK(Near(Run(c2, o2, {10.f, 0}, now).speedCmPerSec, 10.f), "nguong 400: front 400 khong phanh");

    // Đứng yên (tốc độ 0) không sinh tốc độ khác 0.
    ACC_CHECK(Near(Run(c, with_front(10), {0.f, 0}, now).speedCmPerSec, 0.f), "toc do 0 giu nguyen 0");

    // Đang LÙI: ToF trước gần KHÔNG được chặn lùi.
    ACC_CHECK(Near(Run(c, with_front(30), {-10.f, 0}, now).speedCmPerSec, -10.f), "lui khi front gan khong bi chan boi ToF truoc");
    Observation invalid = with_front(-1);
    invalid.tof[front].valid = false;
    ACC_CHECK(Near(Run(c, invalid, {-10.f, 0}, now).speedCmPerSec, -10.f), "lui khi front = -1 khong bi chan boi ToF truoc");

    // Chỉ ToF TRƯỚC mới phanh: các hướng khác gần thì vẫn tiến được.
    Observation others = Healthy(c, now);
    for (int i = 0; i < 4; ++i) if (i != front) others.tof[i].distanceMm = 20;
    ACC_CHECK(Near(Run(c, others, {10.f, 0}, now).speedCmPerSec, 10.f), "ToF sau/trai/phai gan khong duoc phanh tien");
}

void TofSlotMapping() {
    // Yêu cầu: "tof_names: [front, rear, left, right] theo thứ tự slot": vị trí 'front' do config quyết định.
    common::Config c;
    c.tof_names = {"rear", "front", "left", "right"};
    ACC_CHECK(c.FrontSlot() == 1, "tof_names={rear,front,left,right} => FrontSlot()==1, nhan " + std::to_string(c.FrontSlot()));
    c.tof_names = {"left", "right", "rear", "front"};
    ACC_CHECK(c.FrontSlot() == 3, "tof_names={left,right,rear,front} => FrontSlot()==3");

    c.tof_names = {"rear", "front", "left", "right"};
    const auto now = Clock::now();
    Observation o = Healthy(c, now);
    o.tof[0].distanceMm = 50;  // slot 0 là 'rear' => không phanh tiến
    ACC_CHECK(Near(Run(c, o, {10.f, 0}, now).speedCmPerSec, 10.f), "slot 0 = rear gan: khong phanh tien");
    o.tof[0].distanceMm = 2000;
    o.tof[1].distanceMm = 50;  // slot 1 là 'front' => phanh
    ACC_CHECK(Near(Run(c, o, {10.f, 0}, now).speedCmPerSec, 0.f), "slot 1 = front gan: phai phanh tien");
}

void TofInvalidAndStale() {
    // Yêu cầu: btc_code_notes mục 5 "không coi khoảng cách -1 là đường trống"; docs tof_timeout_ms "mất/không hợp lệ/quá cũ
    // cũng chặn tiến". Quy tắc áp dụng trong test: ToF trước không hợp lệ/-1/quá cũ => chặn TIẾN (tốc độ 0), không chặn lùi.
    common::Config c;
    const int front = c.FrontSlot();
    const auto now = Clock::now();

    Observation inv = Healthy(c, now);
    inv.tof[front].distanceMm = -1;
    inv.tof[front].valid = false;
    ACC_CHECK(Near(Run(c, inv, {10.f, 0}, now).speedCmPerSec, 0.f), "front = -1 (valid=false) khi tien phai ve 0");

    Observation neg = Healthy(c, now);
    neg.tof[front].distanceMm = -1;  // valid=true nhưng khoảng cách -1: vẫn là không hợp lệ
    ACC_CHECK(Near(Run(c, neg, {10.f, 0}, now).speedCmPerSec, 0.f), "front distance -1 (du valid=true) khi tien phai ve 0");

    Observation none;  // chưa từng có mẫu ToF nào (mọi trường mặc định)
    none.frame_id = 3; none.captured_at = now;
    ACC_CHECK(Near(Run(c, none, {10.f, 0}, now).speedCmPerSec, 0.f), "chua co mau ToF (mac dinh) khi tien phai ve 0");

    Observation stale = Healthy(c, now);
    stale.tof[front].updatedAt = now - std::chrono::milliseconds(c.tof_timeout_ms + 300);
    ACC_CHECK(Near(Run(c, stale, {10.f, 0}, now).speedCmPerSec, 0.f), "mau ToF qua cu (> tof_timeout_ms) khi tien phai ve 0");

    Observation fresh = Healthy(c, now);
    fresh.tof[front].updatedAt = now - std::chrono::milliseconds(c.tof_timeout_ms / 5);
    ACC_CHECK(Near(Run(c, fresh, {10.f, 0}, now).speedCmPerSec, 10.f), "mau ToF con moi (tuoi 1/5 timeout) khong bi chan");

    // Tắt phanh ToF bằng config thì không chặn (cờ tồn tại để chỉnh tại phòng).
    common::Config off;
    off.tof_safety_enabled = 0;
    Observation o = Healthy(off, now);
    o.tof[off.FrontSlot()].distanceMm = 10;
    ACC_CHECK(Near(Run(off, o, {10.f, 0}, now).speedCmPerSec, 10.f), "tof_safety_enabled=0: khong phanh");
}

void ImageLoss() {
    // Yêu cầu: quy tắc an toàn (a) "dừng khi mất ảnh quá 0,5 s"; docs image_timeout_ms "mất ảnh trả tốc độ và góc tương đối về 0".
    common::Config c;
    ACC_CHECK(c.image_timeout_ms > 0 && c.image_timeout_ms <= 500, "image_timeout_ms mac dinh phai trong (0, 500]");
    const auto now = Clock::now();

    Observation old = Healthy(c, now);
    old.captured_at = now - std::chrono::milliseconds(c.image_timeout_ms + 100);
    auto r = Run(c, old, {10.f, 7}, now);
    ACC_CHECK(Near(r.speedCmPerSec, 0.f) && r.steeringDeg == 0, "anh cu hon nguong: toc do va goc phai ve 0");
    r = Run(c, old, {-10.f, -7}, now);
    ACC_CHECK(Near(r.speedCmPerSec, 0.f) && r.steeringDeg == 0, "anh cu hon nguong: ca khi lui cung phai ve 0");

    Observation ok = Healthy(c, now);
    ok.captured_at = now - std::chrono::milliseconds(c.image_timeout_ms - 100);
    r = Run(c, ok, {10.f, 7}, now);
    ACC_CHECK(Near(r.speedCmPerSec, 10.f) && r.steeringDeg == 7, "anh con trong han (nguong-100ms) khong bi chan");

    Observation never = Healthy(c, now);
    never.frame_id = 0;  // chưa nhận được ảnh nào
    never.captured_at = now;
    r = Run(c, never, {10.f, 7}, now);
    ACC_CHECK(Near(r.speedCmPerSec, 0.f) && r.steeringDeg == 0, "chua co anh nao (frame_id=0): phai dung");

    // Ngưỡng theo config.
    common::Config c2;
    c2.image_timeout_ms = 100;
    Observation o2 = Healthy(c2, now);
    o2.captured_at = now - std::chrono::milliseconds(150);
    r = Run(c2, o2, {10.f, 7}, now);
    ACC_CHECK(Near(r.speedCmPerSec, 0.f) && r.steeringDeg == 0, "image_timeout_ms=100, anh cu 150ms: phai dung");

    // Mất ảnh thắng mọi thứ: lệnh cực đoan vẫn ra 0.
    r = Run(c, old, {1e9f, 1000000}, now);
    ACC_CHECK(Near(r.speedCmPerSec, 0.f) && r.steeringDeg == 0, "mat anh + lenh cuc doan van phai ra 0");
}

void Defaults() {
    // Yêu cầu: quy tắc an toàn (b)(c) – giới hạn tốc độ có trong config; phanh ToF bật mặc định.
    common::Config c;
    ACC_CHECK(c.tof_safety_enabled == 1, "tof_safety_enabled mac dinh phai bat (quy tac an toan: lop an toan ToF bat mac dinh)");
    ACC_CHECK(c.max_speed_cms > 0.f, "max_speed_cms mac dinh phai > 0");
    ACC_CHECK(c.max_steering_deg > 0 && c.max_steering_deg <= 60, "max_steering_deg mac dinh trong (0, 60]");
    ACC_CHECK(c.tof_brake_mm > 0, "tof_brake_mm mac dinh phai > 0");
    ACC_CHECK(c.tof_names.size() == 4 && c.FrontSlot() == 0, "tof_names mac dinh [front,rear,left,right] => FrontSlot()==0");
    ACC_CHECK(c.control_hz >= 10 && c.sensor_hz >= 10, "control_hz / sensor_hz mac dinh >= 10");
}

void MissingKeysUseDefaults() {
    // Yêu cầu: "Thiếu khoá -> dùng mặc định và in cảnh báo" (phần in cảnh báo được kiểm ở acc_cli.sh).
    const std::string dir = acc::MakeTempDir("acc_cfg_");
    ACC_CHECK(!dir.empty(), "tao duoc thu muc tam");
    const std::string path = dir + "/it_khoa.yaml";
    ACC_CHECK(acc::WriteFile(path, "%YAML:1.0\n---\nmax_speed_cms: 7.0\n"), "ghi duoc file cau hinh thieu khoa");
    common::Config ref;
    common::Config c;
    bool threw = false;
    try { c.Load(path); } catch (...) { threw = true; }
    ACC_CHECK(!threw, "cau hinh thieu khoa khong duoc lam Load nem loi (phai dung mac dinh)");
    if (!threw) {
        ACC_CHECK(Near(c.max_speed_cms, 7.0f), "khoa co mat phai doc dung (7.0)");
        ACC_CHECK(c.tof_safety_enabled == ref.tof_safety_enabled && c.tof_safety_enabled == 1, "khoa thieu tof_safety_enabled => mac dinh bat");
        ACC_CHECK(c.image_timeout_ms == ref.image_timeout_ms, "khoa thieu image_timeout_ms => mac dinh");
        ACC_CHECK(c.max_steering_deg == ref.max_steering_deg, "khoa thieu max_steering_deg => mac dinh");
        ACC_CHECK(c.tof_brake_mm == ref.tof_brake_mm, "khoa thieu tof_brake_mm => mac dinh");
    }
    unlink(path.c_str());
    rmdir(dir.c_str());
}

}  // namespace

int main() {
    SpeedAndSteerLimits();
    TofBrakeForward();
    TofSlotMapping();
    TofInvalidAndStale();
    ImageLoss();
    Defaults();
    MissingKeysUseDefaults();
    return acc::Finish("acc_safety_logic");
}
