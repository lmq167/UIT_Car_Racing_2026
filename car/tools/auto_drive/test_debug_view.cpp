// Kiểm bằng điểm ảnh các hàm vẽ gỡ lỗi (--debug-windows) trên dữ liệu tổng hợp; không cần màn hình.
#include "../collect_common/debug_view.h"
#include "debug_fill.h"
#include <chrono>
#include <iostream>
#include <memory>

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { std::cerr << __LINE__ << ": " #cond "\n"; ++failures; } } while (0)

namespace {
using drive::PreviewDebug;
bool near(const cv::Vec3b& p, int b, int g, int r, int tol = 1) {
    return std::abs(p[0] - b) <= tol && std::abs(p[1] - g) <= tol && std::abs(p[2] - r) <= tol;
}
double ms(std::chrono::steady_clock::time_point a) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - a).count();
}
// Mặt đường 160x96 giữa cột 40..119 (lớp 1), vạch lớp 2 ở cột 80.
cv::Mat roadMask() {
    cv::Mat mask(96, 160, CV_8UC1, cv::Scalar(0));
    mask(cv::Rect(40, 0, 80, 96)).setTo(1);
    mask.col(80).setTo(2);
    return mask;
}
control::ControlDebug planFixture() {
    control::ControlDebug plan;
    plan.state = "BAM_LAN"; plan.reason = "BAM_LAN"; plan.valid_rows = 1; plan.e = 0.9; plan.p = 27; plan.d = 1;
    control::RowDebug good; good.y = 80; good.left = 40; good.right = 120; good.target = 80; good.conf = 1; good.valid = true;
    control::RowDebug bad; bad.y = 60; bad.valid = false; bad.left = 70; bad.right = 90;
    plan.rows = {good, bad}; plan.look_x = 100; plan.look_y = 70;
    return plan;
}

void slotRoundTrip() {
    std::unique_ptr<drive::PreviewSlot<PreviewDebug>> slot(new drive::PreviewSlot<PreviewDebug>());
    std::unique_ptr<PreviewDebug> in(new PreviewDebug()), out(new PreviewDebug());
    in->frame_id = 7; in->mask[100] = 2; in->mask[drive::kDebugMaskBytes - 1] = 1;
    in->yolo_state = 1; in->det_ms = 1234;
    slot->write(*in);
    CHECK(slot->read(*out));
    CHECK(out->frame_id == 7 && out->mask[100] == 2 && out->mask[drive::kDebugMaskBytes - 1] == 1);
    CHECK(out->yolo_state == 1 && out->det_ms == 1234);
}

void maskWindow() {
    std::unique_ptr<PreviewDebug> d(new PreviewDebug());
    d->mask_w = 80; d->mask_h = 48;
    std::fill(d->mask, d->mask + 80 * 48, 0);
    for (int y = 5; y < 10; ++y) for (int x = 10; x < 20; ++x) d->mask[y * 80 + x] = 1;
    for (int y = 5; y < 10; ++y) d->mask[y * 80 + 30] = 2;
    const cv::Mat view = drive::renderDebugMask(*d, 2);
    CHECK(view.cols == 160 && view.rows == 96 && view.type() == CV_8UC3);
    CHECK(near(view.at<cv::Vec3b>(2, 2), 40, 40, 40, 0));       // nền
    CHECK(near(view.at<cv::Vec3b>(12, 25), 200, 100, 30, 0));     // mặt đường
    CHECK(near(view.at<cv::Vec3b>(12, 61), 200, 100, 30, 0));    // lớp 2 cũng là mặt đường
    d->mask_w = 0;
    const cv::Mat blank = drive::renderDebugMask(*d, 2);
    CHECK(blank.cols == 320 && blank.rows == 192);               // placeholder có kích thước cố định
    d->mask_w = 1000; d->mask_h = 1000;                          // quá sức chứa -> cũng placeholder
    CHECK(drive::renderDebugMask(*d, 2).cols == 320);
}

std::unique_ptr<PreviewDebug> overlayFixture() {
    std::unique_ptr<PreviewDebug> d(new PreviewDebug());
    const cv::Mat mask = roadMask();
    const auto plan = planFixture();
    auto_drive::fillPreviewDebug(*d, mask, 1000, 5, &plan, 0.0, cv::Size(1280, 720), cv::Rect(0, 288, 1280, 432));
    return d;
}

void overlay() {
    const auto d = overlayFixture();
    drive::PreviewState state{};
    std::snprintf(state.state, sizeof(state.state), "BAM_LAN");
    state.cmd_speed = 6.5; state.cmd_steer = -12;
    cv::Mat display(360, 640, CV_8UC3, cv::Scalar(100, 100, 100));
    d->boxes[0].x = 100; d->boxes[0].y = 100; d->boxes[0].w = 200; d->boxes[0].h = 100;
    d->boxes[0].confidence = 0.87f; std::snprintf(d->boxes[0].label, sizeof(d->boxes[0].label), "do");
    d->nboxes = 1;
    drive::drawDebugOverlay(display, *d, state, 1040);
    // Hàng hợp lệ (y mask 80 -> điểm ảnh 325) xanh lá, hàng không hợp lệ (y mask 60 -> 280) đỏ.
    CHECK(near(display.at<cv::Vec3b>(325, 100), 0, 255, 0, 0));
    CHECK(near(display.at<cv::Vec3b>(280, 100), 0, 0, 255, 0));
    CHECK(near(display.at<cv::Vec3b>(328, 322), 0, 255, 0, 0));  // chấm tâm làn hợp lệ
    // Phủ alpha 0,4 lên mặt đường (mask x=60,y=20 -> điểm ảnh ~(241,190)); nền mask và ngoài vùng cắt giữ nguyên.
    CHECK(near(display.at<cv::Vec3b>(190, 241), 140, 100, 72));
    CHECK(near(display.at<cv::Vec3b>(190, 40), 100, 100, 100, 0));
    CHECK(near(display.at<cv::Vec3b>(20, 300), 100, 100, 100, 0));  // ngoài vùng cắt
    // Đích điều khiển = trục 80 + e*80 = 152 -> x=608, hàng cuối vùng cắt trừ 10.
    CHECK(near(display.at<cv::Vec3b>(350, 608), 255, 0, 255, 0));
    // Khung YOLO (50,50)-(150,100) màu cam.
    bool orange = false;
    for (int x = 49; x <= 51; ++x) orange = orange || near(display.at<cv::Vec3b>(75, x), 0, 165, 255, 0);
    CHECK(orange);
    // Dữ liệu hỏng: không ném ngoại lệ, không vẽ mask.
    auto broken = overlayFixture();
    broken->nrows = 99; broken->nboxes = -3; broken->mask_w = 100000;
    cv::Mat plain(360, 640, CV_8UC3, cv::Scalar(100, 100, 100));
    drive::drawDebugOverlay(plain, *broken, state, 0);
    CHECK(near(plain.at<cv::Vec3b>(190, 241), 100, 100, 100, 0));
    cv::Mat empty;
    drive::drawDebugOverlay(empty, *d, state, 0);
    CHECK(empty.empty());
}

void fill() {
    std::unique_ptr<PreviewDebug> d(new PreviewDebug());
    auto plan = planFixture();
    auto_drive::fillPreviewDebug(*d, roadMask(), 1234, 9, &plan, 0.1, cv::Size(1280, 720), cv::Rect(0, 288, 1280, 432));
    CHECK(d->mask_w == 160 && d->mask_h == 96 && d->mask[80] == 2 && d->mask[10] == 0);
    CHECK(d->nrows == 2 && d->valid_rows == 1 && d->rows[0].valid == 1 && d->rows[1].valid == 0);
    CHECK(d->rows[1].left == 70.0f && d->rows[1].right == 90.0f && d->rows[0].target == 80.0f && d->rows[0].conf == 1.0f);
    CHECK(d->look_x == 100.0f && d->look_y == 70.0f);
    CHECK(std::abs(d->ref_x - (80 + 0.1f * 160)) < 1e-4 && d->mask_ms == 1234 && d->frame_id == 9);
    CHECK(std::string(d->state) == "BAM_LAN" && d->crop_y == 288);
    // Cắt theo sức chứa, luôn kết thúc chuỗi bằng NUL.
    plan.rows.assign(drive::kDebugMaxRows + 5, plan.rows[1]);
    plan.reason = std::string(200, 'x');
    auto_drive::fillPreviewDebug(*d, cv::Mat(300, 300, CV_8UC1, cv::Scalar(1)), 1, 2, &plan, 0, cv::Size(1280, 720), cv::Rect());
    CHECK(d->mask_w == 0 && d->nrows == drive::kDebugMaxRows);
    CHECK(std::strlen(d->reason) == sizeof(d->reason) - 1);
    auto_drive::fillPreviewDebug(*d, roadMask(), 1, 2, nullptr, 0, cv::Size(1280, 720), cv::Rect());
    CHECK(d->nrows == 0 && d->state[0] == 0 && d->mask_w == 160 && d->look_x < 0);
}

void fillBoxes() {
    std::unique_ptr<PreviewDebug> d(new PreviewDebug());
    CHECK(d->yolo_state == 0 && d->det_ms == -1);
    control::Detections det;
    det.captured_ms = 1000;
    det.items = {{"do", cv::Rect2f(100.4f, 100.6f, 200.5f, 100.4f), 0.87f, 1000},
                 {"abcdefghijklmnopq", cv::Rect2f(300, 200, 50, 60), 0.01f, 1000}};
    auto_drive::fillPreviewBoxes(*d, det, 1500, 500, true);
    CHECK(d->nboxes == 2 && d->yolo_state == 1 && d->det_ms == 1000);
    CHECK(d->boxes[0].x == 100 && d->boxes[0].y == 101 && d->boxes[0].w == 201 && d->boxes[0].h == 100);
    CHECK(std::string(d->boxes[0].label) == "do" && d->boxes[0].confidence == 0.87f);
    CHECK(std::string(d->boxes[1].label) == "abcdefghijklmno" && d->boxes[1].label[15] == '\0');
    CHECK(d->boxes[1].x == 300 && d->boxes[1].y == 200 && d->boxes[1].w == 50 && d->boxes[1].h == 60);
    CHECK(d->boxes[1].confidence == 0.01f);
    auto_drive::fillPreviewBoxes(*d, det, 1501, 500, true);
    CHECK(d->nboxes == 0 && d->yolo_state == 1 && d->det_ms == 1000);
    auto_drive::fillPreviewBoxes(*d, det, 1000, 500, true);
    auto_drive::fillPreviewBoxes(*d, det, 1000, 500, false);
    CHECK(d->nboxes == 0 && d->yolo_state == 0 && d->det_ms == 1000);
    det.items.resize(10, det.items.front());
    auto_drive::fillPreviewBoxes(*d, det, 1000, 500, true);
    CHECK(d->nboxes == 8);
    det.captured_ms = -1;
    auto_drive::fillPreviewBoxes(*d, det, 1000, 500, true);
    CHECK(d->nboxes == 0 && d->yolo_state == 1 && d->det_ms == -1);
}

void boxesOverlay() {
    std::unique_ptr<PreviewDebug> d(new PreviewDebug());
    d->src_w = 1280; d->src_h = 720;
    control::Detections det;
    det.captured_ms = 1000;
    det.items = {{"do", cv::Rect2f(100, 100, 200, 100), 0.87f, 1000}};
    auto_drive::fillPreviewBoxes(*d, det, 1040, 500, true);
    cv::Mat display(360, 640, CV_8UC3, cv::Scalar(100, 100, 100));
    drive::PreviewState state{};
    drive::drawDebugOverlay(display, *d, state, 1040);
    CHECK(near(display.at<cv::Vec3b>(75, 50), 0, 165, 255, 0));
    CHECK(near(display.at<cv::Vec3b>(75, 100), 100, 100, 100, 0));
}

// Chỉ in số đo để ước tính chi phí trên Jetson; không đặt ngưỡng vì máy chạy test khác xe.
void timing() {
    const auto d = overlayFixture();
    std::unique_ptr<drive::PreviewSlot<PreviewDebug>> slot(new drive::PreviewSlot<PreviewDebug>());
    drive::PreviewState state{};
    const auto plan = planFixture();
    const cv::Mat mask = roadMask();
    constexpr int N = 200;
    auto t = std::chrono::steady_clock::now();
    for (int i = 0; i < N; ++i) auto_drive::fillPreviewDebug(*d, mask, i, i, &plan, 0, cv::Size(1280, 720), cv::Rect(0, 288, 1280, 432));
    const double fill_ms = ms(t) / N;
    t = std::chrono::steady_clock::now();
    for (int i = 0; i < N; ++i) slot->write(*d);
    const double write_ms = ms(t) / N;
    t = std::chrono::steady_clock::now();
    std::unique_ptr<PreviewDebug> copy(new PreviewDebug());
    for (int i = 0; i < N; ++i) slot->read(*copy);
    const double read_ms = ms(t) / N;
    cv::Mat display(360, 640, CV_8UC3, cv::Scalar(100, 100, 100));
    t = std::chrono::steady_clock::now();
    for (int i = 0; i < N; ++i) { drive::drawDebugOverlay(display, *d, state, 0); drive::renderDebugMask(*d, 2); }
    const double draw_ms = ms(t) / N;
    std::cout << "debug_view timing ms: fill=" << fill_ms << " slot_write=" << write_ms << " slot_read=" << read_ms
              << " draw_both=" << draw_ms << '\n';
}
}

int main() {
    slotRoundTrip(); maskWindow(); overlay(); fill(); fillBoxes(); boxesOverlay(); timing();
    if (failures) { std::cerr << failures << " ca hỏng\n"; return 1; }
    std::cout << "test_debug_view OK\n";
    return 0;
}
