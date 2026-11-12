#include "../../tools/auto_drive/telemetry_state.h"
#include <array>
#include <cstdlib>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <string>

namespace {
constexpr int64_t kFrameMs = 1000;
std::size_t checks = 0;

control::Detection box(const std::string &name, float confidence=0.5f,
                       int64_t capturedMs=kFrameMs) {
    control::Detection result;
    result.class_name = name;
    result.confidence = confidence;
    result.captured_ms = capturedMs;
    return result;
}

control::Detections batch(std::initializer_list<control::Detection> items,
                          int64_t capturedMs=kFrameMs) {
    control::Detections result;
    result.items = items;
    result.captured_ms = capturedMs;
    return result;
}

void expect(const std::string &name, const control::Detections &detections,
            const std::array<int, 3> &expected, const control::Plan &plan={},
            const control::Command &sent={}, int64_t frameMs=kFrameMs) {
    ++checks;
    const auto actual = auto_drive::telemetryStates(detections, frameMs, plan, sent);
    if (actual != expected) {
        std::cerr << "FAIL " << name << ": nhan [" << actual[0] << ','
                  << actual[1] << ',' << actual[2] << "], can ["
                  << expected[0] << ',' << expected[1] << ',' << expected[2]
                  << "]\n";
        std::exit(EXIT_FAILURE);
    }
}

void testYoloNames() {
    const struct { const char *name; int code; } cases[] = {
        {"green", 1}, {"red", 2}, {"work_barrier", 3},
        {"round_barrier", 5}, {"obstacle", 6}, {"right", 7},
        {"no_right", 8}, {"left", 9}, {"no_left", 10},
        {"parking", 11}, {"people", 13}
    };
    for (const auto &item : cases)
        expect(item.name, batch({box(item.name)}), {item.code, 0, 0});
    expect("off va ten la", batch({box("off"), box("unknown"), box("RED")}),
           {0, 0, 0});
    expect("confidence khong hop le", batch({
           box("people", std::numeric_limits<float>::quiet_NaN()),
           box("red", std::numeric_limits<float>::infinity()),
           box("obstacle", -0.1f), box("green", 0.1f)}), {1, 0, 0});
    expect("rong", batch({}), {0, 0, 0});
}

void testCommandSnapshot() {
    const struct { float speed; int angle; } cases[] = {
        {0, 0}, {12.25f, -17}, {-7.5f, 45}
    };
    for (const auto &item : cases) {
        ++checks;
        const auto command = auto_drive::unpackTelemetryCommand(
            auto_drive::packTelemetryCommand(item.speed, item.angle));
        if (command.cms != item.speed || command.deg != item.angle) {
            std::cerr << "FAIL snapshot lenh: can " << item.speed << ','
                      << item.angle << "; nhan " << command.cms << ','
                      << command.deg << '\n';
            std::exit(EXIT_FAILURE);
        }
    }
}

void testFreshness() {
    expect("lo tuoi 0", batch({box("green")}), {1, 0, 0});
    expect("lo va hop tuoi 500", batch({box("green", 0.5f, 500)}, 500),
           {1, 0, 0});
    expect("lo tuoi 501", batch({box("green")}, 499), {0, 0, 0});
    expect("lo tuong lai", batch({box("green")}, 1001), {0, 0, 0});
    expect("hop tuoi 501", batch({box("red", 0.5f, 499)}), {0, 0, 0});
    expect("hop tuong lai", batch({box("people", 0.5f, 1001)}), {0, 0, 0});
    expect("chi giu hop tuoi", batch({box("red", 0.9f, 499),
           box("people", 0.9f, 1001), box("left", 0.8f, 500),
           box("green", 0.5f)}), {9, 1, 0});
    expect("moc thoi gian 0", batch({box("green", 0.5f, 0)}, 0),
           {1, 0, 0}, {}, {}, 0);

    control::Plan parked;
    parked.state = control::PlannerState::DO_XE;
    expect("lo cu khong xoa trang thai Planner", batch({box("people")}, 499),
           {12, 0, 0}, parked);
}

void testPlanner() {
    const auto empty = batch({});
    control::Plan plan;
    for (const auto state : {control::PlannerState::DO_XE,
                            control::PlannerState::DO_XE_XONG}) {
        plan = {};
        plan.state = state;
        expect("enum do xe", empty, {12, 0, 0}, plan);
    }
    for (const std::string name : {"DO_XE", "DO_XE_LUI1", "DO_XE_XONG"}) {
        plan = {};
        plan.reason = name;
        expect("reason " + name, empty, {12, 0, 0}, plan);
        plan = {};
        plan.debug.state = name;
        expect("debug.state " + name, empty, {12, 0, 0}, plan);
        plan = {};
        plan.debug.reason = name;
        expect("debug.reason " + name, empty, {12, 0, 0}, plan);
    }

    for (int field=0; field<3; ++field) {
        plan = {};
        plan.state = control::PlannerState::GIU_HUONG;
        if (field == 0) plan.reason = "GIU_HUONG_CAU";
        if (field == 1) plan.debug.state = "GIU_HUONG_CAU";
        if (field == 2) plan.debug.reason = "GIU_HUONG_CAU";
        expect("giu huong cau khong thanh vong xuyen", empty, {4, 0, 0}, plan);
    }
    plan = {};
    plan.state = control::PlannerState::GIU_HUONG;
    expect("giu huong sau vong xuyen khong bao dang qua", empty, {0, 0, 0}, plan);
    plan.mode = control::TrajMode::ROUNDABOUT;
    expect("giu huong khi dang qua vong xuyen", empty, {5, 0, 0}, plan);
    for (int field=0; field<3; ++field) {
        plan = {};
        if (field == 0) plan.reason = "GIU_HUONG";
        if (field == 1) plan.debug.state = "GIU_HUONG";
        if (field == 2) plan.debug.reason = "GIU_HUONG";
        expect("nhan giu huong khong bao dang qua", empty, {0, 0, 0}, plan);
    }
    plan = {};
    plan.mode = control::TrajMode::ROUNDABOUT;
    expect("mode vong xuyen", empty, {5, 0, 0}, plan);
    for (const std::string name : {"VX_CHO", "VX_GIU_HUONG"}) {
        plan = {};
        plan.reason = name;
        expect("reason " + name, empty, {5, 0, 0}, plan);
        plan = {};
        plan.debug.state = name;
        expect("debug.state " + name, empty, {5, 0, 0}, plan);
        plan = {};
        plan.debug.reason = name;
        expect("debug.reason " + name, empty, {5, 0, 0}, plan);
    }

    for (const std::string name : {"NE_TRAI", "NE_QUA", "VE_PHAI",
                                   "KHONG_DUONG_NE"}) {
        plan = {};
        plan.reason = name;
        expect("reason " + name, empty, {6, 0, 0}, plan);
        plan = {};
        plan.debug.state = name;
        expect("debug.state " + name, empty, {6, 0, 0}, plan);
        plan = {};
        plan.debug.reason = name;
        expect("debug.reason " + name, empty, {6, 0, 0}, plan);
    }
    for (const auto mode : {control::TrajMode::AVOID_LEFT,
                           control::TrajMode::RETURN_RIGHT}) {
        plan = {};
        plan.mode = mode;
        expect("mode ne vat", empty, {6, 0, 0}, plan);
    }

    const struct { const char *name; int code; } stops[] = {
        {"NGUOI_DI_BO", 13}, {"DUNG_DEN_DO", 2}
    };
    for (const auto &item : stops) {
        for (int field=0; field<3; ++field) {
            plan = {};
            plan.cmd.cms = 10;
            if (field == 0) plan.reason = item.name;
            if (field == 1) plan.debug.state = item.name;
            if (field == 2) plan.debug.reason = item.name;
            expect("dung theo lenh da gui", empty, {item.code, 0, 0}, plan);
            control::Command moving;
            moving.cms = 1;
            plan.cmd.cms = 0;
            expect("dang chay khong bao da dung", empty, {0, 0, 0}, plan, moving);
            moving.cms = -1;
            expect("dang lui khong bao da dung", empty, {0, 0, 0}, plan, moving);
        }
    }
    plan = {};
    plan.state = control::PlannerState::DUNG_DEN_DO;
    expect("enum dung den do", empty, {2, 0, 0}, plan);
    control::Command moving;
    moving.cms = 1;
    expect("enum den do nhung con chay", empty, {0, 0, 0}, plan, moving);
    plan = {};
    plan.state = control::PlannerState::DUNG;
    plan.reason = "SPACE";
    expect("dung thuong khong tu sinh ma", empty, {0, 0, 0}, plan);
}

void testPriorityAndDuplicates() {
    control::Plan plan;
    expect("13 truoc 2", batch({box("red", 1), box("people", 0.1f)}),
           {13, 2, 0});
    plan.state = control::PlannerState::DO_XE;
    expect("2 truoc 12", batch({box("red")}), {2, 12, 0}, plan);
    expect("12 truoc 6", batch({box("obstacle")}), {12, 6, 0}, plan);
    expect("6 truoc 5", batch({box("round_barrier", 1),
           box("obstacle", 0.1f)}), {6, 5, 0});
    plan = {};
    plan.reason = "GIU_HUONG_CAU";
    expect("5 truoc 4", batch({box("round_barrier")}), {5, 4, 0}, plan);
    expect("4 truoc YOLO thuong", batch({box("work_barrier", 1)}),
           {4, 3, 0}, plan);
    plan.state = control::PlannerState::DO_XE;
    plan.mode = control::TrajMode::AVOID_LEFT;
    expect("nhieu tin hieu chi lay ba ma dau", batch({box("work_barrier", 1),
           box("round_barrier", 1), box("red", 0.1f), box("people", 0.1f)}),
           {13, 2, 12}, plan);

    expect("YOLO thuong giam confidence", batch({box("green", 0.1f),
           box("right", 0.8f), box("parking", 0.7f), box("left", 0.9f)}),
           {9, 7, 11});
    expect("hoa giu thu tu hop", batch({box("no_right", 0.8f),
           box("parking", 0.8f), box("green", 0.8f), box("left", 0.8f)}),
           {8, 11, 1});
    expect("bo trung sau khi xep confidence", batch({box("right", 0.2f),
           box("left", 0.8f), box("right", 0.9f), box("no_left", 0.7f)}),
           {7, 9, 10});
    plan = {};
    plan.state = control::PlannerState::DO_XE;
    plan.reason = "DO_XE_LUI1";
    plan.debug.state = "DO_XE_XONG";
    plan.debug.reason = "DO_XE_KIEM";
    expect("Planner khong lap ma", batch({box("parking"), box("parking")}),
           {12, 11, 0}, plan);
    plan = {};
    plan.reason = "NGUOI_DI_BO";
    expect("YOLO va Planner khong lap ma", batch({box("people"),
           box("people"), box("red"), box("red")}), {13, 2, 0}, plan);
}
}

int main() {
    testYoloNames();
    testFreshness();
    testPlanner();
    testPriorityAndDuplicates();
    testCommandSnapshot();
    std::cout << "PASS telemetry: " << checks
              << " checks; ten YOLO, do tuoi, Planner, uu tien, bo trung, snapshot lenh\n";
    return 0;
}
