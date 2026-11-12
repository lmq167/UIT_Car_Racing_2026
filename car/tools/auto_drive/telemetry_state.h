#pragma once
#include "control/Planner.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

namespace auto_drive {
// Gộp lệnh đã gửi vào một atomic để luồng mạng không đọc tốc độ/góc từ hai lượt khác nhau.
inline uint64_t packTelemetryCommand(float speed,int angle) {
    uint32_t speedBits=0;
    std::memcpy(&speedBits,&speed,sizeof(speedBits));
    return (uint64_t(static_cast<uint32_t>(angle))<<32) | speedBits;
}
inline control::Command unpackTelemetryCommand(uint64_t value) {
    const uint32_t speedBits=static_cast<uint32_t>(value),angleBits=static_cast<uint32_t>(value>>32);
    float speed=0; int32_t angle=0;
    std::memcpy(&speed,&speedBits,sizeof(speed));
    std::memcpy(&angle,&angleBits,sizeof(angle));
    return {speed,static_cast<double>(angle)};
}
inline std::array<int,3> telemetryStates(const control::Detections& detections,int64_t frameMs,
                                         const control::Plan& plan,const control::Command& sent) {
    // Bảng mã BTC theo TÊN lớp: không phụ thuộc thứ tự class_names trong model.
    const std::pair<const char*,int> codes[]={{"green",1},{"red",2},{"work_barrier",3},
        {"round_barrier",5},{"obstacle",6},{"right",7},{"no_right",8},{"left",9},
        {"no_left",10},{"parking",11},{"people",13}}; // off không có mã.
    std::array<bool,14> present{};
    std::vector<std::pair<int,float>> signs;
    const auto fresh=[&](int64_t captured) {
        return captured>=0 && captured<=frameMs && frameMs-captured<=500;
    };
    if (fresh(detections.captured_ms)) {
        for (const auto& box:detections.items) {
            if (!fresh(box.captured_ms) || !std::isfinite(box.confidence) || box.confidence<0) continue;
            for (const auto& code:codes) if (box.class_name==code.first) {
                present[code.second]=true;
                signs.emplace_back(code.second,box.confidence);
                break;
            }
        }
    }
    const auto matches=[&](const char* text) {
        return plan.reason==text || plan.debug.reason==text || plan.debug.state==text;
    };
    const auto prefix=[&](const char* text) {
        return plan.reason.find(text)==0 || plan.debug.reason.find(text)==0 || plan.debug.state.find(text)==0;
    };
    if (plan.state==control::PlannerState::DO_XE || plan.state==control::PlannerState::DO_XE_XONG || prefix("DO_XE"))
        present[12]=true;
    const bool bridge=matches("GIU_HUONG_CAU");
    if (bridge) present[4]=true;
    // GIU_HUONG trong lần qua còn mode ROUNDABOUT; GIU_HUONG sau lần qua không suy ra mã 5.
    if (plan.mode==control::TrajMode::ROUNDABOUT || prefix("VX_")) present[5]=true;
    if (plan.mode==control::TrajMode::AVOID_LEFT || plan.mode==control::TrajMode::RETURN_RIGHT ||
        matches("NE_TRAI") || matches("NE_QUA") || matches("VE_PHAI") || matches("KHONG_DUONG_NE")) present[6]=true;
    if (sent.cms==0 && matches("NGUOI_DI_BO")) present[13]=true;
    if (sent.cms==0 && (plan.state==control::PlannerState::DUNG_DEN_DO || matches("DUNG_DEN_DO"))) present[2]=true;

    // Ưu tiên leader chốt: người > đỏ > đang đỗ > vật > vòng xuyến > cầu,
    // rồi biển/đèn YOLO theo độ tin cậy giảm dần; bỏ trùng, thiếu điền 0.
    std::array<int,3> states{};
    std::size_t count=0;
    const auto append=[&](int code) {
        if (count<states.size() && std::find(states.begin(),states.end(),code)==states.end()) states[count++]=code;
    };
    for (int code:{13,2,12,6,5,4}) if (present[code]) append(code);
    std::stable_sort(signs.begin(),signs.end(),[](const auto& a,const auto& b) { return a.second>b.second; });
    for (const auto& sign:signs) append(sign.first);
    return states;
}
}
