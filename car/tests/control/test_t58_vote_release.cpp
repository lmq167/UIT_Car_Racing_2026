#include "control/Planner.h"
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
void Check(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}

struct Rig {
    control::Planner planner;
    control::ControlParams p;
    control::PlannerInput in;
    int64_t now=0;
    uint64_t id=0;

    Rig() {
        in.yolo_enabled=true;
        in.mask=cv::Mat(60,200,CV_8UC1,cv::Scalar(0));
        for (int y=0; y<60; ++y) {
            in.mask.at<unsigned char>(y,60)=2;
            in.mask.at<unsigned char>(y,140)=2;
        }
        p.lane_width_frac={0.4,0.4,0.4}; p.kd=0;
    }
    control::Plan step() {
        in.mask_ms=now; in.image_ms=now;
        const auto plan=planner.Step(in,now,p);
        std::cout << "lo=" << id << " ms=" << now
                  << " state=" << control::StateName(plan.state)
                  << " reason=" << plan.reason << " cms=" << plan.cmd.cms << '\n';
        return plan;
    }
    control::Plan go() {
        in.go=true; const auto plan=step(); in.go=false; return plan;
    }
    control::Plan send(std::vector<control::Detection> items) {
        now+=40;
        for (auto& item:items) item.captured_ms=now;
        in.detections={std::move(items),now,++id};
        return step();
    }
    control::Plan send(const std::string& name, float right=1152) {
        return send({{name,cv::Rect2f(right-128,100,128,72),0.5f,now+40}});
    }
};

void Stopped(const control::Plan& plan, control::PlannerState state) {
    Check(plan.cmd.cms==0,"Chưa được nhả phải có cmd.cms==0");
    Check(plan.state==state,"Chưa được nhả phải giữ trạng thái chờ");
}

void Released(const control::Plan& plan) {
    Check(plan.state==control::PlannerState::BAM_LAN && plan.cmd.cms>0,
          "Đủ điều kiện phải nhả BAM_LAN và có lệnh chạy");
}

void Latch(Rig& r, int vote_min, int vote_window=5, int green_frames=3) {
    r.p.start_wait_green=0;
    r.p.vote_min=vote_min; r.p.vote_window=vote_window; r.p.green_frames=green_frames;
    r.p.det_max_age_ms=400;
    Released(r.go());
    for (int i=1; i<vote_min; ++i) r.send("red");
    Stopped(r.send("red"),control::PlannerState::DUNG_DEN_DO);
}

void TwoGreenWaiting(Rig& r, control::PlannerState state) {
    Stopped(r.send("green"),state);
    Stopped(r.send("green"),state);
}

void Run(const std::string& scenario) {
    using control::PlannerState;
    Rig r;
    if (scenario=="hold_2of5" || scenario=="release_2of5") {
        Latch(r,2);
        TwoGreenWaiting(r,PlannerState::DUNG_DEN_DO);
        const auto third=r.send("green"); // ĐĐXXX: còn hai phiếu đỏ tươi.
        if (scenario=="release_2of5") {
            // Kiểm cả lô nhả dù nền cũ đã nhả sớm ở lô trước.
            const auto fourth=r.send("green"); // ĐXXXX: chỉ còn một phiếu đỏ.
            Released(fourth);
        }
        Stopped(third,PlannerState::DUNG_DEN_DO);
    } else if (scenario=="hold_1of5") {
        Latch(r,1);
        TwoGreenWaiting(r,PlannerState::DUNG_DEN_DO);
        Stopped(r.send("green"),PlannerState::DUNG_DEN_DO);
        Stopped(r.send("green"),PlannerState::DUNG_DEN_DO);
        Released(r.send("green"));
    } else if (scenario=="start_2of5") {
        r.p.start_wait_green=1;
        r.p.vote_min=2; r.p.vote_window=5; r.p.green_frames=3;
        r.p.det_max_age_ms=400;
        Stopped(r.go(),PlannerState::CHO_DEN_XUAT_PHAT);
        // Đỏ hợp lệ nhưng chưa tới vạch dừng, để kiểm riêng chốt xuất phát.
        Stopped(r.send("red",800),PlannerState::CHO_DEN_XUAT_PHAT);
        Stopped(r.send("red",800),PlannerState::CHO_DEN_XUAT_PHAT);
        TwoGreenWaiting(r,PlannerState::CHO_DEN_XUAT_PHAT);
        Stopped(r.send("green"),PlannerState::CHO_DEN_XUAT_PHAT);
        Released(r.send("green"));
    } else if (scenario=="start_green_only") {
        r.p.start_wait_green=1;
        r.p.vote_min=2; r.p.vote_window=5; r.p.green_frames=3;
        Stopped(r.go(),PlannerState::CHO_DEN_XUAT_PHAT);
        TwoGreenWaiting(r,PlannerState::CHO_DEN_XUAT_PHAT);
        Released(r.send("green"));
        Check(r.id==static_cast<uint64_t>(r.p.green_frames),"Không đỏ: nhả đúng xanh thứ green_frames");
    } else if (scenario=="repeat_batch") {
        Latch(r,2);
        for (int i=0; i<10; ++i) {
            ++r.now;
            Stopped(r.step(),PlannerState::DUNG_DEN_DO);
        }
        Stopped(r.send("green"),PlannerState::DUNG_DEN_DO);
        for (int i=0; i<10; ++i) {
            ++r.now;
            Stopped(r.step(),PlannerState::DUNG_DEN_DO);
        }
        Stopped(r.send("green"),PlannerState::DUNG_DEN_DO);
        Stopped(r.send("green"),PlannerState::DUNG_DEN_DO);
        Released(r.send("green"));
        Check(r.id==6,"Đọc lặp không đổi số lô xanh cần để nhả");
    } else if (scenario=="window1") {
        Latch(r,1,1,1); // green_frames không được vượt vote_window.
        Released(r.send("green"));
    } else if (scenario=="even_2of4") {
        Latch(r,2,4,2);
        TwoGreenWaiting(r,PlannerState::DUNG_DEN_DO); // ĐĐXX: đủ xanh nhưng còn hai đỏ.
        Released(r.send("green")); // ĐXXX: chỉ còn một đỏ.
    } else if (scenario=="mixed_batch") {
        Latch(r,2);
        TwoGreenWaiting(r,PlannerState::DUNG_DEN_DO);
        Stopped(r.send("green"),PlannerState::DUNG_DEN_DO);
        const std::vector<control::Detection> mixed={
            {"red",cv::Rect2f(672,100,128,72),0.5f,0},
            {"green",cv::Rect2f(1024,100,128,72),0.5f,0}};
        // ĐXXX(ĐX): đỏ right=800 vẫn là phiếu đỏ, không nhả và ngắt chuỗi xanh.
        Stopped(r.send(mixed),PlannerState::DUNG_DEN_DO);
        TwoGreenWaiting(r,PlannerState::DUNG_DEN_DO);
        Released(r.send("green"));

        Rig votes; Latch(votes,3); votes.p.green_frames=1;
        Stopped(votes.send("green"),PlannerState::DUNG_DEN_DO);
        Stopped(votes.send(mixed),PlannerState::DUNG_DEN_DO);
        // Còn hai đỏ, nhưng chỉ hai phiếu xanh: lô hỗn hợp không được tính xanh.
        Stopped(votes.send("green"),PlannerState::DUNG_DEN_DO);
        Released(votes.send("green"));
    } else if (scenario=="stop_while_waiting") {
        Latch(r,2);
        r.in.stop=true;
        Check(r.step().cmd.cms==0,"Đang DUNG_DEN_DO, stop phải giữ lệnh 0");
    } else if (scenario=="relatch_after_release") {
        Latch(r,2);
        TwoGreenWaiting(r,PlannerState::DUNG_DEN_DO);
        Stopped(r.send("green"),PlannerState::DUNG_DEN_DO);
        Released(r.send("green"));
        Released(r.send("red"));
        Stopped(r.send("red"),PlannerState::DUNG_DEN_DO);
    } else if (scenario=="default_3of5") { // Hồi quy: giữ nguyên luật mặc định.
        Check(r.p.vote_min==3 && r.p.vote_window==5 && r.p.green_frames==3,
              "Mặc định phải là 3/5, green_frames=3");
        r.p.start_wait_green=0;
        Released(r.go());
        r.send("red"); r.send("red");
        Stopped(r.send("red"),PlannerState::DUNG_DEN_DO);
        TwoGreenWaiting(r,PlannerState::DUNG_DEN_DO);
        Released(r.send("green")); // ĐĐĐXXX: nhả tại lô YOLO số 6 (xanh thứ 3).
        Check(r.id==6,"Mặc định phải nhả đúng lô YOLO số 6");
    } else if (scenario=="stale_red") { // Hồi quy: đỏ quá tuổi không chặn nhả.
        Latch(r,2);
        r.now+=r.p.det_max_age_ms+1;
        Stopped(r.step(),PlannerState::DUNG_DEN_DO);
        TwoGreenWaiting(r,PlannerState::DUNG_DEN_DO);
        // Tổng mới có năm lô; hai phiếu đỏ phải bị loại do tuổi, không do dung lượng.
        Released(r.send("green"));
    } else {
        throw std::runtime_error("Tên ca T58 không hợp lệ");
    }
}
}

int main(int argc, char** argv) {
    try {
        Check(argc==2,"Cần đúng một đối số là tên ca T58");
        Run(argv[1]);
        std::cout << "ĐẠT: " << argv[1] << '\n';
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "KHÔNG ĐẠT: " << e.what() << '\n';
        return 1;
    }
}
