// Quyết định L/T05g(f), S2: auto xin ±10 qua API công khai, không nhận phím xác nhận.
// Giữ main thật để kiểm cả CLI/cổng smoke/vòng đời; chỉ thay controller trước Start.
#include "app/App.h"
#include <cstdlib>
#include <memory>

namespace {
class ConstantController final : public app::IController {
public:
    common::ControlCommand step(const common::Observation &) override {
        common::ControlCommand command;
        command.speedCmPerSec = std::getenv("ACC_T05G_REVERSE") ? -10.0f : 10.0f;
        return command;
    }
};
}

extern "C" void __real__ZN3app3App5StartEv(app::App *instance);
extern "C" void __wrap__ZN3app3App5StartEv(app::App *instance) {
    instance->SetController(std::unique_ptr<app::IController>(new ConstantController));
    __real__ZN3app3App5StartEv(instance);
}
