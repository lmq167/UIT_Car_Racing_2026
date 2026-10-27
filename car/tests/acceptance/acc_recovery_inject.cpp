// R18/A07: giữ điểm vào main thật; chỉ tiêm yêu cầu +20 qua giao diện trước Start.
// GNU ld đổi lời gọi Start từ main sang đây; không thay logic phím, phục hồi hay tín hiệu.
#ifndef UCR_FAKE_HW
#error "Bộ tiêm điều khiển chi duoc lien ket vao ban phan cung gia"
#endif
#include "app/App.h"
#include <memory>

namespace {
class ForwardController : public app::IController {
public:
    common::ControlCommand step(const common::Observation &) override { return {20,10}; }
};
}
extern "C" void __real__ZN3app3App5StartEv(app::App *instance);
extern "C" void __wrap__ZN3app3App5StartEv(app::App *instance) {
    instance->SetController(std::unique_ptr<app::IController>(new ForwardController));
    __real__ZN3app3App5StartEv(instance);
}
