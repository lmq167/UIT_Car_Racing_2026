#pragma once
#include "common/Types.h"
namespace app {
// Giao diện điều khiển nhận quan sát chỉ đọc và trả lệnh mong muốn.
class IController {
public:
    virtual ~IController() = default;
    virtual common::ControlCommand step(const common::Observation &observation) = 0;
};
class StopController : public IController {
public:
    common::ControlCommand step(const common::Observation &) override { return {}; }
};
}
