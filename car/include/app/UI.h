#pragma once

#include "common/Types.h"
#include "hardware/SH1106.h"

namespace app
{
class UI
{
public:
    void Render(SH1106& oled, const common::ControlCommand& command);

};
} // namespace app
