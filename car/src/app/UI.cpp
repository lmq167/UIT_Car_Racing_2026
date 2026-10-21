#include "app/UI.h"

#include <string>

namespace app
{
void UI::Render(SH1106& oled, const common::ControlCommand& command)
{
    oled.clear();
    oled.drawString(1, 1, "SPD " + std::to_string(command.speedCmPerSec));
    oled.drawString(1, 13, "ANG " + std::to_string(command.steeringDeg));
    oled.display();
}
} // namespace app
