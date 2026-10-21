#include "hardware/SH1106.h"

#include <iostream>

int main()
{
    SH1106 oled;
    if (!oled.Init())
    {
        std::cerr << "OLED init failed\n";
        return 1;
    }

    oled.clear();
    oled.drawString(1, 1, "OLED TEST");
    oled.drawString(1, 15, "UCR2026");
    oled.drawString(1, 29, "OK");
    oled.display();
    return 0;
}
