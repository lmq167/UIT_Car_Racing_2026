#include "StTransport.h"
#include <chrono>
#include <thread>
#include <sstream>
#include <exception>

namespace tof {
namespace {
Hardware* hardware = nullptr;
std::string lastError;
bool deadlineActive = false;
bool deadlineExpired = false;
std::chrono::steady_clock::time_point deadline;
void describeError(const char* operation, uint8_t address8, uint16_t reg, uint32_t count, const char* reason) {
    std::ostringstream out;
    out << operation << " address7=0x" << std::hex << static_cast<unsigned>(address8 >> 1)
        << " register=0x" << reg << std::dec << " count=" << count << ": " << reason;
    lastError = out.str();
}
}
void setStHardware(Hardware* value) { hardware = value; lastError.clear(); stClearDeadline(); }
const std::string& stLastError() { return lastError; }
void stSetDeadlineMs(unsigned timeoutMs) {
    deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    deadlineActive = true;
    deadlineExpired = false;
    lastError.clear();
}
void stClearDeadline() { deadlineActive = false; deadlineExpired = false; }
bool stDeadlineExpired() {
    if (deadlineActive && std::chrono::steady_clock::now() >= deadline) deadlineExpired = true;
    return deadlineExpired;
}
int stRead(uint8_t address8, uint16_t reg, bool wide, uint8_t* data, uint32_t count) noexcept {
    try {
        lastError.clear();
        if (stDeadlineExpired()) {
            describeError("read", address8, reg, count, "ST operation deadline exceeded");
            return -1;
        }
        if (!hardware || !data || !count) return -1;
        hardware->readBlock(address8 >> 1, reg, wide, data, count);
        return 0;
    } catch (const std::exception& error) {
        describeError("read", address8, reg, count, error.what());
        return -1;
    } catch (...) {
        describeError("read", address8, reg, count, "Unknown transport error");
        return -1;
    }
}
int stWrite(uint8_t address8, uint16_t reg, bool wide, const uint8_t* data, uint32_t count) noexcept {
    try {
        lastError.clear();
        if (stDeadlineExpired()) {
            describeError("write", address8, reg, count, "ST operation deadline exceeded");
            return -1;
        }
        if (!hardware || !data || !count) return -1;
        hardware->writeBlock(address8 >> 1, reg, wide, data, count);
        return 0;
    } catch (const std::exception& error) {
        describeError("write", address8, reg, count, error.what());
        return -1;
    } catch (...) {
        describeError("write", address8, reg, count, "Unknown transport error");
        return -1;
    }
}
uint32_t monotonicMs() {
    return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}
void sleepUs(int32_t us) {
    if (us > 0) std::this_thread::sleep_for(std::chrono::microseconds(us));
}
}
