#pragma once
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>

namespace auto_drive {
struct HookState {
    std::atomic<bool> cameraStall{false}, roadStall{false}, roadThrow{false};
    std::atomic<bool> controllerStall{false}, controllerThrow{false};
    std::atomic<bool> writerStall{false}, jpegFail{false}, csvFail{false};
    std::atomic<bool> overridePending{false};
    std::atomic<double> overrideCms{0}, overrideDeg{0}, freeMb{-1};
    std::atomic<int> controlDelay{0};
    std::atomic<int64_t> cameraPauseUntil{0};
};
extern HookState hooks;
inline bool fakeOpenFail(const char* name) {
#if defined(UCR_AUTO_TEST_HOOKS) && UCR_AUTO_TEST_HOOKS
    const char* v=std::getenv("UCR_AUTO_FAKE_OPEN_FAIL");
    return v && std::string(v)==name;
#else
    (void)name; return false;
#endif
}
inline bool fakeMotorFail() {
#if defined(UCR_AUTO_TEST_HOOKS) && UCR_AUTO_TEST_HOOKS
    const char* v=std::getenv("UCR_AUTO_FAKE_MOTOR_FAIL");
    return v && std::string(v)=="1";
#else
    return false;
#endif
}
#if defined(UCR_AUTO_TEST_HOOKS) && UCR_AUTO_TEST_HOOKS
void pollHooks(int64_t now, const std::function<void(const char*,const std::string&)>& trace);
#else
inline void pollHooks(int64_t, const std::function<void(const char*,const std::string&)>&) {}
#endif
}
