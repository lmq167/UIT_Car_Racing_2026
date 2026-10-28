#pragma once

#include <mutex>
#include <string>
#include <vector>

namespace common
{
enum class LogLevel
{
    Debug = 0,
    Info,
    Warn,
    Error,
    Fatal,
    Off
};

// Khai báo trước khoá thiết bị: gom thông báo theo luồng, xuất sau khi khoá đã nhả.
class DeferredDeviceLog {
public:
    DeferredDeviceLog();
    ~DeferredDeviceLog() noexcept;
    DeferredDeviceLog(const DeferredDeviceLog&) = delete;
    DeferredDeviceLog& operator=(const DeferredDeviceLog&) = delete;
private:
    friend class Logger;
    struct Entry { LogLevel level; std::string component, operation, message; };
    static thread_local DeferredDeviceLog *current_;
    DeferredDeviceLog *parent_;
    std::vector<Entry> entries_;
};

class Logger
{
public:
    static void SetLevel(LogLevel level);
    static void Debug(const std::string& component,
                      const std::string& operation,
                      const std::string& message);
    static void Info(const std::string& component,
                     const std::string& operation,
                     const std::string& message);
    static void Warn(const std::string& component,
                     const std::string& operation,
                     const std::string& message);
    static void Error(const std::string& component,
                      const std::string& operation,
                      const std::string& message);
    static void Fatal(const std::string& component,
                      const std::string& operation,
                      const std::string& message);

private:
    friend class DeferredDeviceLog;
    static void Log(LogLevel level,
                    const std::string& component,
                    const std::string& operation,
                    const std::string& message);
    static const char* LevelName(LogLevel level);

    static std::mutex mutex_;
    static LogLevel level_;
};
} // namespace common
