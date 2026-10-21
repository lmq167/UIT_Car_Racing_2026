#pragma once

#include <mutex>
#include <string>

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
    static void Log(LogLevel level,
                    const std::string& component,
                    const std::string& operation,
                    const std::string& message);
    static const char* LevelName(LogLevel level);

    static std::mutex mutex_;
    static LogLevel level_;
};
} // namespace common
