#include "common/Logger.h"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace common
{
std::mutex Logger::mutex_;
LogLevel Logger::level_{LogLevel::Info};
thread_local DeferredDeviceLog *DeferredDeviceLog::current_=nullptr;

DeferredDeviceLog::DeferredDeviceLog() : parent_(current_) { current_=this; }
DeferredDeviceLog::~DeferredDeviceLog() noexcept {
    current_=parent_;
    try {
        for(const auto &entry:entries_)
            Logger::Log(entry.level,entry.component,entry.operation,entry.message);
    } catch(...) { /* Lỗi đầu ra không được thay thế lỗi thiết bị đang xử lý. */ }
}

void Logger::SetLevel(LogLevel level)
{
    std::lock_guard<std::mutex> lock(mutex_);
    level_ = level;
}

void Logger::Debug(const std::string& component,
                   const std::string& operation,
                   const std::string& message)
{
    Log(LogLevel::Debug, component, operation, message);
}

void Logger::Info(const std::string& component,
                  const std::string& operation,
                  const std::string& message)
{
    Log(LogLevel::Info, component, operation, message);
}

void Logger::Warn(const std::string& component,
                  const std::string& operation,
                  const std::string& message)
{
    Log(LogLevel::Warn, component, operation, message);
}

void Logger::Error(const std::string& component,
                   const std::string& operation,
                   const std::string& message)
{
    Log(LogLevel::Error, component, operation, message);
}

void Logger::Fatal(const std::string& component,
                   const std::string& operation,
                   const std::string& message)
{
    Log(LogLevel::Fatal, component, operation, message);
}

void Logger::Log(LogLevel level,
                 const std::string& component,
                 const std::string& operation,
                 const std::string& message)
{
    if(DeferredDeviceLog::current_) {
        DeferredDeviceLog::current_->entries_.push_back({level,component,operation,message});
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (level_ == LogLevel::Off ||
        static_cast<int>(level) < static_cast<int>(level_))
    {
        return;
    }

    const auto now = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &time);
#else
    localtime_r(&time, &tm);
#endif

    std::ostringstream timestamp;
    timestamp << std::put_time(&tm, "%F %T");

    std::cout << '[' << timestamp.str() << "] "
              << '[' << LevelName(level) << "] "
              << '[' << component << "] "
              << '[' << operation << "] "
              << "message=\"" << message << "\""
              << std::endl;
}

const char* Logger::LevelName(LogLevel level)
{
    switch (level)
    {
    case LogLevel::Debug: return "DEBUG";
    case LogLevel::Info: return "INFO";
    case LogLevel::Warn: return "WARN";
    case LogLevel::Error: return "ERROR";
    case LogLevel::Fatal: return "FATAL";
    case LogLevel::Off: return "OFF";
    }
    return "INFO";
}
} // namespace common
