#pragma once

#include "common/Logger.h"

#include <exception>
#include <memory>

namespace app
{
    // An operation failure is reported to its caller; it never stops the application.
    template <typename Operation>
    bool RunDeviceOperation(const char *name, const char *operation, Operation action,
                            bool logFailure = true)
    {
        try
        {
            if (action()) return true;
            if (logFailure)
                common::Logger::Error(name, operation, "operation failed; application continues");
        }
        catch (const std::exception &ex)
        {
            common::Logger::Error(name, operation, ex.what());
        }
        catch (...)
        {
            common::Logger::Error(name, operation, "unknown exception; application continues");
        }
        return false;
    }

    // Retain partially initialized devices and their ports until explicit shutdown.
    template <typename Device, typename Initializer>
    bool InitializeDevice(const char *name, std::unique_ptr<Device> &, Initializer action)
    {
        try
        {
            common::Logger::Info(name, "init", "initialization starting");
            if (action())
            {
                common::Logger::Info(name, "init", "device ready");
                return true;
            }
            common::Logger::Error(name, "init", "initialization failed; application continues");
        }
        catch (const std::exception &ex)
        {
            common::Logger::Error(name, "init", ex.what());
        }
        catch (...)
        {
            common::Logger::Error(name, "init", "unknown initialization exception; application continues");
        }
        return false;
    }
} // namespace app
