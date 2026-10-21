#pragma once
#include "common/Types.h"
#include <mutex>
namespace common {
struct SharedData { std::mutex mutex; Snapshot value; };
}
