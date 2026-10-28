// T05h §4: tiêm lỗi encoder và logger nghẽn tại API công khai, không sửa mã xe.
// Giới hạn: đây là biên Position; không xác minh vị trí khoá bên trong nhánh UART thật.
#include "app/Backend.h"
#include "app/FakeHardware.h"
#include "common/Logger.h"
#include <atomic>
#include <fstream>

// C1: đánh dấu ngay cửa vào khởi tạo, trước cả UART/camera để bắt tác dụng phụ sớm.
extern "C" void __real__ZN3app7Backend4InitEv(app::Backend *);
extern "C" void __wrap__ZN3app7Backend4InitEv(app::Backend *backend) {
    app::fake::Trace("backend_init", "1");
    __real__ZN3app7Backend4InitEv(backend);
}

extern "C" bool __real__ZN3app7Backend8PositionERf(app::Backend *, float &);
extern "C" bool __wrap__ZN3app7Backend8PositionERf(app::Backend *backend, float &cm) {
    const bool ok = __real__ZN3app7Backend8PositionERf(backend, cm);
    static std::atomic<bool> injected{false};
    const auto path = app::fake::Env("ACC_T05H_ENCODER_GATE");
    if (!path.empty() && std::ifstream(path).good() && !injected.exchange(true)) {
        app::fake::Trace("encoder_no_reply", "1");
        common::Logger::Warn("Motor", "GetPositionCm", std::string(1024 * 1024, 'X'));
        app::fake::Trace("encoder_logger_returned", "1");
        return false;
    }
    return ok;
}
