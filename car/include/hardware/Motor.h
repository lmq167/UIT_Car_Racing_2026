#ifndef MOTOR_H
#define MOTOR_H

#include <string>
#include <cstddef>
#include <cstdint>
#include <termios.h>

class Motor
{
public:
    /**
     * @brief Khởi tạo đối tượng Motor với đường dẫn cổng kết nối
     * @param device_port Đường dẫn tới cổng serial (\
     */
    Motor(const char *device_port = "/dev/ttyTHS1") : _device_port(device_port), _device(-1) {}

    /**
     * @brief Giải phóng tài nguyên và đóng cổng kết nối nếu đang mở
     */
    ~Motor();

    /**
     * @brief Thiết lập cấu hình cổng Serial (Baudrate 115200, Raw mode, Timeout)
     * @return true nếu khởi tạo thành công, false nếu thất bại
     */
    bool Init();
    // Chỉ mở cổng và đặt định dạng UART, không gửi lệnh tới driver.
    bool OpenPortOnly();
    // Cổng riêng, ghi không chặn; không đọc/sửa trạng thái UART hay cấu hình driver.
    bool EmergencyStop() const noexcept;
    bool LastWriteOk() const { return lastWriteOk_; }
    // Chuyển chế độ vận hành trong RAM, không lưu ROM, không khởi tạo lại.
    void PositionControl(bool enabled);

    // Bật bản ghi UART gửi/nhận, kể cả phản hồi rỗng.
    void SetUartLogging(bool enabled) { _uartLogging = enabled; }

    /**
     * @brief Gửi một chuỗi lệnh ASCII UART trực tiếp tới driver
     * @param cmd Chuỗi lệnh cần gửi (phải kết thúc bằng \n)
     */
    void SendCommand(const char *cmd, bool IsReadRes = true);

    /**
     * @brief Đọc toàn bộ dữ liệu phản hồi từ bộ đệm của driver (nếu có) và log ra console
     */
    void ReadDriverResponse();

    /**
     * @brief Gửi lệnh lưu cấu hình hiện tại vào ROM và thực hiện reset driver ($101=1)
     */
private:
    void SaveAndResetDriver();

    /**
     * @brief Thực hiện chuỗi lệnh khởi tạo ban đầu (Cài đặt phương thức giao tiếp, dòng encoder, mode...)
     */
    void SendInitCommand();

    /**
     * @brief Thay đổi chế độ điều khiển của driver
     * @param mode 1: Chế độ Vận tốc PI (M3), 2: Chế độ Vị trí thông minh (M4)
     */
    void SetMode(uint8_t mode);
public:

    /**
     * @brief Điều khiển motor chạy với vận tốc góc cố định (Chế độ Vận tốc)
     * @param Rad Tốc độ góc bằng đơn vị Rad/s
     */
    void SetSpeedRad(float Rad);

    /**
     * @brief Điều khiển motor chạy với vận tốc tuyến tính (Chế độ Vận tốc)
     * @param Cms Tốc độ dài bằng đơn vị Cm/s (Quy đổi dựa trên đường kính bánh xe 6.5cm)
     */
    void SetSpeedCms(float Cms);

    /**
     * @brief Điều khiển kết hợp cả Vị trí đích và Vận tốc tối đa bằng đơn vị Radian (Chế độ Vị trí)
     * @param Position Vị trí đích cần tới (Rad)
     * @param Rad Vận tốc tối đa (Rad/s)
     */
    void SetSpeedPositionRad(float Position, float Rad);

    /**
     * @brief Điều khiển kết hợp cả Vị trí đích và Vận tốc tối đa bằng đơn vị Centimet (Chế độ Vị trí)
     * @param PositionCm Khoảng cách đích cần di chuyển tới (Cm)
     * @param Cms Vận tốc tuyến tính tối đa (Cm/s)
     */
    void SetSpeedPositionCms(float PositionCm, float Cms);

    bool GetPositionCm(float *positionCm);

private:
    const char *_device_port; ///< Lưu đường dẫn cổng kết nối serial
    int _device;              ///< File descriptor quản lý cổng kết nối sau khi open()
    bool _uartLogging = false;
    bool lastWriteOk_ = false;
    const float D = 6.5;
    const float Pi = 3.1416;
    const uint16_t PulsePerRound = 330;

    bool GetStatus(char *buffer, size_t bufferSize);
};

#endif // MOTOR_H
