# Kiến trúc hệ thống

## Luồng dữ liệu

Camera cung cấp ảnh mới nhất cùng thời điểm chụp. Nhận thức chạy TwinLiteNet+ để tạo mặt nạ mặt đường và YOLO để tạo hộp đối tượng, điểm tin cậy và tên lớp.
Hai mạng suy luận bằng TensorRT FP16 trên Jetson; JSON đi kèm quy định chuẩn hóa và cách giải mã riêng cho từng mạng.

Planner nhận kết quả nhận thức, IMU (cảm biến quán tính), encoder (bộ đo vòng quay bánh xe) và dữ liệu ToF hai bên khi cần quét bãi đỗ.
LaneController tìm hành lang đi được từ mặt nạ, tính sai lệch và lệnh lái. Planner kết hợp lệnh này với hành vi đang thực hiện để tạo tốc độ/góc lái mong muốn.
Lệnh đi qua kiểm tra an toàn trước khi gửi tới motor và servo; log (nhật ký) lưu diễn biến phục vụ đối chiếu sau lượt chạy.

## Các module trong `car/src`

| Module | Vai trò |
|---|---|
| `perception` | Đọc siêu dữ liệu, quản lý engine TensorRT, phân vùng đường và phát hiện đối tượng |
| `control` | Tham số điều khiển, Planner, LaneController và ParkScan quét ô đỗ |
| `hardware` | Camera, motor UART, servo và màn hình OLED |
| `sensor` | IMU BNO055 và giao tiếp ToF |
| `app` | Vòng đời ứng dụng, kết nối thiết bị, giao diện và đưa lệnh ra phần cứng |
| `common` | Kiểu dữ liệu, cấu hình, ghi nhật ký và truyền trạng thái |

`car/tools/auto_drive` ghép nhận thức và điều khiển cho chương trình tự hành `ucr_auto`.
`car/include` chứa khai báo tương ứng, trong đó có lớp kiểm tra an toàn và các kênh trao đổi dữ liệu.

## Máy trạng thái chính

Planner tổ chức hành vi theo các nhóm: chờ chạy/đèn xanh, bám đường, tiếp cận và xử lý đèn/biển, rẽ, né và vượt vật cản, giữ hướng, quét bãi và đỗ theo pha, dừng do lỗi hoặc hoàn tất.
Đây là mô tả nhóm hành vi, không phải danh sách đầy đủ tên enum (tập giá trị trạng thái) trong mã.

Ý định rẽ được tích lũy qua nhiều khung ảnh thay vì quyết định từ một lần phát hiện.
Khi vào pha rẽ, IMU hỗ trợ xác định góc quay; encoder và giới hạn thời gian/quãng đường chặn hành vi kéo dài.
Né vật cản phân biệt pha né ngang và pha vượt, rồi trở lại bám đường khi điều kiện cho phép.
Đỗ xe gồm tiến/quét, chọn ô, chờ và các pha lùi/căn chỉnh. ToF đo khi xe đứng, không phải phanh tránh va chạm.

## An toàn

- Dữ liệu ảnh và lệnh có giới hạn tuổi. Mất ảnh quá hạn phải đưa xe về dừng; quá trình phục hồi không được dùng lại lệnh cũ.
- Tốc độ và góc lái bị chặn theo giới hạn cấu hình; cấu hình cần phù hợp xe thực tế.
- Watchdog (bộ giám sát hoạt động) phát hiện vòng điều khiển không cập nhật để yêu cầu dừng.
- Khi thoát hoặc lỗi, thứ tự yêu cầu là dừng motor rồi đưa servo về góc không.
- Người vận hành có thể dừng bằng Space tại màn hình xe; việc nạp chương trình không thay cho xác nhận an toàn trên sa bàn.
- ToF hai bên chỉ hỗ trợ đọc khoảng cách/quét bãi; không được diễn giải thành cảm biến bảo vệ trước/sau xe.

Biên dịch hoặc test với phần cứng giả chỉ cung cấp bằng chứng phần mềm. Chiều motor, tâm lái, phản ứng dừng và hành vi trên sa bàn phải xác nhận riêng trên xe thật.
