# Kết quả huấn luyện YOLO 12 lớp

So sánh bốn biến thể phát hiện vật thể 12 lớp (`red`, `green`, `off`, `left`, `right`, `no_left`, `no_right`, `round_barrier`, `work_barrier`, `parking`, `people`, `obstacle`) huấn luyện trên cùng dataset `det_b4` (xem [docs/datasets.md](../datasets.md)). Model trong repo là D6 (`models/yolo.onnx`).

## Bảng so sánh

| Biến thể | Kiến trúc | Đầu vào (cao × rộng) | mAP50 val | mAP50-95 val | Kích thước ONNX | Suy luận GPU trên Jetson Nano (xe car6) | Ghi chú |
|---|---|---|---|---|---|---|---|
| D2 | YOLOv8n | 256 × 416 | chưa có số liệu | chưa có số liệu | chưa có số liệu | chưa có số liệu | |
| D6 | YOLOv5n (`yolov5nu`) | 192 × 320 | chưa có số liệu | chưa có số liệu | 10.129.770 B | mean 9,148 ms, p99 11,23 ms (đo trên xe) | 2,51 triệu tham số, 1,08 GFLOPs; **dùng khi thi** |
| D7 | YOLOv5n (`yolov5nu`) | 256 × 416 | chưa có số liệu | chưa có số liệu | chưa có số liệu | chưa có số liệu | cùng cỡ đầu vào với D2 |
| D8 | YOLO11n | 288 × 512 | chưa có số liệu | chưa có số liệu | chưa có số liệu | chưa có số liệu | tỉ lệ 16:9 khớp ảnh camera |

Các file kết quả của lần huấn luyện (bảng tổng hợp so sánh, `results.csv` theo epoch, ma trận nhầm lẫn, đường PR) không được lưu cùng mã nguồn, nên cột mAP để trống thay vì chép số chưa đối chiếu được với file gốc. Bảng sẽ được bổ sung khi có lại các file đó.

## Nguồn từng số

- Kiến trúc và cỡ đầu vào: bảng biến thể trong `training/notebooks/yolo_train_car.ipynb`; với D6 khớp `models/yolo_info.json` (`architecture`, `input_hw`).
- Số tham số, GFLOPs, dataset `det_b4`: `models/yolo_info.json`.
- Kích thước ONNX D6: kích thước file `models/yolo.onnx`.
- Thời gian suy luận D6: báo cáo dựng engine TensorRT FP16 trên xe car6 bằng `trtexec` (TensorRT 8.2). Đây chỉ là thời gian GPU của mạng, chưa gồm tiền xử lý (letterbox, chuẩn hóa) và hậu xử lý (NMS); throughput khoảng 108 lần/giây.

## Biến thể dùng khi thi

D6 là mô hình phát hiện dùng khi thi, ghép với mô hình phân vùng mặt đường TwinLiteNet+; thời gian suy luận GPU đo trên xe ~9,1 ms (đầu vào nhỏ nhất trong bốn biến thể). D2, D7, D8 chưa được dựng engine và đo trên xe, nên chưa có so sánh tốc độ cùng điều kiện.

## Ảnh preview

Chưa có: ma trận nhầm lẫn, đường PR, `results.png` và ảnh dự đoán trên tập valid không được lưu lại cùng mã nguồn.
