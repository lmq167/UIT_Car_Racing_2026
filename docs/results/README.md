# Kết quả huấn luyện

Các chỉ số chất lượng dùng thang từ 0 đến 1; số thập phân trong bảng dùng dấu phẩy.

## Phân vùng đường TwinLiteNet+

Model `models/twinlitenetplus_192x320.onnx` (biến thể small, 2 lớp `background`/`road`, đầu vào 192 × 320) được huấn luyện bằng `training/notebooks/twinlitenetplus_train.ipynb` trên dataset `ucr_road`.

- Chia tập theo `split.csv` (theo lượt thu; xem [cách chia giữ nguyên khi huấn luyện](../datasets.md#chia-theo-lượt-thu-và-mức-đầy-đủ-metadata)): train 635 / valid 263 ảnh; 108 nhóm, không nhóm nào nằm ở hai tập; không có tập test riêng.
- Checkpoint tốt nhất theo mIoU valid: epoch 96 / 100. Valid vừa dùng để chọn epoch nên số đo hơi lạc quan.

| Tập | IoU road | IoU background | mIoU | Precision road | Recall road |
|---|---|---|---|---|---|
| Valid (263 ảnh) | 0,9659 | 0,9809 | 0,9734 | 0,9801 | 0,9852 |

IoU (tỉ lệ giao trên hợp) tính theo pixel trên mặt nạ 192 × 320; mIoU là trung bình IoU hai lớp; precision/recall tính cho lớp `road`. Các số này đo trên cách chia theo lượt thu, nên không so trực tiếp được với số đo trên cách chia ngẫu nhiên theo ảnh.

Xuất ONNX: opset 12, đầu vào `input` 1×3×192×320, đầu ra `output` 1×2×192×320 (logits). So với PyTorch trên 100 ảnh valid: sai số tuyệt đối lớn nhất 8,8e-06, argmax trùng 100%.

Môi trường huấn luyện: Python 3.13.16, torch 2.14.1+cu130, onnx 1.23.2, onnxruntime 1.32.0; mã nguồn [TwinLiteNetPlus](https://github.com/chequanghuy/TwinLiteNetPlus) commit `90f1b8695ae311d5123b05f8534b2e11e42499d2` (MIT License). Siêu tham số: ô cấu hình đầu notebook.

## Phát hiện YOLO 12 lớp

So sánh bốn biến thể trên bộ dữ liệu `ucr_det`, đánh giá trên tập val (tập kiểm định). `people` là hình nộm, `obstacle` là vật cản và `parking` là biển đỗ xe. Xem [bộ dữ liệu](../datasets.md) và [CSV tổng hợp](yolo_compare_summary.csv).

## Bảng so sánh

| Biến thể | Kiến trúc | Đầu vào (cao × rộng) | Tham số | GFLOPs | mAP50 | mAP50-95 | Precision | Recall | Recall people | Recall obstacle | Recall parking | Train (phút) | Suy luận GPU trên xe |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| D8 | yolo11n | 288 × 512 | 2592180 | 2,33 | 0,907 | 0,682 | 0,868 | 0,826 | 0,893 | 0,905 | 0,769 | 26,8 | — |
| D2 | yolov8n | 256 × 416 | 3013188 | 2,133 | 0,894 | 0,654 | 0,893 | 0,836 | 0,893 | 0,952 | 0,692 | 17,9 | — |
| D7 | yolov5nu | 256 × 416 | 2510804 | 1,869 | 0,869 | 0,629 | 0,89 | 0,803 | 0,857 | 0,952 | 0,462 | 20,5 | — |
| D6 | yolov5nu | 192 × 320 | 2510804 | 1,078 | 0,827 | 0,587 | 0,886 | 0,834 | 0,857 | 0,905 | 0,538 | 16,6 | 9,148 ms (trung bình); 11,23 ms (p99) |

Độ trễ D6 là thời gian GPU của engine (bản thực thi tối ưu) TensorRT FP16 trên Jetson Nano, đo bằng `trtexec`; p99 là phân vị 99. Phép đo chưa gồm tiền xử lý, hậu xử lý hay các tác vụ điều khiển.

`—`: không đo. D6 có lượng tính toán thấp nhất nên được triển khai trực tiếp; D2, D7, D8 không dựng engine trên xe.

## Nhầm lẫn biển rẽ

Các cột là số hộp nhãn thật bị dự đoán thành lớp ở bên phải mũi tên, không phải tỉ lệ. Bảng tổng hợp lấy trung bình giữa các lần chạy ở độ sáng gốc và cùng ngưỡng tin cậy.

| Biến thể | left → no_left | no_left → left | right → no_right | no_right → right |
|---|---|---|---|---|
| D8 | 0,0 | 0,0 | 0,0 | 0,0 |
| D2 | 0,0 | 0,0 | 0,0 | 0,0 |
| D7 | 0,0 | 0,0 | 0,0 | 0,0 |
| D6 | 0,0 | 0,0 | 0,0 | 0,0 |

Các giá trị bằng 0 chỉ xác nhận không có nhầm lẫn giữa các cặp này trong phép đánh giá; không chứng minh mọi biển đều được phát hiện.

## Recall khi giảm độ sáng

| Biến thể | recall_people_toi06 | recall_obstacle_toi06 |
|---|---|---|
| D8 | 0,893 | 0,905 |
| D2 | 0,893 | 0,952 |
| D7 | 0,857 | 0,905 |
| D6 | 0,75 | 0,905 |

## Giải nghĩa cột

- `variant`, `model`: mã biến thể và kiến trúc; `input_h`, `input_w`: chiều cao, chiều rộng đầu vào suy luận. `params` là tổng số phần tử tham số mạng; `gflops` là lượng phép toán dấu phẩy động theo tỉ đơn vị, tính bằng `get_flops` ở đúng cỡ đầu vào.
- `train_min`: số phút từ lúc bắt đầu tạo/huấn luyện mạng đến khi lưu trọng số và siêu dữ liệu; mã tính thời gian thực rồi chia 60, làm tròn một chữ số thập phân. Khi dùng lại trọng số, hàm trả thời gian rỗng. Summary lấy giá trị đầu của nhóm, không lấy trung bình thời gian.
- `map50`: mAP (trung bình độ chính xác trung bình) tại IoU (tỉ lệ giao trên hợp) 0,50; `map50_95`: mAP trên các ngưỡng IoU từ 0,50 đến 0,95. Mã lấy `m.box.map50` và `m.box.map` từ `det.val(split="val", conf=0.001, iou=0.6)`. Tham số `iou=0.6` ở lời gọi này là ngưỡng NMS (loại hộp trùng), không phải ngưỡng IoU tính mAP50. Các giá trị mAP được chép lại vào mọi hàng độ sáng; chúng không phải mAP đo lại trên ảnh giảm sáng.
- `map50_people`, `map50_obstacle`: AP tại IoU 0,50 riêng từng lớp từ `m.box.ap50`.
- `precision` (độ chính xác dự đoán) = số hộp đúng lớp / tổng hộp dự đoán; `recall` (độ bao phủ) = số hộp đúng lớp / tổng hộp nhãn thật. Mã cộng số hộp trên toàn tập val, ghép hộp một-một theo IoU giảm dần với IoU ≥ 0,5; hộp không ghép được tính là thừa hoặc bỏ sót.
- `recall_people`, `recall_obstacle`, `recall_parking`: recall riêng từng lớp; `precision_people`, `precision_obstacle`: precision riêng từng lớp. `parking` đo phát hiện biển P, không đo tỉ lệ hoàn tất thao tác đỗ.
- `confuse_A_as_B`: số hộp lớp thật A ghép với dự đoán lớp B; hàng ma trận là lớp thật, cột là dự đoán.
- Precision, recall và nhầm lẫn ở bảng chính dùng `EVAL_CONF=0,25`, `NMS_IOU=0,45`, độ sáng 1,0. Summary lọc đúng bộ dữ liệu, biến thể và ngưỡng, rồi lấy trung bình từng chỉ số giữa các hàng cùng biến thể, làm tròn ba chữ số thập phân.
- `recall_people_toi06`, `recall_obstacle_toi06`: recall khi nhân các kênh ảnh với 0,6, chặn về miền ảnh hợp lệ; vẫn dùng `conf=0,25`. Hậu tố này biểu thị độ sáng, không phải ngưỡng tin cậy 0,6.

## Phân tích

D6 là biến thể dùng khi thi vì có lượng tính toán nhỏ nhất: 1,078 GFLOPs (xấp xỉ 1,08 GFLOPs), đầu vào 192 × 320. Đây là đánh đổi tốc độ/độ chính xác: mAP50 của D6 là 0,827, thấp hơn D2 (0,894) và D8 (0,907); GFLOPs không thay thế phép đo độ trễ thực tế.

D2 có độ chính xác cao hơn mức cần thiết cho bài toán trên sa bàn: biển báo có thể được nhận từ khoảng 2 m, trong khi xe phải đi qua biển rồi mới thực hiện tín hiệu. D6 vẫn đủ chính xác cho nhu cầu đó và xử lý nhanh vượt trội (1,078 GFLOPs so với 2,133 GFLOPs của D2), nên D6 được chọn cho xe.

Recall biển đỗ còn thấp: D6 đạt 0,538, D7 đạt 0,462; D8 cao nhất trong nhóm ở 0,769. Khi giảm độ sáng xuống 0,6, recall hình nộm của D6 giảm từ 0,857 xuống 0,750, trong khi recall vật cản giữ ở 0,905. Các chỉ số val mô tả khả năng phát hiện trên tập kiểm định, không xác nhận an toàn hay hiệu quả điều khiển trên sa bàn.

FPS (số khung hình mỗi giây) toàn hệ thống chưa đo.

Chưa có ảnh preview YOLO.

## Nguồn từng số

- Toàn bộ giá trị trong các bảng, trừ thời gian suy luận: bảng tổng hợp `yolo_compare_summary.csv` do notebook YOLO sinh ra; bản chép nguyên văn nằm [tại đây](yolo_compare_summary.csv).
- Định nghĩa cột: [training/notebooks/yolo_train_car.ipynb](../../training/notebooks/yolo_train_car.ipynb), số ô tính từ 1, dòng tính trong phần source của ô: ô 2 dòng 16–20 và 45–53 (ngưỡng, biến thể); ô 10 dòng 1–25 (thời gian train); ô 12 dòng 10–19, 28–65 (ghép hộp, val, mAP, precision/recall, nhầm lẫn); ô 18 dòng 12–21 (tham số, GFLOPs, đầu vào); ô 22 dòng 5–16 (lọc, tổng hợp, độ sáng 0,6, làm tròn).
- Độ trễ D6: `trtexec` (TensorRT 8.2) khi dựng engine FP16 trên Jetson Nano của xe; chỉ đo thời gian GPU của mạng.
