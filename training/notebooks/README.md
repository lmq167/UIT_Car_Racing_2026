# Huấn luyện mô hình cho camera xe car6 (train v0)

Hai notebook tự chứa mã, chạy trên Google Colab (Python hiện đại). Không chạy các thư viện huấn luyện này trên Python 3.6 của Jetson. Quyết định thiết kế: `ucr_car6_data_handover.md` mục 5, 6 (Project).

| Notebook | Chức năng | Biến thể |
|---|---|---|
| `road_train_car.ipynb` | U-Net 3 lớp `background`, `track_surface`, `line`; vật cản = "lỗ" trên mặt sa bàn; gán nhãn sơ bộ `car6_seg` | S1 B-minus 96×160, S2 NanoUNet 96×160, S3 B-minus 128×256, S4 MobileNetV3-small + giải mã 128×256 |
| `yolo_train_car.ipynb` | YOLO 9 lớp `red`, `green`, `off`, `left`, `right`, `no_left`, `no_right`, `round_barrier`, `work_barrier`; gán nhãn sơ bộ `car6_det` | D1 v8n 320, D2 v8n 256×416, D3 v8n 416, D4 v8n 384×640, D5 YOLO11n 320 |
| `export_candidates.ipynb` | Bản cũ (28/10): xuất 6 ứng viên ONNX trọng số thử để đo FPS | Không dùng nữa: đọc `INPUT_HW` từ notebook đường cũ nên sẽ dừng với bản mới. ONNX của từng biến thể mới được xuất trong lúc so sánh (seed đầu) |

Cả hai notebook có `MODE`: `"compare"` (train + đánh giá nhiều biến thể × seed, ghi một CSV chung), `"final"` (FINAL_FIT trên toàn bộ ảnh có nhãn, lấy bước/epoch cuối, xuất ONNX), `"prelabel"` (đẩy dự đoán lên Roboflow để người duyệt).

## Dữ liệu trên Drive

Mọi đầu vào/đầu ra ở `MyDrive/UIT_Car_Racing/Vong3/`:

| Đường dẫn | Nội dung | Tạo bằng |
|---|---|---|
| `data/split_v0.csv` | Chia train/val theo lượt (19 lượt val) | Chép từ `data/round3/data_clean_car6/split_v0.csv` |
| `data/car6_images.zip` | 2.226 ảnh gốc 1280×720 (một lần) | Nén `data/round3/data_clean_car6/images` |
| `data/seg_masks_v0.zip` | PNG mặt nạ 0/1/2, tên `car6_pNNNN_MMMM.png` | `round3_final/tools/build_seg_masks.py` trên export COCO `car6_seg` |
| Roboflow `car6_det` | Version không Resize, không Augmentation | Notebook YOLO tải dạng `yolov8` |

```powershell
# PowerShell trên laptop, thư mục <thư mục repo>
Compress-Archive -Path data\round3\data_clean_car6\images\* -DestinationPath car6_images.zip
python round3_final\tools\build_seg_masks.py --coco <thư mục export COCO> --qa data\round3\seg_qa_v0
Compress-Archive -Path data\round3\seg_masks_v0\* -DestinationPath seg_masks_v0.zip
```

Sau đó tải `car6_images.zip`, `seg_masks_v0.zip`, `split_v0.csv` lên `Vong3/data/` (Drive web hoặc Google Drive cho máy tính). Khoá Roboflow: Colab Secrets `ROBOFLOW_API_KEY` (bật quyền cho notebook) hoặc nhập ẩn khi được hỏi.

## Chạy thử bằng dữ liệu giả (bắt buộc một lần trước khi train thật)

1. Mở notebook trên Colab, môi trường CPU, giữ `FAKE_DATA=True`, chạy tất cả ô.
2. Notebook tự tạo ảnh giả 1280×720, chạy mọi biến thể (đường: 3 bước/biến thể; YOLO: 1 epoch ở cỡ 96), đánh giá, xuất ONNX, thử prelabel cục bộ (không gọi Roboflow).
3. Kết quả mong đợi: không lỗi; `Vong3/road/fake/<giờ>/` và `Vong3/yolo/fake/<giờ>/` có `.pth`/`.onnx`/`_info.json`/`_onnx_check.csv`; `Vong3/compare/road_compare_fake.csv` và `yolo_compare_fake.csv` có dòng cho từng biến thể × seed × mức sáng (YOLO thêm × ngưỡng conf). Chỉ số giả không có ý nghĩa.
4. Kiểm riêng S4 (MobileNetV3) và D5 (YOLO11n) xuất được opset 12; build TensorRT 8.2 vẫn phải thử trên xe.

## Train thật trên Colab miễn phí (T4)

Chọn GPU, đặt `FAKE_DATA=False`.

**Đường:** `DATA_TAG` theo đợt nhãn (`seg_b1`, `seg_b2`…). Mặc định `VARIANTS=["S1","S2","S3"]`, `SEEDS=[0,1]`, `STEPS=4000` (ước ~6–15 phút một lần chạy; tăng cường chạy trên CPU nên CPU là nút cổ chai). Thêm `"S4"` khi muốn thử encoder tiền huấn luyện.

**YOLO:** `RF_VERSION`, `DATA_TAG` (`det_b1`…). Mặc định `VARIANTS=["D1","D2","D3","D5"]`, `EPOCHS=60`. D2 và D3 dùng chung một lần train. Ước ~25–40 phút một lần train ở 320–416, tổng ~3 giờ cho 3 lần train × 2 seed; nếu phiên bị ngắt, chạy lại từ đầu: lần chạy đã có trong CSV được bỏ qua, trọng số trong `Vong3/yolo/weights/<DATA_TAG>/` được dùng lại. D4 (640) chậm hơn ~2,5 lần, chỉ thêm khi còn thời gian GPU.

## Quy ước chính

- **Chia tập:** theo lượt (`split_v0.csv`); ảnh không có trong file chia làm notebook dừng. Không chia ngẫu nhiên theo ảnh. FINAL_FIT train trên train + val.
- **Đường, tiền xử lý:** cắt `CROP_Y = (320, 720)` của ảnh 1280×720, `cv2.resize INTER_AREA` về cỡ biến thể, RGB /255. Mặt nạ thu nhỏ giữ `line` ở ô có vạch.
- **Đường, train:** CE (trọng số `line` × `LINE_WEIGHT`) + Dice; lấy mẫu theo lượt (tối đa `PASS_CAP`=40 ảnh/lượt/epoch); tăng cường sáng/tương phản/ám màu, hầm, nhoè, nhiễu, vệt chói, lật ngang; dán vật ngẫu nhiên gán `background`; giữ bước cuối.
- **Đường, chỉ số** (`compare/road_compare.csv`, mỗi dòng = lần chạy × mức sáng): IoU từng lớp, mIoU; sai số mép `track_surface`/`line` ở hàng 0,60/0,70/0,80; lỗ thật: tỉ lệ bắt theo dải hàng `near`/`mid`/`far` (tạm thay cho cm khi chưa có ảnh hiệu chuẩn A4), lỗ giả/100 ảnh không lỗ; lỗ tổng hợp: vật chưa thấy dán lên ảnh val.
- **YOLO, tiền xử lý:** letterbox giữ tỉ lệ bằng INTER_AREA, đệm 114, căn giữa (ảnh train được thu nhỏ trước cùng cách). Ánh xạ lớp theo tên. Không lật ngẫu nhiên; bản lật đổi nhãn chỉ cho ảnh có `left`/`right` và không có `no_left`/`no_right`.
- **YOLO, chỉ số** (`compare/yolo_compare.csv`, mỗi dòng = lần chạy × mức sáng × ngưỡng conf): mAP50, mAP50-95 (toàn bộ, từng lớp), precision/recall toàn bộ và từng lớp, số lần nhầm `left`↔`no_left`, `right`↔`no_right`, `red`↔`green`; ma trận nhầm lẫn và ảnh lỗi theo lần chạy.
- Mỗi biến thể ≥ 2 seed; chênh lệch nhỏ hơn dao động giữa seed coi như không khác. Chỉ số val chỉ lọc thô; chọn cuối theo FPS đo trên xe và chạy thật. Bảng tóm tắt ở cuối mỗi notebook.

## Gán nhãn sơ bộ

**YOLO:** vẽ tay ~30% mỗi lô → version Roboflow → `MODE="compare"`, `VARIANTS=["D1"]`, `SEEDS=[0]` → `MODE="prelabel"`, `PRELABEL_WEIGHTS` = `.pt` vừa train (cạnh nó có `.json` cùng tên), `PRELABEL_LIMIT=20` → kiểm trên Roboflow → đặt `None` cho phần còn lại. Dự đoán được gửi dạng chờ duyệt (`is_prediction=True`, không ghi đè nhãn có sẵn, không tự đưa ảnh vào Dataset); nhật ký `Vong3/yolo/prelabel_done.json`.

**Đường (đợt 3):** `MODE="prelabel"`, `PRELABEL_WEIGHTS` = `.pth`. Mặt nạ dự đoán đổi thành đa giác `track_surface` (bao ngoài thảm + vạch) và `object` (lỗ), gửi dạng COCO một ảnh. `glare` vẫn do người vẽ. **Chưa thử với project Roboflow thật:** chạy `PRELABEL_LIMIT=3` trước rồi mở Roboflow kiểm đa giác.

## Hợp đồng đầu vào, đầu ra và chuyển sang Jetson

| Mục | Đường | YOLO |
|---|---|---|
| Ảnh nguồn | 1280×720 BGR từ camera | 1280×720 BGR từ camera |
| Tiền xử lý (C++ phải làm giống) | Cắt hàng `crop_y`, `cv::resize` INTER_AREA về `input_hw`, BGR→RGB, /255 | Letterbox: `cv::resize` INTER_AREA giữ tỉ lệ, đệm 114 căn giữa (`letterbox_1280x720` ghi sẵn tỉ lệ, đệm), BGR→RGB, /255 |
| Đầu vào ONNX | `[1,3,H,W]` float32 NCHW | `[1,3,H,W]` float32 NCHW |
| Đầu ra | `[1,3,H,W]` logits; argmax theo kênh (0 background, 1 track_surface, 2 line) | `[1,13,N]`: `cx,cy,w,h` pixel ảnh letterbox + 9 xác suất lớp; lọc conf, NMS theo lớp ngoài mạng |
| ONNX | Opset 12, FP32, cố định, simplify | Opset 12, FP32, cố định, simplify, không NMS |
| Kiểm tra | onnxruntime vs PyTorch ≥ 20 ảnh thật: sai số xác suất < 0,001, argmax trùng ≥ 99,9% | ≥ 20 ảnh: sai số điểm lớp < 0,001, toạ độ < 0,01 px; tensor tham chiếu trong `<tên>_onnx_reference/` |

S4 có chuẩn hoá ImageNet nằm trong mạng (`imagenet_normalization_in_model: true`); bên ngoài vẫn chỉ chia 255. C++ đọc `_info.json`, không viết cứng cỡ, lớp hay vùng cắt.

Chép `.onnx` và `_info.json` lên xe, build và đo tại đó:

```bash
/usr/src/tensorrt/bin/trtexec --onnx=road_S1_s0.onnx --saveEngine=road_S1_fp16.engine --fp16 --workspace=1024
/usr/src/tensorrt/bin/trtexec --onnx=yolo_D2_s0.onnx --saveEngine=yolo_D2_fp16.engine --fp16 --workspace=1024
/usr/src/tensorrt/bin/trtexec --loadEngine=road_S1_fp16.engine --iterations=200 --avgRuns=50
/usr/src/tensorrt/bin/trtexec --loadEngine=yolo_D2_fp16.engine --iterations=200 --avgRuns=50
```

Engine phụ thuộc GPU và phiên bản TensorRT, phải build trên chính xe. Đối chiếu engine với ONNX trên cùng ảnh thật trước khi tích hợp điều khiển. Sai lệch lớn sau TensorRT thường do sai tiền xử lý (thứ tự màu, phép thu nhỏ, vùng cắt), không phải do FP16.

## Tài liệu API

- [Ultralytics: xuất mô hình](https://docs.ultralytics.com/modes/export/) — tham số xuất ONNX, kích thước và opset.
- [Roboflow: Project](https://roboflow.github.io/roboflow-python/core/project/) — tìm ảnh và lưu chú thích lên ảnh đã có.
