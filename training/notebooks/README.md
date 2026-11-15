# Notebook huấn luyện

Hai notebook tự chứa mã, chạy trên Google Colab GPU (Python hiện đại, không chạy trên Python 3.6 của Jetson). Dữ liệu: [docs/datasets.md](../../docs/datasets.md).

| Notebook | Mô hình | Dữ liệu vào | Đầu ra |
|---|---|---|---|
| `twinlitenetplus_train.ipynb` | TwinLiteNetPlus (biến thể small) phân vùng 2 lớp `background`, `road`; vào 192×320 | zip COCO segmentation `ucr_road.zip` + `split.csv` theo nhóm thu | checkpoint, ONNX opset 12 khớp `models/twinlitenetplus_192x320_road_info.json`, `results/twinlite_metrics.json` |
| `yolo_train_car.ipynb` | YOLO 12 lớp `red`, `green`, `off`, `left`, `right`, `no_left`, `no_right`, `round_barrier`, `work_barrier`, `parking`, `people`, `obstacle`; so sánh biến thể D1–D8 (D6 = YOLOv5nu 192×320 dùng trên xe) | zip YOLO `ucr_det.zip` với `train.txt`/`val.txt` ở gốc | trọng số, ONNX opset 12 + `_info.json`, CSV so sánh |

## Hợp đồng đầu vào, đầu ra

| Mục | Phân vùng đường | YOLO |
|---|---|---|
| Ảnh nguồn | 1280×720 BGR từ camera | 1280×720 BGR từ camera |
| Tiền xử lý (C++ làm giống) | `cv::resize` INTER_LINEAR toàn ảnh về 320×192, BGR→RGB, /255, chuẩn hóa mean/std ImageNet | Letterbox giữ tỉ lệ, đệm 114 căn giữa, BGR→RGB, /255 |
| Đầu vào ONNX | `input` `[1,3,192,320]` float32 NCHW | `[1,3,H,W]` float32 NCHW |
| Đầu ra | `output` `[1,2,192,320]` logits; road khi kênh 1 > kênh 0 | `cx,cy,w,h` theo ảnh letterbox + xác suất lớp; lọc conf, NMS ngoài mạng |

C++ đọc file `_info.json` đi kèm ONNX, không viết cứng cỡ, lớp hay chuẩn hóa.

## Chuyển sang Jetson

Chép `.onnx` và `_info.json` lên xe rồi build engine TensorRT FP16 tại đó (engine phụ thuộc GPU và phiên bản TensorRT):

```bash
/usr/src/tensorrt/bin/trtexec --onnx=<model>.onnx --saveEngine=<model>.engine --fp16 --workspace=1024
/usr/src/tensorrt/bin/trtexec --loadEngine=<model>.engine --iterations=200 --avgRuns=50
```

Đối chiếu engine với ONNX trên cùng ảnh thật trước khi dùng cho điều khiển. Sai lệch lớn sau TensorRT thường do sai tiền xử lý (thứ tự màu, phép thu nhỏ, chuẩn hóa), không phải do FP16.

## Tài liệu API

- [TwinLiteNetPlus](https://github.com/chequanghuy/TwinLiteNetPlus) — mã mô hình gốc (MIT License).
- [Ultralytics: xuất mô hình](https://docs.ultralytics.com/modes/export/) — tham số xuất ONNX, kích thước và opset.
