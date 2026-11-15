# Bộ dữ liệu

Dataset do nhóm tự thu thập và tự gắn nhãn trên sa bàn thi UIT Car Racing 2026. Nhóm không cấp giấy phép: chỉ xem, không dùng lại khi chưa được nhóm đồng ý.

[Release v1.0-final](https://github.com/lmq167/UIT_Car_Racing_2026/releases/tag/v1.0-final)

## Phân vùng đường: ucr_roadfinal-2

Nguồn: export từ Roboflow, định dạng COCO segmentation (phân vùng bằng đa giác).

- Tệp đính kèm: `road_UCR_RoadFinal_2.v5i.coco-segmentation_public.zip`, `road_manifest.json`, `road_SHA256SUMS.txt`.
- Zip: 62496590 byte, nhỏ hơn 2 GB; 898 ảnh, 3 JSON nhãn, 921 annotation (đối tượng gắn nhãn).
- Mask (mặt nạ): polygon COCO; toàn bộ annotation dùng đa giác, không có RLE hay mask raster riêng.
- Category (lớp) gốc: ID 0 `UCR-Road-zXwd-r2fc` là lớp cha, không có annotation; ID 1 `road` là mặt đường.
- Model xuất `background` (nền) = 0 và `road` (mặt đường) = 1. Không đồng nhất ID category COCO với mã mask model.
- Theo JSON model: ảnh nguồn cao × rộng 720 × 1280; đầu vào 192 × 320, RGB, chuẩn hóa ImageNet, thu toàn ảnh.

| Split (tập) | Ảnh | JSON nhãn | Annotation | ID 0 | ID 1 | Kích thước rộng × cao (số ảnh), từ JSON | Ảnh không annotation |
|---|---:|---:|---:|---:|---:|---|---:|
| train | 721 | 1 | 738 | 0 | 738 | 1280x720 (613), 640x384 (108) | 0 |
| valid | 165 | 1 | 171 | 0 | 171 | 1280x720 (134), 640x384 (31) | 0 |
| test | 12 | 1 | 12 | 0 | 12 | 640x384 (12) | 0 |

Số ảnh JSON đã đối chiếu với số mục ảnh thực trong từng thư mục zip.

## Phát hiện đối tượng: car6_det_b4

Nguồn: export từ Roboflow, định dạng YOLO detection (phát hiện bằng hộp giới hạn). `README_det_b4.txt` mô tả kết hợp ảnh cũ và ảnh bổ sung ngày 09/11, giữ cách chia theo lượt.

- Tệp đính kèm: `yolo_car6_det_b4_public.zip`, `yolo_manifest.json`, `yolo_SHA256SUMS.txt`.
- Zip: 102554563 byte, nhỏ hơn 2 GB; 1.488 ảnh, 1.488 tệp nhãn, 12 lớp.
- Ảnh rộng × cao 1280 × 720; nhãn gồm mã lớp, hộp tâm/rộng/cao theo tỉ lệ.
- Đầu vào model D6 cao × rộng 192 × 320; giữ tỉ lệ, thêm đệm.
- README export ghi 1.818 ảnh, khác nội dung zip; dùng số đếm thực tế 1.488. Metadata `Private` không phải giấy phép sử dụng lại.

### Chia theo thư mục vật lý

`data.yaml` trỏ tới `train/images`, `valid/images`, `test/images`.

| Thư mục | Ảnh | Tệp nhãn |
|---|---:|---:|
| train | 1161 | 1161 |
| valid | 252 | 252 |
| test | 75 | 75 |

### Chia theo danh sách huấn luyện

`train.txt` và `val.txt` trỏ xuyên các thư mục vật lý; không suy ra tập huấn luyện cuối từ tên thư mục.

| Danh sách | Ảnh | Tệp nhãn |
|---|---:|---:|
| train.txt | 1027 | 1027 |
| val.txt | 461 | 461 |

Không có `test.txt`. Không coi thư mục `test/` là tập đánh giá độc lập khi dùng hai danh sách trên. Thứ tự lớp dataset theo `data.yaml`, khác thứ tự D6; cần ánh xạ theo tên.

### Số hộp mỗi lớp

Hai cách chia cùng bộ dữ liệu, không cộng các cột với nhau.

| ID | Lớp | train/ | valid/ | test/ | train.txt | val.txt |
|---:|---|---:|---:|---:|---:|---:|
| 0 | green | 85 | 18 | 4 | 71 | 36 |
| 1 | left | 95 | 22 | 4 | 96 | 25 |
| 2 | no_left | 105 | 24 | 8 | 87 | 50 |
| 3 | no_right | 118 | 17 | 10 | 99 | 46 |
| 4 | obstacle | 87 | 26 | 3 | 95 | 21 |
| 5 | off | 68 | 12 | 1 | 63 | 18 |
| 6 | parking | 24 | 6 | 2 | 19 | 13 |
| 7 | people | 83 | 21 | 12 | 88 | 28 |
| 8 | red | 94 | 22 | 5 | 92 | 29 |
| 9 | right | 148 | 34 | 7 | 111 | 78 |
| 10 | round_barrier | 34 | 7 | 1 | 33 | 9 |
| 11 | work_barrier | 175 | 37 | 10 | 184 | 38 |

## Kiểm tra SHA256

Tải zip và SHA256SUMS cùng bộ từ Release vào một thư mục. SHA256SUMS ghi đúng tên tệp đính kèm (có tiền tố `road_`/`yolo_`), nên trên Linux chạy được `sha256sum -c road_SHA256SUMS.txt`; trên PowerShell chạy `Get-FileHash -Algorithm SHA256 -LiteralPath <đường_dẫn_zip>` rồi đối chiếu. `manifest.json` ghi tên zip trước khi thêm tiền tố.
