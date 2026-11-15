# Bộ dữ liệu

Hai dataset (bộ dữ liệu) do nhóm tự thu thập và gắn nhãn trên sa bàn UIT Car Racing 2026: `ucr_road` và `ucr_det`. Nhóm chưa cấp giấy phép; metadata (siêu dữ liệu) `Private` không phải giấy phép sử dụng lại.
Tải tại [Release v1.0-final](https://github.com/lmq167/UIT_Car_Racing_2026/releases/tag/v1.0-final).

## Phân vùng đường: ucr_road

Tệp release: `ucr_road.zip`, `ucr_road_manifest.json`, `ucr_road_SHA256SUMS.txt`. Manifest (bản kê) và SHA256SUMS là bản kèm của dataset, được đặt tiền tố khi phát hành.
Zip có 898 ảnh, 921 annotation (đối tượng gắn nhãn), 3 JSON COCO segmentation (phân vùng bằng đa giác) và `split.csv`. Không có RLE hay mặt nạ raster riêng. Category ID 0 `ucr_road` là lớp cha, không có annotation; ID 1 `road` là mặt đường. Mặt nạ cho mô hình dùng nền = 0, đường = 1; không đồng nhất mã category với mã mặt nạ.

| Tập | Ảnh | Tỉ lệ ảnh | Annotation |
|---|---:|---:|---:|
| train | 635 | 70.71% | 644 |
| valid | 134 | 14.92% | 135 |
| test | 129 | 14.37% | 142 |

### Chia theo lượt thu và mức đầy đủ metadata

Cột `split` trong `split.csv` là nguồn xác định tập; không chia ngẫu nhiên lại từng ảnh. Có 108 `group_id`, mỗi nhóm chỉ thuộc một tập, không giao nhóm giữa train/valid/test. Tỉ lệ thực tế xấp xỉ 70,71% / 14,92% / 14,37% do giữ nguyên nhóm.

| Series (nhóm nguồn thu, giữ mã gốc) | train | valid | test | metadata_level (số ảnh) |
|---|---:|---:|---:|---|
| car6_1031 | 310 | 17 | 104 | frame_manifest: 157, pass_only: 219, pass_ticket: 55 |
| car6_1031_hoac_park_0411 | 194 | 78 | 0 | pass_only: 272 |
| det_0911 | 26 | 11 | 0 | pass_only: 37 |
| h_nguon_chua_xac_dinh | 7 | 0 | 0 | none: 7 |
| image_nguon_chua_xac_dinh | 8 | 0 | 0 | none: 8 |
| park_nguon_chua_xac_dinh | 69 | 26 | 11 | none: 106 |
| road_them_nguon_chua_xac_dinh | 21 | 2 | 14 | none: 37 |

`frame_manifest`: có metadata tới khung hình; `pass_ticket`: metadata từ phiếu lượt thu; `pass_only`: chỉ truy được lượt thu; `none`: thiếu metadata thu. Ô trống là chưa biết, không tự suy ra thời điểm/tốc độ/điều kiện thu. Tổng: 157 frame_manifest, 55 pass_ticket, 528 pass_only, 158 none.

Cơ sở nhóm lấy đúng từ `group_basis`: 740 ảnh có số lượt trong tên. Với tên không có số lượt, đây là **giả định**, không phải lượt thu đã xác minh: `park_` (106 ảnh) và `road_them_` (37 ảnh) gom theo khối 10 số liền nhau; `image` gom cả nhóm 8 ảnh; `h9342_` gom 7 ảnh với giả định cùng một video. Không gán quy tắc khối 10 cho hai nhóm cuối vì CSV không ghi như vậy.

| Cột split.csv | Ý nghĩa |
|---|---|
| `file` | Tên ảnh duy nhất, giữ nguyên |
| `original_name` | Tên ảnh gốc, giữ nguyên |
| `split` | Nguồn sự thật cho train/valid/test theo lượt thu |
| `split_roboflow_v5` | Cách chia Roboflow cũ để đối chiếu; không dùng chia tập mới |
| `series` | Nhóm nguồn thu |
| `group_id` | Định danh lượt thu, không giao giữa các tập |
| `group_basis` | Cơ sở xác định nhóm |
| `pass_id` | Số lượt thu nếu có |
| `frame_idx` | Số khung hình nếu có |
| `width` | Chiều rộng ảnh |
| `height` | Chiều cao ảnh |
| `metadata_level` | Mức truy xuất metadata |
| `capture_time` | Thời điểm thu nếu biết |
| `c` | Mã metadata c từ CSV nguồn |
| `t` | Mã metadata t từ CSV nguồn |
| `s` | Mã metadata s từ CSV nguồn |
| `e` | Mã metadata e từ CSV nguồn |
| `k` | Mã metadata k từ CSV nguồn |
| `v` | Mã metadata v từ CSV nguồn |
| `l` | Mã metadata l từ CSV nguồn |
| `speed_cms` | Tốc độ thu, cm/s nếu có |
| `result` | Kết quả ghi nhận |

Các mã `c`, `t`, `s`, `e`, `k`, `v`, `l` giữ nguyên từ CSV nguồn; không suy diễn thêm ý nghĩa. 747 ảnh có kích thước 1280×720 và 151 ảnh 640×384 (rộng × cao).

Model đường trong `models/` được huấn luyện trên đúng cách chia này (SHA256 `split.csv`: `0229e37c05fe34c9f6fbd8b356197936cf128e29f5e27237badca140e4e750ac`). Test 129 ảnh: IoU road 0,9700, mIoU 0,9768 (chi tiết ở [kết quả huấn luyện](results/README.md)). IoU này đo trên cách chia theo lượt thu nên không so trực tiếp được với số đo trên cách chia ngẫu nhiên theo ảnh.

## Phát hiện đối tượng: ucr_det

Tệp release: `ucr_det.zip`, `ucr_det_manifest.json`, `ucr_det_SHA256SUMS.txt`.
Zip có 1.488 ảnh 1280×720, 1.488 tệp nhãn YOLO detection (phát hiện bằng hộp giới hạn), 12 lớp. Mỗi dòng nhãn gồm `class_id cx cy width height`, tọa độ hộp được chuẩn hóa theo kích thước ảnh. Ảnh/nhãn được đặt theo `train/images`, `train/labels`, `val/images`, `val/labels`.

| Tập | Ảnh / tệp nhãn | Tỉ lệ ảnh | Số hộp |
|---|---:|---:|---:|
| train | 1027 | 69.02% | 1038 |
| val | 461 | 30.98% | 391 |

Giữ nguyên danh sách `train.txt` (1.027 dòng) và `val.txt` (461 dòng) đã dùng khi train D2/D6/D7/D8; zip có hai tệp này ở gốc, đường dẫn dạng `./<split>/images/<ảnh>` để notebook đọc. Không có tập test độc lập; không còn cách chia thư mục vật lý cũ khác danh sách. Có 91 lượt, **0 lượt bị chia đôi** giữa train/val. mAP YOLO của D2/D6/D7/D8 được tính trên đúng cách chia của `ucr_det`.

| Cột split.csv | Ý nghĩa |
|---|---|
| `file` | Đường dẫn ảnh trong zip, gồm tập và thư mục images |
| `original_name` | Tên ảnh gốc; bỏ đuôi export `_jpg.rf.<mã>`, khôi phục tiền tố xe theo notebook |
| `split` | train hoặc val theo danh sách đã dùng huấn luyện |
| `pass` | Số lượt: NNNN từ pNNNN_MMMM hoặc SSSS từ rec0411_SSSS_ID, chuyển thành số nguyên |
| `width`, `height` | Chiều rộng, chiều cao ảnh, pixel |

Không có `metadata_level` trong CSV phát hiện; chỉ công bố các trường hiện có, không suy ra metadata thu khác.

| ID | Lớp | Hộp train | Hộp val |
|---:|---|---:|---:|
| 0 | green | 71 | 36 |
| 1 | left | 96 | 25 |
| 2 | no_left | 87 | 50 |
| 3 | no_right | 99 | 46 |
| 4 | obstacle | 95 | 21 |
| 5 | off | 63 | 18 |
| 6 | parking | 19 | 13 |
| 7 | people | 88 | 28 |
| 8 | red | 92 | 29 |
| 9 | right | 111 | 78 |
| 10 | round_barrier | 33 | 9 |
| 11 | work_barrier | 184 | 38 |

Thứ tự lớp dataset khác thứ tự model D6; phải ánh xạ theo tên. `people` là hình nộm, `parking` là biển đỗ xe.

## SHA256 và kích thước

| Tệp | Kích thước (byte) | SHA256 |
|---|---:|---|
| `ucr_road.zip` | 62181749 | `9fd0d3707b779d4b778f36fc642912b242b895562e08284bf143714a75c58db1` |
| `ucr_det.zip` | 102593582 | `175d0de2508d51645c9df754903a2460616e72b0d83fe2d11037149349e9111e` |

Tải zip và `<dataset>_SHA256SUMS.txt` vào cùng thư mục. Trên Linux: `sha256sum -c ucr_road_SHA256SUMS.txt` hoặc `sha256sum -c ucr_det_SHA256SUMS.txt`. Trên PowerShell: `Get-FileHash -Algorithm SHA256 -LiteralPath <đường_dẫn_zip>` rồi đối chiếu. SHA256SUMS ghi đúng tên zip không có hậu tố phiên bản.
