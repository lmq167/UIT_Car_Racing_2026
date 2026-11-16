# Bộ dữ liệu

Hai dataset (bộ dữ liệu) do nhóm tự thu thập và gắn nhãn trên sa bàn UIT Car Racing 2026: `ucr_road` và `ucr_det`. Nhóm chưa cấp giấy phép; metadata (siêu dữ liệu) `Private` không phải giấy phép sử dụng lại.
Tải tại [Release v1.0-final](https://github.com/lmq167/UIT_Car_Racing_2026/releases/tag/v1.0-final).

## Phân vùng đường: ucr_road

Tệp release: `ucr_road.zip`, `ucr_road_manifest.json`, `ucr_road_SHA256SUMS.txt`. Manifest (bản kê) và SHA256SUMS là bản kèm của dataset, được đặt tiền tố khi phát hành.
Zip có 898 ảnh, 921 annotation (đối tượng gắn nhãn), 2 JSON COCO segmentation (phân vùng bằng đa giác) và `split.csv`. Không có RLE hay mặt nạ raster riêng. Category ID 0 `ucr_road` là lớp cha, không có annotation; ID 1 `road` là mặt đường. Mặt nạ cho mô hình dùng nền = 0, đường = 1; không đồng nhất mã category với mã mặt nạ.

| Tập | Ảnh | Tỉ lệ ảnh | Annotation |
|---|---:|---:|---:|
| train | 635 | 70.71% | 644 |
| valid | 263 | 29.29% | 277 |

### Chia theo lượt thu và mức đầy đủ metadata

Cột `split` trong `split.csv` là nguồn xác định tập; không chia ngẫu nhiên lại từng ảnh. Có 108 `group_id`, mỗi nhóm chỉ thuộc một tập, không giao nhóm giữa train và valid. Không có tập test riêng. Tỉ lệ thực tế xấp xỉ 70,71% / 29,29% do giữ nguyên nhóm.

| Nguồn thu | Ai thu / ngày / cách thu | train | valid | Mức metadata | Cách xác định nguồn |
|---|---|---:|---:|---|---|
| `car6_1031` (157 ảnh 1280×720) | Đội / 31/10/2026 / khung hình xe car6 | 94 | 63 | `frame_manifest`: 157 ảnh | bảng khung hình (manifest); 8 ảnh còn khớp ảnh với dữ liệu đội (`ucr_det`, trùng pixel) |
| `car6_1031` (55 ảnh 1280×720) | Đội / 31/10/2026 / khung hình xe car6 | 54 | 1 | `pass_ticket`: 55 ảnh | phiếu lượt 31/10/2026 |
| `car6_1031` (219 ảnh 1280×720) | Đội / 31/10/2026 (ngày của bộ thu car6 khi dựng dataset) / khung hình xe car6 | 162 | 57 | `pass_only`: 219 ảnh | số lượt lấy từ tên tệp gốc; 219 ảnh này không có phiếu lượt hay bảng khung hình; đội xác nhận |
| `car6_luot_02_12` (272 ảnh 1280×720) | Đội / không nhớ ngày thu / khung hình lượt p2–p12 của xe đội | 194 | 78 | `pass_only`: 272 ảnh | đã đối chiếu theo pixel với ảnh còn lưu của đội (lượt 1 ngày 30/10/2026, ảnh 04/11/2026 và 09/11/2026), không khớp ảnh nào; phiếu lượt 31/10/2026 không có lượt 2–12; ảnh gốc và bảng khung hình đã bị xóa ngày 15/11/2026; đội xác nhận sở hữu |
| `det_0911` (37 ảnh 1280×720) | Đội / 09/11/2026 / khung hình xe car6, lượt p91xx | 26 | 11 | `pass_only`: 37 ảnh | khớp ảnh với dữ liệu đội (`ucr_det`, 37/37 ảnh trùng pixel) |
| `bo_sung_park` (106 ảnh 640×384) | Đội / không nhớ ngày thu / khung hình từ các lượt record (ghi hình) xe đội, bổ sung sau các lượt model phân vùng nhiễu mạnh | 69 | 37 | `none`: 106 ảnh | đội xác nhận cách bổ sung và sở hữu; đối chiếu pixel với ảnh đội còn lưu: 0 khớp; ảnh gốc và bảng khung hình đã bị xóa ngày 15/11/2026 |
| `bo_sung_road_them` (37 ảnh 640×384) | Đội / không nhớ ngày thu / khung hình từ các lượt record (ghi hình) xe đội, bổ sung sau các lượt model phân vùng nhiễu mạnh | 21 | 16 | `none`: 37 ảnh | đội xác nhận cách bổ sung và sở hữu; đối chiếu pixel với ảnh đội còn lưu: 0 khớp; ảnh gốc và bảng khung hình đã bị xóa ngày 15/11/2026 |
| `bo_sung_image` (8 ảnh 640×384) | Đội / không nhớ ngày thu / khung hình từ các lượt record (ghi hình) xe đội, bổ sung sau các lượt model phân vùng nhiễu mạnh | 8 | 0 | `none`: 8 ảnh | đội xác nhận cách bổ sung và sở hữu; đối chiếu pixel với ảnh đội còn lưu: 0 khớp; ảnh gốc và bảng khung hình đã bị xóa ngày 15/11/2026 |
| `bo_sung_h9342` (7 ảnh 1280×720) | Đội / không nhớ ngày thu / khung hình từ các lượt record (ghi hình) xe đội, bổ sung sau các lượt model phân vùng nhiễu mạnh | 7 | 0 | `none`: 7 ảnh | đội xác nhận cách bổ sung và sở hữu; đối chiếu pixel với ảnh đội còn lưu: 0 khớp; ảnh gốc và bảng khung hình đã bị xóa ngày 15/11/2026 |

Toàn bộ 898 ảnh do đội thu bằng camera xe. Ngày ghi trong bảng là ngày của cả nhóm, không phải thời điểm chụp từng ảnh.

`frame_manifest` = biết thời điểm chụp từng khung hình; `pass_ticket` = có phiếu lượt (chặng, mục tiêu, biển, ánh sáng, kiểu lái, hướng, đèn, tốc độ, kết quả); `pass_only` = chỉ biết số lượt thu; `none` = không có thông tin gì ngoài tên tệp. Các thông tin đội xác nhận về sở hữu và cách bổ sung được nêu riêng trong bảng nguồn. Tổng: 157 frame_manifest, 55 pass_ticket, 528 pass_only, 158 none.

Phương pháp nhóm: 740 ảnh có số lượt trong tên được nhóm theo series và số lượt. Với 106 ảnh `park_` và 37 ảnh `road_them_`, nhóm theo 10 số thứ tự liền nhau của tên tệp để các ảnh gần nhau không bị chia ra nhiều tập. Cả 8 ảnh `image<số>` là một nhóm, cả 7 ảnh `h9342_<số>` là một nhóm. Bốn nhóm bổ sung không có số lượt thu trong tên; các nhóm này dùng để chia tập, không đại diện cho lượt thu đã được ghi nhận.

| Cột split.csv | Ý nghĩa |
|---|---|
| `file` | Tên ảnh duy nhất, giữ nguyên |
| `original_name` | Tên ảnh gốc, giữ nguyên |
| `split` | Nguồn sự thật cho train/valid theo lượt thu |
| `split_roboflow` | Cách chia Roboflow cũ để đối chiếu; không dùng chia tập mới |
| `series` | Nhóm nguồn thu |
| `group_id` | Định danh lượt thu, không giao giữa các tập |
| `group_basis` | Cơ sở xác định nhóm |
| `pass_id` | Số lượt thu nếu có |
| `frame_idx` | Số khung hình nếu có |
| `width` | Chiều rộng ảnh |
| `height` | Chiều cao ảnh |
| `metadata_level` | Mức truy xuất metadata |
| `capture_time` | Thời điểm thu nếu biết |
| `c` | Chặng |
| `t` | Mục tiêu |
| `s` | Biển |
| `e` | Ánh sáng |
| `k` | Kiểu lái |
| `v` | Hướng |
| `l` | Đèn |
| `speed_cms` | Tốc độ (cm/s) |
| `result` | Kết quả |

Các mã `c`, `t`, `s`, `e`, `k`, `v`, `l` giữ nguyên từ CSV nguồn, được giải thích theo bảng mã phiếu lượt thu của đội. Riêng bộ car6 dùng thêm `t` 10–12 và `s=5` như trong bảng. Nhãn ở mức lượt không bảo đảm biển hay đèn hiện trong mọi khung hình.

| Mã | Giá trị và ý nghĩa |
|---|---|
| `c` | 0: ngoài chặng/khác; 1–8: CP1→CP2 … CP8→CP9 |
| `t` | 0: thường; 1: công trình chặng 2; 2: ngã rẽ biển chặng 5; 3: giao lộ đèn+biển chặng 6; 4: ngã ba biển cuối chặng 6; 5: giao lộ đèn+biển chặng 7; 6: đèn xuất phát; 7: vòng xuyến; 8: người đi bộ; 9: vật cản; bổ sung car6: 10 cầu, 11 hầm, 12 bãi đỗ |
| `s` | 0: không biển; 1: rẽ trái; 2: rẽ phải; 3: cấm rẽ trái; 4: cấm rẽ phải; 5: biển khác trong bản mẫu; riêng car6 dùng 5 cho hàng rào đỏ trắng vòng xuyến |
| `e` | 1: nắng chiếu sa bàn; 2: sáng thường/râm; 3: tối (đèn trần/hầm) |
| `k` | 1: đúng tâm làn; 2: lệch trái; 3: lệch phải; 4: chếch/xiên |
| `v` | 1: thuận CP1→CP9; 2: ngược chiều |
| `l` | 0: không có/không bật; 1: đỏ suốt; 2: xanh suốt; 3: đỏ→xanh; 4: xanh→đỏ |

Nguồn: bảng mã phiếu lượt thu của đội; các giá trị bổ sung và cách dùng riêng car6 theo tài liệu bàn giao dữ liệu của đội.

747 ảnh có kích thước 1280×720 và 151 ảnh 640×384 (rộng × cao).

Cột file và split trùng bản dùng khi huấn luyện (SHA256 cặp file,split = 17fa3ddaf543a85bbee07a9ca7f64dc30ad3d0a383868dffe60758c2c30796f2).

Model đường trong `models/` được huấn luyện trên tập train của cách chia này, chọn epoch theo valid. Valid 263 ảnh: IoU road 0,9659, mIoU 0,9734 (chi tiết ở [kết quả huấn luyện](results/README.md)); valid dùng để chọn epoch nên số đo hơi lạc quan. IoU này đo trên cách chia theo lượt thu nên không so trực tiếp được với số đo trên cách chia ngẫu nhiên theo ảnh.

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
| `split` | train hay val theo danh sách đã dùng huấn luyện |
| `pass` | Số lượt: NNNN từ pNNNN_MMMM hay SSSS từ rec0411_SSSS_ID, chuyển thành số nguyên |
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
| `ucr_road.zip` | 62180936 | `c5a6bd6094f065b5c7294261c6f65685496158f17109b29527b90066b616ed63` |
| `ucr_det.zip` | 102593582 | `175d0de2508d51645c9df754903a2460616e72b0d83fe2d11037149349e9111e` |

Tải zip và `<dataset>_SHA256SUMS.txt` vào cùng thư mục. Trên Linux: `sha256sum -c ucr_road_SHA256SUMS.txt` hay `sha256sum -c ucr_det_SHA256SUMS.txt`. Trên PowerShell: `Get-FileHash -Algorithm SHA256 -LiteralPath <đường_dẫn_zip>` rồi đối chiếu. SHA256SUMS ghi đúng tên zip không có hậu tố phiên bản.
