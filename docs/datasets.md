# Bộ dữ liệu

## Phân vùng đường: ucr_roadfinal-2

| Mục | Nội dung |
|---|---|
| Nguồn | Bản export Roboflow ucr_roadfinal-2 dùng huấn luyện TwinLiteNet+; workspace/project/version đầy đủ: TODO |
| Zip phát hành | TODO |
| Số ảnh / nhãn train, val, test | TODO |
| Chia tập | TODO |
| Lớp đầu ra mô hình | `background`, `road` (nền, mặt đường); mã mặt nạ tương ứng 0, 1 |
| Lớp và mã trong dataset gốc | TODO — chưa có zip để đối chiếu |
| Kích thước ảnh trong dataset | TODO |
| Ảnh nguồn theo JSON mô hình | Cao × rộng 720 × 1280 |
| Đầu vào mô hình | Cao × rộng 192 × 320, RGB, chuẩn hóa ImageNet, thu toàn ảnh |
| Giấy phép | TODO |
| Quyền công khai | TODO |
| Link Release (bản phát hành) | TODO |

Chưa có zip để đếm hoặc kiểm nhãn. Thông tin mô hình lấy từ `models/twinlitenetplus_192x320_road_info.json`, không thay thế thống kê dataset.

## Phát hiện đối tượng: car6_det_b4

Nguồn trong `data.yaml`: Roboflow workspace `<roboflow-workspace>`, project `car6_det-tmy0d`, version 9.
Địa chỉ nguồn được lưu trong gói: https://app.roboflow.com/<roboflow-workspace>/car6_det-tmy0d/9 (chưa kiểm truy cập công khai).
`README_det_b4.txt` mô tả bộ dữ liệu kết hợp ảnh cũ và ảnh bổ sung ngày 09/11, giữ cách chia theo lượt.

| Mục | Nội dung |
|---|---|
| Tệp | `car6_det_b4.zip` |
| Kích thước | 102.550.079 byte; nhỏ hơn 2 GB |
| Tổng ảnh / tệp nhãn thực tế | 1.488 / 1.488 |
| Kích thước ảnh đọc từ nội dung zip | Rộng × cao 1280 × 720 cho mọi ảnh |
| Định dạng nhãn | YOLO: mã lớp và hộp tâm/rộng/cao theo tỉ lệ |
| Đầu vào model D6 | Cao × rộng 192 × 320; giữ tỉ lệ, thêm đệm |
| Giấy phép phát hành | TODO — metadata nguồn ghi `Private`, chưa phải giấy phép phân phối |
| Quyền công khai | TODO |
| Link Release | TODO |

Số liệu dưới đây đếm trực tiếp bằng `zipfile`, đọc ảnh/nhãn trong bộ nhớ, không giải nén.
README export ghi 1.818 ảnh, khác nội dung zip hiện có; dùng số đếm thực tế 1.488.

### Chia theo thư mục vật lý

`data.yaml` trỏ đến `train/images`, `valid/images`, `test/images`.

| Thư mục | Ảnh | Tệp nhãn |
|---|---:|---:|
| train | 1161 | 1161 |
| valid | 252 | 252 |
| test | 75 | 75 |

### Chia theo danh sách huấn luyện

`train.txt` và `val.txt` trỏ đến ảnh nằm xuyên các thư mục vật lý; không lấy tên thư mục để suy ra tập huấn luyện cuối.

| Danh sách | Ảnh | Tệp nhãn |
|---|---:|---:|
| train.txt | 1027 | 1027 |
| val.txt | 461 | 461 |

Không có `test.txt` trong zip. Tập `test/` vật lý không được coi là tập đánh giá độc lập khi dùng hai danh sách trên.
Thứ tự lớp dataset lấy đúng `data.yaml`, khác `yolo_info.json` của D6; cần ánh xạ theo tên khi huấn luyện/đối chiếu.

### Số hộp mỗi lớp

Các cột thư mục và cột danh sách là hai cách chia cùng bộ dữ liệu, không cộng chúng với nhau.

| ID | Class | train/ | valid/ | test/ | train.txt | val.txt |
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
