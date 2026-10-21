# Huấn luyện mô hình cho camera xe thật

Hai notebook tự chứa mã, dùng trên Google Colab với Python hiện đại. Không chạy các thư viện huấn luyện này trên Python 3.6 của Jetson.

| Notebook | Chức năng | Đầu vào mặc định khi huấn luyện thật |
|---|---|---|
| `road_train_car.ipynb` | Phân vùng mặt đường hoặc vạch làn, một hay nhiều lớp tiền cảnh | RGB, 96×160 (cao×rộng) |
| `yolo_train_car.ipynb` | Phát hiện biển, vật cản, đèn, người theo lớp của dữ liệu | RGB, 320×320 |

## Chạy thử bằng dữ liệu giả

1. Tải một notebook lên Colab, chọn môi trường CPU (bộ xử lý trung tâm).
2. Giữ `FAKE_DATA=True`, chạy lần lượt toàn bộ ô và cấp quyền kết nối Google Drive. Ô cài đặt dùng pip của đúng Python đang chạy; nên dùng phiên Colab mới. Nếu pip yêu cầu khởi động lại phiên do thay thư viện, khởi động lại rồi chạy từ đầu.
3. Notebook đường chạy hai bước; YOLO chạy một epoch (lượt đi hết dữ liệu) ở 64×64, khởi tạo ngẫu nhiên. Chế độ này không cần khoá Roboflow hay tải trọng số COCO (bộ dữ liệu tiền huấn luyện).
4. Kiểm tra thư mục `MyDrive/UIT_Car_Racing/Vong3/<road hoặc yolo>/fake/<thời gian>/`: phải có trọng số, ONNX, `_info.json`, chỉ số CSV và ảnh xem trước tăng cường dữ liệu. YOLO còn thử tạo nhãn sơ bộ cục bộ, không kết nối Roboflow.
5. Cả hai notebook phải vượt kiểm tra đối chiếu ONNX trên ít nhất 20 ảnh. Không dùng trọng số hoặc chỉ số giả để điều khiển xe.

Máy ngoài Colab có thể chạy bằng Jupyter. Đặt `LOCAL_ROOT` đến thư mục `MyDrive/UIT_Car_Racing/Vong3` đã đồng bộ trên máy, `INSTALL_DEPS=False` nếu đã cài các thư viện trong ô cài đặt. Không cần chép thêm module mã nguồn.

```powershell
python -m jupyter nbconvert --to notebook --execute road_train_car.ipynb --output road_cpu_executed.ipynb --ExecutePreprocessor.timeout=1800
python -m jupyter nbconvert --to notebook --execute yolo_train_car.ipynb --output yolo_cpu_executed.ipynb --ExecutePreprocessor.timeout=1800
```

Chạy lệnh trong thư mục chứa notebook. Chỉ dùng các lệnh này với cấu hình dữ liệu giả để kiểm tra nhanh; cài Jupyter riêng trong môi trường Python của máy. Kết quả huấn luyện vẫn nằm trong `LOCAL_ROOT`.

## Huấn luyện dữ liệu thật

Chọn GPU (bộ xử lý đồ hoạ) trong Colab. Đặt `FAKE_DATA=False`, điền `PROJECT`, `VERSION` và `WORKSPACE` nếu cần. Trong mục Secrets (kho bí mật) của Colab, thêm `ROBOFLOW_API_KEY` và cho notebook quyền đọc. Nếu không có, notebook hỏi bằng `getpass` (nhập ẩn). Không đưa khoá vào ô mã hay lưu đầu ra có thông tin xác thực.

Tạo phiên bản dữ liệu không tăng cường và không cắt ảnh. Giữ mẫu tên ảnh thu `phiên_p<pass>_l<lượt>_t<mục tiêu>_<khung>.jpg`, ví dụ `s1027_p003_l2_t05_0123.jpg`. Hậu tố `_jpg.rf.<mã>.jpg` do Roboflow thêm được loại bỏ khi phân tích tên. Tên lỗi và tên gốc trùng sẽ làm notebook dừng để sửa dữ liệu, không tự chia ngẫu nhiên từng ảnh.

Các mục tiêu khác vị trí phải có mã khác nhau; cùng vị trí ở các phiên/lượt phải dùng cùng mã mục tiêu. Notebook giữ cả mục tiêu ở một tập, bất kể phiên, lượt hay pass (lần đi qua). Với `t00` chưa xác định vị trí, giữ toàn bộ lượt; đây chỉ là phương án dự phòng. Nếu các lượt đi qua cùng một đoạn đường, cách này chưa loại được rò rỉ không gian: cần chia mã vị trí trước khi đánh giá chính thức. Không đưa phiên vào khoá nhóm vì sẽ khiến cùng vị trí ở hai phiên lọt sang cả hai tập.

Dùng `VAL_GROUPS` để giữ vị trí chủ động, ví dụ `["target_3", "target_7"]` hoặc `["road_l4"]`; để rỗng sẽ chọn nhóm bằng seed (giá trị khởi tạo ngẫu nhiên). Cần ít nhất hai nhóm, và kiểm tra `split.csv`. Xem thống kê lớp của YOLO để bổ sung lớp bị thiếu mẫu ở tập huấn luyện hoặc đánh giá. Không dùng tập đánh giá để huấn luyện lại trước khi chốt các chỉ số báo cáo.

Tất cả dữ liệu tải về, ảnh biến thể, nhật ký, trọng số và kết quả nằm trong `Vong3`; mỗi lần chạy tạo thư mục mới nên không ghi đè lần trước. Tăng cường dữ liệu gồm sáng ±25%, tương phản ±20%, vùng hầm tối ám màu, nhoè chuyển động, nhiễu cảm biến, lệch cân bằng trắng và vệt chói. Với YOLO, tạo `AUG_COPIES` biến thể cho từng ảnh huấn luyện sau khi chia tập, giữ nhãn khung nguyên vẹn vì các phép biến đổi chỉ thay đổi màu/cường độ. Tắt lật ngang và các phép hình học của Ultralytics. Notebook đường được lật ngang cả ảnh và mặt nạ đồng thời.

## Phân vùng đường

Roboflow cần xuất `png-mask-semantic`, gồm ảnh, mặt nạ `<tên ảnh>_mask.png` và `_classes.csv`. Mặt nạ PNG phải chứa chỉ số lớp (kể cả PNG bảng màu), không phải RGB ba kênh. Notebook lấy giá trị điểm ảnh theo tên lớp trong CSV; thiếu lớp hoặc bảng lớp mâu thuẫn sẽ dừng.

Đặt `CLASS_NAMES=["road"]` cho mặt đường hoặc `["lane"]` cho vạch; `NUM_CLASSES` phải bằng số tên lớp. Nhiều lớp tạo nhiều kênh nhị phân, mỗi kênh sigmoid (hàm đổi điểm thành xác suất) riêng; nền là phần không thuộc lớp được chọn. Với nhiều lớp vạch, số đo mép được báo riêng từng lớp.

`ARCH="unet_b_minus"` dùng hai mức 8/16/32, mặc định 96×160; `ARCH="nanounet"` dùng ba mức 16/32/64/128. Có thể chọn 128×224. Cả hai là biến thể U-Net (mạng mã hoá/giải mã có nối tắt) tự chứa để huấn luyện mới. Mã `lane_models.py` được notebook cũ tham chiếu không có trong kho hiện tại, nên các định nghĩa này chưa được đối chiếu với kiến trúc cũ và không bảo đảm nạp được trọng số cũ.

Đếm GFLOPs (tỉ phép toán) trước huấn luyện, với quy ước một phép nhân cộng bằng hai phép toán. Cấu hình mặc định có kiểm tra ước lượng không vượt 20 ms, lấy 6 ms/GFLOP. Đây là ước lượng phần suy luận, chưa tính chụp ảnh, tiền xử lý, hậu xử lý hay tranh chấp GPU. Cần đo thật trên Nano.

`road_metrics.csv` chứa IoU (tỉ lệ giao trên hợp) từng lớp ở độ sáng 0,6/0,8/1,0/1,2/1,4; sai số mép trái/phải trên ba hàng `SCAN_RATIOS=[0.60,0.70,0.80]`. Hàng quét tính theo tỉ lệ chiều cao; sai số mép chia chiều rộng. Báo số hàng ghép được, mất mép và dự đoán thừa. Ô trống nghĩa là không có mẫu hợp lệ, không phải sai số bằng 0. Mép là điểm ngoài cùng của lớp trên hàng; với nhiều vạch rời nhau đây không phải phép ghép từng vạch.

`road.pth` chứa `state_dict` (bảng trọng số), tên kiến trúc, kích thước và tên lớp. Nạp bằng định nghĩa kiến trúc trong notebook. Mô hình giữ trọng số có IoU đánh giá tốt nhất.

## YOLO

`MODEL="yolov8n"` mặc định; có thể đổi `"yolo11n"`. Cài Ultralytics 8.4.161 để đồng nhất API (giao diện lập trình) với quy trình đã có. YOLO11 có khối attention (cơ chế chú ý), cần kiểm tra build với TensorRT 8.2; nếu không tương thích thì dùng YOLOv8n.

Tên và số lớp lấy trực tiếp từ `data.yaml`, không có danh sách lớp thi viết cứng. Nhãn `.txt` phải gồm năm số theo định dạng phát hiện YOLO; ảnh nền có file rỗng. Kiểm tra nhãn đa giác ở Roboflow trước khi xuất: notebook này yêu cầu khung chữ nhật, không tự coi đa giác là khung.

`yolo.pt` là trọng số tốt nhất. `yolo_metrics.csv` chứa precision (tỉ lệ dự đoán đúng), recall (tỉ lệ phát hiện đủ) và số đúng/sai/bỏ sót theo lớp, độ sáng và ngưỡng conf (độ tin cậy). `confusion_matrix.csv/.png` có hàng nhãn thật, cột dự đoán, ô nền cuối. `error_images.csv` liệt kê phiên/lượt/mục tiêu/pass của ảnh lỗi tại `EVAL_CONF` cho năm mức sáng. `augmentation_manifest.csv` ghi nguồn của từng ảnh biến thể để đối chiếu việc chia tập.

### Gán nhãn sơ bộ

Đặt `MODE="prelabel"`, `FAKE_DATA=False`, `PRELABEL_WEIGHTS` đến trọng số đã học trong `Vong3`; chọn đúng phiên bản dataset (bộ dữ liệu) để kiểm tra thứ tự lớp. Chạy các ô; ô huấn luyện chuyển sang nạp trọng số có sẵn. Vẫn có thể đánh giá và xuất ONNX từ trọng số này.

Phần gán sơ bộ tìm các ảnh Roboflow có `in_dataset=False`, tải ảnh theo mã, dự đoán rồi gọi `save_annotation` với `is_prediction=True`, `annotation_overwrite=False`, `add_to_dataset=False`. Không tạo lại ảnh hay tự duyệt vào Dataset. Mở Roboflow để kiểm tra khung và lớp. Mặc định giới hạn 20 ảnh mỗi lần; đặt `PRELABEL_LIMIT=None` khi đã kiểm tra chất lượng. Mã ảnh gửi thành công được lưu trong `yolo/prelabel_done.json`; ảnh không thấy vật chỉ ghi nhật ký và được phép xét lại lần sau. Có thể chạy lại ô gán sơ bộ trong cùng phiên: thư mục nhãn được dùng lại, nhật ký mã ảnh đã gửi được đọc để bỏ qua các ảnh đó và giữ các mã cũ khi bổ sung mã mới. Nếu máy chủ báo lỗi, notebook dừng; xem nhật ký đã lưu đến ảnh trước đó rồi chạy lại. Khi đổi trọng số và muốn gán lại các ảnh đã gửi, xem lại nhật ký và trạng thái nhãn trên Roboflow trước khi tự chỉnh danh sách này.

## Hợp đồng đầu vào, đầu ra và chuyển sang Jetson

| Mục | Đường | YOLO |
|---|---|---|
| Đầu vào ONNX | `[1,3,H,W]`, RGB float32 /255 | `[1,3,S,S]`, RGB float32 /255 |
| Đổi kích thước | Kéo về đúng W×H, nội suy song tuyến | Letterbox: giữ tỉ lệ, đệm 114, căn giữa, không tự giảm kích thước đệm |
| Tiền xử lý trong mạng | Không | Không |
| Đầu ra | `[1,nc,H,W]` logits (điểm trước sigmoid) | `[1,4+nc,N]`, bốn số `cx,cy,w,h` theo pixel đầu vào và xác suất từng lớp |
| Hậu xử lý | Sigmoid từng lớp và ngưỡng | Lọc conf, đổi khung về ảnh gốc, NMS (loại khung trùng) theo từng lớp |
| ONNX | Opset 12, FP32, batch (số ảnh mỗi lô) = 1 cố định | Opset 12, FP32, batch = 1 cố định, không NMS trong mạng |

Mỗi mô hình có `_info.json` ghi thứ tự lớp, tên/kích thước tensor (mảng nhiều chiều), màu, chuẩn hoá, kiểu đầu ra, số tham số, GFLOPs, ước lượng thời gian và phiên bản thư viện. Trường `fake_data` phân biệt kết quả giả. Đọc metadata (thông tin mô tả) này khi tích hợp; không dùng tiền xử lý BGR 0–255 của mô hình cũ cho các file mới.

Kiểm tra ONNX Runtime (bộ thực thi ONNX) và PyTorch trên cùng ít nhất 20 ảnh: sai số xác suất tối đa < 0,001; YOLO còn yêu cầu sai số toạ độ < 0,01 pixel. `onnx_check.csv` ghi từng ảnh. YOLO lưu cả đầu vào và đầu ra PyTorch ở `onnx_reference/` để đối chiếu engine (bản thực thi tối ưu). Nếu ít hơn 20 ảnh thật thì xuất sẽ dừng ở bước kiểm tra, cần bổ sung dữ liệu.

Chép `.onnx` và `_info.json` vào thư mục mô hình của đội trên Jetson, chạy tại đó:

```bash
/usr/src/tensorrt/bin/trtexec --onnx=road.onnx --saveEngine=road_fp16.engine --fp16 --workspace=1024
/usr/src/tensorrt/bin/trtexec --onnx=yolo.onnx --saveEngine=yolo_fp16.engine --fp16 --workspace=1024
/usr/src/tensorrt/bin/trtexec --loadEngine=road_fp16.engine --iterations=200 --avgRuns=50
/usr/src/tensorrt/bin/trtexec --loadEngine=yolo_fp16.engine --iterations=200 --avgRuns=50
```

FP16 là số thực 16 bit; ONNX vẫn xuất FP32, để TensorRT chọn FP16 khi build (biên dịch tối ưu). Engine phụ thuộc GPU và phiên bản TensorRT, phải build trên chính xe. Đối chiếu engine với ONNX trên cùng ảnh thật trước khi tích hợp điều khiển. Giữ `.onnx` để build lại khi môi trường thay đổi.

## Tài liệu API

- [Ultralytics: xuất mô hình](https://docs.ultralytics.com/modes/export/) — tham số xuất ONNX, kích thước và opset.
- [Roboflow: Project](https://roboflow.github.io/roboflow-python/core/project/) — tìm ảnh và lưu chú thích lên ảnh đã có.
