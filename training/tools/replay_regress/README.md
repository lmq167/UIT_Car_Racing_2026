# Replay hồi quy Planner trên Windows

Chạy từ gốc kho mã, Python 3 trên laptop, MSYS2 UCRT64 tại `C:/msys64/ucrt64/bin` có g++, pkg-config và OpenCV. Không cần checkout (đổi cây mã), stash (cất thay đổi), SSH hay quyền điều khiển xe.

```powershell
python round3_final/tools/replay_regress/replay_regress.py build 9846f74 round3_final/test_logs/V1p_1213/old.exe
python round3_final/tools/replay_regress/replay_regress.py build 9846f74 round3_final/test_logs/V1p_1213/new.exe --patch round3_final/tasks/V1a_planner_3fix.patch
python round3_final/tools/replay_regress/replay_regress.py build WORKTREE round3_final/test_logs/V1p_1213/worktree.exe
python round3_final/tools/replay_regress/replay_regress.py compare round3_final/test_logs/V1p_1213/old.exe round3_final/test_logs/V1p_1213/new.exe data/auto/runs_auto/runs_1212_l27/x/1794484813_16925
```

`build` dùng `git archive`, giải nén bằng pathlib vào `round3_final/test_logs/V1p_1213/build_*`. WORKTREE chụp các tệp đang theo dõi và tệp mới không bị Git bỏ qua; không lấy thư mục build. Không thay đổi tệp gốc. Không bảo đảm ảnh chụp nguyên tử nếu phiên khác đang ghi cùng lúc. Nên dùng rev đã chốt hoặc rev + patch để so chính thức. Patch chỉ được chạm cây `round3_final/ucr_car`. Giữ bản nguồn tạm để tra cứu; không tự xóa. Mỗi lần g++ giới hạn 300 giây. Có `.build.log` và `.json` cạnh exe.

Để đo lộ trình, bộ đọc replay **trong bản tạm** được bổ sung yaw/odo và xuất route_step/turn_dir/traj_mode/route_note/source_row. Thuật toán Planner/LaneController lấy đúng bản nguồn đã chọn. Không tạo ảnh phủ trong replay để giảm dung lượng. G được giữ chờ dữ liệu hợp lệ, bỏ sau khi Planner nhận; Reset(true) tại G. Các sự kiện R/epoch không đủ thông tin để phục dựng chính xác.

`compare` chỉ nhận phiên t94 qua SHA trong session.json, dùng đúng control.yaml từng phiên, giới hạn 900 giây mỗi exe/phiên. Kết quả dưới `compare_*`: `summary.json`, `segments.csv`, `<phiên>_diff.csv`, hai CSV replay và log. Bảng đoạn nhóm các bản ghi khác liên tiếp có cùng thay đổi trạng thái/lý do/chặng/ý định/chế độ; góc và tốc độ trong bảng là giá trị đầu đoạn, đầy đủ từng dòng nằm ở CSV chi tiết. Thời gian trong bảng công cụ là t_ms tuyệt đối; báo cáo V1p quy về đầu phiên. Dung sai lệnh 1e-6; so khóa source_row/frame_id/t_ms, không gộp các dòng dùng chung frame. Summary tách bản ghi khác và frame_id khác. Không có dòng khác thì không tạo CSV diff rỗng.

**Giới hạn bắt buộc:** log không lưu mọi mặt nạ, tuổi mẫu cảm biến, cờ sensor_ok/encoder_ok, ToF hay toàn bộ epoch. Bộ đọc tìm mặt nạ cùng mask_ms đã được lưu; thiếu thì đưa mặt nạ rỗng, không đoán bằng ảnh tương lai/giữ mặt nạ cũ dưới timestamp mới. Yaw/odo hữu hạn được coi là tươi ở t_ms. Vì vậy đây là so sánh hai chương trình trên cùng đầu vào quan sát được, không phải tái hiện đầy đủ lượt xe. Đọc mask_rows_available/mask_rows_missing và log cảnh báo trước khi nhận kết quả. **0 khác biệt không phải bằng chứng không có hồi quy**; sai khác cũng phải đối chiếu log gốc và ảnh. Thay lệnh trong phát lại không làm thay đổi yaw/odo/ảnh đã ghi: không xác nhận được xe sẽ đi theo quỹ đạo mới.

Các công cụ hỗ trợ chỉ đọc log:

- `analyze_sessions.py`: khử trùng phiên l17–l27, xuất danh mục, G/Space, đoạn reason/state/route/intent, số đo VX1. Dòng cuối CSV bị cắt được ghi số lượng trong catalog; không sửa log.
- `mask_probe.py`: thử thay chọn dải trong ROUNDABOUT ở ngoại tuyến; giữ dựng biên/tâm làn/độ tin cậy/khớp đường. Cột midpoint là biến thể lấy trung điểm như ucr, chưa phải toàn bộ thuật toán ucr. Điểm nhìn đo từng mặt nạ, chưa áp kẹp dịch chuyển qua thời gian hay vòng điều khiển xe.
- `verify_mask_port.py`: biên dịch harness (chương trình kiểm nhỏ) tại thư mục tạm, kiểm điểm nhìn Python với LaneController C++ gốc.
- `evidence_sheets.py`: ghép ảnh gốc, xuất bảng mốc yêu cầu/mốc ảnh thật; không coi ảnh gần nhất là ảnh đúng thời điểm.
- `test_replay_regress.py`: kiểm khác biệt ý định/lệnh và mặt nạ rỗng.

```powershell
python round3_final/tools/replay_regress/analyze_sessions.py
python round3_final/tools/replay_regress/mask_probe.py
python round3_final/tools/replay_regress/verify_mask_port.py
python round3_final/tools/replay_regress/evidence_sheets.py
python round3_final/tools/replay_regress/test_replay_regress.py
```
