#!/usr/bin/env bash
# Test chấp nhận – CHƯƠNG TRÌNH THẬT (bản UCR_FAKE_HW=ON) qua dòng lệnh / run.sh, quan sát bằng vết phần cứng
# giả UCR_FAKE_HW_TRACE (định dạng "<ms>,<loại>,<giá_trị>", xem docs/ucr_car.md mục "Điểm cắm kiểm thử bản giả lập").
# Dùng: acc_hwtrace.sh <main> <ucr_stop> <acc_sigrun>
#
# Yêu cầu gốc:
#  - Dừng khẩn: "ucr_stop KHÔNG gửi lệnh cấu hình driver ($..., lưu ROM, reset), chỉ gửi tốc độ 0 + servo trim"; run.sh gọi
#    ucr_stop đúng 1 lần khi main thoát vì bất kỳ lý do gì (kể cả kill -9, terminate).
#  - Quy tắc cứng của dự án: chuỗi khởi tạo motor (Motor::Init -> SendInitCommand, có SaveAndResetDriver) chỉ chạy đúng MỘT lần lúc khởi động
#    chương trình chính; "ucr_stop tuyệt đối không gửi lệnh cấu hình".
#  - Lệnh cuối: lệnh cuối cùng gửi phần cứng là tốc độ 0, servo về trim sau thoát bình thường / lỗi / tín hiệu.
#  - docs/ucr_car.md: "ucr_stop ... đưa servo về trim ... không đảo dấu trim"; mọi lỗi cấu hình của ucr_stop
#    (không mở được file, thiếu khoá, sai kiểu, ngoài giới hạn) đều in lỗi nêu servo_trim_deg, dùng trim 0, VẪN dừng motor và
#    đưa servo về 0, rồi thoát mã khác 0; `calib` thiếu/sai bước không khởi tạo thiết bị; với main, khoá sai kiểu/vượt giới hạn
#    "dừng chương trình trước khi khởi tạo phần cứng"; terminate -> mã 134; SIGKILL -> 137;
#    lỗi luồng làm chương trình thoát khác 0 và trình bao gọi ucr_stop.
# Tách vết main/ucr_stop vì hai tiến trình có mốc ms riêng; acc_sigrun đo bằng đồng hồ quan sát chung.
source "$(dirname "$0")/acc_lib.sh"
REAL_MAIN=${1:?can main}
REAL_STOP=${2:?can ucr_stop}
export ACC_SIGRUN=${3:?can acc_sigrun}
CASE=${4:?can ten ca}

n=0
newrun() { n=$((n + 1)); RUN="$T/run$n"; mkdir -p "$RUN/runs"; export ACC_RUN="$RUN"; TR="$RUN/trace.csv"; }
tcount() { [[ -f $1 ]] || { echo 0; return; }; awk -F, -v k="$2" '$2==k{c++} END{print c+0}' "$1"; }
tlast()  { [[ -f $1 ]] || return 0; awk -F, -v k="$2" '$2==k{v=$3} END{if(v!="")print v}' "$1"; }
tlines() { [[ -f $1 ]] || { echo 0; return; }; wc -l < "$1" | tr -d ' '; }
isnum()  { [[ $1 =~ ^-?[0-9]+(\.[0-9]+)?([eE][-+]?[0-9]+)?$ ]]; }
numeq()  { isnum "$1" && awk -v a="$1" -v b="$2" 'BEGIN{d=a-b; if(d<0)d=-d; exit !(d<=0.001)}'; }
# số dòng loại $2 có chứa chuỗi $3 (cố định)
tcfg_has() { [[ -f $1 ]] || { echo 0; return; }; awk -F, '$2=="motor_cfg"' "$1" | grep -cF -- "$2" || true; }

# Kiểm tra chuỗi init của chương trình chính: đúng 1 lần.
init_checks() { # <nhãn> <trace>
    local l=$1 f=$2
    check "[$l] motor_init dung 1 dong (thay $(tcount "$f" motor_init))" test "$(tcount "$f" motor_init)" -eq 1
    check "[$l] motor_cfg dung 7 dong (chuoi init ban gia lap, thay $(tcount "$f" motor_cfg))" test "$(tcount "$f" motor_cfg)" -eq 7
    check "[$l] lenh luu ROM/reset (\$101=1) dung 1 lan (thay $(tcfg_has "$f" '$101=1'))" test "$(tcfg_has "$f" '$101=1')" -eq 1
    check "[$l] khong co lenh tu do PID (O T)" test "$(awk -F, '$2=="motor_cfg" && $3 ~ /O T$/' "$f" | wc -l | tr -d ' ')" -eq 0
}
# Lệnh cuối cùng đã gửi: tốc độ 0 + servo về trim.
tail_checks() { # <nhãn> <trace> <trim>
    local l=$1 f=$2 trim=$3
    check "[$l] lenh motor_speed cuoi cung phai la 0 (thay '$(tlast "$f" motor_speed)')" numeq "$(tlast "$f" motor_speed)" 0
    check "[$l] lenh servo_angle cuoi cung phai la trim=$trim (thay '$(tlast "$f" servo_angle)')" numeq "$(tlast "$f" servo_angle)" "$trim"
}

# =====================================================================================================================
# A. ucr_stop THẬT chạy riêng: chỉ ghi motor_speed 0 + servo_angle trim; không motor_cfg/motor_init.
stop_alone() { # <nhãn> <trim-mong-đợi-của-servo> <mã-thoát-mong-đợi:0|nonzero> <đường dẫn config hoặc rỗng> [tham số thêm]
    local l=$1 want=$2 wantrc=$3 cfg=$4
    newrun
    ( cd "$RUN" && UCR_FAKE_HW_TRACE="$TR" acc_run timeout --foreground -k 2s 20 "$REAL_STOP" --config "$cfg" >"$RUN/out" 2>&1 ); local rc=$?
    if [[ $wantrc == 0 ]]; then check "[$l] ucr_stop phai thoat 0 (thay $rc)" test "$rc" -eq 0
    else check "[$l] ucr_stop phai tra ma loi khac 0 sach (1..127) (thay $rc); dau ra: $(head -c 200 "$RUN/out")" test "$rc" -ge 1 -a "$rc" -le 127; fi
    check "[$l] ucr_stop khong duoc ghi motor_cfg (lenh cau hinh driver; thay $(tcount "$TR" motor_cfg))" test "$(tcount "$TR" motor_cfg)" -eq 0
    check "[$l] ucr_stop khong duoc ghi motor_init" test "$(tcount "$TR" motor_init)" -eq 0
    if [[ $wantrc != 0 ]]; then
        check "[$l] thong bao loi phai neu ten khoa servo_trim_deg; dau ra: $(head -c 200 "$RUN/out")" grep -q 'servo_trim_deg' "$RUN/out"
    fi
    check "[$l] ucr_stop chi ghi dung 2 dong (motor_speed + servo_angle), thay $(tlines "$TR"): $(tr '\n' ' ' < "$TR" 2>/dev/null)" test "$(tlines "$TR")" -eq 2
    check "[$l] motor_speed = 0 (thay '$(tlast "$TR" motor_speed)')" numeq "$(tlast "$TR" motor_speed)" 0
    check "[$l] servo_angle = $want (thay '$(tlast "$TR" servo_angle)')" numeq "$(tlast "$TR" servo_angle)" "$want"
}
stop_case() {
    newrun
    case "$CASE" in
        stop_trim_pos) mkcfg "$RUN/c.yaml" "$RUN/runs" servo_trim_deg=7 servo_invert=1; stop_alone "trim=7 invert=1" 7 0 "$RUN/c.yaml" ;;
        stop_trim_neg) mkcfg "$RUN/c.yaml" "$RUN/runs" servo_trim_deg=-12 servo_invert=0; stop_alone "trim=-12" -12 0 "$RUN/c.yaml" ;;
        stop_trim_neg_invert) mkcfg "$RUN/c.yaml" "$RUN/runs" servo_trim_deg=-12 servo_invert=1; stop_alone "trim=-12 invert=1" -12 0 "$RUN/c.yaml" ;;
        stop_missing) stop_alone "config khong ton tai" 0 nonzero "$T/khong-co-file.yaml" ;;
        stop_range) mkcfg "$RUN/c.yaml" "$RUN/runs" servo_trim_deg=99; stop_alone "trim=99" 0 nonzero "$RUN/c.yaml" ;;
        stop_type_*)
            local bad
            case "$CASE" in
                stop_type_text) bad=abc ;; stop_type_empty) bad='""' ;;
                stop_type_list) bad='[]' ;; stop_type_map) bad='{}' ;;
                stop_type_fraction) bad=1.5 ;;
            esac
            mkcfg "$RUN/c.yaml" "$RUN/runs" "servo_trim_deg=$bad"
            stop_alone "trim sai kieu ($bad)" 0 nonzero "$RUN/c.yaml" ;;
    esac
}

# Tham số dòng lệnh sai của ucr_stop (người gõ tay lúc khẩn cấp): KHÔNG được im lặng bỏ cuộc. Tài liệu (docs/ucr_car.md, mục
# ucr_stop): in lỗi tiếng Việt + cách dùng `ucr_stop [--config <file>]` ra stderr, dùng lại cấu hình mặc định (`../config.yaml`
# tương đối với thư mục đang đứng), VẪN dừng motor + servo, thoát mã 2. Cấu hình mặc định hợp lệ thì dùng trim trong file;
# không có/ lỗi thì trim 0.
stop_badargs() { # <nhãn> <trim-servo-mong-đợi> <có-cấu-hình-mặc-định:1|0> tham số...
    local l=$1 want=$2 hasdef=$3; shift 3
    newrun
    mkdir -p "$RUN/sub"
    if [[ $hasdef == 1 ]]; then mkcfg "$RUN/config.yaml" "$RUN/runs" servo_trim_deg=7; fi
    ( cd "$RUN/sub" && UCR_FAKE_HW_TRACE="$TR" acc_run timeout --foreground -k 2s 20 "$REAL_STOP" "$@" >"$RUN/out" 2>"$RUN/err" ); local rc=$?
    check "[$l] ucr_stop tham so sai phai thoat ma 2 (thay $rc)" test "$rc" -eq 2
    check "[$l] stderr phai co cach dung (ucr_stop, --config); thay: $(head -c 200 "$RUN/err")" grep -q -- '--config' "$RUN/err"
    check "[$l] stderr phai nhac ten chuong trinh ucr_stop" grep -q 'ucr_stop' "$RUN/err"
    check "[$l] van dung xe: vet co dung 2 dong (thay $(tlines "$TR"): $(tr '\n' ' ' < "$TR" 2>/dev/null))" test "$(tlines "$TR")" -eq 2
    check "[$l] motor_speed = 0 (thay '$(tlast "$TR" motor_speed)')" numeq "$(tlast "$TR" motor_speed)" 0
    check "[$l] servo_angle = $want (thay '$(tlast "$TR" servo_angle)')" numeq "$(tlast "$TR" servo_angle)" "$want"
    check "[$l] khong duoc ghi motor_cfg/motor_init (thay $(( $(tcount "$TR" motor_cfg) + $(tcount "$TR" motor_init) )))"         test "$(( $(tcount "$TR" motor_cfg) + $(tcount "$TR" motor_init) ))" -eq 0
}
stop_args_case() {
    newrun; mkcfg "$T/da-nhan.yaml" "$RUN/runs" servo_trim_deg=3
    case "$CASE" in
        stop_args_short_no_default) stop_badargs "ucr_stop -c x (khong co cau hinh mac dinh)" 0 0 -c x ;;
        stop_args_missing_no_default) stop_badargs "ucr_stop --config thieu gia tri (khong co cau hinh mac dinh)" 0 0 --config ;;
        stop_args_unknown_no_default) stop_badargs "ucr_stop --lanh (tham so la, khong co cau hinh mac dinh)" 0 0 --lanh ;;
        stop_args_short_default) stop_badargs "ucr_stop -c x (co cau hinh mac dinh trim 7)" 7 1 -c x ;;
        stop_args_missing_default) stop_badargs "ucr_stop --config thieu gia tri (co cau hinh mac dinh trim 7)" 7 1 --config ;;
        stop_args_unknown_default) stop_badargs "ucr_stop --lanh (co cau hinh mac dinh trim 7)" 7 1 --lanh ;;
        stop_args_option_no_default) stop_badargs "ucr_stop --config -x (gia tri trong giong tuy chon; khong co cau hinh mac dinh)" 0 0 --config -x ;;
        stop_args_option_default) stop_badargs "ucr_stop --config -x (co cau hinh mac dinh trim 7)" 7 1 --config -x ;;
        stop_args_empty_no_default) stop_badargs "ucr_stop --config chuoi rong (khong co cau hinh mac dinh)" 0 0 --config "" ;;
        stop_args_empty_default) stop_badargs "ucr_stop --config chuoi rong (co cau hinh mac dinh trim 7)" 7 1 --config "" ;;
        stop_args_trailing_default) stop_badargs "ucr_stop --config <file trim 3 dung> --lanh: bo file da nhan, dung cau hinh mac dinh trim 7" 7 1 --config "$T/da-nhan.yaml" --lanh ;;
        stop_args_trailing_no_default) stop_badargs "ucr_stop --config <file trim 3 dung> --lanh: khong co cau hinh mac dinh thi trim 0 (khong dung trim 3)" 0 0 --config "$T/da-nhan.yaml" --lanh ;;
    esac
}

# =====================================================================================================================
# B. main THẬT: init một lần; calib thiếu/sai bước và cấu hình xấu không init (hiệu chuẩn, bổ sung 26/10).
main_signal() { # <nhãn> <chế độ> <tín hiệu> [khoá=giá-trị ...]
    local l=$1 mode=$2 sig=$3; shift 3
    newrun
    mkcfg "$RUN/c.yaml" "$RUN/runs" servo_trim_deg=7 "$@"
    local kind=init
    [[ $mode == auto && $l != main_tof_timeout_1 ]] && kind=moving
    signal_timing_config "$RUN/c.yaml"
    local observed=""
    [[ $kind == moving ]] && observed="$TR"
    ACC_READY_FILE="$TR" ACC_READY_KIND="$kind" ACC_SIGNAL_TRACE="$observed" \
        sigrun "$sig" 0 "$RUN/out" env UCR_FAKE_HW_TRACE="$TR" "$REAL_MAIN" --config "$RUN/c.yaml" --mode "$mode"
    check "[$l] main phai dang chay khi da san sang" test "$ALIVE" = 1
    check "[$l] main phai tu thoat chu dong sau $sig (nhan $RES)" graceful_exit "$RES"
    check "[$l] main thoat <= 4000 ms (mat $STOP_MS)" test "$STOP_MS" -le 4000
    init_checks "$l" "$TR"
    tail_checks "$l" "$TR" 7
}

calib_invalid() {
    local step
    case "$CASE" in
        calib_missing) step='' ;; calib_zero) step=0 ;;
        calib_high) step=11 ;; calib_word) step=khong-co-buoc-nay ;;
    esac
    newrun; mkcfg "$RUN/c.yaml" "$RUN/runs"
    args=(--mode calib)
    [[ -z $step ]] || args+=(--step "$step")
    UCR_FAKE_HW_TRACE="$TR" acc_run timeout --foreground -k 2s 15s "$REAL_MAIN" --config "$RUN/c.yaml" "${args[@]}" >"$RUN/out" 2>&1; rc=$?
    check "[calib thieu/sai buoc] ma loi chu dong (nhan $rc)" test "$rc" -ge 1 -a "$rc" -lt 124
    check "[calib thieu/sai buoc] phai co thong bao" test -s "$RUN/out"
    check "[calib thieu/sai buoc] khong khoi tao thiet bi" test "$(tlines "$TR")" -eq 0
}
# QĐ10: calib nhận --step <n>. Bước hợp lệ phải có chuỗi init một lần và dừng sạch.
calib_servo() {
newrun; mkcfg "$RUN/c.yaml" "$RUN/runs" servo_trim_deg=7
# Giữ đầu vào mở như người đang chờ thao tác; stdin của CTest là EOF nên calib sẽ tự thoát.
mkfifo "$RUN/input"
exec 9<>"$RUN/input"
ACC_READY_FILE="$TR" ACC_READY_KIND=init sigrun TERM 0 "$RUN/out" env UCR_FAKE_HW_TRACE="$TR" "$REAL_MAIN" --config "$RUN/c.yaml" --mode calib --step 2 <&9
exec 9>&-
check "[calib servo] dang cho thao tac truoc TERM" test "$ALIVE" = 1
check "[calib servo] tu xu ly TERM" graceful_exit "$RES"
init_checks "calib servo" "$TR"
tail_checks "calib servo" "$TR" 7
}

reject() { # <nhãn> [khoá=giá-trị ...]   (main bị từ chối, không đụng phần cứng)
    local l=$1; shift
    newrun
    mkcfg "$RUN/c.yaml" "$RUN/runs" "$@"
    UCR_FAKE_HW_TRACE="$TR" acc_run timeout --foreground -k 2s 20 "$REAL_MAIN" --config "$RUN/c.yaml" --mode auto >"$RUN/out" 2>&1; local rc=$?
    check "[$l] phai tu choi (ma 1..127), thay $rc; dau ra: $(head -c 200 "$RUN/out")" test "$rc" -ge 1 -a "$rc" -le 127
    check "[$l] tu choi truoc khi khoi tao phan cung: vet phai rong (thay $(tlines "$TR") dong)" test "$(tlines "$TR")" -eq 0
}
# tof_timeout_ms ngoài [1, 500] bị main từ chối trước khi khởi tạo phần cứng (docs/ucr_car.md, bảng khoá): mã 1, vết rỗng,
# thông báo nêu tên khoá. Mẫu ToF cũ quá lâu vẫn được coi là hợp lệ là nguy hiểm.
reject_tof() { # <giá trị>
    local v=$1
    newrun
    mkcfg "$RUN/c.yaml" "$RUN/runs" "tof_timeout_ms=$v"
    UCR_FAKE_HW_TRACE="$TR" acc_run timeout --foreground -k 2s 20 "$REAL_MAIN" --config "$RUN/c.yaml" --mode auto >"$RUN/out" 2>&1; local rc=$?
    check "[tof_timeout_ms=$v] main phai tu choi voi ma 1 (thay $rc); dau ra: $(head -c 200 "$RUN/out")" test "$rc" -eq 1
    check "[tof_timeout_ms=$v] thong bao phai neu ten khoa tof_timeout_ms" grep -q 'tof_timeout_ms' "$RUN/out"
    check "[tof_timeout_ms=$v] tu choi truoc khi khoi tao phan cung: vet rong (thay $(tlines "$TR") dong)" test "$(tlines "$TR")" -eq 0
    check "[tof_timeout_ms=$v] khong tao thu muc chay" test -z "$(ls -A "$RUN/runs")"
}

main_badargs() {
newrun; mkcfg "$RUN/c.yaml" "$RUN/runs"
UCR_FAKE_HW_TRACE="$TR" acc_run timeout --foreground -k 2s 20 "$REAL_MAIN" --config "$RUN/c.yaml" --mode auto --khong-co-tham-so-nay >"$RUN/out" 2>&1; rc=$?
check "[tham so la] main tu choi (1..127), thay $rc" test "$rc" -ge 1 -a "$rc" -le 127
check "[tham so la] vet phai rong (thay $(tlines "$TR") dong)" test "$(tlines "$TR")" -eq 0
}

# =====================================================================================================================
# C. run.sh + main THẬT + ucr_stop THẬT (bọc để biết số dòng vết trước/sau và số lần gọi).
B="$T/build"
mkdir -p "$B"
cat > "$B/main" <<'EOF'
#!/usr/bin/env bash
echo $$ > "$ACC_RUN/mainpid"
acc_track_group "$$" || exit 1
exec "$ACC_REAL_MAIN" "$@"
EOF
cat > "$B/ucr_stop" <<'EOF'
#!/usr/bin/env bash
acc_track_group "$$" || exit 1
before=0
if [[ -f ${UCR_FAKE_HW_TRACE:-/khong/co} ]]; then before=$(wc -l < "$UCR_FAKE_HW_TRACE" | tr -d ' '); fi
echo "$before" > "$ACC_RUN/stop_before"
# ucr_stop không có lý do nhận điểm cắm lỗi/ToF/camera của main
env -u UCR_FAKE_FAULT -u UCR_FAKE_TOF_SCRIPT -u UCR_FAKE_CAMERA_GAPS UCR_FAKE_HW_TRACE="$ACC_RUN/stop_trace.csv" "$ACC_REAL_STOP" "$@"; rc=$?
echo "rc=$rc" >> "$ACC_RUN/stop_calls"
exit $rc
EOF
chmod +x "$B/main" "$B/ucr_stop"
export ACC_REAL_MAIN="$REAL_MAIN" ACC_REAL_STOP="$REAL_STOP"
calls() { if [[ -f $RUN/stop_calls ]]; then wc -l < "$RUN/stop_calls" | tr -d ' '; else echo 0; fi; }

# Các dòng vết do ucr_stop thêm vào: đúng motor_speed 0 + servo_angle trim.
stop_lines_checks() { # <nhãn> <trim>
    local l=$1 trim=$2
    local added="$RUN/stop_trace.csv"
    check "[$l] ucr_stop THAT goi dung 1 lan (thay $(calls))" test "$(calls)" -eq 1
    check "[$l] ucr_stop thoat 0" grep -qx 'rc=0' "$RUN/stop_calls"
    check "[$l] ucr_stop them dung 2 dong vao vet (thay $(tlines "$added"): $(tr '\n' ' ' < "$added"))" test "$(tlines "$added")" -eq 2
    check "[$l] ucr_stop khong ghi motor_cfg/motor_init" test "$(( $(tcount "$added" motor_cfg) + $(tcount "$added" motor_init) ))" -eq 0
    check "[$l] ucr_stop ghi motor_speed 0 (thay '$(tlast "$added" motor_speed)')" numeq "$(tlast "$added" motor_speed)" 0
    check "[$l] ucr_stop ghi servo_angle $trim (thay '$(tlast "$added" servo_angle)')" numeq "$(tlast "$added" servo_angle)" "$trim"
    tail_checks "$l" "$added" "$trim"
    check "[$l] chuoi init van dung 1 lan trong ca phien (motor_init=$(tcount "$TR" motor_init), \$101=1: $(tcfg_has "$TR" '$101=1'))" \
        test "$(tcount "$TR" motor_init)" -eq 1 -a "$(tcfg_has "$TR" '$101=1')" -eq 1
}

# kill -9 main (run.sh sống) và tín hiệu vào run.sh
scenario() { # <nhãn> <đích:runsh|killmain> <tín-hiệu> <mã-run.sh-mong-đợi: số hoặc 'graceful'>
    local l=$1 target=$2 sig=$3 wantrc=$4
    newrun
    mkcfg "$RUN/c.yaml" "$RUN/runs" servo_trim_deg=7
    signal_timing_config "$RUN/c.yaml"
    local observed="$TR" target_file=""
    if [[ $target == killmain ]]; then
        observed="$RUN/stop_trace.csv"; target_file="$RUN/mainpid"; ACC_STOP_BUDGET_MS=1000
    fi
    ACC_READY_FILE="$TR" ACC_READY_KIND=moving ACC_SIGNAL_TRACE="$observed" ACC_SIGNAL_TARGET_FILE="$target_file" \
        UCR_FAKE_HW_TRACE="$TR" UCR_BUILD_DIR="$B" \
        sigrun "$sig" 0 "$RUN/out" bash "$RUNSH" --config "$RUN/c.yaml" --mode auto
    local rc=${RES#exited:}; [[ $rc =~ ^[0-9]+$ ]] || rc=255
    check "[$l] main dang chay truoc tin hieu" test "$ALIVE" = 1
    check "[$l] run.sh ket thuc <= 10 s" test "$HUNG" = 0
    if [[ $wantrc == graceful ]]; then check "[$l] run.sh thoat 0/130/143 (thay $rc)" test "$rc" -eq 0 -o "$rc" -eq 130 -o "$rc" -eq 143
    else check "[$l] run.sh thoat $wantrc (thay $rc)" test "$rc" -eq "$wantrc"; fi
    stop_lines_checks "$l" 7
}

# Lỗi giả trong chương trình chính: chạy run.sh ở tiền cảnh, main tự chết.
fault_case() { # <nhãn> <UCR_FAKE_FAULT> <mã-mong-đợi: số hoặc 'nonzero'>
    local l=$1 fault=$2 wantrc=$3
    newrun
    mkcfg "$RUN/c.yaml" "$RUN/runs" servo_trim_deg=7
    local t0; t0=$(now_ms)
    UCR_FAKE_HW_TRACE="$TR" UCR_FAKE_FAULT="$fault" UCR_BUILD_DIR="$B" acc_run timeout --foreground -k 2s 40 bash "$RUNSH" --config "$RUN/c.yaml" --mode auto >"$RUN/out" 2>&1; local rc=$?
    local dt=$(( $(now_ms) - t0 ))
    if [[ $wantrc == nonzero ]]; then check "[$l] chuong trinh phai thoat khac 0 (thay $rc); dau ra: $(tail -c 200 "$RUN/out")" test "$rc" -ne 0 -a "$rc" -ne 124
    else check "[$l] chuong trinh phai thoat $wantrc (thay $rc); dau ra: $(tail -c 200 "$RUN/out")" test "$rc" -eq "$wantrc"; fi
    check "[$l] main tu ket thuc (khong treo), mat $dt ms <= 12000" test "$dt" -le 12000
    stop_lines_checks "$l" 7
}

wrapper_invalid() {
# main bị từ chối (cấu hình xấu) qua run.sh: ucr_stop vẫn 1 lần, ghi đúng 2 dòng (main không ghi dòng nào).
newrun
mkcfg "$RUN/c.yaml" "$RUN/runs" servo_trim_deg=7 control_hz=5
UCR_FAKE_HW_TRACE="$TR" UCR_BUILD_DIR="$B" acc_run timeout --foreground -k 2s 30 bash "$RUNSH" --config "$RUN/c.yaml" --mode auto >"$RUN/out" 2>&1; rc=$?
check "[run.sh cau hinh xau] thoat 1..127, thay $rc" test "$rc" -ge 1 -a "$rc" -le 127
check "[run.sh cau hinh xau] main khong ghi dong vet nao truoc ucr_stop (stop_before=$(cat "$RUN/stop_before" 2>/dev/null))" test "$(cat "$RUN/stop_before" 2>/dev/null || echo -1)" -eq 0
check "[run.sh cau hinh xau] ucr_stop dung 1 lan (thay $(calls))" test "$(calls)" -eq 1
TR="$RUN/stop_trace.csv"
check "[run.sh cau hinh xau] vet chi co 2 dong cua ucr_stop (thay $(tlines "$TR"))" test "$(tlines "$TR")" -eq 2
check "[run.sh cau hinh xau] khong co motor_init/motor_cfg" test "$(( $(tcount "$TR" motor_cfg) + $(tcount "$TR" motor_init) ))" -eq 0
tail_checks "run.sh cau hinh xau" "$TR" 7

}
case "$CASE" in
    stop_trim_pos|stop_trim_neg|stop_trim_neg_invert|stop_missing|stop_range|stop_type_text|stop_type_empty|stop_type_list|stop_type_map|stop_type_fraction) stop_case ;;
    stop_args_short_no_default|stop_args_missing_no_default|stop_args_unknown_no_default|stop_args_short_default|stop_args_missing_default|stop_args_unknown_default|stop_args_option_no_default|stop_args_option_default|stop_args_empty_no_default|stop_args_empty_default|stop_args_trailing_default|stop_args_trailing_no_default) stop_args_case ;;
    main_auto_TERM) main_signal main_auto_TERM auto TERM ;;
    main_auto_INT) main_signal main_auto_INT auto INT ;;
    main_manual_TERM) main_signal main_manual_TERM manual TERM ;;
    main_odometry) main_signal main_auto_odometry_nhanh_TERM auto TERM odometry_hz=100 sensor_hz=100 ;;
    main_tof_min) main_signal main_tof_timeout_1 auto TERM tof_timeout_ms=1 ;;
    main_tof_max) main_signal main_tof_timeout_500 auto TERM tof_timeout_ms=500 ;;
    calib_missing|calib_zero|calib_high|calib_word) calib_invalid ;;
    calib_servo) calib_servo ;;
    reject_control_range) reject "main control_hz=5" control_hz=5 ;;
    reject_control_type) reject "main control_hz sai kieu (abc)" control_hz=abc ;;
    reject_speed_type) reject "main max_speed_cms sai kieu (abc)" max_speed_cms=abc ;;
    reject_trim_type) reject "main servo_trim_deg sai kieu (abc)" servo_trim_deg=abc ;;
    reject_tof_zero) reject_tof 0 ;;
    reject_tof_high) reject_tof 501 ;;
    reject_tof_large) reject_tof 100000 ;;
    reject_tof_neg) reject_tof -1 ;;
    main_badargs) main_badargs ;;
    run_TERM) scenario run_sh_TERM runsh TERM graceful ;;
    run_INT) scenario run_sh_INT runsh INT graceful ;;
    kill_main) scenario main_kill9 killmain KILL 137 ;;
    fault_terminate) fault_case fault_terminate terminate@2500 134 ;;
    fault_actuator) fault_case fault_actuator_throw actuator_throw@2500 nonzero ;;
    fault_sensor) fault_case fault_sensor_throw sensor_throw@2500 nonzero ;;
    wrapper_invalid) wrapper_invalid ;;
    *) fail "Ten ca khong hop le: $CASE" ;;
esac
acc_finish "acc_hwtrace_$CASE"
