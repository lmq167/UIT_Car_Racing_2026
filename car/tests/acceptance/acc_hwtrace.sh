#!/usr/bin/env bash
# Test ch?p nh?n main th?t (UCR_FAKE_HW=ON), quan s?t v?t ph?n c?ng gi?.
# D?ng: acc_hwtrace.sh <main> <acc_sigrun> <t?n-ca>
# Main kh?i t?o motor ??ng m?t l?n; d?ng motor r?i servo v? trim khi tho?t/l?i.
# C?u h?nh/tham s? sai b? t? ch?i tr??c khi kh?i t?o ph?n c?ng.
# terminate tr? 134; l?i lu?ng tr? kh?c 0; kh?ng c?u h?nh ngo?i chu?i init.
source "$(dirname "$0")/acc_lib.sh"
REAL_MAIN=${1:?can main}
export ACC_SIGRUN=${2:?can acc_sigrun}
CASE=${3:?can ten ca}

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

# L?i gi? trong main: g?i tr?c ti?p, gi? m? tho?t v? h?n th?i gian.
fault_case() { # <nhãn> <UCR_FAKE_FAULT> <mã-mong-đợi: số hoặc 'nonzero'>
    local l=$1 fault=$2 wantrc=$3
    newrun
    mkcfg "$RUN/c.yaml" "$RUN/runs" servo_trim_deg=7
    local t0; t0=$(now_ms)
    UCR_FAKE_HW_TRACE="$TR" UCR_FAKE_FAULT="$fault" acc_run timeout --foreground -k 2s 40 "$REAL_MAIN" --config "$RUN/c.yaml" --mode auto >"$RUN/out" 2>&1; local rc=$?
    local dt=$(( $(now_ms) - t0 ))
    if [[ $wantrc == nonzero ]]; then check "[$l] chuong trinh phai thoat khac 0 (thay $rc); dau ra: $(tail -c 200 "$RUN/out")" test "$rc" -ne 0 -a "$rc" -ne 124
    else check "[$l] chuong trinh phai thoat $wantrc (thay $rc); dau ra: $(tail -c 200 "$RUN/out")" test "$rc" -eq "$wantrc"; fi
    check "[$l] main tu ket thuc (khong treo), mat $dt ms <= 12000" test "$dt" -le 12000
    check "[$l] chuoi init van dung 1 lan trong ca phien (motor_init=$(tcount "$TR" motor_init), \$101=1: $(tcfg_has "$TR" '$101=1'))" \
        test "$(tcount "$TR" motor_init)" -eq 1 -a "$(tcfg_has "$TR" '$101=1')" -eq 1
    tail_checks "$l" "$TR" 7
}

case "$CASE" in
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
    fault_terminate) fault_case fault_terminate terminate@2500 134 ;;
    fault_actuator) fault_case fault_actuator_throw actuator_throw@2500 nonzero ;;
    fault_sensor) fault_case fault_sensor_throw sensor_throw@2500 nonzero ;;
    *) fail "Ten ca khong hop le: $CASE" ;;
esac
acc_finish "acc_hwtrace_$CASE"
