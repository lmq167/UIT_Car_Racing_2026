#!/usr/bin/env bash
# Test chấp nhận – cấu hình xấu, tham số dòng lệnh lạ, SIGINT/SIGTERM trực tiếp và
# (mất ảnh đầu-cuối bằng chương trình thật). Chạy ở UCR_FAKE_HW=ON.
# Dùng: acc_cli.sh <main> <ucr_stop> <acc_make_frames> <acc_sigrun>
#
# Yêu cầu gốc:
#  - Tham số lạ: "Tham số dòng lệnh lạ -> thông báo 'tham số không biết', mã thoát khác 0."
#  - Thông báo tham số: tham số lạ ở cuối dòng lệnh phải báo "tham số không biết: <tên>" (nêu đúng tên), KHÔNG báo nhầm
#    "thiếu giá trị"; chỉ báo "thiếu giá trị" khi tham số ĐÃ BIẾT thiếu giá trị.
#  - Thiếu khoá cấu hình: "Thiếu khoá -> dùng mặc định và in cảnh báo".
#  - Cấu hình xấu: "khoá vượt giới hạn an toàn (ví dụ control_hz quá thấp) -> từ chối chạy, mã thoát khác 0".
#  - docs/ucr_car.md (bảng khoá, mục Cấu hình): control_hz/sensor_hz >= 10; image_timeout_ms 1..500 (quy tắc an toàn: mất ảnh
#    quá 0,5 s phải dừng); max_speed_cms > 0; |trim| + max_steering_deg <= 60; motor_keepalive_ms 1..200; jpeg_quality
#    1..100; image_every_n >= 1; camera_hz/odometry_hz/ui_hz >= 1; tof_names là hoán vị của front/rear/left/right;
#    "file không mở được hoặc giá trị sai dừng chương trình trước khi khởi tạo phần cứng".
#  - Quy tắc an toàn (dừng khi thoát): thoát bằng Ctrl+C/SIGTERM phải dừng sạch.
source "$(dirname "$0")/acc_lib.sh"
MAIN=${1:?can duong dan main}
STOP=${2:?can duong dan ucr_stop}
FRAMES=${3:-}
export ACC_SIGRUN=${4:?can acc_sigrun}

# Đạt khi mã thoát là "từ chối sạch": 1..127 (không phải 0, không bị tín hiệu giết, không hết giờ).
clean_nonzero() { [[ $1 -ge 1 && $1 -le 127 && $1 -ne 124 ]]; }
not_match() { ! grep -qiE "$1" <<<"$2"; }
does_match() { grep -qiE "$1" <<<"$2"; }

UNKNOWN_RE='tham s(o|ố) kh(o|ô)ng bi(e|ế)t'
MISSING_RE='thi(e|ế)u gi(a|á) tr(i|ị)'

# ---------------------------------------------------------------------------------------------------------------
# Ca 9: tham số lạ
mkdir -p "$T/u/runs"
mkcfg "$T/u/ok.yaml" "$T/u/runs"
unknown_case() {   # unknown_case "mô tả" tên-tham-số-lạ args...
    local desc=$1 name=$2; shift 2
    local out rc
    out=$(cd "$T/u" && timeout 10 "$MAIN" "$@" 2>&1); rc=$?
    check "$desc: ma thoat phai khac 0 va sach (1..127), nhan $rc" clean_nonzero "$rc"
    check "$desc: phai bao 'tham so khong biet' (yeu cau thong bao tham so); nhan: $out" does_match "$UNKNOWN_RE" "$out"
    check "$desc: phai neu dung ten tham so '$name'; nhan: $out" grep -qF -- "$name" <<<"$out"
    check "$desc: KHONG duoc bao nham 'thieu gia tri'; nhan: $out" not_match "$MISSING_RE" "$out"
    check "$desc: khong duoc tao thu muc chay (chua khoi tao gi)" test -z "$(ls -A "$T/u/runs")"
}
unknown_case "tham so la dung mot minh" "--bogus-flag" --bogus-flag
unknown_case "tham so la o cuoi sau mode+config" "--bogus-flag" --config "$T/u/ok.yaml" --mode auto --bogus-flag
unknown_case "tham so la o dau" "--xyz" --xyz --config "$T/u/ok.yaml" --mode auto
unknown_case "tham so la co gia tri o cuoi" "--speed-boost" --config "$T/u/ok.yaml" --mode auto --speed-boost 9
unknown_case "tham so la dang -x" "-x" --config "$T/u/ok.yaml" -x

missing_case() {   # tham số ĐÃ BIẾT thiếu giá trị
    local desc=$1; shift
    local out rc
    out=$(cd "$T/u" && timeout 10 "$MAIN" "$@" 2>&1); rc=$?
    check "$desc: ma thoat phai khac 0 va sach, nhan $rc" clean_nonzero "$rc"
    check "$desc: phai bao thieu gia tri; nhan: $out" does_match "$MISSING_RE" "$out"
    check "$desc: KHONG duoc goi la 'tham so khong biet'; nhan: $out" not_match "$UNKNOWN_RE" "$out"
}
missing_case "--mode thieu gia tri" --config "$T/u/ok.yaml" --mode
missing_case "--config thieu gia tri" --mode auto --config

out=$(cd "$T/u" && timeout 10 "$MAIN" --config "$T/u/ok.yaml" --mode fly 2>&1); rc=$?
check "che do khong hop le '--mode fly': ma thoat phai khac 0 va sach, nhan $rc" clean_nonzero "$rc"
check "che do khong hop le: phai co thong bao (khong im lang)" test -n "$out"
out=$(cd "$T/u" && timeout 10 "$MAIN" --config "$T/u/khong_ton_tai.yaml" --mode auto 2>&1); rc=$?
check "file config khong ton tai: ma thoat phai khac 0 va sach, nhan $rc" clean_nonzero "$rc"
check "file config khong ton tai: phai co thong bao" test -n "$out"
printf 'day khong phai yaml {{{ [[[ \n::::\n' > "$T/u/hong.yaml"
out=$(cd "$T/u" && timeout 10 "$MAIN" --config "$T/u/hong.yaml" --mode auto 2>&1); rc=$?
check "file config hong: ma thoat phai khac 0 va sach, nhan $rc" clean_nonzero "$rc"
check "khong tao thu muc chay khi config loi" test -z "$(ls -A "$T/u/runs")"

# ---------------------------------------------------------------------------------------------------------------
# Ca 8: khoá vượt giới hạn an toàn => từ chối chạy
reject_case() {   # reject_case "tên" khoá=giá-trị...
    local name=$1; shift
    local d="$T/r_$RANDOM$RANDOM"; mkdir -p "$d/runs"
    mkcfg "$d/c.yaml" "$d/runs" "$@"
    local out rc
    out=$(cd "$d" && timeout 10 "$MAIN" --config "$d/c.yaml" --mode auto 2>&1); rc=$?
    check "tu choi [$name] ($*): ma thoat phai khac 0 va sach (1..127), nhan $rc" clean_nonzero "$rc"
    check "tu choi [$name]: phai co thong bao ly do" test -n "$out"
    check "tu choi [$name]: khong duoc tao thu muc chay (chua khoi tao thiet bi)" test -z "$(ls -A "$d/runs")"
}
reject_case "control_hz thap 9"        control_hz=9
reject_case "control_hz thap 5"        control_hz=5
reject_case "control_hz thap 1"        control_hz=1
reject_case "control_hz 0"             control_hz=0
reject_case "control_hz am"            control_hz=-3
reject_case "control_hz sai kieu"      control_hz=abc
reject_case "sensor_hz thap 9"         sensor_hz=9
reject_case "sensor_hz thap 1"         sensor_hz=1
reject_case "image_timeout 501"        image_timeout_ms=501
reject_case "image_timeout 5000"       image_timeout_ms=5000
reject_case "image_timeout 0"          image_timeout_ms=0
reject_case "image_timeout am"         image_timeout_ms=-5
reject_case "max_speed 0"              max_speed_cms=0
reject_case "max_speed am"             max_speed_cms=-3.0
reject_case "max_speed sai kieu"       max_speed_cms=abc
reject_case "max_steering 70"          max_steering_deg=70
reject_case "trim 41 + max 20"         servo_trim_deg=41 max_steering_deg=20
reject_case "trim am -45 + max 20"     servo_trim_deg=-45 max_steering_deg=20
reject_case "keepalive 201"            motor_keepalive_ms=201
reject_case "keepalive 0"              motor_keepalive_ms=0
reject_case "jpeg_quality 0"           jpeg_quality=0
reject_case "jpeg_quality 101"         jpeg_quality=101
reject_case "image_every_n 0"          image_every_n=0
reject_case "camera_hz 0"              camera_hz=0
reject_case "odometry_hz 0"            odometry_hz=0
reject_case "ui_hz 0"                  ui_hz=0
reject_case "tof_names trung"          'tof_names=[front, front, left, right]'
reject_case "tof_names thieu front"    'tof_names=[rear, rear, left, right]'
reject_case "tof_names ten la"         'tof_names=[front, rear, left, middle]'

# ---------------------------------------------------------------------------------------------------------------
# Chạy được ở biên hợp lệ + tắt bằng SIGINT / SIGTERM trực tiếp + cảnh báo khoá thiếu
run_and_signal() {   # run_and_signal <nhãn> <tín-hiệu> <file-config> -> đặt RES, ALIVE, STOP_MS, HUNG, OUTFILE
    local label=$1 sig=$2 cfg=$3
    OUTFILE="$T/out_$label.txt"
    sigrun "$sig" 2500 "$OUTFILE" "$MAIN" --config "$cfg" --mode auto
}

accept_case() {   # accept_case <nhãn> <tín-hiệu> khoá=giá-trị...
    local label=$1 sig=$2; shift 2
    local d="$T/a_$label"; mkdir -p "$d/runs"
    mkcfg "$d/c.yaml" "$d/runs" "$@"
    run_and_signal "$label" "$sig" "$d/c.yaml"
    check "[$label] bien hop le phai chay duoc (con song sau 2.5 s); dau ra: $(head -c 400 "$OUTFILE")" test "$ALIVE" = 1
    check "[$label] phai thoat trong <= 8 s sau $sig (khong treo)" test "$HUNG" = 0
    check "[$label] sau $sig phai tu xu ly va thoat binh thuong (exited:0/130/143, khong bi tin hieu giet), nhan $RES" graceful_exit "$RES"
    check "[$label] dung xong trong <= 4000 ms, mat $STOP_MS" test "$STOP_MS" -le 4000
    local csv; csv=$(find_csv "$d/runs" _auto)
    check "[$label] phai co log.csv" test -n "$csv"
    if [[ -n $csv ]]; then
        check "[$label] log.csv co du lieu" test "$(csv_rows "$csv")" -ge 5
        check "[$label] khong dong log nao co lenh khac 0 (StopController)" test "$(csv_nonzero_rows "$csv")" -eq 0
    fi
}
accept_case control10_int   INT  control_hz=10
accept_case bien_term       TERM sensor_hz=10 control_hz=10 image_timeout_ms=500 motor_keepalive_ms=200
accept_case trim_bien_int   INT  servo_trim_deg=40 max_steering_deg=20
accept_case trim_am_term    TERM servo_trim_deg=-40 max_steering_deg=20

# Thiếu khoá: chạy với mặc định và in cảnh báo
mkdir -p "$T/m/runs"
printf '%%YAML:1.0\n---\nlog_dir: "%s"\nfake_source: ""\nenable_display: 0\nenable_oled: 0\n' "$T/m/runs" > "$T/m/c.yaml"
run_and_signal missing INT "$T/m/c.yaml"
check "thieu khoa: van phai chay voi mac dinh (con song sau 2.5 s); dau ra: $(head -c 400 "$OUTFILE")" test "$ALIVE" = 1
check "thieu khoa: phai in canh bao/ghi chu dung mac dinh; dau ra: $(head -c 600 "$OUTFILE")" \
      grep -qiE 'c(a|ả)nh b(a|á)o|warn|m(a|ặ)c (d|đ)(i|ị)nh|default|thi(e|ế)u' "$OUTFILE"
check "thieu khoa: thoat chu dong sau INT, nhan $RES" graceful_exit "$RES"

# ---------------------------------------------------------------------------------------------------------------
# Ca 3: mất ảnh đầu-cuối bằng chương trình thật (camera giả hết ảnh): chương trình phải còn chạy, không crash và tắt sạch.
if [[ -n $FRAMES ]]; then
    d="$T/anh thu"; mkdir -p "$d/frames" "$d/runs"
    "$FRAMES" "$d/frames" 5
    mkcfg "$d/c.yaml" "$d/runs" "fake_source=\"$d/frames/frame_%06d.jpg\""
    run_and_signal imgloss TERM "$d/c.yaml"
    check "[mat anh] khi nguon anh het, chuong trinh phai TIEP TUC chay an toan (khong crash/khong tu thoat); dau ra: $(head -c 400 "$OUTFILE")" test "$ALIVE" = 1
    check "[mat anh] thoat chu dong sau TERM, nhan $RES" graceful_exit "$RES"
    csv=$(find_csv "$d/runs" _auto)
    check "[mat anh] phai co log.csv" test -n "$csv"
    if [[ -n $csv ]]; then
        # Sau 2.5 s kể từ khi hết 5 ảnh, tuổi ảnh cuối phải vượt 600 ms và lệnh phải bằng 0 ở mọi dòng đó.
        agecol=$(head -n1 "$csv" | tr ',' '\n' | grep -n '^image_age_ms$' | cut -d: -f1)
        check "[mat anh] log.csv phai co cot image_age_ms" test -n "$agecol"
        if [[ -n $agecol ]]; then
            maxage=$(awk -F, -v c="$agecol" 'NR>1{if($c+0>m)m=$c+0}END{print m+0}' "$csv")
            check "[mat anh] log phai ghi nhan tuoi anh > 600 ms sau khi nguon het (max=$maxage)" test "$(printf '%.0f' "$maxage")" -gt 600
        fi
        check "[mat anh] khong dong log nao co lenh khac 0" test "$(csv_nonzero_rows "$csv")" -eq 0
    fi
fi

# ---------------------------------------------------------------------------------------------------------------
# ucr_stop (bản giả lập): thoát 0 khi cấu hình tốt; mã khác 0 sạch khi cấu hình không đọc được (docs/ucr_car.md)
out=$(cd "$T/u" && timeout 10 "$STOP" --config "$T/u/ok.yaml" 2>&1); rc=$?
check "ucr_stop voi config tot (ban gia lap) phai thoat 0, nhan $rc; dau ra: $out" test "$rc" -eq 0
out=$(cd "$T/u" && timeout 10 "$STOP" --config "$T/u/khong_ton_tai.yaml" 2>&1); rc=$?
check "ucr_stop voi config khong doc duoc phai thoat khac 0 va sach, nhan $rc" clean_nonzero "$rc"

acc_finish acc_cli
