#!/usr/bin/env bash
# Thư viện dùng chung cho test chấp nhận viết bằng bash. Dùng bằng: source "$(dirname "$0")/acc_lib.sh"
# Không phụ thuộc cách cài đặt; chỉ gọi chương trình qua dòng lệnh / script như người dùng.
set -u
set -m   # job control: tiến trình nền nhận SIGINT mặc định (không bị bỏ qua như khi chạy nền không job control)

ACC_DIR=$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
ROOT=$(CDPATH= cd -- "$ACC_DIR/../.." && pwd -P)       # thư mục gốc ucr_car
RUNSH="$ROOT/scripts/run.sh"
ACC_CHECKS=0
ACC_FAILS=0
ACC_TMP_PARENT=$(CDPATH= cd -- "${TMPDIR:-/tmp}" && pwd -P)
T=$(mktemp -d "$ACC_TMP_PARENT/ucr-acc.XXXXXX")

acc_cleanup() {
    # Dọn tiến trình nền còn sót rồi xoá thư mục tạm của chính test (đã xác định tuyệt đối).
    local p
    for p in $(jobs -p 2>/dev/null); do kill -KILL "$p" 2>/dev/null || :; done
    case "$T" in "$ACC_TMP_PARENT"/ucr-acc.*) rm -r -- "$T" 2>/dev/null || : ;; esac
}
trap acc_cleanup EXIT

# check "mô tả" lệnh-kiểm-tra...   (đạt khi lệnh trả 0)
check() {
    local desc=$1; shift
    ACC_CHECKS=$((ACC_CHECKS + 1))
    if ! "$@"; then
        ACC_FAILS=$((ACC_FAILS + 1))
        echo "[HONG] $desc" >&2
        return 1
    fi
    return 0
}
# fail "mô tả" – ghi lỗi vô điều kiện
fail() { ACC_CHECKS=$((ACC_CHECKS + 1)); ACC_FAILS=$((ACC_FAILS + 1)); echo "[HONG] $*" >&2; }

acc_finish() {
    echo "[$1] $ACC_CHECKS phep kiem, $ACC_FAILS loi"
    [[ $ACC_FAILS -eq 0 ]]
}

# Chờ file xuất hiện (tối đa $2 giây, mặc định 10).
wait_file() {
    local f=$1 n=$(( ${2:-10} * 20 )) i
    for ((i = 0; i < n; ++i)); do [[ -e $f ]] && return 0; sleep 0.05; done
    return 1
}
# Chờ tiến trình $1 kết thúc, tối đa $2 giây; trả 0 nếu đã kết thúc.
wait_gone() {
    local p=$1 n=$(( ${2:-10} * 20 )) i
    for ((i = 0; i < n; ++i)); do kill -0 "$p" 2>/dev/null || return 0; sleep 0.05; done
    return 1
}
now_ms() { echo $(( $(date +%s%N) / 1000000 )); }

# mkcfg <file> <log_dir> [khoá=giá-trị-YAML ...]
# Lấy config.yaml đang ship làm gốc (đã hợp lệ), tắt cửa sổ/OLED, đưa log_dir và đường dẫn hiệu chuẩn về tuyệt đối,
# rồi áp các ghi đè. Khoá bị ghi đè được xoá khỏi bản gốc trước khi nối giá trị mới.
mkcfg() {
    local out=$1 logdir=$2; shift 2
    cp "$ROOT/config.yaml" "$out"
    local kv key val
    for kv in "log_dir=\"$logdir\"" 'fake_source=""' 'enable_display=0' 'enable_oled=0' \
              "imu_calibration_file=\"$ROOT/include/sensor/bno055_calibration.txt\"" "$@"; do
        key=${kv%%=*}; val=${kv#*=}
        sed -i "/^${key}:/d" "$out"
        printf '%s: %s\n' "$key" "$val" >> "$out"
    done
}

# Số dòng dữ liệu của log.csv có cmd_speed hoặc cmd_steer khác 0 (bỏ dòng cụt); in ra số đó.
csv_nonzero_rows() {
    awk -F, 'NR==1{for(i=1;i<=NF;i++){if($i=="cmd_speed")s=i; if($i=="cmd_steer")a=i}; n=NF; next}
             NF==n && s>0 && a>0 {if ($s+0!=0 || $a+0!=0) c++} END{print c+0}' "$1"
}
csv_rows() { awk 'END{print (NR>1)?NR-1:0}' "$1"; }
# Tìm log.csv của lượt chạy trong thư mục log_dir với hậu tố chế độ (ví dụ _auto).
find_csv() { ls -d "$1"/*"$2"/log.csv 2>/dev/null | tail -n 1; }

# sigrun <tín-hiệu> <chờ-ms> <file-dầu-ra> lệnh...   (cần biến ACC_SIGRUN = đường dẫn acc_sigrun)
# Đặt: ALIVE (còn chạy trước khi gửi tín hiệu), RES (exited:<mã> | signaled:<n> | hung), STOP_MS, HUNG.
sigrun() {
    local sig=$1 w=$2 out=$3; shift 3
    local res
    res=$("$ACC_SIGRUN" "$sig" "$w" 10000 "$out" -- "$@")
    ALIVE=$(sed -n 's/^alive=//p' <<<"$res")
    RES=$(sed -n 's/^result=//p' <<<"$res")
    STOP_MS=$(sed -n 's/^elapsed_ms=//p' <<<"$res")
    if [[ $RES == hung ]]; then HUNG=1; else HUNG=0; fi
}
# Thoát chủ động (không bị tín hiệu giết) với mã 0 hoặc 128+tín hiệu (quy ước shell cho "đã xử lý tín hiệu và dừng").
graceful_exit() { [[ $1 == exited:0 || $1 == exited:130 || $1 == exited:143 ]]; }
