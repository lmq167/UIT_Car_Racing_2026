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
T=$(mktemp -d "$ACC_TMP_PARENT/ucr-acc.XXXXXX") || exit 1
# Không nối vết vào tệp dùng chung được kế thừa từ môi trường chạy CTest.
export UCR_FAKE_HW_TRACE="$T/trace.csv"
mkdir -p "$T/processes"
export ACC_PROCESS_DIR="$T/processes"

# Trình bao được phép bật job control và tạo nhóm con; ghi nhóm ngay trong main/stop giả trước exec.
acc_track_group() {
    local pid=$1 pgid
    pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d '[:space:]')
    if [[ -z $pgid ]]; then
        # ps của Git Bash không có -o; đọc đúng cột PGID theo tiêu đề.
        pgid=$(ps -l -p "$pid" 2>/dev/null | awk 'NR==1{for(i=1;i<=NF;i++)if($i=="PGID")g=i} NR==2 && g{print $g}')
    fi
    [[ $pgid =~ ^[1-9][0-9]*$ ]] || { echo "[HONG] Khong doc duoc PGID cua $pid" >&2; return 1; }
    printf '%s\n' "$pgid" > "$ACC_PROCESS_DIR/$pid"
}
export -f acc_track_group

# Job control tạo nhóm riêng cho từng lệnh nền; lưu ngay khi khởi chạy, kể cả cha chết trước con.
acc_start() {
    set -m
    "$@" &
    ACC_PID=$!
    # Sổ nhóm dùng chung với cả command substitution (lệnh trong $(...)).
    printf '%s\n' "$ACC_PID" > "$T/processes/$ACC_PID" || {
        kill -KILL -- "-$ACC_PID" 2>/dev/null || :
        wait "$ACC_PID" 2>/dev/null || :
        return 1
    }
}

acc_reap() {
    local pid=$1 pgid
    [[ -f $T/processes/$pid ]] || return 0
    pgid=$(cat "$T/processes/$pid")
    kill -KILL -- "-$pgid" 2>/dev/null || :
    wait "$pid" 2>/dev/null || :
    rm -f -- "$T/processes/$pid"
}

acc_run() {
    acc_start "$@" || return 1
    local pid=$ACC_PID rc
    wait "$pid"; rc=$?
    acc_reap "$pid"
    return "$rc"
}

acc_cleanup() {
    trap '' INT TERM
    # Chỉ giết các nhóm do chính ca tạo; thu hồi con trực tiếp trước khi xoá dữ liệu.
    local p
    for p in "$T/processes/"*; do [[ -f $p ]] && acc_reap "${p##*/}"; done
    case "$T" in "$ACC_TMP_PARENT"/ucr-acc.*) rm -r -- "$T" 2>/dev/null || : ;; esac
}
trap acc_cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

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
    local f=$1 deadline=$(( $(now_ms) + ${2:-10} * 1000 ))
    while [[ $(now_ms) -lt $deadline ]]; do [[ -e $f ]] && return 0; sleep 0.05; done
    return 1
}
# Chờ tiến trình $1 kết thúc, tối đa $2 giây; trả 0 nếu đã kết thúc.
wait_gone() {
    local p=$1 deadline=$(( $(now_ms) + ${2:-10} * 1000 ))
    while [[ $(now_ms) -lt $deadline ]]; do kill -0 "$p" 2>/dev/null || return 0; sleep 0.05; done
    return 1
}
now_ms() { echo $(( $(date +%s%N) / 1000000 )); }

trace_lines() { if [[ -f $1 ]]; then wc -l < "$1" | tr -d ' '; else echo 0; fi; }
trace_moving() {
    [[ -f $1 ]] || return 1
    awk -F, 'NF==3 && $2=="motor_speed" && $3 ~ /^-?[0-9]+(\.[0-9]+)?([eE][-+]?[0-9]+)?$/ &&
        ($3+0>0.001 || $3+0< -0.001){n++} END{exit !(n>=5)}' "$1"
}
wait_moving() {
    local trace=$1 pid=$2 logdir=${3:-} csv deadline=$(( $(now_ms) + 10000 ))
    while [[ $(now_ms) -lt $deadline ]]; do
        kill -0 "$pid" 2>/dev/null || return 1
        if trace_moving "$trace"; then
            [[ -n $logdir ]] || return 0
            csv=$(find_csv "$logdir" _auto)
            if [[ -n $csv ]] && [[ $(csv_rows "$csv") -ge 5 ]]; then return 0; fi
        fi
        sleep 0.05
    done
    return 1
}
# docs/ucr_car.md mục Luồng/an toàn không có hạn ms cho INT/TERM: dùng 2 chu kỳ theo yêu cầu T09e.
# Dung sai không viết cứng: acc_sigrun đo khoảng quan sát, quá một chu kỳ thì HỎNG phép đo.
signal_timing_config() {
    export ACC_CONTROL_PERIOD_MS
    ACC_CONTROL_PERIOD_MS=$(awk '$1=="control_hz:" {print 1000/$2}' "$1")
    export ACC_STOP_BUDGET_MS
    ACC_STOP_BUDGET_MS=$(awk -v period="$ACC_CONTROL_PERIOD_MS" 'BEGIN{print 2*period}')
}
trace_tail() {
    [[ -f $1 ]] || return 1
    awk -F, -v trim="$2" '$2=="motor_speed"{s=$3; ns++} $2=="servo_angle"{a=$3; na++}
        END{d=a-trim; if(d<0)d=-d; num="^-?[0-9]+(\\.[0-9]+)?([eE][-+]?[0-9]+)?$";
            exit !(ns && na && s~num && a~num && s+0<=0.001 && s+0>=-0.001 && d<=0.001)}' "$1"
}

# mkcfg <file> <log_dir> [khoá=giá-trị-YAML ...]
# Lấy config.yaml đang ship làm gốc (đã hợp lệ), tắt cửa sổ/OLED, đưa log_dir và đường dẫn hiệu chuẩn về tuyệt đối,
# rồi áp các ghi đè. Khoá bị ghi đè được xoá khỏi bản gốc trước khi nối giá trị mới.
mkcfg() {
    local out=$1 logdir=$2; shift 2
    cp "$ROOT/config.yaml" "$out"
    local kv key val
    for kv in "log_dir=\"$logdir\"" 'fake_source=""' 'enable_display=0' 'enable_oled=0' 'tof_mapping_confirmed=1' \
              "imu_calibration_file=\"$ROOT/include/sensor/bno055_calibration.txt\"" "$@"; do
        key=${kv%%=*}; val=${kv#*=}
        sed -i "/^${key}:/d" "$out"
        printf '%s: %s\n' "$key" "$val" >> "$out"
    done
    # Test auto cũ cần vượt cổng smoke để còn kiểm được mục tiêu riêng (smoke S13 kiểm gate lỗi riêng).
    local seed=${ACC_SEED_GATE:-${ACC_SIGRUN%/*}/acc_seed_gate}
    mkdir -p "$logdir"
    "$seed" "$logdir" "$out" || { fail 'Khong dung duoc tien dieu kien smoke gia'; return 1; }
}

# Số dòng dữ liệu của log.csv có cmd_speed hoặc cmd_steer khác 0 (bỏ dòng cụt); in ra số đó.
csv_nonzero_rows() {
    awk -F, 'NR==1{for(i=1;i<=NF;i++){if($i=="cmd_speed")s=i; if($i=="cmd_steer")a=i}; n=NF; next}
             NF==n && s>0 && a>0 {v=$s+0; if (v>0.001 || v< -0.001 || $a+0!=0) c++} END{print c+0}' "$1"
}
csv_rows() { awk 'END{print (NR>1)?NR-1:0}' "$1"; }
# Tìm log.csv của lượt chạy trong thư mục log_dir với hậu tố chế độ (ví dụ _auto).
find_csv() { ls -d "$1"/*"$2"/log.csv 2>/dev/null | tail -n 1; }

# sigrun <tín-hiệu> <chờ-ms> <file-dầu-ra> lệnh...   (cần biến ACC_SIGRUN = đường dẫn acc_sigrun)
# Đặt: ALIVE (còn chạy trước khi gửi tín hiệu), RES (exited:<mã> | signaled:<n> | hung), STOP_MS, HUNG.
sigrun() {
    local sig=$1 w=$2 out=$3; shift 3
    local res
    local result="$T/sigrun_result"
    acc_start "$ACC_SIGRUN" "$sig" "$w" 10000 "$out" -- "$@" >"$result" || return 1
    local helper=$ACC_PID rc
    wait "$helper"; rc=$?
    acc_reap "$helper"
    res=$(cat "$result")
    ALIVE=$(sed -n 's/^alive=//p' <<<"$res")
    RES=$(sed -n 's/^result=//p' <<<"$res")
    STOP_MS=$(sed -n 's/^elapsed_ms=//p' <<<"$res")
    READY=$(sed -n 's/^ready=//p' <<<"$res")
    if [[ -n ${ACC_SIGNAL_TRACE:-} ]]; then
        check 'Do dung theo han va dung sai da in [DO_DUNG]' grep -qx 'timing_ok=1' <<<"$res"
    fi
    check 'Bo gui tin hieu chay thanh cong' test "$rc" -eq 0
    if [[ -n ${ACC_READY_FILE:-} ]]; then
        check "Tien de san sang phai xay ra trong <=10 s (ready=$READY)" test "$READY" = 1
    fi
    if [[ $RES == hung ]]; then HUNG=1; else HUNG=0; fi
}
# Thoát chủ động (không bị tín hiệu giết) với mã 0 hoặc 128+tín hiệu (quy ước shell cho "đã xử lý tín hiệu và dừng").
graceful_exit() { [[ $1 == exited:0 || $1 == exited:130 || $1 == exited:143 ]]; }
