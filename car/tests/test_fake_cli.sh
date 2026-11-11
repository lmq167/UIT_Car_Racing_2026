#!/usr/bin/env bash
# Kiểm tra lỗi kiểu cấu hình, kịch bản phím giả và vết thiết bị của main bản giả.
set -euo pipefail
main_bin="$1"
root="$(mktemp -d '/tmp/ucr fake cli.XXXXXX')"
cfg="$root/config.yaml"
trace="$root/trace.csv"
unset UCR_FAKE_HW_TRACE UCR_FAKE_FAULT UCR_FAKE_TOF_SCRIPT UCR_FAKE_CAMERA_GAPS UCR_FAKE_KEYS
printf '%%YAML:1.0\n---\nservo_trim_deg: 9\nenable_display: 0\nenable_oled: 0\ntof_mapping_confirmed: 1\nlog_dir: "%s"\n' "$root/runs" > "$cfg"
export UCR_FAKE_HW_TRACE="$trace"

for key in control_hz max_speed_cms; do
    for value in abc '""' '[]' '{}'; do
        printf '%%YAML:1.0\n---\n%s: %s\n' "$key" "$value" > "$root/bad.yaml"
        : > "$trace"
        rc=0
        timeout 5s "$main_bin" --config "$root/bad.yaml" --mode auto > "$root/error.txt" 2>&1 || rc=$?
        [[ "$rc" -eq 1 && ! -s "$trace" ]]
        grep -F "$key" "$root/error.txt" > /dev/null
    done
done
# Kịch bản phím sai phải bị từ chối trước cả tạo runs và khởi tạo thiết bị.
for keys in ':w' '-1:w' '+1:w' '1.5:w' '1:' '1:ww' '1:w,' '1:w,,2:s' '9223372036854775808:w' '1:é'; do
    : > "$trace"
    rc=0
    UCR_FAKE_KEYS="$keys" timeout 5s "$main_bin" --config "$cfg" --mode manual > "$root/error.txt" 2>&1 || rc=$?
    [[ "$rc" -eq 1 && ! -s "$trace" && ! -d "$root/runs" ]]
    grep -F UCR_FAKE_KEYS "$root/error.txt" > /dev/null
done
# Bàn phím giả vẫn lái/dừng/thoát qua nhánh manual khi không có cửa sổ.
: > "$trace"
UCR_FAKE_KEYS='500:w,1000:x,1200:q' timeout 5s "$main_bin" --config "$cfg" --mode manual > "$root/keys.txt" 2>&1
awk -F, '$2=="motor_speed" {if($3>0) moved=1; else if(moved) stopped=1}
    END {if(!moved || !stopped) exit 1}' "$trace"

export UCR_BUILD_DIR="$(dirname "$main_bin")"
check_last() {
    awk -F, '$2=="motor_init" {init++} $2=="motor_cfg" {cfg++}
        $2=="motor_speed" {speed=$3; motor++} {kind=$2; value=$3}
        END {if(init!=1 || cfg!=7 || motor<2 || speed!=0 || kind!="servo_angle" || value!=9) exit 1}' "$trace"
}
# Thiếu cổng phải bị từ chối cả ở bản giả dùng bộ điều khiển mặc định.
: > "$trace"
rc=0
timeout 5s "$main_bin" --config "$cfg" --mode auto > "$root/no_gate.txt" 2>&1 || rc=$?
[[ "$rc" -eq 1 && ! -s "$trace" ]]
python_cmd=$(command -v python3 || command -v python)
"$python_cmd" - "$main_bin" "$cfg" "$root/runs" <<'PY'
import json
import pathlib
import re
import subprocess
import sys
info = subprocess.check_output([sys.argv[1], '--env-info'], timeout=5).decode('utf-8')
assert re.search(r'simulated=1(?:\s|$)', info), 'Chỉ cấp mẫu cho bản giả'
gate = dict(source_id=re.search(r'source_id=([^;\s]+)', info).group(1),
            simulated=1, config_text=pathlib.Path(sys.argv[2]).read_bytes().decode('utf-8'))
gate.update({key: 1 for key in ('camera', 'sensors', 'servo', 'tof_brake', 'camera_brake',
                              'encoder_moving', 'kill9', 'encoder_stopped', 'passed')})
folder = pathlib.Path(sys.argv[3])
folder.mkdir(parents=True, exist_ok=True)
(folder / 'smoke_gate.json').write_text(json.dumps(gate), encoding='utf-8')
PY
# Mẫu smoke ĐẠT chỉ dùng kiểm bản giả; phải khớp mã nguồn và toàn văn cấu hình.
: > "$trace"
rc=0
timeout --preserve-status -s TERM -k 2s 3s "$main_bin" --config "$cfg" --mode auto > "$root/auto.txt" 2>&1 || rc=$?
[[ "$rc" -eq 143 ]]
check_last

# Đầu vào đóng không kết thúc phiên quan sát; lượt motor đang chạy phải bị huỷ.
for step in imu motor; do
    printf '%%YAML:1.0\n---\nservo_trim_deg: 9\nenable_display: 0\nenable_oled: 0\ntof_mapping_confirmed: 1\nlog_dir: "%s"\n' "$root/runs" > "$cfg"
    : > "$trace"
    rc=0
    if [[ $step == motor ]]; then
        { sleep 0.3; printf 'y\n'; sleep 0.5; } |
            timeout --preserve-status -s TERM -k 2s 2s "$main_bin" --config "$cfg" --mode calib "$step" > "$root/eof_$step.txt" 2>&1 || rc=$?
        awk -F, '$2=="motor_speed" {if($3>0) {if(stopped) exit 1; moved=1} else if(moved) stopped=1}
            END {if(!moved || !stopped) exit 1}' "$trace"
    else
        timeout --preserve-status -s TERM -k 2s 2s "$main_bin" --config "$cfg" --step 6 --mode calib < /dev/null > "$root/eof_$step.txt" 2>&1 || rc=$?
    fi
    [[ "$rc" -eq 143 ]]
    check_last
done
printf 'Đạt kiểm tra dòng lệnh bản giả; dữ liệu ở %s\n' "$root"
