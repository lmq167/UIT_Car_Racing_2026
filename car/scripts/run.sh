#!/usr/bin/env bash
# Trình bao chờ tiến trình con rồi luôn gửi lệnh dừng độc lập.
set -u
# Tách nhóm tiến trình để con nhận được SIGINT khi chạy nền.
set -m
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd -P)
build=${UCR_BUILD_DIR:-"$root/build"}
# Chuẩn hóa trước khi đổi thư mục: --config tính từ nơi gọi trình bao.
case "$build" in /*) ;; *) build="$PWD/$build" ;; esac
config="$root/config.yaml"
has_config=0
args=("$@")
for ((i=0; i<${#args[@]}; ++i)); do
    if [[ ${args[i]} == --config && $((i+1)) -lt ${#args[@]} ]]; then
        config=${args[i+1]}
        case "$config" in /*) ;; *) config="$PWD/$config" ;; esac
        args[i+1]=$config
        has_config=1
    fi
done
if [[ $has_config == 0 ]]; then args+=(--config "$config"); fi
cd "$build" || exit 1
child=
forward() {
    if [[ -n $child ]]; then kill -s "$1" "$child" 2>/dev/null || :; fi
}
cleanup() {
    local status=$?
    trap - EXIT INT TERM
    "$build/ucr_stop" --config "$config" || printf '%s\n' 'LOI: ucr_stop khong xac nhan dung duoc thiet bi' >&2
    exit "$status"
}
trap cleanup EXIT
trap 'forward INT' INT
trap 'forward TERM' TERM
# Không dùng exec: tiến trình bao phải sống để dừng khi con bị SIGKILL.
# Đặt lại SIGINT vốn có thể bị bash bỏ qua cho tác vụ nền không tương tác.
( trap - INT TERM; exec "$build/main" "${args[@]}" ) &
child=$!
status=0
while :; do
    wait "$child"
    status=$?
    # wait bị ngắt bởi tín hiệu vào trình bao: tiếp tục đợi con thực sự kết thúc.
    if kill -0 "$child" 2>/dev/null; then continue; fi
    break
done
exit "$status"
