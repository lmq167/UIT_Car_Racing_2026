#!/usr/bin/env bash
# Giữ con ở tiền cảnh để đọc tty; trình bao sống để gọi đường dừng độc lập.
# SIGKILL trình bao/mất điện có thể giữ lệnh cuối: dùng công tắc nguồn motor.
set -u
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd -P) || exit 1
build=${UCR_BUILD_DIR:-"$root/build"}
case "$build" in /*) ;; *) build="$PWD/$build" ;; esac
if [[ ! -f "$build/ucr_stop" || ! -x "$build/ucr_stop" ]]; then
    printf '%s\n' 'Thiếu ucr_stop thực thi được; không khởi động lái xe.' >&2
    exit 1
fi
config="$root/config.yaml"
tty_state=$(stty -g <&0 2>/dev/null) || tty_state=
args=()
parse_status=0
while (( $# )); do
    if [[ $1 == --config ]]; then
        if (( $# < 2 )) || [[ $2 == --* || -z $2 ]]; then
            printf '%s\n' 'Thiếu giá trị --config' >&2
            parse_status=2
            break
        fi
        config=$2
        case "$config" in /*) ;; *) config="$PWD/$config" ;; esac
        shift 2
    else
        args+=("$1")
        shift
    fi
done
if (( parse_status )); then exit "$parse_status"; fi
# Chỉ thử khóa rồi nhả; công cụ tự giữ lại cùng khóa suốt thời gian điều khiển.
lock_file=/tmp/ucr_vehicle.lock
if ! exec {lock_fd}>>"$lock_file"; then
    printf '%s\n' 'Không mở được khóa xe; không khởi động lái xe.' >&2
    exit 1
fi
flock -n -E 6 "$lock_fd"
lock_status=$?
exec {lock_fd}>&-
if (( lock_status )); then
    if (( lock_status == 6 )); then
        printf '%s\n' 'Xe đang được phiên khác giữ khóa; đóng phiên đó trước. Không gọi ucr_stop.' >&2
        exit 6
    fi
    printf '%s\n' 'Không kiểm tra được khóa xe; không khởi động lái xe.' >&2
    exit 1
fi
# Dừng lệnh còn sót của lần chạy trước; chưa cài EXIT để không gọi lại nếu lỗi.
if ! timeout -k 1 10 "$build/ucr_stop" --config "$config"; then
    printf '%s\n' 'KIEM TRA XE, DUNG CONG TAC NGUON: ucr_stop trước khi chạy thất bại' >&2
    exit 1
fi
cleanup() {
    local status=$?
    trap - EXIT INT TERM HUP QUIT TSTP
    if [[ $status == 6 ]]; then
        printf '%s\n' 'Xe đang được phiên khác giữ khóa; đóng phiên đó trước. Không gọi ucr_stop.' >&2
    elif ! timeout -k 1 10 "$build/ucr_stop" --config "$config"; then
        printf '%s\n' 'KIEM TRA XE, DUNG CONG TAC NGUON: ucr_stop không xác nhận dừng' >&2
        if [[ $status == 0 ]]; then status=1; fi
    fi
    # Khôi phục cả khi con bị SIGKILL nên không tự trả terminal về được.
    if [[ -n $tty_state ]]; then stty "$tty_state" <&0 2>/dev/null || :; fi
    exit "$status"
}
trap cleanup EXIT
# Tín hiệu terminal được gửi cho cả nhóm tiền cảnh; đợi con tự dừng rồi mới cleanup.
trap ':' INT TERM HUP QUIT TSTP
cd "$build" || exit 1
"$build/ucr_drive_collect" "${args[@]}"
status=$?
exit "$status"
