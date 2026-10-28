#!/usr/bin/env bash
# Kiểm tra trình bao bằng tiến trình giả, không mở phần cứng.
set -eu
set -m
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd -P)
tmp_parent=$(CDPATH= cd -- "${TMPDIR:-/tmp}" && pwd)
tmp=$(mktemp -d "$tmp_parent/ucr-wrapper.XXXXXX")
cleanup() {
    # Chỉ xóa thư mục tạm vừa tạo trong thư mục cha đã xác định tuyệt đối.
    case "$tmp" in "$tmp_parent"/ucr-wrapper.*)
        if [[ -s "$tmp/child" ]]; then kill -TERM "$(cat "$tmp/child")" 2>/dev/null || :; fi
        rm -rf -- "$tmp" ;;
    esac
}
trap cleanup EXIT
cat > "$tmp/ucr_stop" <<'STOP'
#!/usr/bin/env bash
printf '%s\n' "$@" >> "$UCR_TEST_DIR/stops"
exit "${UCR_TEST_STOP_STATUS:-0}"
STOP
cat > "$tmp/main" <<'MAIN'
#!/usr/bin/env bash
printf '%s\n' "$$" > "$UCR_TEST_DIR/child"
printf '%s\n' "$@" > "$UCR_TEST_DIR/main_args"
case "$1" in
    --mode)
        [[ $2 == collect ]] || exit 8
        IFS= read -r answer
        printf '%s\n' "$answer" > "$UCR_TEST_DIR/answer"
        exit 0 ;;
    normal) exit 0 ;;
    error) exit 7 ;;
    crash) kill -SEGV "$$" ;;
    hold)
        trap 'exit 130' INT
        trap 'exit 143' TERM
        while :; do sleep 0.05; done ;;
esac
MAIN
chmod +x "$tmp/main" "$tmp/ucr_stop"
export UCR_BUILD_DIR="$tmp" UCR_TEST_DIR="$tmp"
run_case() {
    local name=$1 expected=$2 target=${3:-} config_arg=${4:-'tep co dau cach.yaml'} status=0 count
    local expected_config=$config_arg
    case "$config_arg" in
        MAC_DINH) expected_config="$root/config.yaml" ;;
        /*) ;;
        *) expected_config="$PWD/$config_arg" ;;
    esac
    : > "$tmp/stops"
    rm -f -- "$tmp/child"
    rm -f -- "$tmp/main_args"
    if [[ $config_arg == MAC_DINH ]]; then
        bash "$root/scripts/run.sh" "$name" &
    else
        bash "$root/scripts/run.sh" "$name" --config "$config_arg" &
    fi
    local wrapper=$!
    # Giới hạn chờ để lỗi xử lý tín hiệu không làm treo bộ kiểm tra.
    (sleep 8; if [[ -s "$tmp/child" ]]; then kill -KILL "$(cat "$tmp/child")" 2>/dev/null || :; fi; kill -KILL "$wrapper" 2>/dev/null || :) &
    local watchdog=$!
    if [[ -n $target ]]; then
        for ((count=0;count<200;++count)); do
            [[ -s "$tmp/child" ]] && break
            sleep 0.01
        done
        local child; child=$(cat "$tmp/child")
        if [[ $target == child ]]; then kill -KILL "$child"; else kill -s "$target" "$wrapper"; fi
    fi
    wait "$wrapper" || status=$?
    kill "$watchdog" 2>/dev/null || :
    wait "$watchdog" 2>/dev/null || :
    [[ $status == "$expected" ]] || { printf 'Sai ma thoat: %s/%s\n' "$status" "$expected"; exit 1; }
    # Mỗi đối số trên một dòng: kiểm tra cả ranh giới khi đường dẫn có dấu cách.
    [[ $(wc -l < "$tmp/stops") == 2 ]]
    [[ $(cat "$tmp/stops") == "$(printf '%s\n' --config "$expected_config")" ]]
    if [[ -f "$tmp/main_args" ]]; then
        [[ $(cat "$tmp/main_args") == "$(printf '%s\n' "$name" --config "$expected_config")" ]]
    else
        [[ $expected == 127 ]]
    fi
    printf 'DAT: %s %s -> %s\n' "$name" "$target" "$status"
}
run_case normal 0
run_case error 7
export UCR_TEST_STOP_STATUS=9
run_case normal 1 2>"$tmp/stop_error"
grep -Fx 'LOI: ucr_stop khong xac nhan dung duoc thiet bi' "$tmp/stop_error"
run_case error 7 2>"$tmp/stop_error"
grep -Fx 'LOI: ucr_stop khong xac nhan dung duoc thiet bi' "$tmp/stop_error"
unset UCR_TEST_STOP_STATUS
run_case crash 139
run_case hold 137 child
run_case hold 143 TERM
run_case hold 130 INT

# Gọi từ nơi khác thư mục mã và thư mục xây dựng.
mkdir -p "$tmp/noi goi/cau hinh"
cd "$tmp/noi goi"
run_case normal 0 '' config.yaml
run_case normal 0 '' 'cau hinh/tep co dau cach.yaml'
run_case normal 0 '' "$tmp/noi goi/cau hinh/tuyet doi.yaml"
run_case normal 0 '' MAC_DINH

# Thu ảnh phải nhận được stdin và vẫn dừng đúng một lần khi con kết thúc.
: > "$tmp/stops"
printf '%s\n' 'tên người thu có dấu cách' | bash "$root/scripts/run.sh" --mode collect --config 'cau hinh/thu.yaml'
[[ $(cat "$tmp/answer") == 'tên người thu có dấu cách' ]]
[[ $(cat "$tmp/main_args") == "$(printf '%s\n' --mode collect --config "$PWD/cau hinh/thu.yaml")" ]]
[[ $(cat "$tmp/stops") == "$(printf '%s\n' --config "$PWD/cau hinh/thu.yaml")" ]]
printf '%s\n' 'DAT: collect nhận đầu vào và gọi dừng'

mv "$tmp/main" "$tmp/main.saved"
run_case normal 127
