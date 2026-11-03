#!/usr/bin/env bash
# Chỉ kiểm tra lỗi cú pháp trước khởi tạo thiết bị.
set -eu
main=$1
check() {
    local expected=$1 result status=0
    shift
    result=$("$main" "$@" 2>&1) || status=$?
    [[ $status == 1 && $result == *"$expected"* ]] || {
        printf 'Sai thông báo/mã thoát: %s\n' "$result" >&2; exit 1;
    }
}
check 'tham số không biết: --khong-co' --khong-co
check 'tham số không biết: --khong-co' --mode manual --khong-co
check 'thiếu giá trị tham số: --config' --config
check 'thiếu giá trị tham số: --mode' --mode
check 'thiếu giá trị tham số: --config' --config --mode manual
check 'thiếu bước hiệu chuẩn' --mode calib
check 'Bước hiệu chuẩn không hợp lệ' --mode calib khong-co
for step in 0 11 -1 1.5 abc; do
    check 'Bước hiệu chuẩn không hợp lệ' --mode calib --step "$step"
done
check 'thiếu giá trị tham số: --step' --mode calib --step
check 'Bước hiệu chuẩn bị lặp' --mode calib servo --step 2
check '--step chỉ dùng với --mode calib' --step 6 --mode manual
folder=$(mktemp -d)
trap 'rm -rf -- "$folder"' EXIT
config="$folder/cau hinh.yaml"
cp -- "$(dirname -- "${BASH_SOURCE[0]}")/../config.yaml" "$config"
"$main" --check-config --config "$config" >"$folder/out" 2>"$folder/err"
printf 'CAU_HINH_OK: %s\n' "$config" >"$folder/expected"
cmp "$folder/out" "$folder/expected"
check_config_error() {
    local expected=$1 status=0
    shift
    "$main" "$@" >"$folder/out" 2>"$folder/err" || status=$?
    [[ $status == 2 && ! -s "$folder/out" && $(cat "$folder/err") == *"$expected"* ]] || {
        printf 'Sai hợp đồng kiểm cấu hình: mã %s\n' "$status" >&2; cat "$folder/err" >&2; exit 1;
    }
}
check_config_error 'Cú pháp:' --check-config
check_config_error 'Cú pháp:' --check-config "$config"
check_config_error 'Cú pháp:' --config "$config" --check-config
check_config_error 'Cú pháp:' --check-config --config "$config" --mode auto
check_config_error 'Cú pháp:' --check-config --help
check_config_error 'Cú pháp:' --check-config --config
check_config_error "không đọc được: $folder/khong-co.yaml" --check-config --config "$folder/khong-co.yaml"
printf '%%YAML:1.0\n---\nresume_stable_s: "2"\n' >"$config"
check_config_error 'resume_stable_s = "2" ngoài khoảng hợp lệ [0.5, 30] s' --check-config --config "$config"
printf '%%YAML:1.0\n---\nresume_key: r\n' >"$config"
check_config_error 'khoá lạ: resume_key' --check-config --config "$config"
[[ ! -e "$folder/runs" ]]
printf '%s\n' 'Đạt kiểm tra tham số dòng lệnh'
