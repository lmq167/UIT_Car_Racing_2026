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
check 'tham số không biết: --khong-co' --mode collect --khong-co
check 'thiếu giá trị tham số: --config' --config
check 'thiếu giá trị tham số: --mode' --mode
check 'thiếu giá trị tham số: --config' --config --mode collect
printf '%s\n' 'Đạt kiểm tra tham số dòng lệnh'
