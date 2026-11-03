#!/usr/bin/env bash
# Người đứng cạnh xe thao tác; không tự trả lời thay các xác nhận.
set -euo pipefail
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd -P)
echo "Phiên bản git: $(git -C "$root" rev-parse HEAD 2>/dev/null || echo 'không có metadata git')"
echo 'Bánh phải kê; ToF phải được đối chiếu. y + Enter để xác nhận; x dừng; q thoát.'
marker=$(mktemp "${TMPDIR:-/tmp}/ucr-smoke.XXXXXXXX")
trap 'rm -f -- "$marker"' EXIT
rc=0
timeout -k 3s --signal=TERM 125s bash "$root/scripts/run.sh" --mode smoke --smoke-drop-camera --smoke-kill-file "$marker" "$@" || rc=$?
if [[ -s $marker ]]; then
    IFS= read -r report < "$marker"
    echo "Kết quả: $report/smoke.json"
else
    echo 'KHÔNG ĐẠT: chưa hoàn tất kiểm tra motor và kill -9; xem smoke.json trong lượt vừa in.'
    rc=1
fi
exit "$rc"
