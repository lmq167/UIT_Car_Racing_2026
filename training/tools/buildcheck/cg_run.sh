#!/usr/bin/env bash
# Chạy một lệnh trong scope cgroup hiện tại rồi ghi memory.peak và oom_kill của scope đó
# (scope bị gỡ khi tiến trình thoát nên phải đọc trước). Dùng: cg_run.sh <tiền-tố-file> <lệnh...>
prefix=$1; shift
"$@"
rc=$?
cg=/sys/fs/cgroup$(sed -n 's/^0:://p' /proc/self/cgroup)
cat "$cg/memory.peak" >"$prefix.cgpeak" 2>/dev/null || echo - >"$prefix.cgpeak"
awk '$1 == "oom_kill" { print $2 }' "$cg/memory.events" >"$prefix.cgoom" 2>/dev/null || echo - >"$prefix.cgoom"
exit "$rc"
