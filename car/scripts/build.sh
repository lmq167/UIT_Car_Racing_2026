#!/usr/bin/env bash
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
mkdir -p "$root/build"
cd "$root/build"
# Bản xe mặc định không biên dịch test; có thể bật lại khi cần kiểm tra.
cmake .. -DBUILD_TESTING="${UCR_BUILD_TESTING:-OFF}" "$@"
cmake --build . -- -j2
