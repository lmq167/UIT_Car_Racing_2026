#!/usr/bin/env bash
set -euo pipefail
if [[ $# -lt 2 || $# -gt 3 ]]; then
    echo "Cách dùng: $0 model.onnx model_HxW_fp16.engine [workspace_MB=256]" >&2
    exit 2
fi
if [[ ! -f "$1" || ! "$2" =~ _[0-9]+x[0-9]+_fp16\.engine$ || ! "${3:-256}" =~ ^[1-9][0-9]*$ ]]; then
    echo "Lỗi: cần ONNX có thật, tên engine _HxW_fp16.engine và workspace nguyên dương" >&2
    exit 2
fi
trtexec=/usr/src/tensorrt/bin/trtexec
if [[ ! -x "$trtexec" ]]; then
    echo "Lỗi: thiếu chương trình $trtexec; không tự cài thêm" >&2
    exit 2
fi
"$trtexec" --onnx="$1" --saveEngine="$2" --fp16 --workspace="${3:-256}"
