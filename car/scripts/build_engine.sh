#!/usr/bin/env bash
set -euo pipefail
# --fp32 (đứng đầu): build engine FP32 để so với FP16 khi mask lệch; tên engine phải đuôi _fp32.engine
precision=fp16
if [[ "${1:-}" == --fp32 ]]; then precision=fp32; shift; fi
if [[ $# -lt 2 || $# -gt 3 ]]; then
    echo "Cách dùng: $0 [--fp32] model.onnx model_HxW_${precision}.engine [workspace_MB=256]" >&2
    exit 2
fi
if [[ ! -f "$1" || ! "$2" =~ _[0-9]+x[0-9]+_${precision}\.engine$ || ! "${3:-256}" =~ ^[1-9][0-9]*$ ]]; then
    echo "Lỗi: cần ONNX có thật, tên engine _HxW_${precision}.engine và workspace nguyên dương" >&2
    exit 2
fi
trtexec=/usr/src/tensorrt/bin/trtexec
if [[ ! -x "$trtexec" ]]; then
    echo "Lỗi: thiếu chương trình $trtexec; không tự cài thêm" >&2
    exit 2
fi
fp16_flag=(); [[ $precision == fp16 ]] && fp16_flag=(--fp16)
"$trtexec" --onnx="$1" --saveEngine="$2" ${fp16_flag[@]+"${fp16_flag[@]}"} --workspace="${3:-256}"
