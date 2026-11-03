#!/usr/bin/env bash
set -euo pipefail
if [[ $# -lt 1 || $# -gt 3 || ( "$1" != unet && "$1" != yolo ) ]]; then
    echo "Cách dùng: $0 <unet|yolo> [thư_mục=\$HOME/ucr_models] [workspace_MB=256]" >&2
    exit 2
fi
name=$1
dir=${2:-$HOME/ucr_models}
workspace=${3:-256}
onnx=$dir/$name.onnx
engine=$dir/$name.engine
if [[ ! -f "$onnx" || ! "$workspace" =~ ^[1-9][0-9]*$ ]]; then
    echo "Lỗi: cần $onnx có thật và workspace nguyên dương" >&2
    exit 2
fi
trtexec=/usr/src/tensorrt/bin/trtexec
if [[ ! -x "$trtexec" ]]; then
    echo "Lỗi: thiếu chương trình $trtexec; không tự cài thêm" >&2
    exit 2
fi
trap 'rm -f -- "$engine.tmp" "$engine.src.tmp"' EXIT
rm -f -- "$engine.tmp" "$engine.src.tmp"
# Băm trước build và ghi nguồn tạm trước khi thay engine cũ.
source_hash=$(sha256sum -- "$onnx")
source_hash=${source_hash:0:64}
[[ "$source_hash" =~ ^[0-9a-f]{64}$ ]]
"$trtexec" --onnx="$onnx" --saveEngine="$engine.tmp" --fp16 --workspace="$workspace"
[[ -s "$engine.tmp" ]]
current_hash=$(sha256sum -- "$onnx")
[[ "${current_hash:0:64}" == "$source_hash" ]]
printf '%s\n' "$source_hash" > "$engine.src.tmp"
# bị ngắt giữa hai lần mv thì thiếu .src, app từ chối thay vì nhận cặp lệch
rm -f -- "$engine.src"
mv -f -- "$engine.tmp" "$engine"
mv -f -- "$engine.src.tmp" "$engine.src"
