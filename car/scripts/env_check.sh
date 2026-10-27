#!/usr/bin/env bash
# Chỉ đọc môi trường, không cài gói, không thay chế độ nguồn.
set -u
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd -P)
build=${UCR_BUILD_DIR:-"$root/build"}
check() { printf '\n[%s]\n' "$1"; shift; timeout -k 1s 8s "$@" 2>&1 || echo 'Chưa đọc được / không có công cụ'; }
check date date --iso-8601=seconds
check JetPack cat /etc/nv_tegra_release
check TensorRT dpkg-query -W 'libnvinfer*' 'tensorrt*' 'nvidia-jetpack'
check CUDA nvcc --version
check Python python3 --version
check OpenCV-C++ "${1:-$build/main}" --env-info
check OpenCV-Python python3 -c 'import cv2; print(cv2.__version__); print("\n".join(s for s in cv2.getBuildInformation().splitlines() if "GStreamer" in s))'
check trtexec /usr/src/tensorrt/bin/trtexec --help | sed -n '1,8p'
check disk df -h .
check nvpmodel nvpmodel -q
