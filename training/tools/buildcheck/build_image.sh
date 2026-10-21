#!/usr/bin/env bash
# Dựng image kiểm tra biên dịch kiểu xe (chạy trên máy Ubuntu có Docker).
# Dùng:  bash build_image.sh [tên-image]     (mặc định ucr-jetson-buildcheck:opencv4.1.1)
# Số luồng make:  JOBS=12 bash build_image.sh
set -euo pipefail

cd "$(dirname "$0")"
image="${1:-ucr-jetson-buildcheck:opencv4.1.1}"
docker build --build-arg "JOBS=${JOBS:-8}" -t "$image" .
echo "Đã dựng image: $image"
