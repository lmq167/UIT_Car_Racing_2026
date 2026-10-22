#!/usr/bin/env bash
# Build bản giả lập (UCR_FAKE_HW=ON) và chạy ctest trong container kiểu xe (GCC 7, CMake 3.10, OpenCV 4.1.1).
# Mã nguồn được mount chỉ đọc; thư mục build nằm trong container và bị xóa khi container thoát,
# nên không để lại file nào trong thư mục mã nguồn.
# Dùng:  bash run_check.sh <thư-mục-ucr_car> [tên-image]
set -euo pipefail

if [[ $# -lt 1 ]]; then
    echo "Dùng: bash run_check.sh <thư-mục-ucr_car> [tên-image]" >&2
    exit 2
fi
src="$(cd "$1" && pwd)"
image="${2:-ucr-jetson-buildcheck:opencv4.1.1}"

docker run --rm -v "$src":/src:ro "$image" bash -c '
set -e
echo "== phiên bản"; gcc --version | head -1; cmake --version | head -1
echo "OpenCV_DIR=$OpenCV_DIR"; grep -m1 "OpenCV_VERSION " "$OpenCV_DIR/OpenCVConfig-version.cmake" || true
mkdir -p /tmp/build && cd /tmp/build
echo "== cấu hình"; cmake -DUCR_FAKE_HW=ON -DBUILD_TESTING=ON /src
echo "== biên dịch"; cmake --build . -- -j4
rc=0
echo "== ctest (tất cả test)"; ctest --output-on-failure || rc=$?
echo "== ctest (riêng nhãn acceptance: test chấp nhận)"; ctest -L acceptance --output-on-failure || rc=$?
exit $rc
'
