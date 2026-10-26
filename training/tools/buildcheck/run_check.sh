#!/usr/bin/env bash
# Một lượt ctest mỗi bản build; XML được đưa vào log để laptop đọc thống kê.
set -euo pipefail
[[ $# -ge 1 ]] || { echo 'Dùng: run_check.sh <thư-mục-ucr_car> [tên-image]' >&2; exit 2; }
src=$(cd "$1" && pwd) || exit 2
build_seconds=${UCR_BUILD_TIMEOUT:-900}
test_seconds=${UCR_TEST_TIMEOUT:-600}
for value in "$build_seconds" "$test_seconds"; do
    [[ $value =~ ^[1-9][0-9]*$ ]] || { echo 'Giới hạn thời gian không hợp lệ.' >&2; exit 2; }
done
container_seconds=${UCR_CONTAINER_TIMEOUT:-$((build_seconds + test_seconds + 60))}
[[ $container_seconds =~ ^[1-9][0-9]*$ ]] || { echo 'Giới hạn container không hợp lệ.' >&2; exit 2; }
kind=${UCR_NATIVE:-container}
if [[ $kind == container && ${UCR_IN_CONTAINER:-0} != 1 ]]; then
    image=${2:-ucr-jetson-buildcheck:opencv4.1.1}
    name="ucr-buildcheck-$$-$(date +%s)"
    cleanup() { timeout -k 2s 15s docker rm -f "$name" >/dev/null 2>&1 || true; }
    trap cleanup EXIT
    trap 'exit 3' HUP INT TERM
    echo 'UCR_STEP container'
    rc=0
    timeout -k 5s "${container_seconds}s" docker run --pull=never --rm --name "$name" \
        -v "$src:/src:ro" -v "$(cd "$(dirname "$0")" && pwd)/run_check.sh:/runner.sh:ro" \
        -e UCR_IN_CONTAINER=1 -e UCR_BUILD_TIMEOUT="$build_seconds" \
        -e UCR_TEST_TIMEOUT="$test_seconds" -e UCR_ONLY_ACCEPTANCE="${UCR_ONLY_ACCEPTANCE:-0}" \
        "$image" bash /runner.sh /src /tmp/build || rc=$?
    case "$rc" in 124|137) exit 3 ;; *) exit "$rc" ;; esac
fi
build=${2:-/tmp/build}
flags='-Wall -Wextra'
build_type=Release
labels=()
[[ ${UCR_ONLY_ACCEPTANCE:-0} != 1 ]] || labels=(-L acceptance)
case "$kind" in
    asan) build_type=Debug; flags+=' -fsanitize=address,undefined -fno-omit-frame-pointer'
          export ASAN_OPTIONS=halt_on_error=1:detect_leaks=1 UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 ;;
    tsan) build_type=Debug; flags+=' -fsanitize=thread -fno-omit-frame-pointer'
          export TSAN_OPTIONS=halt_on_error=1
          [[ ${UCR_ONLY_ACCEPTANCE:-0} == 1 ]] || labels=(-L 'threads|acceptance') ;;
    container) ;;
    *) echo 'Môi trường build không hợp lệ.' >&2; exit 2 ;;
esac
mkdir -p "$build" || exit 2
cd "$build" || exit 2
echo "UCR_STEP build $kind"
rc=0
timeout -k 5s "${build_seconds}s" bash -c '
    set -e
    gcc --version | head -1
    cmake --version | head -1
    cmake -DUCR_FAKE_HW=ON -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE="$2" \
        -DCMAKE_C_FLAGS="$3" -DCMAKE_CXX_FLAGS="$3" "$1"
    cmake --build . -- -j4
' bash "$src" "$build_type" "$flags" || rc=$?
case "$rc" in 0) ;; 124|137) echo 'HẾT GIỜ ở bước build'; exit 3 ;; *) exit 2 ;; esac
echo UCR_BUILD_OK
echo "UCR_STEP ctest $kind"
# Không dùng --test-dir hoặc -S/-B vì container còn CMake 3.10.
# Chỉ xoá kết quả trong thư mục build đã phân giải, tránh đọc XML lượt cũ.
rm -rf -- "$(pwd -P)/Testing" || exit 1
rc=0
timeout -k 5s "${test_seconds}s" ctest -T Test --no-compress-output --timeout 120 --output-on-failure "${labels[@]}" || rc=$?
if [[ -f Testing/TAG ]]; then
    tag=$(head -n 1 Testing/TAG)
    if [[ -f Testing/$tag/Test.xml ]]; then
        echo UCR_XML_BEGIN
        cat "Testing/$tag/Test.xml"
        echo
        echo UCR_XML_END
        # Ctest trả mã lỗi chung khi từng test hết giờ; đọc đúng trường XML.
        while IFS= read -r test_name; do
            echo "HẾT GIỜ ở bước $test_name"
            rc=3
        done < <(awk '
            BEGIN { RS="<" }
            /^Test([[:space:]>])/ { timedout=($0 ~ /Status=["\047][Tt]imeout["\047]/); name="" }
            /^Name>/ { name=substr($0, 6); sub(/[[:space:]]+$/, "", name) }
            /^NamedMeasurement[[:space:]]/ { exit_code=($0 ~ /name=["\047]Exit Code["\047]/) }
            /^Value>/ && exit_code {
                value=substr($0, 7)
                gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
                if (tolower(value) == "timeout") timedout=1
            }
            /^\/NamedMeasurement>/ { exit_code=0 }
            /^\/Test>/ && timedout { print name }
        ' "Testing/$tag/Test.xml")
    fi
fi
case "$rc" in 0) exit 0 ;; 3) exit 3 ;; 124|137) echo 'HẾT GIỜ ở bước ctest'; exit 3 ;; *) exit 1 ;; esac
