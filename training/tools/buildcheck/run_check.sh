#!/usr/bin/env bash
# Một lượt ctest mỗi bản build; XML được đưa vào log để laptop đọc thống kê.
set -euo pipefail
[[ $# -ge 1 ]] || { echo 'Dùng: run_check.sh <thư-mục-ucr_car> [tên-image]' >&2; exit 2; }
src=$(cd "$1" && pwd) || exit 2
build_seconds=${UCR_BUILD_TIMEOUT:-900}
test_seconds=${UCR_TEST_TIMEOUT:-1500}  # TSan full đo 880-950 s
for value in "$build_seconds" "$test_seconds"; do
    [[ $value =~ ^[1-9][0-9]*$ ]] || { echo 'Giới hạn thời gian không hợp lệ.' >&2; exit 2; }
done
container_seconds=${UCR_CONTAINER_TIMEOUT:-$((build_seconds + test_seconds + 60))}
[[ $container_seconds =~ ^[1-9][0-9]*$ ]] || { echo 'Giới hạn container không hợp lệ.' >&2; exit 2; }
kind=${UCR_NATIVE:-container}
[[ ${UCR_IN_CONTAINER:-0} == 1 ]] || echo "UCR_TREE ${UCR_SOURCE_TREE:-khong-ro}"
if [[ ${UCR_IN_CONTAINER:-0} == 1 ]]; then
    # Trong container: in memory.peak và oom_kill của cgroup container khi thoát (laptop đọc từ log).
    cg_report() {
        echo "UCR_CG_PEAK $(cat /sys/fs/cgroup/memory.peak 2>/dev/null || echo -)"
        echo "UCR_CG_OOM $(awk '$1 == "oom_kill" { print $2 }' /sys/fs/cgroup/memory.events 2>/dev/null || echo -)"
    }
    trap cg_report EXIT
fi
profile=${UCR_PROFILE:-full}
[[ $profile == quick || $profile == full || $profile == build ]] || { echo 'Chế độ không hợp lệ.' >&2; exit 2; }
[[ $profile != quick || $kind == asan || $kind == pycompile ]] || { echo 'Quick chỉ chạy ASan/UBSan và py_compile.' >&2; exit 2; }
jobs=${UCR_JOBS:-4}
[[ $kind != carbuild ]] || jobs=1  # xe luôn build -j1
[[ $jobs =~ ^[1-9][0-9]{0,3}$ ]] || { echo 'Số job không hợp lệ.' >&2; exit 2; }
(( jobs <= $(nproc) )) || { echo 'Số job vượt số nhân CPU.' >&2; exit 2; }
if [[ ( $kind == container || $kind == pycompile || $kind == carbuild ) && ${UCR_IN_CONTAINER:-0} != 1 ]]; then
    image=${2:-ucr-jetson-buildcheck:opencv4.1.1}
    name="ucr-buildcheck-$$-$(date +%s)"
    cleanup() { timeout -k 2s 15s docker rm -f "$name" >/dev/null 2>&1 || true; }
    trap cleanup EXIT
    trap 'exit 3' HUP INT TERM
    if [[ $kind == pycompile ]]; then
        echo 'UCR_STEP py_compile (container)'
        rc=0
        timeout -k 2s 30s docker run --pull=never --rm --name "$name" \
            --user "$(id -u):$(id -g)" -v "$src:/src:ro" "$image" bash -c '
                set -euo pipefail
                tmp=$(mktemp -d /tmp/ucr-pycompile.XXXXXX)
                trap '\''rm -rf -- "$tmp"'\'' EXIT
                mkdir "$tmp/tests"
                cd /src
                # py_compile vẫn ghi .pyc dù đặt PYTHONDONTWRITEBYTECODE; chỉ biên dịch bản sao.
                find tests -type f -name "*.py" -exec cp --parents -t "$tmp" -- {} +
                cd "$tmp"
                find tests -type f -name "*.py" -print0 | xargs -0 -r python3 -m py_compile
            ' || rc=$?
        case "$rc" in 0) echo UCR_PYCOMPILE_OK; exit 0 ;; 123) exit 1 ;; 124|137) exit 3 ;; *) exit 2 ;; esac
    fi
    # Giữ build ngoài nguồn; chạy bằng UID/GID hiện tại để không tạo file của root.
    build=${UCR_CONTAINER_BUILD_DIR:-$HOME/ucr_build_container}
    mkdir -p "$build" || exit 2
    build=$(cd "$build" && pwd)
    # carbuild: trần 2 GB như xe (AGENTS.md); container chạy test rộng hơn vì ctest song song.
    mem=${UCR_CONTAINER_MEM:-4g}
    [[ $kind != carbuild ]] || mem=2g
    [[ $mem =~ ^[1-9][0-9]*[gm]$ ]] || { echo 'UCR_CONTAINER_MEM không hợp lệ.' >&2; exit 2; }
    echo "UCR_STEP $kind"
    rc=0
    here=$(cd "$(dirname "$0")" && pwd)
    run_container() {  # $1 = pha: all | build | test; --memory-swap = --memory: không dùng swap
        timeout -k 5s "${container_seconds}s" docker run --pull=never --rm --name "$name" \
            --memory="$mem" --memory-swap="$mem" \
            --user "$(id -u):$(id -g)" -v "$build:/build" \
            -v "$src:/src:ro" -v "$here/run_check.sh:/runner.sh:ro" -v "$here/rss_wrap.py:/rss_wrap.py:ro" \
            -e UCR_IN_CONTAINER=1 -e UCR_NATIVE="$kind" -e UCR_BUILD_TIMEOUT="$build_seconds" \
            -e UCR_TEST_TIMEOUT="$test_seconds" -e UCR_PHASE="$1" \
            -e UCR_PROFILE="$profile" -e UCR_JOBS="$jobs" \
            "$image" bash /runner.sh /src /build
    }
    if [[ -n ${UCR_BUILD_LOCK:-} && $kind == container ]]; then
        # Biên dịch nối tiếp với ASan/TSan (khoá chung); ctest vẫn song song.
        ( flock 9; run_container build ) 9>"$UCR_BUILD_LOCK" || rc=$?
        (( rc != 0 )) || run_container test || rc=$?
    else
        run_container all || rc=$?
    fi
    case "$rc" in 124|137) exit 3 ;; *) exit "$rc" ;; esac
fi
build=${2:-/tmp/build}
flags='-Wall -Wextra'
build_type=Release
labels=()
fake_hw=ON; testing=ON; launcher=''
case "$kind" in
    asan) build_type=Debug; flags+=' -fsanitize=address,undefined -fno-omit-frame-pointer'
          export ASAN_OPTIONS=halt_on_error=1:detect_leaks=1 UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 ;;
    tsan) build_type=Debug; flags+=' -fsanitize=thread -fno-omit-frame-pointer'
          export TSAN_OPTIONS=halt_on_error=1
          labels=(-L threads) ;;
    container) ;;
    # Cờ như xe: không -O, không UCR_FAKE_HW, -funsigned-char (char của ARM không dấu), chỉ biên dịch; rss_wrap.py đo từng file.
    carbuild) build_type=''; flags='-funsigned-char'; fake_hw=OFF; testing=OFF; launcher='python3;/rss_wrap.py' ;;
    *) echo 'Môi trường build không hợp lệ.' >&2; exit 2 ;;
esac
[[ $profile != quick ]] || labels+=(-LE slow)
mkdir -p "$build" || exit 2
cd "$build" || exit 2
phase=${UCR_PHASE:-all}
if [[ $phase != test ]]; then
if [[ -n ${UCR_BUILD_LOCK:-} && ${UCR_IN_CONTAINER:-0} != 1 ]]; then
    exec 9>"$UCR_BUILD_LOCK"; flock 9  # configure+build các nhánh chạy nối tiếp
fi
build_jobs=$jobs
if [[ $kind == asan || $kind == tsan ]]; then
    # Số tiến trình biên dịch theo RAM còn trống lúc bắt đầu build (mặc định 3 GB mỗi tiến trình).
    per=${UCR_MEM_PER_JOB_GB:-3}
    mem_gb=$(awk '/^MemAvailable:/ { print int($2 / 1048576) }' /proc/meminfo)
    cap=$((mem_gb / per)); (( cap >= 1 )) || cap=1
    (( cap >= build_jobs )) || build_jobs=$cap
    echo "UCR_BUILD_JOBS $build_jobs (MemAvailable ${mem_gb} GB, ${per} GB/tiến trình, UCR_JOBS $jobs)"
fi
echo "UCR_STEP build $kind"
rc=0
timeout -k 5s "${build_seconds}s" bash -c '
    set -e
    gcc --version | head -1
    cmake --version | head -1
    cmake -DUCR_FAKE_HW="$5" -DBUILD_TESTING="$6" -DCMAKE_BUILD_TYPE="$2" \
        -DCMAKE_C_FLAGS="$3" -DCMAKE_CXX_FLAGS="$3" -DCMAKE_CXX_COMPILER_LAUNCHER="$7" "$1"
    cmake --build . -- -j"$4"
' bash "$src" "$build_type" "$flags" "$build_jobs" "$fake_hw" "$testing" "$launcher" || rc=$?
case "$rc" in 0) ;; 124|137) echo 'HẾT GIỜ ở bước build'; exit 3 ;; *) exit 2 ;; esac
echo UCR_BUILD_OK
# Chỉ nhả khóa build do chính runner lấy; fd 9 kế thừa là khóa toàn lượt của bộ điều phối.
if [[ -n ${UCR_BUILD_LOCK:-} && ${UCR_IN_CONTAINER:-0} != 1 ]]; then { flock -u 9; } 2>/dev/null || true; fi
fi
[[ $kind != carbuild ]] || { echo UCR_CARBUILD_OK; exit 0; }
[[ $phase != build ]] || exit 0
echo "UCR_STEP ctest $kind"
# Không dùng --test-dir hoặc -S/-B vì container còn CMake 3.10.
# Chỉ xoá kết quả trong thư mục build đã phân giải, tránh đọc XML lượt cũ.
rm -rf -- "$(pwd -P)/Testing" || exit 1
rc=0
timeout -k 5s "${test_seconds}s" ctest -j"$jobs" -T Test --no-compress-output --timeout 120 --output-on-failure "${labels[@]}" || rc=$?
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
