#!/usr/bin/env bash
# Tiến trình giả kiểm giao kèo kill/dừng của trình bao; không chứng minh phanh phần cứng.
set -euo pipefail
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd -P)
parent=$(CDPATH= cd -- "${TMPDIR:-/tmp}" && pwd -P)
folder=$(mktemp -d "$parent/ucr-smoke-wrapper.XXXXXXXX")
cleanup() {
    case "$folder" in "$parent"/ucr-smoke-wrapper.*) rm -rf -- "$folder" ;; esac
}
trap cleanup EXIT
cat > "$folder/main" <<'MAIN'
#!/usr/bin/env bash
printf '%s\n' "$@" > "$UCR_TEST_SMOKE_DIR/args"
while [[ $# -gt 0 ]]; do
    case "$1" in --smoke-kill-file) marker=$2; shift ;; esac
    shift
done
printf '%s\n' "$UCR_TEST_SMOKE_DIR/run" > "$marker"
while :; do sleep 0.01; done
MAIN
cat > "$folder/ucr_stop" <<'STOP'
#!/usr/bin/env bash
printf '%s\n' "$@" >> "$UCR_TEST_SMOKE_DIR/stops"
exit "$UCR_TEST_STOP_RC"
STOP
chmod +x "$folder/main" "$folder/ucr_stop"
export UCR_BUILD_DIR="$folder" UCR_TEST_SMOKE_DIR="$folder"
for expected in 0 1; do
    export UCR_TEST_STOP_RC="$expected"
    : > "$folder/marker"; : > "$folder/stops"
    rc=0
    timeout -k 1s 5s bash "$root/scripts/run.sh" --mode smoke --smoke-kill-file "$folder/marker" || rc=$?
    [[ $rc == "$expected" ]]
    [[ $(wc -l < "$folder/stops") == 6 ]]
    [[ $(sed -n '3p' "$folder/stops") == --smoke-report ]]
    [[ $(sed -n '4p' "$folder/stops") == "$folder/run/smoke.json" ]]
    [[ $(sed -n '5p' "$folder/stops") == --stop-since ]]
    [[ $(sed -n '6p' "$folder/stops") =~ ^[0-9]+[.][0-9]+$ ]]
done
export UCR_TEST_STOP_RC=0
output=$(timeout -k 1s 8s bash "$root/scripts/smoke_test.sh" --config "$folder/cau hinh.yaml")
grep -Fx -- --smoke-drop-camera "$folder/args" > /dev/null
command_line=$(printf '%s\n' "$output" | sed -n 's/^Smoke ĐẠT. Lệnh thu dữ liệu: //p')
[[ $command_line == UCR_BUILD_DIR=* || $command_line == 'sudo env UCR_BUILD_DIR='* ]]
[[ $command_line == *"--config $(printf '%q' "$folder/cau hinh.yaml") --mode collect" ]]
bash -n <<< "$command_line"
echo 'ĐẠT: kill -9 trong run.sh, gọi dừng đúng một lần, mốc trước kill, mã kết quả 0/1'
