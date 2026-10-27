#!/usr/bin/env bash
# Test chấp nhận đầu-cuối (thoát / lỗi / tín hiệu và run.sh + ucr_stop): chương trình chính THẬT (bản UCR_FAKE_HW=ON) chạy qua scripts/run.sh với
# ucr_stop THẬT, nhận SIGINT / SIGTERM / kill -9, hoặc bị từ chối vì cấu hình/tham số xấu.
# Dùng: acc_signals.sh <main> <ucr_stop> <acc_sigrun>
#
# Yêu cầu gốc:
#  - Quy tắc an toàn: mọi chương trình điều khiển xe dừng motor + servo 0 khi thoát/lỗi; yêu cầu của run.sh: luôn gọi ucr_stop
#    đúng khi chương trình chính thoát vì bất kỳ lý do gì, thoát với mã của chương trình chính.
#  - Cấu hình xấu: "cấu hình xấu -> từ chối chạy, mã thoát khác 0, run.sh vẫn gọi ucr_stop".
#  - Tham số lạ: tham số lạ -> mã thoát khác 0 (run.sh vẫn gọi ucr_stop).
#  - docs/ucr_car.md: UCR_BUILD_DIR; ucr_stop bản giả lập thoát 0.
# Phiếu làm chắc test §1: bắt buộc đã có lệnh tiến trước tín hiệu; kiểm lệnh cuối trên vết phần cứng.
source "$(dirname "$0")/acc_lib.sh"
REAL_MAIN=${1:?can main}
REAL_STOP=${2:?can ucr_stop}
export ACC_SIGRUN=${3:?can acc_sigrun}
CASE=${4:?can ten ca}

B="$T/build"
mkdir -p "$B"
cat > "$B/main" <<'EOF'
#!/usr/bin/env bash
echo $$ > "$ACC_RUN/mainpid"
acc_track_group "$$" || exit 1
exec "$ACC_REAL_MAIN" "$@"
EOF
cat > "$B/ucr_stop" <<'EOF'
#!/usr/bin/env bash
acc_track_group "$$" || exit 1
UCR_FAKE_HW_TRACE="$ACC_RUN/stop_trace.csv" "$ACC_REAL_STOP" "$@"; rc=$?
echo "rc=$rc" >> "$ACC_RUN/stop_calls"
exit $rc
EOF
chmod +x "$B/main" "$B/ucr_stop"
export ACC_REAL_MAIN="$REAL_MAIN" ACC_REAL_STOP="$REAL_STOP"

n=0
newrun() { n=$((n + 1)); RUN="$T/run$n"; mkdir -p "$RUN/runs"; export ACC_RUN="$RUN"; export UCR_FAKE_HW_TRACE="$RUN/trace.csv"; }
calls() { if [[ -f $RUN/stop_calls ]]; then wc -l < "$RUN/stop_calls" | tr -d ' '; else echo 0; fi; }

signal_trace_checks() {
    local label=$1 trace=$2
    check "[$label] lenh motor cuoi = 0, servo cuoi = trim 0" trace_tail "$trace" 0
}

# Chờ đủ năm lệnh motor khác 0 (tối đa 10 s), rồi mới gửi tín hiệu.
# scenario <nhãn> <đích:runsh|killmain> <tín-hiệu>
scenario() {
    local label=$1 target=$2 sig=$3
    newrun
    mkcfg "$RUN/c.yaml" "$RUN/runs"
    signal_timing_config "$RUN/c.yaml"
    local observed="$UCR_FAKE_HW_TRACE" target_file=""
    if [[ $target == killmain ]]; then
        observed="$RUN/stop_trace.csv"; target_file="$RUN/mainpid"
        # docs/ucr_car.md mục Smoke: hạn 1 s tính từ trước kill, không cộng dung sai vào hạn này.
        ACC_STOP_BUDGET_MS=1000
    fi
    ACC_READY_FILE="$UCR_FAKE_HW_TRACE" ACC_READY_KIND=moving ACC_READY_LOG_DIR="$RUN/runs" \
        ACC_SIGNAL_TRACE="$observed" ACC_SIGNAL_TARGET_FILE="$target_file" UCR_BUILD_DIR="$B" \
        sigrun "$sig" 0 "$RUN/out" bash "$RUNSH" --config "$RUN/c.yaml" --mode auto
    RC=${RES#exited:}; [[ $RC =~ ^[0-9]+$ ]] || RC=255
    DT=$STOP_MS
    signal_trace_checks "$label" "$observed"
    check "[$label] dang chay truoc tin hieu" test "$ALIVE" = 1
    check "[$label] run.sh phai ket thuc trong <= 10 s sau $sig" test "$HUNG" = 0
    check "[$label] run.sh ket thuc trong <= 4000 ms (mat $DT ms)" test "$DT" -le 4000
    check "[$label] ucr_stop THAT goi dung 1 lan (nhan $(calls))" test "$(calls)" -eq 1
    check "[$label] ucr_stop ban gia lap phai thoat 0 (nhan: $(cat "$RUN/stop_calls" 2>/dev/null | tr '\n' ' '))" grep -qx 'rc=0' "$RUN/stop_calls"
    if [[ $sig == KILL ]]; then
        check "[$label] kill -9 main => run.sh thoat 137 (ma cua main), nhan $RC" test "$RC" -eq 137
    else
        check "[$label] sau $sig main xu ly va thoat => run.sh thoat 0 hoac 128+tin hieu (130/143), khong phai 137/139/134 (nhan $RC)" test "$RC" -eq 0 -o "$RC" -eq 130 -o "$RC" -eq 143
    fi
    local csv; csv=$(find_csv "$RUN/runs" _auto)
    check "[$label] phai co log.csv" test -n "$csv"
    if [[ -n $csv ]]; then
        check "[$label] log.csv co du lieu (>= 5 dong)" test "$(csv_rows "$csv")" -ge 5
        check "[$label] log phai co lenh khac 0 truoc tin hieu" test "$(csv_nonzero_rows "$csv")" -gt 0
    fi
}


# Tín hiệu gửi thẳng vào main (không qua run.sh): phải tự dừng sạch
direct() {
    local label=$1 sig=$2
    newrun
    mkcfg "$RUN/c.yaml" "$RUN/runs"
    signal_timing_config "$RUN/c.yaml"
    ACC_READY_FILE="$UCR_FAKE_HW_TRACE" ACC_READY_KIND=moving ACC_READY_LOG_DIR="$RUN/runs" ACC_SIGNAL_TRACE="$UCR_FAKE_HW_TRACE" \
        sigrun "$sig" 0 "$RUN/out" "$REAL_MAIN" --config "$RUN/c.yaml" --mode auto
    signal_trace_checks "$label" "$UCR_FAKE_HW_TRACE"
    check "[$label] main phai dang chay truoc tin hieu" test "$ALIVE" = 1
    check "[$label] main phai tu thoat trong <= 10 s sau $sig" test "$HUNG" = 0
    check "[$label] thoat trong <= 4000 ms (mat $STOP_MS ms)" test "$STOP_MS" -le 4000
    check "[$label] main phai tu xu ly $sig va thoat chu dong (exited:0/130/143), khong bi tin hieu giet; nhan $RES" graceful_exit "$RES"
    local csv; csv=$(find_csv "$RUN/runs" _auto)
    check "[$label] phai co log.csv" test -n "$csv"
    [[ -z $csv ]] || check "[$label] log phai co lenh khac 0" test "$(csv_nonzero_rows "$csv")" -gt 0
}


# Cấu hình xấu / tham số lạ đi qua run.sh: main từ chối (mã khác 0, sạch), run.sh vẫn gọi ucr_stop 1 lần, mã = mã của main
invalid_config() {
newrun
mkcfg "$RUN/c.yaml" "$RUN/runs" control_hz=5
UCR_BUILD_DIR="$B" acc_run timeout --foreground -k 2s 30 bash "$RUNSH" --config "$RUN/c.yaml" --mode auto >"$RUN/out" 2>&1; RC=$?
check "control_hz=5 qua run.sh: thoat khac 0 va sach (1..127), nhan $RC" test "$RC" -ge 1 -a "$RC" -le 127
check "control_hz=5 qua run.sh: ucr_stop van duoc goi dung 1 lan (nhan $(calls))" test "$(calls)" -eq 1
check "control_hz=5 qua run.sh: khong tao thu muc chay" test -z "$(ls -A "$RUN/runs")"

}

invalid_args() {
newrun
mkcfg "$RUN/c.yaml" "$RUN/runs"
UCR_BUILD_DIR="$B" acc_run timeout --foreground -k 2s 30 bash "$RUNSH" --config "$RUN/c.yaml" --mode auto --khong-co-tham-so-nay >"$RUN/out" 2>&1; RC=$?
check "tham so la qua run.sh: thoat khac 0 va sach, nhan $RC" test "$RC" -ge 1 -a "$RC" -le 127
check "tham so la qua run.sh: ucr_stop van duoc goi dung 1 lan (nhan $(calls))" test "$(calls)" -eq 1
check "tham so la qua run.sh: thong bao nhac ten tham so; dau ra: $(head -c 300 "$RUN/out")" grep -qF -- '--khong-co-tham-so-nay' "$RUN/out"

}
case "$CASE" in
    run_INT) scenario run_sh_INT runsh INT ;;
    run_TERM) scenario run_sh_TERM runsh TERM ;;
    kill_main) scenario main_KILL killmain KILL ;;
    direct_INT) direct main_truc_tiep_INT INT ;;
    direct_TERM) direct main_truc_tiep_TERM TERM ;;
    invalid_config) invalid_config ;;
    invalid_args) invalid_args ;;
    *) fail "Ten ca khong hop le: $CASE" ;;
esac
acc_finish "acc_signals_$CASE"
