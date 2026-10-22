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
# Lệnh cuối cùng đã chấp hành được lấy từ log.csv (cmd_speed, cmd_steer = lệnh sau lớp an toàn).
source "$(dirname "$0")/acc_lib.sh"
REAL_MAIN=${1:?can main}
REAL_STOP=${2:?can ucr_stop}
export ACC_SIGRUN=${3:?can acc_sigrun}

B="$T/build"
mkdir -p "$B"
cat > "$B/main" <<'EOF'
#!/usr/bin/env bash
echo $$ > "$ACC_RUN/mainpid"
exec "$ACC_REAL_MAIN" "$@"
EOF
cat > "$B/ucr_stop" <<'EOF'
#!/usr/bin/env bash
"$ACC_REAL_STOP" "$@"; rc=$?
echo "rc=$rc" >> "$ACC_RUN/stop_calls"
exit $rc
EOF
chmod +x "$B/main" "$B/ucr_stop"
export ACC_REAL_MAIN="$REAL_MAIN" ACC_REAL_STOP="$REAL_STOP"

n=0
newrun() { n=$((n + 1)); RUN="$T/run$n"; mkdir -p "$RUN/runs"; export ACC_RUN="$RUN"; }
calls() { if [[ -f $RUN/stop_calls ]]; then wc -l < "$RUN/stop_calls" | tr -d ' '; else echo 0; fi; }

# Chạy run.sh nền, chờ chương trình chạy ổn định 2 giây, gửi tín hiệu tới đích rồi đo thời gian kết thúc.
# scenario <nhãn> <đích:runsh|main|killmain> <tín-hiệu>
scenario() {
    local label=$1 target=$2 sig=$3
    newrun
    mkcfg "$RUN/c.yaml" "$RUN/runs"
    UCR_BUILD_DIR="$B" bash "$RUNSH" --config "$RUN/c.yaml" --mode auto >"$RUN/out" 2>&1 &
    local rpid=$!
    if ! wait_file "$RUN/mainpid" 10; then kill -KILL "$rpid" 2>/dev/null; fail "[$label] main khong khoi dong"; return; fi
    sleep 2
    local mpid; mpid=$(cat "$RUN/mainpid")
    check "[$label] main phai dang chay sau 2 s; dau ra: $(head -c 300 "$RUN/out")" kill -0 "$mpid"
    local t0; t0=$(now_ms)
    case $target in
        runsh) kill -s "$sig" "$rpid" ;;
        main|killmain) kill -s "$sig" "$mpid" ;;
    esac
    if wait_gone "$rpid" 10; then HUNG=0; else HUNG=1; kill -KILL "$mpid" "$rpid" 2>/dev/null; fi
    DT=$(( $(now_ms) - t0 ))
    wait "$rpid" 2>/dev/null; RC=$?
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
        check "[$label] lenh da chap hanh cuoi cung phai la 0: khong dong nao co cmd != 0" test "$(csv_nonzero_rows "$csv")" -eq 0
    fi
}

scenario run_sh_INT    runsh INT
scenario run_sh_TERM   runsh TERM
scenario main_KILL     killmain KILL

# Tín hiệu gửi thẳng vào main (không qua run.sh): phải tự dừng sạch
direct() {
    local label=$1 sig=$2
    newrun
    mkcfg "$RUN/c.yaml" "$RUN/runs"
    sigrun "$sig" 2500 "$RUN/out" "$REAL_MAIN" --config "$RUN/c.yaml" --mode auto
    check "[$label] main phai dang chay sau 2.5 s" test "$ALIVE" = 1
    check "[$label] main phai tu thoat trong <= 10 s sau $sig" test "$HUNG" = 0
    check "[$label] thoat trong <= 4000 ms (mat $STOP_MS ms)" test "$STOP_MS" -le 4000
    check "[$label] main phai tu xu ly $sig va thoat chu dong (exited:0/130/143), khong bi tin hieu giet; nhan $RES" graceful_exit "$RES"
    local csv; csv=$(find_csv "$RUN/runs" _auto)
    check "[$label] phai co log.csv" test -n "$csv"
    [[ -z $csv ]] || check "[$label] khong dong log nao co lenh != 0" test "$(csv_nonzero_rows "$csv")" -eq 0
}
direct main_truc_tiep_INT  INT
direct main_truc_tiep_TERM TERM

# Cấu hình xấu / tham số lạ đi qua run.sh: main từ chối (mã khác 0, sạch), run.sh vẫn gọi ucr_stop 1 lần, mã = mã của main
newrun
mkcfg "$RUN/c.yaml" "$RUN/runs" control_hz=5
UCR_BUILD_DIR="$B" timeout 30 bash "$RUNSH" --config "$RUN/c.yaml" --mode auto >"$RUN/out" 2>&1; RC=$?
check "control_hz=5 qua run.sh: thoat khac 0 va sach (1..127), nhan $RC" test "$RC" -ge 1 -a "$RC" -le 127
check "control_hz=5 qua run.sh: ucr_stop van duoc goi dung 1 lan (nhan $(calls))" test "$(calls)" -eq 1
check "control_hz=5 qua run.sh: khong tao thu muc chay" test -z "$(ls -A "$RUN/runs")"

newrun
mkcfg "$RUN/c.yaml" "$RUN/runs"
UCR_BUILD_DIR="$B" timeout 30 bash "$RUNSH" --config "$RUN/c.yaml" --mode auto --khong-co-tham-so-nay >"$RUN/out" 2>&1; RC=$?
check "tham so la qua run.sh: thoat khac 0 va sach, nhan $RC" test "$RC" -ge 1 -a "$RC" -le 127
check "tham so la qua run.sh: ucr_stop van duoc goi dung 1 lan (nhan $(calls))" test "$(calls)" -eq 1
check "tham so la qua run.sh: thong bao nhac ten tham so; dau ra: $(head -c 300 "$RUN/out")" grep -qF -- '--khong-co-tham-so-nay' "$RUN/out"

acc_finish acc_signals
