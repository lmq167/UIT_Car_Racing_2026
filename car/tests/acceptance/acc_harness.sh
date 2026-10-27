#!/usr/bin/env bash
# Phiếu làm chắc test §1–4: tự kiểm bộ chạy test, không thay hay giả sửa mã xe.
# Các nhánh âm phải bị phát hiện; dọn đúng nhóm kể cả cha đã thoát trước con.
source "$(dirname "$0")/acc_lib.sh"

TR="$T/check.csv"
printf '0,motor_speed,0\n0,servo_angle,7\n' > "$TR"
check 'Vet dung hop le' trace_tail "$TR" 7
check 'Lenh 0 khong duoc coi la da di chuyen' test "$(trace_moving "$TR"; echo $?)" -ne 0
for ((i=0; i<5; ++i)); do printf '%s,motor_speed,10\n' "$i" >> "$TR"; done
check 'Nhan duoc nam lenh tien' trace_moving "$TR"
printf '10,motor_speed,0\n10,servo_angle,7\n' >> "$TR"
check 'Cuoi motor 0 va servo trim' trace_tail "$TR" 7
printf '0,motor_speed,sai\n0,servo_angle,7\n' > "$TR"
check 'Vet sai kieu khong duoc dat' test "$(trace_tail "$TR" 7; echo $?)" -ne 0
for hz in 50 10; do
    printf 'control_hz: %s\n' "$hz" > "$T/timing.yaml"
    signal_timing_config "$T/timing.yaml"
    check "Chu ky lay tu control_hz=$hz" test "$ACC_CONTROL_PERIOD_MS" -eq "$((1000/hz))"
    check "Han bang hai chu ky khi control_hz=$hz" test "$ACC_STOP_BUDGET_MS" -eq "$((2000/hz))"
done

# Chế độ chẩn đoán riêng cho Windows: không phải toàn bộ test đã đăng ký trong CTest.
if [[ ${1:-} == --trace-only ]]; then acc_finish 'chan_doan_bo_doc_vet'; exit $?; fi
export ACC_SIGRUN=${1:?can acc_sigrun}
check 'Ca bien thoi gian va doc vet: loc vai ms, tre that, thieu dung, phep do tre' "$ACC_SIGRUN" --timing-self-test "$T/parser.csv"
export ACC_CONTROL_PERIOD_MS=20 ACC_STOP_BUDGET_MS=40

cat > "$T/fake_main" <<'EOF'
#!/usr/bin/env bash
trap 'printf "20,motor_speed,0\n20,servo_angle,7\n" >> "$UCR_FAKE_HW_TRACE"; exit 0' INT TERM
printf '0,motor_init,1\n' >> "$UCR_FAKE_HW_TRACE"
for ((i=0; i<5; ++i)); do printf '%s,motor_speed,%s\n%s,servo_angle,7\n' "$i" "${SPEED:-10}" "$i" >> "$UCR_FAKE_HW_TRACE"; done
while :; do sleep 0.02; done
EOF
chmod +x "$T/fake_main"
for sig in INT TERM; do
    export UCR_FAKE_HW_TRACE="$T/ready_$sig.csv"
    ACC_READY_FILE="$UCR_FAKE_HW_TRACE" ACC_READY_KIND=moving ACC_SIGNAL_TRACE="$UCR_FAKE_HW_TRACE" \
        sigrun "$sig" 0 "$T/out" "$T/fake_main"
    check 'Chi gui tin hieu khi co lenh tien' test "$READY" = 1
    check 'Main gia xu ly tin hieu chu dong' graceful_exit "$RES"
    check 'Lenh cuoi van dung' trace_tail "$UCR_FAKE_HW_TRACE" 7
done

mkdir -p "$T/logs/session_auto"
printf 'image_age_ms,cmd_speed\n0,0\n0,0\n0,0\n0,0\n700,0\n' > "$T/logs/session_auto/log.csv"
export UCR_FAKE_HW_TRACE="$T/ready_init.csv"
ACC_READY_FILE="$UCR_FAKE_HW_TRACE" ACC_READY_KIND=init ACC_READY_LOG_DIR="$T/logs" ACC_READY_STALE=1 \
    sigrun TERM 0 "$T/out" "$T/fake_main"
check 'Nhan init, du nam dong CSV va tuoi anh cu' test "$READY" = 1
check 'Duong cho log van xu ly tin hieu chu dong' graceful_exit "$RES"

# Không có chuyển động và chết sớm đều phải HỎNG, không 77 và không gửi tín hiệu đang kiểm.
for kind in no_motion no_log early_exit; do
    export UCR_FAKE_HW_TRACE="$T/$kind.csv"
    cmd=("$T/fake_main")
    [[ $kind == early_exit ]] && cmd=(bash -c 'exit 0')
    export ACC_READY_KIND=moving SPEED=0
    unset ACC_READY_LOG_DIR
    if [[ $kind == no_log ]]; then
        export ACC_READY_KIND=init SPEED=10 ACC_READY_LOG_DIR="$T/khong-co-log"
    fi
    t0=$(now_ms)
    ACC_READY_FILE="$UCR_FAKE_HW_TRACE" \
        acc_run "$ACC_SIGRUN" TERM 0 1000 "$T/out" -- "${cmd[@]}" > "$T/result"
    rc=$?
    check "[$kind] thieu tien de phai tra 1" test "$rc" -eq 1
    check "[$kind] bao ready=0" grep -qx 'ready=0' "$T/result"
    check "[$kind] dung cho trong <=11.5 s" test "$(( $(now_ms)-t0 ))" -le 11500
done
unset ACC_READY_KIND ACC_READY_LOG_DIR SPEED

cat > "$T/stage" <<'EOF'
#!/usr/bin/env bash
source "$1/acc_lib.sh"
acc_start bash -m -c 'acc_track_group "$$" || exit 3; sleep 60 & child=$!; acc_track_group "$child" || exit 3; echo "$child" > "$1/descendant"; exit 0' -- "$2"
wait "$ACC_PID"
wait_file "$2/descendant" 2 || exit 2
case "$3" in
    ok) exit 0 ;; fail) exit 1 ;;
    term) kill -TERM $$ ;;
    timeout) while :; do sleep 0.02; done ;;
esac
EOF
chmod +x "$T/stage"
not_running() {
    kill -0 "$1" 2>/dev/null || return 0
    # Con mồ côi có thể còn zombie trong container; zombie không còn thực thi lệnh.
    [[ -r /proc/$1/status ]] && grep -q '^State:.*Z' "/proc/$1/status"
}
wait_dead() {
    local i
    for ((i=0; i<100; ++i)); do not_running "$1" && return 0; sleep 0.02; done
    return 1
}
acc_start sleep 60
unrelated=$ACC_PID
for kind in ok fail term timeout; do
    d="$T/cleanup_$kind"; mkdir -p "$d"
    acc_run timeout --foreground -k 2s 2s bash "$T/stage" "$ACC_DIR" "$d" "$kind" > "$d/out" 2>&1
    rc=$?
    case "$kind" in ok) want=0 ;; fail) want=1 ;; term) want=143 ;; timeout) want=124 ;; esac
    check "[$kind] giu ma ket thuc ($rc)" test "$rc" -eq "$want"
    if check "[$kind] da tao con chau" test -s "$d/descendant"; then
        check "[$kind] EXIT don ca con chau khi cha da chet" wait_dead "$(cat "$d/descendant")"
    fi
    check "[$kind] nhom khac van song" kill -0 "$unrelated"
done
acc_reap "$unrelated"
acc_finish acc_shell_harness
