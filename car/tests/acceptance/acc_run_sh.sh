#!/usr/bin/env bash
# Test chấp nhận run.sh + ucr_stop bằng chương trình chính GIẢ (script) và ucr_stop GIẢ ghi lại lệnh gọi.
# Dùng: acc_run_sh.sh   (không cần build; chỉ cần scripts/run.sh)
#
# Yêu cầu gốc:
#  - Yêu cầu của run.sh: "scripts/run.sh: chạy chương trình chính (tham số truyền qua), luôn gọi ucr_stop khi chương trình
#    chính thoát vì bất kỳ lý do gì (mã lỗi, crash, bị kill -9, nhận SIGINT/SIGTERM vào script) bằng trap + wait; sau
#    đó thoát với mã của chương trình chính."
#  - Yêu cầu dừng khẩn: "ucr_stop luôn được gọi đúng 1 lần".
#  - docs/ucr_car.md mục Xây dựng và chạy: UCR_BUILD_DIR chứa `main` và `ucr_stop`; --config tương đối tính từ nơi gọi
#    run.sh và chuyển thành tuyệt đối, cả main và ucr_stop nhận cùng đường dẫn; mặc định config.yaml ở gốc ucr_car;
#    chuyển tiếp SIGINT/SIGTERM, chờ con thực sự kết thúc rồi mới gọi ucr_stop; lỗi ucr_stop được in rõ nhưng không thay
#    mã thoát của chương trình chính; đường dẫn có dấu cách phải dùng được.
source "$(dirname "$0")/acc_lib.sh"

B="$T/build"
mkdir -p "$B"
cat > "$B/main" <<'EOF'
#!/usr/bin/env bash
echo $$ > "$ACC_RUN/mainpid"
acc_track_group "$$" || exit 1
printf '%s\n' "$@" > "$ACC_RUN/main_args"
case "${ACC_MAIN_MODE:-exit0}" in
    exit0) exit 0 ;;
    exit3) exit 3 ;;
    exit127) exit 127 ;;
    self9) kill -9 $$ ;;
    wait)
        trap 'echo TERM >> "$ACC_RUN/sig"; exit 7' TERM
        trap 'echo INT >> "$ACC_RUN/sig"; exit 9' INT
        : > "$ACC_RUN/ready"
        while :; do sleep 0.1; done ;;
esac
EOF
cat > "$B/ucr_stop" <<'EOF'
#!/usr/bin/env bash
acc_track_group "$$" || exit 1
mp=$(cat "$ACC_RUN/mainpid" 2>/dev/null)
if [ -n "$mp" ] && kill -0 "$mp" 2>/dev/null; then st=alive; else st=dead; fi
echo "$st" >> "$ACC_RUN/stop_calls"
printf '%s\n' "$@" > "$ACC_RUN/stop_args"
exit "${ACC_STOP_RC:-0}"
EOF
chmod +x "$B/main" "$B/ucr_stop"

n=0
newrun() { n=$((n + 1)); RUN="$T/run$n"; mkdir -p "$RUN"; export ACC_RUN="$RUN"; }
calls() { if [[ -f $RUN/stop_calls ]]; then wc -l < "$RUN/stop_calls" | tr -d ' '; else echo 0; fi; }
stop_state() { head -n1 "$RUN/stop_calls" 2>/dev/null; }

# run_sync <chế-độ-main> [args-run.sh...]  -> RC
run_sync() {
    local mode=$1; shift
    newrun
    ACC_MAIN_MODE=$mode UCR_BUILD_DIR="$B" acc_run timeout --foreground -k 2s 30 bash "$RUNSH" "$@" >"$RUN/out" 2>&1
    RC=$?
}

run_sync exit0 --mode auto
check "main thoat 0 => run.sh thoat 0 (nhan $RC)" test "$RC" -eq 0
check "main thoat 0 => ucr_stop goi dung 1 lan (nhan $(calls))" test "$(calls)" -eq 1
check "ucr_stop chi duoc chay SAU khi main ket thuc (thay: $(stop_state))" test "$(stop_state)" = dead

run_sync exit3 --mode auto
check "main thoat 3 => run.sh thoat 3 (nhan $RC)" test "$RC" -eq 3
check "main thoat 3 => ucr_stop goi dung 1 lan (nhan $(calls))" test "$(calls)" -eq 1

run_sync exit127 --mode auto
check "main thoat 127 => run.sh thoat 127 (nhan $RC)" test "$RC" -eq 127
check "main thoat 127 => ucr_stop goi dung 1 lan" test "$(calls)" -eq 1

run_sync self9 --mode auto
check "main tu SIGKILL => run.sh thoat 137 (ma cua chuong trinh chinh), nhan $RC" test "$RC" -eq 137
check "main tu SIGKILL => ucr_stop goi dung 1 lan (nhan $(calls))" test "$(calls)" -eq 1
check "main tu SIGKILL: ucr_stop chay khi main da chet (thay: $(stop_state))" test "$(stop_state)" = dead

# kill -9 từ bên ngoài vào chương trình chính khi run.sh đang chờ
newrun
ACC_MAIN_MODE=wait UCR_BUILD_DIR="$B" acc_start bash "$RUNSH" --mode auto >"$RUN/out" 2>&1 || exit 1
rpid=$ACC_PID
if wait_file "$RUN/ready" 10; then
    kill -KILL "$(cat "$RUN/mainpid")" 2>/dev/null
    if wait_gone "$rpid" 10; then wait "$rpid"; RC=$?; else RC=999; fi
    acc_reap "$rpid"
    check "kill -9 main tu ngoai => run.sh thoat 137, nhan $RC" test "$RC" -eq 137
    check "kill -9 main tu ngoai => ucr_stop goi dung 1 lan (nhan $(calls))" test "$(calls)" -eq 1
    check "kill -9 main tu ngoai: ucr_stop chay khi main da chet (thay: $(stop_state))" test "$(stop_state)" = dead
else
    acc_reap "$rpid"
    fail "main gia (che do wait) khong bao san sang"
fi

# SIGINT / SIGTERM vào run.sh: chuyển tiếp cho main, chờ main kết thúc, gọi ucr_stop đúng 1 lần, thoát bằng mã của main
for sig in INT TERM; do
    newrun
    ACC_MAIN_MODE=wait UCR_BUILD_DIR="$B" acc_start bash "$RUNSH" --mode auto >"$RUN/out" 2>&1 || exit 1
    rpid=$ACC_PID
    if wait_file "$RUN/ready" 10; then
        kill -s "$sig" "$rpid"
        if wait_gone "$rpid" 10; then wait "$rpid"; RC=$?; else RC=999; fi
        acc_reap "$rpid"
        want=$([[ $sig == INT ]] && echo 9 || echo 7)
        check "$sig vao run.sh: main phai nhan duoc $sig (chuyen tiep); sig file: $(cat "$RUN/sig" 2>/dev/null)" grep -qx "$sig" "$RUN/sig"
        check "$sig vao run.sh: run.sh thoat voi ma cua main ($want), nhan $RC" test "$RC" -eq "$want"
        check "$sig vao run.sh: ucr_stop goi dung 1 lan (nhan $(calls))" test "$(calls)" -eq 1
        check "$sig vao run.sh: ucr_stop chay SAU khi main ket thuc (thay: $(stop_state))" test "$(stop_state)" = dead
    else
        acc_reap "$rpid"
        fail "main gia (che do wait) khong bao san sang"
    fi
done

# Hai tín hiệu liên tiếp vẫn chỉ một lần ucr_stop
newrun
ACC_MAIN_MODE=wait UCR_BUILD_DIR="$B" acc_start bash "$RUNSH" --mode auto >"$RUN/out" 2>&1 || exit 1
rpid=$ACC_PID
if wait_file "$RUN/ready" 10; then
    kill -s INT "$rpid"; kill -s TERM "$rpid" 2>/dev/null
    if wait_gone "$rpid" 10; then wait "$rpid"; RC=$?; else RC=999; fi
    acc_reap "$rpid"
    check "INT roi TERM lien tiep: run.sh phai ket thuc (khong treo), nhan $RC" test "$RC" -ne 999
    check "INT roi TERM lien tiep: ucr_stop van dung 1 lan (nhan $(calls))" test "$(calls)" -eq 1
else
    acc_reap "$rpid"; fail "main gia khong san sang"
fi

# ucr_stop lỗi: vẫn gọi 1 lần, không đổi mã thoát của main, có in thông báo
newrun
ACC_STOP_RC=5 ACC_MAIN_MODE=exit0 UCR_BUILD_DIR="$B" acc_run timeout --foreground -k 2s 30 bash "$RUNSH" --mode auto >"$RUN/out" 2>&1; RC=$?
check "ucr_stop loi (5) + main 0 => run.sh van thoat 0 (docs: loi dung khan duoc in nhung khong doi ma), nhan $RC" test "$RC" -eq 0
check "ucr_stop loi: phai in thong bao ro (dau ra khong rong)" test -s "$RUN/out"
check "ucr_stop loi: van chi goi 1 lan" test "$(calls)" -eq 1
newrun
ACC_STOP_RC=5 ACC_MAIN_MODE=exit3 UCR_BUILD_DIR="$B" acc_run timeout --foreground -k 2s 30 bash "$RUNSH" --mode auto >"$RUN/out" 2>&1; RC=$?
check "ucr_stop loi (5) + main 3 => run.sh thoat 3, nhan $RC" test "$RC" -eq 3

# Không có chương trình chính (không chạy được): vẫn phải gọi ucr_stop 1 lần và báo lỗi
mkdir -p "$T/build_nomain"
cp "$B/ucr_stop" "$T/build_nomain/ucr_stop"
newrun
UCR_BUILD_DIR="$T/build_nomain" acc_run timeout --foreground -k 2s 30 bash "$RUNSH" --mode auto >"$RUN/out" 2>&1; RC=$?
check "khong co main => run.sh phai thoat khac 0, nhan $RC" test "$RC" -ne 0
check "khong co main => ucr_stop van duoc goi dung 1 lan (nhan $(calls))" test "$(calls)" -eq 1

# Tham số: --config tương đối (có dấu cách) -> tuyệt đối, truyền cho cả main lẫn ucr_stop; các tham số khác giữ nguyên
mkdir -p "$T/work/cau hinh"
cp "$ROOT/config.yaml" "$T/work/cau hinh/xe.yaml"
newrun
(cd "$T/work" && ACC_MAIN_MODE=exit0 UCR_BUILD_DIR="$B" acc_run timeout --foreground -k 2s 30 bash "$RUNSH" --config "cau hinh/xe.yaml" --mode auto >"$RUN/out" 2>&1); RC=$?
want="$T/work/cau hinh/xe.yaml"
check "main nhan duong dan config tuyet doi co dau cach ('$want'); nhan: $(tr '\n' '|' < "$RUN/main_args")" \
      grep -qxF -- "$want" "$RUN/main_args"
check "main nhan '--config' ngay truoc duong dan" bash -c 'grep -x -A1 -- "--config" "$1" | grep -qxF -- "$2"' _ "$RUN/main_args" "$want"
check "main nhan '--mode' roi 'auto' (tham so duoc truyen qua)" bash -c 'grep -x -A1 -- "--mode" "$1" | grep -qx auto' _ "$RUN/main_args"
check "ucr_stop nhan cung duong dan config; nhan: $(tr '\n' '|' < "$RUN/stop_args")" grep -qxF -- "$want" "$RUN/stop_args"
check "ucr_stop nhan '--config' ngay truoc duong dan" bash -c 'grep -x -A1 -- "--config" "$1" | grep -qxF -- "$2"' _ "$RUN/stop_args" "$want"

# Không có --config: mặc định config.yaml ở gốc ucr_car, truyền cho cả hai
newrun
(cd "$T/work" && ACC_MAIN_MODE=exit0 UCR_BUILD_DIR="$B" acc_run timeout --foreground -k 2s 30 bash "$RUNSH" --mode auto >"$RUN/out" 2>&1)
check "khong --config: main nhan config.yaml o goc ucr_car ($ROOT/config.yaml); nhan: $(tr '\n' '|' < "$RUN/main_args")" \
      bash -c 'grep -x -A1 -- "--config" "$1" | grep -qxF -- "$2"' _ "$RUN/main_args" "$ROOT/config.yaml"
check "khong --config: ucr_stop nhan cung config mac dinh" bash -c 'grep -x -A1 -- "--config" "$1" | grep -qxF -- "$2"' _ "$RUN/stop_args" "$ROOT/config.yaml"

# File phải có xuống dòng kiểu Unix (yêu cầu của run.sh) và quyền chạy được bằng bash
no_cr() { [[ -z $(tr -d -c '\r' < "$1") ]]; }
check "scripts/run.sh khong duoc co ky tu CR (phai LF)" no_cr "$RUNSH"
check ".gitattributes phai co '*.sh text eol=lf'" grep -qxF '*.sh text eol=lf' "$ROOT/.gitattributes"

acc_finish acc_run_sh
