#!/usr/bin/env python3
"""T05h/C1: dừng qua FIFO nghẽn, lỗi trình bao và khoá bước 5; Python 3.6."""
import errno
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

sys.dont_write_bytecode = True
from acc_calib_limits import LimitsSuite, near, observed_timing
from acc_vehicle_cli import (collect_metadata, held_at, kill_group, log_ready,
                             log_rows, poll, read, require, run, tail, trace)

LOCKED = "Bước 5 đang khoá cho tới khi kiểm chế độ vị trí trên xe"
STOP_ERROR = "LOI: ucr_stop khong xac nhan dung duoc thiet bi"
GAP = (5500, 6700)
TOF_AT = 5900


def no_devices(folder):
    require(not trace(folder / "trace.csv"), "C1: không được vào Backend::Init/motor_init/UART/lệnh thiết bị")
    require(not (folder / "runs").exists(), "C1: không được tạo runs/")


def rejection(code, stdout, stderr):
    require(code == 2, "C1: cần exit 2, có {}".format(code))
    require(LOCKED in stderr, "C1: thiếu nguyên văn câu khoá trên stderr")
    require(LOCKED not in stdout, "C1: thông báo khoá phải ở stderr")


def reject_position(suite, step="position"):
    # Phân xử 28/10 sáng C1: áp dụng cho cả ca position cũ, trước mọi thiết bị/runs.
    folder, config, env = LimitsSuite.setup(suite)
    require(not (folder / "runs").exists(), "Bộ kiểm C1 phải bắt đầu khi chưa có runs/")
    code, _ = run([suite.main, "--config", config, "--mode", "calib", "--step", step],
                  folder, env, timeout=8)
    rejection(code, read(folder / "output.txt"), read(folder / "error.txt"))
    no_devices(folder)


def deadline_check(rows, marker, stop_by, end, not_before=0):
    # H1: chỉ xét vết trước tháo nghẽn/dọn tiến trình; phải đang chạy tại lúc chặn.
    require(rows and end >= stop_by+500, "Thiếu cửa sổ giữ dừng khi nghẽn")
    require(abs(held_at(rows, "motor_speed", marker)) > 0.1,
            "Tiền đề: phải còn chạy khi encoder bắt đầu nghẽn")
    zeros = [ms for ms, kind, value in rows
             if kind == "motor_speed" and ms > marker and near(float(value), 0)]
    require(zeros and zeros[0] <= stop_by,
            "Motor 0 trễ/không gửi: hạn {}, có {}".format(stop_by, zeros[:1]))
    require(zeros[0] >= not_before, "Motor đã dừng trước sự cố; không chứng minh được đường phanh cần kiểm")
    # Chưa hết tuổi encoder 500 ms: không nhận dừng do encoder cũ làm phanh ảnh/ToF.
    require(zeros[0] < marker+400, "Dừng quá sát hết tuổi encoder, không phân giải được nguyên nhân")
    # Luồng có thể kẹt logger SAU khi đã gửi dừng/nhả khoá: không đòi vết mới mỗi tick.
    for kind, value in (("motor_speed", 0), ("servo_angle", 7)):
        require(near(held_at(rows, kind, stop_by), value), "Sai lệnh giữ tại hạn: " + kind)
        require(all(near(float(v), value) for ms, k, v in rows if k == kind and stop_by <= ms <= end),
                "Lệnh an toàn bị ghi đè trong cửa sổ nghẽn: " + kind)


class StopSuite(LimitsSuite):
    def check_config(self):
        # Hợp đồng T05 §4 và T05h C1: kiểm cấu hình không khởi tạo thiết bị.
        for bad in (False, True):
            folder, config, env = self.setup({"image_timeout_ms": "0"} if bad else {})
            code, _ = run([self.main, "--check-config", "--config", config], folder, env, timeout=8)
            out, err = read(folder / "output.txt"), read(folder / "error.txt")
            require(code == (2 if bad else 0), "--check-config đổi mã thoát")
            if bad:
                require("image_timeout_ms" in err and "khoảng hợp lệ" in err,
                        "--check-config sai phải nêu khoá/khoảng")
            else:
                require(out == "CAU_HINH_OK: {}\n".format(config), "Sai dòng CAU_HINH_OK")
            require(LOCKED not in err, "C1 không áp lên --check-config")
            no_devices(folder)

    def other_step(self, step):
        # T05h C1: kiểm số 1–4,6–10 qua giao diện đang chạy, không chỉ kiểm parser.
        folder, config, env = self.setup()
        def observe(process):
            require(poll(lambda: process.poll() is not None or log_ready(folder, "calib"), 8),
                    "Bước {} không tới log/cảm biến".format(step))
            require(process.poll() is None and log_ready(folder, "calib"),
                    "Bước {} bị từ chối trước giao diện".format(step))
            process.send_signal(signal.SIGTERM)
        code, output = run([self.main, "--config", config, "--mode", "calib", "--step", step],
                           folder, env, None if step == "1" else observe, timeout=15 if step == "1" else 5)
        require(LOCKED not in output and code in (0, 143), "C1 chặn nhầm bước " + step)
        if step == "1":
            no_devices(folder)
            require(output.strip(), "Bước 1 phải in thông tin môi trường")
        else:
            rows = trace(folder / "trace.csv")
            require(sum(k == "motor_init" for _, k, _ in rows) == 1, "Motor init đúng một lần")
            require(all(near(float(v), 0) for _, k, v in rows if k == "motor_speed"),
                    "Chưa xác nhận y: các bước phải đứng yên")
            tail(rows)

    def wrapper_failure(self):
        # T05h §3: chương trình giả chỉ điều khiển exit; chạy run.sh thật qua UCR_BUILD_DIR.
        for main_code, stop_code in ((0, 9), (7, 9), (137, 9), (0, 126), (0, 0), (7, 0)):
            folder, _, env = self.setup()
            build = folder / "build"
            build.mkdir()
            for name, code in (("main", main_code), ("ucr_stop", stop_code)):
                program = build / name
                interpreter = "/khong-co-trinh-thong-dich" if name == "ucr_stop" and stop_code == 126 else "/bin/sh"
                program.write_text("#!{}\nprintf '%s\\n' {} >> \"$ACC_CALLS\"\nexit {}\n".format(interpreter, name, code),
                                   encoding="ascii")
                program.chmod(0o755)
            env["UCR_BUILD_DIR"] = build.as_posix()
            env["ACC_CALLS"] = (folder / "calls").as_posix()
            code, _ = run(["bash", (self.root / "scripts/run.sh").as_posix(), "--mode", "manual"],
                          folder, env, timeout=8)
            expected = main_code if main_code else (1 if stop_code else 0)
            require(code == expected, "run.sh đổi mã: main={} stop={} cần {} có {}".format(
                main_code, stop_code, expected, code))
            require((folder / "calls").exists(), "Không chạy được chương trình giả: " + read(folder / "error.txt")[-500:])
            require(read(folder / "calls").splitlines().count("main") == 1, "Phải chạy main giả")
            if stop_code != 126:
                require(read(folder / "calls").splitlines().count("ucr_stop") == 1,
                        "Phải gọi ucr_stop đúng một lần")
            err = read(folder / "error.txt")
            require((STOP_ERROR in err) == bool(stop_code), "Sai/thiếu LOI trên stderr: " + err[-500:])

    def blocked_stdout(self, mode, kind):
        # T05h §4: FIFO đầy không đọc trong cửa sổ đo; lỗi encoder tại API Position.
        require(os.name == "posix", "FIFO cần Linux; không bỏ qua ca")
        folder, config, env = self.setup({"calib_max_duration_s": "30", "keyboard_speed_step_cms": "10"})
        env["UCR_FAKE_KEYS"] = "4000:{}".format("y" if mode == "calib" else "w")
        if kind == "image":
            env["UCR_FAKE_CAMERA_GAPS"] = "{}:{}".format(*GAP)
        else:
            path = folder / "tof.txt"
            path.write_text("0 2000 2000 2000 2000\n{} 100 2000 2000 2000\n".format(TOF_AT), encoding="ascii")
            env["UCR_FAKE_TOF_SCRIPT"] = str(path)
        gate = folder / "encoder.gate"
        env["ACC_T05H_ENCODER_GATE"] = str(gate)
        fifo = folder / "stdout.fifo"
        os.mkfifo(str(fifo))
        reader = os.open(str(fifo), os.O_RDONLY | os.O_NONBLOCK)
        writer = os.open(str(fifo), os.O_WRONLY | os.O_NONBLOCK)
        process = None
        total = 0
        def drain():
            nonlocal total
            while True:
                try:
                    data = os.read(reader, 65536)
                    if not data:
                        break
                    total += len(data)
                except BlockingIOError:
                    break
        def wait(predicate, seconds, message, draining=False):
            def ready():
                if draining:
                    drain()
                return predicate() or process.poll() is not None
            require(poll(ready, seconds) and process.poll() is None and predicate(), message)
        command = (self.command(config) if mode == "calib" else
                   [self.main, "--config", config, "--mode", mode])
        with fifo.open("wb", buffering=0) as output, (folder / "error.txt").open("wb") as error:
            try:
                process = subprocess.Popen([str(x) for x in command], cwd=str(folder), env=env,
                                           stdin=subprocess.PIPE, stdout=output, stderr=error,
                                           start_new_session=True)
                if mode == "collect":
                    collect_metadata(process)
                wait(lambda: any(k == "motor_speed" and near(float(v), 10)
                                 for _, k, v in trace(folder / "trace.csv", live=True)),
                     8, "Tiền đề: chưa có chuyển động trước chặn", True)
                rows = trace(folder / "trace.csv", live=True)
                require(log_ready(folder, mode), "Ảnh/ToF phải mới khi bắt đầu chạy")
                cycle, frame, _ = observed_timing(rows, log_rows(folder, mode))
                # Đầu ca đo đủ chu kỳ trước 3300 ms, không lấy nhịp lúc stdout đã nghẽn.
                stop_by = (GAP[0]+500 if kind == "image" else TOF_AT+20)+cycle+100+100
                if kind == "image":
                    require(stop_by < GAP[1], "Tiền đề: stop_by < gap.second")
                # Chặn sát trước phanh để mẫu encoder trước nghẽn vẫn còn hiệu lực.
                block_at = 5700 if kind == "image" else 5750
                wait(lambda: bool(trace(folder / "trace.csv", live=True)) and
                     trace(folder / "trace.csv", live=True)[-1][0] >= block_at,
                     4, "Không tới mốc chặn", True)
                # Chỉ descriptor của bộ nhồi có O_NONBLOCK; stdout của con vẫn chặn bình thường.
                filled = 0
                while True:
                    try:
                        filled += os.write(writer, b"F"*4096)
                    except BlockingIOError as failure:
                        require(failure.errno in (errno.EAGAIN, errno.EWOULDBLOCK), "FIFO lỗi khác đầy")
                        break
                require(filled > 0 and total > 0, "Tiền đề: FIFO có stdout thật và đã nhồi đầy")
                gate.write_text("1", encoding="ascii")
                wait(lambda: any(k == "encoder_no_reply" for _, k, _ in trace(folder / "trace.csv", live=True)),
                     1, "Thiếu điểm tiêm encoder; không được tính ĐẠT")
                rows = trace(folder / "trace.csv", live=True)
                marker = next(ms for ms, k, _ in rows if k == "encoder_no_reply")
                require(block_at <= marker < block_at+150, "Điểm tiêm encoder trễ, sai thiết lập")
                end = stop_by+800
                print("{} {}: cycle={:.1f} frame={:.1f} FIFO={} encoder={} stop_by={} end={:.1f}".format(
                    mode, kind, cycle, frame, filled, marker, stop_by, end), flush=True)
                until = time.monotonic()+(end-marker)/1000.0
                def observed():
                    current = trace(folder / "trace.csv", live=True)
                    require(not any(k == "encoder_logger_returned" for _, k, _ in current),
                            "Tiền đề: logger phải nghẽn trong toàn bộ cửa sổ")
                    return time.monotonic() >= until
                wait(observed, 4, "Không giữ được cửa sổ nghẽn tới hạn quan sát")
                rows = trace(folder / "trace.csv", live=True)
                require(not any(k == "encoder_logger_returned" for _, k, _ in rows),
                        "Tiền đề: logger phải còn nghẽn tới cuối cửa sổ")
                not_before = GAP[0]+500-frame-100 if kind == "image" else TOF_AT
                deadline_check(rows, marker, stop_by, end, not_before)
                # Chỉ sau phán xét mới tháo FIFO, gửi TERM, chờ và thu hồi cả nhóm.
                process.send_signal(signal.SIGTERM)
                require(poll(lambda: (drain() or process.poll() is not None), 5), "Không thoát sau tháo nghẽn/TERM")
                require(process.returncode in (0, 143), "Thoát lỗi sau TERM")
                tail(trace(folder / "trace.csv"))
            finally:
                if process is not None:
                    kill_group(process)
                    process.stdin.close()
                os.close(writer)
                os.close(reader)


def self_test():
    # Skill §3: bộ phán xét phải bắt trễ, thiếu chuyển động và thiếu câu lỗi.
    good = [(5700, "motor_speed", "10"), (5750, "encoder_no_reply", "1"),
            (6000, "motor_speed", "0"), (6000, "servo_angle", "7"), (7000, "servo_angle", "7")]
    deadline_check(good, 5750, 6220, 7000)
    rejection(2, "", LOCKED)
    wrong = [lambda: deadline_check(good, 5750, 5999, 7000),
             lambda: deadline_check([(t, k, "0" if k == "motor_speed" else v) for t, k, v in good], 5750, 6220, 7000),
             lambda: deadline_check([row for row in good if row[1] != "servo_angle"], 5750, 6220, 7000),
             lambda: deadline_check(good+[(6500, "motor_speed", "10")], 5750, 6220, 7000),
             lambda: deadline_check(good+[(6500, "servo_angle", "27")], 5750, 6220, 7000),
             lambda: deadline_check(good, 5750, 6220, 7000, 6100),
             lambda: rejection(0, "", LOCKED), lambda: rejection(2, LOCKED, ""),
             lambda: rejection(2, "", "Bước 5 đang khoá")]
    for check in wrong:
        try:
            check()
        except AssertionError:
            continue
        raise AssertionError("Bộ kiểm bỏ lọt mẫu sai")
    from unittest.mock import patch
    from acc_vehicle_cli import Suite
    from acc_review_fixes import ReviewSuite
    from acc_safety_latch import LatchSuite
    # Ba đường gọi cũ đều phải tới kiểm C1, không còn khởi chạy lượt position.
    sys.modules["acc_stop_path"] = sys.modules[__name__]
    with tempfile.TemporaryDirectory(prefix="acc_t05h_self_") as name:
        folder = Path(name)
        def fake_run(command, where, env, timeout):
            require(command[-1] in ("5", "position") and "--step" in command, "C1 sai đối số")
            (where / "output.txt").write_text("", encoding="utf-8")
            (where / "error.txt").write_text(LOCKED, encoding="utf-8")
            return 2, LOCKED
        with patch.object(sys.modules[__name__], "run", side_effect=fake_run) as call:
            for index in range(3):
                (folder / str(index)).mkdir()
            Suite(".", "main", "stop", folder / "0").calib("position", signal.SIGTERM)
            ReviewSuite(".", "main", "stop", folder / "1").repeated("deadline", "position")
            LatchSuite(".", "main", "stop", folder / "2").limit("position")
            require(call.call_count == 3, "Thiếu đường gọi position cũ")
        for bad in ("trace", "runs"):
            where = folder / ("bad_"+bad)
            where.mkdir()
            if bad == "trace":
                (where / "trace.csv").write_text("0,motor_init,1\n", encoding="ascii")
            else:
                (where / "runs").mkdir()
            try:
                no_devices(where)
            except AssertionError:
                continue
            raise AssertionError("C1 bỏ lọt khởi tạo/ghi runs")
    print("ĐẠT tự kiểm 11 mẫu sai và 3 đường position cũ; chưa kiểm sản phẩm")


def main():
    require(len(sys.argv) >= 2, "Cần tên ca")
    scenario = sys.argv[1]
    if scenario == "self-test":
        self_test()
        return
    require(len(sys.argv) == 5, "Cần scenario root main ucr_stop")
    with tempfile.TemporaryDirectory(prefix="acc_t05h_") as folder:
        suite = StopSuite(sys.argv[2], sys.argv[3], sys.argv[4], folder)
        if scenario != "wrapper_stop_failure":
            suite.fake_only()
        if scenario in ("position_5", "position_named"):
            reject_position(suite, "5" if scenario.endswith("5") else "position")
        elif scenario == "check_config":
            suite.check_config()
        elif scenario.startswith("other_step_"):
            suite.other_step(scenario[len("other_step_"):])
        elif scenario == "wrapper_stop_failure":
            suite.wrapper_failure()
        elif scenario.startswith("stdout_"):
            _, mode, kind = scenario.split("_")
            suite.blocked_stdout(mode, kind)
        else:
            raise AssertionError("Ca không tồn tại: " + scenario)
    print("ĐẠT " + scenario)


if __name__ == "__main__":
    def interrupted(_sig, _frame):
        raise KeyboardInterrupt("Test bị ngắt; thu hồi cả nhóm tiến trình con")
    signal.signal(signal.SIGTERM, interrupted)
    try:
        main()
    except (AssertionError, OSError, ValueError, subprocess.SubprocessError) as failure:
        print("HỎNG: " + str(failure), file=sys.stderr)
        sys.exit(1)
