#!/usr/bin/env python3
"""Kiểm E1/E2/E3/E5 qua CLI và vết phần cứng; Python 3.6, không đọc src."""
import json
import re
import signal
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True
from acc_calib_limits import LimitsSuite, held_window, near, observed_timing, stopped
from acc_vehicle_cli import (SPEED_EPS, held_at, log_ready, log_rows, read, require,
                             run, tail, trace)

MANUAL_GAP = (5500, 6700)
MANUAL_STALE_KEY = 6500
MANUAL_NEW_KEY = 10000
SMOKE_GAP = (11800, 15000)
SMOKE_STALL = (11500, 10000)
SMOKE_CANCEL = 11700


def range_error(stderr, key, value, lower, upper, closed):
    # E3/K: kiểm cả giá trị bị vượt, hai cận và tên khoá liên quan khi ràng buộc chéo.
    bracket = r"\[" if closed else r"\("
    pattern = (re.escape(key)+r"\s*=\s*(\S+)\s+ngoài khoảng hợp lệ\s*"+
               bracket+r"\s*([0-9.]+)\s*,\s*([0-9.]+)\s*\]")
    match = re.search(pattern, stderr)
    require(match and abs(float(match.group(1))-float(value)) <= 1e-6 and
            abs(float(match.group(2))-lower) <= 1e-6 and abs(float(match.group(3))-upper) <= 1e-6,
            "K: thiếu khoá = giá trị ngoài khoảng đúng: " + stderr[-1000:])
    if key == "camera_resume_max_s":
        require("image_timeout_ms" in stderr, "K: thiếu tên khoá ràng buộc image_timeout_ms")


def manual_timing(cycle, frame, delta):
    stop_by = MANUAL_GAP[0]+500+frame+delta
    print("Đo đầu ca: cycle={} frame={:.1f} delta={} stop_by={} phím_cũ={} gap_end={}".format(
        cycle, frame, delta, stop_by, MANUAL_STALE_KEY, MANUAL_GAP[1]))
    require(stop_by < MANUAL_STALE_KEY < MANUAL_GAP[1] and MANUAL_GAP[1]+3000 < MANUAL_NEW_KEY,
            "Tiền đề: stop_by < gap.second, phím cũ lúc mất ảnh, đủ cửa sổ không phím")
    return stop_by


def smoke_timing(kind, t0, cycle, frame, delta):
    stop_by, held_until = t0+3000+delta, t0+1500
    if kind == "image":
        stop_by = SMOKE_GAP[0]+500+frame+delta
        # Ảnh còn mới trước gap; giữ cửa sổ qua phím x để chứng minh UI chưa huỷ lượt.
        held_until = SMOKE_GAP[0]-1
    if kind == "distance":
        stop_by, held_until = t0+1500+50+delta, t0+900
    print("Đo đầu ca smoke {}: cycle={} frame={:.1f} delta={} t0={} stop_by={} held_until={} "
          "stall={} x={} gap={}".format(kind, cycle, frame, delta, t0, stop_by, held_until,
                                       SMOKE_STALL, SMOKE_CANCEL, SMOKE_GAP), flush=True)
    if kind == "image":
        require(stop_by < SMOKE_GAP[1] and stop_by < t0+1900,
                "Tiền đề: stop_by < gap.second và trước hạn smoke 2 s cũ")
    if kind == "distance":
        require(stop_by < t0+1900, "Tiền đề: quãng phải dừng trước hạn smoke 2 s cũ")
    require(SMOKE_STALL[0]+SMOKE_STALL[1] >= stop_by+1000 and
            t0+100 < SMOKE_STALL[0] < SMOKE_CANCEL < held_until,
            "Tiền đề: UI chặn qua hạn +1 s, phím x phải chưa được xử lý")
    return stop_by, held_until


class ReviewSuite(LimitsSuite):
    def repeated(self, kind, step):
        # E1/H1: y trong lượt không được thay mốc thời gian hoặc mốc encoder của lượt đầu.
        if step == "position":
            # C1: không còn được chạy vị trí; motor/radius vẫn kiểm đủ hạn/quãng/y lặp.
            from acc_stop_path import reject_position
            reject_position(self)
            return
        distance = kind == "distance"
        duration = 12 if distance else 3
        folder, config, env = self.setup({"calib_max_duration_s": str(duration),
                                          "calib_distance_cm": "30" if distance else "300",
                                          "calib_position_cm": "100"})
        keys = [(3500, "d"), (4000, "y"), (5000, "y"), (6000, "y")]
        if not distance:
            keys += [(10000, "d"), (10500, "y")]
        env["UCR_FAKE_KEYS"] = ",".join("{}:{}".format(*key) for key in keys)

        def observe(process):
            rows = self.reach(process, folder, 4350)
            require(log_ready(folder, "calib"), "Tiền đề: ảnh/cảm biến chưa mới")
            cycle, frame, delta = observed_timing(rows, log_rows(folder, "calib"))
            moves = [ms for ms, k, v in rows if k == "motor_speed" and ms >= 4000 and abs(float(v)) > SPEED_EPS]
            require(moves and moves[0] < 4300, "Tiền đề: y đầu chưa cho motor chạy trước hạn")
            t0 = moves[0]
            require(near(held_at(rows, "motor_speed", 4300), 10), "Tiền đề: phải chạy 10 cm/s")
            if distance:
                moving = [float(r["odom_cm"]) for r in log_rows(folder, "calib")
                          if abs(float(r["cmd_speed"])) > SPEED_EPS]
                require(len(moving) >= 3 and max(moving)-min(moving) >= 1,
                        "Tiền đề: encoder giả phải đang tích phân quãng theo lệnh")
            stop_by = t0+3000+delta+(50 if distance else 0)
            require(duration >= 3 and 6000 < t0+3000-2*cycle and stop_by < 10000,
                    "Tiền đề: y lặp nằm trong lượt đầu, cửa sổ dừng trước y mới")
            print("Đo đầu ca: cycle={} frame={:.1f} delta={} t0={} stop_by={}".format(cycle, frame, delta, t0, stop_by))
            rows = self.reach(process, folder, 9500)
            held_window(rows, "motor_speed", t0+300, t0+3000-2*cycle-100, 10)
            stopped(rows, stop_by, 9400)
            if not distance:
                rows = self.reach(process, folder, 15000)
                restart = [ms for ms, k, v in rows if k == "motor_speed" and ms >= 10500 and abs(float(v)) > SPEED_EPS]
                require(restart and restart[0] <= 10500+delta+cycle, "y sau dừng phải bắt đầu lượt mới")
                t1 = restart[0]
                require(t1+3000+delta+1000 < 15000, "Tiền đề: đủ quan sát hạn riêng lượt mới")
                held_window(rows, "motor_speed", t1+delta+cycle, t1+3000-2*cycle, 10)
                held_window(rows, "servo_angle", t1+delta+cycle, t1+3000-2*cycle, 27)
                stopped(rows, t1+3000+delta, 14900)
            self.finish(process)

        code, _ = run(self.command(config, step), folder, env, observe)
        require(code in (0, 143), "Calib không thoát chủ động")
        tail(trace(folder / "trace.csv"))

    def manual(self, reverse):
        # E2/C': phím khi ảnh cũ bị bỏ, ảnh mới không được phục hồi requested cũ.
        key, speed = ("s", -20) if reverse else ("w", 20)
        folder, config, env = self.setup({"keyboard_speed_step_cms": "20"})
        env["UCR_FAKE_CAMERA_GAPS"] = "{}:{}".format(*MANUAL_GAP)
        env["UCR_FAKE_KEYS"] = "3500:d,4000:{0},{1}:{0},{2}:{0}".format(key, MANUAL_STALE_KEY, MANUAL_NEW_KEY)

        def observe(process):
            rows = self.reach(process, folder, 4350)
            require(log_ready(folder, "manual"), "Tiền đề: ảnh/ToF chưa mới")
            cycle, frame, delta = observed_timing(rows, log_rows(folder, "manual"))
            require(near(held_at(rows, "motor_speed", 4300), speed), "Tiền đề: manual chưa chạy đúng chiều")
            stop_by = manual_timing(cycle, frame, delta)
            rows = self.reach(process, folder, 11500)
            held_window(rows, "motor_speed", 4350, MANUAL_GAP[0]-2*cycle, speed)
            stopped(rows, stop_by, MANUAL_NEW_KEY-10)
            samples = log_rows(folder, "manual")
            stale = [r for r in samples if float(r["image_age_ms"]) >= 500 and r["state"] == "MAT_ANH"]
            require(stale, "Phải quan sát MAT_ANH khi mất ảnh")
            old_id = max(int(r["frame_id"]) for r in stale)
            require(any(int(r["frame_id"]) > old_id and 0 <= float(r["image_age_ms"]) < 500 and
                        abs(float(r["cmd_speed"])) <= SPEED_EPS for r in samples),
                    "Phải quan sát ảnh mới nhưng motor vẫn 0")
            held_window(rows, "motor_speed", MANUAL_NEW_KEY+delta+cycle, 11400, speed)
            self.finish(process)

        code, _ = run([self.main, "--config", config, "--mode", "manual"], folder, env, observe)
        require(code in (0, 143), "Manual không thoát chủ động")
        tail(trace(folder / "trace.csv"))

    def bounds(self, kind):
        # E3/K/CFG11: cả check-config và đường chạy thật đều kiểm trước phần cứng.
        cases = []
        if kind == "valid":
            cases = [("soft_start_s", str(v), 500) for v in (0.2, 0.201, 9.999, 10)]
            cases += [("camera_resume_max_s", str(v), 500) for v in (0.501, 4.999, 5)]
            cases += [("camera_resume_max_s", "0.201", 200)]
        else:
            cases = [("soft_start_s", v, 500) for v in ("0", "-0.1", "0.199", "10.001", "abc", "[]", "{}", '"1.0"')]
            cases += [("camera_resume_max_s", v, 500) for v in ("0", "0.499", "0.5", "5.001", "abc", "[]", "{}", '"2.0"')]
            cases += [("camera_resume_max_s", "0.2", 200)]
            if kind in ("before_init", "stop"):
                cases = [("soft_start_s", "0.199", 500), ("camera_resume_max_s", "5.001", 500)]
        for key, value, timeout in cases:
            modes = ("manual", "auto", "collect", "calib", "smoke") if kind == "before_init" else (None,)
            for mode in modes:
                folder, config, env = self.setup({key: value, "image_timeout_ms": str(timeout)})
                command = ([self.main, "--check-config", "--config", config] if mode is None else
                           self.command(config) if mode == "calib" else
                           [self.main, "--config", config, "--mode", mode])
                code, _ = run(command, folder, env, timeout=5)
                stdout, stderr = read(folder / "output.txt"), read(folder / "error.txt")
                require(code == (0 if kind == "valid" else 2) if mode is None else 0 < code < 124,
                        "Sai mã thoát kiểm cấu hình: {} {} {} -> {}".format(mode, key, value, code))
                require(not trace(folder / "trace.csv") and not (folder / "runs").exists(),
                        "Kiểm/lỗi cấu hình không được khởi tạo phần cứng hoặc runs")
                if kind == "valid":
                    require(stdout.splitlines() == ["CAU_HINH_OK: " + str(config)], "Sai dòng CAU_HINH_OK")
                else:
                    require(key in stderr and "ngoài khoảng hợp lệ" in stderr and "CAU_HINH_OK:" not in stdout,
                            "Lỗi phải nêu khoá + khoảng, không OK")
                    try:
                        float(value)
                    except ValueError:
                        pass
                    else:
                        range_error(stderr, key, value, 0.2 if key == "soft_start_s" else timeout/1000.0,
                                    10 if key == "soft_start_s" else 5, key == "soft_start_s")
                if kind == "stop":
                    # CFG11 so cùng lệnh chạy calib; mã 2 riêng của --check-config không phải mã chuẩn chạy xe.
                    command = self.command(config)
                    main_code, main_output = run(command, folder, env, timeout=5)
                    require(0 < main_code < 124 and key in main_output and "ngoài khoảng hợp lệ" in main_output,
                            "CFG11: main chạy thường phải từ chối đúng khoá ngoài khoảng")
                    require(not trace(folder / "trace.csv") and not (folder / "runs").exists(),
                            "CFG11: main lỗi trước phần cứng/runs")
                    wrapper_code, wrapper_output = run(self.wrapper(folder, env, command), folder, env, timeout=5)
                    require(wrapper_code == main_code,
                            "CFG11: run.sh không giữ mã lỗi cùng lệnh main: main={}, run.sh={}, khoá={}\n{}".format(
                                main_code, wrapper_code, key, wrapper_output[-1000:]))
                    rows = trace(folder / "trace.csv")
                    require(len(rows) == 2 and all(k not in ("motor_cfg", "motor_init") for _, k, _ in rows),
                            "CFG11: chỉ ucr_stop gửi hai lệnh, không cấu hình motor")
                    tail(rows)

    def smoke_stall(self, kind):
        # E5/H1/H5: stage motor đầu sau bốn xác nhận servo và xác nhận bánh kê.
        # Smoke đối chiếu kích thước ảnh; giữ kích thước mặc định của nguồn tổng hợp, không giảm ngưỡng kiểm.
        folder, config, env = self.setup({"camera_width": "1280", "camera_height": "720",
                                          "calib_max_duration_s": "3", "calib_distance_cm": "15" if kind == "distance" else "300"})
        env["UCR_FAKE_KEYS"] = "3500:y,4500:y,5500:y,6500:y,11000:y,{}:x".format(SMOKE_CANCEL)
        env["UCR_FAKE_UI_STALL"] = "{}:{}".format(*SMOKE_STALL)
        if kind == "image":
            env["UCR_FAKE_CAMERA_GAPS"] = "{}:{}".format(*SMOKE_GAP)

        def observe(process):
            try:
                rows = self.reach(process, folder, 3300)
            except AssertionError:
                for path in (folder / "runs").glob("*_smoke/smoke.json"):
                    result = json.loads(read(path))
                    print("Tiền đề smoke: config 1280x720, ánh xạ=1, mã={} kết quả={}".format(
                        process.poll(), {key: result.get(key) for key in ("camera", "sensors", "servo", "passed")}))
                raise
            require(log_ready(folder, "smoke"), "Tiền đề: smoke cần ảnh/cảm biến mới")
            cycle, frame, delta = observed_timing(rows, log_rows(folder, "smoke"))
            rows = self.reach(process, folder, 11300)
            moves = [ms for ms, k, v in rows if k == "motor_speed" and abs(float(v)) > SPEED_EPS]
            require(moves and 11000 <= moves[0] <= 11250 and near(held_at(rows, "motor_speed", 11300), 10),
                    "Tiền đề: phải có motor 10 sau xác nhận bánh kê, trước UI chặn")
            if kind == "distance":
                moving = [float(r["odom_cm"]) for r in log_rows(folder, "smoke")
                          if abs(float(r["cmd_speed"])) > SPEED_EPS]
                require(len(moving) >= 3 and max(moving)-min(moving) >= 1,
                        "Tiền đề: encoder giả phải tích phân quãng trước UI chặn")
            t0 = moves[0]
            stop_by, held_until = smoke_timing(kind, t0, cycle, frame, delta)
            rows = self.reach(process, folder, 16000)
            held_window(rows, "motor_speed", t0+100, held_until, 10)
            stopped(rows, stop_by, 15900)
            if kind == "image":
                require(any(r["state"] == "MAT_ANH" and float(r["image_age_ms"]) >= 500
                            for r in log_rows(folder, "smoke")), "Thiếu bằng chứng ảnh cũ ở smoke")
            self.finish(process)

        code, _ = run([self.main, "--config", config, "--mode", "smoke"] + self.smoke_args(folder),
                      folder, env, observe, timeout=15)
        require(code in (0, 1, 143), "Smoke phải thoát có kiểm soát")
        tail(trace(folder / "trace.csv"))


def self_test():
    # Tự kiểm bộ phán xét bằng vết sai có chủ ý; không phải bằng chứng mã xe đạt.
    good = [(0, "motor_speed", "10"), (20, "servo_angle", "27"),
            (3000, "motor_speed", "0"), (3000, "servo_angle", "7"),
            (5000, "servo_angle", "7")]
    stopped(good, 3100, 4900)
    require(manual_timing(103, 48, 306) == 6354, "Lịch manual phải chứa hạn đo, phím cũ và ảnh về")
    require(manual_timing(104, 60, 308) == 6368, "Lịch manual phải chứa chu kỳ ảnh container 60 ms")
    for cycle, frame in ((20, 42), (103, 48), (104, 60)):
        stop_by, held_until = smoke_timing("image", 11020, cycle, frame, 2*cycle+100)
        require(SMOKE_CANCEL < held_until < SMOKE_GAP[0] < stop_by < 15000,
                "Lịch smoke phải giữ motor qua x, trước gap; không nới hạn dừng ảnh")
    for kind in ("deadline", "distance"):
        smoke_timing(kind, 11020, 104, 60, 308)
    range_error("camera_resume_max_s = 5.001 ngoài khoảng hợp lệ (0.5, 5] theo image_timeout_ms",
                "camera_resume_max_s", "5.001", 0.5, 5, False)
    bad_checks = [lambda: stopped(good, 2500, 4900), lambda: stopped(good, 4900, 3100),
                  lambda: range_error("soft_start_s = 0.199 ngoài khoảng hợp lệ (0, 10]",
                                      "soft_start_s", "0.199", 0.2, 10, True),
                  lambda: manual_timing(250, 250, 600),
                  lambda: smoke_timing("image", 11020, 250, 250, 600),
                  lambda: smoke_timing("image", 11500, 20, 42, 140)]
    for check in bad_checks:
        try:
            check()
        except AssertionError:
            continue
        raise AssertionError("Bộ kiểm bỏ lọt vết/khoảng/cửa sổ sai")
    require(not near(float("nan"), 0) and not near(float("inf"), 0), "Không chấp nhận số không hữu hạn")
    # Kiểm hồi quy CFG11: check-config=2, chạy main=1; wrapper phải giữ 1 và vẫn gửi cặp dừng.
    from unittest.mock import patch
    with tempfile.TemporaryDirectory(prefix="acc_bounds_selfcheck_") as folder:
        calls = []
        wrapper_exit = [1]

        def fake_run(command, where, env, timeout=5):
            calls.append(command)
            config = read(where / "config.yaml")
            message = ("soft_start_s = 0.199 ngoài khoảng hợp lệ [0.2, 10]" if "soft_start_s: 0.199" in config else
                       "camera_resume_max_s = 5.001 ngoài khoảng hợp lệ (0.5, 5] theo image_timeout_ms")
            (where / "output.txt").write_text("", encoding="utf-8")
            (where / "error.txt").write_text(message, encoding="utf-8")
            if command[0] == "wrapper":
                (where / "trace.csv").write_text("0,motor_speed,0\n0,servo_angle,7\n", encoding="utf-8")
                return wrapper_exit[0], message
            return (2 if "--check-config" in command else 1), message

        suite = ReviewSuite(folder, "main-gia", "stop-gia", folder)
        suite.wrapper = lambda where, env, command: ["wrapper"]+command
        with patch(__name__+".run", side_effect=fake_run):
            suite.bounds("stop")
            require(len(calls) == 6 and all(calls[i+1][1:] == calls[i] for i in (1, 4)),
                    "CFG11 phải so đúng cùng đối số/cấu hình main với wrapper")
            wrapper_exit[0] = 2
            try:
                suite.bounds("stop")
            except AssertionError as error:
                require("run.sh không giữ mã lỗi cùng lệnh main" in str(error), "Phải bắt đúng lỗi mã wrapper")
            else:
                raise AssertionError("CFG11 bỏ lọt wrapper đổi mã 1 thành 2")
    print("ĐẠT: tự kiểm bộ phán xét; bắt 6 mẫu vết/khoảng/lịch sai và wrapper đổi mã lỗi")


def main():
    if len(sys.argv) == 2 and sys.argv[1] == "self-test":
        self_test()
        return
    require(len(sys.argv) >= 5, "Cần scenario root main ucr_stop [step]")
    with tempfile.TemporaryDirectory(prefix="acc_review_fixes_") as folder:
        suite = ReviewSuite(sys.argv[2], sys.argv[3], sys.argv[4], folder)
        suite.fake_only()
        scenario = sys.argv[1]
        if scenario == "require_fake":
            return
        if scenario.startswith("repeat_"):
            suite.repeated(scenario[7:], sys.argv[5])
        elif scenario.startswith("manual_"):
            suite.manual(scenario.endswith("reverse"))
        elif scenario.startswith("bounds_"):
            suite.bounds(scenario[7:])
        elif scenario.startswith("smoke_"):
            suite.smoke_stall(scenario[6:])
        else:
            raise AssertionError("Ca không tồn tại: " + scenario)


if __name__ == "__main__":
    def interrupted(_sig, _frame):
        raise KeyboardInterrupt("Test bị ngắt, thu hồi nhóm tiến trình con")
    signal.signal(signal.SIGTERM, interrupted)
    try:
        main()
    except (AssertionError, OSError, ValueError, subprocess.SubprocessError) as error:
        print("HỎNG: " + str(error), file=sys.stderr)
        sys.exit(1)
