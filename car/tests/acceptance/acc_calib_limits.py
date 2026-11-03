#!/usr/bin/env python3
"""Kiểm hộp đen giới hạn hiệu chuẩn H1–H5/L1–L7; Python 3.6, thư viện chuẩn.

Kỳ vọng chỉ từ phiếu/giao diện/docs; dùng lại bộ đọc vết và dọn tiến trình của acceptance.
Không đọc src, không sửa cấu hình gốc, không có mã bỏ qua khi thiếu bản giả.
"""
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import time

sys.dont_write_bytecode = True
from acc_vehicle_cli import (SPEED_EPS, STEPS, Suite, held_at, log_ready, log_rows,
                             poll, read, require, run, tail, trace, wait_ready)

Y_AT = 4000
STALL = 5000
STALL_LENGTH = 6500
GAP = (5500, 6700)
LIMITS = (("calib_distance_cm", 300), ("calib_position_cm", 100),
          ("calib_max_duration_s", 120))


def near(a, b):
    return math.isfinite(a) and abs(a-b) <= SPEED_EPS


def images(folder):
    return {p for p in folder.rglob("*") if p.suffix.lower() in (".jpg", ".jpeg", ".png")}


def held_window(rows, kind, begin, end, want):
    """Kiểm cả giá trị giữ từ trước cửa sổ; không cần có dòng mới ở đúng begin."""
    require(end >= begin and rows and rows[-1][0] >= end,
            "Thiếu độ dài quan sát cửa sổ {} {}..{}".format(kind, begin, end))
    require(near(held_at(rows, kind, begin), want),
            "{} đang giữ sai tại {}: cần {}".format(kind, begin, want))
    require(all(near(float(v), want) for ms, k, v in rows if k == kind and begin <= ms <= end),
            "{} đổi sai trong {}..{}: cần {}".format(kind, begin, end, want))


def stopped(rows, begin, end, trim=7):
    held_window(rows, "motor_speed", begin, end, 0)
    held_window(rows, "servo_angle", begin, end, trim)


def check_restart(rows, cycle, delta, duration_ms):
    # H1/H3/L7: cửa sổ thuộc lượt y mới, trước hạn riêng; d phải tính góc từ 0.
    restarted = [ms for ms, k, v in rows if k == "motor_speed" and ms >= 12500 and abs(float(v)) > SPEED_EPS]
    require(restarted, "y mới phải tạo chuyển động")
    t0 = restarted[0]
    require(t0 <= 12500+delta+cycle, "y mới phải có hiệu lực trong dung sai đã đo")
    begin, end = t0+delta+cycle, t0+duration_ms-2*cycle
    require(end-begin >= 1000, "Tiền đề: cửa sổ lượt mới phải dài ít nhất 1 s")
    held_window(rows, "servo_angle", begin, end, 27)
    held_window(rows, "motor_speed", begin, end, 10)
    stop_by = t0+duration_ms+delta
    require(rows[-1][0] >= stop_by+1000, "Phải quan sát thêm ít nhất 1 s sau hạn dừng lượt mới")
    stopped(rows, stop_by, rows[-1][0])


def observed_timing(rows, logs, control_hz=50):
    # Đo trước sự kiện; dùng timestamp vết cho chu kỳ gửi servo và hiệu thời gian CSV cho ảnh.
    stamps = [ms for ms, kind, _ in rows if kind == "servo_angle" and 1000 <= ms <= 3300]
    cycles = [b-a for a, b in zip(stamps, stamps[1:]) if b > a]
    frames = []
    for row in logs:
        if int(row["frame_id"]) > 0 and 0 <= float(row["image_age_ms"]) < 500:
            captured = 1000*float(row["t"])-float(row["image_age_ms"])
            if not frames or row["frame_id"] != frames[-1][0]:
                frames.append((row["frame_id"], captured))
    periods = [b[1]-a[1] for a, b in zip(frames, frames[1:]) if b[1] > a[1]]
    require(len(cycles) >= 10 and len(periods) >= 5, "Tiền đề: thiếu mẫu đo chu kỳ trước sự kiện")
    control_period = 1000.0/control_hz
    cycle = max(control_period, max(cycles))
    frame = max(40, max(periods))  # camera_hz=25; container có thể chậm hơn.
    delta = 2*cycle+100
    # T05g ở 10 Hz: quá ba chu kỳ không còn phân giải được sự cố ToF 300 ms.
    cycle_limit = 300 if control_hz == 10 else 250
    require(cycle <= cycle_limit and frame <= 250,
            "Tiền đề: cycle={:.1f} ms (trần {}), frame={:.1f} ms (trần 250), delta={:.1f} ms; "
            "nhịp quá chậm để phân giải sự cố/cửa sổ ảnh đã thiết kế".format(
                cycle, cycle_limit, frame, delta))
    return cycle, frame, delta


class LimitsSuite(Suite):
    def setup(self, overrides=None, remove=(), gate=False):
        # Cấu hình tối thiểu từ Config.h/docs; không đọc hay sửa config.yaml của DEV.
        self.number += 1
        folder = self.folder / str(self.number)
        folder.mkdir()
        values = {"log_dir": '"{}"'.format((folder / "runs").as_posix()),
                  "fake_source": '""', "enable_display": "0", "enable_oled": "0",
                  "servo_trim_deg": "7", "servo_invert": "0", "tof_mapping_confirmed": "1",
                  "tof_names": "[front, rear, left, right]", "tof_safety_enabled": "1",
                  "tof_timeout_ms": "500", "image_timeout_ms": "500", "tof_brake_mm": "250",
                  "camera_hz": "25", "camera_width": "160", "camera_height": "96",
                  "control_hz": "50", "sensor_hz": "50", "odometry_hz": "20", "ui_hz": "50",
                  "imu_drift_ms": "100", "image_every_n": "1000000", "motor_keepalive_ms": "100",
                  "max_speed_cms": "25", "max_steering_deg": "30", "calib_servo_step_deg": "20",
                  "calib_servo_limit_deg": "30", "calib_speed_cms": "10",
                  "calib_distance_cm": "300", "calib_position_cm": "50", "calib_max_duration_s": "12"}
        values.update(overrides or {})
        config = folder / "config.yaml"
        config.write_text("%YAML:1.0\n---\n" + "".join("{}: {}\n".format(k, v)
                          for k, v in values.items() if k not in remove), encoding="utf-8")
        env = {k: v for k, v in os.environ.items() if not k.startswith("UCR_FAKE_")}
        env["UCR_FAKE_HW_TRACE"] = str(folder / "trace.csv")
        env["PYTHONIOENCODING"] = "utf-8"
        return folder, config, env

    def fake_only(self):
        # L5: OFF/thiếu binary là FAIL trước chạy, tuyệt đối không chạm phần cứng thật.
        info = subprocess.check_output([str(self.main), "--env-info"], timeout=5).decode("utf-8")
        require(re.search(r"simulated=1(?:\s|;|$)", info), "Cần bản UCR_FAKE_HW=ON; bản thật là FAIL")

    def command(self, config, step="motor"):
        return [self.main, "--config", config, "--mode", "calib", "--step", str(STEPS[step])]

    def wrapper(self, folder, env, command):
        build = folder / "build"
        build.mkdir()
        for name, binary in (("main", self.main), ("ucr_stop", self.stop)):
            (build / name).symlink_to(binary)
        env["UCR_BUILD_DIR"] = str(build)
        return ["bash", self.root / "scripts/run.sh"] + command[1:]

    def reach(self, process, folder, at):
        def reached():
            rows = trace(folder / "trace.csv", live=True)
            return bool(rows and rows[-1][0] >= at)
        wait_ready(process, reached, "Không quan sát tới mốc {} ms".format(at))
        return trace(folder / "trace.csv", live=True)

    def finish(self, process):
        process.send_signal(signal.SIGTERM)

    def motion(self, scenario, step="motor"):
        # H1/H2/H3/L1/L4/L5: vết phần cứng cuối, tiền đề thật và dung sai đo trước sự kiện.
        duration = 3 if scenario in ("deadline", "capture_deadline", "expiry_braked", "expiry_restart") else 12
        distance = scenario == "distance"
        image = scenario.startswith("image_")
        stalled_image = scenario == "image_stalled"
        inverted = scenario == "image_inverted"
        overrides = {"calib_max_duration_s": str(duration), "servo_invert": "1" if inverted else "0"}
        if distance:
            overrides["calib_distance_cm"] = "20"
        folder, config, env = self.setup(overrides)
        keys = [(3500, "d"), (Y_AT, "y")]
        if distance:
            keys = [(2500, "y"), (3500, "x"), (3800, "d"), (Y_AT, "y")]
        if scenario == "capture_deadline":
            keys.append((4500, "c"))
        if scenario == "expiry_restart":
            keys += [(12200, "d"), (12500, "y")]
        if image:
            env["UCR_FAKE_CAMERA_GAPS"] = "{}:{}".format(*GAP)
            if scenario == "image_stale_key":
                keys += [(6300, "y"), (8000, "y")]
            elif scenario.startswith("image_key_"):
                keys.append((8000, {"a": "a", "plus": "+", "minus": "-"}[scenario[10:]]))
        env["UCR_FAKE_KEYS"] = ",".join("{}:{}".format(ms, key) for ms, key in sorted(keys))
        stall_length = 4000 if image else STALL_LENGTH
        stalled = not image or stalled_image
        if stalled:
            env["UCR_FAKE_UI_STALL"] = "{}:{}".format(STALL, stall_length)
            # Nếu H5 không chặn thật, x sẽ dừng sớm và ca phải hỏng tiền đề duy trì chuyển động.
            keys.append((5200, "x"))
            env["UCR_FAKE_KEYS"] = ",".join("{}:{}".format(ms, key) for ms, key in sorted(keys))
        tof = scenario.startswith("tof_") or scenario == "expiry_braked"
        tof_at = 6000 if scenario == "expiry_braked" else 5500
        if tof:
            bad = [2000]*4
            slot = 1 if "rear" in scenario else 0
            bad[slot] = 100 if scenario == "tof_front_near" or scenario == "expiry_braked" else -1
            script = "0 2000 2000 2000 2000\n{} {}\n".format(tof_at, " ".join(map(str, bad)))
            if scenario == "expiry_braked":
                script += "8500 2000 2000 2000 2000\n"
            tof_file = folder / "tof.txt"
            tof_file.write_text(script, encoding="ascii")
            env["UCR_FAKE_TOF_SCRIPT"] = str(tof_file)

        def observe(process):
            # Mốc kiểm này luôn ở trước stall/gap/ToF/capture; motor phải đang chạy ở đây.
            rows = self.reach(process, folder, 4350)
            logs = log_rows(folder, "calib")
            require(log_ready(folder, "calib"), "Tiền đề: ảnh và cảm biến phải mới trước sự kiện")
            cycle, frame, delta = observed_timing(rows, logs)
            moves = [(ms, float(v)) for ms, k, v in rows if k == "motor_speed" and ms >= 3900 and abs(float(v)) > SPEED_EPS]
            require(moves, "Tiền đề: chưa thấy motor khác 0 trước hạn/chặn UI")
            t0, speed = moves[0]
            require(Y_AT <= t0 <= 4300 and near(held_at(rows, "motor_speed", 4300), speed),
                    "Tiền đề: y phải có hiệu lực trước chặn, motor vẫn khác 0")
            require(near(speed, 10), "Tiền đề: sai hướng/tốc độ đã thiết lập")
            expected_angle = -13 if inverted else 27
            require(near(held_at(rows, "servo_angle", 4300), expected_angle), "Tiền đề: chưa có góc 20 độ + trim")
            stop_by = t0+duration*1000+delta
            require(duration >= 3 and t0 < STALL, "Tiền đề: hạn >= 3 s, chạy trước UI chặn")
            if duration == 3:
                require(STALL+stall_length >= stop_by+1000, "Tiền đề: UI chặn ít nhất tới hạn +1 s")
            if image:
                stop_by = GAP[0]+500+frame+delta
                require(stop_by < GAP[1], "Tiền đề: stop_by < gap.second")
                if stalled_image:
                    require(STALL+stall_length >= stop_by+1000 and STALL < GAP[0],
                            "Tiền đề: UI chặn qua toàn bộ mất ảnh và hạn +1 s")
            if distance:
                stop_by = t0+2000+50+delta
                require(STALL+stall_length >= stop_by+1000 and stop_by < t0+duration*1000,
                        "Tiền đề: chạm quãng trước hạn thời gian/UI tỉnh")
            if tof:
                require(STALL+stall_length >= tof_at+20+delta+1000,
                        "Tiền đề: UI chặn qua hạn phanh ToF +1 s")
            print("Đo đầu ca: cycle={} frame={:.1f} delta={} t0={} stop_by={:.1f}".format(cycle, frame, delta, t0, stop_by))
            captures_before = images(folder) if scenario == "capture_deadline" else set()

            if scenario in ("deadline", "capture_deadline", "expiry_restart"):
                rows = self.reach(process, folder, int(stop_by+100))
                held_window(rows, "motor_speed", t0+300, t0+3000-2*cycle, speed)
                stopped(rows, stop_by, rows[-1][0])
            if image:
                rows = self.reach(process, folder, int(stop_by+60))
                held_window(rows, "motor_speed", t0+300, GAP[0]-2*cycle, speed)
                stale_logs = log_rows(folder, "calib")
                require(any(float(row["image_age_ms"]) >= 500 and row["state"] == "MAT_ANH"
                            for row in stale_logs), "Thiếu mẫu mất ảnh MAT_ANH")
                last_stale_id = max(int(row["frame_id"]) for row in stale_logs)
                rows = self.reach(process, folder, 7750)
                stopped(rows, stop_by, 7700)
                wait_ready(process, lambda: any(int(row["frame_id"]) > last_stale_id and
                           0 <= float(row["image_age_ms"]) < 500 for row in log_rows(folder, "calib")),
                           "Phải có bằng chứng ảnh mới sau ngắt")
                end = 9700
                rows = self.reach(process, folder, end)
                if scenario.startswith("image_key_"):
                    want = -13 if scenario.endswith("_a") else 7
                    held_window(rows, "motor_speed", stop_by, end, 0)
                    held_window(rows, "servo_angle", 8000+delta+cycle, end, want)
                elif scenario == "image_stale_key":
                    stopped(rows, stop_by, 7900)
                    held_window(rows, "motor_speed", 8000+delta+cycle, end, 10)
                    held_window(rows, "servo_angle", 8000+delta+cycle, end, 7)
                else:
                    stopped(rows, stop_by, end)
            elif scenario.startswith("tof_"):
                rows = self.reach(process, folder, 8500)
                brake_by = tof_at+20+delta
                require(brake_by < t0+duration*1000, "Tiền đề: phanh ToF trước hạn lượt")
                held_window(rows, "motor_speed", t0+300, tof_at-2*cycle, speed)
                if scenario == "tof_forward_rear_invalid":
                    held_window(rows, "motor_speed", t0+300, 8500, speed)
                else:
                    held_window(rows, "motor_speed", brake_by, 8500, 0)
            elif scenario == "distance":
                rows = self.reach(process, folder, 12200)
                # Không dừng sớm theo odom tuyệt đối của lượt thứ nhất.
                held_window(rows, "motor_speed", t0+300, t0+1500-2*cycle, speed)
                stopped(rows, stop_by, 12200)
                segments = []
                for row in log_rows(folder, "calib"):
                    moving = abs(float(row["cmd_speed"])) > SPEED_EPS
                    if not segments or segments[-1][0] != moving:
                        segments.append((moving, []))
                    segments[-1][1].append(row)
                runs = [group for moving, group in segments if moving]
                require(len(runs) == 2, "Quãng: phải có đúng hai lượt thực sự chuyển động trong CSV")
                begin_odom = float(runs[1][0]["odom_cm"])
                final_odom = float(log_rows(folder, "calib")[-1]["odom_cm"])
                tolerance = abs(speed)*(50+frame+delta)/1000
                require(begin_odom > 5 and abs(final_odom-begin_odom-20) <= tolerance,
                        "Quãng phải tính từ odom khác 0; delta_cm={} dung_sai={}".format(final_odom-begin_odom, tolerance))
            else:
                rows = self.reach(process, folder, 12000)
                stopped(rows, stop_by, 12000)
                if scenario == "expiry_braked":
                    held_window(rows, "motor_speed", t0+300, tof_at-2*cycle, speed)
                    held_window(rows, "motor_speed", tof_at+20+delta, 12000, 0)
                if scenario == "capture_deadline":
                    require(images(folder)-captures_before, "Phím c phải chụp ảnh trước hạn")
            if scenario == "expiry_restart":
                rows = self.reach(process, folder, 12500+duration*1000+2*delta+cycle+1000)
                check_restart(rows, cycle, delta, duration*1000)
            # Kiểm cả phần sau khi UI tỉnh; không chỉ cận trong thời gian chặn.
            if scenario in ("deadline", "capture_deadline"):
                stopped(rows, stop_by, 12000)
            self.finish(process)

        code, output = run(self.command(config, step), folder, env, observe, timeout=15)
        require(code in (0, 143), "Calib phải kết thúc chủ động, không timeout/lỗi")
        if tof and scenario != "tof_forward_rear_invalid":
            # L đổi hành vi: state phanh chỉ một tick; stdout giữ bằng chứng khi UI chặn.
            require(output.splitlines().count("AN_TOAN_CHOT: phanh_tof") == 1,
                    "ToF phải chốt đúng một dòng AN_TOAN_CHOT: phanh_tof")
        rows = trace(folder / "trace.csv")
        require(sum(k == "motor_init" for _, k, _ in rows) == 1, "Không khởi tạo lại motor")
        tail(rows)

    def exit_motion(self, kind):
        # L4: q thoát bình thường; EOF huỷ lượt nhưng giữ tiến trình quan sát cảm biến.
        folder, config, env = self.setup()
        env["UCR_FAKE_KEYS"] = "3500:d,4000:y" + (",5000:q" if kind == "q" else "")
        def observe(process):
            rows = self.reach(process, folder, 4500)
            require(near(held_at(rows, "motor_speed", 4400), 10) and near(held_at(rows, "servo_angle", 4400), 27),
                    "Tiền đề q/EOF: motor khác 0 và góc 20 độ")
            _, _, delta = observed_timing(rows, log_rows(folder, "calib"))
            if kind == "eof":
                before = rows[-1][0]
                old_frame = max(int(row["frame_id"]) for row in log_rows(folder, "calib"))
                process.stdin.close()
                rows = self.reach(process, folder, before+1800)
                stopped(rows, before+delta+100, rows[-1][0])
                require(any(int(row["frame_id"]) > old_frame and 0 <= float(row["image_age_ms"]) < 500
                            for row in log_rows(folder, "calib")), "EOF vẫn cho quan sát ảnh/cảm biến mới")
                self.finish(process)
        code, _ = run(self.command(config), folder, env, observe)
        require(code == 0 if kind == "q" else code in (0, 143), "q/EOF phải kết thúc đúng hợp đồng")
        rows = trace(folder / "trace.csv")
        tail(rows)
        if kind == "q":
            last_move = max(ms for ms, k, v in rows if k == "motor_speed" and abs(float(v)) > SPEED_EPS)
            require(last_move < 5500, "q phải dừng ngay sau phím, không chạy đến hết hạn lượt")

    def signal_stalled(self, sig):
        # H2/A: dừng khi UI chặn, qua trình bao; không dùng thời gian thoát làm bằng chứng dừng.
        folder, config, env = self.setup()
        env["UCR_FAKE_KEYS"] = "3500:d,4000:y"
        env["UCR_FAKE_UI_STALL"] = "5000:6500"
        command = self.wrapper(folder, env, self.command(config))
        sent_index = []
        def observe(process):
            rows = self.reach(process, folder, 5400)
            require(near(held_at(rows, "motor_speed", 5350), 10), "Tiền đề: đang chạy giữa UI chặn")
            _, _, delta = observed_timing(rows, log_rows(folder, "calib"))
            index, sent = len(rows), rows[-1][0]
            sent_index.append(index)
            process.send_signal(getattr(signal, "SIG"+sig))
            def has_stop():
                new = trace(folder / "trace.csv", live=True)[index:]
                return (any(k == "motor_speed" and near(float(v), 0) for _, k, v in new) and
                        any(k == "servo_angle" and near(float(v), 7) for _, k, v in new))
            require(poll(has_stop, (delta+200)/1000), "Tín hiệu phải dừng motor trước UI tỉnh")
            new = trace(folder / "trace.csv", live=True)[index:]
            zero = next(i for i, (_, k, v) in enumerate(new) if k == "motor_speed" and near(float(v), 0))
            require(all(near(float(v), 0) for _, k, v in new[zero:] if k == "motor_speed"), "Không lệnh chạy muộn sau dừng")
            require(sent+delta+200+1000 < 11500, "Tiền đề: UI chặn qua hạn dừng tín hiệu +1 s")
        code, _ = run(command, folder, env, observe, timeout=15)
        require(code in (0, 130, 143), "Trình bao phải giữ mã thoát tín hiệu hợp lệ")
        rows = trace(folder / "trace.csv")
        tail(rows)
        new = rows[sent_index[0]:]
        zero = next(i for i, (_, k, v) in enumerate(new) if k == "motor_speed" and near(float(v), 0))
        require(all(near(float(v), 0 if k == "motor_speed" else 7)
                    for _, k, v in new[zero:] if k in ("motor_speed", "servo_angle")),
                "UI tỉnh/thoát không ghi đè dừng sau tín hiệu")
        # ucr_stop có gốc ms riêng: xác nhận theo thứ tự dòng, không so ms giữa hai tiến trình.
        require(len(rows) >= 2 and rows[-2][1:] == ("motor_speed", "0") and rows[-1][1] == "servo_angle",
                "Trình bao phải gửi cặp dừng độc lập cuối phiên")

    def stall(self, kind, step="motor"):
        # H5/L6: từ chối cú pháp sai; chuỗi rỗng tương đương không đặt, bỏ khỏi ca lỗi.
        if kind == "syntax":
            for value in ("abc", "100", ":100", "100:", "1:2:3", "-1:2", "1:-2", "1.5:2", "1:2.5",
                          "1x:2", "1:2,3:4", " 1:2", "1:2 ", "1 :2", "+1:2", "1:+2", "1:\t2",
                          "9223372036854775808:1", "1:9223372036854775808"):
                folder, config, env = self.setup()
                env["UCR_FAKE_UI_STALL"] = value
                code, output = run(self.command(config), folder, env, timeout=5)
                require(code > 0 and "UCR_FAKE_UI_STALL" in output, "H5 phải từ chối {!r} với lý do rõ".format(value))
                require(not trace(folder / "trace.csv") and not (folder / "runs").exists(), "H5 lỗi phải trước thiết bị/log")
            return
        schedules = (None, "0:0", "1500:0", "0000:0000") if kind == "zero" else ("0:3000",) if kind == "once" else ("5000:3000",)
        for schedule in schedules:
            folder, config, env = self.setup({"calib_max_duration_s": "3"})
            if schedule is not None:
                env["UCR_FAKE_UI_STALL"] = schedule
            env["UCR_FAKE_KEYS"] = "1200:d,1500:y,8000:d,8500:y" if kind == "once" else "1200:d"
            def observe(process):
                if kind == "once":
                    early = self.reach(process, folder, 2300)
                    require(log_ready(folder, "calib"), "Trong chặn UI vẫn phải có ảnh/cảm biến")
                    require(not any(k == "motor_speed" and abs(float(v)) > SPEED_EPS for _, k, v in early),
                            "y trong chặn UI chưa được xử lý")
                rows = self.reach(process, folder, 10000 if kind == "once" else 9000 if kind == "unconfirmed" else 2300)
                require(log_ready(folder, "calib"), "H5: camera/cảm biến phải hoạt động")
                if kind == "once":
                    moves = [ms for ms, k, v in rows if k == "motor_speed" and abs(float(v)) > SPEED_EPS]
                    require(moves and 3000 <= min(moves) <= 4000, "UI phải xử lý y ngay sau chặn, không ngủ lại")
                    require(near(held_at(rows, "motor_speed", 9900), 10), "H5 chỉ chặn một lần, y mới phải chạy")
                else:
                    require(not any(k == "motor_speed" and abs(float(v)) > SPEED_EPS for _, k, v in rows),
                            "Không y thì motor phải luôn 0")
                self.finish(process)
            code, _ = run(self.command(config, step), folder, env, observe)
            require(code in (0, 143), "Lịch H5 hợp lệ không làm chương trình lỗi/treo")
            rows = trace(folder / "trace.csv")
            require(sum(k == "motor_init" for _, k, _ in rows) == 1, "Mỗi main chỉ một chuỗi motor_init")
            tail(rows)

    def config_cases(self, kind):
        # H4/L3/Q K: ba miền (0,trần], đúng mã lỗi/thông báo, không thiết bị hay runs.
        variants = []
        if kind == "defaults":
            variants = [({}, [key]) for key, _ in LIMITS] + [({}, [key for key, _ in LIMITS])]
        elif kind == "valid":
            variants = [({key: str(value)}, ()) for key, ceiling in LIMITS for value in (0.01, ceiling-0.01, ceiling)]
            variants += [({key: str(ceiling) for key, ceiling in LIMITS}, ())]
        else:
            variants = [({key: value}, ()) for key, ceiling in LIMITS
                        for value in (("0", "-0.01", str(ceiling+0.01)) if kind != "type" else ("[]", "{}", "abc"))]
        for overrides, remove in variants:
            modes = ("calib", "auto") if kind == "before_init" else (None,)
            for mode in modes:
                folder, config, env = self.setup(overrides, remove)
                if mode == "calib":
                    command = self.command(config)
                else:
                    command = [self.main, "--check-config", "--config", config] if mode is None else [self.main, "--config", config, "--mode", mode]
                code, _ = run(command, folder, env, timeout=5)
                valid = kind in ("defaults", "valid")
                if mode is None:
                    require(code == (0 if valid else 2), "H4: check-config phải trả đúng 0/2")
                else:
                    require(0 < code < 124, "H4: chạy thường phải từ chối cấu hình trước thiết bị")
                stdout, stderr = read(folder / "output.txt"), read(folder / "error.txt")
                if valid:
                    if kind != "defaults":
                        require(stdout.splitlines() == ["CAU_HINH_OK: " + str(config)], "check-config: đúng một dòng OK")
                else:
                    key = next(iter(overrides))
                    require(key in stderr and "CAU_HINH_OK:" not in stdout,
                            "H4: lỗi phải có khoá, không OK; mode={}, khoá={}, mã={}, stdout={!r}, stderr={!r}".format(
                                mode or "check-config", key, code, stdout, stderr))
                    if kind != "type":
                        ceiling = dict(LIMITS)[key]
                        match = re.search(re.escape(key)+r"\s*=\s*(\S+)\s+ngoài khoảng hợp lệ\s*\(\s*0\s*,\s*"+str(ceiling)+r"\s*\]", stderr)
                        require(match and near(float(match.group(1)), float(overrides[key])), "H4/K: khoá = giá trị và khoảng (0,trần]")
                require(not trace(folder / "trace.csv") and not (folder / "runs").exists(), "check-config/lỗi trước thiết bị/runs")

    def stop_invalid(self):
        # H4/CFG11: cấu hình lỗi không cản dừng; trình bao giữ lỗi main.
        for key, ceiling in LIMITS:
            folder, config, env = self.setup({key: str(ceiling+1)})
            main_code, _ = run(self.command(config), folder, env, timeout=5)
            require(0 < main_code < 124 and not trace(folder / "trace.csv"), "Main lỗi trước thiết bị")
            code, _ = run(self.wrapper(folder, env, self.command(config)), folder, env)
            require(code == main_code, "run.sh phải giữ mã lỗi cấu hình main")
            rows = trace(folder / "trace.csv")
            require(len(rows) == 2, "Chỉ ucr_stop gửi đúng hai lệnh, main không khởi tạo")
            tail(rows)
            (folder / "trace.csv").unlink()
            code, _ = run([self.stop, "--config", config], folder, env)
            require(code == 0, "ucr_stop vẫn thành công với trần calib sai")
            rows = trace(folder / "trace.csv")
            require(len(rows) == 2 and all(k not in ("motor_init", "motor_cfg") for _, k, _ in rows),
                    "ucr_stop không gửi cấu hình motor")
            tail(rows)


def self_test():
    # S/L5: bộ kiểm phải bắt motor quá hạn, servo trả chậm, phục hồi lệnh và cửa sổ chưa quan sát.
    good = [(0, "motor_speed", "10"), (0, "servo_angle", "27"),
            (3000, "motor_speed", "0"), (3000, "servo_angle", "7"), (5000, "motor_speed", "0")]
    stopped(good, 3200, 5000)
    # L7: tái hiện cycle=105/delta=310 từng làm cửa sổ đảo; kiểm cả nhịp chậm nhất cho phép.
    for cycle in (20, 105, 250):
        delta = 2*cycle+100
        restarted = [(0, "motor_speed", "0"), (0, "servo_angle", "7"),
                     (12220, "servo_angle", "27"), (12520, "motor_speed", "10"),
                     (15520, "motor_speed", "0"), (15520, "servo_angle", "7"),
                     (18000, "motor_speed", "0")]
        check_restart(restarted, cycle, delta, 3000)
        for bad in (restarted[:3]+restarted[4:],
                    restarted[:4]+[(13520, "motor_speed", "0")]+restarted[4:],
                    restarted[:4]+restarted[5:], restarted[:5]+restarted[6:],
                    restarted[:2]+[(12220, "servo_angle", "47")]+restarted[3:],
                    restarted[:-1]):
            try:
                check_restart(bad, cycle, delta, 3000)
            except AssertionError:
                continue
            raise AssertionError("Bộ kiểm lượt mới chấp nhận vết cố ý sai")
    bads = [good[:2]+good[3:], good[:3]+good[4:],
            good[:4]+[(4500, "motor_speed", "10"), good[-1]], good[:-1], good[:2]]
    for bad in bads:
        try:
            stopped(bad, 3200, 5000)
        except AssertionError:
            continue
        raise AssertionError("Bộ kiểm chấp nhận vết cố ý sai")
    hw = [(ms, "servo_angle", "7") for ms in range(1000, 3301, 20)]
    logs = [{"frame_id": str(i+1), "image_age_ms": "10", "t": str(1+i*0.06)} for i in range(20)]
    cycle, frame, delta = observed_timing(hw, logs)
    require(cycle == 20 and frame >= 59 and delta == 140 and GAP[0]+500+frame+delta < GAP[1],
            "Nhịp ảnh container 60 ms phải vừa cửa sổ mất ảnh 1200 ms")
    try:
        observed_timing([], [])
    except AssertionError:
        pass
    else:
        raise AssertionError("Không được mặc định thời gian khi chưa đo được")
    # L5: bản thật phải FAIL ở --env-info, trước mọi lệnh chuyển động, không trả 77.
    from unittest.mock import patch
    with tempfile.TemporaryDirectory(prefix="acc_calib_self_") as name:
        suite = LimitsSuite(".", "main-gia", "stop-gia", name)
        # L6: tên ca vẫn dùng motor/radius, nhưng CLI phải nhận số bước đúng tài liệu.
        for step, number in (("motor", "4"), ("radius", "3")):
            require(suite.command("config.yaml", step)[-2:] == ["--step", number], "CLI calib phải dùng số bước")
        with patch("acc_calib_limits.subprocess.check_output", return_value=b"simulated=0; source_id=mau"):
            try:
                suite.fake_only()
            except AssertionError:
                pass
            else:
                raise AssertionError("Bộ test không được chạy với bản thật")
        with patch("acc_calib_limits.subprocess.check_output", return_value=b"simulated=1; source_id=mau"):
            suite.fake_only()
        # L đổi hành vi: chấp nhận stdout dù CSV bỏ lỡ tick phanh; bắt thiếu/lặp/sai lý do.
        final_trace = [(0, "motor_init", "1"), (1, "motor_speed", "0"), (1, "servo_angle", "7")]
        for scenario in ("tof_front_near", "expiry_braked"):
            for output in ("AN_TOAN_CHOT: phanh_tof\n", "", "AN_TOAN_CHOT: mat_anh\n",
                           "AN_TOAN_CHOT: phanh_tof\n"*2):
                with patch.object(sys.modules[__name__], "run", return_value=(0, output)), \
                        patch.object(sys.modules[__name__], "trace", return_value=final_trace):
                    try:
                        suite.motion(scenario)
                    except AssertionError as error:
                        require(output != "AN_TOAN_CHOT: phanh_tof\n" and "AN_TOAN_CHOT" in str(error),
                                "Tự kiểm stdout hỏng ngoài phép đếm dòng chốt")
                    else:
                        require(output == "AN_TOAN_CHOT: phanh_tof\n", "Bỏ lọt stdout thiếu/lặp/sai lý do")
    print("ĐẠT tự kiểm bộ đọc vết/tiền đề; chưa kiểm sản phẩm")


def main():
    if len(sys.argv) == 2 and sys.argv[1] == "self-test":
        self_test()
        return
    require(len(sys.argv) >= 5, "Cần scenario root main ucr_stop [step]")
    scenario = sys.argv[1]
    with tempfile.TemporaryDirectory(prefix="acc_calib_limits_") as name:
        suite = LimitsSuite(sys.argv[2], sys.argv[3], sys.argv[4], name)
        suite.fake_only()
        if scenario.startswith("config_"):
            suite.config_cases(scenario[7:])
        elif scenario.startswith("stall_"):
            suite.stall(scenario[6:], sys.argv[5] if len(sys.argv) > 5 else "motor")
        elif scenario.startswith("exit_"):
            suite.exit_motion(scenario[5:])
        elif scenario.startswith("signal_"):
            suite.signal_stalled(scenario[7:])
        elif scenario == "stop_invalid":
            suite.stop_invalid()
        elif scenario == "require_fake":
            return
        else:
            suite.motion(scenario, sys.argv[5] if len(sys.argv) > 5 else "motor")


if __name__ == "__main__":
    try:
        main()
    except (AssertionError, OSError, ValueError, subprocess.SubprocessError) as error:
        print("HỎNG: " + str(error), file=sys.stderr)
        sys.exit(1)
