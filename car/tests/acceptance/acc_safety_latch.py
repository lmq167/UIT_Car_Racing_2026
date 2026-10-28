#!/usr/bin/env python3
"""T05g/Quyết định L và phân xử 28/10: kiểm hộp đen chốt an toàn.

Python 3.6, chỉ thư viện chuẩn; không đọc src hoặc log phiên DEV.
Mỗi ca kiểm cả vết đầu ra, CSV, phím mới và số thông báo, không dùng cleanup làm phanh.
"""
import math
from pathlib import Path
import re
import sys
import tempfile

sys.dont_write_bytecode = True
from acc_calib_limits import LimitsSuite, held_window, near, observed_timing
from acc_seed_gate import seed_gate
from acc_vehicle_cli import (SPEED_EPS, collect_metadata, held_at, log_ready,
                             log_rows, read, require, run, tail, trace)

PHASES = tuple(range(0, 100, 10))
REASONS = ("mat_anh", "phanh_tof", "phanh_tof_sau", "gioi_han_calib")
SAFETY_STATES = {"DUNG", "MAT_ANH", "PHANH_TOF", "PHANH_TOF_SAU",
                 "DUNG_GIOI_HAN_CALIB", "DUNG_LENH_VI_TRI"}


def notifications(output, expected):
    # L.4/S5: so toàn danh sách để bắt cả dòng thừa, sai lý do hoặc lặp mỗi tick.
    lines = [line.strip() for line in output.splitlines() if "AN_TOAN_CHOT:" in line]
    require(lines == ["AN_TOAN_CHOT: " + reason for reason in expected],
            "Sai thông báo chốt: cần {}, có {}".format(expected, lines))


def samples(logs):
    # Autonomy có thể ghi lại cùng snapshot nhiều lần; mỗi t chỉ là một tick.
    unique = {}
    for row in logs:
        unique[float(row["t"])*1000] = row
    return sorted(unique.items())


def offset_ms(rows, logs, speed):
    hardware = [ms for ms, kind, value in rows
                if kind == "motor_speed" and near(float(value), speed)]
    recorded = [t for t, row in samples(logs) if near(float(row["cmd_speed"]), speed)]
    require(hardware and recorded, "Tiền đề: thiếu chuyển động để nối hai đồng hồ")
    return hardware[0] - recorded[0]


def csv_window(logs, offset, begin, end, speed):
    selected = [row for t, row in samples(logs) if begin <= t+offset <= end]
    require(end-begin >= 300 and len(selected) >= 2, "Thiếu CSV trong cửa sổ giữ lệnh")
    require(all(near(float(row["cmd_speed"]), speed) for row in selected),
            "CSV không giữ cmd_speed={} trong {}..{}".format(speed, begin, end))


def old_states(logs, before):
    # S1/L.2: tên trạng thái thường chưa được liệt kê đầy đủ; lấy mốc trước sự cố,
    # chỉ cho phép các tên an toàn công bố ngoài tập đó. Không suy từ mã triển khai.
    baseline = {row["state"] for row in before}
    require(baseline, "Tiền đề S1: thiếu trạng thái bình thường trước sự cố")
    allowed = baseline | SAFETY_STATES
    actual = {row["state"] for row in logs}
    require(not any("AN_TOAN_CHOT" in state for state in actual) and actual <= allowed,
            "S1: state mới ngoài tập cũ: {}".format(sorted(actual-allowed)))


def phase_evidence(rows, logs, offset, gap):
    """Chứng minh đủ tick quanh hai frame liền kề qua ngắt; trả True nếu lọt pha.

    CSV có thể bỏ tick; chỉ suy ảnh mới nếu cận tuổi từ snapshot trước vẫn <500 ms.
    captured_at = t - image_age_ms; hai đồng hồ chỉ khác một hằng số offset.
    """
    data = samples(logs)
    frames = {}
    for t, row in data:
        if int(row["frame_id"]) and float(row["image_age_ms"]) >= 0:
            fid = int(row["frame_id"])
            captured = t-float(row["image_age_ms"])+offset
            frames.setdefault(fid, captured)
    pairs = [(a, b, ca, cb) for (a, ca), (b, cb) in zip(sorted(frames.items()), sorted(frames.items())[1:])
             if b == a+1 and ca < gap[0] and cb >= gap[1] and cb-ca > 500]
    if not pairs:
        print("Gap {}:{}: CSV chưa bắt được hai frame kề; cần bằng chứng từ pha khác".format(*gap), flush=True)
        return False
    require(len(pairs) == 1, "Tiền đề pha: nhiều khoảng mất ngoài lịch")
    _, _, before, after = pairs[0]
    ticks = [ms for ms, kind, _ in rows if kind == "servo_angle" and before+10 <= ms <= after+150]
    require(len(ticks) >= 5, "Tiền đề pha: thiếu tick chấp hành qua khoảng mất")
    selected = [(t, row) for t, row in data if before+10 <= t+offset <= after+150]
    if any(not 0 <= float(row["image_age_ms"]) < 500 for _, row in selected):
        print("Gap {}:{}: có mẫu thấy tuổi ảnh >=500; không lọt".format(*gap), flush=True)
        return False
    direct = 0
    for tick in ticks:
        candidates = [row for t, row in data if abs(t+offset-tick) <= 8]
        if len(candidates) == 1:
            if not 0 <= float(candidates[0]["image_age_ms"]) < 500:
                return False
            direct += 1
            continue
        # Khung đã thấy ở snapshot trước vẫn tồn tại hoặc được thay bằng khung mới hơn.
        # Cộng 8 ms như cửa sổ ghép đồng hồ; không suy từ FPS hay việc thiếu dòng CSV.
        previous = [(t, row) for t, row in data if t+offset < tick-8 and
                    int(row["frame_id"]) > 0 and float(row["image_age_ms"]) >= 0]
        upper_age = (tick-(previous[-1][0]+offset)+float(previous[-1][1]["image_age_ms"])+8
                     if previous else math.inf)
        if len(candidates) > 1 or not 0 <= upper_age < 500:
            print("Gap {}:{}: tick {} thiếu mẫu, cận tuổi={:.1f} ms; chưa chứng minh lọt".format(
                gap[0], gap[1], tick, upper_age), flush=True)
            return False
    print("Gap {}:{}: captured_at={:.1f}..{:.1f}, {} tick: {} CSV + {} cận tuổi <500; lọt".format(
        gap[0], gap[1], before, after, len(ticks), direct, len(ticks)-direct), flush=True)
    return True


LOT_FRAME_MAX_MS = 150


def phase_verdict(result, frames):
    """Điều kiện 'lọt' bắt buộc chỉ khi nhịp khung đo được <= 150 ms.

    Khung chậm hơn: khoảng không khung = 550 ms + phần dư tới khung kế, nên luôn có
    tick thấy tuổi >=500; không chứng minh được lọt bằng log (PHƯƠNG ÁN DỰ PHÒNG, leader duyệt).
    Hành vi mọi pha vẫn được kiểm đầy đủ trong event(); hàm này chỉ quyết định điều kiện lọt.
    """
    worst = max(frames)
    if worst > LOT_FRAME_MAX_MS:
        print("GHI CHU: khong chung minh duoc lot o nhip khung {:.0f} ms".format(worst), flush=True)
        return
    require(any(result), "Lỗi thiết lập: {}/{} pha lọt giữa tick (khung {:.0f} ms); không tính ĐẠT".format(
        sum(result), len(result), worst))


class LatchSuite(LimitsSuite):
    def setup(self, overrides=None, remove=(), gate=False):
        values = {"control_hz": "10", "ui_hz": "50", "keyboard_speed_step_cms": "5",
                  "calib_servo_step_deg": "10", "calib_max_duration_s": "30"}
        values.update(overrides or {})
        return super().setup(values, remove, gate)

    def execute(self, mode, config, folder, env, observe, step="motor", auto=False):
        self.baseline = []
        self.frames = getattr(self, "frames", [])
        command = (self.command(config, step) if mode == "calib" else
                   [self.main, "--config", config, "--mode", mode])

        def watching(process):
            if mode == "collect":
                collect_metadata(process)
            observe(process)
        code, output = run(command, folder, env, watching)
        require(code in (0, 143), "Tiến trình phải thoát có kiểm soát, mã={}".format(code))
        tail(trace(folder / "trace.csv"))
        logs = log_rows(folder, mode)
        require(logs, "Thiếu log.csv")
        if not auto:
            old_states(logs, self.baseline)
        return output, logs

    def event(self, mode, kind, stalled=False, phase=0, opposite=False, reverse=False, twice=False, mixed=False):
        # T01–04/06–08/11–12/14–19/21/S3: cắt requested, giữ huỷ độc lập UI.
        image = kind in ("phase", "image", "long", "stale", "short")
        twice = twice or mixed
        passive = kind in ("short", "front_reverse", "rear_forward", "rear_forward_invalid", "idle")
        reverse = reverse or kind in ("rear", "rear_invalid", "front_reverse")
        initial = -10 if reverse else 10
        key = "s" if reverse else "w"
        new_key = ("w" if reverse else "s") if opposite else key
        restart = 10 if mode == "calib" else (5 if new_key == "w" else -5)
        begin = 5500+phase
        length = (550 if kind == "phase" else 250 if kind == "short" else
                  1800 if kind == "stale" else 1200 if kind == "long" else 650)
        end = begin+length if image else begin+300
        new_at = 10000 if stalled else 9000
        finish_at = new_at+1200
        folder, config, env = self.setup()
        keys = [(3500, "d"), (4000, "y")] if mode == "calib" else [(3800, key), (4100, key)]
        if kind == "idle":
            keys = [(3500, " ")]  # Không đặt lệnh chuyển động, vẫn có kịch bản phím hợp lệ cho collect.
            initial = 0
        if not passive:
            keys.append((new_at, "y" if mode == "calib" else new_key))
        if kind == "stale":
            keys.append((begin+1100, "y" if mode == "calib" else new_key))
        if stalled:
            env["UCR_FAKE_UI_STALL"] = "5000:3000"
            keys.append((5200, " " if mode == "collect" else "x"))
        events = [(begin, end)]
        if twice:
            events.append((new_at+1500, new_at+1500+(1200 if mixed else length if image else 300)))
            finish_at = events[-1][1]+1500
        if mixed:
            env["UCR_FAKE_CAMERA_GAPS"] = "{}:{}".format(*events[-1])
        if image:
            env["UCR_FAKE_CAMERA_GAPS"] = ",".join("{}:{}".format(*event) for event in events)
        else:
            slot = 1 if kind.startswith("rear") else 0
            bad = [2000]*4
            bad[slot] = -1 if "invalid" in kind else 100
            # T19: thêm vật cản khi lệnh đã bị xoá không được tạo lần chốt thứ hai.
            scripted = events[:1] if mixed else events + ([] if twice or passive else [(7200, 7500)])
            script = "0 2000 2000 2000 2000\n"
            for start, stop in scripted:
                script += "{} {}\n{} 2000 2000 2000 2000\n".format(start, " ".join(map(str, bad)), stop)
            path = folder / "tof.txt"
            path.write_text(script, encoding="ascii")
            env["UCR_FAKE_TOF_SCRIPT"] = str(path)
        env["UCR_FAKE_KEYS"] = ",".join("{}:{}".format(*item) for item in sorted(keys))
        evidence = []

        def observe(process):
            rows = self.reach(process, folder, 4500)
            logs = log_rows(folder, mode)
            self.baseline = logs
            require(log_ready(folder, mode), "Tiền đề: ảnh/ToF chưa mới")
            cycle, frame, _ = observed_timing(rows, logs, control_hz=10)
            delta = cycle+20+100  # cảm biến 50 Hz, một tick chấp hành và dung sai lịch.
            print("Đo trước sự cố: cycle={:.1f} ms frame={:.1f} ms delta={:.1f} ms".format(
                cycle, frame, delta), flush=True)
            self.frames.append(frame)
            held_window(rows, "motor_speed", 4350, 4450, initial)
            offset = offset_ms(rows, logs, initial) if initial else 0
            stop_by = (begin+500+frame+cycle+100 if image else begin+delta)
            if kind in ("long", "stale"):
                require(stop_by < end, "Tiền đề: stop_by < gap.second")
            if kind == "stale":
                require(stop_by < begin+1100 < end-100, "Tiền đề: phím phải ở giữa đoạn ảnh đã cũ")
            require(new_at-end >= 1000 and (not stalled or 8000+1000 <= new_at),
                    "Tiền đề: cửa sổ phục hồi không phím phải ≥1 s")
            if stalled:
                # UI chạy 50 Hz; độ trễ chấp hành không quyết định lúc UI bắt đầu chặn.
                require(5000+20+100 < 5200 < begin and 8000 >= end+1000,
                        "Tiền đề: UI phải bị chặn phủ sự cố và phục hồi")
                rows = self.reach(process, folder, begin-100)
                held_window(rows, "motor_speed", 5250, begin-100, initial)
            print("{} {} pha={}: cycle={} frame={:.1f} stop_by={:.1f} gap_end={}".format(
                mode, kind, phase, cycle, frame, stop_by, end), flush=True)
            rows = self.reach(process, folder, finish_at)
            logs = log_rows(folder, mode)
            held_window(rows, "motor_speed", 4350, begin-1, initial)
            if not image:
                require(any(near(float(r["tof"+str(slot)]), bad[slot]) for r in logs),
                        "Tiền đề: CSV chưa thấy giá trị ToF đã tiêm")
            if passive:
                held_window(rows, "motor_speed", 4350, finish_at-100, initial)
                if initial:
                    csv_window(logs, offset, begin, end+1000, initial)
                require(not any(row["state"] in ("PHANH_TOF", "PHANH_TOF_SAU") for row in logs),
                        "Đối chứng không được có trạng thái phanh ToF")
                if kind == "short":
                    require(any(150 <= float(r["image_age_ms"]) < 500
                                for t, r in samples(logs) if begin <= t+offset <= end+frame),
                            "Tiền đề đối chứng 250 ms: chưa thấy camera thực sự ngắt")
            else:
                first_zero = [ms for ms, k, v in rows if k == "motor_speed" and
                              begin <= ms <= (stop_by if image else min(stop_by, end-1)) and near(float(v), 0)]
                require(first_zero, "Không có motor 0 trong hạn, trước cleanup")
                held_window(rows, "motor_speed", first_zero[0], new_at-1, 0)
                csv_window(logs, offset, stop_by+150, new_at-100, 0)
                recovered = [row for t, row in samples(logs) if end+frame+delta <= t+offset < new_at-100]
                require(recovered and all(0 <= float(r["image_age_ms"]) < 500 for r in recovered),
                        "Tiền đề: phải có ảnh mới thực sự trong cửa sổ giữ dừng")
                if mode == "calib" and image:
                    held_window(rows, "servo_angle", stop_by+delta, new_at-1, 7)
                if image and kind != "phase":
                    require(any(float(r["image_age_ms"]) >= 500 for r in logs),
                            "Thiếu bằng chứng ảnh cũ đã được quan sát")
                # State phát hiện có thể chỉ sống một tick, CSV theo ảnh dễ bỏ lỡ.
                # Bằng chứng chốt là vết motor ở trên + đúng danh sách stdout ở cuối ca.
                held_window(rows, "motor_speed", new_at+delta+100, new_at+1000, restart)
                csv_window(logs, offset, new_at+delta+100, new_at+1000, restart)
                if mode == "calib" and image:
                    held_window(rows, "servo_angle", new_at+delta+100, new_at+1000, 7)
                if twice:
                    start2, end2 = events[-1]
                    by2 = start2+(500+frame+cycle+100 if image or mixed else delta)
                    if mixed:
                        require(by2 < end2, "Tiền đề: stop_by < gap.second của sự cố ảnh thứ hai")
                        require(any(float(r["image_age_ms"]) >= 500 for r in logs),
                                "Hai nguyên nhân: thiếu bằng chứng mất ảnh ở sự cố thứ hai")
                    held_window(rows, "motor_speed", new_at+delta+100, start2-1, restart)
                    held_window(rows, "motor_speed", by2, finish_at-100, 0)
                    csv_window(logs, offset, by2+150, finish_at-100, 0)
            if kind == "phase":
                evidence.append(phase_evidence(rows, logs, offset, events[0]))
            self.finish(process)

        output, _ = self.execute(mode, config, folder, env, observe)
        reason = "mat_anh" if image else "phanh_tof_sau" if reverse else "phanh_tof"
        notifications(output, [] if passive else [reason, "mat_anh"] if mixed else [reason]*(2 if twice else 1))
        return bool(evidence and evidence[0])

    def startup(self, mode):
        # T13/J/S5: chưa từng có frame không được in chốt nếu không có lệnh để cắt.
        folder, config, env = self.setup()
        env["UCR_FAKE_CAMERA_GAPS"] = "0:1200"
        env["UCR_FAKE_KEYS"] = "4000:{}".format("y" if mode == "calib" else "w")

        def observe(process):
            rows = self.reach(process, folder, 5500)
            logs = log_rows(folder, mode)
            initial = [r for r in logs if int(r["frame_id"]) == 0]
            self.baseline = logs
            require(initial and all(float(r["image_age_ms"]) == -1 and r["state"] == "MAT_ANH" and
                                    near(float(r["cmd_speed"]), 0) for r in initial),
                    "J: chưa có ảnh phải frame=0/age=-1/MAT_ANH/lệnh 0")
            first_motor = next(ms for ms, k, _ in rows if k == "motor_speed")
            held_window(rows, "motor_speed", first_motor, 3900, 0)
            held_window(rows, "motor_speed", 4400, 5400, 10 if mode == "calib" else 5)
            self.finish(process)
        output, _ = self.execute(mode, config, folder, env, observe)
        notifications(output, [])

    def limit(self, kind):
        # T20/S4/H1/L: huỷ một lượt đúng một dòng, kể cả UI không đọc được state.
        if kind == "position":
            # C1: bước 5 bị khoá; giữ nguyên limit_deadline/limit_distance ở motor.
            from acc_stop_path import reject_position
            reject_position(self)
            return
        distance = kind != "deadline"
        folder, config, env = self.setup({"calib_max_duration_s": "30" if distance else "3",
                                          "calib_distance_cm": "30" if distance else "300",
                                          "calib_position_cm": "30"})
        env["UCR_FAKE_KEYS"] = "3500:d,4000:y,5200:x,11000:y"
        env["UCR_FAKE_UI_STALL"] = "5000:4500"

        def observe(process):
            rows = self.reach(process, folder, 4500)
            logs = log_rows(folder, "calib")
            self.baseline = logs
            cycle, _, _ = observed_timing(rows, logs, control_hz=10)
            delta = cycle+50+100
            moves = [ms for ms, k, v in rows if k == "motor_speed" and near(float(v), 10)]
            require(moves and 4000 <= moves[0] < 4300, "Tiền đề: y phải tạo lượt trước hạn")
            t0 = moves[0]
            offset = offset_ms(rows, logs, 10)
            moving = [r for r in logs if near(float(r["cmd_speed"]), 10)]
            require(len(moving) >= 3 and float(moving[-1]["odom_cm"])-float(moving[0]["odom_cm"]) > 1,
                    "Tiền đề: encoder giả phải tích phân khi chuyển động")
            stop_by = t0+3000+delta
            require(5000+cycle < 5200 < t0+3000-500 and 9500 >= stop_by+1000,
                    "Tiền đề: UI chặn phủ hạn +1 s, x không được xử lý trước hạn")
            # L/H1: UI có lớp kiểm hạn riêng; chỉ buộc Actuator in chốt ở hạn bị chặn.
            # Lượt y mới được quan sát trước hạn riêng, nên ca này chỉ có một lần hết hạn.
            rows = self.reach(process, folder, 12300)
            held_window(rows, "motor_speed", 5250, t0+2400, 10)
            held_window(rows, "motor_speed", stop_by, 10900, 0)
            held_window(rows, "servo_angle", stop_by, 10900, 7)
            csv_window(log_rows(folder, "calib"), offset, stop_by+150, 10900, 0)
            if distance:
                stopped_rows = [r for t, r in samples(log_rows(folder, "calib"))
                                if t0+2000 <= t+offset <= stop_by and near(float(r["cmd_speed"]), 0)]
                require(stopped_rows, "Thiếu CSV dừng do quãng")
                start = float(moving[0]["odom_cm"])
                traveled = float(stopped_rows[0]["odom_cm"])-start
                tolerance = 10*(cycle+50)/1000.0+1
                require(30-tolerance <= traveled <= 30+tolerance,
                        "S4: dừng sai quãng, delta_cm={} dung_sai={}".format(traveled, tolerance))
            held_window(rows, "motor_speed", 11000+delta+100, 12000, 10)
            if not distance:
                second = [ms for ms, k, v in rows if ms >= 11000 and k == "motor_speed" and near(float(v), 10)]
                require(second, "y mới phải tạo lượt mới")
                require(rows[-1][0] < second[0]+3000,
                        "Tiền đề: kết thúc quan sát trước hạn lượt mới")
            self.finish(process)
        output, _ = self.execute("calib", config, folder, env, observe, "motor")
        notifications(output, ["gioi_han_calib"])

    def automatic(self, kind):
        # T09/T10/S2: auto không chốt ToF/ảnh, không cần thao tác bàn phím mới.
        folder, config, env = self.setup({"resume_stable_s": "1", "soft_start_s": "1"})
        (folder / "runs").mkdir()
        seed_gate(self.main, config, folder / "runs")
        image = kind == "image"
        reverse = kind == "rear"
        if reverse:
            env["ACC_T05G_REVERSE"] = "1"
        else:
            env.pop("ACC_T05G_REVERSE", None)
        if image:
            env["UCR_FAKE_CAMERA_GAPS"] = "5500:8500"
        else:
            bad = "2000 100 2000 2000" if reverse else "100 2000 2000 2000"
            path = folder / "tof.txt"
            path.write_text("0 2000 2000 2000 2000\n5500 {}\n5800 2000 2000 2000 2000\n".format(bad), encoding="ascii")
            env["UCR_FAKE_TOF_SCRIPT"] = str(path)

        def observe(process):
            rows = self.reach(process, folder, 4500)
            speed = -10 if reverse else 10
            held_window(rows, "motor_speed", 4000, 4400, speed)
            cycle, frame, _ = observed_timing(rows, log_rows(folder, "auto"), control_hz=10)
            delta = cycle+20+100
            end = 11200 if image else 7500
            rows = self.reach(process, folder, end)
            logs = log_rows(folder, "auto")
            if image:
                require(any(r["state"] == "MAT_ANH" and near(float(r["cmd_speed"]), 0) for r in logs),
                        "Auto ngắt dài phải ghi MAT_ANH/lệnh 0")
                require(5500+500+frame+delta < 8500, "Tiền đề: stop_by < gap.second")
                held_window(rows, "motor_speed", 5500+500+frame+delta, 8400, 0)
                require(any(r["state"] == "DUNG_CHO_ANH_ON_DINH" and near(float(r["cmd_speed"]), 0) for r in logs),
                        "Auto ngắt dài phải chờ ảnh ổn định")
                offset = offset_ms(rows, logs, speed)
                fresh = [(t+offset, t+offset-float(r["image_age_ms"])) for t, r in samples(logs)
                         if t+offset-float(r["image_age_ms"]) >= 8500 and 0 <= float(r["image_age_ms"]) < 500]
                require(fresh, "Tiền đề auto: thiếu captured_at phục hồi thực tế")
                sampled, captured = fresh[0]
                wait_until = captured+1000-frame-100
                require(wait_until-sampled-cycle >= 300, "Tiền đề auto: không đủ cửa sổ chờ ảnh ổn định")
                held_window(rows, "motor_speed", sampled+cycle, wait_until, 0)
                held_window(rows, "motor_speed", 8500+frame+2000+delta, end-100, speed)
            else:
                held_window(rows, "motor_speed", 5500+delta, 5790, 0)
                held_window(rows, "motor_speed", 5800+delta, end-100, speed)
            self.finish(process)
        output, _ = self.execute("auto", config, folder, env, observe, auto=True)
        notifications(output, [])

    def smoke(self):
        # S2/E5: smoke stage motor mất ảnh khi UI bị chặn vẫn dừng, không in chốt L.
        from acc_review_fixes import ReviewSuite
        original = self.main
        suite = ReviewSuite(self.root, original, self.stop, self.folder)
        suite.smoke_stall("image")
        for path in self.folder.glob("*/output.txt"):
            notifications(read(path), [])


def docs(root):
    # T22: câu cố định đúng file/mục; chạy trên laptop vì remote chỉ chép ucr_car.
    root = Path(root)
    phrase = "xe dừng do an toàn (mất ảnh, ToF) thì phải bấm lại phím mới chạy"
    car = read(root / "ucr_car.md")
    require(phrase in car, "ucr_car.md thiếu câu hướng dẫn cố định")
    paragraphs = [p for p in car.split("\n\n") if "AN_TOAN_CHOT: <ly_do>" in p]
    require(paragraphs, "ucr_car.md thiếu mẫu AN_TOAN_CHOT: <ly_do>")
    for reason in REASONS:
        require(any("`"+reason+"`" in p for p in paragraphs), "ucr_car.md thiếu lý do AN_TOAN_CHOT " + reason)
    runbook = read(root / "day1_runbook.md")
    for section in ("F", "H"):
        match = re.search(r"^## "+section+r"\. .*?(?=^## |\Z)", runbook, re.M | re.S)
        require(match and phrase in match.group(0), "day1_runbook thiếu câu ở mục " + section)
    require(phrase in read(root / "team_collect_guide.md"), "team_collect_guide thiếu câu cố định")
    print("ĐẠT T22: chuỗi hướng dẫn và bốn lý do ở đúng file/mục")


def self_test():
    # Tự kiểm bộ phán xét bằng dữ liệu giả sai có chủ ý; không phải chạy mã xe.
    from copy import deepcopy
    good = [(0, "motor_speed", "10"), (100, "motor_speed", "0"), (300, "servo_angle", "7")]
    held_window(good, "motor_speed", 100, 300, 0)
    notifications("AN_TOAN_CHOT: mat_anh\n", ["mat_anh"])
    logs = []
    rows = []
    # 2 frame liên tiếp cách 550 ms, tick cuối trước phục hồi chỉ thấy tuổi 450.
    for tick in range(4900, 5901, 100):
        captured = 4950 if 5000 <= tick < 5500 else 5500 if tick >= 5500 else 4850
        fid = 2 if 5000 <= tick < 5500 else 3 if tick >= 5500 else 1
        logs.append({"t": str(tick/1000.0), "image_age_ms": str(tick-captured),
                     "frame_id": str(fid), "state": "LAI_TAY", "cmd_speed": "10"})
        rows.append((tick, "servo_angle", "7"))
    require(phase_evidence(rows, logs, 0, (5000, 5500)), "Tự kiểm pha không thấy tuổi cũ")
    bad_logs = deepcopy(logs)
    bad_logs[5]["image_age_ms"] = "500"
    require(not phase_evidence(rows, bad_logs, 0, (5000, 5500)), "Phải phát hiện tick thấy ảnh cũ")
    failures = [lambda: notifications("AN_TOAN_CHOT: mat_anh\n"*2, ["mat_anh"]),
                lambda: notifications("AN_TOAN_CHOT: phanh_tof\n", []),
                lambda: held_window(good, "motor_speed", 50, 300, 0),
                lambda: require(phase_evidence(rows+[(5460, "servo_angle", "7")], logs, 0, (5000, 5500)),
                                      "tick không có mẫu CSV, cận tuổi >=500"),
                lambda: csv_window(logs, 0, 5000, 5500, 0)]
    for check in failures:
        try:
            check()
        except AssertionError:
            continue
        raise AssertionError("Bộ phán xét bỏ lọt mẫu sai")
    require(not near(math.inf, 0) and not near(math.nan, 0), "Không chấp nhận NaN/Inf")
    # Kiểm cửa sổ ở cả nhịp Ubuntu và container 60 ms; không cần sleep/mã xe.
    for cycle, frame in ((100, 40), (104, 60), (104, 125), (125, 150)):
        require(5500+500+frame+cycle+100 < 6700, "stop_by phải còn trong gap 1200 ms")
        require(5500+cycle+20+100 < 5800, "Phải bắt ToF 300 ms trước khi hết sự cố")
    # Nhịp >150 ms hợp lệ dưới tải ASan; kiểm chính hàm đo, gồm cận và lỗi thiết lập.
    for want_cycle, want_frame, valid in ((160, 200, True), (300, 250, True),
                                         (301, 200, False), (160, 251, False)):
        hw = [(ms, "servo_angle", "7") for ms in [1000]+list(range(1000+want_cycle, 3301, 100))]
        measured_logs = [{"frame_id": str(i+1), "image_age_ms": "10",
                          "t": str((1000+i*want_frame)/1000.0)} for i in range(12)]
        try:
            cycle, frame, _ = observed_timing(hw, measured_logs, control_hz=10)
        except AssertionError as error:
            require(not valid and "cycle=" in str(error) and "frame=" in str(error),
                    "Lỗi thiết lập phải có nhịp đo được")
        else:
            require(valid and near(cycle, want_cycle) and near(frame, want_frame),
                    "Sai ngưỡng/giá trị nhịp đo được")
            require(5500+500+frame+cycle+100 < 6700,
                    "Nhịp chậm hợp lệ vẫn phải dừng trong gap 1200 ms")
    # Khung chậm: ghi chú, không lỗi; khung nhanh: 0 lọt vẫn là lỗi thiết lập.
    phase_verdict([False]*3, [124.0, 206.0])
    phase_verdict([True, False], [127.0])
    try:
        phase_verdict([False]*3, [127.0, 150.0])
    except AssertionError:
        pass
    else:
        raise AssertionError("Khung <=150 ms mà 0 pha lọt phải là lỗi thiết lập")
    # T20: một hạn dưới UI chặn, y mới chạy lại nhưng chưa tới hạn thứ hai.
    from unittest.mock import Mock, patch
    limit_rows = [(4000, "motor_speed", "10"), (4000, "servo_angle", "17"),
                  (7000, "motor_speed", "0"), (7000, "servo_angle", "7"),
                  (11000, "motor_speed", "10"), (12300, "servo_angle", "7")]
    limit_logs = [{"t": str(t), "cmd_speed": str(speed), "odom_cm": str(odom),
                   "state": state} for t, speed, odom, state in
                  ((4, 10, 0, "CHAY"), (4.1, 10, 1, "CHAY"), (4.2, 10, 2, "CHAY"),
                   (8, 0, 30, "DUNG_GIOI_HAN_CALIB"), (9, 0, 30, "DUNG"), (10, 0, 30, "DUNG"))]
    with tempfile.TemporaryDirectory(prefix="acc_t05g_limit_self_") as folder:
        suite = LatchSuite(".", "main-gia", "stop-gia", folder)
        suite.reach = Mock(return_value=limit_rows)
        suite.finish = Mock()

        def execute(mode, config, directory, env, observe, step):
            observe(None)
            return "AN_TOAN_CHOT: gioi_han_calib\n", limit_logs

        suite.execute = execute
        with patch.object(sys.modules[__name__], "log_rows", return_value=limit_logs), \
                patch.object(sys.modules[__name__], "observed_timing", return_value=(100, 200, 300)):
            suite.limit("deadline")
        require(suite.reach.call_args[0][-1] == 12300 and suite.finish.call_count == 1,
                "T20 phải kết thúc sau khởi động lại, trước hạn thứ hai")
    print("ĐẠT: tự kiểm vết/CSV/thông báo/pha/lịch container và nhịp ASan; bắt 5 mẫu sai")


def main():
    require(len(sys.argv) >= 2, "Cần tên ca")
    scenario = sys.argv[1]
    if scenario == "self-test":
        self_test()
        return
    if scenario == "docs":
        docs(sys.argv[2])
        return
    require(len(sys.argv) == 5, "Cần scenario root main ucr_stop")
    with tempfile.TemporaryDirectory(prefix="acc_t05g_") as folder:
        suite = LatchSuite(sys.argv[2], sys.argv[3], sys.argv[4], folder)
        suite.fake_only()
        if scenario == "require_fake":
            raise AssertionError("Cần build UCR_FAKE_HW=ON để chạy T05g; không bỏ qua")
        if scenario.startswith("auto_"):
            suite.automatic(scenario[5:])
        elif scenario == "smoke":
            suite.smoke()
        elif scenario.startswith("limit_"):
            suite.limit(scenario[6:])
        else:
            mode, kind = scenario.split("_", 1)
            if kind == "startup":
                suite.startup(mode)
            elif kind == "phase":
                result = [suite.event(mode, "phase", phase=p) for p in PHASES]
                phase_verdict(result, suite.frames)
            else:
                # Nhận nguyên tên nguyên nhân trước khi tách hậu tố (front_reverse là đối chứng).
                bases = ("rear_forward_invalid", "rear_forward", "front_reverse", "rear_invalid",
                         "short", "image", "long", "stale", "front", "rear", "idle")
                base = next((b for b in bases if kind == b or kind.startswith(b+"_")), None)
                require(base, "Tên nguyên nhân không hợp lệ: " + kind)
                options = kind[len(base):].split("_")
                flags = {name: name in options for name in ("stalled", "opposite", "reverse", "twice", "mixed")}
                suite.event(mode, base, **flags)
    print("ĐẠT " + scenario)


if __name__ == "__main__":
    main()
