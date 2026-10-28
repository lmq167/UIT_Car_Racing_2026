#!/usr/bin/env python3
"""Hiệu chuẩn/an toàn CFG03–09/11, C01–03/13/14, S01/02/14, A01/02.

Chỉ gọi giao diện CLI/tài liệu; không đọc mã .cpp. Python 3.6, thư viện chuẩn.
Tiến trình con có hạn giờ và được thu hồi cả nhóm kể cả khi test lỗi.
"""
import csv
import io
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import time
from acc_seed_gate import FIELDS, gate_data, seed_gate

# Chỉ là hạn chờ tiến trình/khởi động, không thay các mốc an toàn hoặc tăng mềm.
STARTUP_TIMEOUT = 15
EXIT_TIMEOUT = 15
SPEED_EPS = 1e-3
STEPS = {"env": 1, "servo": 2, "radius": 3, "motor": 4, "position": 5,
         "imu": 6, "tof": 7, "camera": 8, "latency": 9, "timing": 10}

# F/CFG12–14: danh mục độc lập từ Config.h, config.yaml và bảng khóa docs/ucr_car.md.
# Mỗi mục: mẫu hợp lệ, mẫu ngoài miền, biểu thức miền phải có trong lỗi.
POSITIVE = r"(?:>\s*0\b|\(\s*0\s*,)"
INTEGER_POSITIVE = r"(?:>=\s*1\b|≥\s*1\b|>\s*0\b|\[\s*1\s*,)"
CHECK_RULES = {}
for _key, _good in (("camera_hz", "25"), ("odometry_hz", "10"), ("ui_hz", "10"),
                    ("tof_brake_mm", "300"), ("imu_drift_ms", "100"), ("image_every_n", "2"),
                    ("image_queue_size", "4"), ("speed_filter_size", "3"),
                    ("keyboard_steering_step_deg", "3"), ("fps_report_ms", "1000"),
                    ("collect_confirm_ms", "1000"), ("camera_width", "320"), ("camera_height", "240")):
    CHECK_RULES[_key] = (_good, "0", INTEGER_POSITIVE)
for _key in ("sensor_hz", "control_hz"):
    CHECK_RULES[_key] = ("25", "9", r"(?:>=\s*10\b|≥\s*10\b|\[\s*10\s*,)")
for _key, _good in (("soft_start_s", "0.7"), ("resume_stable_s", "0.7"),
                    ("keyboard_speed_step_cms", "3"), ("collect_min_speed_cms", "2"),
                    ("smoke_min_fps", "5"), ("smoke_stop_speed_cms", "0.5"),
                    ("calib_speed_cms", "15"), ("calib_distance_cm", "100"),
                    ("calib_position_cm", "25"), ("calib_max_duration_s", "20")):
    CHECK_RULES[_key] = (_good, "0", POSITIVE)
for _key in ("servo_invert", "tof_safety_enabled", "enable_oled", "enable_display", "tof_mapping_confirmed"):
    CHECK_RULES[_key] = ("0", "2", r"(?:[\[{]\s*0\s*,\s*1\s*[\]}]|0\s*(?:hoặc|hoac|/)\s*1)")
for _key in ("tof_timeout_ms", "image_timeout_ms"):
    CHECK_RULES[_key] = ("200", "501", r"\[\s*1\s*,\s*500\s*\]")
for _key in ("yaw_axis", "pitch_axis", "roll_axis"):
    CHECK_RULES[_key] = ("0", "3", r"[\[{]\s*0\s*,\s*(?:1\s*,\s*)?2\s*[\]}]")
for _key in ("yaw_sign", "pitch_sign", "roll_sign"):
    CHECK_RULES[_key] = ("-1", "0", r"(?:\{\s*-1\s*,\s*\+?1\s*\}|±\s*1|[+-]\s*1\s*/\s*[+-]\s*1)")
CHECK_RULES.update({
    "soft_start_s": ("0.7", "0.199", r"\[\s*0\.2\s*,\s*10\s*\]"),
    "resume_stable_s": ("0.7", "30.001", r"\[\s*0\.5\s*,\s*30\s*\]"),
    "max_speed_cms": ("100", "100.001", r"\(\s*0\s*,\s*100\s*\]"),
    "camera_resume_max_s": ("3", "0.5", r"(?:>\s*(?:image_timeout_ms\s*/\s*1000|0\.5)|\(\s*0\.5\s*,)"),
    "max_steering_deg": ("15", "61", r"(?:60|servo_trim_deg)"),
    "servo_trim_deg": ("5", "61", r"(?:60|max_steering_deg)"),
    "jpeg_quality": ("80", "101", r"\[\s*1\s*,\s*100\s*\]"),
    "motor_keepalive_ms": ("100", "201", r"\[\s*1\s*,\s*200\s*\]"),
    "collect_fps": ("5", "11", r"\[\s*1\s*,\s*10\s*\]"),
    "collect_duplicate_threshold": ("2", "-1", r"(?:>=\s*0\b|≥\s*0\b|\[\s*0\s*,)"),
    "smoke_servo_deg": ("5", "61", r"\[\s*1\s*,\s*60\s*\]"),
    "calib_servo_step_deg": ("3", "0", INTEGER_POSITIVE),
    "calib_servo_limit_deg": ("25", "61", r"(?:60|\[\s*1\s*,)"),
    "camera_pipeline": ('"pipeline-kiem-tra-khong-duoc-mo"', None, r"(?:chuỗi|chuoi|string)"),
    "fake_source": ('"nguon-khong-ton-tai"', None, r"(?:chuỗi|chuoi|string)"),
    "imu_calibration_file": ('"imu-khong-ton-tai"', None, r"(?:chuỗi|chuoi|string)"),
    "log_dir": ('"khong-duoc-tao"', None, r"(?:chuỗi|chuoi|string)"),
    "tof_names": ("[rear, left, right, front]", "[front, front, left, right]", r"(?:front|rear|left|right)"),
    "collect_keys": ('["i","k","j","l"," ","c","u","esc","[","]","+","-"]',
                     '["w","w","a","d"," ","c","u","esc","[","]","+","-"]', r"(?:12|ASCII|ascii|khác|khac|trùng|trung)"),
    "collect_luot_keys": ('"987654321"', '"111111111"', r"(?:9|khác|khac|trùng|trung)"),
})


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def read(path):
    return Path(path).read_text(encoding="utf-8")


def trace(path, live=False):
    if not path.exists():
        return []
    rows = []
    text = read(path)
    # Khi thăm dò, dòng cuối có thể đang được ghi; kiểm cuối phiên vẫn đọc nghiêm ngặt.
    if live and text and not text.endswith("\n"):
        text = text.rsplit("\n", 1)[0] if "\n" in text else ""
    for line in text.splitlines():
        parts = line.split(",", 2)
        require(len(parts) == 3, "Vết sai định dạng: " + line)
        rows.append((int(parts[0]), parts[1], parts[2]))
    return rows


def tail(rows, trim=7):
    for kind, want in [("motor_speed", 0), ("servo_angle", trim)]:
        values = [float(v) for _, k, v in rows if k == kind]
        require(values and math.isfinite(values[-1]) and abs(values[-1] - want) <= SPEED_EPS,
                "Lệnh cuối {} phải bằng {}".format(kind, want))


def poll(predicate, seconds):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        if predicate():
            return True
        time.sleep(0.01)
    return bool(predicate())


def diagnostic(text):
    # CTest tự mã hoá dấu XML; chỉ thoát ký tự điều khiển XML 1.0 không chấp nhận.
    return re.sub(r"[\x00-\x08\x0b\x0c\x0e-\x1f]", lambda m: "\\x{:02x}".format(ord(m[0])), text)


def log_ready(folder, mode):
    for path in (folder / "runs").glob("*_" + mode + "/log.csv"):
        text = read(path)
        text = text[:text.rfind("\n") + 1]
        for row in csv.DictReader(io.StringIO(text)):
            if None in row or any(value is None for value in row.values()):
                continue
            if (float(row["t"]) >= 0.1 and int(row["frame_id"]) > 0 and
                    float(row["image_age_ms"]) <= 500 and
                    all(float(row["tof" + str(i)]) > 0 for i in range(4))):
                return True
    return False


def log_rows(folder, mode="auto"):
    rows = []
    for path in (folder / "runs").glob("*_" + mode + "/log.csv"):
        text = read(path)
        text = text[:text.rfind("\n") + 1]
        rows.extend(row for row in csv.DictReader(io.StringIO(text))
                    if None not in row and all(value is not None for value in row.values()))
    return rows


def collect_metadata(process):
    # Sáu mục theo docs: mã, họ tên, thời tiết, rèm, đèn, đạo cụ; giữ stdin mở sau khi nhập.
    process.stdin.write("a\nNguoi kiem thu\n1\n1\n1\nkhông\n".encode("utf-8"))
    process.stdin.flush()


def held_at(rows, kind, at):
    values = [float(v) for ms, k, v in rows if k == kind and ms <= at]
    require(values, "Thiếu vết {} trước mốc {}".format(kind, at))
    return values[-1]


def check_collect_trace(rows, gap_end, key_at):
    # Đọc cả lệnh đang giữ giữa hai lần gửi, không chỉ các dòng mới hoặc lệnh dừng khi dọn test.
    require(rows and rows[-1][0] >= key_at + 120, "C': phải quan sát qua hạn xử lý phím mới")
    require(key_at - gap_end > 3000, "C': giữ ảnh mới không phím lâu hơn W+S")
    speeds = [(ms, float(v)) for ms, k, v in rows if k == "motor_speed"]
    require(all(math.isfinite(v) and abs(v) <= 25 + SPEED_EPS for _, v in speeds), "C': motor hữu hạn, không vượt 25")
    require(any(1500 <= ms < 3000 and abs(v - 20) <= SPEED_EPS for ms, v in speeds) and
            abs(held_at(rows, "motor_speed", 2800) - 20) <= SPEED_EPS, "C': w đầu phải cho +20 trước mất ảnh")
    require(abs(held_at(rows, "motor_speed", 3610)) <= SPEED_EPS and
            abs(held_at(rows, "servo_angle", 3610) - 7) <= SPEED_EPS, "C': tới timeout + một chu kỳ +100 ms phải 0/trim")
    require(all(abs(v) <= SPEED_EPS for ms, v in speeds if 3610 <= ms < key_at) and
            abs(held_at(rows, "motor_speed", key_at - 1)) <= SPEED_EPS, "C': ảnh cũ/ảnh về vẫn 0 tới w mới, không ghi nhớ phím cũ")
    require(abs(held_at(rows, "motor_speed", key_at + 120) - 20) <= SPEED_EPS and
            all(abs(v - 20) <= SPEED_EPS for ms, v in speeds if ms >= key_at + 120),
            "C': w mới chạy +20 ngay, không chờ W/tăng mềm hoặc cộng lại lệnh đã xóa")


def wait_ready(process, predicate, message):
    require(poll(lambda: predicate() or process.poll() is not None, STARTUP_TIMEOUT), message)
    require(process.poll() is None and predicate(), message + ": tiến trình đã thoát hoặc chưa sẵn sàng")


def kill_group(process):
    # Kể cả cha đã thoát, nhóm của nó có thể còn tiến trình con giữ tệp/thiết bị.
    try:
        if os.name == "posix":
            os.killpg(process.pid, signal.SIGKILL)
        elif process.poll() is None:
            process.kill()
    except ProcessLookupError:
        pass
    process.wait(timeout=3)


def run(command, folder, env=None, observe=None, timeout=EXIT_TIMEOUT):
    folder.mkdir(parents=True, exist_ok=True)
    with (folder / "output.txt").open("wb") as out, (folder / "error.txt").open("wb") as err:
        process = subprocess.Popen([str(x) for x in command], cwd=str(folder), env=env,
                                   stdin=subprocess.PIPE, stdout=out, stderr=err,
                                   start_new_session=True)
        try:
            if observe:
                observe(process)
            code = process.wait(timeout=timeout)
        except (AssertionError, subprocess.TimeoutExpired):
            output = b"\n".join((folder / name).read_bytes() for name in ("output.txt", "error.txt"))
            output = output.decode("utf-8", errors="backslashreplace")
            print(diagnostic("Đầu ra tiến trình con (cuối):\n" + output[-2000:]), file=sys.stderr)
            raise
        finally:
            kill_group(process)
            process.stdin.close()
    return code, read(folder / "output.txt") + "\n" + read(folder / "error.txt")


class Suite:
    def __init__(self, root, main, stop, folder):
        self.root = Path(root).resolve()
        self.main = Path(main).resolve()
        self.stop = Path(stop).resolve()
        self.folder = Path(folder).resolve()
        self.number = 0

    def setup(self, overrides=None, remove=(), gate=False):
        self.number += 1
        folder = self.folder / str(self.number)
        folder.mkdir()
        logs = folder / "runs"
        logs.mkdir()
        values = {"log_dir": json.dumps(str(logs)), "fake_source": '""',
                  "enable_display": "0", "enable_oled": "0", "servo_trim_deg": "7"}
        values.update(overrides or {})
        lines = [line for line in read(self.root / "config.yaml").splitlines()
                 if line.split(":", 1)[0] not in set(values).union(remove)]
        text = "\n".join(lines) + "\n"
        text += "".join("{}: {}\n".format(k, v) for k, v in values.items())
        config = folder / "config.yaml"
        config.write_text(text, encoding="utf-8")
        env = dict(os.environ)
        for key in ("UCR_FAKE_FAULT", "UCR_FAKE_CAMERA_GAPS", "UCR_FAKE_TOF_SCRIPT", "UCR_FAKE_KEYS"):
            env.pop(key, None)
        env["UCR_FAKE_HW_TRACE"] = str(folder / "trace.csv")
        if gate:
            seed_gate(self.main, config, logs)
        return folder, config, env

    def smoke_args(self, folder):
        # Cùng tiền điều kiện do smoke_test.sh chuẩn bị; test không xác nhận bánh kê.
        marker = folder / "smoke.marker"
        marker.write_text("", encoding="utf-8")
        return ["--smoke-kill-file", marker]

    def reject(self, key, value, wrapper=False, remove=()):
        folder, config, env = self.setup({key: value} if key else {}, remove)
        if not key:
            if value == "missing":
                config.unlink()
            else:
                config.write_text("%YAML:1.0\n---\n[not: valid\n", encoding="utf-8")
        cmd = [self.main, "--config", config, "--mode", "manual"]
        if wrapper:
            build = folder / "build"
            build.mkdir()
            for name, binary in [("main", self.main), ("ucr_stop", self.stop)]:
                (build / name).symlink_to(binary)
            env["UCR_BUILD_DIR"] = str(build)
            cmd = ["bash", self.root / "scripts/run.sh", "--config", config, "--mode", "manual"]
        label = "{}={}".format(key or "file", value)
        try:
            code, output = run(cmd, folder, env)
        except subprocess.TimeoutExpired as error:
            raise AssertionError(label + ": cấu hình cố ý sai phải bị từ chối trước thiết bị; "
                                 "hết hạn chờ không phải kết quả đạt") from error
        require(0 < code < 124, label + ": phải thoát lỗi chủ động, nhận " + str(code))
        require(bool(output.strip()), label + ": phải nêu lỗi")
        if key:
            require(key in output, label + ": thông báo thiếu tên khóa")
        if key == "max_speed_cms" and value in ("0", "-1", "100.001", "101"):
            require("(0, 100]" in output, label + ": thiếu khoảng hợp lệ")
        if key in ("resume_stable_s", "soft_start_s", "camera_resume_max_s"):
            require(re.search(CHECK_RULES[key][2], output, re.IGNORECASE),
                    label + ": thiếu khoảng hợp lệ của khóa phục hồi")
        if key in ("image_timeout_ms", "tof_timeout_ms") and value in ("0", "-1", "501"):
            errors = read(folder / "error.txt")
            require(key in errors and re.search(r"\[\s*1\s*,\s*500\s*\]", errors),
                    label + ": stderr phải có khóa và khoảng [1,500]")
        rows = trace(folder / "trace.csv")
        if wrapper:
            require(len(rows) == 2, "CFG11: chỉ hai lệnh dừng, main không được khởi tạo")
            require(not any(k in ("motor_cfg", "motor_init") for _, k, _ in rows), "CFG11: dừng không cấu hình motor")
            tail(rows)
        else:
            require(not rows, label + ": phải từ chối trước mọi thiết bị")
            require(not list((folder / "runs").iterdir()), label + ": không tạo lượt chạy")

    def config_numeric(self):
        # CFG03/04: chuỗi số vẫn sai kiểu; không hữu hạn/tràn không được thành mặc định.
        bad_types = ['"25"', '"abc"', '""', "[]", "{}", "", "null", ".Nan", ".Inf", "-.Inf", "1e300"]
        for key in ("max_speed_cms", "camera_resume_max_s", "soft_start_s", "resume_stable_s"):
            for value in bad_types:
                self.reject(key, value)
        for value in ("0", "-1", "100.001", "101"):
            self.reject("max_speed_cms", value)
        for value in ("-1", "0", "0.499", "0.5"):
            self.reject("camera_resume_max_s", value)
        for value in ("-1", "0"):
            self.reject("soft_start_s", value)
            self.reject("resume_stable_s", value)
        # CFG05: quan hệ theo timeout=200, không viết cứng 0.5.
        folder, config, env = self.setup({"image_timeout_ms": "200", "camera_resume_max_s": "0.2"})
        code, output = run([self.main, "--config", config, "--mode", "manual"], folder, env)
        require(0 < code < 124 and "camera_resume_max_s" in output and not trace(folder / "trace.csv"),
                "CFG05: R bằng timeout thực phải bị từ chối trước thiết bị")

    def config_keys(self):
        # CFG08/09: cờ số nguyên, file và các khóa an toàn cũ.
        for value in ("-1", "2", "0.0", "1.0", "0.5", "true", "false", '"true"', '"false"',
                      '"0"', '"1"', '"abc"', '""', "[]", "{}", "null"):
            self.reject("tof_mapping_confirmed", value)
        for key in ("image_timeout_ms", "tof_timeout_ms"):
            for value in ("0", "-1", "501"):
                self.reject(key, value)
        for value in ("[front, rear, left]", "[front, front, left, right]", "[rear, left, right, nowhere]"):
            self.reject("tof_names", value)
        for value in ("missing", "malformed"):
            self.reject(None, value)

    def wrapper_config(self):
        # CFG11: lỗi từng khóa mới vẫn gọi dừng độc lập qua run.sh.
        for key, values in [("camera_resume_max_s", ['"2"', "0.5"]),
                            ("soft_start_s", ['"1"', "0"]),
                            ("resume_stable_s", ['"2"', "0", "0.499", "30.001"]),
                            ("tof_mapping_confirmed", ['"true"', "2"])]:
            for value in values:
                self.reject(key, value, wrapper=True)

    def check_config(self, scenario):
        # F: dùng cùng lệnh kiểm cho mọi khóa, không cần smoke và không được mở thiết bị.
        published = set(re.findall(r"(?m)^([a-z][a-z0-9_]*):", read(self.root / "config.yaml")))
        require(published == set(CHECK_RULES), "CFG13: bảng kiểm phải bao phủ đúng mọi khóa công bố: " +
                str(published.symmetric_difference(CHECK_RULES)))

        def prepare(overrides=None, minimal=False):
            values = {"camera_pipeline": '"pipeline-khong-duoc-mo"', "fake_source": '"nguon-khong-ton-tai"'}
            values.update(overrides or {})
            folder, config, env = self.setup(values)
            (folder / "runs").rmdir()
            destination = folder / "cau hinh co khoang trang.yaml"
            config.rename(destination)  # Python 3.6 đổi tên thành công nhưng trả None.
            config = destination
            if minimal:
                config.write_text("%YAML:1.0\n---\n", encoding="utf-8")
            (folder / "giu-nguyen.txt").write_text("doi-chung", encoding="utf-8")
            return folder, config, env

        def unchanged(folder):
            require(not trace(folder / "trace.csv"), "CFG17: không có vết mở/khởi tạo thiết bị")
            require(not (folder / "runs").exists() and not (folder / "khong-duoc-tao").exists() and
                    not (self.folder / "runs").exists(), "CFG17: không tạo log_dir/runs, kể cả mặc định")
            require(read(folder / "giu-nguyen.txt") == "doi-chung", "CFG17: giữ nguyên nội dung đã có")
            require({p.name for p in folder.iterdir()} <= {"cau hinh co khoang trang.yaml", "giu-nguyen.txt", "trace.csv", "output.txt", "error.txt"},
                    "CFG17: không tạo file/thư mục ngoài đầu ra của khung kiểm")

        def check(overrides, key=None, domain=None, valid=False, minimal=False, related_key=None):
            folder, config, env = prepare(overrides, minimal)
            code, output = run([self.main, "--check-config", "--config", config], folder, env)
            label = "CFG12/13 " + str(overrides)
            stdout, stderr = read(folder / "output.txt"), read(folder / "error.txt")
            require(code == (0 if valid else 2), label + ": mã thoát phải đúng 0/2")
            if valid:
                require(stdout.splitlines() == ["CAU_HINH_OK: " + str(config)], label + ": stdout phải đúng một dòng hợp đồng")
                require(not re.search(r"ngoài khoảng hợp lệ|khoá lạ:|không đọc được:|\[lỗi\]|\berror\b", stderr, re.I),
                        label + ": stderr không chứa thông báo lỗi (được có cảnh báo thiếu khóa)")
            else:
                shown = re.search(re.escape(key) + r"\s*=\s*(.+?)\s+ngoài khoảng hợp lệ\s+\S", stderr)
                require(shown,
                        label + ": stderr phải nêu khóa = giá trị ngoài khoảng hợp lệ")
                raw = overrides[key].strip()
                if raw.startswith('"') and raw.endswith('"') and len(raw) > 2:
                    require(raw[1:-1] in shown.group(1), label + ": lỗi sai kiểu phải có giá trị chuỗi đã truyền")
                if re.fullmatch(r"-?\d+(?:\.\d+)?", raw):
                    try:
                        reported = float(shown.group(1).strip('" '))
                    except ValueError:
                        reported = float("nan")
                    require(math.isfinite(reported) and abs(reported-float(raw)) <= 1e-3,
                            label + ": lỗi ngoài miền phải nêu đúng giá trị số")
                if domain:
                    require(re.search(domain, stderr, re.IGNORECASE), label + ": lỗi ngoài miền thiếu khoảng đúng")
                if related_key:
                    require(related_key in stderr, label + ": lỗi ràng buộc chéo phải nhắc tên khóa " + related_key)
                require("CAU_HINH_OK:" not in stdout, label + ": không báo thành công khi lỗi")
            unchanged(folder)

        if scenario == "check_config_valid":
            check({}, valid=True)
            check({}, valid=True, minimal=True)
            for key, (good, _, _) in CHECK_RULES.items():
                check({key: good}, valid=True)
        elif scenario == "check_config_types":
            for key, (good, _, domain) in CHECK_RULES.items():
                # Kiểu sai cho cả số, chuỗi, danh sách; số nguyên thử thêm phân số.
                for bad in ('"25"', "[]", "{}", "null") if not good.startswith(('"', '[')) else ("25", "{}"):
                    check({key: bad}, key)
                if key.endswith(("_hz", "_deg", "_ms", "_axis")) or key in ("tof_brake_mm", "jpeg_quality", "image_every_n", "image_queue_size", "speed_filter_size", "collect_fps", "camera_width", "camera_height"):
                    check({key: "1.5"}, key)
                if key in ("servo_invert", "tof_safety_enabled", "enable_oled", "enable_display", "tof_mapping_confirmed"):
                    check({key: "1.0"}, key)
        elif scenario == "check_config_ranges":
            for key, (_, bad, domain) in CHECK_RULES.items():
                if bad is not None:
                    check({key: bad}, key, domain)
            check({"image_timeout_ms": "200", "camera_resume_max_s": "0.2"}, "camera_resume_max_s",
                  r"(?:>\s*(?:image_timeout_ms\s*/\s*1000|0\.2)|\(\s*0\.2\s*,)")
            check({"max_steering_deg": "20", "servo_trim_deg": "50"}, "max_steering_deg",
                  r"\[\s*0\s*,\s*10\s*\]", related_key="servo_trim_deg")
            check({"image_timeout_ms": "0"}, "image_timeout_ms", r"\[\s*1\s*,\s*500\s*\]")
            check({"max_speed_cms": "101"}, "max_speed_cms", r"\(\s*0\s*,\s*100\s*\]")
            for timeout in ("200", "500"):
                for value in ("0.5", "0.501", "1", "2.0", "29.999", "30.0"):
                    check({"image_timeout_ms": timeout, "resume_stable_s": value}, valid=True)
                for value in ("-1", "0", "0.001", "0.499", "30.001", "1000", ".Nan", ".Inf", "-.Inf", "1e300"):
                    check({"image_timeout_ms": timeout, "resume_stable_s": value}, "resume_stable_s", CHECK_RULES["resume_stable_s"][2])
        else:
            for key, value in (("resume_key", '"r"'), ("resume_key", '"x"'), ("khoa_la_kiem_thu", "1")):
                folder, config, env = prepare({key: value})
                code, _ = run([self.main, "--check-config", "--config", config], folder, env)
                require(code == 2 and "khoá lạ: " + key in read(folder / "error.txt"), "CFG15: từ chối đúng khóa lạ " + key)
                require("CAU_HINH_OK:" not in read(folder / "output.txt"), "CFG15: lỗi không báo thành công")
                unchanged(folder)
            for kind in ("missing", "directory", "malformed", "cli_no_config", "cli_old", "cli_missing", "cli_unknown", "cli_extra"):
                folder, config, env = prepare()
                command = [self.main, "--check-config", "--config", config]
                if kind == "missing":
                    config.unlink()
                elif kind == "directory":
                    config.unlink()
                    config.mkdir()
                elif kind == "malformed":
                    config.write_text("%YAML:1.0\n---\n[not: valid\n", encoding="utf-8")
                elif kind == "cli_no_config":
                    command = [self.main, "--check-config"]
                elif kind == "cli_old":
                    command = [self.main, "--check-config", config]
                elif kind == "cli_missing":
                    command.pop()
                else:
                    command.append("--unknown" if kind == "cli_unknown" else "thua")
                code, _ = run(command, folder, env)
                stderr = read(folder / "error.txt")
                require(code == 2, "CFG14/16: lỗi file/CLI phải trả đúng mã 2")
                if kind.startswith("cli_"):
                    require(stderr.strip() and re.search(r"cú pháp|cách dùng|usage|--check-config", stderr, re.I), "CFG16: stderr có thông báo cú pháp")
                else:
                    require("không đọc được: " + str(config) in stderr, "CFG14: stderr nêu đúng file không đọc được")
                require("CAU_HINH_OK:" not in read(folder / "output.txt"), "CFG14/16: lỗi không báo thành công")
                unchanged(folder)

    def smoke_gate(self):
        # D/S13/15/16: mẫu đúng là đối chứng; mọi mẫu lỗi đều bị chặn trước thiết bị.
        for speed in ("10", "25"):
            folder, config, env = self.setup({"tof_mapping_confirmed": "1", "max_speed_cms": speed}, gate=True)
            def observe(process):
                wait_ready(process, lambda: log_ready(folder, "auto"), "S13: mẫu ĐẠT phải cho auto tạo log")
                process.send_signal(signal.SIGTERM)
            code, _ = run([self.main, "--config", config, "--mode", "auto"], folder, env, observe)
            require(code in (0, 143), "S13: auto với mẫu ĐẠT phải dừng chủ động")
            tail(trace(folder / "trace.csv"))

        variants = [("missing", None, None), ("broken", None, None),
                    ("source", "source_id", "nguon-khac"), ("config", "config_text", "config-khac"),
                    ("simulated", "simulated", 0), ("mapping", None, None),
                    ("changed_config", None, None)]
        # Mỗi trường được kiểm riêng: không cho cờ tổng che mục thiếu/sai kiểu.
        for field in FIELDS + ("source_id", "config_text", "simulated"):
            variants.append(("omit", field, None))
            for value in (0, 1.0, "1", True, [], {}):
                if field == "simulated" and value == 0:
                    continue
                variants.append(("wrong", field, value))
        for kind, field, value in variants:
            folder, config, env = self.setup({"tof_mapping_confirmed": "0" if kind == "mapping" else "1"})
            data = seed_gate(self.main, config, folder / "runs")
            gate = folder / "runs/smoke_gate.json"
            if kind == "missing":
                gate.unlink()
            elif kind == "broken":
                gate.write_text('{"camera":', encoding="utf-8")
            elif kind == "changed_config":
                with config.open("ab") as out:
                    out.write(b"\n# Config doi sau khi tao mau\n")
            else:
                if kind == "omit":
                    del data[field]
                elif field:
                    data[field] = value
                gate.write_text(json.dumps(data), encoding="utf-8")
            label = "S13 {} {}={!r}".format(kind, field, value)
            code, output = run([self.main, "--config", config, "--mode", "auto"], folder, env)
            require(0 < code < 124, label + ": phải chủ động từ chối")
            require("smoke" in output.lower() or (kind == "mapping" and "tof_mapping_confirmed" in output),
                    label + ": thiếu lý do gate/ánh xạ")
            require(not trace(folder / "trace.csv"), label + ": từ chối trước thiết bị, không chuyển động")
            if not (kind == "missing" or
                    (kind == "wrong" and field == "passed" and type(value) is int and value == 0)):
                continue
            # Gate thiếu và KHÔNG ĐẠT không được chặn đường dừng độc lập.
            build = folder / "build"
            build.mkdir()
            for name, binary in (("main", self.main), ("ucr_stop", self.stop)):
                (build / name).symlink_to(binary)
            env["UCR_BUILD_DIR"] = str(build)
            code, _ = run(["bash", self.root / "scripts/run.sh", "--config", config, "--mode", "auto"], folder, env)
            rows = trace(folder / "trace.csv")
            require(0 < code < 124 and len(rows) == 2, "S15/16: run.sh giữ lỗi main và vẫn dừng")
            require(not any(k in ("motor_init", "motor_cfg") for _, k, _ in rows), "S15/16: dừng không cấu hình motor")
            tail(rows)
            (folder / "trace.csv").unlink()
            code, _ = run([self.stop, "--config", config], folder, env)
            require(code == 0, "S15/16: ucr_stop độc lập không bị cổng chặn")
            rows = trace(folder / "trace.csv")
            require(len(rows) == 2 and not any(k in ("motor_init", "motor_cfg") for _, k, _ in rows),
                    "S15/16: ucr_stop chỉ dừng motor/servo")
            tail(rows)

    def cli(self):
        # C01/C14: giao diện leader chốt --step <1..10>, lỗi trước thiết bị.
        invalid = [["--mode", "calib"], ["--mode", "calib", "--step"]]
        invalid += [["--mode", "calib", "--step", value] for value in ("0", "11", "unknown")]
        invalid.append(["--mode", "calib", "--step", "2", "--unknown"])
        for mode in ("auto", "collect", "calib"):
            invalid.append(["--mode", mode] + (["--step", "2"] if mode == "calib" else []) + ["--smoke-drop-camera"])
        for args in invalid:
            folder, config, env = self.setup({"tof_mapping_confirmed": "1"}, gate=True)
            code, output = run([self.main, "--config", config] + args, folder, env)
            require(0 < code < 124 and output.strip(), "C01/C14: phải từ chối CLI " + str(args))
            require(not trace(folder / "trace.csv"), "C01/C14: lỗi CLI trước thiết bị")

    def no_gate(self, mode):
        # S17: phải thấy log/cảm biến trước tín hiệu; mã thoát một mình không chứng minh đã khởi động.
        folder, config, env = self.setup({"tof_mapping_confirmed": "1"})
        require(not (folder / "runs/smoke_gate.json").exists(), "S17: tiền điều kiện không có cổng smoke")
        args = [self.main, "--config", config, "--mode", mode]
        if mode == "calib":
            args += ["--step", "2"]
        if mode == "smoke":
            args += self.smoke_args(folder)
        if mode == "collect":
            env["UCR_FAKE_KEYS"] = "0:+"  # Có đường nhận phím không cửa sổ, không phát lệnh chuyển động.
        def observe(process):
            if mode == "collect":
                collect_metadata(process)
            wait_ready(process, lambda: log_ready(folder, mode), "S17: " + mode + " phải tạo log/ảnh/cảm biến khi thiếu gate")
            process.send_signal(signal.SIGTERM)
        code, _ = run(args, folder, env, observe)
        require(code in (0, 1, 143) if mode == "smoke" else code in (0, 143), "S17: kết thúc chủ động sau khi đã khởi động")
        rows = trace(folder / "trace.csv")
        require(sum(k == "motor_init" for _, k, _ in rows) == 1, "S17: khởi tạo một lần")
        require(not any(k == "motor_speed" and abs(float(v)) > SPEED_EPS for _, k, v in rows), "S17: chưa phím chuyển động luôn giữ motor 0")
        tail(rows)

    def collect_cancel(self):
        # C'/A03/A06: main thật, phím vào collect không cửa sổ; H10 vẫn dành cho xe thật.
        for duration in (600, 2300):
            folder, config, env = self.setup({"tof_mapping_confirmed": "1", "camera_hz": "25",
                "control_hz": "100", "sensor_hz": "100", "ui_hz": "100", "imu_drift_ms": "100",
                "keyboard_speed_step_cms": "20", "max_speed_cms": "25", "resume_stable_s": "2",
                "soft_start_s": "1", "camera_resume_max_s": "2", "image_timeout_ms": "500", "image_every_n": "1000000",
                "collect_keys": '["w","s","a","d"," ","c","u","esc","[","]","+","-"]'})
            gap_end, key_at = 3000 + duration, 3000 + duration + 3700
            env["UCR_FAKE_CAMERA_GAPS"] = "3000:" + str(gap_end)
            env["UCR_FAKE_KEYS"] = "1500:w," + ("4000:w," if duration > 2000 else "") + str(key_at) + ":w"
            def observe(process):
                collect_metadata(process)
                wait_ready(process, lambda: log_ready(folder, "collect") and
                           any(k == "motor_speed" and 1500 <= ms < 3000 and abs(float(v)-20) <= SPEED_EPS
                               for ms, k, v in trace(folder / "trace.csv", live=True)), "C': phải có ảnh/ToF và +20 trước ngắt")
                def held_after_return():
                    hw, logs = trace(folder / "trace.csv", live=True), log_rows(folder, "collect")
                    return bool(hw and hw[-1][0] >= gap_end + 3100 and logs and
                                int(logs[-1]["frame_id"]) > 0 and float(logs[-1]["image_age_ms"]) < 500 and
                                all(float(logs[-1]["tof"+str(i)]) > 0 for i in range(4)))
                wait_ready(process, held_after_return, "C': ảnh/ToF phải có lại >W+S trước phím mới")
                hw = trace(folder / "trace.csv", live=True)
                require(hw[-1][0] < key_at and abs(held_at(hw, "motor_speed", hw[-1][0])) <= SPEED_EPS,
                        "C': có ảnh mới mà chưa bấm lại vẫn phải 0, quan sát trước phím mới")
                wait_ready(process, lambda: bool(trace(folder / "trace.csv", live=True)) and
                           trace(folder / "trace.csv", live=True)[-1][0] >= key_at + 200, "C': phải quan sát sau phím mới")
                check_collect_trace(trace(folder / "trace.csv", live=True), gap_end, key_at)
                process.send_signal(signal.SIGTERM)
            code, _ = run([self.main, "--mode", "collect", "--config", config], folder, env, observe)
            require(code in (0, 143), "C': collect kết thúc chủ động sau SIGTERM")
            rows = trace(folder / "trace.csv")
            require(sum(k == "motor_init" for _, k, _ in rows) == 1, "C': không khởi tạo motor lại")
            tail(rows)

    def auto_input(self, state, sig=None):
        # R18/A07: cùng điểm vào main thật; bản test tiêm +20 qua SetController trước Start.
        # Bộ tiêm không được nối đúng cũng phải HỎNG, không lấy phiên luôn 0 làm bằng chứng.
        keys = list("rRws+-") if state == "DUNG_CHO_ANH_ON_DINH" else ["rRws+-"]
        for key in ([None] if sig is not None else keys):
            folder, config, env = self.setup({"tof_mapping_confirmed": "1", "camera_hz": "25", "control_hz": "100",
                "sensor_hz": "100", "camera_resume_max_s": "2", "resume_stable_s": "2", "soft_start_s": "1"}, gate=True)
            env["UCR_FAKE_CAMERA_GAPS"] = "3000:5300"
            key_at = 4000 if state == "MAT_ANH" else 6300
            if sig is None:
                env["UCR_FAKE_KEYS"] = ",".join("{}:{}".format(key_at+i*10, char) for i, char in enumerate(key))
            sent = []
            def observe(process):
                wait_ready(process, lambda: any(k == "motor_speed" and ms < 3000 and abs(float(v)-20) <= SPEED_EPS
                           for ms, k, v in trace(folder / "trace.csv", live=True)),
                           "R18/A07: cần lệnh +20 trước ngắt qua main; kiểm liên kết bộ tiêm SetController")
                def target_state():
                    rows, hw = log_rows(folder), trace(folder / "trace.csv", live=True)
                    # Không nhận nhầm MAT_ANH thoáng qua lúc khởi động thành lần mất ảnh đã thiết lập.
                    return bool(rows and hw and hw[-1][0] >= (3500 if state == "MAT_ANH" else 5300) and
                                rows[-1]["state"] == state and
                                (float(rows[-1]["image_age_ms"]) >= 500 if state == "MAT_ANH" else float(rows[-1]["image_age_ms"]) < 500))
                wait_ready(process, target_state, "R18/A07: phải quan sát đúng " + state + " sau ngắt đã thiết lập")
                before = trace(folder / "trace.csv", live=True)
                require(before and abs(float([v for _, k, v in before if k == "motor_speed"][-1])) <= SPEED_EPS,
                        "R18/A07: tại trạng thái đích, Backend phải đang dừng")
                if sig is not None:
                    sent.extend((time.monotonic(), len(before)))
                    process.send_signal(sig)
                    return
                require(before[-1][0] < key_at, "R18: phải thấy trạng thái đích trước mốc tiêm UCR_FAKE_KEYS")
                first_fresh = []
                def recovered():
                    rows = log_rows(folder)
                    stale = [i for i, row in enumerate(rows) if row["state"] == "MAT_ANH"]
                    if not stale:
                        return False
                    fresh = [row for row in rows[stale[-1]+1:] if float(row["image_age_ms"]) < 500]
                    if not fresh:
                        return False
                    first_fresh[:] = [float(fresh[0]["t"]) - float(fresh[0]["image_age_ms"])/1000]
                    return float(rows[-1]["t"]) >= first_fresh[0] + 3.3
                wait_ready(process, recovered, "R18: phải qua W+S bằng ảnh thực nhận, không dùng mốc kịch bản làm t0")
                rows = log_rows(folder)
                fresh_at = first_fresh[0]
                # Log có thể bỏ khung: dùng cận sớm hơn một chu kỳ ảnh quan sát + dung sai lịch 100 ms.
                stamps = []
                for row in rows:
                    if float(row["image_age_ms"]) < 500 and (not stamps or row["frame_id"] != stamps[-1][0]):
                        stamps.append((row["frame_id"], float(row["t"]) - float(row["image_age_ms"])/1000))
                periods = [b[1]-a[1] for a,b in zip(stamps, stamps[1:]) if 0 < b[1]-a[1] < 0.5]
                require(periods, "R18: phải đo được chu kỳ ảnh ngoài khoảng ngắt")
                earliest = fresh_at - max(periods) - 0.1
                ramp = []
                for row in rows:
                    t, v = float(row["t"]), float(row["cmd_speed"])
                    require(math.isfinite(v) and abs(v) <= 25+SPEED_EPS, "R18: tốc độ hữu hạn và đúng trần")
                    require(row["state"] != "DUNG_CHOT_CAN_XAC_NHAN", "R18: cấm trạng thái cũ")
                    if row["state"] in ("MAT_ANH", "DUNG_CHO_ANH_ON_DINH"):
                        require(abs(v) <= SPEED_EPS, "R18: ảnh cũ/chờ ổn định vẫn giữ 0 dù đã bấm phím")
                    if t >= fresh_at:
                        if t < earliest+2:
                            require(abs(v) <= SPEED_EPS, "R18: phím không bỏ thời gian chờ W")
                        else:
                            require(abs(v) <= 20*min(1, max(0, t-earliest-2))+SPEED_EPS, "R18: mọi mẫu theo trần tăng mềm sau W")
                        if 0.1 < v < 20-SPEED_EPS:
                            ramp.append(row)
                require(ramp, "R18: phải có mẫu tăng mềm dương, không được luôn 0")
                require(abs(float(rows[-1]["cmd_speed"])-20) <= SPEED_EPS, "R18: sau W+S phải đạt +20")
                hw = trace(folder / "trace.csv", live=True)
                for ms, kind, value in hw:
                    if kind == "motor_speed" and ms >= 3610:
                        v = float(value)
                        require(abs(v) <= 20*min(1, max(0, (ms-7300)/1000))+SPEED_EPS,
                                "R18: vết motor giữ 0 qua ảnh cũ/W, sau đó không vượt trần từ cận sớm nhất")
                    if kind == "servo_angle" and 3610 <= ms < 7300:
                        require(abs(float(value)-7) <= SPEED_EPS, "R18: vết servo về trim suốt ảnh cũ/chờ W")
                speeds = [float(v) for _,k,v in hw if k == "motor_speed"]
                require(speeds and abs(speeds[-1]-20) <= SPEED_EPS and any(k == "motor_speed" and ms >= 7300 and SPEED_EPS < float(v) < 20-SPEED_EPS for ms,k,v in hw),
                        "R18: tăng mềm/đạt +20 phải có cả ở lệnh cuối Backend")
                process.send_signal(signal.SIGTERM)
            code, _ = run([self.main, "--mode", "auto", "--config", config], folder, env, observe, timeout=10)
            require(code in (0, 130, 143), "R18/A07: thoát chủ động sau tín hiệu")
            rows = trace(folder / "trace.csv")
            if sig is not None:
                require(sent and time.monotonic()-sent[0] <= 10, "A07: thoát trong 10 s từ tín hiệu")
                require(not any(k == "motor_speed" and (not math.isfinite(float(v)) or abs(float(v)) > SPEED_EPS) for _,k,v in rows[sent[1]:]), "A07: không lệnh chạy muộn sau tín hiệu")
            require(sum(k == "motor_init" for _,k,_ in rows) == 1 and sum(k == "motor_cfg" and "$101=1" in v for _,k,v in rows) == 1,
                    "R18/A07: không khởi tạo/lưu reset motor lại")
            tail(rows)

    def calib(self, step, sig):
        # C02/C13/A01: mỗi bước số hợp lệ, stdin giữ mở tới hết quan sát.
        if step == "position":
            # C1 (28/10): bước vị trí bị khoá trước khởi tạo, giữ các bước còn lại.
            from acc_stop_path import reject_position
            reject_position(self, "5")
            return
        folder, config, env = self.setup({"tof_mapping_confirmed": "1"})
        if step == "env":
            code, output = run([self.main, "--config", config, "--mode", "calib", "--step", "1"], folder, env)
            require(code == 0 and output.strip(), "C13: bước 1 phải in môi trường và thoát chủ động")
            require(not trace(folder / "trace.csv"), "C13: bước môi trường không khởi tạo thiết bị")
            return
        def observe(process):
            def ready():
                return (any(k == "motor_init" for _, k, _ in trace(folder / "trace.csv", live=True)) and
                        log_ready(folder, "calib"))
            wait_ready(process, ready, step + ": calib không tới giao diện/log trong hạn")
            if step in ("servo", "radius", "motor"):
                process.stdin.write(b"d\ny\n" if step == "radius" else b"y\n")
                process.stdin.flush()
                require(poll(lambda: any(k == "motor_speed" and abs(float(v)) > 0.1
                                        for _, k, v in trace(folder / "trace.csv", live=True)), 5),
                        "A01: bước có chuyển động phải phát lệnh trước tín hiệu")
            process.send_signal(sig)
        code, output = run([self.main, "--config", config, "--mode", "calib", "--step", str(STEPS[step])], folder, env, observe)
        require(code in (0, 130, 143), "A01: tín hiệu phải được xử lý để dừng chủ động: " + str(code))
        require(output.strip(), "C13: phải in hướng dẫn bước")
        words = {"servo": ("servo",), "radius": ("radius", "bán kính"), "motor": ("motor",),
                 "position": ("position", "vị trí"), "imu": ("imu",), "tof": ("tof",),
                 "camera": ("camera",), "latency": ("latency", "độ trễ"),
                 "timing": ("timing", "vòng", "nhịp")}
        require(any(word in output.lower() for word in words[step]), "C13: thiếu hướng dẫn đúng bước " + step)
        rows = trace(folder / "trace.csv")
        require(sum(k == "motor_init" for _, k, _ in rows) == 1, "C02: motor khởi tạo đúng một lần")
        require(sum(k == "motor_cfg" and "$101=1" in v for _, k, v in rows) == 1, "C02: lưu/reset một lần")
        tail(rows)

    def smoke_signal(self, sig):
        # A01: smoke nhận tín hiệu cả khi chờ xác nhận; kiểm dừng lệnh, không coi là đã phanh xe thật.
        folder, config, env = self.setup({"tof_mapping_confirmed": "1"})
        def observe(process):
            wait_ready(process, lambda: log_ready(folder, "smoke"), "Smoke chưa tạo log trong hạn")
            process.send_signal(sig)
        code, _ = run([self.main, "--config", config, "--mode", "smoke"] + self.smoke_args(folder), folder, env, observe)
        require(code in (0, 1, 130, 143), "A01: smoke thoát chủ động sau tín hiệu")
        tail(trace(folder / "trace.csv"))

    def smoke_unconfirmed(self):
        # S01/02/C14: có/thiếu cờ ánh xạ đều dừng trước motor, vẫn nhận cờ ngắt ảnh ở smoke.
        for missing in (False, True):
            folder, config, env = self.setup({}, ("tof_mapping_confirmed",) if missing else ())
            code, output = run([self.main, "--config", config, "--mode", "smoke", "--smoke-drop-camera"] + self.smoke_args(folder),
                               folder, env, timeout=EXIT_TIMEOUT)
            require(0 < code < 124, "S02: chưa xác nhận ánh xạ không được toàn bộ ĐẠT")
            require("tof_mapping_confirmed" in output and "config" in output.lower(), "S02: phải chỉ cách sửa cấu hình")
            require(re.search(r"tof_mapping_confirmed[^\n]*(?:[:=]\s*1\b|(?:nguyên|nguyen)\s+1\b)", output),
                    "S02: hướng dẫn đổi sang số nguyên 1")
            require(all(name in output for name in ("front", "rear", "left", "right")), "S01/04: in đủ bốn tên cảm biến")
            rows = trace(folder / "trace.csv")
            require(rows and not any(k == "motor_speed" and abs(float(v)) > SPEED_EPS for _, k, v in rows), "S01/02: không chạy motor")
            tail(rows)
            reports = list((folder / "runs").glob("*_smoke/smoke.json"))
            require(len(reports) == 1, "S12: có kết quả riêng trong lượt smoke")
            result = json.loads(read(reports[0]))
            require(result["passed"] == 0 and result["simulated"] == 1, "S02/S12: kết quả lỗi, đánh dấu giả lập")
            require(result["config_text"] == read(config), "S12: bản sao cấu hình khớp đầu vào")
            for field in ("source_id", "camera", "sensors", "servo", "tof_brake", "camera_brake",
                          "encoder_moving", "kill9", "encoder_stopped"):
                require(field in result, "S12: thiếu trường " + field)
            require(not list((folder / "runs").glob("*_collect")), "S14: smoke không tự chạy collect")

    def fault(self, mode, fault):
        # A02: tiêm lỗi khi chờ nhập calib/smoke; terminate qua trình bao vẫn phải dừng độc lập.
        folder, config, env = self.setup({"tof_mapping_confirmed": "1"})
        # Lỗi xảy ra sau pha chuẩn bị; không tính thời gian khởi động ASan vào hạn dừng an toàn.
        env["UCR_FAKE_FAULT"] = fault + "@15000"
        build = folder / "build"
        build.mkdir()
        for name, binary in [("main", self.main), ("ucr_stop", self.stop)]:
            (build / name).symlink_to(binary)
        env["UCR_BUILD_DIR"] = str(build)
        args = ["bash", self.root / "scripts/run.sh", "--config", config, "--mode", mode]
        if mode == "calib":
            args += ["--step", "4"]
        else:
            args += self.smoke_args(folder)
        def observe(process):
            if mode != "calib":
                return
            wait_ready(process, lambda: log_ready(folder, mode), "A02: phải sẵn sàng trước điểm tiêm lỗi")
            process.stdin.write(b"y\n")
            process.stdin.flush()
            require(poll(lambda: any(k == "motor_speed" and float(v) > 0.1
                                    for _, k, v in trace(folder / "trace.csv", live=True)), 5),
                    "A02: phải có lệnh chạy trước lỗi calib")
            require(any(k == "motor_speed" and float(v) > 0.1 and ms < 15000
                        for ms, k, v in trace(folder / "trace.csv", live=True)),
                    "A02: thiết lập phải phát lệnh chạy trước mốc lỗi 15000 ms")
        code, _ = run(args, folder, env, observe=observe, timeout=25)
        require(code != 0, "A02: lỗi luồng phải thoát lỗi")
        if fault == "terminate":
            require(code == 134, "A02: giữ mã terminate 134")
        rows = trace(folder / "trace.csv")
        require(sum(k == "motor_init" for _, k, _ in rows) == 1, "A02: không khởi tạo motor lại")
        require(sum(k == "motor_cfg" and "$101=1" in v for _, k, v in rows) == 1, "A02: không lưu/reset lại")
        tail(rows)


def static(root, scenario):
    root = Path(root).resolve()
    if scenario == "env":
        text = read(root / "scripts/env_check.sh")
        require(not re.search(r"\bapt(?:-get)?\b|\bpip[0-9]*\s+install\b|\bnvpmodel\s+-s\b", text),
                "C03: env_check không được cài gói/đổi nguồn")
    elif scenario == "collect_print":
        lines = read(root / "scripts/smoke_test.sh").splitlines()
        matches = [line for line in lines if "--mode collect" in line]
        require(matches and all(re.search(r"\b(?:printf|echo)\b", line) for line in matches),
                "S14: collect chỉ nằm trong lệnh được in")
    elif scenario == "defaults":
        text = read(root / "config.yaml")
        for key, expected in [("max_speed_cms", 25), ("camera_resume_max_s", 2), ("soft_start_s", 1), ("resume_stable_s", 2)]:
            value = re.search(r"(?m)^" + key + r":\s*([^#\n]+)", text)
            require(value and abs(float(value.group(1)) - expected) <= SPEED_EPS, "CFG01: mặc định " + key)
        require(re.search(r"(?m)^tof_mapping_confirmed:\s*0\s*(?:#.*)?$", text), "CFG01: ánh xạ chưa xác nhận")
        require(not re.search(r"(?m)^resume_key:", text), "CFG01: config không còn khóa resume_key")
    else:
        raise AssertionError("Ca tĩnh không hợp lệ: " + scenario)


def self_test():
    # Đối chứng đúng/sai cho bộ đọc vết, kiểm lệnh cuối và giới hạn tiến trình của chính test.
    with tempfile.TemporaryDirectory(prefix="acc_vehicle_self_") as name:
        folder = Path(name)
        config = folder / "config.yaml"
        config.write_bytes("%YAML:1.0\r\n---\r\n# Cấu hình mẫu\r\n".encode("utf-8"))
        data = gate_data("source_id=ma-mau; simulated=1", config)
        require(data["config_text"].encode("utf-8") == config.read_bytes(), "Mẫu gate phải giữ nguyên CRLF/UTF-8")
        require(data["source_id"] == "ma-mau" and all(type(data[key]) is int and data[key] == 1 for key in FIELDS),
                "Mẫu gate đủ các mục số nguyên 1")
        require(set(data) == set(FIELDS) | {"source_id", "config_text", "simulated"} and
                type(data["simulated"]) is int and data["simulated"] == 1,
                "Mẫu JSON khớp đủ 12 trường theo docs/ucr_car.md, không dùng cờ bool")
        from unittest.mock import patch
        with patch("acc_seed_gate.subprocess.check_output", return_value=b"source_id=ma-mau; simulated=1") as probe:
            require(seed_gate("main-gia", config, folder) == data, "Hàm chung phải ghi đúng mẫu ĐẠT")
            probe.assert_called_once_with(["main-gia", "--env-info"], timeout=5)
        require(json.loads(read(folder / "smoke_gate.json")) == data, "File mẫu phải giữ đúng dữ liệu")
        require(not (folder / "smoke_gate.json.tmp").exists(), "Không còn file tạm sau khi thay thế mẫu")
        config.write_bytes(config.read_bytes() + b"# changed\n")
        require(gate_data("source_id=ma-mau; simulated=1", config)["config_text"] != data["config_text"],
                "Đổi config phải tạo lại mẫu")
        for info in ("source_id=ma-mau; simulated=0", "simulated=1", "source_id=ma-mau; simulated=10"):
            caught = False
            try:
                gate_data(info, config)
            except AssertionError:
                caught = True
            require(caught, "Không được cấp mẫu khi không chứng minh bản giả/dấu vân tay")
        path = folder / "trace.csv"
        path.write_text("0,motor_speed,20\n1,motor_speed,0\n2,servo_angle,7\n", encoding="utf-8")
        tail(trace(path))
        for bad in ("0,motor_speed,20\n1,servo_angle,7\n", "sai-dinh-dang\n"):
            path.write_text(bad, encoding="utf-8")
            caught = False
            try:
                tail(trace(path))
            except AssertionError:
                caught = True
            require(caught, "Bộ kiểm tra phải bắt đối chứng sai")
        path.write_text("0,motor_speed,0\n1,servo_angle,7\n2,motor_", encoding="utf-8")
        tail(trace(path, live=True))
        caught = False
        try:
            trace(path)
        except AssertionError:
            caught = True
        require(caught, "Kiểm cuối vẫn phải bắt dòng vết cụt")
        for gap_end in (3600, 5300):
            key_at = gap_end + 3700
            good = [(0, "motor_speed", "0"), (0, "servo_angle", "7"),
                    (1500, "motor_speed", "20"), (3500, "motor_speed", "0"),
                    (3500, "servo_angle", "7"), (key_at, "motor_speed", "20"),
                    (key_at+200, "servo_angle", "7")]
            check_collect_trace(good, gap_end, key_at)
            variants = [good[:2] + good[3:],
                        [row for row in good if row[:2] != (3500, "motor_speed")],
                        good[:5] + [(gap_end+200, "motor_speed", "20")] + good[5:],
                        good[:5] + [(key_at, "motor_speed", "25")] + good[6:],
                        good[:5] + [(key_at, "motor_speed", "2")] + good[6:]]
            for bad in variants:
                caught = False
                try:
                    check_collect_trace(bad, gap_end, key_at)
                except AssertionError:
                    caught = True
                require(caught, "C': phải bắt thiếu lệnh chạy/dừng, tự hồi lệnh, giữ phím cũ hoặc tăng mềm sai")
        with io.BytesIO() as metadata:
            from types import SimpleNamespace
            collect_metadata(SimpleNamespace(stdin=metadata))
            require(len(metadata.getvalue().splitlines()) == 6 and not metadata.closed,
                    "Collect: gửi đủ sáu mục thông tin, không đóng stdin trước quan sát")
        require(diagnostic("lỗi <&>\x00\x1b\n") == "lỗi <&>\\x00\\x1b\n", "Chỉ thoát ký tự cấm trong XML")
        log = folder / "runs/now_calib/log.csv"
        log.parent.mkdir(parents=True)
        log.write_text("t,frame_id,image_age_ms,tof0,tof1,tof2,tof3\n", encoding="utf-8")
        require(not log_ready(folder, "calib"), "Có file chưa phải log sẵn sàng")
        with log.open("a", encoding="utf-8") as out:
            out.write("0.2,1,10,500,500,500,500\n0.3,")
        require(log_ready(folder, "calib"), "Log đủ mẫu được nhận dù dòng cuối còn đang ghi")
        auto_log = folder / "runs/now_auto/log.csv"
        auto_log.parent.mkdir()
        auto_log.write_text("t,state,cmd_speed\n1,MAT_ANH,0\n2,DUNG_CHO_ANH_ON_DINH,0\n3,", encoding="utf-8")
        require([row["state"] for row in log_rows(folder)] == ["MAT_ANH", "DUNG_CHO_ANH_ON_DINH"],
                "Bộ đọc auto chỉ nhận dòng hoàn chỉnh, giữ đúng thứ tự trạng thái")
        ready = folder / "ready"
        child = "import time; from pathlib import Path; time.sleep(0.1); Path('ready').touch(); time.sleep(0.2)"
        code, _ = run([sys.executable, "-c", child], folder,
                      observe=lambda process: wait_ready(process, ready.exists, "Đối chứng tạo log trễ"))
        require(code == 0, "Phải chờ điều kiện đến trễ")
        caught = False
        started = time.monotonic()
        try:
            run([sys.executable, "-c", "pass"], folder,
                observe=lambda process: wait_ready(process, lambda: False, "Đối chứng thoát sớm"))
        except AssertionError:
            caught = True
        require(caught and time.monotonic() - started < 4, "Thoát sớm phải hỏng ngay, không chờ hết hạn log")
        code, _ = run([sys.executable, "-c", "print('ok')"], folder)
        require(code == 0, "Chạy tiến trình đối chứng")
        started = time.monotonic()
        caught = False
        try:
            run([sys.executable, "-c", "import time; time.sleep(20)"], folder, timeout=0.1)
        except subprocess.TimeoutExpired:
            caught = True
        require(caught and time.monotonic() - started < 4, "Phải ngắt tiến trình treo trong hạn")
        from unittest.mock import patch
        (folder / "config.yaml").write_text("%YAML:1.0\n---\n", encoding="utf-8")
        suite = Suite(folder, sys.executable, sys.executable, folder)
        with patch(__name__ + ".run", side_effect=subprocess.TimeoutExpired(["manual"], 15)):
            caught = False
            try:
                suite.reject("resume_stable_s", '"2"')
            except AssertionError as error:
                caught = True
                require('resume_stable_s="2"' in str(error) and "hết hạn" in str(error),
                        "Phải nêu khóa/giá trị và nguyên nhân hết hạn")
            require(caught, "Cấu hình cố ý sai chạy tới hết hạn vẫn phải HỎNG")
        # Đối chứng giao thức F: chỉ giả kết quả CLI, không coi đây là chạy Config/thiết bị thật.
        (folder / "config.yaml").write_text("%YAML:1.0\n---\n" + "".join(k+": "+v[0]+"\n" for k,v in CHECK_RULES.items()), encoding="utf-8")
        protocol = folder / "giao-thuc"
        protocol.mkdir()
        suite = Suite(folder, sys.executable, sys.executable, protocol)
        rename = Path.rename
        def rename_py36(source, destination):
            rename(source, destination)  # Giữ tác dụng đổi tên; trả None như Python 3.6.
        for wrong in (None, "exit", "stdout", "stderr", "side_effect"):
            def fake_check(command, work, env):
                config = command[-1]
                (work / "output.txt").write_text("OK\n" if wrong == "stdout" else "CAU_HINH_OK: " + str(config) + "\n", encoding="utf-8")
                (work / "error.txt").write_text("max_speed_cms = 101 ngoài khoảng hợp lệ (0, 100]" if wrong == "stderr" else "cảnh báo thiếu khóa\n", encoding="utf-8")
                if wrong == "side_effect":
                    (work / "runs").mkdir()
                return (1 if wrong == "exit" else 0), ""
            caught = False
            try:
                with patch(__name__ + ".run", side_effect=fake_check), patch.object(Path, "rename", rename_py36):
                    suite.check_config("check_config_valid")
            except AssertionError:
                if wrong is None:
                    raise
                caught = True
            require(caught == (wrong is not None), "Bộ kiểm F phải phân biệt đúng hợp đồng và đối chứng sai " + str(wrong))
    print("self-test: ĐẠT")


def main():
    if sys.argv[1:] == ["self-test"]:
        self_test()
        return
    scenario, root = sys.argv[1:3]
    if scenario in ("env", "collect_print", "defaults"):
        static(root, scenario)
    else:
        with tempfile.TemporaryDirectory(prefix="acc_vehicle_cli_") as folder:
            suite = Suite(root, sys.argv[3], sys.argv[4], folder)
            if scenario in ("config_numeric", "config_keys", "wrapper_config", "cli", "smoke_unconfirmed", "smoke_gate", "collect_cancel"):
                getattr(suite, scenario)()
            elif scenario.startswith("check_config_"):
                suite.check_config(scenario)
            elif scenario == "calib":
                suite.calib(sys.argv[5], signal.SIGINT if sys.argv[6] == "INT" else signal.SIGTERM)
            elif scenario == "fault":
                suite.fault(sys.argv[5], sys.argv[6])
            elif scenario == "smoke_signal":
                suite.smoke_signal(signal.SIGINT if sys.argv[5] == "INT" else signal.SIGTERM)
            elif scenario == "no_gate":
                suite.no_gate(sys.argv[5])
            elif scenario in ("auto_keys", "auto_signal"):
                suite.auto_input(sys.argv[5], (signal.SIGINT if sys.argv[6] == "INT" else signal.SIGTERM) if scenario == "auto_signal" else None)
            else:
                raise AssertionError("Ca không hợp lệ: " + scenario)
    print(scenario + ": ĐẠT")


if __name__ == "__main__":
    def interrupted(_sig, _frame):
        raise KeyboardInterrupt("Test bị ngắt, thu hồi tiến trình con")
    signal.signal(signal.SIGTERM, interrupted)
    try:
        main()
    except (AssertionError, subprocess.TimeoutExpired, OSError, ValueError) as error:
        print(diagnostic("[HỎNG] " + str(error)), file=sys.stderr)
        sys.exit(1)
