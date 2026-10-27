#!/usr/bin/env python3
"""Kiểm tra DEV: giới hạn calib khi UI kẹt, huỷ góc và cấu hình; chỉ bản giả."""
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile


def config_text(source, values):
    text = source
    for key, value in values.items():
        text, count = re.subn(r"(?m)^" + re.escape(key) + r":.*$", key + ": " + str(value), text)
        assert count == 1, "Không tìm thấy đúng một khoá: " + key
    return text


def trace_rows(path):
    if not path.exists():
        return []
    return [(int(ms), kind, float(value)) for ms, kind, value in
            (line.split(",", 2) for line in path.read_text(encoding="utf-8").splitlines())
            if kind in ("motor_speed", "servo_angle")]


def main():
    executable, stop, source, scenario = sys.argv[1:]
    source = Path(source).read_text(encoding="utf-8")
    env = {key: value for key, value in os.environ.items() if not key.startswith("UCR_FAKE_")}
    with tempfile.TemporaryDirectory(prefix="ucr_dev_limits_") as directory:
        root = Path(directory)
        config, trace, output = root / "config.yaml", root / "trace.csv", root / "process.log"
        env["UCR_FAKE_HW_TRACE"] = str(trace)
        text = config_text(source, {
            "log_dir": '"' + (root / "runs").as_posix() + '"',
            "tof_mapping_confirmed": 1, "enable_display": 0, "enable_oled": 0,
            "image_timeout_ms": 200, "servo_trim_deg": 7, "calib_servo_step_deg": 20,
            "calib_speed_cms": 10, "calib_distance_cm": 300, "calib_max_duration_s": 2,
        })
        config.write_text(text, encoding="utf-8")
        if scenario == "config":
            for key, maximum, unit in (("calib_distance_cm", 300, "cm"),
                                       ("calib_position_cm", 100, "cm"),
                                       ("calib_max_duration_s", 120, "s")):
                for value, valid in (("0.1", True), (str(maximum), True),
                                     (str(maximum) + ".0000001", False), ("0", False)):
                    config.write_text(config_text(text, {key: value}), encoding="utf-8")
                    result = subprocess.run([executable, "--check-config", "--config", str(config)],
                                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env, timeout=5)
                    if valid:
                        assert result.returncode == 0, result.stderr.decode("utf-8", "replace")
                        assert result.stdout.decode("utf-8").strip() == "CAU_HINH_OK: " + str(config)
                    else:
                        message = result.stderr.decode("utf-8", "replace")
                        assert result.returncode == 2 and key + " = " in message, message
                        assert "ngoài khoảng hợp lệ (0, {}] {}".format(maximum, unit) in message, message
                assert not trace.exists() and not (root / "runs").exists(), "Kiểm cấu hình đã mở thiết bị/ghi log"
            # Khoá calib sai không được ngăn đường dừng độc lập đọc trim hợp lệ.
            result = subprocess.run([stop, "--config", str(config)], stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE, env=env, timeout=5)
            rows = trace_rows(trace)
            assert result.returncode == 0 and rows, result.stderr.decode("utf-8", "replace")
            assert all(value == (0 if kind == "motor_speed" else 7) for _, kind, value in rows), "Dừng không về motor 0/trim"
            assert "motor_init" not in trace.read_text(encoding="utf-8"), "Dừng đã cấu hình lại motor"
            return
        if scenario == "invalid_stall":
            for bad in ("1", ":1", "1:", "1:2:3", "-1:2", "1:-2", "+1:2", "1.5:2", "1:2,3:4",
                        "9223372036854775808:1", "1:9223372036854775808"):
                env["UCR_FAKE_UI_STALL"] = bad
                result = subprocess.run([executable, "--config", str(config), "--mode", "calib", "motor"],
                                        stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env, timeout=5)
                assert result.returncode != 0 and b"UCR_FAKE_UI_STALL" in result.stderr, bad
                assert not trace.exists() and not (root / "runs").exists(), "Kịch bản sai đã khởi tạo thiết bị/log"
            return
        step = scenario.split("_")[0] if scenario != "cancel_angle" else "motor"
        distance = scenario.endswith("_distance")
        if distance:
            key = "calib_position_cm" if step == "position" else "calib_distance_cm"
            config.write_text(config_text(text, {key: 8, "calib_max_duration_s": 5}), encoding="utf-8")
        env["UCR_FAKE_KEYS"] = "500:d,600:y,4600:q"
        if scenario == "cancel_angle":
            env["UCR_FAKE_CAMERA_GAPS"] = "1100:2300"
            config.write_text(config_text(text, {"calib_max_duration_s": 5}), encoding="utf-8")
        else:
            env["UCR_FAKE_UI_STALL"] = "1000:3000"
        # Giữ stdin mở; EOF (hết đầu vào) sẽ chủ động huỷ lượt calib và làm sai phép thử.
        with output.open("wb") as log:
            process = subprocess.Popen([executable, "--config", str(config), "--mode", "calib", step],
                                       stdin=subprocess.PIPE, stdout=log, stderr=subprocess.STDOUT, env=env)
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
                raise AssertionError("Tiến trình không thoát sau phím q")
            finally:
                process.stdin.close()
        assert process.returncode == 0, output.read_text(encoding="utf-8", errors="replace")[-3000:]
        rows = trace_rows(trace)
        motors = [(ms, value) for ms, kind, value in rows if kind == "motor_speed"]
        first = next(ms for ms, value in motors if value > 0)
        stopped = next(ms for ms, value in motors if ms > first and value == 0)
        expected = 1300 - first if scenario == "cancel_angle" else (800 if distance else 2000)
        assert first + expected - 200 <= stopped <= first + expected + 350, (first, stopped, scenario)
        assert any(ms >= 1000 and value > 0 for ms, value in motors), "Lượt chưa đi qua mốc chặn/mất ảnh"
        assert all(value == 0 for ms, value in motors if ms >= stopped), "Lệnh cũ tự chạy lại"
        angles = [(ms, value) for ms, kind, value in rows if kind == "servo_angle"]
        assert any(value == 27 for _, value in angles), "Chưa đặt góc 20° cộng trim 7°"
        assert all(value == 7 for ms, value in angles if ms >= stopped), "Huỷ rồi vẫn quay lại góc cũ"
        assert any(ms >= 4100 for ms, _ in angles), "Thiếu quan sát sau khi UI/ảnh phục hồi"
        print("ĐẠT DEV:", scenario, "bắt đầu ms=", first, "dừng ms=", stopped)


if __name__ == "__main__":
    main()
