#!/usr/bin/env python3
"""Kiểm tra DEV qua tiến trình giả: phím lặp, manual mất ảnh và smoke kẹt UI."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile

from test_calib_limits import config_text, trace_rows


def main():
    executable, source, scenario = sys.argv[1:]
    env = {key: value for key, value in os.environ.items() if not key.startswith("UCR_FAKE_")}
    with tempfile.TemporaryDirectory(prefix="ucr_dev_review_") as directory:
        root = Path(directory)
        config, trace, output = root / "config.yaml", root / "trace.csv", root / "process.log"
        values = {"log_dir": '"' + (root / "runs").as_posix() + '"',
                  "tof_mapping_confirmed": 1, "enable_display": 0, "enable_oled": 0,
                  "image_timeout_ms": 200, "ui_hz": 20, "calib_max_duration_s": 3,
                  "calib_distance_cm": 300, "servo_trim_deg": 7}
        env["UCR_FAKE_HW_TRACE"] = str(trace)
        if scenario == "manual":
            args = ["--mode", "manual"]
            env["UCR_FAKE_KEYS"] = "600:w,1800:w,4000:w,4800:q"
            env["UCR_FAKE_CAMERA_GAPS"] = "1000:3000"
        elif scenario == "repeat_y":
            args = ["--mode", "calib", "--step", "4"]
            env["UCR_FAKE_KEYS"] = "600:y,1500:y,2400:y,3300:y,4700:q"
        else:
            assert scenario in ("smoke_duration", "smoke_distance", "smoke_image"), scenario
            args = ["--mode", "smoke", "--smoke-kill-file", str(root / "kill.ready")]
            env["UCR_FAKE_KEYS"] = "3400:y,3600:y,3800:y,4000:y,4200:y"
            env["UCR_FAKE_UI_STALL"] = "4500:7000"
            if scenario == "smoke_distance":
                values["calib_distance_cm"] = 8
            if scenario == "smoke_image":
                env["UCR_FAKE_CAMERA_GAPS"] = "4800:10000"
        config.write_text(config_text(Path(source).read_text(encoding="utf-8"), values), encoding="utf-8")
        # Giữ stdin mở: EOF chủ động huỷ chuyển động, sẽ che lỗi cần tìm.
        with output.open("wb") as log:
            process = subprocess.Popen([executable, "--config", str(config)] + args,
                                       stdin=subprocess.PIPE, stdout=log, stderr=subprocess.STDOUT, env=env)
            try:
                process.wait(timeout=18)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait(timeout=3)
                process.stdin.close()
        message = output.read_text(encoding="utf-8", errors="replace")[-3000:]
        expected_code = 1 if scenario.startswith("smoke_") else 0
        assert process.returncode == expected_code, message
        rows = trace_rows(trace)
        motors = [(ms, value) for ms, kind, value in rows if kind == "motor_speed"]
        first = next(ms for ms, value in motors if value > 0)
        stopped = next(ms for ms, value in motors if ms > first and value == 0)
        if scenario == "manual":
            assert 600 <= first < 1000 and 1100 <= stopped <= 1450, motors
            assert all(value == 0 for ms, value in motors if stopped <= ms < 4000), motors
            assert any(3200 <= ms < 3900 and value == 0 for ms, value in motors), motors
            assert any(4000 <= ms < 4700 and value > 0 for ms, value in motors), motors
        else:
            if scenario == "smoke_image":
                lower, upper = 4900, 5250
            else:
                duration = 800 if scenario == "smoke_distance" else 3000
                lower, upper = first + duration - 200, first + duration + 350
            assert lower <= stopped <= upper, (scenario, first, stopped)
            assert all(value == 0 for ms, value in motors if ms >= stopped), motors
            if scenario.startswith("smoke_"):
                assert first < 4500 < stopped < 11500, "Chưa chứng minh dừng trong lúc UI kẹt"
                assert not (root / "kill.ready").exists(), "Smoke hỏng vẫn cho phép kill"
            angles = [(ms, value) for ms, kind, value in rows if kind == "servo_angle"]
            assert all(value == 7 for ms, value in angles if ms >= stopped), angles
        print("ĐẠT DEV:", scenario, "bắt đầu ms=", first, "dừng ms=", stopped)


if __name__ == "__main__":
    main()
