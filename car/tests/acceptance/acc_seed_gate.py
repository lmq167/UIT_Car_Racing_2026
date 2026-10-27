"""Dữ liệu smoke mẫu cho tiến trình giả: đúng dấu vân tay và từng byte config."""
import json
from pathlib import Path
import re
import subprocess

FIELDS = ("camera", "sensors", "servo", "tof_brake", "camera_brake",
          "encoder_moving", "kill9", "encoder_stopped", "passed")


def gate_data(info, config):
    source = re.search(r"source_id=([^;\s]+)", info)
    if not source or not re.search(r"simulated=1(?:\s|;|$)", info):
        raise AssertionError("Chỉ cấp mẫu smoke cho bản UCR_FAKE_HW=ON có source_id")
    data = {"source_id": source.group(1), "simulated": 1,
            "config_text": Path(config).read_bytes().decode("utf-8")}
    data.update({field: 1 for field in FIELDS})
    return data


def seed_gate(main, config, logs):
    info = subprocess.check_output([str(main), "--env-info"], timeout=5).decode("utf-8")
    data = gate_data(info, config)
    path = Path(logs) / "smoke_gate.json"
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(data, ensure_ascii=False), encoding="utf-8")
    temporary.replace(path)
    return data
