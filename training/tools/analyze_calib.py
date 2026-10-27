"""Đọc log hiệu chuẩn; chỉ in kết quả, không sửa dữ liệu hay cấu hình."""
import argparse
import csv
import math
import statistics
import tempfile
import sys
from pathlib import Path


def analyze(path):
    with Path(path).open(encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))
    required = ("t", "frame_id", "image_age_ms", "odom_cm", "yaw_rel", "fps", "meas_speed", "state")
    if not rows or not set(required).issubset(rows[0]):
        raise ValueError("Log rỗng hoặc thiếu cột hiệu chuẩn")
    for row in rows:
        for key in required:
            if key != "state":
                row[key] = float(row[key])
                if not math.isfinite(row[key]):
                    raise ValueError("Số không hữu hạn tại cột " + key)
        for key in ("actuator_ms", "tof0_age_ms", "tof1_age_ms", "tof2_age_ms", "tof3_age_ms"):
            if row.get(key):
                row[key] = float(row[key])
                if not math.isfinite(row[key]):
                    raise ValueError("Số không hữu hạn tại cột " + key)
    times = [r["t"] for r in rows]
    if any(b < a for a, b in zip(times, times[1:])):
        raise ValueError("Mốc thời gian phải tăng dần")
    duration = times[-1] - times[0]
    distance = rows[-1]["odom_cm"] - rows[0]["odom_cm"]
    yaw = rows[-1]["yaw_rel"] - rows[0]["yaw_rel"]
    frames = {}
    for row in rows:
        if row["frame_id"] > 0 and row["image_age_ms"] >= 0:
            frames.setdefault(row["frame_id"], row["t"] - row["image_age_ms"] / 1000)
    captures = sorted(frames.values())
    fps = (len(captures) - 1) / (captures[-1] - captures[0]) if len(captures) > 1 and captures[-1] > captures[0] else None
    result = {"duration_s": duration, "distance_cm": distance, "yaw_deg": yaw,
              "radius_cm": abs(distance / math.radians(yaw)) if abs(yaw) > 1e-6 else None,
              "yaw_drift_deg_s": yaw / duration if duration else None,
              "observed_fps": fps}
    loops = [float(r["actuator_ms"]) for r in rows if r.get("actuator_ms") and float(r["actuator_ms"]) > 0]
    if loops:
        result["actuator_mean_ms"] = statistics.mean(loops)
        result["actuator_max_ms"] = max(loops)
    for slot in range(4):
        key = "tof{}_age_ms".format(slot)
        samples = sorted(set(round(r["t"] - float(r[key]) / 1000, 2) for r in rows
                             if r.get(key) and float(r.get("tof{}".format(slot), -1)) >= 0))
        result["tof{}_observed_hz".format(slot)] = ((len(samples) - 1) / (samples[-1] - samples[0])
                                                   if len(samples) > 1 and samples[-1] > samples[0] else None)
    return result


def self_test():
    with tempfile.TemporaryDirectory(prefix="ucr calib ") as folder:
        path = Path(folder) / "đo log.csv"
        path.write_text("t,frame_id,image_age_ms,odom_cm,yaw_rel,fps,meas_speed,state,actuator_ms\n"
                        "0,1,0,0,0,10,10,DO,20\n1,11,0,100,180,10,10,DO,22\n", encoding="utf-8")
        result = analyze(path)
        assert abs(result["radius_cm"] - 100 / math.pi) < 1e-9
        assert result["duration_s"] == 1 and result["actuator_mean_ms"] == 21
        assert result["observed_fps"] == 1  # Chỉ đếm khung đã thấy trong log, không suy đoán khung bị bỏ.
        for content in ("", "t\n0\n", "t,frame_id,image_age_ms,odom_cm,yaw_rel,fps,meas_speed,state\n"
                        "0,1,0,nan,0,0,0,DO\n"):
            path.write_text(content, encoding="utf-8")
            try:
                analyze(path)
            except ValueError:
                continue
            raise AssertionError("Phải từ chối log không hợp lệ")
    print("ĐẠT: bán kính, thống kê, đường dẫn Unicode, log rỗng/sai/NaN")


def main():
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="backslashreplace")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", nargs="?", type=Path)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if args.log is None:
        parser.error("Cần đường dẫn log.csv hoặc --self-test")
    try:
        result = analyze(args.log)
    except (OSError, ValueError, KeyError) as exc:
        parser.exit(1, "Không phân tích được: {}\n".format(exc))
    print("Ước lượng từ log, cần đối chiếu thước và đồng hồ trên xe:")
    for key, value in result.items():
        print("{}: {}".format(key, "chưa đủ mẫu" if value is None else "{:.4f}".format(value)))
    print("Chỉ dùng yaw_drift khi xe đứng yên; radius khi chạy tròn. FPS/Hz là số mẫu quan sát được trong log.")


if __name__ == "__main__":
    main()
