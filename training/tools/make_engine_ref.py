"""Tạo tensor tham chiếu đường từ ảnh thật và ONNX Runtime trên laptop."""
import argparse
import csv
import json
from pathlib import Path
import shutil
import sys

import cv2
import numpy as np


def load_info(path):
    info = json.loads(Path(path).read_text(encoding="utf-8"))
    fixed = {
        "task": "segmentation", "class_names": ["background", "track_surface", "line"],
        "num_classes": 3, "input_dtype": "float32", "color_order": "RGB",
        "layout": "NCHW", "output_type": "logits",
        "activation": "softmax_over_channels_argmax",
        "resize": "cv2.resize INTER_AREA (crop -> input)",
        "preprocessing_in_model": False, "fake_data": False, "dynamic": False,
    }
    for key, value in fixed.items():
        if key not in info or type(info[key]) is not type(value) or info[key] != value:
            raise ValueError("Trường không hợp lệ: " + key)
    for key, length in (("source_image_hw", 2), ("input_hw", 2), ("crop_y", 2), ("crop_x", 2)):
        values = info.get(key)
        if not isinstance(values, list) or len(values) != length or any(type(v) is not int for v in values):
            raise ValueError("Kích thước không hợp lệ: " + key)
    h, w = info["input_hw"]
    sh, sw = info["source_image_hw"]
    y0, y1 = info["crop_y"]
    x0, x1 = info["crop_x"]
    if min(h, w, sh, sw) <= 0 or not (0 <= y0 < y1 <= sh and 0 <= x0 < x1 <= sw):
        raise ValueError("Crop hoặc cỡ ảnh không hợp lệ")
    for key in ("input_shape", "output_shape"):
        shape = info.get(key)
        if shape != [1, 3, h, w] or any(type(v) is not int for v in shape):
            raise ValueError("Shape không hợp lệ: " + key)
    norm = info.get("normalization", {})
    if (not isinstance(norm, dict) or norm.get("scale") != 1 / 255
            or norm.get("mean") != [0, 0, 0] or norm.get("std") != [1, 1, 1]):
        raise ValueError("normalization phải là /255, mean=0, std=1")
    if (type(norm["scale"]) not in (float, int)
            or any(type(v) is not int for key in ("mean", "std") for v in norm[key])):
        raise ValueError("Kiểu normalization không hợp lệ")
    for key in ("input_name", "output_name"):
        if not isinstance(info.get(key), str) or not info[key].strip():
            raise ValueError("Thiếu tên tensor: " + key)
    if info["input_name"] == info["output_name"]:
        raise ValueError("Tên input/output trùng nhau")
    return info


def preprocess(image, info):
    if image is None or image.dtype != np.uint8 or image.shape != (*info["source_image_hw"], 3):
        raise ValueError("Ảnh phải là BGR uint8 đúng source_image_hw")
    y0, y1 = info["crop_y"]
    x0, x1 = info["crop_x"]
    h, w = info["input_hw"]
    resized = cv2.resize(image[y0:y1, x0:x1], (w, h), interpolation=cv2.INTER_AREA)
    if resized.dtype != np.uint8:
        raise ValueError("Ảnh resize phải là uint8 trước khi chia 255")
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    return np.ascontiguousarray((rgb.astype(np.float32) * np.float32(info["normalization"]["scale"]))
                                .transpose(2, 0, 1)[None], dtype="<f4")


def read_rows(path, required):
    with Path(path).open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if not required.issubset(reader.fieldnames or []):
            raise ValueError("Thiếu cột CSV: " + str(path))
        return list(reader)


def candidates(images, manifest, split):
    splits = {}
    for row in read_rows(split, {"file", "pass", "split"}):
        if row["file"] in splits:
            raise ValueError("Trùng ảnh trong split: " + row["file"])
        splits[row["file"]] = row
    rows, seen = [], set()
    for row in read_rows(manifest, {"file", "car", "pass", "t"}):
        name = row["file"]
        if row["car"] != "car6":
            continue
        if not name.isascii() or Path(name).name != name or "\\" in name or any(c.isspace() for c in name):
            raise ValueError("Tên ảnh không an toàn trong manifest: " + name)
        if name in seen:
            raise ValueError("Trùng ảnh trong manifest: " + name)
        seen.add(name)
        if not (images / name).is_file():
            continue
        entry = splits.get(name)
        if entry is None or entry["pass"] != row["pass"] or entry["split"] not in {"train", "val", "test"}:
            raise ValueError("Split thiếu hoặc sai lượt: " + name)
        t = int(row["t"])
        labels = []
        if t == 11:
            labels.append("ham")
        if t == 10:
            labels.append("cau")
        if 2 <= t <= 5:
            labels.append("giao_lo")
        if entry["split"] == "val":
            labels.append("val")
        pass_id = row["pass"]
        if not pass_id.isdigit():
            raise ValueError("Lượt không hợp lệ: " + pass_id)
        rows.append((name, pass_id, labels))
    return sorted(rows, key=lambda r: (int(r[1]), r[0]))


def select_rows(rows, count):
    if count < 20:
        raise ValueError("Cần ít nhất 20 ảnh tham chiếu")
    if len(rows) < 20:
        raise ValueError("Dữ liệu có ít hơn 20 ảnh car6")
    selected = []
    required = ("ham", "cau", "giao_lo", "val")
    missing = []
    for label in required:
        choices = [r for r in rows if label in r[2]]
        if not choices:
            missing.append(label)
        elif not any(label in r[2] for r in selected):
            selected.append(choices[len(choices) // 2])
    groups = {}
    for row in rows:
        groups.setdefault(row[1], []).append(row)
    # Lấy giữa mỗi lượt, sau đó rải đều các vị trí trong lượt.
    queues = []
    for group in groups.values():
        n = min(count, len(group))
        positions = [len(group) // 2] + [i * (len(group) - 1) // max(1, n - 1) for i in range(n)]
        queues.append([group[i] for i in dict.fromkeys(positions) if group[i] not in selected])
    while len(selected) < min(count, len(rows)):
        active = [queue for queue in queues if queue]
        represented = {row[1] for row in selected}
        fresh = [queue for queue in active if queue[0][1] not in represented]
        if fresh:
            active = fresh
        if not active:
            break
        remaining = min(count, len(rows)) - len(selected)
        if len(active) > remaining:
            indices = ([len(active) // 2] if remaining == 1 else
                       [i * (len(active) - 1) // (remaining - 1) for i in range(remaining)])
            active = [active[i] for i in indices]
        for queue in active:
            selected.append(queue.pop(0))
    return sorted(selected, key=lambda r: (int(r[1]), r[0])), missing


def write_npy(path, tensor):
    with path.open("wb") as stream:
        np.lib.format.write_array(stream, np.ascontiguousarray(tensor, dtype="<f4"),
                                  version=(1, 0), allow_pickle=False)


def generate(args):
    import onnxruntime as ort
    info = load_info(args.info)
    rows, missing = select_rows(candidates(args.images, args.manifest, args.split), 30)
    print("Thiếu loại trong dữ liệu: " + (", ".join(missing) if missing else "không"))
    session = ort.InferenceSession(str(args.onnx), providers=["CPUExecutionProvider"])
    for tensors, prefix in ((session.get_inputs(), "input"), (session.get_outputs(), "output")):
        if (len(tensors) != 1 or tensors[0].name != info[prefix + "_name"]
                or tensors[0].shape != info[prefix + "_shape"] or tensors[0].type != "tensor(float)"):
            raise ValueError("ONNX không khớp metadata: " + prefix)
    args.output.mkdir(parents=True, exist_ok=False)
    lines = []
    for index, (name, pass_id, labels) in enumerate(rows):
        source = args.images / name
        image = cv2.imdecode(np.frombuffer(source.read_bytes(), dtype=np.uint8), cv2.IMREAD_COLOR)
        blob = preprocess(image, info)
        logits = session.run([info["output_name"]], {info["input_name"]: blob})[0]
        if logits.shape != tuple(info["output_shape"]) or logits.dtype != np.float32 or not np.isfinite(logits).all():
            raise ValueError("Logits không hợp lệ: " + name)
        write_npy(args.output / (f"{index:03d}_input.npy"), blob)
        write_npy(args.output / (f"{index:03d}_logits.npy"), logits)
        shutil.copyfile(source, args.output / name)
        lines.append(f"{name}\t{pass_id}\t{','.join(labels) or '-'}\n")
    (args.output / "list.txt").write_text("".join(lines), encoding="ascii")
    print(f"Đã tạo {len(rows)} ảnh, {len({r[1] for r in rows})} lượt: {args.output.resolve()}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("onnx", "info", "images", "manifest", "split", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    try:
        generate(args)
    except Exception as error:
        print("Lỗi: " + str(error), file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
