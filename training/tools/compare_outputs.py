"""Đối chiếu mặt nạ/khung đã lưu hoặc sinh kết quả tham chiếu từ ONNX."""

import argparse
import importlib
import json
import math
from pathlib import Path
import struct
import sys
import zlib


def dependency(name, package=None):
    try:
        return importlib.import_module(name)
    except ImportError as exc:
        raise ValueError("Thiếu {}. Cài trên Ubuntu/Colab: python -m pip install {}".format(
            name, package or name)) from exc


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def write_json(path, data):
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")


def read_image(path):
    """Đọc ảnh màu qua bộ đệm byte để hỗ trợ đường dẫn có dấu trên Windows."""
    cv2, np = dependency("cv2", "opencv-python"), dependency("numpy")
    path = Path(path)
    try:
        data = np.frombuffer(path.read_bytes(), dtype=np.uint8)
        image = cv2.imdecode(data, cv2.IMREAD_COLOR) if data.size else None
    except (OSError, cv2.error) as exc:
        raise ValueError("Không đọc được ảnh: " + str(path)) from exc
    if image is None:
        raise ValueError("Không đọc được ảnh: " + str(path))
    return image


def write_image(path, image):
    """Mã hoá ảnh rồi ghi byte để hỗ trợ đường dẫn có dấu trên Windows."""
    cv2 = dependency("cv2", "opencv-python")
    path = Path(path)
    try:
        ok, data = cv2.imencode(path.suffix, image)
        if not ok:
            raise ValueError("Không mã hoá được ảnh: " + str(path))
        path.write_bytes(data.tobytes())
    except (OSError, cv2.error) as exc:
        raise ValueError("Không ghi được ảnh: " + str(path)) from exc


def load_info(path):
    """Chỉ nhận hợp đồng RGB chuẩn hoá ngoài mạng, không tự suy đoán bản cũ."""
    d = read_json(path)
    common = {"color_order": "RGB", "layout": "NCHW", "input_dtype": "float32",
              "preprocessing_in_model": False, "opset": 12, "dynamic": False}
    for k, v in common.items():
        if k not in d or d[k] != v:
            raise ValueError("Thông tin mô hình không hỗ trợ: {} cần bằng {}".format(k, v))
    names = d["class_names"]
    if not names or len(set(names)) != len(names) or len(names) != d["num_classes"]:
        raise ValueError("Danh sách lớp rỗng, trùng hoặc sai số lớp")
    h, w = d["input_hw"]
    if any(type(v) is not int or v <= 0 for v in (h, w)) or d["input_shape"] != [1, 3, h, w]:
        raise ValueError("Kích thước đầu vào phải cố định [1,3,H,W]")
    n = d["normalization"]
    if not math.isclose(n["scale"], 1 / 255) or n["mean"] != [0, 0, 0] or n["std"] != [1, 1, 1]:
        raise ValueError("Chuẩn hoá phải là chia 255, không trừ trung bình")
    if d["task"] == "segmentation":
        if (d["resize"] != "stretch_bilinear" or d["output_type"] != "logits"
                or d["activation"] != "sigmoid_per_class"
                or d["output_shape"] != [1, len(names), h, w]):
            raise ValueError("Hợp đồng phân vùng không hợp lệ")
        probability(d["threshold"])
    elif d["task"] == "detection":
        expected = {"auto": False, "center": True, "padding_value": 114, "scaleup": True, "stride": 32}
        if (d["resize"] != "letterbox" or d["letterbox"] != expected
                or d["output_type"] != "cxcywh_pixels_and_class_probabilities"
                or d["box_format"] != "cxcywh" or d["box_units"] != "input_pixels"
                or d["objectness"] is not False or d["nms_in_model"] is not False
                or d["scores_are_probabilities"] is not True or d["nms_class_aware"] is not True
                or len(d["output_shape"]) != 3 or d["output_shape"][:2] != [1, 4 + len(names)]):
            raise ValueError("Hợp đồng phát hiện không hợp lệ")
        probability(d["conf_threshold"])
        probability(d["nms_iou"])
    else:
        raise ValueError("Loại mô hình không hỗ trợ")
    return d


def probability(value):
    if not math.isfinite(value) or not 0 <= value <= 1:
        raise ValueError("Ngưỡng hoặc xác suất phải trong [0,1]")
    return value


def image_paths(root):
    paths = sorted(p for p in Path(root).rglob("*") if p.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp"})
    if not paths:
        raise ValueError("Thư mục không có ảnh")
    return paths


def preprocess(image, info):
    cv2, np = dependency("cv2", "opencv-python"), dependency("numpy")
    if image is None:
        raise ValueError("Không đọc được ảnh")
    oh, ow = image.shape[:2]
    h, w = info["input_hw"]
    transform = {"original_hw": [oh, ow], "input_hw": [h, w], "resize": info["resize"]}
    if info["resize"] == "stretch_bilinear":
        image = cv2.resize(image, (w, h), interpolation=cv2.INTER_LINEAR)
        transform.update(scale_xy=[w / ow, h / oh], pad_ltrb=[0, 0, 0, 0])
    else:
        r = min(h / oh, w / ow)
        nw, nh = round(ow * r), round(oh * r)
        dx, dy = (w - nw) / 2, (h - nh) / 2
        left, right = round(dx - 0.1), round(dx + 0.1)
        top, bottom = round(dy - 0.1), round(dy + 0.1)
        image = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_LINEAR)
        image = cv2.copyMakeBorder(image, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))
        transform.update(scale_xy=[r, r], pad_ltrb=[left, top, right, bottom])
    x = np.ascontiguousarray(image[:, :, ::-1].transpose(2, 0, 1)[None], dtype=np.float32) / 255
    return x, transform


def box_iou(a, b):
    inter = max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(0, min(a[3], b[3]) - max(a[1], b[1]))
    area = lambda z: max(0, z[2] - z[0]) * max(0, z[3] - z[1])
    union = area(a) + area(b) - inter
    return inter / union if union else 0.0


def coords(d):
    return [d[k] for k in ("x1", "y1", "x2", "y2")]


def match_boxes(a, b, threshold=0.9):
    """Ghép một-một nhiều khung nhất bằng đường tăng, cùng mã và tên lớp."""
    edges = {i: sorted([j for j, y in enumerate(b) if x["class_id"] == y["class_id"]
                       and x["class_name"] == y["class_name"] and box_iou(coords(x), coords(y)) >= threshold],
                      key=lambda j: -box_iou(coords(x), coords(b[j]))) for i, x in enumerate(a)}
    owner = {}

    def augment(i, seen):
        for j in edges[i]:
            if j in seen:
                continue
            seen.add(j)
            if j not in owner or augment(owner[j], seen):
                owner[j] = i
                return True
        return False

    for i in edges:
        augment(i, set())
    pairs = [(i, j) for j, i in owner.items()]
    return {"matched": len(pairs), "only_a": len(a) - len(pairs), "only_b": len(b) - len(pairs),
            "min_box_iou": min((box_iou(coords(a[i]), coords(b[j])) for i, j in pairs), default=1.0),
            "max_score_error": max((abs(a[i]["score"] - b[j]["score"]) for i, j in pairs), default=0.0)}


def mask_iou(a, b):
    """Nhận dãy điểm ảnh phẳng; hai mặt nạ rỗng có IoU bằng 1."""
    if len(a) != len(b) or not len(a):
        raise ValueError("Mặt nạ rỗng hoặc khác kích thước")
    if any(v not in (0, 255) for v in a) or any(v not in (0, 255) for v in b):
        raise ValueError("Mặt nạ phải chỉ chứa 0/255")
    intersection = sum(bool(x) and bool(y) for x, y in zip(a, b))
    union = sum(bool(x) or bool(y) for x, y in zip(a, b))
    return intersection / union if union else 1.0


def read_mask(path, hw):
    """Đọc PNG xám 8 bit không xen kẽ, kiểm CRC, không cần thư viện ngoài."""
    data = Path(path).read_bytes()
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("Mặt nạ không phải PNG")
    pos, compressed, header, ended = 8, bytearray(), None, False
    while pos < len(data):
        length = struct.unpack_from(">I", data, pos)[0]
        tag = data[pos+4:pos+8]
        payload = data[pos+8:pos+8+length]
        crc = struct.unpack_from(">I", data, pos+8+length)[0]
        if zlib.crc32(tag + payload) & 0xffffffff != crc:
            raise ValueError("PNG có CRC sai")
        if tag == b"IHDR":
            header = struct.unpack(">IIBBBBB", payload)
        elif tag == b"IDAT":
            compressed.extend(payload)
        elif tag == b"IEND":
            ended = True
            break
        pos += length + 12
    h, w = hw
    if not ended or header != (w, h, 8, 0, 0, 0, 0):
        raise ValueError("PNG phải là ảnh xám 8 bit không xen kẽ đúng kích thước mô hình")
    raw = zlib.decompress(compressed)
    if len(raw) != h * (w + 1):
        raise ValueError("PNG sai số byte điểm ảnh")
    result, previous = bytearray(), bytearray(w)
    for y in range(h):
        mode = raw[y * (w+1)]
        if mode > 4:
            raise ValueError("PNG có bộ lọc không hợp lệ")
        row = bytearray(raw[y*(w+1)+1:(y+1)*(w+1)])
        for x in range(w):
            left, up, corner = (row[x-1] if x else 0), previous[x], (previous[x-1] if x else 0)
            p = left + up - corner
            distances = [abs(p-left), abs(p-up), abs(p-corner)]
            paeth = [left, up, corner][distances.index(min(distances))]
            predictor = [0, left, up, (left+up)//2, paeth][mode]
            row[x] = (row[x] + predictor) & 255
        result.extend(row)
        previous = row
    return result


def decode_detection(raw, info, transform):
    np = dependency("numpy")
    pred = raw[0].T
    cls = pred[:, 4:].argmax(1)
    scores = pred[np.arange(len(pred)), cls + 4]
    selected = np.flatnonzero(scores >= info["conf_threshold"])
    left, top = transform["pad_ltrb"][:2]
    sx, sy = transform["scale_xy"]
    oh, ow = transform["original_hw"]
    candidates = []
    for i in selected:
        cx, cy, w, h = map(float, pred[i, :4])
        candidates.append(dict(class_id=int(cls[i]), class_name=info["class_names"][cls[i]], score=float(scores[i]),
                               x1=(cx-w/2-left)/sx, y1=(cy-h/2-top)/sy,
                               x2=(cx+w/2-left)/sx, y2=(cy+h/2-top)/sy))
    # NMS theo lớp trước khi chặn biên ảnh, giữ thứ tự điểm giảm dần ổn định.
    kept = []
    for d in sorted(candidates, key=lambda d: -d["score"]):
        if any(d["class_id"] == e["class_id"] and box_iou(coords(d), coords(e)) > info["nms_iou"] for e in kept):
            continue
        kept.append(d)
    for d in kept:
        for k, limit in (("x1", ow), ("x2", ow), ("y1", oh), ("y2", oh)):
            d[k] = min(limit, max(0, d[k]))
    return kept


def infer_folder(model, info, images, output):
    np, ort = dependency("numpy"), dependency("onnxruntime")
    session = ort.InferenceSession(str(model), providers=["CPUExecutionProvider"])
    if (session.get_inputs()[0].shape != info["input_shape"] or session.get_inputs()[0].name != info["input_name"]
            or session.get_inputs()[0].type != "tensor(float)"
            or session.get_outputs()[0].shape != info["output_shape"]
            or session.get_outputs()[0].name != info["output_name"]):
        raise ValueError("Mô hình ONNX khác thông tin mô tả")
    for p in image_paths(images):
        rel = p.relative_to(images)
        dest = Path(output) / (rel.as_posix() + ".json")
        if dest.exists():
            raise ValueError("Kết quả đã tồn tại: " + str(dest))
        x, transform = preprocess(read_image(p), info)
        raw = session.run([info["output_name"]], {info["input_name"]: x})[0]
        if raw.shape != tuple(info["output_shape"]) or not np.isfinite(raw).all():
            raise ValueError("Đầu ra ONNX sai kích thước hoặc không hữu hạn")
        data = dict(schema_version=1, image=rel.as_posix(), task=info["task"], transform=transform)
        dest.parent.mkdir(parents=True, exist_ok=True)
        if info["task"] == "detection":
            data.update(conf_threshold=info["conf_threshold"], nms_iou=info["nms_iou"], detections=decode_detection(raw, info, transform))
        else:
            masks = (1 / (1 + np.exp(-np.clip(raw[0], -80, 80)))) >= info["threshold"]
            data.update(threshold=info["threshold"], masks=[])
            for i, mask in enumerate(masks):
                name = rel.name + ".class_{}.png".format(i)
                target = dest.parent / name
                if target.exists():
                    raise ValueError("Không ghi được mặt nạ mới: " + str(target))
                write_image(target, mask.astype(np.uint8) * 255)
                data["masks"].append(dict(class_id=i, class_name=info["class_names"][i], file=name))
        write_json(dest, data)


def validate_result(d, info):
    if d["schema_version"] != 1 or d["task"] != info["task"]:
        raise ValueError("Sai phiên bản hoặc loại kết quả")
    t = d["transform"]
    if t["input_hw"] != info["input_hw"] or t["resize"] != info["resize"]:
        raise ValueError("Kết quả dùng phép đổi kích thước khác mô hình")
    if any(v <= 0 for v in t["original_hw"] + t["scale_xy"]):
        raise ValueError("Phép đổi toạ độ không hợp lệ")
    oh, ow = t["original_hw"]
    h, w = info["input_hw"]
    if t["resize"] == "stretch_bilinear":
        scales, pads = [w/ow, h/oh], [0, 0, 0, 0]
    else:
        r = min(h/oh, w/ow)
        dx, dy = (w-round(ow*r))/2, (h-round(oh*r))/2
        scales, pads = [r, r], [round(dx-0.1), round(dy-0.1), round(dx+0.1), round(dy+0.1)]
    if t["pad_ltrb"] != pads or any(not math.isclose(a, b, rel_tol=1e-9) for a, b in zip(t["scale_xy"], scales)):
        raise ValueError("Tham số biến đổi không khớp kích thước ảnh")
    if d["task"] == "detection":
        if d["conf_threshold"] != info["conf_threshold"] or d["nms_iou"] != info["nms_iou"]:
            raise ValueError("Ngưỡng phát hiện/NMS khác mô hình")
        for det in d["detections"]:
            i = det["class_id"]
            if type(i) is not int or not 0 <= i < info["num_classes"] or det["class_name"] != info["class_names"][i]:
                raise ValueError("Sai thứ tự lớp trong kết quả")
            probability(det["score"])
            c = coords(det)
            oh, ow = t["original_hw"]
            if not all(math.isfinite(v) for v in c) or not (0 <= c[0] <= c[2] <= ow and 0 <= c[1] <= c[3] <= oh):
                raise ValueError("Khung không hợp lệ")
    else:
        if d["threshold"] != info["threshold"] or [(m["class_id"], m["class_name"]) for m in d["masks"]] != list(enumerate(info["class_names"])):
            raise ValueError("Ngưỡng hoặc lớp mặt nạ khác mô hình")


def compare_folders(a, b, info, mask_threshold=0.98, box_threshold=0.9):
    files_a = {p.relative_to(a).as_posix(): p for p in Path(a).rglob("*.json")}
    files_b = {p.relative_to(b).as_posix(): p for p in Path(b).rglob("*.json")}
    if not files_a and not files_b:
        raise ValueError("Không có kết quả JSON")
    rows = []
    for name in sorted(files_a.keys() | files_b.keys()):
        if name not in files_a or name not in files_b:
            rows.append(dict(image=name, passed=False, reason="Thiếu kết quả một bên"))
            continue
        x, y = read_json(files_a[name]), read_json(files_b[name])
        validate_result(x, info)
        validate_result(y, info)
        if (x["image"] != y["image"] or x["transform"]["original_hw"] != y["transform"]["original_hw"]):
            raise ValueError("Hai kết quả khác ảnh hoặc phép đổi toạ độ: " + name)
        row = dict(image=x["image"])
        if info["task"] == "detection":
            row.update(match_boxes(x["detections"], y["detections"], box_threshold))
            row["passed"] = row["only_a"] == row["only_b"] == 0
        else:
            values = []
            for mx, my in zip(x["masks"], y["masks"]):
                masks = []
                for entry, file in ((mx, files_a[name]), (my, files_b[name])):
                    target = (file.parent / entry["file"]).resolve()
                    if not target.is_relative_to(file.parent.resolve()):
                        raise ValueError("Đường dẫn mặt nạ phải nằm cạnh JSON")
                    masks.append(read_mask(target, info["input_hw"]))
                values.append(mask_iou(*masks))
            row.update(mask_ious=values, min_mask_iou=min(values), passed=min(values) >= mask_threshold)
        rows.append(row)
    return sorted(rows, key=lambda r: (r["passed"], r.get("min_mask_iou", r.get("min_box_iou", 0))))


def main():
    # Đầu ra chuyển hướng trên Windows cũng phải in được đầy đủ dấu tiếng Việt.
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--info", required=True, type=Path, help="File _info.json")
    ap.add_argument("--onnx", type=Path, help="Sinh tham chiếu ONNX trước khi so")
    ap.add_argument("--images", type=Path, help="Thư mục ảnh gốc, duyệt cả thư mục con")
    ap.add_argument("--reference", required=True, type=Path, help="Thư mục tham chiếu, hoặc nơi ghi ONNX mới")
    ap.add_argument("--candidate", type=Path, help="Thư mục kết quả cần so; bỏ trống để chỉ sinh tham chiếu")
    ap.add_argument("--report", type=Path, help="Lưu bảng đối chiếu JSON")
    ap.add_argument("--mask-iou", type=float, default=0.98)
    ap.add_argument("--box-iou", type=float, default=0.9)
    ap.add_argument("--worst", type=int, default=20, help="Số ảnh lệch nhất cần in")
    args = ap.parse_args()
    try:
        info = load_info(args.info)
        probability(args.mask_iou)
        probability(args.box_iou)
        if args.worst <= 0:
            raise ValueError("Số ảnh cần in phải dương")
        if args.onnx:
            if not args.images:
                raise ValueError("--onnx cần --images")
            infer_folder(args.onnx, info, args.images, args.reference)
        elif not args.candidate:
            raise ValueError("Cần --candidate hoặc --onnx")
        if args.candidate:
            rows = compare_folders(args.reference, args.candidate, info, args.mask_iou, args.box_iou)
            print("Ảnh | Đạt | Chỉ số đối chiếu")
            for row in rows[:args.worst]:
                print(row["image"], "|", row["passed"], "|", json.dumps(row, ensure_ascii=False))
            print("Đạt {}/{} ảnh".format(sum(r["passed"] for r in rows), len(rows)))
            if args.report:
                write_json(args.report, rows)
            return 0 if all(r["passed"] for r in rows) else 1
        print("Đã sinh kết quả tham chiếu:", args.reference)
        return 0
    except Exception as exc:
        print("Lỗi:", exc, file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
