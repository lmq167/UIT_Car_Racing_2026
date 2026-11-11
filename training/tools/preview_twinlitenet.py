#!/usr/bin/env python3
"""Preview mask TwinLiteNetPlus 192x320 theo lượt thu (công cụ CHẨN ĐOÁN).

Chạy:  python preview_twinlitenet.py            (hỏi "Nhập lượt thu:")
       python preview_twinlitenet.py 106        (truyền thẳng lượt thu)

Chỉ xử lý ảnh car6_pXXXX_YYYY.(jpg|jpeg|png) của đúng lượt nhập, không bao giờ quét cả thư mục.

Pipeline mặc định (THEO BENCHMARK benchmark.cpp/benchmark.py trên Jetson Nano, do người dùng cung cấp
qua ghi chú phân tích; công cụ này chưa tự đọc hai file đó):
  - Tiền xử lý: resize toàn ảnh về 320x192 bằng cv2.INTER_LINEAR (không crop, không padding) -> BGR2RGB
    -> chia 255 -> trừ mean/chia std ImageNet -> NCHW float32 [1,3,192,320].
  - Giải mã: output [1,2,192,320]; mặt đường khi output[1] > output[0] (không sigmoid/softmax/ngưỡng 0,5).
  - Overlay: xanh lá BGR (0,220,0), alpha 0,5.
Pipeline U-Net của xe (RoadSegmenter.cpp: crop y320-720, INTER_AREA, chia 255) KHÔNG dùng làm chuẩn cho
TwinLiteNetPlus; vẫn nằm trong các biến thể chẩn đoán và qua --config/--resize.
CHƯA XÁC MINH: ONNX trên laptop có cùng trọng số với engine TensorRT đã benchmark trên xe hay không.
"""
import argparse
import csv
import hashlib
import json
import os
import re
import sys
import time
from collections import namedtuple
from pathlib import Path

try:
    import cv2
    import numpy as np
    import onnxruntime as ort
except ImportError as exc:
    sys.exit("Thiếu thư viện: %s\nCài bằng: python -m pip install numpy opencv-python onnxruntime" % exc)

MODEL = Path(r"models\twinlitenetplus_192x320.onnx")
IMAGES = Path(r"data\round3\clean_car6_v3\images")
OUT = Path(r"data\round3\preview_twinlitenet")
DIAG = Path(r"data\round3\preview_twinlitenet_diag")

SRC_HW = (720, 1280)          # source_image_hw của camera car6 (theo info.json của U-Net)
CROP_Y = (320, 720)           # crop_y của U-Net car6; chỉ là GIẢ THUYẾT cho TwinLiteNetPlus
MEAN = np.array([0.485, 0.456, 0.406], np.float32)   # ImageNet, thứ tự R,G,B
STD = np.array([0.229, 0.224, 0.225], np.float32)
INTERP_NAME = {"linear": "INTER_LINEAR", "area": "INTER_AREA"}
KINDS = ("resized", "mask", "overlay", "comparison")  # đúng bốn thư mục đầu ra bắt buộc
ALPHA = 0.5
GREEN = (0, 220, 0)  # BGR, màu overlay theo benchmark
ZOOM = 2
UNVERIFIED = "CHUA XAC MINH"  # ASCII: cv2.putText không vẽ được dấu tiếng Việt
BENCH = "THEO BENCHMARK"
INTERP = {"linear": cv2.INTER_LINEAR, "area": cv2.INTER_AREA}

IMAGES_DIR = IMAGES
Cfg = namedtuple("Cfg", "crop color norm")  # crop: full|y320, color: rgb|bgr, norm: div255|imagenet


def cfg_id(c):
    return "-".join(c)


def parse_cfg(text):
    parts = text.split("-")
    if len(parts) != 3 or parts[0] not in ("full", "y320") or parts[1] not in ("rgb", "bgr") \
            or parts[2] not in ("div255", "imagenet"):
        raise argparse.ArgumentTypeError("cấu hình phải dạng {full|y320}-{rgb|bgr}-{div255|imagenet}")
    return Cfg(*parts)


VARIANTS = [Cfg(a, b, c) for a in ("y320", "full") for b in ("rgb", "bgr") for c in ("div255", "imagenet")]
DEFAULT_CFG = Cfg("full", "rgb", "imagenet")  # theo benchmark; resize mặc định INTER_LINEAR
DEFAULT_RESIZE = "linear"


def pass_prefix(text):
    """'106' hoặc 'p0106' -> ('p0106', regex khớp đúng car6_p0106_<số>.<đuôi>)."""
    m = re.fullmatch(r"[pP]?(\d{1,4})", text.strip())
    if not m or int(m.group(1)) == 0:
        raise ValueError("Lượt thu phải là số 1-9999 (ví dụ 106), nhận: %r" % text)
    tag = "p%04d" % int(m.group(1))
    return tag, re.compile(r"car6_%s_\d+\.(?:jpe?g|png)" % tag, re.I)


def find_images(folder, pattern):
    """Chỉ duyệt một cấp, chỉ giữ tệp khớp hoàn toàn mẫu của lượt (không khớp chuỗi con)."""
    names = [p.name for p in Path(folder).iterdir() if p.is_file() and pattern.fullmatch(p.name)]
    stems = {}
    for n in sorted(names):
        stems.setdefault(Path(n).stem, []).append(n)
    ok = [v[0] for v in stems.values() if len(v) == 1]
    conflicts = {k: v for k, v in stems.items() if len(v) > 1}
    return ok, conflicts


def read_bgr(path):
    img = cv2.imdecode(np.fromfile(str(path), np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("không giải mã được ảnh")
    if img.shape[:2] != SRC_HW:
        raise ValueError("kích thước %dx%d khác %dx%d" % (img.shape[1], img.shape[0], SRC_HW[1], SRC_HW[0]))
    return img


def write_png(path, img):
    """Ghi PNG qua tệp tạm cùng thư mục rồi os.replace; không bao giờ xóa tệp có sẵn của người dùng.
    Lỗi ghi được báo kèm errno/winerror, đường dẫn, độ dài đường dẫn và tình trạng tệp đích."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, buf = cv2.imencode(".png", img)
    if not ok:
        raise OSError("không mã hóa được PNG: %s" % path)
    data = buf.tobytes()
    tmp = path.with_name(".%s.%d.tmp" % (path.name, os.getpid()))
    try:
        with open(tmp, "wb") as f:
            f.write(data)
        os.replace(tmp, path)
    except OSError as exc:
        try:
            tmp.unlink()  # chỉ xóa tệp tạm do chính lần ghi này tạo
        except OSError:
            pass
        existed = path.exists()
        ro = existed and not os.access(path, os.W_OK)
        raise OSError(exc.errno, "ghi lỗi %s [%s: errno=%s winerror=%s; đường dẫn dài %d ký tự; tệp đích %s%s; "
                      "tệp cũ được giữ nguyên, không xóa. Nếu tệp đang mở trong trình xem ảnh/Explorer/OneDrive thì đóng rồi chạy lại]"
                      % (path, type(exc).__name__, exc.errno, getattr(exc, "winerror", None), len(str(path)),
                         "ĐÃ TỒN TẠI từ trước" if existed else "chưa có", ", chỉ-đọc" if ro else "")) from exc


def preprocess(bgr, cfg, size, resize=DEFAULT_RESIZE):
    """Trả (ảnh resize BGR uint8, tensor 1x3xHxW float32). resize: linear (benchmark) | area (U-Net xe)."""
    w, h = size
    src = bgr if cfg.crop == "full" else bgr[CROP_Y[0]:CROP_Y[1]]
    small = cv2.resize(src, (w, h), interpolation=INTERP[resize])
    img = cv2.cvtColor(small, cv2.COLOR_BGR2RGB) if cfg.color == "rgb" else small
    x = img.astype(np.float32) * np.float32(1 / 255)
    if cfg.norm == "imagenet":  # thống kê đi theo kênh màu thực tế của tensor
        m, s = (MEAN, STD) if cfg.color == "rgb" else (MEAN[::-1], STD[::-1])
        x = (x - m) / s
    return small, np.ascontiguousarray(x.transpose(2, 0, 1)[None], dtype=np.float32)


class Model:
    def __init__(self, path):
        self.sess = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
        ins, outs = self.sess.get_inputs(), self.sess.get_outputs()
        if len(ins) != 1 or len(outs) != 1:
            raise RuntimeError("Cần đúng 1 input và 1 output, thực tế %d/%d" % (len(ins), len(outs)))
        i, o = ins[0], outs[0]
        if i.type != "tensor(float)" or len(i.shape) != 4 or i.shape[0] != 1 or i.shape[1] != 3 \
                or not all(isinstance(d, int) for d in i.shape):
            raise RuntimeError("Input phải là float32 [1,3,H,W] tĩnh, thực tế %s %s" % (i.type, i.shape))
        if len(o.shape) != 4 or o.shape[2:] != i.shape[2:] or not isinstance(o.shape[1], int) or o.shape[1] < 2:
            raise RuntimeError("Output phải là [1,C>=2,H,W] cùng H,W với input, thực tế %s" % (o.shape,))
        self.inp, self.out = i.name, o.name
        self.size = (i.shape[3], i.shape[2])  # (W, H)
        self.classes = o.shape[1]
        self.table = [("input", i.name, i.shape, i.type), ("output", o.name, o.shape, o.type)]

    def logits(self, x):
        y = self.sess.run([self.out], {self.inp: x})[0]
        if y.shape != (1, self.classes) + self.size[::-1] or not np.isfinite(y).all():
            raise RuntimeError("Output sai shape hoặc có NaN/Inf: %s" % (y.shape,))
        return y[0]


def road_mask(logits, road):
    """Mặt đường khi điểm của kênh road lớn hơn mọi kênh còn lại (2 lớp, road=1: output[1] > output[0])."""
    return logits[road] > np.delete(logits, road, axis=0).max(axis=0)


def tint(img, mask, bgr, alpha=ALPHA):
    out = img.copy()
    out[mask] = (img[mask] * (1 - alpha) + np.array(bgr, np.float32) * alpha).astype(np.uint8)
    return out


def zoom(img):
    return cv2.resize(img, None, fx=ZOOM, fy=ZOOM, interpolation=cv2.INTER_NEAREST)


def caption(img, text, color=(255, 255, 255), bar=22):
    canvas = np.zeros((img.shape[0] + bar, img.shape[1], 3), np.uint8)
    canvas[bar:] = img
    cv2.putText(canvas, text, (6, bar - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    return canvas


def comparison(small, mask, overlay, head):
    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    panels = [caption(zoom(p), t) for p, t in ((small, "1. resized %dx%d" % small.shape[1::-1]),
                                               (mask_bgr, "2. mask mat duong (trang)"),
                                               (overlay, "3. overlay xanh la"))]
    return caption(np.hstack(panels), head, (0, 255, 255), bar=26)


def diag_grid(small_by_cfg, logits_by_cfg):
    """Mỗi biến thể 2 ô: argmax==0 (đỏ) và argmax==1 (xanh lá); ghi tỷ lệ pixel từng lớp."""
    rows = []
    for cfg in VARIANTS:
        cls = logits_by_cfg[cfg].argmax(0)
        s = small_by_cfg[cfg]
        cells = []
        for k, col in ((0, (0, 0, 255)), (1, (0, 255, 0))):
            m = cls == k
            cells.append(caption(tint(s, m, col, 0.6), "%s ch%d=%.1f%%" % (cfg_id(cfg), k, 100 * m.mean()), bar=18))
        rows.append(np.hstack(cells))
    return np.vstack([np.hstack(rows[i:i + 2]) for i in range(0, len(rows), 2)])


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def status(cfg, resize, road):
    return BENCH if (cfg, resize, road) == (DEFAULT_CFG, DEFAULT_RESIZE, 1) else UNVERIFIED


def header(tag, stem, cfg, resize, road):
    return "%s %s | %s %s | kenh mat duong=%d | %s" % (tag, stem, cfg_id(cfg), INTERP_NAME[resize], road,
                                                       status(cfg, resize, road))


def process(name, tag, model, cfg, road, resize, out, diag):
    """Ghi 4 thư mục chính TRƯỚC, rồi ảnh chẩn đoán. Trả (tỷ lệ đường, số liệu biến thể, lỗi ghi chẩn đoán hoặc None).
    Lỗi ghi 4 thư mục chính ném ngoại lệ (ảnh tính là lỗi); lỗi ghi chẩn đoán chỉ là cảnh báo riêng."""
    stem = Path(name).stem
    bgr = read_bgr(IMAGES_DIR / name)
    small, x = preprocess(bgr, cfg, model.size, resize)
    mask = (road_mask(model.logits(x), road) * 255).astype(np.uint8)
    overlay = tint(small, mask > 0, GREEN)
    head = header(tag, stem, cfg, resize, road)
    files = {"resized": small, "mask": mask, "overlay": overlay, "comparison": comparison(small, mask, overlay, head)}
    for kind, img in files.items():
        write_png(out / kind / tag / (stem + ".png"), img)
    stats, diag_err = [], None
    if diag is not None:
        smalls, logs = {}, {}
        for v in VARIANTS:
            smalls[v], xv = preprocess(bgr, v, model.size, resize)
            logs[v] = model.logits(xv)
            stats.append([stem, cfg_id(v)] + ["%.5f" % (logs[v].argmax(0) == k).mean() for k in range(model.classes)])
        try:
            write_png(diag / "variants" / tag / (stem + ".png"), diag_grid(smalls, logs))
        except OSError as exc:
            diag_err = str(exc)
    return float((mask > 0).mean()), stats, diag_err


def main(argv=None):
    global IMAGES_DIR
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("luot", nargs="?", help="lượt thu, ví dụ 106 (bỏ trống sẽ hỏi)")
    ap.add_argument("--config", type=parse_cfg, default=DEFAULT_CFG,
                    help="cấu hình tiền xử lý cho 4 thư mục chính (mặc định %s, theo benchmark)" % cfg_id(DEFAULT_CFG))
    ap.add_argument("--resize", choices=sorted(INTERP), default=DEFAULT_RESIZE,
                    help="nội suy khi resize (mặc định linear theo benchmark; area là của U-Net trên xe)")
    ap.add_argument("--road-channel", type=int, default=1,
                    help="kênh output coi là mặt đường (mặc định 1 theo benchmark: output[1] > output[0])")
    ap.add_argument("--no-diag", action="store_true", help="không ghi ảnh/số liệu so sánh các biến thể")
    ap.add_argument("--model", type=Path, default=MODEL)
    ap.add_argument("--images", type=Path, default=IMAGES)
    ap.add_argument("--out", type=Path, default=OUT)
    ap.add_argument("--diag-out", type=Path, default=DIAG)
    a = ap.parse_args(argv)
    IMAGES_DIR = a.images

    try:
        text = a.luot if a.luot is not None else input("Nhập lượt thu: ")
    except EOFError:
        sys.exit("Lỗi: không nhận được lượt thu (hãy nhập số, ví dụ 106). Không xử lý gì.")
    try:
        tag, pattern = pass_prefix(text)
    except ValueError as exc:
        sys.exit("Lỗi: %s" % exc)
    if not a.images.is_dir() or not a.model.is_file():
        sys.exit("Lỗi: không thấy thư mục ảnh %s hoặc model %s" % (a.images, a.model))
    out, diag = a.out.resolve(), a.diag_out.resolve()
    if a.images.resolve() in (out, diag) or a.images.resolve() in out.parents or a.images.resolve() in diag.parents:
        sys.exit("Lỗi: thư mục kết quả nằm trong thư mục ảnh gốc, từ chối ghi.")

    names, conflicts = find_images(a.images, pattern)
    total = len(names) + sum(len(v) for v in conflicts.values())
    print("Lượt thu: %s | tiền tố tên ảnh: car6_%s_ | tổng số ảnh khớp: %d" % (text.strip(), tag, total))
    if total == 0:
        passes = sorted({m.group(1) for p in a.images.iterdir() if (m := re.match(r"car6_(p\d{4})_", p.name))})
        sys.exit("Không có ảnh nào thuộc lượt %s; dừng, không xử lý ảnh khác. Các lượt có trong thư mục: %s"
                 % (tag, ", ".join(passes[:8]) + (" ..." if len(passes) > 8 else "")))

    t0 = time.time()
    model = Model(a.model)
    if not 0 <= a.road_channel < model.classes:
        sys.exit("Lỗi: --road-channel phải trong 0..%d" % (model.classes - 1))
    print("ONNX:", a.model, "| sha256", sha256(a.model)[:16], "| onnxruntime", ort.__version__, "| OpenCV", cv2.__version__)
    for row in model.table:
        print("  %-6s %-8s %-18s %s" % row)
    st = status(a.config, a.resize, a.road_channel)
    print("Cấu hình 4 thư mục chính: %s, resize cv2.%s, kênh mặt đường=%d  [%s]"
          % (cfg_id(a.config), INTERP_NAME[a.resize], a.road_channel, st))

    ok, fails, rows, diag_fails = 0, [], [], []
    for stem, files in conflicts.items():
        fails.append((", ".join(files), "xung đột: cùng ra %s.png, bỏ qua, không ghi đè" % stem))
    for name in names:
        try:
            frac, stats, diag_err = process(name, tag, model, a.config, a.road_channel, a.resize, out,
                                            None if a.no_diag else diag)
            rows += stats
            ok += 1
            if diag_err:
                diag_fails.append((name, diag_err))
        except Exception as exc:  # một ảnh lỗi không dừng cả lượt
            fails.append((name, str(exc)))
    if rows:
        path = diag / "stats" / (tag + "_variants.csv")
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["image", "variant"] + ["frac_ch%d" % k for k in range(model.classes)])
            w.writerows(rows)
    secs = time.time() - t0
    info = {"pass": tag, "config": cfg_id(a.config), "road_channel": a.road_channel, "resize": "cv2." + INTERP_NAME[a.resize],
            "status": st, "onnx_equals_trt_engine_verified": False, "model_sha256": sha256(a.model), "opencv": cv2.__version__, "onnxruntime": ort.__version__,
            "matched": total, "ok": ok, "failed": len(fails), "diag_write_failed": len(diag_fails), "seconds": round(secs, 2)}
    (diag / "run_info").mkdir(parents=True, exist_ok=True)
    (diag / "run_info" / (tag + ".json")).write_text(json.dumps(info, indent=1), encoding="utf-8")

    print("---- Báo cáo ----")
    print("Lượt thu: %s | tiền tố: car6_%s_" % (text.strip(), tag))
    print("Tổng số ảnh khớp: %d | thành công: %d | lỗi/bỏ qua: %d | thời gian: %.1f s" % (total, ok, len(fails), secs))
    for n, why in fails:
        print("  LỖI %s: %s" % (n, why))
    if diag_fails:
        print("Ảnh chẩn đoán ghi lỗi: %d (4 thư mục chính của các ảnh này đã ghi xong)" % len(diag_fails))
        for n, why in diag_fails:
            print("  CẢNH BÁO chẩn đoán %s: %s" % (n, why))
    for kind in KINDS:
        print("Kết quả %-10s: %s" % (kind, out / kind / tag))
    if not a.no_diag:
        print("Chẩn đoán các biến thể: %s" % (diag / "variants" / tag))
    print("Lưu ý: trạng thái [%s]. Chưa xác minh ONNX laptop cùng trọng số với engine TensorRT đã benchmark." % st)
    return 0 if not (fails or diag_fails) else 1


if __name__ == "__main__":
    sys.exit(main())
