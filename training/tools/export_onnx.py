"""Xuất trọng số đường/YOLO thành ONNX cố định và kiểm tra trên ít nhất 20 ảnh."""

import argparse
import csv
from pathlib import Path
import shutil
import sys
import tempfile

from compare_outputs import dependency, image_paths, load_info, preprocess, read_image, read_json, write_json


def export(args):
    torch = dependency("torch")
    np = dependency("numpy")
    onnx = dependency("onnx")
    ort = dependency("onnxruntime")
    simplifier = dependency("onnxsim")
    profile = dependency("thop").profile
    images = image_paths(args.images)
    if len(images) < 20:
        raise ValueError("Cần ít nhất 20 ảnh riêng biệt để kiểm tra xuất ONNX")
    images = images[:args.check_images] if args.check_images else images
    source = read_json(args.info) if args.info else {}
    suffix = args.weights.suffix.lower()
    if suffix not in (".pt", ".pth"):
        raise ValueError("Trọng số phải có đuôi .pt hoặc .pth")
    if args.output.suffix.lower() != ".onnx":
        raise ValueError("Đầu ra phải có đuôi .onnx")
    info_path = args.output.with_name(args.output.stem + "_info.json")
    csv_path = args.output.with_name(args.output.stem + "_onnx_check.csv")
    if any(p.exists() for p in (args.output, info_path, csv_path)):
        raise ValueError("Đầu ra đã tồn tại; chọn tên mới để giữ kết quả trước")
    ultralytics_version = None
    if suffix == ".pth":
        from lane_models import build_model
        checkpoint = torch.load(str(args.weights), map_location="cpu", weights_only=True)
        wrapped = isinstance(checkpoint, dict) and "state_dict" in checkpoint
        metadata = checkpoint if wrapped else {}
        arch = args.arch or metadata.get("arch") or source.get("architecture")
        names = args.classes or metadata.get("class_names") or source.get("class_names")
        hw = args.size or metadata.get("input_hw") or source.get("input_hw")
        if not arch or not names or not hw:
            raise ValueError("Bảng trọng số rời cần --arch, --classes và --size hoặc --info")
        if metadata.get("arch", arch) != arch or list(metadata.get("class_names", names)) != list(names):
            raise ValueError("Kiến trúc/tên lớp yêu cầu khác thông tin trong trọng số")
        model = build_model(arch, len(names)).cpu().float().eval()
        model.load_state_dict(checkpoint["state_dict"] if wrapped else checkpoint, strict=True)
        task = "segmentation"
    else:
        ultra = dependency("ultralytics")
        detector = ultra.YOLO(str(args.weights))
        if detector.task != "detect":
            raise ValueError("Chỉ hỗ trợ trọng số YOLO phát hiện khung")
        model = detector.model.cpu().float().eval()
        class_map = detector.names
        names = [class_map[i] for i in range(len(class_map))]
        if args.classes and args.classes != names:
            raise ValueError("--classes khác thứ tự lớp trong trọng số YOLO")
        hw = args.size or source.get("input_hw") or [320, 320]
        arch = source.get("architecture", Path(str(detector.ckpt.get("train_args", {}).get("model", "yolo"))).stem)
        task = "detection"
        ultralytics_version = ultra.__version__
    h, w = map(int, hw)
    if min(h, w) <= 0 or (task == "detection" and (h % 32 or w % 32)):
        raise ValueError("Kích thước phải dương; YOLO yêu cầu bội số 32")
    if not names or any(not isinstance(n, str) or not n for n in names) or len(set(names)) != len(names):
        raise ValueError("Tên lớp phải khác nhau và không rỗng")
    if source:
        for key, value in (("task", task), ("architecture", arch), ("class_names", list(names)), ("input_hw", [h, w])):
            if key in source and source[key] != value:
                raise ValueError("--info khác lựa chọn xuất: " + key)
        # Không nhận thông tin cũ mô tả BGR/chuẩn hoá trong mạng.
        load_info(args.info)
    info = dict(source)
    info.update(task=task, architecture=arch, class_names=list(names), num_classes=len(names),
                input_name="images", input_shape=[1, 3, h, w], input_hw=[h, w], input_dtype="float32",
                color_order="RGB", layout="NCHW", normalization=dict(scale=1/255, mean=[0, 0, 0], std=[1, 1, 1]),
                preprocessing_in_model=False, opset=12, dynamic=False, latency_measured=False,
                fake_data=args.fake_data or source.get("fake_data", False),
                torch_version=str(torch.__version__), onnxruntime_version=ort.__version__,
                onnx_version=onnx.__version__, onnxsim_version=simplifier.__version__,
                export_tool_version="1.0", seed=source.get("seed"), project=source.get("project"), version=source.get("version"))
    if task == "segmentation":
        info.update(resize="stretch_bilinear", output_type="logits", activation="sigmoid_per_class",
                    threshold=args.threshold if args.threshold is not None else source.get("threshold", 0.5),
                    mask_pixel_values=source.get("mask_pixel_values"), scan_row_ratios=source.get("scan_row_ratios", [0.6, 0.7, 0.8]))
    else:
        info.update(resize="letterbox", letterbox=dict(auto=False, center=True, padding_value=114, scaleup=True, stride=32),
                    output_type="cxcywh_pixels_and_class_probabilities", box_format="cxcywh", box_units="input_pixels",
                    scores_are_probabilities=True, objectness=False, nms_in_model=False, nms_class_aware=True,
                    conf_threshold=args.conf if args.conf is not None else source.get("conf_threshold", 0.25),
                    nms_iou=args.nms_iou if args.nms_iou is not None else source.get("nms_iou", 0.7),
                    ultralytics_version=ultralytics_version)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Chỉ công bố bộ kết quả sau khi xuất và đối chiếu đều đạt.
    with tempfile.TemporaryDirectory(prefix="onnx_", dir=args.output.parent) as work:
        work = Path(work)
        target = work / "model.onnx"
        x = torch.zeros(1, 3, h, w)
        if task == "segmentation":
            torch.onnx.export(model, x, str(target), opset_version=12, input_names=["images"],
                              output_names=["logits"], do_constant_folding=True, dynamo=False)
            macs, _ = profile(model, inputs=(x,), verbose=False)
            gflops = float(2 * macs / 1e9)
        else:
            # Bản sao trọng số tạm tránh để Ultralytics ghi đè cạnh trọng số gốc.
            copied = work / "weights.pt"
            shutil.copy2(args.weights, copied)
            temp_detector = ultra.YOLO(str(copied))
            exported = temp_detector.export(format="onnx", imgsz=(h, w), batch=1, opset=12,
                                             simplify=False, dynamic=False, half=False, nms=False, device="cpu")
            shutil.copy2(exported, target)
            from ultralytics.utils.torch_utils import get_flops
            gflops = float(get_flops(model, imgsz=[h, w]))
        graph, valid = simplifier.simplify(onnx.load(str(target)))
        if not valid:
            raise ValueError("Rút gọn ONNX không vượt kiểm tra")
        onnx.checker.check_model(graph)
        if next(v.version for v in graph.opset_import if v.domain in ("", "ai.onnx")) != 12:
            raise ValueError("ONNX sai opset 12")
        for value in list(graph.graph.input) + list(graph.graph.output):
            if any(d.dim_param or d.dim_value <= 0 for d in value.type.tensor_type.shape.dim):
                raise ValueError("ONNX còn chiều động")
        onnx.save(graph, str(target))
        session = ort.InferenceSession(str(target), providers=["CPUExecutionProvider"])
        if len(session.get_inputs()) != 1 or len(session.get_outputs()) != 1 or session.get_inputs()[0].shape != [1, 3, h, w]:
            raise ValueError("ONNX sai số đầu vào/ra hoặc kích thước")
        info.update(input_name=session.get_inputs()[0].name, output_name=session.get_outputs()[0].name,
                    output_shape=session.get_outputs()[0].shape,
                    parameters=int(sum(p.numel() for p in model.parameters())), gflops=gflops,
                    flops_convention="2 FLOPs per MAC", nano_estimated_ms=6*gflops)
        draft = work / "info.json"
        write_json(draft, info)
        load_info(draft)
        checks = []
        for image in images:
            array, _ = preprocess(read_image(image), info)
            with torch.no_grad():
                a = model(torch.from_numpy(array))
                a = a[0] if isinstance(a, (tuple, list)) else a
                a = a.detach().cpu().numpy()
            b = session.run([info["output_name"]], {info["input_name"]: array})[0]
            if a.shape != b.shape or not np.isfinite(a).all() or not np.isfinite(b).all():
                raise ValueError("Đầu ra khác kích thước hoặc không hữu hạn")
            row = dict(image=image.relative_to(args.images).as_posix())
            if task == "segmentation":
                sigmoid = lambda z: 1 / (1 + np.exp(-np.clip(z, -80, 80)))
                error = float(np.max(np.abs(sigmoid(a) - sigmoid(b))))
                row["max_probability_error"] = error
                good = error < 1e-3
            else:
                row["max_score_error"] = float(np.max(np.abs(a[:, 4:] - b[:, 4:])))
                row["max_box_error_px"] = float(np.max(np.abs(a[:, :4] - b[:, :4])))
                good = row["max_score_error"] < 1e-3 and row["max_box_error_px"] < 1e-2
            if not good:
                raise ValueError("ONNX lệch PyTorch: " + str(row))
            checks.append(row)
        info["onnx_check_images"] = len(checks)
        for key in checks[0]:
            if key != "image":
                info["onnx_" + key] = max(row[key] for row in checks)
        write_json(draft, info)
        with (work / "check.csv").open("w", encoding="utf-8-sig", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(checks[0]))
            writer.writeheader()
            writer.writerows(checks)
        shutil.copy2(target, args.output)
        shutil.copy2(draft, info_path)
        shutil.copy2(work / "check.csv", csv_path)
    print("Đã xuất và đối chiếu {} ảnh: {}".format(len(checks), args.output))


def main():
    # Đầu ra chuyển hướng trên Windows cũng phải in được đầy đủ dấu tiếng Việt.
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("weights", type=Path, help="Trọng số .pt hoặc .pth tin cậy")
    ap.add_argument("--output", required=True, type=Path, help="File ONNX mới")
    ap.add_argument("--images", required=True, type=Path, help="Ít nhất 20 ảnh kiểm tra")
    ap.add_argument("--info", type=Path, help="Thông tin mô hình từ notebook để giữ nguồn dữ liệu và ngưỡng")
    ap.add_argument("--arch", choices=["nanounet", "unet_b_minus"], help="Bắt buộc nếu bảng trọng số không chứa kiến trúc")
    ap.add_argument("--classes", nargs="+", help="Tên lớp đúng thứ tự kênh đầu ra")
    ap.add_argument("--size", nargs=2, type=int, metavar=("H", "W"), help="Kích thước cố định cao rộng")
    ap.add_argument("--threshold", type=float, help="Ngưỡng sigmoid phân vùng")
    ap.add_argument("--conf", type=float, help="Ngưỡng phát hiện")
    ap.add_argument("--nms-iou", type=float, help="Ngưỡng loại khung trùng theo lớp")
    ap.add_argument("--check-images", type=int, default=0, help="0: mọi ảnh; giá trị khác phải từ 20")
    ap.add_argument("--fake-data", action="store_true", help="Đánh dấu trọng số/ảnh thử giả")
    args = ap.parse_args()
    try:
        if args.check_images != 0 and args.check_images < 20:
            raise ValueError("--check-images phải bằng 0 hoặc ít nhất 20")
        export(args)
        return 0
    except Exception as exc:
        print("Lỗi xuất ONNX ({}): {}".format(type(exc).__name__, exc), file=sys.stderr)
        print("Thư viện trên Ubuntu/Colab: python -m pip install torch ultralytics==8.4.161 numpy opencv-python onnx onnxruntime onnxsim thop", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
