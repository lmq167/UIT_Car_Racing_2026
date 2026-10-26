"""Chuẩn bị trọng số thử tốc độ, dùng lại bộ xuất ONNX và đối chiếu hiện có."""

import argparse
import ast
import math
from pathlib import Path
import shutil
import sys
import tempfile
from types import SimpleNamespace

from compare_outputs import dependency, read_json, write_json


def road_input_hw(notebook):
    """Đọc giá trị cấu hình, không thực thi mã của notebook huấn luyện."""
    values = []
    for cell in read_json(notebook)["cells"]:
        if cell["cell_type"] != "code":
            continue
        for node in ast.parse("".join(cell["source"])).body:
            if isinstance(node, ast.Assign) and any(
                    isinstance(t, ast.Name) and t.id == "INPUT_HW" for t in node.targets):
                values.append(ast.literal_eval(node.value))
    if len(values) != 1 or not isinstance(values[0], (list, tuple)) or len(values[0]) != 2:
        raise ValueError("Cần đúng một cấu hình INPUT_HW gồm cao, rộng")
    if any(type(v) is not int or v <= 0 for v in values[0]):
        raise ValueError("INPUT_HW phải gồm hai số nguyên dương")
    return list(values[0])


def candidate_specs(road_hw, yolo_candidates, road_candidates):
    specs = [dict(architecture=arch, input_hw=[size, size], name="{}_{}".format(arch, size))
             for arch, size in yolo_candidates]
    specs += [dict(architecture=arch, input_hw=list(road_hw),
                   name="{}_{}x{}".format(arch, *road_hw)) for arch in road_candidates]
    if len({s["name"] for s in specs}) != len(specs):
        raise ValueError("Danh sách ứng viên có tên trùng")
    return specs


def speed_info(info, architecture, random_weights, seed):
    """Giữ hợp đồng tensor, bỏ thông tin nhận dạng không áp dụng khi đo tốc độ."""
    info = dict(info)
    info.update(architecture=architecture, weights_purpose="speed_test_only",
                weights_source="random_initialization" if random_weights else "pretrained_coco",
                random_weights=random_weights, recognition_validated=False,
                validation_images_kind="synthetic", fake_data=True, seed=seed,
                latency_measured=False)
    for key in ("project", "version", "nano_estimated_ms", "threshold", "conf_threshold",
                "nms_iou", "scan_row_ratios", "mask_pixel_values"):
        info[key] = None
    if not isinstance(info.get("gflops"), (int, float)) or not math.isfinite(info["gflops"]) or info["gflops"] <= 0:
        info["gflops"] = None
    return info


def export_candidate(architecture, hw, output, images, fake_data, seed):
    """Một tiến trình cho một ứng viên; bên gọi giới hạn thời gian toàn tiến trình."""
    from export_onnx import export

    if architecture not in ("yolov8n", "yolo11n", "nanounet", "unet_b_minus"):
        raise ValueError("Kiến trúc không nằm trong danh sách thử tốc độ")
    if len(hw) != 2 or any(type(v) is not int or v <= 0 for v in hw):
        raise ValueError("Kích thước phải gồm hai số nguyên dương")
    output = Path(output)
    suffixes = (".onnx", "_info.json", "_onnx_check.csv")
    targets = [output.with_name(output.stem + suffix) for suffix in suffixes]
    if output.suffix != ".onnx" or any(p.exists() for p in targets):
        raise ValueError("Cần tên ONNX mới; không ghi đè kết quả đã có")
    torch = dependency("torch")
    torch.manual_seed(seed)
    torch.set_num_threads(2)
    is_road = architecture in ("nanounet", "unet_b_minus")
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="candidate_", dir=output.parent) as folder:
        work = Path(folder)
        if is_road:
            from lane_models import build_model
            weights = work / "weights.pth"
            torch.save(dict(state_dict=build_model(architecture, 1).state_dict(),
                            arch=architecture, input_hw=list(hw),
                            class_names=["speed_test_channel_0"]), str(weights))
        else:
            # YAML nằm trong gói Ultralytics: chế độ giả không tải trọng số COCO.
            detector = dependency("ultralytics").YOLO(
                architecture + ".yaml" if fake_data else str(work / (architecture + ".pt")))
            weights = work / "weights.pt"
            detector.save(str(weights))
        draft = work / "candidate.onnx"
        export(SimpleNamespace(weights=weights, output=draft, images=Path(images),
                               info=None, arch=architecture if is_road else None,
                               classes=None, size=list(hw), threshold=None, conf=None,
                               nms_iou=None, check_images=20, fake_data=True))
        info_path = work / "candidate_info.json"
        info = speed_info(read_json(info_path), architecture, is_road or fake_data, seed)
        write_json(info_path, info)
        for suffix, target in zip(suffixes, targets):
            shutil.copy2(work / ("candidate" + suffix), target)


def main():
    # Đầu ra chuyển hướng trên Windows cũng phải in được đầy đủ dấu tiếng Việt.
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("architecture")
    parser.add_argument("--size", nargs=2, type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--images", type=Path, required=True)
    parser.add_argument("--fake-data", action="store_true")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    export_candidate(args.architecture, args.size, args.output, args.images, args.fake_data, args.seed)


if __name__ == "__main__":
    main()
