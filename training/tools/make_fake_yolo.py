"""Tạo mạng phát hiện GIẢ để kiểm đường suy luận, không đánh giá nhận biết."""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper


CLASS_NAMES = ["red", "green", "off", "left", "right", "no_left", "no_right",
               "round_barrier", "work_barrier"]


def make_fake_yolo(out_dir, input_hw=(320, 320), seed=0):
    """Ghi yolo.onnx + yolo_info.json vào thư mục mới, từ chối ghi đè."""
    height, width = input_hw
    if height < 32 or width < 32:
        raise ValueError("input_hw mỗi chiều phải ≥ 32 (một bước stride YOLO)")
    rng = np.random.default_rng(seed)
    # Tích chập 1×1 có bước 32 cho N nhỏ, vẫn phụ thuộc ảnh đầu vào.
    count = ((height + 31) // 32) * ((width + 31) // 32)
    weights = rng.normal(0, 0.1, (13, 3, 1, 1)).astype(np.float32)
    bias = rng.normal(0, 0.1, 13).astype(np.float32)
    scale = np.array([width, height, width, height] + [1] * 9,
                     dtype=np.float32).reshape(1, 13, 1)
    initializers = [numpy_helper.from_array(value, name) for name, value in (
        ("weights", weights), ("bias", bias),
        ("shape", np.array([1, 13, count], dtype=np.int64)), ("pixel_scale", scale))]
    nodes = [
        helper.make_node("Conv", ["images", "weights", "bias"], ["features"],
                         kernel_shape=[1, 1], strides=[32, 32]),
        helper.make_node("Reshape", ["features", "shape"], ["flat"]),
        helper.make_node("Sigmoid", ["flat"], ["probabilities"]),
        helper.make_node("Mul", ["probabilities", "pixel_scale"], ["output0"]),
    ]
    graph = helper.make_graph(
        nodes, "fake_yolo_random",
        [helper.make_tensor_value_info("images", TensorProto.FLOAT, [1, 3, height, width])],
        [helper.make_tensor_value_info("output0", TensorProto.FLOAT, [1, 13, count])],
        initializers)
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 12)],
                              producer_name="make_fake_yolo", ir_version=7)
    onnx.checker.check_model(model)
    ratio = min(height / 720, width / 1280)
    nw, nh = round(1280 * ratio), round(720 * ratio)
    if nw == 0 or nh == 0:
        raise ValueError("input_hw làm mất một chiều ảnh sau letterbox")
    info = {
        "task": "detection", "architecture": "fake_conv_random",
        "class_names": CLASS_NAMES, "num_classes": 9,
        "source_image_hw": [720, 1280],
        "input_name": "images", "input_shape": [1, 3, height, width],
        "input_hw": [height, width], "input_dtype": "float32",
        "color_order": "RGB", "layout": "NCHW",
        "normalization": {"scale": 1 / 255, "mean": [0, 0, 0], "std": [1, 1, 1]},
        "preprocessing_in_model": False, "resize": "letterbox",
        "letterbox": {"auto": False, "center": True, "padding_value": 114,
                      "scaleup": True, "stride": 32},
        "letterbox_1280x720": {"scale": ratio, "resized_wh": [nw, nh],
                              "pad_left": round((width - nw) / 2 - 0.1),
                              "pad_top": round((height - nh) / 2 - 0.1)},
        "output_name": "output0", "output_shape": [1, 13, count],
        "output_type": "cxcywh_pixels_and_class_probabilities",
        "box_format": "cxcywh", "box_units": "input_pixels",
        "scores_are_probabilities": True, "objectness": False,
        "nms_in_model": False, "nms_class_aware": True,
        "conf_threshold": 0.25, "nms_iou": 0.7, "opset": 12, "dynamic": False,
        "seed": seed, "fake_data": True, "recognition_validated": False,
    }
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=False)
    onnx.save_model(model, str(out_dir / "yolo.onnx"))
    (out_dir / "yolo_info.json").write_text(
        json.dumps(info, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return out_dir / "yolo.onnx", out_dir / "yolo_info.json"


def main():
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--input-hw", type=int, nargs=2, default=[320, 320], metavar=("H", "W"))
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    model, info = make_fake_yolo(args.out_dir, args.input_hw, args.seed)
    print("Mô hình GIẢ (chưa huấn luyện):", model, info)


if __name__ == "__main__":
    main()
