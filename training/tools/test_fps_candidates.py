"""Kiểm thử cấu hình, thông tin thử tốc độ và ranh giới với bộ xuất dùng chung."""

from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import fps_candidates as fps
import export_onnx

ROOT = Path(__file__).resolve().parents[1]
NOTEBOOK = ROOT / "notebooks" / "export_candidates.ipynb"


class CandidateTests(unittest.TestCase):
    def test_notebook_syntax_and_six_matching_commands(self):
        nb = fps.read_json(NOTEBOOK)
        code = ["".join(c["source"]) for c in nb["cells"] if c["cell_type"] == "code"]
        for source in code:
            compile(source, str(NOTEBOOK), "exec")
        config = {}
        exec(code[0], config)
        self.assertIs(config["FAKE_DATA"], True)
        specs = fps.candidate_specs(fps.road_input_hw(ROOT / "notebooks/road_train_car.ipynb"),
                                    config["YOLO_CANDIDATES"], config["ROAD_CANDIDATES"])
        self.assertEqual(len(specs), 6)
        doc = (ROOT / "docs/model_fps.md").read_text(encoding="utf-8")
        for spec in specs:
            self.assertIn("--onnx=" + spec["name"] + ".onnx", doc)
            self.assertIn("--loadEngine=" + spec["name"] + "_fp16.engine", doc)

    def test_size_is_read_without_executing_training(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "road.ipynb"
            def write(source):
                fps.write_json(path, {"cells": [{"cell_type": "code", "source": source}]})
            write("INPUT_HW = (128, 224)\nraise RuntimeError('không chạy')")
            self.assertEqual(fps.road_input_hw(path), [128, 224])
            for source in ("X=1", "INPUT_HW=(0,160)", "INPUT_HW=(True,160)",
                           "INPUT_HW=(96,160)\nINPUT_HW=(128,224)", "INPUT_HW=get_size()"):
                write(source)
                with self.subTest(source=source), self.assertRaises((ValueError, TypeError)):
                    fps.road_input_hw(path)

    def test_duplicate_names_rejected(self):
        with self.assertRaises(ValueError):
            fps.candidate_specs([96, 160], [("yolov8n", 320)] * 2, [])

    def test_speed_metadata_never_claims_recognition_or_latency(self):
        original = dict(gflops=1.25, input_shape=[1, 3, 96, 160],
                        nano_estimated_ms=7.5, threshold=0.5, project="không áp dụng")
        for random in (True, False):
            info = fps.speed_info(original, "yolov8n", random, 42)
            self.assertIs(info["random_weights"], random)
            self.assertEqual(info["weights_source"], "random_initialization" if random else "pretrained_coco")
            self.assertFalse(info["recognition_validated"])
            self.assertFalse(info["latency_measured"])
            self.assertTrue(info["fake_data"])
            for key in ("nano_estimated_ms", "threshold", "conf_threshold", "nms_iou", "project", "version"):
                self.assertIsNone(info[key])
            self.assertEqual(info["input_shape"], original["input_shape"])
        self.assertEqual(original["nano_estimated_ms"], 7.5)
        for value in (0, -1, float("nan"), float("inf"), None):
            self.assertIsNone(fps.speed_info({"gflops": value}, "nanounet", True, 42)["gflops"])

    def test_reuses_exporter_and_preserves_failed_or_existing_outputs(self):
        with tempfile.TemporaryDirectory(prefix="ứng viên ") as folder:
            output = Path(folder) / "yolov8n_320.onnx"
            torch = Mock()
            ultra = Mock()
            def export(args):
                self.assertEqual(args.check_images, 20)
                self.assertTrue(args.fake_data)
                self.assertEqual(args.size, [320, 320])
                self.assertIsNone(args.info)
                args.output.write_bytes("onnx thử".encode("utf-8"))
                fps.write_json(args.output.with_name("candidate_info.json"), {"gflops": 2.0})
                args.output.with_name("candidate_onnx_check.csv").write_text("ảnh,sai số\n", encoding="utf-8")
            with patch.object(fps, "dependency", side_effect=lambda name: {"torch": torch, "ultralytics": ultra}[name]), \
                    patch.object(export_onnx, "export", side_effect=export) as shared:
                fps.export_candidate("yolov8n", [320, 320], output, folder, True, 42)
                ultra.YOLO.assert_called_once_with("yolov8n.yaml")
                shared.assert_called_once()
                self.assertEqual(output.read_bytes(), "onnx thử".encode("utf-8"))
                self.assertEqual(fps.read_json(output.with_name("yolov8n_320_info.json"))["weights_purpose"], "speed_test_only")
                with self.assertRaises(ValueError):
                    fps.export_candidate("yolov8n", [320, 320], output, folder, True, 42)
            failed = output.with_name("failed.onnx")
            with patch.object(fps, "dependency", return_value=Mock()), \
                    patch.object(export_onnx, "export", side_effect=RuntimeError("đối chiếu sai")):
                with self.assertRaises(RuntimeError):
                    fps.export_candidate("yolo11n", [416, 416], failed, folder, True, 42)
                self.assertFalse(failed.exists())
                self.assertFalse(failed.with_name("failed_info.json").exists())

    def test_weight_sources_for_road_and_pretrained_yolo(self):
        with tempfile.TemporaryDirectory() as folder:
            torch, ultra, build = Mock(), Mock(), Mock()
            def export(args):
                if args.weights.suffix == ".pth":
                    checkpoint = torch.save.call_args.args[0]
                    self.assertEqual(checkpoint["arch"], "unet_b_minus")
                    self.assertEqual(checkpoint["class_names"], ["speed_test_channel_0"])
                    self.assertEqual(checkpoint["input_hw"], [128, 224])
                args.output.write_bytes(b"onnx")
                fps.write_json(args.output.with_name("candidate_info.json"), {"gflops": 1.0})
                args.output.with_name("candidate_onnx_check.csv").write_text("ok", encoding="utf-8")
            with patch.dict(sys.modules, lane_models=SimpleNamespace(build_model=build)), \
                    patch.object(fps, "dependency", side_effect=lambda name: {"torch": torch, "ultralytics": ultra}[name]), \
                    patch.object(export_onnx, "export", side_effect=export):
                for arch, hw in (("unet_b_minus", [128, 224]), ("yolo11n", [416, 416])):
                    output = Path(folder) / (arch + ".onnx")
                    fps.export_candidate(arch, hw, output, folder, False, 42)
                    info = fps.read_json(output.with_name(arch + "_info.json"))
                    self.assertEqual(info["random_weights"], arch == "unet_b_minus")
                build.assert_called_once_with("unet_b_minus", 1)
                self.assertTrue(ultra.YOLO.call_args.args[0].endswith("yolo11n.pt"))

    def test_notebook_export_loop_is_bounded_and_stops_on_failure(self):
        nb = fps.read_json(NOTEBOOK)
        source = "".join(nb["cells"][7]["source"])
        import subprocess
        with tempfile.TemporaryDirectory() as folder:
            context = dict(CANDIDATES=[dict(name="yolov8n_320", architecture="yolov8n", input_hw=[320, 320])],
                           RUN=Path(folder), TOOLS=ROOT / "tools", sys=sys, subprocess=subprocess,
                           IMAGES=Path(folder), SEED=42, FAKE_DATA=True, EXPORT_TIMEOUT_SECONDS=900)
            for error in (subprocess.TimeoutExpired("export", 900), subprocess.CalledProcessError(2, "export")):
                with self.subTest(error=type(error).__name__), \
                        patch.object(subprocess, "run", side_effect=error) as run:
                    with self.assertRaisesRegex(RuntimeError, "_export.log"):
                        exec(source, context)
                    self.assertEqual(run.call_args.kwargs["timeout"], 900)
                    self.assertTrue(run.call_args.kwargs["check"])
                    self.assertIn("--fake-data", run.call_args.args[0])
                (Path(folder) / "yolov8n_320_export.log").unlink()


if __name__ == "__main__":
    unittest.main()
