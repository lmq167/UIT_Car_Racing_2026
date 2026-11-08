"""Kiểm tra hợp đồng, tiền xử lý và chọn ảnh tham chiếu."""
import argparse
import copy
import csv
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np

import make_engine_ref as ref


def metadata():
    return {
        "task": "segmentation", "class_names": ["background", "track_surface", "line"],
        "num_classes": 3, "source_image_hw": [7, 11], "crop_y": [2, 7], "crop_x": [1, 10],
        "input_hw": [3, 4], "input_shape": [1, 3, 3, 4], "output_shape": [1, 3, 3, 4],
        "input_name": "images", "output_name": "logits", "input_dtype": "float32",
        "color_order": "RGB", "layout": "NCHW", "output_type": "logits",
        "activation": "softmax_over_channels_argmax", "resize": "cv2.resize INTER_AREA (crop -> input)",
        "preprocessing_in_model": False, "fake_data": False, "dynamic": False,
        "normalization": {"scale": 1 / 255, "mean": [0, 0, 0], "std": [1, 1, 1]},
    }


def twin_metadata():
    info = metadata()
    info.update({"class_names": ["background", "road"], "num_classes": 2, "crop_y": [0, 7], "crop_x": [0, 11],
                 "output_shape": [1, 2, 3, 4], "resize": "cv2.resize INTER_LINEAR (full -> input)",
                 "normalization": {"scale": 1 / 255, "mean": [0.485, 0.456, 0.406],
                                   "std": [0.229, 0.224, 0.225]}})
    return info


class TwinLiteTests(unittest.TestCase):
    def test_twinlite_info_accepted_and_rejected(self):
        info = twin_metadata()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "info.json"
            path.write_text(json.dumps(info), encoding="utf-8")
            self.assertEqual(ref.load_info(path), info)
            for key, value in (("output_shape", [1, 3, 3, 4]), ("num_classes", 3), ("crop_y", [1, 7]),
                               ("resize", "cv2.resize INTER_AREA (crop -> input)"),
                               ("normalization", {"scale": 1 / 255, "mean": [0.485, 0.456, 0.406],
                                                  "std": [0.229, 0, 0.225]}),
                               ("normalization", {"scale": 1 / 255, "mean": [0.485, 0.456],
                                                  "std": [0.229, 0.224, 0.225]})):
                broken = copy.deepcopy(info)
                broken[key] = value
                path.write_text(json.dumps(broken), encoding="utf-8")
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    ref.load_info(path)

    def test_twinlite_preprocess_linear_full_imagenet(self):
        info = twin_metadata()
        image = np.arange(7 * 11 * 3, dtype=np.uint8).reshape(7, 11, 3)
        blob = ref.preprocess(image, info)
        rgb = cv2.resize(image, (4, 3), interpolation=cv2.INTER_LINEAR)[:, :, ::-1].astype(np.float32)
        mean = np.array([0.485, 0.456, 0.406], np.float32)
        std = np.array([0.229, 0.224, 0.225], np.float32)
        expected = np.moveaxis((rgb * np.float32(1 / 255) - mean) / std, -1, 0)[None]
        np.testing.assert_allclose(blob, expected, rtol=0, atol=1e-6)
        self.assertEqual(blob.dtype, np.dtype("<f4"))


class ReferenceTests(unittest.TestCase):
    def test_info_required_and_types(self):
        info = metadata()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "info.json"
            path.write_text(json.dumps(info), encoding="utf-8")
            self.assertEqual(ref.load_info(path), info)
            for key in info:
                broken = copy.deepcopy(info)
                del broken[key]
                path.write_text(json.dumps(broken), encoding="utf-8")
                with self.subTest(missing=key), self.assertRaises(ValueError):
                    ref.load_info(path)
            for key, value in (("fake_data", True), ("fake_data", 0), ("crop_x", [0, 12]),
                               ("crop_y", [7, 2]), ("input_hw", [True, 4]),
                               ("output_shape", [1, 1, 3, 4]), ("color_order", "BGR"),
                               ("normalization", {"scale": 1, "mean": [0]*3, "std": [1]*3})):
                broken = copy.deepcopy(info)
                broken[key] = value
                path.write_text(json.dumps(broken), encoding="utf-8")
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    ref.load_info(path)

    def test_uint8_resize_before_scale_and_rgb(self):
        info = metadata()
        image = np.arange(7 * 11 * 3, dtype=np.uint8).reshape(7, 11, 3)
        resize = cv2.resize
        with patch.object(ref.cv2, "resize", wraps=resize) as spy:
            blob = ref.preprocess(image, info)
        self.assertEqual(spy.call_args.args[0].dtype, np.uint8)
        expected = resize(image[2:7, 1:10], (4, 3), interpolation=cv2.INTER_AREA)
        expected = np.moveaxis(expected[:, :, ::-1].astype(np.float32) / 255, -1, 0)[None]
        np.testing.assert_allclose(blob, expected, rtol=0, atol=1e-7)
        self.assertEqual(blob.shape, (1, 3, 3, 4))
        self.assertTrue(blob.flags.c_contiguous)
        self.assertEqual(blob.dtype, np.dtype("<f4"))
        for wrong in (image[:6], image.astype(np.float32), image[:, :, 0], None):
            with self.assertRaises(ValueError):
                ref.preprocess(wrong, info)

    def test_coverage_and_pass_distribution(self):
        rows = [(f"car6_{p}_{i}.jpg", str(p), [tag] if i == 5 else [])
                for p, tag in enumerate(("ham", "cau", "giao_lo", "val"), 1) for i in range(30)]
        selected, missing = ref.select_rows(rows, 30)
        self.assertEqual(len(selected), 30)
        self.assertEqual(len({r[0] for r in selected}), 30)
        self.assertEqual({tag for r in selected for tag in r[2]}, {"ham", "cau", "giao_lo", "val"})
        self.assertEqual(missing, [])
        counts = [sum(r[1] == str(p) for r in selected) for p in range(1, 5)]
        self.assertLessEqual(max(counts) - min(counts), 1)
        selected, missing = ref.select_rows([(r[0], r[1], []) for r in rows], 30)
        self.assertEqual(missing, ["ham", "cau", "giao_lo", "val"])
        many_passes = [(f"car6_{p}.jpg", str(p), []) for p in range(100)]
        spread, _ = ref.select_rows(many_passes, 30)
        self.assertEqual(len({r[1] for r in spread}), 30)
        self.assertEqual((spread[0][1], spread[-1][1]), ("0", "99"))
        with self.assertRaises(ValueError):
            ref.select_rows(rows, 19)
        with self.assertRaises(ValueError):
            ref.select_rows(rows[:10], 30)

    def test_csv_tags_and_invalid_paths(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manifest, split = root / "manifest.csv", root / "split.csv"
            rows = []
            for index, t in enumerate((11, 10, 2, 3, 4, 5, 6)):
                name = f"car6_{index}.jpg"
                (root / name).touch()
                rows.append({"file": name, "car": "car6", "pass": str(index), "t": t})
            def write(path, data):
                with path.open("w", encoding="utf-8", newline="") as stream:
                    writer = csv.DictWriter(stream, fieldnames=list(data[0]))
                    writer.writeheader()
                    writer.writerows(data)
            write(manifest, rows)
            write(split, [{"file": r["file"], "pass": r["pass"], "split": "val"} for r in rows])
            result = ref.candidates(root, manifest, split)
            self.assertEqual([r[2] for r in result], [["ham", "val"], ["cau", "val"]]
                             + [["giao_lo", "val"]]*4 + [["val"]])
            rows[0]["file"] = "../bad.jpg"
            write(manifest, rows)
            with self.assertRaises(ValueError):
                ref.candidates(root, manifest, split)

    def test_npy_v1_little_endian_contiguous(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "ảnh.npy"
            values = np.arange(36, dtype=">f4").reshape(1, 3, 3, 4)[:, :, :, ::-1]
            ref.write_npy(path, values)
            self.assertEqual(path.read_bytes()[:8], b"\x93NUMPY\x01\x00")
            loaded = np.load(path, allow_pickle=False)
            self.assertTrue(loaded.flags.c_contiguous)
            self.assertEqual(loaded.dtype.str, "<f4")
            np.testing.assert_array_equal(loaded, values)

    def test_generate_synthetic_images_with_mock_session(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "ảnh thật"
            root.mkdir()
            info = metadata()
            (root / "info.json").write_text(json.dumps(info), encoding="utf-8")
            session = SimpleNamespace(
                get_inputs=lambda: [SimpleNamespace(name="images", shape=[1, 3, 3, 4], type="tensor(float)")],
                get_outputs=lambda: [SimpleNamespace(name="logits", shape=[1, 3, 3, 4], type="tensor(float)")],
                run=lambda names, inputs: [inputs["images"].copy()])
            manifest = "file,car,pass,t\n"
            split = "file,pass,split\n"
            for i in range(20):
                name = f"car6_{i}.png"
                encoded = cv2.imencode(".png", np.full((7, 11, 3), (3, 27, 251), np.uint8))[1]
                (root / name).write_bytes(encoded.tobytes())
                manifest += f"{name},car6,{i % 4},{(11, 10, 2, 6)[i % 4]}\n"
                split += f"{name},{i % 4},val\n"
            (root / "manifest.csv").write_text(manifest)
            (root / "split.csv").write_text(split)
            args = argparse.Namespace(onnx=root / "model.onnx", info=root / "info.json", images=root,
                                      manifest=root / "manifest.csv", split=root / "split.csv",
                                      output=root / "ref")
            with patch("onnxruntime.InferenceSession", return_value=session):
                ref.generate(args)
            self.assertEqual(len((args.output / "list.txt").read_text().splitlines()), 20)
            self.assertEqual(len(list(args.output.glob("*.npy"))), 40)
            np.testing.assert_array_equal(np.load(args.output / "000_input.npy"),
                                          np.load(args.output / "000_logits.npy"))
            with patch("onnxruntime.InferenceSession", return_value=session), self.assertRaises(FileExistsError):
                ref.generate(args)


if __name__ == "__main__":
    unittest.main()
