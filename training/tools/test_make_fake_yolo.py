"""Kiểm ONNX giả bằng checker và suy luận CPU thật."""
import json
import sys
from pathlib import Path
import tempfile
import unittest

import numpy as np
import cv2
import onnx
import onnxruntime as ort

from make_fake_yolo import CLASS_NAMES, make_fake_yolo

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")


class LetterboxReferenceTest(unittest.TestCase):
    def test_reference_pixels_and_rounding(self):
        # Hàm tham chiếu độc lập: tính trên uint8 trước RGB/float32/NCHW.
        y, x = np.indices((720, 1280))
        image = np.stack([(3*x + 5*y) % 251, (7*x + 11*y) % 253,
                          (13*x + 17*y) % 255], axis=-1).astype(np.uint8)
        for h, w, expected in [(320, 320, (320, 180, 0, 70, 0, 70)),
                               (320, 321, (321, 181, 0, 69, 0, 70)),
                               (320, 328, (328, 184, 0, 68, 0, 68)),
                               (320, 344, (344, 194, 0, 63, 0, 63)),
                               (180, 321, (320, 180, 0, 0, 1, 0))]:
            r = min(h / 720, w / 1280)
            nw, nh = round(1280*r), round(720*r)
            dx, dy = (w-nw)/2, (h-nh)/2
            left, right = round(dx-0.1), round(dx+0.1)
            top, bottom = round(dy-0.1), round(dy+0.1)
            self.assertEqual((nw, nh, left, top, right, bottom), expected)
            resized = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_AREA)
            padded = cv2.copyMakeBorder(resized, top, bottom, left, right,
                                       cv2.BORDER_CONSTANT, value=(114, 114, 114))
            blob = np.ascontiguousarray(padded[:, :, ::-1].transpose(2, 0, 1)[None],
                                        dtype=np.float32) / 255
            self.assertEqual(blob.shape, (1, 3, h, w))
            if (h, w) == (320, 320):
                self.assertEqual(r, 0.25)
                for py, px, rgb in [(0, 0, (114, 114, 114)), (69, 319, (114, 114, 114)),
                                    (70, 0, (45, 27, 12)), (70, 1, (97, 55, 24)),
                                    (100, 17, (164, 52, 63)), (160, 160, (205, 118, 218)),
                                    (249, 319, (149, 137, 141)),
                                    (319, 319, (114, 114, 114))]:
                    np.testing.assert_allclose(blob[0, :, py, px], np.array(rgb)/255,
                                               rtol=0, atol=1e-6)
                print("Tham chiếu cv2: scale=0.25, resized_wh=(320,180), pad=(0,70,0,70)")


class FakeYoloTest(unittest.TestCase):
    def test_checker_and_runtime(self):
        with tempfile.TemporaryDirectory() as temporary:
            for height, width in [(320, 320), (193, 321)]:
                folder = Path(temporary) / str(height)
                model_path, info_path = make_fake_yolo(folder, (height, width), seed=47)
                model = onnx.load(str(model_path))
                onnx.checker.check_model(model)
                self.assertEqual(model.opset_import[0].version, 12)
                self.assertEqual(model.ir_version, 7)
                info = json.loads(info_path.read_text(encoding="utf-8"))
                self.assertEqual(info["class_names"], CLASS_NAMES)
                self.assertTrue(info["fake_data"])
                session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
                self.assertEqual(session.get_inputs()[0].name, info["input_name"])
                self.assertEqual(session.get_outputs()[0].name, info["output_name"])
                self.assertEqual(session.get_inputs()[0].shape, info["input_shape"])
                image = np.zeros(info["input_shape"], dtype=np.float32)
                output = session.run([info["output_name"]], {info["input_name"]: image})[0]
                count = ((height + 31) // 32) * ((width + 31) // 32)
                self.assertEqual(list(output.shape), [1, 13, count])
                self.assertEqual(list(output.shape), info["output_shape"])
                self.assertEqual(output.dtype, np.float32)
                self.assertTrue(np.isfinite(output).all())
                self.assertTrue(((output[:, 4:] >= 0) & (output[:, 4:] <= 1)).all())
                self.assertTrue((output[:, [0, 2]] <= width).all())
                self.assertTrue((output[:, [1, 3]] <= height).all())
                self.assertTrue((output[:, :4] > 0).all())
                changed = session.run(None, {info["input_name"]: np.ones_like(image)})[0]
                self.assertFalse(np.array_equal(output, changed))
                other, _ = make_fake_yolo(Path(temporary) / (str(height) + "_same"),
                                         (height, width), seed=47)
                self.assertEqual(model_path.read_bytes(), other.read_bytes())
                with self.assertRaises(FileExistsError):
                    make_fake_yolo(folder)

    def test_bad_dimensions(self):
        with tempfile.TemporaryDirectory() as temporary:
            for shape in [(0, 320), (320, -1), (320, 1)]:
                with self.assertRaises(ValueError):
                    make_fake_yolo(Path(temporary) / "bad", shape)


if __name__ == "__main__":
    unittest.main()
