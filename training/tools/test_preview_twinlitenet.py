"""Kiểm thử công cụ preview TwinLiteNetPlus: lọc tên lượt thu, tiền xử lý, cấu trúc thư mục đầu ra.

Chạy:  python -m unittest round3_final/tools/test_preview_twinlitenet.py -v   (hoặc python test_preview_twinlitenet.py)
Ca cần model/ảnh tham chiếu thật sẽ tự bỏ qua nếu thiếu tệp.
"""
import contextlib
import hashlib
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import preview_twinlitenet as P

REF = Path(r"data\round3\ref_road_onnx")


def touch(folder, name):
    img = np.random.default_rng(0).integers(0, 255, (720, 1280, 3), dtype=np.uint8)
    cv2.imwrite(str(Path(folder) / name), img)


class FilterTest(unittest.TestCase):
    def test_only_exact_pass(self):
        with tempfile.TemporaryDirectory() as d:
            names = ["car6_p0106_0001.jpg", "car6_p0106_0100.PNG", "car6_p0105_0001.jpg", "car6_p0107_0001.jpg",
                     "car6_p1060_0001.jpg", "car6_p0106_0001_x.jpg", "p0106_0001.jpg", "car6_p0106_0002.txt",
                     "xcar6_p0106_0003.jpg"]
            for n in names:
                (Path(d) / n).write_bytes(b"x")
            _, pat = P.pass_prefix("106")
            ok, conflicts = P.find_images(d, pat)
            self.assertEqual(ok, ["car6_p0106_0001.jpg", "car6_p0106_0100.PNG"])
            self.assertEqual(conflicts, {})

    def test_conflict_same_stem(self):
        with tempfile.TemporaryDirectory() as d:
            for n in ("car6_p0106_0001.jpg", "car6_p0106_0001.png", "car6_p0106_0002.jpg"):
                (Path(d) / n).write_bytes(b"x")
            ok, conflicts = P.find_images(d, P.pass_prefix("p0106")[1])
            self.assertEqual(ok, ["car6_p0106_0002.jpg"])
            self.assertEqual(list(conflicts), ["car6_p0106_0001"])

    def test_bad_input(self):
        for bad in ("", "abc", "0", "12345", "10 6", "-1"):
            with self.assertRaises(ValueError):
                P.pass_prefix(bad)
        self.assertEqual(P.pass_prefix("7")[0], "p0007")


class BenchmarkPipelineTest(unittest.TestCase):
    """Pipeline mặc định theo benchmark: resize toàn ảnh INTER_LINEAR, RGB, /255, ImageNet, NCHW; mask output[1] > output[0]."""

    def test_defaults(self):
        self.assertEqual(P.cfg_id(P.DEFAULT_CFG), "full-rgb-imagenet")
        self.assertEqual(P.DEFAULT_RESIZE, "linear")
        self.assertEqual(P.status(P.DEFAULT_CFG, "linear", 1), P.BENCH)
        self.assertEqual(P.status(P.Cfg("y320", "rgb", "div255"), "area", 1), P.UNVERIFIED)
        self.assertEqual(P.status(P.DEFAULT_CFG, "area", 1), P.UNVERIFIED)
        self.assertEqual(P.status(P.DEFAULT_CFG, "linear", 0), P.UNVERIFIED)
        self.assertEqual(P.GREEN, (0, 220, 0))
        self.assertEqual(P.ALPHA, 0.5)

    def test_preprocess_matches_manual(self):
        img = np.random.default_rng(1).integers(0, 255, (720, 1280, 3), dtype=np.uint8)
        small, x = P.preprocess(img, P.DEFAULT_CFG, (320, 192))
        ref_small = cv2.resize(img, (320, 192), interpolation=cv2.INTER_LINEAR)
        self.assertTrue(np.array_equal(small, ref_small))
        rgb = cv2.cvtColor(ref_small, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        mean = np.array([0.485, 0.456, 0.406], np.float32)
        std = np.array([0.229, 0.224, 0.225], np.float32)
        ref = ((rgb - mean) / std).transpose(2, 0, 1)[None]
        self.assertEqual(x.shape, (1, 3, 192, 320))
        self.assertEqual(x.dtype, np.float32)
        self.assertLess(float(np.abs(x - ref).max()), 1e-5)

    def test_road_mask_rule(self):
        rng = np.random.default_rng(2)
        lg = rng.normal(size=(2, 192, 320)).astype(np.float32)
        self.assertTrue(np.array_equal(P.road_mask(lg, 1), lg[1] > lg[0]))
        lg[1] = lg[0]  # hòa -> không phải đường (strict >)
        self.assertFalse(P.road_mask(lg, 1).any())
        lg = np.stack([np.full((4, 4), -5, np.float32), np.full((4, 4), -1, np.float32)])  # cả hai âm: vẫn so sánh trực tiếp
        self.assertTrue(P.road_mask(lg, 1).all())

    def test_overlay_blend(self):
        img = np.random.default_rng(3).integers(0, 255, (192, 320, 3), dtype=np.uint8)
        m = np.zeros((192, 320), bool)
        m[50:100, 60:200] = True
        ov = P.tint(img, m, P.GREEN)
        self.assertTrue(np.array_equal(ov[~m], img[~m]))
        exp = 0.5 * img[m].astype(np.float32) + 0.5 * np.array(P.GREEN, np.float32)
        self.assertLessEqual(float(np.abs(ov[m].astype(np.float32) - exp).max()), 1.0)


@unittest.skipUnless(REF.is_dir(), "thiếu data/round3/ref_road_onnx")
class PreprocessTest(unittest.TestCase):
    def test_matches_unet_reference_tensors(self):
        """Cùng hàm tiền xử lý với make_engine_ref.py (cấu hình U-Net của xe): phải khớp từng bit."""
        names = [l.split("\t")[0] for l in (REF / "list.txt").read_text().splitlines()[:5]]
        for i, n in enumerate(names):
            _, x = P.preprocess(cv2.imread(str(REF / n)), P.Cfg("y320", "rgb", "div255"), (160, 96), "area")
            self.assertTrue(np.array_equal(x, np.load(REF / ("%03d_input.npy" % i))), n)


class WritePngTest(unittest.TestCase):
    def test_overwrite_existing_and_no_temp_left(self):
        with tempfile.TemporaryDirectory() as d:
            f = Path(d) / "a" / "x.png"
            f.parent.mkdir()
            f.write_bytes(b"cu")
            P.write_png(f, np.full((4, 4, 3), 7, np.uint8))
            self.assertTrue(f.read_bytes().startswith(b"\x89PNG"))
            self.assertEqual([p.name for p in f.parent.iterdir()], ["x.png"])

    def test_failure_keeps_old_file_and_reports_cause(self):
        """Hồi quy lỗi [Errno 22] khi ghi đè ảnh có sẵn: không xóa tệp cũ, không để tệp tạm, báo rõ nguyên nhân."""
        with tempfile.TemporaryDirectory() as d:
            f = Path(d) / "x.png"
            f.write_bytes(b"CU")
            with mock.patch.object(P.os, "replace", side_effect=OSError(22, "Invalid argument")):
                with self.assertRaises(OSError) as cm:
                    P.write_png(f, np.zeros((4, 4, 3), np.uint8))
            msg = str(cm.exception)
            self.assertIn("errno=22", msg)
            self.assertIn(str(f), msg)
            self.assertIn("ĐÃ TỒN TẠI", msg)
            self.assertEqual(f.read_bytes(), b"CU")
            self.assertEqual([p.name for p in Path(d).iterdir()], ["x.png"])


@unittest.skipUnless(P.MODEL.is_file(), "thiếu model ONNX")
class RegressionTest(unittest.TestCase):
    def setUp(self):
        self._d = tempfile.TemporaryDirectory()
        self.d = Path(self._d.name)
        (self.d / "img").mkdir()
        for n in ("car6_p0106_0001.jpg", "car6_p0106_0002.jpg"):
            touch(self.d / "img", n)
        self.base = ["106", "--images", str(self.d / "img"), "--out", str(self.d / "out"), "--diag-out", str(self.d / "diag")]

    def tearDown(self):
        self._d.cleanup()

    def run_main(self, extra=()):
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = P.main(self.base + list(extra))
        return rc, buf.getvalue()

    def test_default_without_config_is_benchmark(self):
        """Lỗi 1+2: không truyền --config phải ra full-rgb-imagenet + INTER_LINEAR, không còn y320/INTER_AREA."""
        rc, out = self.run_main()
        self.assertEqual(rc, 0)
        self.assertIn("full-rgb-imagenet, resize cv2.INTER_LINEAR", out)
        self.assertIn("[THEO BENCHMARK]", out)
        self.assertNotIn("INTER_AREA", out)
        self.assertNotIn("y320", out)
        info = json.loads((self.d / "diag" / "run_info" / "p0106.json").read_text(encoding="utf-8"))
        self.assertEqual((info["config"], info["resize"], info["status"]), ("full-rgb-imagenet", "cv2.INTER_LINEAR", "THEO BENCHMARK"))

    def test_explicit_config_shows_linear_and_area_only_on_request(self):
        rc, out = self.run_main(["--config", "full-rgb-imagenet", "--no-diag"])
        self.assertIn("full-rgb-imagenet, resize cv2.INTER_LINEAR", out)
        self.assertNotIn("INTER_AREA", out)
        rc, out = self.run_main(["--config", "y320-rgb-div255", "--resize", "area", "--no-diag"])
        self.assertIn("y320-rgb-div255, resize cv2.INTER_AREA", out)
        self.assertIn("[CHUA XAC MINH]", out)

    def test_header_label(self):
        h = P.header("p0106", "car6_p0106_0001", P.DEFAULT_CFG, P.DEFAULT_RESIZE, 1)
        self.assertIn("full-rgb-imagenet INTER_LINEAR", h)
        self.assertNotIn("INTER_AREA", h)
        self.assertIn(P.BENCH, h)

    def test_overwrite_failure_on_variants_keeps_main_outputs(self):
        """Lỗi 3: ghi đè ảnh variants có sẵn thất bại -> 4 thư mục chính vẫn ghi, tệp cũ còn nguyên, báo cảnh báo riêng."""
        self.assertEqual(self.run_main()[0], 0)  # lần 1: tạo đủ tệp
        old = self.d / "diag" / "variants" / "p0106" / "car6_p0106_0001.png"
        old.write_bytes(b"CU-CHAN-DOAN")
        real = P.os.replace

        def fake(src, dst):
            if Path(dst) == old:
                raise OSError(22, "Invalid argument")
            return real(src, dst)

        with mock.patch.object(P.os, "replace", side_effect=fake):
            rc, out = self.run_main()
        self.assertEqual(rc, 1)
        self.assertEqual(old.read_bytes(), b"CU-CHAN-DOAN")  # không xóa/không ghi hỏng
        self.assertIn("CẢNH BÁO chẩn đoán car6_p0106_0001.jpg", out)
        self.assertIn("errno=22", out)
        self.assertIn("thành công: 2", out)  # 4 thư mục chính của cả 2 ảnh vẫn ghi
        info = json.loads((self.d / "diag" / "run_info" / "p0106.json").read_text(encoding="utf-8"))
        self.assertEqual((info["ok"], info["failed"], info["diag_write_failed"]), (2, 0, 1))
        self.assertFalse(any(p.name.endswith(".tmp") for p in self.d.rglob("*")))

    def test_overwrite_failure_on_main_output_marks_image_failed_only(self):
        self.assertEqual(self.run_main()[0], 0)
        old = self.d / "out" / "mask" / "p0106" / "car6_p0106_0001.png"
        old.write_bytes(b"CU-MASK")
        real = P.os.replace

        def fake(src, dst):
            if Path(dst) == old:
                raise OSError(22, "Invalid argument")
            return real(src, dst)

        with mock.patch.object(P.os, "replace", side_effect=fake):
            rc, out = self.run_main()
        self.assertEqual(rc, 1)
        self.assertEqual(old.read_bytes(), b"CU-MASK")
        self.assertIn("LỖI car6_p0106_0001.jpg", out)
        self.assertIn("thành công: 1", out)
        self.assertTrue((self.d / "out" / "mask" / "p0106" / "car6_p0106_0002.png").stat().st_size > 100)


@unittest.skipUnless(P.MODEL.is_file(), "thiếu model ONNX")
class EndToEndTest(unittest.TestCase):
    def test_structure_and_no_side_effects(self):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            (d / "img").mkdir()
            for n in ("car6_p0106_0001.jpg", "car6_p0106_0025.jpg", "car6_p0105_0001.jpg"):
                touch(d / "img", n)
            digest = lambda: hashlib.md5(b"".join(p.read_bytes() for p in sorted((d / "img").iterdir()))).hexdigest()
            before = digest()
            args = ["106", "--images", str(d / "img"), "--out", str(d / "out"), "--diag-out", str(d / "diag")]
            self.assertEqual(P.main(args), 0)
            self.assertEqual(digest(), before)
            self.assertEqual(sorted(p.name for p in (d / "out").iterdir()), sorted(P.KINDS))
            for kind in P.KINDS:
                self.assertEqual(sorted(p.name for p in (d / "out" / kind).iterdir()), ["p0106"])
                self.assertEqual(sorted(p.name for p in (d / "out" / kind / "p0106").iterdir()),
                                 ["car6_p0106_0001.png", "car6_p0106_0025.png"])
            mask = cv2.imread(str(d / "out" / "mask" / "p0106" / "car6_p0106_0001.png"), cv2.IMREAD_UNCHANGED)
            ov = cv2.imread(str(d / "out" / "overlay" / "p0106" / "car6_p0106_0001.png"))
            rs = cv2.imread(str(d / "out" / "resized" / "p0106" / "car6_p0106_0001.png"))
            self.assertEqual(mask.shape, (192, 320))
            self.assertEqual(rs.shape, ov.shape)
            self.assertTrue(set(np.unique(mask)) <= {0, 255})
            self.assertTrue(np.array_equal(ov[mask == 0], rs[mask == 0]))  # ngoài mask overlay = ảnh resize

    def test_no_match_exits_without_writing(self):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            (d / "img").mkdir()
            touch(d / "img", "car6_p0105_0001.jpg")
            with self.assertRaises(SystemExit):
                P.main(["106", "--images", str(d / "img"), "--out", str(d / "out"), "--diag-out", str(d / "diag")])
            self.assertFalse((d / "out").exists() or (d / "diag").exists())


if __name__ == "__main__":
    unittest.main()
