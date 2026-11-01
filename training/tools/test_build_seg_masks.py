"""Kiểm thử mask bằng ảnh và đa giác tổng hợp, không dùng dữ liệu xe."""

import contextlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import cv2
import numpy as np

import build_seg_masks as tools


class SegMasksTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.images = self.root / 'images'
        self.images.mkdir()
        self.out = self.root / 'masks'
        self.preview = self.root / 'preview'
        self.name = 'car6_p0014_0001'
        self.image = np.full((720, 1280, 3), (20, 60, 100), dtype=np.uint8)
        self.categories = {9: 'mat_sa_ban', 3: 'vat'}
        self.road = {'id': 1, 'image_id': 1, 'category_id': 9,
                     'segmentation': [[100, 100, 700, 100, 700, 600, 100, 600]]}
        self.obstacle = {'id': 2, 'image_id': 1, 'category_id': 3,
                         'segmentation': [[300, 300, 400, 300, 400, 400, 300, 400]]}

    def mask(self, annotations=None, **kwargs):
        return tools.make_mask(self.image, [self.road, self.obstacle] if annotations is None
                               else annotations, self.categories, 1280, 720, **kwargs)

    def save_coco(self, filename=None, annotations=None, width=1280, height=720):
        data = {'categories': [{'id': key, 'name': value} for key, value in self.categories.items()],
                'images': [{'id': 1, 'file_name': filename or self.name + '_jpg.rf.ab12cd.jpg',
                            'width': width, 'height': height}],
                'annotations': [self.road, self.obstacle] if annotations is None else annotations}
        path = self.root / '_annotations.coco.json'
        path.write_text(json.dumps(data), encoding='utf-8')
        return path

    def save_original(self):
        path = self.images / (self.name + '.jpg')
        self.assertTrue(cv2.imwrite(str(path), self.image))
        return path

    def run_cli(self, *extra, coco=None):
        output, error = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(output), contextlib.redirect_stderr(error):
            code = tools.main(['--coco', str(coco or self.save_coco()),
                               '--images', str(self.images), '--out', str(self.out)] + list(extra))
        return code, output.getvalue(), error.getvalue()

    def test_original_names_and_invalid_patterns(self):
        for filename in (self.name + '.jpg', self.name + '_jpg.rf.ab12cd.jpg',
                         self.name + '.rf.123.jpg', 'train/' + self.name + '_jpg.rf.ab.jpg',
                         'valid\\' + self.name + '.jpg'):
            with self.subTest(filename=filename):
                self.assertEqual(tools.original_name(filename), self.name)
        for filename in ('p0014_0001.jpg', 'car6_p14_0001.jpg', self.name + '_extra.jpg',
                         self.name + '_jpg.rf..jpg', self.name + '.txt', ''):
            with self.subTest(filename=filename):
                with self.assertRaisesRegex(ValueError, 'tên gốc'):
                    tools.original_name(filename)

    def test_layers_size_dtype_and_annotation_order(self):
        mask = self.mask([self.obstacle, self.road])
        self.assertEqual(mask.shape, (720, 1280))
        self.assertEqual(mask.dtype, np.dtype('uint8'))
        self.assertEqual(int(mask[200, 200]), 1)
        self.assertEqual(int(mask[350, 350]), 0)
        self.assertEqual(int(mask[50, 50]), 0)
        np.testing.assert_array_equal(mask, self.mask())

    def test_white_only_inside_road_and_obstacle_covers_line(self):
        self.image[190:220, 190:220] = 255
        self.image[30:60, 30:60] = 255
        self.image[330:370, 330:370] = 255
        self.image[230:260, 230:260] = (0, 255, 255)
        mask = self.mask()
        self.assertEqual([int(mask[y, x]) for y, x in ((200, 200), (40, 40), (350, 350), (240, 240))],
                         [2, 0, 0, 1])

    def test_hsv_threshold_boundaries(self):
        # Đổi HSV->BGR->HSV làm tròn nên đọc lại S, V thực của pixel rồi đặt ngưỡng đúng biên.
        hsv = np.zeros_like(self.image)
        hsv[:] = (0, 75, 205)
        self.image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        sat, val = (int(x) for x in cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)[200, 200][1:])
        self.assertEqual(int(self.mask(sat_max=sat, val_min=val)[200, 200]), 2)
        self.assertEqual(int(self.mask(sat_max=sat - 1, val_min=val)[200, 200]), 1)
        self.assertEqual(int(self.mask(sat_max=sat, val_min=val + 1)[200, 200]), 1)

    def test_scaled_json_and_multiple_rings(self):
        annotation = dict(self.road, segmentation=[
            [50, 50, 150, 50, 150, 150, 50, 150],
            [200, 200, 250, 200, 250, 250, 200, 250]])
        mask = tools.make_mask(self.image, [annotation], self.categories, 640, 360)
        self.assertEqual(int(mask[200, 200]), 1)
        self.assertEqual(int(mask[450, 450]), 1)
        self.assertEqual(int(mask[350, 350]), 0)
        self.assertEqual(int(mask[200, 400]), 0)

    def test_unknown_class_fails_even_without_annotations(self):
        self.categories[20] = 'ten_la'
        with self.assertRaisesRegex(ValueError, 'ten_la'):
            self.mask([])
        code, output, error = self.run_cli()
        self.assertEqual(code, 1)
        self.assertIn('ten_la', error)
        self.assertFalse(self.out.exists())

    def test_rle_skipped_with_warning(self):
        ann = dict(self.road, iscrowd=1, segmentation={'size': [720, 1280], 'counts': 'abc'})
        error = io.StringIO()
        with contextlib.redirect_stderr(error):
            mask = self.mask([ann])
        self.assertFalse(mask.any())
        self.assertIn('RLE/iscrowd', error.getvalue())

    def test_bad_polygon_dimensions_and_thresholds(self):
        for polygon in ([0, 0, 10], [0, 0, 1, 1], [0, 0, 1, 0, float('nan'), 1],
                        [0, 0, 1, 0, 1e100, 1]):
            with self.subTest(polygon=polygon), self.assertRaises(ValueError):
                self.mask([dict(self.road, segmentation=[polygon])])
        for width, height in ((0, 720), (1280, -1), (float('inf'), 720)):
            with self.subTest(width=width, height=height), self.assertRaises(ValueError):
                tools.make_mask(self.image, [], self.categories, width, height)
        with self.assertRaises(ValueError):
            self.mask(sat_max=256)
        with self.assertRaises(ValueError):
            tools.make_mask(self.image[:10], [], self.categories, 1280, 720)

    def test_cli_thresholds_preview_and_ratios(self):
        self.image[190:220, 190:220] = 255
        original = self.save_original()
        before = original.read_bytes()
        code, output, error = self.run_cli('--preview', str(self.preview), '--sat-max', '0', '--val-min', '255')
        self.assertEqual((code, error), (0, ''))
        mask = cv2.imread(str(self.out / (self.name + '.png')), cv2.IMREAD_UNCHANGED)
        overlay = cv2.imread(str(self.preview / (self.name + '.png')))
        self.assertEqual(mask.shape, (720, 1280))
        self.assertEqual(mask.dtype, np.dtype('uint8'))
        self.assertEqual(int(mask[200, 200]), 2)
        np.testing.assert_array_equal(overlay[50, 50], cv2.imread(str(original))[50, 50])
        self.assertGreater(int(overlay[250, 250, 1]), int(overlay[250, 250, 2]))
        self.assertGreater(int(overlay[200, 200, 2]), int(overlay[200, 200, 1]))
        self.assertEqual(original.read_bytes(), before)
        self.assertIn('đã tạo mask: 1; thiếu file gốc: 0', output)
        values = [float(item.split('=')[1]) for item in output.splitlines()[-1].split(': ')[1].split(', ')]
        expected = np.bincount(mask.ravel(), minlength=3) / mask.size
        np.testing.assert_allclose(values, expected, atol=0.000001)

    def test_cli_threshold_changes_result(self):
        self.image[190:220, 190:220] = (160, 160, 220)
        self.save_original()
        self.assertEqual(self.run_cli()[0], 0)
        path = self.out / (self.name + '.png')
        self.assertEqual(int(cv2.imread(str(path), cv2.IMREAD_UNCHANGED)[200, 200]), 2)
        self.assertEqual(self.run_cli('--sat-max', '0')[0], 0)
        self.assertEqual(int(cv2.imread(str(path), cv2.IMREAD_UNCHANGED)[200, 200]), 1)
        self.assertEqual(self.run_cli('--val-min', '240')[0], 0)
        self.assertEqual(int(cv2.imread(str(path), cv2.IMREAD_UNCHANGED)[200, 200]), 1)

    def test_missing_original_warns_and_skips(self):
        code, output, error = self.run_cli()
        self.assertEqual(code, 0)
        self.assertIn('thiếu ảnh gốc', error)
        self.assertIn('đã tạo mask: 0; thiếu file gốc: 1', output)
        self.assertEqual(list(self.out.iterdir()), [])

    def test_multiple_coco_paths_and_directory_input(self):
        self.save_original()
        first = self.save_coco()
        data = json.loads(first.read_text(encoding='utf-8'))
        data['images'][0]['file_name'] = 'car6_p0018_0002_jpg.rf.123.jpg'
        second = self.root / 'valid.json'
        second.write_text(json.dumps(data), encoding='utf-8')
        tools.write_image(self.images / 'car6_p0018_0002.jpg', self.image)
        code, output, error = self.run_cli('--coco', str(self.root), str(second))
        self.assertEqual((code, error), (0, ''))
        self.assertIn('đã tạo mask: 2', output)
        self.assertEqual(len(list(self.out.glob('*.png'))), 2)

    def test_duplicate_names_and_bad_references_fail_before_writing(self):
        path = self.save_coco()
        with self.assertRaisesRegex(ValueError, 'Trùng tên gốc'):
            tools.load_jobs([path, path])
        self.road['category_id'] = 999
        code, output, error = self.run_cli()
        self.assertEqual(code, 1)
        self.assertIn('tham chiếu', error)
        self.assertFalse(self.out.exists())

    def test_unsafe_output_and_thresholds_rejected(self):
        for extra in (('--out', str(self.images)), ('--out', str(self.images / 'sub')),
                      ('--preview', str(self.out)), ('--sat-max', '-1'), ('--val-min', '256')):
            with self.subTest(extra=extra):
                self.assertEqual(self.run_cli(*extra)[0], 1)
        self.assertFalse(self.out.exists())

    def test_actual_subprocess(self):
        self.save_original()
        path = self.save_coco()
        result = subprocess.run(
            [sys.executable, '-B', str(Path(tools.__file__).resolve()), '--coco', str(path),
             '--images', str(self.images), '--out', str(self.out)],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=20,
            env=dict(os.environ, PYTHONIOENCODING='utf-8'))
        self.assertEqual(result.returncode, 0, result.stderr.decode('utf-8'))
        self.assertTrue((self.out / (self.name + '.png')).is_file())
        help_result = subprocess.run(
            [sys.executable, '-B', str(Path(tools.__file__).resolve()), '--help'],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=20,
            env=dict(os.environ, PYTHONIOENCODING='cp1252', PYTHONUTF8='0'))
        self.assertEqual(help_result.returncode, 0, help_result.stderr.decode('utf-8'))
        self.assertIn('đa giác', help_result.stdout.decode('utf-8'))


if __name__ == '__main__':
    unittest.main()
