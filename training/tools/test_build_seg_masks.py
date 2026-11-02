"""Kiểm thử mask bằng ảnh và đa giác tổng hợp, không dùng dữ liệu xe."""

import contextlib
import csv
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
        self.categories = {9: tools.TRACK_SURFACE, 3: tools.OBJECT}
        self.track = {'id': 1, 'image_id': 1, 'category_id': 9,
                     'segmentation': [[100, 100, 700, 100, 700, 600, 100, 600]]}
        self.obstacle = {'id': 2, 'image_id': 1, 'category_id': 3,
                         'segmentation': [[300, 300, 400, 300, 400, 400, 300, 400]]}

    def mask(self, annotations=None, **kwargs):
        kwargs.setdefault('line_method', 'median')
        return tools.make_mask(self.image, [self.track, self.obstacle] if annotations is None
                               else annotations, self.categories, 1280, 720, **kwargs)

    def save_coco(self, filename=None, annotations=None, width=1280, height=720):
        data = {'categories': [{'id': key, 'name': value} for key, value in self.categories.items()],
                'images': [{'id': 1, 'file_name': filename or self.name + '_jpg.rf.ab12cd.jpg',
                            'width': width, 'height': height}],
                'annotations': [self.track, self.obstacle] if annotations is None else annotations}
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
                               '--images', str(self.images), '--out', str(self.out),
                               '--line-method', 'median'] + list(extra))
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
        mask = self.mask([self.obstacle, self.track])
        self.assertEqual(mask.shape, (720, 1280))
        self.assertEqual(mask.dtype, np.dtype('uint8'))
        self.assertEqual(int(mask[200, 200]), 1)
        self.assertEqual(int(mask[350, 350]), 0)
        self.assertEqual(int(mask[50, 50]), 0)
        np.testing.assert_array_equal(mask, self.mask())

    def test_white_only_inside_track_and_obstacle_covers_line(self):
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
        annotation = dict(self.track, segmentation=[
            [50, 50, 150, 50, 150, 150, 50, 150],
            [200, 200, 250, 200, 250, 250, 200, 250]])
        mask = tools.make_mask(self.image, [annotation], self.categories, 640, 360,
                               line_method='median')
        self.assertEqual(int(mask[200, 200]), 1)
        self.assertEqual(int(mask[450, 450]), 1)
        self.assertEqual(int(mask[350, 350]), 0)
        self.assertEqual(int(mask[200, 400]), 0)

    def test_roboflow_names_without_car_prefix(self):
        filename = 'p0055_0025_jpg.rf.903e7be6da009a035fabea539e29ce8c.jpg'
        for name in (filename, 'train/' + filename, 'valid\\' + filename,
                     'p0055_0025_jpg.rf.Ab12.jpg'):
            with self.subTest(filename=name):
                self.assertEqual(tools.original_name(name), 'car6_p0055_0025')
        for name in ('p0014_0001.jpg', 'p0055_0025_jpg.jpg',
                     'p0055_0025.rf.ab.jpg', 'p0055_0025_jpg.rf..jpg',
                     'p0055_0025_jpg.rf.ab-cd.jpg', 'p0055_0025_jpg.rf.ab.png',
                     'p0055_0025_jpg.rf.ab.jpeg'):
            with self.subTest(filename=name):
                with self.assertRaisesRegex(ValueError, 'tên gốc'):
                    tools.original_name(name)

    def test_load_jobs_skips_unused_roboflow_parent(self):
        self.categories = {0: 'car6-seg', 9: tools.TRACK_SURFACE}
        path = self.save_coco(filename='p0055_0025_jpg.rf.ab12.jpg',
                              annotations=[self.track])
        error = io.StringIO()
        with contextlib.redirect_stderr(error):
            jobs = tools.load_jobs([path])
        self.assertEqual(len(jobs), 1)
        name, img, annotations, categories = jobs[0]
        self.assertEqual(name, 'car6_p0055_0025')
        self.assertEqual(img['id'], 1)
        self.assertEqual(annotations, [self.track])
        self.assertEqual(categories, {9: tools.TRACK_SURFACE})
        self.assertEqual(error.getvalue(),
                         'CẢNH BÁO: bỏ qua lớp không có annotation: car6-seg\n')

    def test_load_jobs_rejects_annotated_unknown_class(self):
        self.categories[9] = 'ten_la'
        path = self.save_coco()
        with self.assertRaisesRegex(ValueError, 'Tên lớp không hỗ trợ: ten_la'):
            tools.load_jobs([path])

    def test_load_jobs_warns_and_keeps_unannotated_image(self):
        self.name = 'car6_p0117_0020'
        path = self.save_coco(annotations=[])
        error = io.StringIO()
        with contextlib.redirect_stderr(error):
            jobs = tools.load_jobs([path])
        self.assertEqual(len(jobs), 1)
        self.assertEqual(jobs[0][0], self.name)
        self.assertEqual(jobs[0][2], [])
        self.assertEqual(jobs[0][3], self.categories)
        self.assertIn('CẢNH BÁO: ảnh không có annotation, mặt nạ sẽ toàn background: '
                      + self.name + '\n', error.getvalue())

    def test_load_jobs_checks_unused_categories_per_coco_file(self):
        self.categories = {0: 'car6-seg', 9: tools.TRACK_SURFACE}
        first = self.save_coco(annotations=[self.track]).rename(self.root / 'first.json')
        self.name = 'car6_p0055_0025'
        self.categories = {0: tools.TRACK_SURFACE}
        self.track['category_id'] = 0
        second = self.save_coco(annotations=[self.track])
        error = io.StringIO()
        with contextlib.redirect_stderr(error):
            jobs = tools.load_jobs([first, second])
        self.assertEqual(len(jobs), 2)
        self.assertEqual(jobs[0][3], {9: tools.TRACK_SURFACE})
        self.assertEqual(jobs[1][3], {0: tools.TRACK_SURFACE})
        self.assertEqual(error.getvalue(),
                         'CẢNH BÁO: bỏ qua lớp không có annotation: car6-seg\n')

    def test_cli_reads_roboflow_export_with_unused_parent(self):
        self.name = 'car6_p0055_0025'
        self.categories = {0: 'car6-seg', 9: tools.TRACK_SURFACE}
        self.save_original()
        path = self.save_coco(filename='p0055_0025_jpg.rf.ab12.jpg',
                              annotations=[self.track])
        code, output, error = self.run_cli(coco=path)
        self.assertEqual(code, 0, error)
        self.assertIn('CẢNH BÁO: bỏ qua lớp không có annotation: car6-seg', error)
        masks = list(self.out.glob('*.png'))
        self.assertEqual([path.stem for path in masks], [self.name])
        self.assertIsNotNone(cv2.imread(str(masks[0]), cv2.IMREAD_UNCHANGED))

    def test_unknown_class_fails_even_without_annotations(self):
        self.categories[20] = 'ten_la'
        with self.assertRaisesRegex(ValueError, 'ten_la'):
            self.mask([])
        annotation = dict(self.track, category_id=20)
        code, output, error = self.run_cli(coco=self.save_coco(annotations=[annotation]))
        self.assertEqual(code, 1)
        self.assertIn('ten_la', error)
        self.assertFalse(self.out.exists())

    def test_legacy_class_names_rejected(self):
        for name in ('mat_sa_ban', 'vat', 'khong_vach'):
            with self.subTest(name=name):
                self.categories[9] = name
                with self.assertRaisesRegex(ValueError, 'Tên lớp không hỗ trợ: ' + name):
                    self.mask([])
                code, output, error = self.run_cli()
                self.assertEqual(code, 1)
                self.assertIn('Tên lớp không hỗ trợ: ' + name, error)
                self.assertFalse(self.out.exists())

    def test_rle_skipped_with_warning(self):
        ann = dict(self.track, iscrowd=1, segmentation={'size': [720, 1280], 'counts': 'abc'})
        error = io.StringIO()
        with contextlib.redirect_stderr(error):
            mask = self.mask([ann])
        self.assertFalse(mask.any())
        self.assertIn('RLE/iscrowd', error.getvalue())

    def test_bad_polygon_dimensions_and_thresholds(self):
        for polygon in ([0, 0, 10], [0, 0, 1, 1], [0, 0, 1, 0, float('nan'), 1],
                        [0, 0, 1, 0, 1e100, 1]):
            with self.subTest(polygon=polygon), self.assertRaises(ValueError):
                self.mask([dict(self.track, segmentation=[polygon])])
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
        # Ca này chỉ kiểm ngưỡng tuyệt đối, tách ảnh hưởng của quy tắc sáng cục bộ.
        self.assertEqual(self.run_cli('--loc-diff', '255')[0], 0)
        path = self.out / (self.name + '.png')
        self.assertEqual(int(cv2.imread(str(path), cv2.IMREAD_UNCHANGED)[200, 200]), 2)
        self.assertEqual(self.run_cli('--sat-max', '0', '--loc-diff', '255')[0], 0)
        self.assertEqual(int(cv2.imread(str(path), cv2.IMREAD_UNCHANGED)[200, 200]), 1)
        self.assertEqual(self.run_cli('--val-min', '240', '--loc-diff', '255')[0], 0)
        self.assertEqual(int(cv2.imread(str(path), cv2.IMREAD_UNCHANGED)[200, 200]), 1)

    def test_dim_gray_stripe_and_signed_brightness_difference(self):
        self.image[190:211, 190:211] = 140
        self.image[240:261, 190:211] = 60
        mask = self.mask()
        self.assertEqual(int(mask[200, 200]), 2)
        self.assertEqual(int(mask[250, 200]), 1)
        self.assertEqual(int(mask[280, 200]), 1)

    def test_local_threshold_boundaries(self):
        self.image[190:211, 190:211] = (110, 110, 140)
        sat = int(cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)[200, 200, 1])
        self.assertEqual(int(self.mask(loc_diff=40, loc_sat_max=sat)[200, 200]), 2)
        self.assertEqual(int(self.mask(loc_diff=41, loc_sat_max=sat)[200, 200]), 1)
        self.assertEqual(int(self.mask(loc_diff=40, loc_sat_max=sat - 1)[200, 200]), 1)

    def test_closing_fills_holes_without_crossing_track_or_obstacle(self):
        # Khe thảm và vật trắng nhỏ đều dễ bị phép đóng lấp nếu thiếu bước AND lại.
        self.track['segmentation'] = [[100, 100, 700, 100, 700, 600, 100, 600,
                                      100, 202, 102, 202, 102, 199, 100, 199]]
        obstacle = dict(self.obstacle, segmentation=[[104, 195, 105, 195, 105, 196, 104, 196]])
        self.image[190:211, 95:116] = 255
        self.image[200:202, 107:109] = (20, 60, 100)
        annotations = [self.track, self.obstacle, obstacle]
        before = self.mask(annotations, close_kernel=1)
        mask = self.mask(annotations)
        self.assertTrue((before[200:202, 107:109] == 1).all())
        self.assertTrue((mask[200:202, 107:109] == 2).all())
        self.assertEqual(int(mask[200, 99]), 0)
        self.assertEqual(int(mask[200, 101]), 0)
        self.assertEqual(int(mask[195, 104]), 0)
        np.testing.assert_array_equal(mask[before == 0], before[before == 0])

    def test_invalid_local_parameters_rejected_by_function_and_cli(self):
        for name, value in (('loc_blur', 1), ('loc_blur', 2), ('loc_blur', 60),
                            ('loc_diff', -1), ('loc_diff', 256),
                            ('loc_sat_max', -1), ('loc_sat_max', 256),
                            ('close_kernel', 0), ('close_kernel', -1)):
            with self.subTest(name=name, value=value):
                with self.assertRaises(ValueError):
                    self.mask(**{name: value})
                code, output, error = self.run_cli('--' + name.replace('_', '-'), str(value))
                self.assertEqual(code, 1)
                self.assertIn('LỖI:', error)
                self.assertFalse(self.out.exists())

    def test_valid_local_parameter_limits(self):
        for name, value in (('loc_blur', 3), ('loc_diff', 0), ('loc_diff', 255),
                            ('loc_sat_max', 0), ('loc_sat_max', 255),
                            ('close_kernel', 1), ('close_kernel', 2)):
            with self.subTest(name=name, value=value):
                mask = self.mask(**{name: value})
                self.assertEqual(mask.dtype, np.dtype('uint8'))
                self.assertTrue(set(np.unique(mask)) <= {0, 1, 2})

    def test_each_new_parameter_changes_function_and_cli_result(self):
        for name, value in (('loc_blur', 3), ('loc_diff', 60),
                            ('loc_sat_max', 0), ('close_kernel', 1)):
            with self.subTest(name=name):
                self.image[:] = (20, 60, 100)
                self.image[190:211, 190:211] = (110, 110, 140)
                if name == 'close_kernel':
                    self.image[200:202, 200:202] = 0
                self.assertEqual(int(self.mask()[200, 200]), 2)
                self.assertEqual(int(self.mask(**{name: value})[200, 200]), 1)
                self.save_original()
                path = self.out / (self.name + '.png')
                self.assertEqual(self.run_cli()[0], 0)
                self.assertEqual(int(cv2.imread(str(path), cv2.IMREAD_UNCHANGED)[200, 200]), 2)
                self.assertEqual(self.run_cli('--' + name.replace('_', '-'), str(value))[0], 0)
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
        self.track['category_id'] = 999
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
             '--images', str(self.images), '--out', str(self.out), '--line-method', 'median'],
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

    def test_glare_preserves_track_obstacle_and_background(self):
        self.categories[7] = tools.GLARE
        exclusion = dict(self.track, category_id=7,
                         segmentation=[[0, 0, 500, 0, 500, 500, 0, 500]])
        self.image[190:220, 190:220] = 255
        self.image[520:550, 520:550] = 255
        mask = self.mask([exclusion, self.obstacle, self.track])
        self.assertEqual([int(mask[y, x]) for y, x in ((200, 200), (350, 350),
                                                     (50, 50), (530, 530))],
                         [1, 0, 0, 2])
        self.save_original()
        self.assertEqual(self.run_cli(coco=self.save_coco(
            annotations=[self.track, self.obstacle, exclusion]))[0], 0)
        saved = cv2.imread(str(self.out / (self.name + '.png')), cv2.IMREAD_UNCHANGED)
        self.assertEqual(int(saved[200, 200]), 1)

    def test_area_filter_boundaries_and_diagonal_connectivity(self):
        self.image[150:154, 150:156] = 255  # 24 pixel, dưới ngưỡng 25.
        self.image[180:185, 150:155] = 255  # Đúng 25 pixel, phải giữ.
        self.image[200:203, 150:210] = 255
        self.image[250:254, 150:154] = 255
        self.image[254:258, 154:158] = 255  # Hai mảnh 16 pixel nối chéo thành 32.
        mask = self.mask(close_kernel=1)
        self.assertEqual(int(mask[151, 151]), 1)
        self.assertEqual(int(mask[181, 151]), 2)
        self.assertTrue((mask[200:203, 150:210] == 2).all())
        self.assertEqual(int(mask[251, 151]), 2)
        self.assertEqual(int(mask[255, 155]), 2)
        self.assertEqual(int(self.mask(close_kernel=1, min_area=0)[151, 151]), 2)
        self.save_original()
        self.assertEqual(self.run_cli('--min-area', '0', '--close-kernel', '1')[0], 0)
        saved = cv2.imread(str(self.out / (self.name + '.png')), cv2.IMREAD_UNCHANGED)
        self.assertEqual(int(saved[151, 151]), 2)
        self.assertEqual(self.run_cli('--min-area', '100', '--close-kernel', '1')[0], 0)
        saved = cv2.imread(str(self.out / (self.name + '.png')), cv2.IMREAD_UNCHANGED)
        self.assertEqual(int(saved[151, 151]), 1)
        self.assertEqual(int(saved[201, 170]), 2)

    def test_exclusion_after_closing_and_before_area_filter(self):
        self.categories[7] = tools.GLARE
        self.image[190:195, 190:200] = 255
        exclusion = dict(self.track, category_id=7,
                         segmentation=[[194, 190, 195, 190, 195, 194, 194, 194]])
        annotations = [self.track, exclusion]
        unfiltered = self.mask(annotations, min_area=0)
        self.assertTrue((unfiltered[190:195, 194:196] == 1).all())
        self.assertEqual(int(unfiltered[192, 192]), 2)
        self.assertTrue((self.mask(annotations)[190:195, 190:200] == 1).all())

    def test_optional_exclusion_class_keeps_existing_result(self):
        self.image[190:220, 190:220] = 255
        expected = self.mask(min_area=0)
        np.testing.assert_array_equal(self.mask(), expected)
        self.categories[7] = tools.GLARE
        np.testing.assert_array_equal(self.mask(), expected)
        self.save_original()
        self.assertEqual(self.run_cli()[0], 0)
        self.assertEqual(sorted(path.name for path in self.out.iterdir()), [self.name + '.png'])
        self.assertFalse(self.preview.exists())
        self.assertFalse((self.root / 'qa').exists())

    def test_qa_statistics_flags_and_exact_boundaries(self):
        mask = np.zeros((720, 1280), dtype=np.uint8)
        mask[:10, :10] = 1
        mask[:2, :5] = 2
        mask[5:7, :5] = 2
        row = tools.qa_row(self.name, mask, 0.005, 0.15, 1)
        self.assertEqual(row, (self.name, 80, 20, 0.2, 2, 10, 'ratio_cao;nhieu_manh'))
        self.assertEqual(tools.qa_row(self.name, mask, 0.2, 0.2, 2)[-1], '')
        mask[mask == 2] = 1
        self.assertEqual(tools.qa_row(self.name, mask, 0.005, 0.15, 60),
                         (self.name, 100, 0, 0, 0, 0, 'ratio_thap'))
        mask[:] = 0
        self.assertEqual(tools.qa_row(self.name, mask, 0, 1, 0),
                         (self.name, 0, 0, 0, 0, 0, ''))

    def test_qa_csv_and_sheets_sort_flagged_first_and_paginate(self):
        qa = self.root / 'qa'
        coco = self.save_coco()
        data = json.loads(coco.read_text(encoding='utf-8'))
        data['images'], data['annotations'] = [], []
        for index in range(1, 14):
            name = 'car6_p0014_{:04d}'.format(index)
            image = self.image.copy()
            image[480:560, 480:560] = (20, 20 + index * 10, 100)
            if index != 2:
                image[190:280, 190:280] = 255
            tools.write_image(self.images / (name + '.jpg'), image)
            data['images'].append({'id': index, 'file_name': name + '.jpg',
                                   'width': 1280, 'height': 720})
            data['annotations'].append(dict(self.track, image_id=index))
        data['images'].reverse()
        coco.write_text(json.dumps(data), encoding='utf-8')
        code, output, error = self.run_cli('--qa', str(qa), coco=coco)
        self.assertEqual((code, error), (0, ''))
        self.assertIn('Số ảnh có cờ QA: 1', output)
        with (qa / 'qa.csv').open(encoding='utf-8', newline='') as stream:
            reader = csv.DictReader(stream)
            self.assertEqual(reader.fieldnames, ['file', 'track_px', 'line_px', 'line_ratio',
                                                 'n_comp', 'largest_comp_px', 'flag', 'line_method',
                                                 'n_sharp_rejected', 'sharp_rejected_px'])
            rows = list(reader)
        self.assertEqual(len(rows), 13)
        self.assertEqual([row['file'] for row in rows],
                         [img['file_name'][:-4] for img in data['images']])
        for row in rows:
            self.assertEqual(row['line_method'], 'median')
            self.assertEqual((row['n_sharp_rejected'], row['sharp_rejected_px']), ('0', '0'))
            mask = cv2.imread(str(self.out / (row['file'] + '.png')), cv2.IMREAD_UNCHANGED)
            track_px, line_px = int((mask == 1).sum()), int((mask == 2).sum())
            self.assertEqual(int(row['track_px']), track_px)
            self.assertEqual(int(row['line_px']), line_px)
            self.assertAlmostEqual(float(row['line_ratio']), line_px / (track_px + line_px))
            flagged = row['file'].endswith('0002')
            self.assertEqual(row['flag'], 'ratio_thap' if flagged else '')
            self.assertEqual(int(row['n_comp']), 0 if flagged else 1)
            self.assertEqual(int(row['largest_comp_px']), 0 if flagged else line_px)
        sheets = sorted(qa.glob('qa_sheet_*.jpg'))
        self.assertEqual([path.name for path in sheets], ['qa_sheet_01.jpg', 'qa_sheet_02.jpg'])
        first, last = [cv2.imread(str(path)) for path in sheets]
        self.assertEqual(first.shape, (540, 1280, 3))
        self.assertEqual(last.shape, (540, 1280, 3))
        # Ô đầu không có line; ô thứ hai là tên nhỏ nhất trong nhóm bình thường.
        self.assertGreater(int(first[60, 60, 1]), int(first[60, 60, 2]) + 40)
        self.assertGreater(int(first[60, 380, 2]), int(first[60, 60, 2]) + 40)
        np.testing.assert_allclose(first[130, 450], (12, 120, 60), atol=5)
        np.testing.assert_allclose(first[130, 770], (12, 132, 60), atol=5)
        self.assertFalse(last[200:].any())
        self.assertFalse(last[20:160, 340:].any())

    def test_qa_tile_has_solid_text_strip_with_yellow_flag(self):
        image = np.full((720, 1280, 3), 255, dtype=np.uint8)
        zeros = np.zeros((720, 1280), dtype=np.uint8)
        plain = tools.qa_tile(image, zeros, zeros, zeros, 'car6_p0014_0001', '')
        flagged = tools.qa_tile(image, zeros, zeros, zeros, 'car6_p0014_0001', 'ratio_thap;nhieu_manh')
        strip = tools.QA_STRIP
        self.assertEqual(strip, 22)
        for tile in (plain, flagged):
            self.assertEqual(tile.shape, (180, 320, 3))
            self.assertLess(float(tile[:strip].mean()), 80)  # dải nền đen đặc, chỉ chữ là sáng
            self.assertGreater(float(tile[strip:].mean()), 250)  # phần ảnh bên dưới không bị vẽ lên
            self.assertTrue((tile[:strip, :130].min(axis=2) > 200).any())  # chữ trắng của tên
        np.testing.assert_array_equal(plain[strip:], flagged[strip:])
        yellow = lambda tile: (tile[:strip, :, 0] < 60) & (tile[:strip, :, 1] > 200) & (tile[:strip, :, 2] > 200)
        self.assertFalse(yellow(plain).any())
        self.assertTrue(yellow(flagged)[:, 160:].any())
        self.assertFalse(yellow(flagged)[:, :100].any())
        longer = tools.qa_tile(image, zeros, zeros, zeros, 'tien_to_car6_p0014_0001_hau_to_rat_dai', 'ratio_thap;nhieu_manh')
        np.testing.assert_array_equal(longer, flagged)

    def test_qa_overlay_colors_and_does_not_change_preview(self):
        self.categories[7] = tools.GLARE
        self.image[190:280, 190:280] = 255
        exclusion = dict(self.track, category_id=7,
                         segmentation=[[500, 300, 600, 300, 600, 400, 500, 400]])
        annotations = [self.track, self.obstacle, exclusion]
        mask, object_mask, glare, sharp_rejected = tools._make_mask_layers(
            self.image, annotations, self.categories, 1280, 720, line_method='median')
        self.assertFalse(sharp_rejected.any())
        tile = tools.qa_tile(self.image, mask, object_mask, glare, self.name, '')
        for y, x, color in ((500, 500, (0, 255, 0)), (240, 240, (0, 255, 255)),
                             (350, 350, (0, 0, 255)), (350, 550, (255, 0, 255))):
            expected = (self.image[y, x] * 0.6 + np.array(color) * 0.4).astype(np.uint8)
            np.testing.assert_array_equal(tile[y // 4, x // 4], expected)
        self.save_original()
        coco = self.save_coco(annotations=annotations)
        self.assertEqual(self.run_cli('--preview', str(self.preview), coco=coco)[0], 0)
        before = (self.preview / (self.name + '.png')).read_bytes()
        self.assertEqual(self.run_cli('--preview', str(self.preview), '--qa', str(self.root / 'qa'),
                                      coco=coco)[0], 0)
        self.assertEqual((self.preview / (self.name + '.png')).read_bytes(), before)

    def test_cli_qa_custom_thresholds_set_multiple_flags(self):
        self.image[192:216, 192:216] = 255
        self.image[240:264, 192:216] = 255
        self.save_original()
        qa = self.root / 'qa'
        code, output, error = self.run_cli('--qa', str(qa), '--qa-min-ratio', '0',
                                           '--qa-max-ratio', '0', '--qa-max-comp', '1')
        self.assertEqual((code, error), (0, ''))
        with (qa / 'qa.csv').open(encoding='utf-8', newline='') as stream:
            row = next(csv.DictReader(stream))
        self.assertEqual(row['flag'], 'ratio_cao;nhieu_manh')
        self.assertEqual(int(row['n_comp']), 2)
        self.assertGreater(int(row['largest_comp_px']), 25)
        self.assertIn('Số ảnh có cờ QA: 1', output)

    def test_invalid_area_and_qa_parameters_fail_before_writing(self):
        for value in (-1, 1.5, float('nan')):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.mask(min_area=value)
        for extra in (('--min-area', '-1'), ('--qa-max-comp', '-1'),
                      ('--qa-min-ratio', '-0.1'), ('--qa-max-ratio', '1.1'),
                      ('--qa-min-ratio', '0.2'), ('--qa-max-ratio', 'nan'),
                      ('--qa-min-ratio', 'nan'), ('--qa-max-ratio', 'inf')):
            with self.subTest(extra=extra):
                code, output, error = self.run_cli(*extra)
                self.assertEqual(code, 1)
                self.assertIn('LỖI:', error)
                self.assertFalse(self.out.exists())

    def test_unsafe_qa_directories_rejected(self):
        for qa in (self.images, self.images / 'sub', self.out, self.out / 'sub',
                   self.preview, self.preview / 'sub'):
            with self.subTest(qa=qa):
                code, output, error = self.run_cli('--qa', str(qa), '--preview', str(self.preview))
                self.assertEqual(code, 1)
                self.assertIn('--qa', error)
                self.assertFalse(self.out.exists())
                self.assertFalse(self.preview.exists())

    def test_qa_rejects_symlink_directory_parent_and_files(self):
        target = self.root / 'target'
        target.mkdir()
        link = self.root / 'link'
        try:
            link.symlink_to(target, target_is_directory=True)
        except (OSError, NotImplementedError):
            self.skipTest('Máy không cho tạo liên kết tượng trưng để kiểm thử')
        self.save_original()
        for qa in (link, link / 'sub'):
            with self.subTest(qa=qa):
                code, output, error = self.run_cli('--qa', str(qa))
                self.assertEqual(code, 1)
                self.assertIn('liên kết', error)
        qa = self.root / 'qa'
        qa.mkdir()
        protected = target / 'protected'
        protected.write_bytes(b'giu nguyen')
        for filename in ('qa.csv', 'qa_sheet_01.jpg'):
            path = qa / filename
            path.symlink_to(protected)
            code, output, error = self.run_cli('--qa', str(qa))
            self.assertEqual(code, 1)
            self.assertIn('liên kết', error)
            self.assertEqual(protected.read_bytes(), b'giu nguyen')
            path.unlink()

    def test_median_matches_original_pipeline(self):
        self.categories[7] = tools.GLARE
        exclusion = dict(self.track, category_id=7,
                         segmentation=[[194, 190, 195, 190, 195, 230, 194, 230]])
        annotations = [self.track, self.obstacle, exclusion]
        self.image[190:231, 180:211] = 140
        self.image[300:420, 290:420] = 255
        self.image[200:202, 200:202] = 0
        for close_kernel, min_area in ((1, 0), (5, 25), (2, 500)):
            with self.subTest(close_kernel=close_kernel, min_area=min_area):
                expected = np.zeros((720, 1280), dtype=np.uint8)
                cv2.fillPoly(expected, [np.array(self.track['segmentation'][0],
                                                dtype=np.int32).reshape(-1, 2)], 1)
                cv2.fillPoly(expected, [np.array(self.obstacle['segmentation'][0],
                                                dtype=np.int32).reshape(-1, 2)], 0)
                hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
                sat, val = hsv[:, :, 1], hsv[:, :, 2]
                bg = cv2.medianBlur(val, 61)
                white = ((sat <= 75) & (val >= 205)) | (
                    (val.astype(np.int16) - bg.astype(np.int16) >= 30) & (sat <= 90))
                line = (white & (expected == 1)).astype(np.uint8)
                line = cv2.morphologyEx(line, cv2.MORPH_CLOSE,
                                        np.ones((close_kernel, close_kernel), dtype=np.uint8))
                line &= (expected == 1).astype(np.uint8)
                cv2.fillPoly(line, [np.array(exclusion['segmentation'][0],
                                            dtype=np.int32).reshape(-1, 2)], 0)
                if min_area:
                    _, labels, stats, _ = cv2.connectedComponentsWithStats(line, connectivity=8)
                    keep = stats[:, cv2.CC_STAT_AREA] >= min_area
                    keep[0] = False
                    line = keep[labels]
                expected[line != 0] = 2
                np.testing.assert_array_equal(
                    self.mask(annotations, line_method='median', close_kernel=close_kernel,
                              min_area=min_area), expected)

    def test_tophat_wide_dim_close_stripes_and_default(self):
        self.image[:] = 100
        for x in (180, 272, 364):
            self.image[180:270, x:x + 80] = 140
        mask = self.mask([self.track], line_method='tophat_sharp')
        for x in (180, 272, 364):
            self.assertTrue((mask[180:270, x:x + 80] == 2).all())
        self.assertEqual(int(mask[220, 265]), 1)
        np.testing.assert_array_equal(
            mask, tools.make_mask(self.image, [self.track], self.categories, 1280, 720))
        _, _, _, rejected = tools._make_mask_layers(
            self.image, [self.track], self.categories, 1280, 720)
        self.assertFalse(rejected.any())
        self.save_original()
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            code = tools.main(['--coco', str(self.save_coco(annotations=[self.track])),
                               '--images', str(self.images), '--out', str(self.out)])
        self.assertEqual(code, 0)
        saved = cv2.imread(str(self.out / (self.name + '.png')), cv2.IMREAD_UNCHANGED)
        self.assertEqual(int(saved[220, 220]), 2)

    def test_tophat_gaussian_glow_rejected_and_qa_orange(self):
        y, x = np.ogrid[:720, :1280]
        val = (100 + 80 * np.exp(-((x - 220) ** 2 / (2 * 25.0 ** 2)
                                   + (y - 260) ** 2 / (2 * 50.0 ** 2)))).astype(np.uint8)
        self.image = np.repeat(val[:, :, None], 3, axis=2)
        tophat = val - cv2.morphologyEx(val, cv2.MORPH_OPEN,
                                       cv2.getStructuringElement(cv2.MORPH_RECT, (101, 101)))
        self.assertGreaterEqual(int(tophat[260, 220]), 30)
        mask, object_mask, glare, rejected = tools._make_mask_layers(
            self.image, [self.track], self.categories, 1280, 720)
        self.assertEqual(rejected.dtype, np.dtype('uint8'))
        self.assertTrue(set(np.unique(rejected)) <= {0, 1})
        self.assertEqual(int(rejected[260, 220]), 1)
        self.assertTrue((mask[rejected != 0] == 1).all())
        self.assertFalse((mask == 2).any())
        tile = tools.qa_tile(self.image, mask, object_mask, glare, self.name, '', rejected)
        expected = (self.image[260:264, 220:224] * 0.6
                    + np.array((0, 165, 255)) * 0.4).astype(np.uint8)
        np.testing.assert_allclose(tile[65, 55], expected.mean(axis=(0, 1)), atol=1)
        self.save_original()
        qa = self.root / 'qa'
        code, output, error = self.run_cli('--line-method', 'tophat_sharp', '--qa', str(qa),
                                           coco=self.save_coco(annotations=[self.track]))
        self.assertEqual((code, error), (0, ''))
        with (qa / 'qa.csv').open(encoding='utf-8', newline='') as stream:
            reader = csv.DictReader(stream)
            self.assertEqual(reader.fieldnames, ['file', 'track_px', 'line_px', 'line_ratio',
                                                 'n_comp', 'largest_comp_px', 'flag', 'line_method',
                                                 'n_sharp_rejected', 'sharp_rejected_px'])
            row = next(reader)
        self.assertEqual(row['line_method'], 'tophat_sharp')
        self.assertEqual(int(row['n_sharp_rejected']), 1)
        self.assertGreater(int(row['sharp_rejected_px']), 25)
        sheet = cv2.imread(str(qa / 'qa_sheet_01.jpg'))
        self.assertGreater(int(sheet[65, 55, 2]), int(sheet[65, 55, 1]) + 20)
        self.assertGreater(int(sheet[65, 55, 1]), int(sheet[65, 55, 0]) + 40)

    def test_tophat_sharp_boundary_formula_and_tinted_line(self):
        hsv = np.zeros_like(self.image)
        hsv[:] = (0, 0, 100)
        hsv[180:240, 180:240] = (20, 120, 160)
        self.image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        val = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)[:, :, 2]
        component = np.zeros((720, 1280), dtype=np.uint8)
        component[180:240, 180:240] = 1
        border = (component != 0) & (cv2.erode(component, np.ones((3, 3), np.uint8)) == 0)
        smooth = cv2.GaussianBlur(val.astype(np.float32), (3, 3), 0)
        gradient = cv2.magnitude(cv2.Sobel(smooth, cv2.CV_32F, 1, 0, ksize=3),
                                 cv2.Sobel(smooth, cv2.CV_32F, 0, 1, ksize=3))
        tophat = val - cv2.morphologyEx(val, cv2.MORPH_OPEN,
                                       cv2.getStructuringElement(cv2.MORPH_RECT, (101, 101)))
        score = (gradient[border].astype(np.float64).mean()
                 / max(tophat[component != 0].mean(), 1))
        self.assertGreater(score, 0.85)
        self.assertEqual(int(self.mask(line_method='tophat_sharp')[200, 200]), 2)
        self.assertEqual(int(self.mask(line_method='tophat_sharp', sharp_min=score)[200, 200]), 2)
        mask, _, _, rejected = tools._make_mask_layers(
            self.image, [self.track], self.categories, 1280, 720, sharp_min=score + 0.0001)
        self.assertEqual((int(mask[200, 200]), int(rejected[200, 200])), (1, 1))
        self.assertEqual(int(self.mask(line_method='tophat_sharp',
                                       tophat_sat_max=100)[200, 200]), 1)

    def test_tophat_object_dilation_track_clipping_and_closing(self):
        self.image[:] = 100
        self.image[290:412, 290:412] = 150
        self.image[190:220, 90:120] = 150
        self.image[200:202, 110:112] = 100
        mask = self.mask(line_method='tophat_sharp', tophat_kernel=151)
        self.assertEqual(int(mask[350, 350]), 0)
        self.assertTrue((mask[297:404, 297:404][mask[297:404, 297:404] != 0] == 1).all())
        self.assertEqual(int(mask[295, 350]), 2)
        self.assertEqual(int(mask[405, 350]), 2)
        self.assertEqual(int(mask[200, 99]), 0)
        self.assertEqual(int(mask[200, 110]), 2)
        self.assertEqual(int(self.mask(line_method='tophat_sharp', object_dilate=0,
                                       tophat_kernel=151)[298, 350]), 2)

    def test_tophat_glare_after_area_filter_and_eight_neighbors(self):
        self.categories[7] = tools.GLARE
        self.image[:] = 100
        self.image[190:195, 190:200] = 150
        self.image[250:254, 150:154] = 150
        self.image[254:258, 154:158] = 150
        self.image[280:284, 150:156] = 150
        exclusion = dict(self.track, category_id=7,
                         segmentation=[[194, 190, 195, 190, 195, 194, 194, 194]])
        mask = self.mask([self.track, exclusion], line_method='tophat_sharp', close_kernel=1)
        self.assertTrue((mask[190:195, 194:196] == 1).all())
        self.assertEqual(int(mask[192, 192]), 2)  # Lọc diện tích trước glare nên giữ mảnh 20 px.
        self.assertEqual(int(mask[251, 151]), 2)
        self.assertEqual(int(mask[255, 155]), 2)
        self.assertEqual(int(mask[281, 151]), 1)

    def test_invalid_tophat_parameters_function_and_cli(self):
        for name, value in (('tophat_kernel', 1), ('tophat_kernel', 2), ('tophat_kernel', 100),
                            ('tophat_min', -1), ('tophat_min', 256),
                            ('tophat_sat_max', -1), ('tophat_sat_max', 256),
                            ('object_dilate', -1), ('sharp_min', -0.1),
                            ('sharp_min', float('nan')), ('sharp_min', float('inf'))):
            with self.subTest(name=name, value=value):
                with self.assertRaises(ValueError):
                    self.mask(line_method='tophat_sharp', **{name: value})
                code, output, error = self.run_cli('--line-method', 'tophat_sharp',
                                                  '--' + name.replace('_', '-'), str(value))
                self.assertEqual(code, 1)
                self.assertIn('LỖI:', error)
                self.assertFalse(self.out.exists())
        for name in ('tophat_kernel', 'object_dilate'):
            with self.subTest(name=name), self.assertRaises(ValueError):
                self.mask(**{name: 3.5})
        with self.assertRaises(ValueError):
            self.mask(line_method='sai')
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
            self.run_cli('--line-method', 'sai')
        self.assertEqual(error.exception.code, 2)

    def test_tophat_valid_parameter_limits(self):
        for name, value in (('tophat_kernel', 3), ('tophat_min', 0), ('tophat_min', 255),
                            ('tophat_sat_max', 0), ('tophat_sat_max', 255),
                            ('object_dilate', 0), ('sharp_min', 0)):
            with self.subTest(name=name, value=value):
                mask = self.mask(line_method='tophat_sharp', **{name: value})
                self.assertEqual(mask.dtype, np.dtype('uint8'))
                self.assertTrue(set(np.unique(mask)) <= {0, 1, 2})

    def test_tophat_parameters_reach_function_and_cli(self):
        for name, value in (('tophat_kernel', 3), ('tophat_min', 60),
                            ('tophat_sat_max', 0), ('object_dilate', 0), ('sharp_min', 100)):
            with self.subTest(name=name):
                self.image[:] = 100
                self.image[180:240, 180:240] = 140
                y, x, before, after = 200, 200, 2, 1
                if name == 'tophat_sat_max':
                    hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
                    hsv[180:240, 180:240] = (20, 120, 140)
                    self.image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
                if name == 'object_dilate':
                    self.image[290:412, 290:412] = 150
                    y, x, before, after = 298, 350, 1, 2
                params = {'line_method': 'tophat_sharp', 'tophat_kernel': 151}
                self.assertEqual(int(self.mask(**params)[y, x]), before)
                params[name] = value
                self.assertEqual(int(self.mask(**params)[y, x]), after)
                self.save_original()
                code, output, error = self.run_cli(
                    '--line-method', 'tophat_sharp', '--tophat-kernel', '151',
                    '--' + name.replace('_', '-'), str(value))
                self.assertEqual((code, error), (0, ''))
                saved = cv2.imread(str(self.out / (self.name + '.png')), cv2.IMREAD_UNCHANGED)
                self.assertEqual(int(saved[y, x]), after)


if __name__ == '__main__':
    unittest.main()
