"""Kiểm thử danh sách detection (phát hiện vật thể) bằng export giả, không dùng dữ liệu xe."""

import contextlib
import csv
import io
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import build_det_lists as tools


NAMES = ['red', 'green', 'off', 'left', 'right', 'no_left', 'no_right',
         'round_barrier', 'work_barrier']


class DetListsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.export = self.root / 'export'
        self.data = self.root / 'data'
        self.export.mkdir()
        self.data.mkdir()
        self.manifest, self.splits = [], []
        self.write_names()

    def write_names(self, names=NAMES, mapping=False):
        content = 'names:\n' + ''.join('  {}{}\n'.format(
            '{}: '.format(i) if mapping else '- ', name) for i, name in enumerate(names))
        (self.export / 'data.yaml').write_text(content, encoding='utf-8')

    def add(self, pass_id, idx, classes=('red',), t=0, s=0, l=0,
            split='train', subset='train', jpg=True):
        stem = 'p{:04d}_{:04d}'.format(pass_id, idx)
        name = stem + ('_jpg' if jpg else '') + '.rf.abc123.jpg'
        image = self.export / subset / 'images' / name
        label = self.export / subset / 'labels' / (image.stem + '.txt')
        image.parent.mkdir(parents=True, exist_ok=True)
        label.parent.mkdir(parents=True, exist_ok=True)
        image.write_bytes(b'anh gia, khong can giai ma')
        label.write_text(''.join('{} 0.5 0.5 0.2 0.2\n'.format(NAMES.index(c))
                                 for c in classes), encoding='utf-8')
        filename = 'car6_' + stem + '.jpg'
        self.manifest.append(dict(file=filename, **{'pass': pass_id}, idx=idx, t=t, s=s, l=l))
        self.splits.append(dict(file=filename, **{'pass': pass_id}, split=split))
        return image, label

    def write_csvs(self):
        for name, fields, rows in (
                ('manifest.csv', ['file', 'pass', 'idx', 't', 's', 'l'], self.manifest),
                ('split_v0.csv', ['file', 'pass', 'split'], self.splits)):
            with (self.data / name).open('w', encoding='utf-8-sig', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                writer.writerows(rows)

    def args(self, *extra):
        return ['--export', str(self.export), '--data-dir', str(self.data)] + list(extra)

    def run_tool(self, *extra):
        self.write_csvs()
        stdout, stderr = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            code = tools.main(self.args(*extra))
        return code, stdout.getvalue(), stderr.getvalue()

    def rows(self):
        with (self.data / 'frame_labels.csv').open(encoding='utf-8-sig', newline='') as stream:
            return list(csv.DictReader(stream))

    def selected(self, split='train', out=None):
        out = self.export if out is None else out
        return (out / (split + '.txt')).read_text(encoding='utf-8').splitlines()

    def test_names_classes_boxes_and_bom(self):
        self.add(1, 1, ('right', 'red', 'right'))
        self.add(1, 2, ('green',), jpg=False, subset='valid', split='val')
        code, stdout, stderr = self.run_tool('--null-frac', '0')
        self.assertEqual((code, stderr), (0, ''))
        rows = self.rows()
        self.assertEqual([row['file'] for row in rows], ['car6_p0001_0001.jpg', 'car6_p0001_0002.jpg'])
        self.assertEqual(rows[0], {'file': 'car6_p0001_0001.jpg', 'pass': '1',
                                  'classes': 'red|right', 'n_boxes': '3', 'is_null': '0', 'null_type': ''})
        self.assertTrue((self.data / 'frame_labels.csv').read_bytes().startswith(b'\xef\xbb\xbf'))
        self.assertIn('red\t1\t1\t0\t0', stdout)
        self.assertIn('green\t0\t0\t1\t1', stdout)
        self.assertIn('null\t0\t0\t0\t0', stdout)

    def test_null_types_all_branches(self):
        values = [(t, 0, 0, 'hard') for t in sorted(tools.HARD_T)]
        values += [(0, 1, 0, 'hard'), (0, 0, -1, 'hard'), (0, 0, 0, 'easy'), (8, 0, 0, 'easy')]
        for idx, (t, s, l, _) in enumerate(values, 1):
            self.add(1, idx, (), t=t, s=s, l=l)
        self.assertEqual(self.run_tool('--null-frac', '0')[0], 0)
        self.assertEqual([row['null_type'] for row in self.rows()], [value[3] for value in values])
        self.assertTrue(all(row['is_null'] == '1' and row['n_boxes'] == '0' and row['classes'] == ''
                            for row in self.rows()))

    def test_right_cap_only_pure_right_on_selected_t_and_by_idx(self):
        # Đảo idx để phát hiện chọn nhầm thứ tự duyệt hoặc chỉ lấy bốn ảnh đầu.
        for idx in (10, 2, 8, 1, 9, 3, 7, 4, 6, 5):
            self.add(9, idx, ('right',), t=9)
        for idx in range(1, 7):
            self.add(3, idx, ('right', 'red'), t=3)
            self.add(4, idx, ('right',), t=3)
        self.add(9, 11, ('right', 'red'), t=9)
        self.assertEqual(self.run_tool('--null-frac', '0')[0], 0)
        lines = self.selected()
        capped = [line for line in lines if 'p0009_' in line and '_0011_' not in line]
        self.assertEqual([int(Path(line).name[6:10]) for line in capped], [1, 4, 7, 10])
        self.assertEqual(sum('p0003_' in line for line in lines), 6)
        self.assertEqual(sum('p0004_' in line for line in lines), 6)
        self.assertEqual(len(lines), 17)
        self.assertEqual(self.run_tool('--cap-right-t', '3,9', '--max-right-per-pass', '2',
                                       '--null-frac', '0')[0], 0)
        self.assertEqual(sum('p0004_' in line for line in self.selected()), 2)
        self.assertEqual(sum('p0003_' in line for line in self.selected()), 6)

    def test_null_formula_uses_filtered_boxes_and_prioritizes_hard(self):
        for idx in range(1, 11):
            self.add(9, idx, ('right',), t=9)
            self.add(1, idx, (), t=1)
            self.add(2, idx, ())
        self.assertEqual(self.run_tool('--null-frac', '0.5')[0], 0)
        lines = self.selected()
        self.assertEqual(len(lines), 8)
        self.assertEqual([int(Path(line).name[6:10]) for line in lines if 'p0001_' in line], [1, 4, 7, 10])
        self.assertFalse(any('p0002_' in line for line in lines))
        self.assertEqual(self.run_tool('--null-frac', '0.6')[0], 0)
        self.assertEqual(len(self.selected()), 10)

    def test_default_fraction_and_easy_fallback_spread(self):
        for idx in range(1, 10):
            self.add(1, idx)
        self.add(2, 1, (), t=7)
        for idx in range(1, 6):
            self.add(3, idx, ())
        self.assertEqual(self.run_tool()[0], 0)
        self.assertEqual(len(self.selected()), 9 + round(0.18 / 0.82 * 9))
        self.assertTrue(any('p0002_' in line for line in self.selected()))
        self.assertTrue(any('p0003_0003' in line for line in self.selected()))
        self.assertEqual(self.run_tool('--null-frac', '0.25')[0], 0)
        self.assertEqual([int(Path(line).name[6:10]) for line in self.selected() if 'p0003_' in line], [1, 5])
        self.assertEqual(self.run_tool('--null-frac', '0.4')[0], 0)
        self.assertEqual(sum('p0003_' in line for line in self.selected()), 5)

    def test_null_shortage_warns_and_uses_all(self):
        self.add(1, 1)
        self.add(2, 1, ())
        code, _, stderr = self.run_tool('--null-frac', '0.8')
        self.assertEqual(code, 0)
        self.assertIn('Cảnh báo: thiếu null', stderr)
        self.assertEqual(len(self.selected()), 2)

    def test_val_unchanged_and_deterministic(self):
        for idx in range(1, 8):
            self.add(9, idx, ('right',), t=9, split='val', subset='test')
        self.add(9, 8, (), split='val', subset='train')
        self.assertEqual(self.run_tool('--max-right-per-pass', '0', '--null-frac', '0')[0], 0)
        self.assertEqual(len(self.selected('val')), 8)
        self.assertEqual(self.selected(), [])
        paths = [self.export / name for name in ('train.txt', 'val.txt')]
        before = [path.read_bytes() for path in paths]
        self.assertEqual(self.run_tool('--max-right-per-pass', '0', '--null-frac', '0')[0], 0)
        self.assertEqual(before, [path.read_bytes() for path in paths])

    def test_cli_default_and_custom_out_paths_and_readonly_inputs(self):
        for idx, subset in enumerate(('train', 'valid', 'test'), 1):
            self.add(1, idx, split='val' if idx == 2 else 'train', subset=subset)
        self.write_csvs()
        protected = [self.data / 'manifest.csv', self.data / 'split_v0.csv', self.export / 'data.yaml']
        before = [path.read_bytes() for path in protected]
        for out in (self.export, self.root / 'out lists'):
            with self.subTest(out=out):
                args = self.args() + ([] if out == self.export else ['--out-dir', str(out)])
                result = subprocess.run([sys.executable, str(Path(tools.__file__))] + args,
                                        stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=30)
                self.assertEqual(result.returncode, 0, result.stderr.decode('utf-8', errors='replace'))
                for split in ('train', 'val'):
                    lines = self.selected(split, out)
                    self.assertTrue(lines)
                    for line in lines:
                        self.assertTrue(line.startswith('./'))
                        image = (out / line).resolve()
                        label = image.parent.parent / 'labels' / (image.stem + '.txt')
                        self.assertTrue(image.is_file(), line)
                        self.assertTrue(label.is_file(), line)
        self.assertEqual(before, [path.read_bytes() for path in protected])

    def test_errors_do_not_write_outputs(self):
        image, label = self.add(1, 1)
        self.write_csvs()
        cases = ['manifest', 'duplicate', 'right', 'filename', 'label', 'bad_box', 'split']
        for case in cases:
            with self.subTest(case=case):
                self.write_csvs()
                self.write_names()
                label.write_text('0 0.5 0.5 0.2 0.2\n', encoding='utf-8')
                extra_image = None
                if case == 'manifest':
                    self.manifest.clear()
                    self.write_csvs()
                elif case == 'duplicate':
                    extra_image = image.with_name(image.name.replace('abc123', 'xyz789'))
                    extra_image.write_bytes(b'')
                elif case == 'right':
                    self.write_names(['red'])
                elif case == 'filename':
                    extra_image = image.with_name('khong_hop_le.jpg')
                    extra_image.write_bytes(b'')
                elif case == 'label':
                    label.unlink()
                elif case == 'bad_box':
                    label.write_text('99 nan 0.5 0.2 0.2\n', encoding='utf-8')
                else:
                    self.splits[0]['split'] = 'test'
                    self.write_csvs()
                result = subprocess.run([sys.executable, str(Path(tools.__file__))] + self.args(),
                                        stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=30)
                self.assertEqual(result.returncode, 1)
                self.assertIn('Lỗi:', result.stderr.decode('utf-8', errors='replace'))
                if case == 'right':
                    self.assertIn('right', result.stderr.decode('utf-8', errors='replace'))
                self.assertFalse((self.data / 'frame_labels.csv').exists())
                self.assertFalse((self.export / 'train.txt').exists())
                if extra_image is not None:
                    extra_image.unlink()
                if case == 'manifest':
                    self.manifest.append(dict(file='car6_p0001_0001.jpg', **{'pass': 1}, idx=1, t=0, s=0, l=0))

    def test_mapping_names_and_invalid_arguments(self):
        self.add(1, 1)
        self.write_names(mapping=True)
        self.assertEqual(self.run_tool()[0], 0)
        for extra in (['--null-frac', '1'], ['--null-frac', '-1'], ['--null-frac', 'nan'],
                      ['--max-right-per-pass', '-1'], ['--cap-right-t', 'abc'], ['--unknown']):
            with self.subTest(extra=extra):
                self.assertEqual(self.run_tool(*extra)[0], 1)


if __name__ == '__main__':
    unittest.main()
