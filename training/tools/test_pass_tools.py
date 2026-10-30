"""Kiểm thử T28 bằng CSV và ảnh giả, tự dọn thư mục tạm."""

import contextlib
import csv
import io
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import pass_tools as tools


class PassToolsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / 'data_raw'
        self.root.mkdir()
        self.events, self.frames, self.records = [], [], {}
        self.form = Path(self.temp.name) / 'phieu.csv'
        self.destination = Path(self.temp.name) / 'copies'

    def write_csv(self, path, fields, rows, encoding='utf-8'):
        with path.open('w', encoding=encoding, newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)

    def save(self):
        self.write_csv(self.root / 'passes.csv', tools.PASS_COLUMNS, self.events)
        self.write_csv(self.root / 'frames.csv',
                       ('pass', 'file', 'cmd_speed', 'monotonic_ns'), self.frames)

    def label(self, pid, tag=(6, 3, 1, 2, 1, 1), result='tốt', note=''):
        row = dict(zip(tools.AXES, map(str, tag)))
        row.update({'lượt': str(pid), 'kết quả': result, 'ghi chú': note})
        return row

    def save_labels(self):
        self.write_csv(self.root / 'labels.csv', tools.LABEL_COLUMNS,
                       [self.records[pid] for pid in sorted(self.records)])

    def add_pass(self, pid, tag=(6, 3, 1, 2, 1, 1), count=3, end=True, bad=False):
        base = {'pass': str(pid), 'ts_ms': '0', 'frames': '0', 'end_reason': ''}
        self.events.append(dict(base, event='start'))
        names = []
        for index in range(count):
            name = 'p{:04d}_{:04d}.jpg'.format(pid, index)
            (self.root / name).write_bytes(b'anh gia')
            self.frames.append({'pass': str(pid), 'file': name,
                                'cmd_speed': '3', 'monotonic_ns': str(index)})
            names.append(name)
        if end:
            self.events.append(dict(base, event='end', ts_ms='10',
                                    frames=str(count), end_reason='stop'))
        self.records[pid] = self.label(pid, tag, 'hỏng' if bad else 'tốt')
        self.save()
        return names

    def write_form(self, rows, fields=None, bom=False):
        self.write_csv(self.form, fields or tools.LABEL_COLUMNS, rows,
                       'utf-8-sig' if bom else 'utf-8')

    def saved(self):
        return tools.read_labels(self.root / 'labels.csv', tools.read_passes(self.root))

    def issues(self):
        return '\n'.join(tools.inspect_dataset(self.root)[2])

    def run_cli(self, *args):
        stdout, stderr = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            status = tools.main(list(args))
        return status, stdout.getvalue(), stderr.getvalue()

    def apply(self, copy=False):
        args = ['apply', str(self.root), str(self.form)]
        if copy:
            args.extend(['--copy-to', str(self.destination)])
        return self.run_cli(*args)

    def test_valid_dataset_and_read_only(self):
        self.add_pass(123)
        self.save_labels()
        before = {p.name: p.read_bytes() for p in self.root.iterdir()}
        for command in ('check', 'coverage', 'split', 'sheet'):
            self.assertEqual(self.run_cli(command, str(self.root))[0], 0)
        self.assertEqual(before, {p.name: p.read_bytes() for p in self.root.iterdir()})

    def test_zero_bytes_and_tmp(self):
        name = self.add_pass(1)[0]
        (self.root / name).write_bytes(b'')
        (self.root / (name + '.tmp')).write_bytes(b'do dang')
        self.assertIn('0 byte', self.issues())
        self.assertIn('Tệp tạm', self.issues())

    def test_missing_end_blocks_check_split_coverage_apply(self):
        self.add_pass(1, end=False)
        self.write_form([self.records[1]])
        for command in ('check', 'split', 'coverage'):
            code, out, err = self.run_cli(command, str(self.root))
            self.assertEqual((code, out), (1, ''))
            self.assertIn('thiếu end', err)
        self.assertEqual(self.apply()[0], 1)
        self.assertFalse((self.root / 'labels.csv').exists())

    def test_duplicate_pass_and_end(self):
        self.add_pass(1)
        for event, expected in ((self.events[0], 'số lượt trùng'),
                                (self.events[1], 'end trùng')):
            self.events.append(dict(event))
            self.save()
            self.assertIn(expected, self.issues())
            self.events.pop()

    def test_bad_event_and_labelled_source_image_rejected(self):
        self.add_pass(1)
        self.events.append(dict(self.events[-1], event='bad'))
        self.save()
        self.assertIn('start/end', self.issues())
        (self.root / 'p0001_c6_t3_s1_e2_k1_v1_0000.jpg').write_bytes(b'anh')
        self.assertIn('Sai tên ảnh T28', self.issues())

    def test_duplicate_frame_row(self):
        self.add_pass(1)
        self.frames.append(dict(self.frames[0]))
        self.save()
        self.assertIn('Dòng ảnh trùng', self.issues())

    def test_image_csv_cross_checks(self):
        names = self.add_pass(1)
        (self.root / names[0]).unlink()
        self.frames.pop()
        self.frames[0]['pass'] = '2'
        self.save()
        for expected in ('ảnh thiếu/không hợp lệ', 'Ảnh thiếu dòng frames.csv',
                         'pass không có', 'số frames'):
            self.assertIn(expected, self.issues())

    def test_frame_pass_and_orphan_image(self):
        self.add_pass(1)
        self.add_pass(2)
        self.frames[0]['pass'] = '2'
        self.save()
        self.assertIn('pass khác tên ảnh', self.issues())
        self.events = [row for row in self.events if row['pass'] != '1']
        self.save()
        self.assertIn('Ảnh thuộc lượt không có', self.issues())

    def test_csv_missing_columns_and_malformed_row(self):
        self.add_pass(1)
        path = self.root / 'frames.csv'
        path.write_text('file\nabc\n', encoding='utf-8')
        self.assertIn('cột trùng hoặc thiếu', self.issues())
        path.write_text('pass,file\n1\n', encoding='utf-8')
        self.assertIn('thiếu/thừa ô', self.issues())

    def test_missing_csv_and_root(self):
        self.assertTrue(tools.inspect_dataset(self.root)[2])
        self.assertEqual(self.run_cli('check', str(self.root / 'absent'))[0], 1)

    def test_path_traversal_in_frames(self):
        self.add_pass(1)
        self.frames[0]['file'] = '../' + self.frames[0]['file']
        self.save()
        self.assertIn('thư mục phẳng', self.issues())

    def test_sheet_headers_completed_only_and_round_trip(self):
        self.add_pass(123, count=2)
        self.add_pass(5, count=0)
        self.add_pass(9, end=False)
        code, out, err = self.run_cli('sheet', str(self.root))
        self.assertEqual((code, err), (0, ''))
        rows = list(csv.reader(io.StringIO(out)))
        self.assertEqual(rows[0], ['Lượt', 'Chặng (c)', 'Mục tiêu (t)', 'Biển (s)',
                                   'Ánh sáng (e)', 'Kiểu lái (k)', 'Hướng (v)',
                                   'Kết quả', 'Ghi chú', 'Số ảnh'])
        self.assertEqual(rows[1:], [['5'] + [''] * 8 + ['0'],
                                    ['123'] + [''] * 8 + ['2']])
        self.form.write_text(out, encoding='utf-8-sig')
        with contextlib.redirect_stderr(io.StringIO()) as warnings:
            self.assertEqual(tools.read_sheet(self.form, tools.read_passes(
                self.root, require_end=False)), {})
        self.assertIn('ô trống', warnings.getvalue())

    def test_google_bom_codes_case_extra_column_and_multiline_note(self):
        self.add_pass(123)
        fields = tuple(cell.upper() for cell in tools.SHEET_COLUMNS)
        values = ['00123', '6 - Chặng sáu', '3 - Giao lộ đèn + biển (ch.6)',
                  '1 - Rẽ trái', '2 - Sáng thường/râm', '1 - Đúng tâm',
                  '1 - Thuận chiều', 'TốT', 'Ghi chú, có "ngoặc"\nvà dòng mới', '999']
        self.write_form([dict(zip(fields, values))], fields, bom=True)
        self.assertEqual(self.apply()[0], 0)
        saved = self.saved()[123]
        self.assertEqual([saved[key] for key in tools.AXES], ['6', '3', '1', '2', '1', '1'])
        self.assertEqual(saved['kết quả'], 'tốt')
        self.assertEqual(saved['ghi chú'], values[8])
        with (self.root / 'labels.csv').open(encoding='utf-8', newline='') as stream:
            self.assertEqual(next(csv.reader(stream)), list(tools.LABEL_COLUMNS))

    def test_bare_and_parenthesized_headers_case_insensitive(self):
        self.add_pass(1)
        for fields in (('LƯỢT', 'C', 'T', 'S', 'E', 'K', 'V', 'KẾT QUẢ', 'GHI CHÚ'),
                       ('Lượt', 'Tên khác (C)', 'Mục tiêu (t)', 'S',
                        'Ánh sáng (E)', 'k', 'Hướng (v)', 'Kết quả', 'Ghi chú')):
            with self.subTest(fields=fields):
                values = ['1', '6', '3', '1', '2', '1', '1', 'HỎNG', '']
                self.write_form([dict(zip(fields, values))], fields)
                self.assertEqual(self.apply()[0], 0)
                self.assertEqual(self.saved()[1]['kết quả'], 'hỏng')

    def test_blank_rows_ignored(self):
        self.add_pass(1)
        self.write_form([self.records[1]])
        with self.form.open('a', encoding='utf-8') as stream:
            stream.write('\n,,,,,,,,\n   \n')
        code, out, err = self.apply()
        self.assertEqual((code, err), (0, ''))
        self.assertIn('1 lượt', out)

    def test_missing_duplicate_empty_headers_no_mutation(self):
        self.add_pass(1)
        self.save_labels()
        before = (self.root / 'labels.csv').read_bytes()
        for text in ('Lượt,c,t,s,e,k,v,Kết quả\n',
                     'Lượt,c,C,t,s,e,k,v,Kết quả,Ghi chú\n', '',
                     'Lượt,c,t,s,e,k,v,Kết quả (tốt/hỏng),Ghi chú\n'):
            with self.subTest(text=text):
                self.form.write_text(text, encoding='utf-8-sig')
                self.assertEqual(self.apply(copy=True)[0], 1)
                self.assertEqual((self.root / 'labels.csv').read_bytes(), before)
                self.assertFalse(self.destination.exists())

    def test_all_label_ranges_and_unknown_cells_skip_whole_row(self):
        self.add_pass(1)
        for axis, (low, high) in tools.LABELS.items():
            for value in (str(low - 1), str(high + 1), '{} - Sai'.format(high + 1),
                          '', 'chữ lạ', '3abc', '3.0', '3 - '):
                with self.subTest(axis=axis, value=value):
                    self.write_form([dict(self.records[1], **{axis: value})])
                    code, out, err = self.apply(copy=True)
                    self.assertEqual(code, 0)
                    self.assertIn('nhãn ' + axis, err)
                    self.assertIn('không áp dụng', err)
                    self.assertFalse((self.root / 'labels.csv').exists())
                    self.assertFalse(self.destination.exists())

    def test_range_endpoints_accepted(self):
        self.add_pass(1)
        for index in (0, 1):
            row = self.label(1, tuple(tools.LABELS[key][index] for key in tools.AXES))
            self.write_form([row])
            self.assertEqual(self.apply()[0], 0)
            self.assertEqual(self.saved()[1], row)

    def test_unknown_invalid_pass_result_and_malformed_row(self):
        self.add_pass(1)
        for changes in ({'lượt': '999'}, {'lượt': 'p0001'}, {'lượt': '1.0'},
                        {'lượt': '0'}, {'kết quả': 'đạt'}, {'kết quả': ''}):
            with self.subTest(changes=changes):
                self.write_form([dict(self.records[1], **changes)])
                code, out, err = self.apply(copy=True)
                self.assertEqual(code, 0)
                self.assertIn('CẢNH BÁO', err)
                self.assertFalse((self.root / 'labels.csv').exists())
                self.assertFalse(self.destination.exists())
        self.write_form([self.records[1]])
        with self.form.open('a', encoding='utf-8') as stream:
            stream.write('2,3\n')
        self.assertIn('thiếu/thừa ô', self.apply()[2])
        self.assertEqual(set(self.saved()), {1})

    def test_partial_apply_preserves_invalid_and_unlisted_existing(self):
        for pid in (1, 2, 3):
            self.add_pass(pid)
        self.save_labels()
        self.write_form([dict(self.records[1], c='8'),
                         dict(self.records[2], t='không rõ'), self.label(999)])
        code, out, err = self.apply(copy=True)
        self.assertEqual(code, 0)
        saved = self.saved()
        self.assertEqual(saved[1]['c'], '8')
        self.assertEqual(saved[2], self.records[2])
        self.assertEqual(saved[3], self.records[3])
        self.assertIn('lượt không có', err)
        self.assertEqual(len(list(self.destination.iterdir())), 3)
        self.assertTrue(all(p.name.startswith('p0001_c8_') for p in self.destination.iterdir()))

    def test_duplicate_sheet_pass_keeps_first_row(self):
        self.add_pass(1)
        self.write_form([self.records[1], dict(self.records[1], c='7')])
        self.assertIn('số lượt trùng', self.apply()[2])
        self.assertEqual(self.saved()[1]['c'], '6')

    def test_copy_names_bytes_and_originals_unchanged(self):
        names = self.add_pass(123)
        before = {p.name: p.read_bytes() for p in self.root.iterdir()}
        self.write_form([self.records[123]])
        self.assertEqual(self.apply(copy=True)[0], 0)
        for index, name in enumerate(names):
            target = self.destination / 'p0123_c6_t3_s1_e2_k1_v1_{:04d}.jpg'.format(index)
            self.assertEqual(target.read_bytes(), before[name])
        self.assertTrue(all((self.root / name).read_bytes() == value
                            for name, value in before.items()))

    def test_copy_collision_preflight_preserves_labels_and_copy(self):
        self.add_pass(1)
        self.save_labels()
        before = (self.root / 'labels.csv').read_bytes()
        self.write_form([dict(self.records[1], c='8')])
        self.destination.mkdir()
        target = self.destination / 'p0001_c8_t3_s1_e2_k1_v1_0002.jpg'
        target.write_bytes(b'giu nguyen')
        self.assertEqual(self.apply(copy=True)[0], 1)
        self.assertEqual(list(self.destination.iterdir()), [target])
        self.assertEqual(target.read_bytes(), b'giu nguyen')
        self.assertEqual((self.root / 'labels.csv').read_bytes(), before)

    def test_copy_to_source_or_file_rejected(self):
        self.add_pass(1)
        self.write_form([self.records[1]])
        for destination in (self.root, self.form):
            code, out, err = self.run_cli('apply', str(self.root), str(self.form),
                                          '--copy-to', str(destination))
            self.assertEqual(code, 1)
            self.assertIn('copy-to', err)
            self.assertFalse((self.root / 'labels.csv').exists())

    def test_coverage_all_axes_bad_excluded_empty_pass_included(self):
        self.add_pass(1, count=3)
        self.add_pass(2, count=0)
        self.add_pass(3, bad=True)
        self.save_labels()
        code, out, err = self.run_cli('coverage', str(self.root))
        self.assertEqual(code, 0)
        rows = list(csv.DictReader(io.StringIO(out)))
        table = {(r['label'], int(r['value'])): (int(r['images']), int(r['passes'])) for r in rows}
        for axis, value in zip(tools.AXES, (6, 3, 1, 2, 1, 1)):
            self.assertEqual(table[(axis, value)], (3, 2))
        self.assertEqual(table[('c', 0)], (0, 0))
        self.assertEqual(len(rows), 34)

    def test_missing_labels_coverage_error_split_works(self):
        self.add_pass(1)
        self.assertEqual(self.run_cli('coverage', str(self.root))[0], 1)
        self.assertEqual(self.run_cli('split', str(self.root))[0], 0)

    def test_targets_combination_and_validation(self):
        self.add_pass(1, count=3)
        self.save_labels()
        fields = tools.AXES + ('min_images', 'min_passes')
        blank = dict.fromkeys(tools.AXES, '')
        path = Path(self.temp.name) / 'targets.csv'
        rows = [dict(blank, c='6', t='3', min_images='5', min_passes='2'),
                dict(blank, c='8', min_images='2', min_passes='1'),
                dict(blank, e='2', min_images='3', min_passes='1')]
        self.write_csv(path, fields, rows)
        code, out, err = self.run_cli('coverage', str(self.root), '--targets', str(path))
        self.assertEqual(code, 0)
        self.assertIn('c=6 t=3,2,1', out)
        self.assertIn('c=8,2,1', out)
        for invalid in (dict(blank, min_images='1', min_passes='1'),
                        dict(blank, c='6', min_images='-1', min_passes='1'),
                        dict(blank, c='9', min_images='1', min_passes='1')):
            self.write_csv(path, fields, [invalid])
            code, out, err = self.run_cli('coverage', str(self.root), '--targets', str(path))
            self.assertEqual((code, out), (1, ''))

    def test_split_complete_no_leak_bad_empty_excluded_balanced_repeatable(self):
        pid = 0
        for c, t in ((2, 1), (5, 2), (6, 3), (7, 5)):
            for unused in range(6):
                pid += 1
                self.add_pass(pid, tag=(c, t, 0, 2, 1, 1), count=pid % 3 + 1)
        self.add_pass(99, bad=True)
        self.add_pass(98, count=0)
        by_pass, events, errors = tools.inspect_dataset(self.root)
        self.assertEqual(errors, [])
        good = tools.usable_passes(by_pass, events, self.records)
        parts = tools.split_passes(good, self.records, 1 / 3, 42)
        ids = {part: {int(tools.IMAGE.fullmatch(name)['pass']) for name in files}
               for part, files in parts.items()}
        self.assertFalse(ids['train'] & ids['val'])
        self.assertEqual(ids['train'] | ids['val'], set(range(1, 25)))
        self.assertEqual(set(parts['train'] + parts['val']),
                         {name for names in good.values() for name in names})
        for c, t in ((2, 1), (5, 2), (6, 3), (7, 5)):
            matches = [pid for pid in ids['val'] if
                       (self.records[pid]['c'], self.records[pid]['t']) == (str(c), str(t))]
            self.assertEqual(len(matches), 2)
        self.assertEqual(parts, tools.split_passes(dict(reversed(list(good.items()))),
                                                  self.records, 1 / 3, 42))
        self.assertNotEqual(parts, tools.split_passes(good, self.records, 1 / 3, 43))

    def test_split_singletons_and_invalid_ratios(self):
        self.save()
        self.assertEqual(self.run_cli('split', str(self.root))[0], 0)
        self.add_pass(1)
        self.assertIn('một tập rỗng', self.run_cli('split', str(self.root))[2])
        for ratio in ('0', '1', '-1', 'nan', 'inf'):
            self.assertEqual(self.run_cli('split', str(self.root), '--val-ratio', ratio)[0], 1)
        for pid in range(2, 6):
            self.add_pass(pid, tag=(pid, 0, 0, 2, 1, 1), count=1)
        by_pass, events, errors = tools.inspect_dataset(self.root)
        parts = tools.split_passes(by_pass, self.records, 0.4)
        self.assertEqual(len({name[:5] for name in parts['val']}), 2)
        self.assertFalse(set(parts['train']) & set(parts['val']))
        args = ('split', str(self.root), '--seed', '8', '--val-ratio', '0.3')
        self.assertEqual(self.run_cli(*args), self.run_cli(*args))

    def test_actual_cli_end_to_end_all_five_commands(self):
        self.add_pass(1)
        self.add_pass(2)
        script = str(Path(tools.__file__).resolve())

        def command(*args):
            result = subprocess.run([sys.executable, '-B', script] + list(args),
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=15,
                                    env=dict(os.environ, PYTHONIOENCODING='utf-8'))
            self.assertEqual(result.returncode, 0, result.stderr.decode('utf-8'))
            return result.stdout.decode('utf-8')

        command('check', str(self.root))
        rows = list(csv.DictReader(io.StringIO(command('sheet', str(self.root)))))
        for index, row in enumerate(rows):
            for key, value in zip(tools.SHEET_COLUMNS[1:7], ('6', '3', '1', '2', '1', '1')):
                row[key] = value + ' - Nhãn'
            row['Kết quả'] = 'Tốt' if index == 0 else 'Hỏng'
        self.write_form(rows, tools.SHEET_COLUMNS, bom=True)
        command('apply', str(self.root), str(self.form), '--copy-to', str(self.destination))
        self.assertIn('c,6,3,1', command('coverage', str(self.root)))
        split = command('split', str(self.root))
        self.assertIn('p0001_', split)
        self.assertNotIn('p0002_', split)


if __name__ == '__main__':
    unittest.main()
