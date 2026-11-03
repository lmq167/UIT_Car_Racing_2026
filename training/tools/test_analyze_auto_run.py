"""Kiểm thử analyze_auto_run.py bằng phiên giả (CSV tổng hợp), không cần xe."""

import csv
import json
from pathlib import Path
import tempfile
import unittest

import analyze_auto_run as tool

COLUMNS = ['t_ms', 'frame_id', 'mask_file', 'cmd_cms', 'cmd_deg', 'mask_ms', 'image_ms', 'go', 'stop', 'lane_ref',
           'planner_state', 'image_age_ms', 'mask_age_ms', 'det_age_ms', 'scan_rows', 'e', 'p', 'd',
           'requested_steer_deg', 'requested_speed_cms', 'sent_steer_deg', 'sent_speed_cms', 'reason', 'road_ms',
           'yolo_ms', 'loop_ms', 'dropped', 'csv_dropped', 'camera_fps', 'loop_p95_ms', 'rss_mb', 'free_mb',
           'command_id', 'command_ms', 'mask_id']


def scan(valid):
    return json.dumps([{'y_frac': 0.85 - 0.2 * i, 'left_frac': 0.2, 'right_frac': 0.8, 'center_frac': 0.5,
                        'valid': ok} for i, ok in enumerate(valid)])


def make_rows():
    """5 dòng chờ G, 20 dòng BAM_LAN (góc +6/-6 xen kẽ, hàng 2 hỏng nửa số dòng), 5 dòng dừng SPACE, 1 dòng THOAT."""
    rows, state = [], {'t': 1000}

    def row(**kw):
        base = dict.fromkeys(COLUMNS, '0')
        base.update(t_ms=str(state['t']), frame_id=str(len(rows) + 1), planner_state='CHO_SAN_SANG', reason='CHO_G',
                    scan_rows='[]', sent_speed_cms='0', sent_steer_deg='0', loop_ms='0', road_ms='0',
                    rss_mb='300', free_mb='3000')
        base.update(kw)
        rows.append(base)
        state['t'] += 50
    for _ in range(5):
        row()
    for i in range(20):
        row(planner_state='BAM_LAN', reason='BAM_LAN', go='1' if i == 0 else '0', e='0.1',
            sent_steer_deg=str(6 if i % 2 else -6), sent_speed_cms='8', scan_rows=scan([True, True, i % 2 == 0]),
            loop_ms=str(10 + i), road_ms=str(7 + i % 3))
    for _ in range(5):
        row(planner_state='DUNG', reason='SPACE')
    row(planner_state='DUNG', reason='THOAT', stop='1')
    return rows


class AnalyzeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.session = Path(self.temp.name) / '123_45'
        self.session.mkdir()

    def write(self, rows, columns=COLUMNS):
        with (self.session / 'control_log.csv').open('w', encoding='utf-8', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=columns, extrasaction='ignore')
            writer.writeheader()
            writer.writerows(rows)
        (self.session / 'session.json').write_text(json.dumps({'git_hash': 'abcdef0123456789',
                                                               'engine_name': '/home/x/road.engine'}), encoding='utf-8')

    def run_tool(self):
        fields, rows, meta = tool.load_session(self.session)
        stats = tool.analyze(fields, rows, meta)
        return rows, stats, tool.summarize(self.session, stats)

    def test_counts_and_stats(self):
        self.write(make_rows())
        _, stats, lines = self.run_tool()
        self.assertEqual(stats['rows'], 31)
        self.assertEqual(stats['reasons']['BAM_LAN'], 20)
        self.assertEqual(stats['reasons']['SPACE'], 5)
        self.assertEqual(stats['go_events'], 1)
        # 20 dòng BAM_LAN x 50 ms = 1,0 s trên tổng 1,5 s (dòng cuối không có thời lượng).
        self.assertAlmostEqual(stats['lane_s'], 1.0)
        self.assertAlmostEqual(stats['lane_pct'], 100.0 * 1000 / 1500)
        self.assertEqual(stats['valid_by_row'], [1.0, 1.0, 0.5])
        self.assertEqual(stats['flips'], 19)  # +6/-6 xen kẽ => 19 lần đổi dấu giữa 20 dòng
        self.assertAlmostEqual(stats['flips_per_s'], 19.0)
        self.assertEqual(tool.pct(stats['loop_ms'], 0.5), 19)
        self.assertEqual(tool.pct(stats['loop_ms'], 0.95), 28)
        self.assertEqual(len(stats['stops']), 1)
        self.assertEqual(stats['stops'][0]['reason'], 'SPACE')
        self.assertEqual(stats['stops'][0]['length_ms'], 250)  # 6 dòng tốc độ 0: từ dòng SPACE đầu tới dòng THOAT
        self.assertLessEqual(len(lines), tool.MAX_LINES)
        text = '\n'.join(lines)
        self.assertIn('SPACE: 5', text)
        self.assertIn('abcdef0123', text)
        self.assertIn('road.engine', text)

    def test_missing_columns_reported_not_guessed(self):
        self.write(make_rows(), [c for c in COLUMNS if c not in ('rss_mb', 'sent_steer_deg', 'scan_rows')])
        _, stats, lines = self.run_tool()
        text = '\n'.join(lines)
        self.assertIn('rss_mb', stats['missing'])
        self.assertIn('Cột thiếu: ', text)
        self.assertIn('- rss_mb: thiếu', text)
        self.assertIn('- Góc lái gửi (độ): thiếu', text)
        self.assertIn('- Độ lắc: thiếu', text)
        self.assertEqual(stats['valid_by_row'], [])

    def test_summary_is_capped(self):
        rows = make_rows()
        for i, r in enumerate(rows):  # 31 reason khác nhau + nhiều đoạn dừng
            r['reason'] = 'R%d' % i
        for i in range(0, 20, 2):
            rows[5 + i]['sent_speed_cms'] = '0'
        self.write(rows)
        _, _, lines = self.run_tool()
        self.assertLessEqual(len(lines), tool.MAX_LINES)

    def test_main_writes_summary_and_sheet(self):
        self.write(make_rows())
        try:
            import cv2
            import numpy as np
        except ImportError:
            cv2 = None
        if cv2:
            (self.session / 'images').mkdir()
            (self.session / 'masks').mkdir()
            for fid in (5, 25):
                cv2.imwrite(str(self.session / 'images' / '{}.jpg'.format(fid)), np.full((720, 1280, 3), 120, np.uint8))
                cv2.imwrite(str(self.session / 'masks' / '{}.png'.format(fid)), np.ones((96, 160), np.uint8))
        self.assertEqual(tool.main([str(self.session)]), 0)
        report = self.session / 'report'
        self.assertTrue((report / 'summary.md').is_file())
        self.assertLessEqual(len((report / 'summary.md').read_text(encoding='utf-8').splitlines()), tool.MAX_LINES)
        if cv2:
            sheet = cv2.imread(str(report / 'contact_sheet.png'))
            self.assertIsNotNone(sheet)
            self.assertEqual(sheet.shape[1], 1280)  # hai ô (ảnh + mask) mỗi hàng


if __name__ == '__main__':
    unittest.main()
