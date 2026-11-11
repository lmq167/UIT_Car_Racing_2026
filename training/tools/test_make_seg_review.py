import contextlib
import io
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

import make_seg_review as review


class ReviewTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.masks, self.images = self.root / 'masks', self.root / 'train'
        self.masks.mkdir()
        self.images.mkdir()
        (self.root / 'phieu.csv').write_text(
            'Lượt,Chặng (c),Mục tiêu (t)\n89,3,10\n77,7,11\n', encoding='utf-8-sig')

    def add(self, name, export, dim_streak=False):
        image = np.full((720, 1280, 3), 100, dtype=np.uint8)
        mask = np.zeros((720, 1280), dtype=np.uint8)
        mask[100:700, 100:1100] = 1
        mask[300:600, 500:507] = 2
        image[300:600, 500:507] = 200
        if dim_streak:
            mask[450:460, 200:400] = 2
            image[450:460, 200:400] = 110
        cv2.imwrite(str(self.masks / (name + '.png')), mask)
        cv2.imwrite(str(self.images / export), image)

    def run_tool(self, *extra):
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = review.main(['--masks', str(self.masks), '--images-dir', str(self.images),
                                '--phieu', str(self.root / 'phieu.csv'), '--out', str(self.root / 'rv'), *extra])
        return code, out.getvalue(), err.getvalue()

    def test_groups_sheets_and_ranking(self):
        self.add('p0089_0001', 'p0089_0001_jpg.rf.abc.jpg')
        self.add('car6_p0089_0002', 'car6_p0089_0002_jpg.rf.abd.jpg', dim_streak=True)
        self.add('car6_p0077_0003', 'car6_p0077_0003.jpg')
        self.add('h9342_10', 'h9342_10_jpg.rf.xyz.jpg')
        self.add('p0008_0044', 'p0008_0044_jpg.rf.q.jpg')
        code, _, error = self.run_tool()
        self.assertEqual((code, error), (0, ''))
        rv = self.root / 'rv'
        for sheet in ('c3_cau_01.jpg', 'c7_ham_01.jpg', 'h9342_khong_phieu_01.jpg', 'khong_co_phieu_01.jpg'):
            self.assertTrue((rv / sheet).is_file(), sheet)
        rows = (rv / 'review.csv').read_text(encoding='utf-8-sig').splitlines()
        self.assertEqual(len(rows), 6)
        self.assertTrue(rows[1].startswith('car6_p0089_0002,c3_cau'))   # ảnh có vệt yếu xếp đầu
        sheet = cv2.imread(str(rv / 'c3_cau_01.jpg'))
        self.assertEqual(sheet.shape, (1080, 1920, 3))

    def test_invalid_arguments_and_missing_images(self):
        self.add('p0089_0001', 'p0089_0001_jpg.rf.abc.jpg')
        self.assertEqual(self.run_tool('--cols', '0')[0], 1)
        (self.images / 'p0089_0001_jpg.rf.abc.jpg').unlink()
        code, _, error = self.run_tool()
        self.assertEqual(code, 1)
        self.assertIn('LỖI', error)


if __name__ == '__main__':
    unittest.main()
