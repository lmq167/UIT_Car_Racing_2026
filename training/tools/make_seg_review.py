#!/usr/bin/env python3
"""Tạo thư mục preview gọn nhẹ để duyệt mặt nạ line trước khi dùng huấn luyện.

Đầu vào: thư mục mask (kết quả build_seg_masks.py), các thư mục xuất Roboflow (để lấy ảnh gốc),
bảng phiếu lượt (để gom theo chặng). Đầu ra: các tờ JPEG (lưới ảnh gốc + overlay) gom theo nhóm,
sắp ảnh nghi ngờ nhất lên đầu, và review.csv xếp hạng ảnh nghi ngờ.

Màu: ĐỎ = line giữ lại; XANH LƠ = thành phần yếu (vùng giữa/gần, sáng hơn mặt đường chưa tới --weak-ratio
lần) — ứng viên đốm nhiễu cần vẽ glare hoặc để bộ lọc --weak-ratio của build_seg_masks.py bỏ.
Chỉ đọc dữ liệu, không sửa mask/ảnh gốc.
"""
import argparse
import csv
import re
import sys
from pathlib import Path

import cv2
import numpy as np

SIZE = (1280, 720)
TILE = (640, 360)


def export_name(filename):
    match = re.fullmatch(r'([A-Za-z0-9_]+?)(?:_(?:jpg|jpeg|png))?(?:\.rf\.[A-Za-z0-9]+)?\.(?:jpg|jpeg|png)',
                         Path(filename).name)
    return match[1] if match else None


def load_phieu(path):
    table = {}
    if path is None:
        return table
    with Path(path).open(encoding='utf-8-sig', newline='') as stream:
        for row in csv.DictReader(stream):
            try:
                table[int(row['Lượt'])] = (int(row['Chặng (c)']), int(row['Mục tiêu (t)']))
            except (KeyError, ValueError):
                continue
    return table


def group_of(name, phieu):
    match = re.match(r'(?:car6_)?p(\d{4})_\d{4}$', name)
    if not match:
        return 'h9342_khong_phieu', None
    key = int(match[1])
    if key not in phieu:
        return 'khong_co_phieu', None
    chang, muctieu = phieu[key]
    label = {3: 'c3_cau', 7: 'c7_ham', 4: 'c4_vongxuyen', 8: 'c8_baido'}.get(chang, 'c{}_khac'.format(chang))
    return label, (chang, muctieu)


def analyse(image, mask, weak_ratio, weak_zone):
    """Trả về (các số đo, mặt nạ thành phần yếu, mặt nạ nhỏ)."""
    val = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)[:, :, 2]
    surface = mask >= 1
    line = (mask == 2).astype(np.uint8)
    weak = np.zeros(mask.shape, dtype=bool)
    small_px = weak_px = weak_n = 0
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(line, connectivity=8)
    if surface.any():
        rows = np.flatnonzero(surface.sum(axis=1) >= 20)
        top = int(rows[0]) if rows.size else 0
        track_v = max(float(val[surface].mean()), 1.0)
        for index in range(1, count):
            area = int(stats[index, cv2.CC_STAT_AREA])
            if area < 150:
                small_px += area
            yf = (centroids[index][1] - top) / max(SIZE[1] - top, 1)
            if yf >= weak_zone and np.percentile(val[labels == index], 90) / track_v < weak_ratio:
                weak[labels == index] = True
                weak_px += area
                weak_n += 1
    return {'line_px': int(line.sum()), 'n_comp': count - 1, 'small_px': small_px,
            'weak_n': weak_n, 'weak_px': weak_px}, weak


def tile(image, mask, weak, caption):
    over = image.copy()
    line = mask == 2
    over[line] = (0, 0, 255)
    over[weak] = (255, 255, 0)
    out = cv2.addWeighted(over, 0.85, image, 0.15, 0)
    out = cv2.resize(out, TILE, interpolation=cv2.INTER_AREA)
    cv2.rectangle(out, (0, 0), (TILE[0], 22), (0, 0, 0), -1)
    cv2.putText(out, caption, (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--masks', type=Path, required=True, help='Thư mục mask PNG (0/1/2)')
    parser.add_argument('--images-dir', type=Path, nargs='+', required=True,
                        help='Thư mục ảnh gốc (train valid test của bản xuất Roboflow)')
    parser.add_argument('--phieu', type=Path, help='phieu_luot_car6_*.csv để gom theo chặng')
    parser.add_argument('--out', type=Path, required=True, help='Thư mục đầu ra (được tạo mới/ghi đè file cùng tên)')
    parser.add_argument('--cols', type=int, default=3)
    parser.add_argument('--rows', type=int, default=3)
    parser.add_argument('--weak-ratio', type=float, default=1.2)
    parser.add_argument('--weak-zone', type=float, default=0.3)
    parser.add_argument('--quality', type=int, default=82)
    args = parser.parse_args(argv)
    if args.cols < 1 or args.rows < 1 or not 0 < args.weak_ratio or not 0 <= args.weak_zone <= 1:
        print('LỖI: tham số không hợp lệ', file=sys.stderr)
        return 1
    sources = {}
    for directory in args.images_dir:
        for path in directory.iterdir():
            if path.suffix.lower() in ('.jpg', '.jpeg', '.png'):
                name = export_name(path.name)
                if name:
                    sources.setdefault(name, path)
    phieu = load_phieu(args.phieu)
    args.out.mkdir(parents=True, exist_ok=True)
    entries = []
    for mask_path in sorted(args.masks.glob('*.png')):
        name = mask_path.stem
        key = name[5:] if name.startswith('car6_') else name
        source = sources.get(name) or sources.get(key)
        if source is None:
            print('CẢNH BÁO: không thấy ảnh gốc cho {}'.format(name), file=sys.stderr)
            continue
        image = cv2.imread(str(source), cv2.IMREAD_COLOR)
        mask = cv2.imread(str(mask_path), cv2.IMREAD_UNCHANGED)
        if image is None or mask is None or mask.shape != (SIZE[1], SIZE[0]):
            print('CẢNH BÁO: bỏ qua {} (không đọc được hoặc sai cỡ)'.format(name), file=sys.stderr)
            continue
        image = cv2.resize(image, SIZE) if image.shape[:2] != (SIZE[1], SIZE[0]) else image
        stats, weak = analyse(image, mask, args.weak_ratio, args.weak_zone)
        group, meta = group_of(name, phieu)
        stats.update(name=name, group=group, chang=meta[0] if meta else '', muctieu=meta[1] if meta else '',
                     suspect=stats['weak_px'] + stats['small_px'])
        entries.append((stats, tile(image, mask, weak, '{} w={} s={}'.format(
            name, stats['weak_px'], stats['small_px']))))
    if not entries:
        print('LỖI: không có ảnh nào để duyệt', file=sys.stderr)
        return 1
    per_sheet = args.cols * args.rows
    groups = {}
    for stats, picture in entries:
        groups.setdefault(stats['group'], []).append((stats, picture))
    for group, items in sorted(groups.items()):
        items.sort(key=lambda item: -item[0]['suspect'])
        for number, start in enumerate(range(0, len(items), per_sheet), 1):
            chunk = [picture for _, picture in items[start:start + per_sheet]]
            blank = np.zeros_like(chunk[0])
            chunk += [blank] * (per_sheet - len(chunk))
            sheet = np.vstack([np.hstack(chunk[r * args.cols:(r + 1) * args.cols]) for r in range(args.rows)])
            target = args.out / '{}_{:02d}.jpg'.format(group, number)
            if not cv2.imwrite(str(target), sheet, [cv2.IMWRITE_JPEG_QUALITY, args.quality]):
                print('LỖI: không ghi được {}'.format(target), file=sys.stderr)
                return 1
            for stats, _ in items[start:start + per_sheet]:
                stats['sheet'] = target.name
    columns = ('name', 'group', 'chang', 'muctieu', 'sheet', 'suspect', 'weak_px', 'weak_n', 'small_px',
               'line_px', 'n_comp')
    with (args.out / 'review.csv').open('w', encoding='utf-8-sig', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for stats in sorted((s for s, _ in entries), key=lambda s: -s['suspect']):
            writer.writerow({key: stats.get(key, '') for key in columns})
    print('Đã tạo {} ảnh trong {} nhóm -> {}'.format(len(entries), len(groups), args.out))
    for group, items in sorted(groups.items()):
        print('  {}: {} ảnh'.format(group, len(items)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
