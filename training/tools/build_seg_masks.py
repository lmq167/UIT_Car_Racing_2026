"""Đổi đa giác COCO thành mask 0=nền, 1=mặt sa bàn, 2=vạch trắng."""

import argparse
import csv
import json
import re
import sys
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
SIZE = (1280, 720)


def original_name(filename):
    name = re.split(r'[/\\]', filename)[-1]
    match = re.fullmatch(
        r'(car6_p[0-9]{4}_[0-9]{4})(?:_jpg)?(?:\.rf\.[A-Za-z0-9]+)?\.(?:jpg|jpeg|png)', name)
    if not match:
        raise ValueError('Không khôi phục được tên gốc: {}'.format(filename))
    return match[1]


def polygons(annotation, width, height):
    segmentation = annotation.get('segmentation', [])
    if annotation.get('iscrowd', 0) or isinstance(segmentation, dict):
        print('CẢNH BÁO: bỏ qua RLE/iscrowd, annotation {}'.format(
            annotation.get('id', '?')), file=sys.stderr)
        return []
    if not isinstance(segmentation, list):
        raise ValueError('segmentation phải là danh sách đa giác')
    result = []
    for ring in segmentation:
        points = np.asarray(ring, dtype=np.float64)
        if points.ndim != 1 or points.size < 6 or points.size % 2 or not np.isfinite(points).all():
            raise ValueError('Đa giác cần ít nhất 3 cặp tọa độ hữu hạn')
        points = points.reshape(-1, 2) * (SIZE[0] / width, SIZE[1] / height)
        points = np.rint(points)
        if not np.isfinite(points).all() or (np.abs(points) > 2147483647).any():
            raise ValueError('Tọa độ đa giác vượt phạm vi int32')
        result.append(points.astype(np.int32))
    return result


def _make_mask_layers(image, annotations, categories, width, height, sat_max=75, val_min=205,
                      loc_blur=61, loc_diff=30, loc_sat_max=90, close_kernel=5, min_area=25):
    if image.shape != (SIZE[1], SIZE[0], 3):
        raise ValueError('Ảnh gốc phải có kích thước 1280×720, 3 kênh màu')
    if not 0 <= sat_max <= 255 or not 0 <= val_min <= 255:
        raise ValueError('Ngưỡng HSV phải nằm trong 0..255')
    if not isinstance(loc_blur, int) or loc_blur < 3 or loc_blur % 2 != 1:
        raise ValueError('loc_blur phải là số nguyên lẻ ≥ 3')
    if not 0 <= loc_diff <= 255 or not 0 <= loc_sat_max <= 255:
        raise ValueError('Ngưỡng vạch cục bộ phải nằm trong 0..255')
    if not isinstance(close_kernel, int) or close_kernel < 1:
        raise ValueError('close_kernel phải là số nguyên ≥ 1')
    if not isinstance(min_area, int) or min_area < 0:
        raise ValueError('min_area phải là số nguyên ≥ 0')
    if not np.isfinite([width, height]).all() or width <= 0 or height <= 0:
        raise ValueError('width/height COCO phải hữu hạn và lớn hơn 0')
    unknown = set(categories.values()) - {'mat_sa_ban', 'vat', 'khong_vach'}
    if unknown:
        raise ValueError('Tên lớp không hỗ trợ: {}'.format(', '.join(sorted(unknown))))
    layers = defaultdict(list)
    for annotation in annotations:
        category = annotation['category_id']
        if category not in categories:
            raise ValueError('category_id không có trong categories: {}'.format(category))
        layers[categories[category]].extend(polygons(annotation, width, height))
    mask = np.zeros((SIZE[1], SIZE[0]), dtype=np.uint8)
    for ring in layers['mat_sa_ban']:
        cv2.fillPoly(mask, [ring], 1)
    vat = np.zeros_like(mask)
    for ring in layers['vat']:
        cv2.fillPoly(vat, [ring], 1)
    mask[vat != 0] = 0
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    sat, val = hsv[:, :, 1], hsv[:, :, 2]
    white_abs = (sat <= sat_max) & (val >= val_min)
    bg = cv2.medianBlur(val, loc_blur)
    # Chuyển sang số có dấu để pixel tối hơn nền không bị tràn uint8 thành vạch.
    white_loc = (val.astype(np.int16) - bg.astype(np.int16) >= loc_diff) & (sat <= loc_sat_max)
    road = mask == 1
    vach = ((white_abs | white_loc) & road).astype(np.uint8)
    vach = cv2.morphologyEx(vach, cv2.MORPH_CLOSE,
                            np.ones((close_kernel, close_kernel), dtype=np.uint8))
    vach &= road.astype(np.uint8)
    khong_vach = np.zeros_like(mask)
    for ring in layers['khong_vach']:
        cv2.fillPoly(khong_vach, [ring], 1)
    vach[khong_vach != 0] = 0
    if min_area:
        _, labels, stats, _ = cv2.connectedComponentsWithStats(vach, connectivity=8)
        keep = stats[:, cv2.CC_STAT_AREA] >= min_area
        keep[0] = False
        vach = keep[labels]
    mask[vach != 0] = 2
    return mask, vat, khong_vach


def make_mask(image, annotations, categories, width, height, sat_max=75, val_min=205,
              loc_blur=61, loc_diff=30, loc_sat_max=90, close_kernel=5, min_area=25):
    return _make_mask_layers(image, annotations, categories, width, height, sat_max, val_min,
                             loc_blur, loc_diff, loc_sat_max, close_kernel, min_area)[0]


def load_jobs(paths):
    jobs, names = [], set()
    for path in paths:
        path = path / '_annotations.coco.json' if path.is_dir() else path
        with path.open(encoding='utf-8-sig') as stream:
            data = json.load(stream)
        categories = {c['id']: c['name'] for c in data['categories']}
        if len(categories) != len(data['categories']):
            raise ValueError('{}: id lớp trùng'.format(path))
        unknown = set(categories.values()) - {'mat_sa_ban', 'vat', 'khong_vach'}
        if unknown:
            raise ValueError('Tên lớp không hỗ trợ: {}'.format(', '.join(sorted(unknown))))
        images = {img['id']: img for img in data['images']}
        if len(images) != len(data['images']):
            raise ValueError('{}: id ảnh trùng'.format(path))
        annotations = defaultdict(list)
        for ann in data['annotations']:
            if ann['image_id'] not in images or ann['category_id'] not in categories:
                raise ValueError('{}: annotation tham chiếu ảnh/lớp không có'.format(path))
            annotations[ann['image_id']].append(ann)
        for img in images.values():
            name = original_name(img['file_name'])
            if name in names:
                raise ValueError('Trùng tên gốc giữa các ảnh COCO: {}'.format(name))
            names.add(name)
            jobs.append((name, img, annotations[img['id']], categories))
    return jobs


def write_image(path, image):
    if path.is_symlink():
        raise ValueError('Không ghi qua liên kết: {}'.format(path))
    if not cv2.imwrite(str(path), image):
        raise OSError('Không ghi được ảnh: {}'.format(path))


def qa_row(name, mask, min_ratio, max_ratio, max_comp):
    mat_px, vach_px = int(np.count_nonzero(mask == 1)), int(np.count_nonzero(mask == 2))
    ratio = vach_px / (mat_px + vach_px) if mat_px + vach_px else 0
    count, _, stats, _ = cv2.connectedComponentsWithStats(
        (mask == 2).astype(np.uint8), connectivity=8)
    largest = int(stats[1:, cv2.CC_STAT_AREA].max()) if count > 1 else 0
    flags = []
    if ratio < min_ratio:
        flags.append('ratio_thap')
    if ratio > max_ratio:
        flags.append('ratio_cao')
    if count - 1 > max_comp:
        flags.append('nhieu_manh')
    return (name, mat_px, vach_px, ratio, count - 1, largest, ';'.join(flags))


def qa_tile(image, mask, vat, khong_vach, name, flag):
    overlay = image.copy()
    for selected, color in ((mask == 1, (0, 255, 0)), (mask == 2, (0, 255, 255)),
                            (vat != 0, (0, 0, 255)), (khong_vach != 0, (255, 0, 255))):
        overlay[selected] = (image[selected] * 0.6 + np.array(color) * 0.4).astype(np.uint8)
    tile = cv2.resize(overlay, (320, 180), interpolation=cv2.INTER_AREA)
    # Chia các cờ thành hai dòng để chữ không tràn khỏi ô 320 pixel.
    lines = [name] + ([flag.replace(';', ';\n', 1)] if flag else [])
    for index, line in enumerate('\n'.join(lines).splitlines()):
        position = (5, 17 + index * 17)
        cv2.putText(tile, line, position, cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3,
                    cv2.LINE_AA)
        cv2.putText(tile, line, position, cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1,
                    cv2.LINE_AA)
    return tile


def write_qa(directory, entries):
    path = directory / 'qa.csv'
    if path.is_symlink():
        raise ValueError('Không ghi qua liên kết: {}'.format(path))
    with path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(('file', 'mat_px', 'vach_px', 'vach_ratio', 'n_comp',
                         'largest_comp_px', 'flag'))
        writer.writerows(row for row, _ in entries)
    ordered = sorted(entries, key=lambda item: (not bool(item[0][-1]), item[0][0]))
    for start in range(0, len(ordered), 12):
        sheet = np.zeros((540, 1280, 3), dtype=np.uint8)
        for index, (_, tile) in enumerate(ordered[start:start + 12]):
            y, x = (index // 4) * 180, (index % 4) * 320
            sheet[y:y + 180, x:x + 320] = tile
        write_image(directory / 'qa_sheet_{:02d}.jpg'.format(start // 12 + 1), sheet)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--coco', type=Path, nargs='+', required=True,
                        help='JSON hoặc thư mục COCO; nhận nhiều đường dẫn train/valid/test')
    parser.add_argument('--out', type=Path, default=ROOT / 'data/round3/seg_masks_v0',
                        help='Thư mục mask PNG')
    parser.add_argument('--images', type=Path, default=ROOT / 'data/round3/data_clean_car6/images',
                        help='Thư mục ảnh JPG gốc')
    parser.add_argument('--sat-max', type=int, default=75, help='Trần độ bão hòa của vạch (0..255)')
    parser.add_argument('--val-min', type=int, default=205, help='Sàn độ sáng của vạch (0..255)')
    parser.add_argument('--loc-blur', type=int, default=61, help='Cỡ lọc trung vị nền sáng (số lẻ ≥ 3)')
    parser.add_argument('--loc-diff', type=int, default=30, help='Chênh sáng tối thiểu so với nền (0..255)')
    parser.add_argument('--loc-sat-max', type=int, default=90, help='Trần bão hòa vạch cục bộ (0..255)')
    parser.add_argument('--close-kernel', type=int, default=5, help='Cỡ nhân lấp lỗ vạch (≥ 1)')
    parser.add_argument('--preview', type=Path, help='Thư mục ảnh phủ màu để soát nhãn')
    parser.add_argument('--min-area', type=int, default=25, help='Diện tích vạch tối thiểu (pixel; 0 = không lọc)')
    parser.add_argument('--qa', type=Path, help='Thư mục CSV và bảng ảnh soát nhãn')
    parser.add_argument('--qa-min-ratio', type=float, default=0.005, help='Tỉ lệ vạch tối thiểu (0..1)')
    parser.add_argument('--qa-max-ratio', type=float, default=0.15, help='Tỉ lệ vạch tối đa (0..1)')
    parser.add_argument('--qa-max-comp', type=int, default=60, help='Số mảnh vạch tối đa (≥ 0)')
    args = parser.parse_args(argv)
    try:
        if not 0 <= args.sat_max <= 255 or not 0 <= args.val_min <= 255:
            raise ValueError('Ngưỡng HSV phải nằm trong 0..255')
        if args.loc_blur < 3 or args.loc_blur % 2 != 1:
            raise ValueError('loc_blur phải là số nguyên lẻ ≥ 3')
        if not 0 <= args.loc_diff <= 255 or not 0 <= args.loc_sat_max <= 255:
            raise ValueError('Ngưỡng vạch cục bộ phải nằm trong 0..255')
        if args.close_kernel < 1:
            raise ValueError('close_kernel phải là số nguyên ≥ 1')
        if args.min_area < 0:
            raise ValueError('min_area phải là số nguyên ≥ 0')
        if not 0 <= args.qa_min_ratio <= args.qa_max_ratio <= 1:
            raise ValueError('Cần 0 ≤ qa_min_ratio ≤ qa_max_ratio ≤ 1')
        if args.qa_max_comp < 0:
            raise ValueError('qa_max_comp phải là số nguyên ≥ 0')
        destinations = [args.out] + ([args.preview] if args.preview else [])
        if args.qa:
            for protected in [args.images] + destinations:
                if protected.resolve() in (args.qa.resolve(),) + tuple(args.qa.resolve().parents):
                    raise ValueError('--qa không được trùng/nằm trong thư mục ảnh gốc, --out, --preview')
            for path in (args.qa,) + tuple(args.qa.parents):
                if path.is_symlink():
                    raise ValueError('Không ghi qua liên kết: {}'.format(path))
            destinations.append(args.qa)
        for destination in destinations:
            if args.images.resolve() in (destination.resolve(),) + tuple(destination.resolve().parents):
                raise ValueError('Thư mục đầu ra không được nằm trong thư mục ảnh gốc')
        if args.preview and args.preview.resolve() == args.out.resolve():
            raise ValueError('--preview và --out phải khác thư mục')
        jobs = load_jobs(args.coco)
        for destination in destinations:
            destination.mkdir(parents=True, exist_ok=True)
        ratios = np.zeros(3, dtype=np.float64)
        qa_entries = []
        processed = missing = 0
        for name, metadata, annotations, categories in jobs:
            source = args.images / (name + '.jpg')
            if not source.is_file():
                missing += 1
                print('CẢNH BÁO: thiếu ảnh gốc, bỏ qua {}'.format(source), file=sys.stderr)
                continue
            image = cv2.imread(str(source), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError('Không đọc được ảnh gốc: {}'.format(source))
            mask, vat, khong_vach = _make_mask_layers(
                image, annotations, categories, metadata['width'], metadata['height'],
                args.sat_max, args.val_min, args.loc_blur, args.loc_diff,
                args.loc_sat_max, args.close_kernel, args.min_area)
            write_image(args.out / (name + '.png'), mask)
            if args.preview:
                overlay = image.copy()
                for value, color in ((1, (0, 255, 0)), (2, (0, 0, 255))):
                    selected = mask == value
                    overlay[selected] = (image[selected] * 0.5 + np.array(color) * 0.5).astype(np.uint8)
                write_image(args.preview / (name + '.png'), overlay)
            if args.qa:
                row = qa_row(name, mask, args.qa_min_ratio, args.qa_max_ratio, args.qa_max_comp)
                qa_entries.append((row, qa_tile(image, mask, vat, khong_vach, name, row[-1])))
            ratios += np.bincount(mask.ravel(), minlength=3) / mask.size
            processed += 1
        print('Số ảnh COCO: {}; đã tạo mask: {}; thiếu file gốc: {}'.format(
            len(jobs), processed, missing))
        if processed:
            print('Tỉ lệ pixel trung bình: nền={:.6f}, mặt sa bàn={:.6f}, vạch={:.6f}'.format(
                *(ratios / processed)))
        else:
            print('Tỉ lệ pixel trung bình: chưa có ảnh để tính')
        if args.qa:
            write_qa(args.qa, qa_entries)
            print('Số ảnh có cờ QA: {}'.format(sum(bool(row[-1]) for row, _ in qa_entries)))
        return 0
    except (OSError, ValueError, KeyError, TypeError, cv2.error) as exc:
        print('LỖI: {}'.format(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, 'reconfigure'):
            stream.reconfigure(encoding='utf-8')
    sys.exit(main())
