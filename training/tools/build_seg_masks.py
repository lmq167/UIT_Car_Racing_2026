"""Đổi đa giác COCO thành mask 0=background, 1=track_surface, 2=line (đường kẻ)."""

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
BACKGROUND = 'background'
TRACK_SURFACE = 'track_surface'
LINE = 'line'
OBJECT = 'object'
GLARE = 'glare'
QA_STRIP = 22
QA_FONT = cv2.FONT_HERSHEY_SIMPLEX
QA_SCALE = 0.45


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
                      loc_blur=61, loc_diff=30, loc_sat_max=90, close_kernel=5, min_area=25,
                      line_method='tophat_sharp', tophat_kernel=101, tophat_min=30,
                      tophat_sat_max=130, object_dilate=3, sharp_min=0.85):
    if image.shape != (SIZE[1], SIZE[0], 3):
        raise ValueError('Ảnh gốc phải có kích thước 1280×720, 3 kênh màu')
    if not 0 <= sat_max <= 255 or not 0 <= val_min <= 255:
        raise ValueError('Ngưỡng HSV phải nằm trong 0..255')
    if not isinstance(loc_blur, int) or loc_blur < 3 or loc_blur % 2 != 1:
        raise ValueError('loc_blur phải là số nguyên lẻ ≥ 3')
    if not 0 <= loc_diff <= 255 or not 0 <= loc_sat_max <= 255:
        raise ValueError('Ngưỡng {} cục bộ phải nằm trong 0..255'.format(LINE))
    if not isinstance(close_kernel, int) or close_kernel < 1:
        raise ValueError('close_kernel phải là số nguyên ≥ 1')
    if not isinstance(min_area, int) or min_area < 0:
        raise ValueError('min_area phải là số nguyên ≥ 0')
    if line_method not in ('median', 'tophat_sharp'):
        raise ValueError('line_method phải là median hoặc tophat_sharp')
    if not isinstance(tophat_kernel, int) or tophat_kernel < 3 or tophat_kernel % 2 != 1:
        raise ValueError('tophat_kernel phải là số nguyên lẻ ≥ 3')
    if not 0 <= tophat_min <= 255 or not 0 <= tophat_sat_max <= 255:
        raise ValueError('Ngưỡng tophat phải nằm trong 0..255')
    if not isinstance(object_dilate, int) or object_dilate < 0:
        raise ValueError('object_dilate phải là số nguyên ≥ 0')
    if not np.isfinite(sharp_min) or sharp_min < 0:
        raise ValueError('sharp_min phải hữu hạn và ≥ 0')
    if not np.isfinite([width, height]).all() or width <= 0 or height <= 0:
        raise ValueError('width/height COCO phải hữu hạn và lớn hơn 0')
    unknown = set(categories.values()) - {TRACK_SURFACE, OBJECT, GLARE}
    if unknown:
        raise ValueError('Tên lớp không hỗ trợ: {}'.format(', '.join(sorted(unknown))))
    layers = defaultdict(list)
    for annotation in annotations:
        category = annotation['category_id']
        if category not in categories:
            raise ValueError('category_id không có trong categories: {}'.format(category))
        layers[categories[category]].extend(polygons(annotation, width, height))
    mask = np.zeros((SIZE[1], SIZE[0]), dtype=np.uint8)
    for ring in layers[TRACK_SURFACE]:
        cv2.fillPoly(mask, [ring], 1)
    object_mask = np.zeros_like(mask)
    for ring in layers[OBJECT]:
        cv2.fillPoly(object_mask, [ring], 1)
    mask[object_mask != 0] = 0
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    sat, val = hsv[:, :, 1], hsv[:, :, 2]
    white_abs = (sat <= sat_max) & (val >= val_min)
    track = mask == 1
    if line_method == 'median':
        bg = cv2.medianBlur(val, loc_blur)
        # Chuyển sang số có dấu để pixel tối hơn nền không bị tràn uint8 thành line.
        white_loc = (val.astype(np.int16) - bg.astype(np.int16) >= loc_diff) & (sat <= loc_sat_max)
        allowed = track
    else:
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (tophat_kernel, tophat_kernel))
        tophat = val - cv2.morphologyEx(val, cv2.MORPH_OPEN, kernel)
        white_loc = (tophat >= tophat_min) & (sat <= tophat_sat_max)
        # Giãn object bằng nhân chữ nhật, bán kính object_dilate pixel theo mỗi trục.
        diameter = 2 * object_dilate + 1
        expanded = cv2.dilate(object_mask, np.ones((diameter, diameter), dtype=np.uint8))
        allowed = track & (expanded == 0)
    line = ((white_abs | white_loc) & allowed).astype(np.uint8)
    line = cv2.morphologyEx(line, cv2.MORPH_CLOSE,
                            np.ones((close_kernel, close_kernel), dtype=np.uint8))
    line &= allowed.astype(np.uint8)
    sharp_rejected = np.zeros_like(mask)
    if line_method == 'tophat_sharp':
        count, labels, stats, _ = cv2.connectedComponentsWithStats(line, connectivity=8)
        area = stats[:, cv2.CC_STAT_AREA]
        eligible = area >= min_area
        eligible[0] = False
        smooth = cv2.GaussianBlur(val.astype(np.float32), (3, 3), 0)
        gradient = cv2.magnitude(cv2.Sobel(smooth, cv2.CV_32F, 1, 0, ksize=3),
                                 cv2.Sobel(smooth, cv2.CV_32F, 0, 1, ksize=3))
        # Viền trong 1 px; mọi thành phần dùng chung một phép co và các tổng theo nhãn.
        border = (line != 0) & (cv2.erode(line, np.ones((3, 3), dtype=np.uint8)) == 0)
        border_count = np.bincount(labels[border], minlength=count)
        border_sum = np.bincount(labels[border], weights=gradient[border], minlength=count)
        tophat_sum = np.bincount(labels.ravel(), weights=tophat.ravel(), minlength=count)
        sharp = (border_sum / np.maximum(border_count, 1)
                 / np.maximum(tophat_sum / np.maximum(area, 1), 1))
        keep = eligible & (sharp >= sharp_min)
        sharp_rejected = (eligible & ~keep)[labels].astype(np.uint8)
        line = keep[labels].astype(np.uint8)
    glare = np.zeros_like(mask)
    for ring in layers[GLARE]:
        cv2.fillPoly(glare, [ring], 1)
    line[glare != 0] = 0
    if line_method == 'median' and min_area:
        _, labels, stats, _ = cv2.connectedComponentsWithStats(line, connectivity=8)
        keep = stats[:, cv2.CC_STAT_AREA] >= min_area
        keep[0] = False
        line = keep[labels]
    mask[line != 0] = 2
    return mask, object_mask, glare, sharp_rejected


def make_mask(image, annotations, categories, width, height, sat_max=75, val_min=205,
              loc_blur=61, loc_diff=30, loc_sat_max=90, close_kernel=5, min_area=25,
              line_method='tophat_sharp', tophat_kernel=101, tophat_min=30,
              tophat_sat_max=130, object_dilate=3, sharp_min=0.85):
    return _make_mask_layers(image, annotations, categories, width, height, sat_max, val_min,
                             loc_blur, loc_diff, loc_sat_max, close_kernel, min_area,
                             line_method, tophat_kernel, tophat_min, tophat_sat_max,
                             object_dilate, sharp_min)[0]


def load_jobs(paths):
    jobs, names = [], set()
    for path in paths:
        path = path / '_annotations.coco.json' if path.is_dir() else path
        with path.open(encoding='utf-8-sig') as stream:
            data = json.load(stream)
        categories = {c['id']: c['name'] for c in data['categories']}
        if len(categories) != len(data['categories']):
            raise ValueError('{}: id lớp trùng'.format(path))
        unknown = set(categories.values()) - {TRACK_SURFACE, OBJECT, GLARE}
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
    track_px, line_px = int(np.count_nonzero(mask == 1)), int(np.count_nonzero(mask == 2))
    line_ratio = line_px / (track_px + line_px) if track_px + line_px else 0
    count, _, stats, _ = cv2.connectedComponentsWithStats(
        (mask == 2).astype(np.uint8), connectivity=8)
    largest = int(stats[1:, cv2.CC_STAT_AREA].max()) if count > 1 else 0
    flags = []
    if line_ratio < min_ratio:
        flags.append('ratio_thap')
    if line_ratio > max_ratio:
        flags.append('ratio_cao')
    if count - 1 > max_comp:
        flags.append('nhieu_manh')
    return (name, track_px, line_px, line_ratio, count - 1, largest, ';'.join(flags))


def qa_tile(image, mask, object_mask, glare, name, flag, sharp_rejected=None):
    overlay = image.copy()
    for selected, color in ((mask == 1, (0, 255, 0)), (mask == 2, (0, 255, 255)),
                            (object_mask != 0, (0, 0, 255)), (glare != 0, (255, 0, 255))):
        overlay[selected] = (image[selected] * 0.6 + np.array(color) * 0.4).astype(np.uint8)
    if sharp_rejected is not None:
        selected = sharp_rejected != 0
        overlay[selected] = (image[selected] * 0.6 + np.array((0, 165, 255)) * 0.4).astype(np.uint8)
    tile = cv2.resize(overlay, (320, 180), interpolation=cv2.INTER_AREA)
    # Dải nền đen đặc phía trên ô: một dòng chữ trắng cỡ cố định, cờ (nếu có) ở cuối dòng, màu vàng.
    tile[:QA_STRIP] = 0
    match = re.search(r'car6_p\d{4}_\d{4}', name)
    label = match.group(0) if match else name[:15]
    flag_width = cv2.getTextSize(flag, QA_FONT, QA_SCALE, 1)[0][0] if flag else 0
    while flag and flag_width + cv2.getTextSize(label, QA_FONT, QA_SCALE, 1)[0][0] > 312 - 8:
        flag = flag[:-1]
        flag_width = cv2.getTextSize(flag, QA_FONT, QA_SCALE, 1)[0][0] if flag else 0
    cv2.putText(tile, label, (4, 15), QA_FONT, QA_SCALE, (255, 255, 255), 1, cv2.LINE_AA)
    if flag:
        cv2.putText(tile, flag, (316 - flag_width, 15), QA_FONT, QA_SCALE, (0, 255, 255), 1,
                    cv2.LINE_AA)
    return tile


def write_qa(directory, entries):
    path = directory / 'qa.csv'
    if path.is_symlink():
        raise ValueError('Không ghi qua liên kết: {}'.format(path))
    with path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(('file', 'track_px', 'line_px', 'line_ratio', 'n_comp',
                         'largest_comp_px', 'flag', 'line_method', 'n_sharp_rejected',
                         'sharp_rejected_px'))
        writer.writerows(row for row, _ in entries)
    ordered = sorted(entries, key=lambda item: (not bool(item[0][6]), item[0][0]))
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
    parser.add_argument('--sat-max', type=int, default=75, help='Trần độ bão hòa của {} (0..255)'.format(LINE))
    parser.add_argument('--val-min', type=int, default=205, help='Sàn độ sáng của {} (0..255)'.format(LINE))
    parser.add_argument('--loc-blur', type=int, default=61, help='Cỡ lọc trung vị nền sáng (số lẻ ≥ 3)')
    parser.add_argument('--loc-diff', type=int, default=30, help='Chênh sáng tối thiểu so với nền (0..255)')
    parser.add_argument('--loc-sat-max', type=int, default=90, help='Trần bão hòa {} cục bộ (0..255)'.format(LINE))
    parser.add_argument('--close-kernel', type=int, default=5, help='Cỡ nhân lấp lỗ {} (≥ 1)'.format(LINE))
    parser.add_argument('--preview', type=Path, help='Thư mục ảnh phủ màu để soát nhãn')
    parser.add_argument('--min-area', type=int, default=25, help='Diện tích {} tối thiểu (pixel; 0 = không lọc)'.format(LINE))
    parser.add_argument('--line-method', choices=('median', 'tophat_sharp'), default='tophat_sharp',
                        help='Cách tìm đường kẻ (mặc định: tophat_sharp)')
    parser.add_argument('--tophat-kernel', type=int, default=101, help='Cỡ nhân chữ nhật tách nền (số lẻ ≥ 3)')
    parser.add_argument('--tophat-min', type=int, default=30, help='Chênh sáng tối thiểu sau tách nền (0..255)')
    parser.add_argument('--tophat-sat-max', type=int, default=130, help='Trần bão hòa sau tách nền (0..255)')
    parser.add_argument('--object-dilate', type=int, default=3, help='Bán kính giãn vùng vật (pixel, ≥ 0)')
    parser.add_argument('--sharp-min', type=float, default=0.85, help='Độ sắc tối thiểu của thành phần (≥ 0)')
    parser.add_argument('--qa', type=Path, help='Thư mục CSV và bảng ảnh soát nhãn')
    parser.add_argument('--qa-min-ratio', type=float, default=0.005, help='Tỉ lệ {} tối thiểu (0..1)'.format(LINE))
    parser.add_argument('--qa-max-ratio', type=float, default=0.15, help='Tỉ lệ {} tối đa (0..1)'.format(LINE))
    parser.add_argument('--qa-max-comp', type=int, default=60, help='Số mảnh {} tối đa (≥ 0)'.format(LINE))
    args = parser.parse_args(argv)
    try:
        if not 0 <= args.sat_max <= 255 or not 0 <= args.val_min <= 255:
            raise ValueError('Ngưỡng HSV phải nằm trong 0..255')
        if args.loc_blur < 3 or args.loc_blur % 2 != 1:
            raise ValueError('loc_blur phải là số nguyên lẻ ≥ 3')
        if not 0 <= args.loc_diff <= 255 or not 0 <= args.loc_sat_max <= 255:
            raise ValueError('Ngưỡng {} cục bộ phải nằm trong 0..255'.format(LINE))
        if args.close_kernel < 1:
            raise ValueError('close_kernel phải là số nguyên ≥ 1')
        if args.min_area < 0:
            raise ValueError('min_area phải là số nguyên ≥ 0')
        if args.tophat_kernel < 3 or args.tophat_kernel % 2 != 1:
            raise ValueError('tophat_kernel phải là số nguyên lẻ ≥ 3')
        if not 0 <= args.tophat_min <= 255 or not 0 <= args.tophat_sat_max <= 255:
            raise ValueError('Ngưỡng tophat phải nằm trong 0..255')
        if args.object_dilate < 0:
            raise ValueError('object_dilate phải là số nguyên ≥ 0')
        if not np.isfinite(args.sharp_min) or args.sharp_min < 0:
            raise ValueError('sharp_min phải hữu hạn và ≥ 0')
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
            mask, object_mask, glare, sharp_rejected = _make_mask_layers(
                image, annotations, categories, metadata['width'], metadata['height'],
                args.sat_max, args.val_min, args.loc_blur, args.loc_diff,
                args.loc_sat_max, args.close_kernel, args.min_area, args.line_method,
                args.tophat_kernel, args.tophat_min, args.tophat_sat_max,
                args.object_dilate, args.sharp_min)
            write_image(args.out / (name + '.png'), mask)
            if args.preview:
                overlay = image.copy()
                for value, color in ((1, (0, 255, 0)), (2, (0, 0, 255))):
                    selected = mask == value
                    overlay[selected] = (image[selected] * 0.5 + np.array(color) * 0.5).astype(np.uint8)
                write_image(args.preview / (name + '.png'), overlay)
            if args.qa:
                row = qa_row(name, mask, args.qa_min_ratio, args.qa_max_ratio, args.qa_max_comp)
                tile = qa_tile(image, mask, object_mask, glare, name, row[6], sharp_rejected)
                rejected_count = cv2.connectedComponentsWithStats(sharp_rejected, connectivity=8)[0] - 1
                row += (args.line_method, rejected_count, int(np.count_nonzero(sharp_rejected)))
                qa_entries.append((row, tile))
            ratios += np.bincount(mask.ravel(), minlength=3) / mask.size
            processed += 1
        print('Số ảnh COCO: {}; đã tạo mask: {}; thiếu file gốc: {}'.format(
            len(jobs), processed, missing))
        if processed:
            print('Tỉ lệ pixel trung bình: {background}={0:.6f}, '
                  '{track}={1:.6f}, {line}={2:.6f}'.format(
                      *(ratios / processed), background=BACKGROUND, track=TRACK_SURFACE, line=LINE))
        else:
            print('Tỉ lệ pixel trung bình: chưa có ảnh để tính')
        if args.qa:
            write_qa(args.qa, qa_entries)
            print('Số ảnh có cờ QA: {}'.format(sum(bool(row[6]) for row, _ in qa_entries)))
        return 0
    except (OSError, ValueError, KeyError, TypeError, cv2.error) as exc:
        print('LỖI: {}'.format(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, 'reconfigure'):
            stream.reconfigure(encoding='utf-8')
    sys.exit(main())
