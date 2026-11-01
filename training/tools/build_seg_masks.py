"""Đổi đa giác COCO thành mask 0=nền, 1=mặt sa bàn, 2=vạch trắng."""

import argparse
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


def make_mask(image, annotations, categories, width, height, sat_max=75, val_min=205):
    if image.shape != (SIZE[1], SIZE[0], 3):
        raise ValueError('Ảnh gốc phải có kích thước 1280×720, 3 kênh màu')
    if not 0 <= sat_max <= 255 or not 0 <= val_min <= 255:
        raise ValueError('Ngưỡng HSV phải nằm trong 0..255')
    if not np.isfinite([width, height]).all() or width <= 0 or height <= 0:
        raise ValueError('width/height COCO phải hữu hạn và lớn hơn 0')
    unknown = set(categories.values()) - {'mat_sa_ban', 'vat'}
    if unknown:
        raise ValueError('Tên lớp không hỗ trợ: {}'.format(', '.join(sorted(unknown))))
    layers = defaultdict(list)
    for annotation in annotations:
        category = annotation['category_id']
        if category not in categories:
            raise ValueError('category_id không có trong categories: {}'.format(category))
        layers[categories[category]].extend(polygons(annotation, width, height))
    mask = np.zeros((SIZE[1], SIZE[0]), dtype=np.uint8)
    for name, value in (('mat_sa_ban', 1), ('vat', 0)):
        for ring in layers[name]:
            cv2.fillPoly(mask, [ring], value)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask[(mask == 1) & (hsv[:, :, 1] <= sat_max) & (hsv[:, :, 2] >= val_min)] = 2
    return mask


def load_jobs(paths):
    jobs, names = [], set()
    for path in paths:
        path = path / '_annotations.coco.json' if path.is_dir() else path
        with path.open(encoding='utf-8-sig') as stream:
            data = json.load(stream)
        categories = {c['id']: c['name'] for c in data['categories']}
        if len(categories) != len(data['categories']):
            raise ValueError('{}: id lớp trùng'.format(path))
        unknown = set(categories.values()) - {'mat_sa_ban', 'vat'}
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
    parser.add_argument('--preview', type=Path, help='Thư mục ảnh phủ màu để soát nhãn')
    args = parser.parse_args(argv)
    try:
        if not 0 <= args.sat_max <= 255 or not 0 <= args.val_min <= 255:
            raise ValueError('Ngưỡng HSV phải nằm trong 0..255')
        destinations = [args.out] + ([args.preview] if args.preview else [])
        for destination in destinations:
            if args.images.resolve() in (destination.resolve(),) + tuple(destination.resolve().parents):
                raise ValueError('Thư mục đầu ra không được nằm trong thư mục ảnh gốc')
        if args.preview and args.preview.resolve() == args.out.resolve():
            raise ValueError('--preview và --out phải khác thư mục')
        jobs = load_jobs(args.coco)
        for destination in destinations:
            destination.mkdir(parents=True, exist_ok=True)
        ratios = np.zeros(3, dtype=np.float64)
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
            mask = make_mask(image, annotations, categories, metadata['width'], metadata['height'],
                             args.sat_max, args.val_min)
            write_image(args.out / (name + '.png'), mask)
            if args.preview:
                overlay = image.copy()
                for value, color in ((1, (0, 255, 0)), (2, (0, 0, 255))):
                    selected = mask == value
                    overlay[selected] = (image[selected] * 0.5 + np.array(color) * 0.5).astype(np.uint8)
                write_image(args.preview / (name + '.png'), overlay)
            ratios += np.bincount(mask.ravel(), minlength=3) / mask.size
            processed += 1
        print('Số ảnh COCO: {}; đã tạo mask: {}; thiếu file gốc: {}'.format(
            len(jobs), processed, missing))
        if processed:
            print('Tỉ lệ pixel trung bình: nền={:.6f}, mặt sa bàn={:.6f}, vạch={:.6f}'.format(
                *(ratios / processed)))
        else:
            print('Tỉ lệ pixel trung bình: chưa có ảnh để tính')
        return 0
    except (OSError, ValueError, KeyError, TypeError, cv2.error) as exc:
        print('LỖI: {}'.format(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, 'reconfigure'):
            stream.reconfigure(encoding='utf-8')
    sys.exit(main())
