"""Tạo nhãn theo khung hình và danh sách train/val từ export YOLOv8 car6_det."""

import argparse
import csv
import math
import os
import sys
from collections import Counter, defaultdict
from pathlib import Path

import yaml

from build_seg_masks import original_name


ROOT = Path(__file__).resolve().parents[2]
HARD_T = {1, 2, 3, 4, 5, 6, 7, 9}
FIELDS = ['file', 'pass', 'classes', 'n_boxes', 'is_null', 'null_type']


class Parser(argparse.ArgumentParser):
    def error(self, message):
        raise ValueError('Tham số không hợp lệ: ' + message)


def read_table(path, fields):
    with path.open(encoding='utf-8-sig', newline='') as stream:
        reader = csv.DictReader(stream)
        if not set(fields).issubset(reader.fieldnames or []):
            raise ValueError('CSV thiếu cột bắt buộc: {}'.format(path))
        rows = {}
        for row in reader:
            if not row['file'] or row['file'] in rows:
                raise ValueError('Tên file rỗng hoặc trùng trong {}'.format(path))
            rows[row['file']] = row
        return rows


class _NameLoader(yaml.SafeLoader):
    pass


# YAML 1.1 đọc `off` thành False; tên lớp `off` của car6_det phải giữ là chuỗi.
_NameLoader.yaml_implicit_resolvers = {
    key: [item for item in value if item[0] != 'tag:yaml.org,2002:bool']
    for key, value in yaml.SafeLoader.yaml_implicit_resolvers.items()}


def read_names(path):
    with path.open(encoding='utf-8-sig') as stream:
        data = yaml.load(stream, Loader=_NameLoader)
    names = data.get('names') if isinstance(data, dict) else None
    if isinstance(names, list):
        names = dict(enumerate(names))
    if (not isinstance(names, dict) or not names or
            any(type(key) is not int or key < 0 or not isinstance(value, str)
                or not value.strip() for key, value in names.items()) or
            len(set(names.values())) != len(names)):
        raise ValueError('names trong data.yaml phải là danh sách hoặc bảng id: tên lớp')
    if 'right' not in names.values():
        raise ValueError('Thiếu lớp right trong data.yaml')
    return names


def read_boxes(path, names):
    classes = set()
    count = 0
    for number, line in enumerate(path.read_text(encoding='utf-8-sig').splitlines(), 1):
        if not line.strip():
            continue
        try:
            parts = line.split()
            if len(parts) != 5:
                raise ValueError()
            class_id = int(parts[0])
            box = [float(value) for value in parts[1:]]
            if (class_id not in names or
                    any(not math.isfinite(value) or not 0 <= value <= 1 for value in box) or
                    box[2] <= 0 or box[3] <= 0):
                raise ValueError()
        except ValueError:
            raise ValueError('Nhãn YOLO không hợp lệ: {} dòng {}'.format(path, number))
        classes.add(names[class_id])
        count += 1
    return classes, count


def read_frames(export, data_dir, names):
    manifest = read_table(data_dir / 'manifest.csv', ['file', 'pass', 'idx', 't', 's', 'l'])
    splits = read_table(data_dir / 'split_v0.csv', ['file', 'pass', 'split'])
    frames = []
    seen = set()
    for subset in ('train', 'valid', 'test'):
        for image in sorted((export / subset / 'images').glob('*.jpg')):
            try:
                stem = original_name(image.name)
            except ValueError:
                # Hàm chung nhận dạng thiếu _jpg khi tên đã có tiền tố car6_.
                stem = original_name('car6_' + image.name)
            filename = stem + '.jpg'
            if filename in seen:
                raise ValueError('Trùng tên gốc trong export (có thể có augmentation): ' + filename)
            seen.add(filename)
            if filename not in manifest:
                raise ValueError('Ảnh không có trong manifest.csv: ' + filename)
            if filename not in splits:
                raise ValueError('Ảnh không có trong split_v0.csv: ' + filename)
            meta, split = manifest[filename], splits[filename]
            try:
                pass_id, idx, t, s, l = [int(meta[key]) for key in ('pass', 'idx', 't', 's', 'l')]
                if int(split['pass']) != pass_id or split['split'] not in ('train', 'val'):
                    raise ValueError()
            except (TypeError, ValueError):
                raise ValueError('Metadata hoặc split không hợp lệ: ' + filename)
            classes, n_boxes = read_boxes(image.parent.parent / 'labels' / (image.stem + '.txt'), names)
            frames.append(dict(file=filename, **{'pass': pass_id}, idx=idx, t=t,
                               classes=classes, n_boxes=n_boxes, is_null=int(n_boxes == 0),
                               null_type=('hard' if t in HARD_T or s != 0 or l != 0 else 'easy')
                               if n_boxes == 0 else '', split=split['split'], image=image))
    if not frames:
        raise ValueError('Không tìm thấy ảnh JPG trong train/valid/test của export')
    return sorted(frames, key=lambda frame: frame['file'])


def spread(frames, count):
    if count >= len(frames):
        return list(frames)
    if count <= 0:
        return []
    if count == 1:
        return [frames[len(frames) // 2]]
    return [frames[round(i * (len(frames) - 1) / (count - 1))] for i in range(count)]


def filter_train(frames, max_right, cap_t, null_frac):
    keep, right, nulls = [], defaultdict(list), {'hard': [], 'easy': []}
    for frame in frames:
        if frame['is_null']:
            nulls[frame['null_type']].append(frame)
        elif frame['classes'] == {'right'} and frame['t'] in cap_t:
            right[frame['pass']].append(frame)
        else:
            keep.append(frame)
    for group in right.values():
        keep.extend(spread(sorted(group, key=lambda frame: (frame['idx'], frame['file'])), max_right))
    wanted = round(null_frac / (1 - null_frac) * len(keep))
    available = sum(len(group) for group in nulls.values())
    if available < wanted:
        print('Cảnh báo: thiếu null train, cần {} nhưng chỉ có {}.'.format(wanted, available),
              file=sys.stderr)
    remaining = wanted
    for kind in ('hard', 'easy'):
        selected = spread(nulls[kind], min(remaining, len(nulls[kind])))
        keep.extend(selected)
        remaining -= len(selected)
    return sorted(keep, key=lambda frame: frame['file'])


def print_counts(before, after, names):
    def counts(frames):
        return Counter(name for frame in frames for name in
                       (['null'] if frame['is_null'] else frame['classes']))

    counters = [counts(group) for group in before + after]
    print('Lớp\ttrain trước\ttrain sau\tval trước\tval sau')
    for name in sorted(names.values()) + ['null']:
        print('{}\t{}\t{}\t{}\t{}'.format(name, counters[0][name], counters[2][name],
                                               counters[1][name], counters[3][name]))


def main(argv=None):
    parser = Parser(description=__doc__)
    parser.add_argument('--export', required=True, type=Path, help='Thư mục export YOLOv8')
    parser.add_argument('--data-dir', type=Path, default=ROOT / 'data/round3/data_clean_car6',
                        help='Thư mục manifest.csv và split_v0.csv')
    parser.add_argument('--out-dir', type=Path, help='Nơi ghi train.txt/val.txt, mặc định là export')
    parser.add_argument('--max-right-per-pass', type=int, default=4, help='Số ảnh right-thuần tối đa mỗi lượt')
    parser.add_argument('--cap-right-t', default='9', help='Các giá trị t cần giới hạn, ngăn bằng dấu phẩy')
    parser.add_argument('--null-frac', type=float, default=0.18, help='Tỷ lệ null train mong muốn')
    try:
        args = parser.parse_args(argv)
        if args.max_right_per_pass < 0 or not 0 <= args.null_frac < 1:
            raise ValueError('max-right-per-pass phải >= 0 và null-frac phải thuộc [0, 1)')
        try:
            cap_t = {int(value.strip()) for value in args.cap_right_t.split(',')}
        except ValueError:
            raise ValueError('cap-right-t phải là danh sách số nguyên ngăn bằng dấu phẩy')
        export = args.export.resolve()
        out = args.out_dir.resolve() if args.out_dir is not None else export
        names = read_names(export / 'data.yaml')
        frames = read_frames(export, args.data_dir, names)
        train = [frame for frame in frames if frame['split'] == 'train']
        val = [frame for frame in frames if frame['split'] == 'val']
        filtered = filter_train(train, args.max_right_per_pass, cap_t, args.null_frac)
        # Kiểm tra đầu vào xong mới bắt đầu ghi các đầu ra được phép.
        out.mkdir(parents=True, exist_ok=True)
        with (args.data_dir / 'frame_labels.csv').open('w', encoding='utf-8-sig', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=FIELDS)
            writer.writeheader()
            for frame in frames:
                row = {key: frame[key] for key in FIELDS}
                row['classes'] = '|'.join(sorted(frame['classes']))
                writer.writerow(row)
        for name, group in (('train', filtered), ('val', val)):
            lines = ['./' + Path(os.path.relpath(frame['image'], out)).as_posix() for frame in group]
            (out / (name + '.txt')).write_text(''.join(line + '\n' for line in lines), encoding='utf-8')
        print_counts([train, val], [filtered, val], names)
        return 0
    except (OSError, ValueError, yaml.YAMLError) as error:
        print('Lỗi: {}'.format(error), file=sys.stderr)
        return 1


if __name__ == '__main__':
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
        sys.stderr.reconfigure(encoding='utf-8')
    sys.exit(main())
