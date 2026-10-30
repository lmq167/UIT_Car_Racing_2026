"""Kiểm tra, lập phiếu, nhập nhãn và chia dữ liệu T28 theo cả lượt."""

import argparse
import csv
import io
import math
import random
import re
import shutil
import sys
from collections import Counter, defaultdict
from pathlib import Path


LABELS = {'c': (0, 8), 't': (0, 9), 's': (0, 5),
          'e': (1, 3), 'k': (1, 4), 'v': (1, 2)}
AXES = ('c', 't', 's', 'e', 'k', 'v')
IMAGE = re.compile(r'p(?P<pass>[0-9]{4})_(?P<index>[0-9]{4})\.jpg')
PASS_COLUMNS = ('pass', 'event', 'ts_ms', 'frames', 'end_reason')
SHEET_COLUMNS = ('Lượt', 'Chặng (c)', 'Mục tiêu (t)', 'Biển (s)',
                 'Ánh sáng (e)', 'Kiểu lái (k)', 'Hướng (v)',
                 'Kết quả', 'Ghi chú', 'Số ảnh')
LABEL_COLUMNS = ('lượt',) + AXES + ('kết quả', 'ghi chú')


def integer(value, low, high, context):
    if not isinstance(value, str) or not re.fullmatch(r'[0-9]+', value):
        raise ValueError('{}: cần số nguyên không âm'.format(context))
    number = int(value)
    if number < low or (high is not None and number > high):
        raise ValueError('{}: ngoài khoảng {}..{}'.format(context, low, high))
    return number


def read_csv(path, required):
    with path.open(encoding='utf-8-sig', newline='') as stream:
        reader = csv.DictReader(stream, strict=True)
        fields = reader.fieldnames or []
        if len(fields) != len(set(fields)) or not set(required).issubset(fields):
            raise ValueError('{}: cột trùng hoặc thiếu {}'.format(path.name, required))
        rows = []
        for line, row in enumerate(reader, 2):
            if None in row or None in row.values():
                raise ValueError('{} dòng {}: thiếu/thừa ô CSV'.format(path.name, line))
            rows.append(row)
        return rows


def read_passes(root, require_end=True):
    events = defaultdict(list)
    for line, row in enumerate(read_csv(root / 'passes.csv', PASS_COLUMNS), 2):
        context = 'passes.csv dòng {}'.format(line)
        pid = integer(row['pass'], 1, 9999, context + ' pass')
        timestamp = integer(row['ts_ms'], 0, None, context + ' ts_ms')
        count = integer(row['frames'], 0, None, context + ' frames')
        event = row['event']
        if event not in ('start', 'end'):
            raise ValueError(context + ': sự kiện phải là start/end')
        events[pid].append((event, count, timestamp))
    for pid, history in sorted(events.items()):
        names = [item[0] for item in history]
        context = 'p{:04d}'.format(pid)
        if names.count('start') != 1 or names[0] != 'start':
            raise ValueError(context + ': thiếu start hoặc số lượt trùng (start lặp/sai thứ tự)')
        if require_end and 'end' not in names:
            raise ValueError(context + ': lượt dở dang, thiếu end')
        if names.count('end') > 1:
            raise ValueError(context + ': end trùng')
    return dict(events)


def inspect_dataset(root):
    """Trả ảnh theo lượt, lịch sử sự kiện và danh sách lỗi, không sửa dữ liệu."""
    root = Path(root)
    if not root.is_dir():
        raise ValueError('Không có thư mục nguồn: {}'.format(root))
    errors, images = [], {}
    numbers = set()
    for path in sorted(root.iterdir()):
        if not path.is_file():
            continue
        if path.name.endswith('.tmp'):
            errors.append('Tệp tạm còn sót: {}'.format(path.name))
        if path.suffix.lower() not in ('.jpg', '.jpeg', '.png'):
            continue
        if path.stat().st_size == 0:
            errors.append('Ảnh 0 byte: {}'.format(path.name))
        match = IMAGE.fullmatch(path.name)
        if not match:
            errors.append('Sai tên ảnh T28: {}'.format(path.name))
            continue
        try:
            pid = integer(match['pass'], 1, 9999, path.name + ' pass')
            index = int(match['index'])
            if (pid, index) in numbers:
                errors.append('Số ảnh trùng trong lượt p{:04d}: {}'.format(pid, index))
            numbers.add((pid, index))
            images[path.name] = (pid, index)
        except ValueError as exc:
            errors.append(str(exc))

    try:
        events = read_passes(root)
    except (OSError, ValueError, csv.Error) as exc:
        errors.append(str(exc))
        events = {}

    try:
        rows = read_csv(root / 'frames.csv', ('pass', 'file'))
    except (OSError, ValueError, csv.Error) as exc:
        errors.append(str(exc))
        rows = []
    logged = set()
    for line, row in enumerate(rows, 2):
        context = 'frames.csv dòng {}'.format(line)
        try:
            pid = integer(row['pass'], 1, 9999, context + ' pass')
            name = row['file']
            if not IMAGE.fullmatch(name):
                raise ValueError(context + ': file phải là tên ảnh T28 trong thư mục phẳng')
            if name in logged:
                errors.append('Dòng ảnh trùng: {}'.format(name))
            logged.add(name)
            if name not in images:
                errors.append('CSV tham chiếu ảnh thiếu/không hợp lệ: {}'.format(name))
            elif images[name][0] != pid:
                errors.append(context + ': pass khác tên ảnh')
            if pid not in events:
                errors.append(context + ': pass không có trong passes.csv')
        except ValueError as exc:
            errors.append(str(exc))
    for name in sorted(set(images) - logged):
        errors.append('Ảnh thiếu dòng frames.csv: {}'.format(name))

    by_pass = defaultdict(list)
    for name, (pid, index) in sorted(images.items()):
        by_pass[pid].append(name)
        if pid not in events:
            errors.append('Ảnh thuộc lượt không có trong passes.csv: {}'.format(name))
    for pid, history in sorted(events.items()):
        for event, count, timestamp in history:
            if event == 'end' and count != len(by_pass[pid]):
                errors.append('p{:04d}: số frames ở end khác số ảnh'.format(pid))
    return dict(by_pass), dict(events), errors


def label_values(row):
    values, warnings = [], []
    for key in AXES:
        try:
            values.append(integer(row[key], LABELS[key][0], LABELS[key][1], 'nhãn ' + key))
        except ValueError as exc:
            values.append(None)
            warnings.append('nhãn {}: ô trống'.format(key) if row[key] == '' else str(exc))
    if row['kết quả'] not in ('tốt', 'hỏng'):
        warnings.append('kết quả: cần tốt/hỏng' if row['kết quả'] else 'kết quả: ô trống')
    return tuple(values), warnings


def read_labels(path, events):
    records = {}
    for line, row in enumerate(read_csv(path, LABEL_COLUMNS), 2):
        pid = integer(row['lượt'], 1, 9999, 'labels.csv dòng {} lượt'.format(line))
        if pid not in events or pid in records:
            raise ValueError('labels.csv: lượt không có hoặc trùng: {}'.format(pid))
        row = {key: value.strip() for key, value in row.items()}
        row['kết quả'] = row['kết quả'].casefold()
        records[pid] = row
    return records


def warn_labels(records, events):
    for pid in sorted(events):
        if pid not in records:
            print('CẢNH BÁO: p{:04d} chưa có nhãn.'.format(pid), file=sys.stderr)
        else:
            for warning in label_values(records[pid])[1]:
                print('CẢNH BÁO: p{:04d}: {}'.format(pid, warning), file=sys.stderr)


def usable_passes(by_pass, events, records):
    return {pid: by_pass.get(pid, []) for pid in events
            if records.get(pid, {}).get('kết quả') != 'hỏng'}


def coverage_rows(by_pass, records):
    counts = Counter()
    for pid, names in by_pass.items():
        if pid not in records:
            continue
        for axis, value in zip(AXES, label_values(records[pid])[0]):
            if value is None:
                continue
            counts[(axis, value, 'images')] += len(names)
            counts[(axis, value, 'passes')] += 1
    return [(axis, value, counts[(axis, value, 'images')],
             counts[(axis, value, 'passes')])
            for axis in AXES for value in range(LABELS[axis][0], LABELS[axis][1] + 1)]


def target_gaps(path, by_pass, records):
    """Ô trống ở c/t/s/e/k/v nghĩa là không ràng buộc chiều đó."""
    gaps = []
    for line, row in enumerate(read_csv(Path(path), AXES + ('min_images', 'min_passes')), 2):
        context = 'mục tiêu dòng {}'.format(line)
        selected = {key: integer(row[key], LABELS[key][0], LABELS[key][1], context + ' ' + key)
                    for key in AXES if row[key] != ''}
        if not selected:
            raise ValueError(context + ': cần ít nhất một nhãn')
        minimum_images = integer(row['min_images'], 0, None, context + ' min_images')
        minimum_passes = integer(row['min_passes'], 0, None, context + ' min_passes')
        matched = [pid for pid in by_pass if pid in records and all(
            label_values(records[pid])[0][AXES.index(key)] == value
            for key, value in selected.items())]
        images = sum(len(by_pass[pid]) for pid in matched)
        missing_images = max(0, minimum_images - images)
        missing_passes = max(0, minimum_passes - len(matched))
        if missing_images or missing_passes:
            cell = ' '.join('{}={}'.format(key, selected[key]) for key in AXES if key in selected)
            gaps.append((cell, missing_images, missing_passes))
    return gaps


def split_passes(by_pass, records, val_ratio=0.2, seed=28):
    if not math.isfinite(val_ratio) or not 0 < val_ratio < 1:
        raise ValueError('val-ratio phải hữu hạn và nằm trong (0, 1)')
    by_pass = {pid: names for pid, names in by_pass.items()
               if names and records.get(pid, {}).get('kết quả') != 'hỏng'}
    groups = defaultdict(list)
    for pid in sorted(by_pass):
        tag = label_values(records[pid])[0][:2] if pid in records else (None, None)
        # Thiếu c hoặc t: cùng nhóm chưa phân loại, không suy đoán nhãn.
        key = tag if None not in tag else (-1, -1)
        groups[key].append(pid)
    rng = random.Random(seed)
    validation, singletons = set(), []
    for key in sorted(groups):
        group = groups[key]
        rng.shuffle(group)
        if len(group) == 1:
            singletons.extend(group)
        else:
            count = max(1, min(len(group) - 1, int(len(group) * val_ratio + 0.5)))
            validation.update(group[:count])
    rng.shuffle(singletons)
    remaining = max(0, min(len(singletons), int(len(by_pass) * val_ratio + 0.5) - len(validation)))
    validation.update(singletons[:remaining])
    return {name: sorted(file for pid, files in by_pass.items()
                         if (pid in validation) == is_val for file in files)
            for name, is_val in (('train', False), ('val', True))}


def sheet(events):
    stream = io.StringIO()
    writer = csv.writer(stream, lineterminator='\n')
    writer.writerow(SHEET_COLUMNS)
    for pid, history in sorted(events.items()):
        for event, count, timestamp in history:
            if event == 'end':
                writer.writerow([pid] + [''] * 8 + [count])
    return stream.getvalue()


def read_sheet(path, events):
    records, seen = {}, set()
    with path.open(encoding='utf-8-sig', newline='') as stream:
        reader = csv.reader(stream, strict=True)
        fields = None
        for cells in reader:
            if not any(cell.strip() for cell in cells):
                continue
            if fields is None:
                fields = []
                for cell in cells:
                    name = cell.strip().casefold()
                    match = re.search(r'\(([ctsekv])\)$', name)
                    fields.append(match[1] if match else name)
                if len(fields) != len(set(fields)) or not set(LABEL_COLUMNS).issubset(fields):
                    raise ValueError('Phiếu: cột trùng hoặc thiếu {}'.format(LABEL_COLUMNS))
                continue
            context = 'Phiếu dòng {}'.format(reader.line_num)
            try:
                if len(cells) != len(fields):
                    raise ValueError('thiếu/thừa ô CSV')
                row = dict(zip(fields, (cell.strip() for cell in cells)))
                pid = integer(row['lượt'], 1, 9999, 'lượt')
                context += ' (p{:04d})'.format(pid)
                if pid in seen:
                    raise ValueError('số lượt trùng: {}'.format(pid))
                seen.add(pid)
                if pid not in events:
                    raise ValueError('lượt không có trong passes.csv: {}'.format(pid))
                row = {key: row[key] for key in LABEL_COLUMNS}
                row['lượt'] = str(pid)
                row['kết quả'] = row['kết quả'].casefold()
                for key in AXES:
                    match = re.fullmatch(r'([0-9]+)(?: - .+)?', row[key])
                    if match:
                        row[key] = str(int(match[1]))
                warnings = label_values(row)[1]
                if warnings:
                    raise ValueError('; '.join(warnings))
                records[pid] = row
            except ValueError as exc:
                print('CẢNH BÁO: {}: {}; không áp dụng dòng này.'.format(context, exc),
                      file=sys.stderr)
        if fields is None:
            raise ValueError('Phiếu CSV thiếu tiêu đề')
    return records


def apply_sheet(root, path, by_pass, events, copy_to=None):
    records = read_sheet(path, events)
    if not records:
        return 0, 0
    label_path = root / 'labels.csv'
    merged = read_labels(label_path, events) if label_path.exists() else {}
    merged.update(records)
    copies = []
    if copy_to is not None:
        if copy_to.resolve() == root.resolve():
            raise ValueError('copy-to phải khác thư mục nguồn')
        if copy_to.exists() and not copy_to.is_dir():
            raise ValueError('copy-to phải là thư mục')
        for pid, row in sorted(records.items()):
            tag = label_values(row)[0]
            for name in by_pass.get(pid, []):
                index = IMAGE.fullmatch(name)['index']
                target = copy_to / ('p{:04d}_c{}_t{}_s{}_e{}_k{}_v{}_{}.jpg'.format(pid, *tag, index))
                if target.exists() or target.is_symlink():
                    raise ValueError('Không ghi đè bản sao đã có: {}'.format(target))
                copies.append((root / name, target))
        copy_to.mkdir(parents=True, exist_ok=True)
        for source, target in copies:
            shutil.copy2(str(source), str(target))
    # Ghi xong tệp tạm rồi mới thay labels.csv để tránh làm mất phiếu cũ khi lỗi.
    temporary = root / 'labels.csv.tmp'
    with temporary.open('x', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=LABEL_COLUMNS, lineterminator='\n',
                                extrasaction='ignore')
        writer.writeheader()
        writer.writerows(merged[pid] for pid in sorted(merged))
    temporary.replace(label_path)
    return len(records), len(copies)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command')
    for name in ('check', 'sheet', 'apply', 'coverage', 'split'):
        command = commands.add_parser(name)
        command.add_argument('root', type=Path, help='Thư mục phẳng chứa ảnh và CSV T28')
        if name == 'apply':
            command.add_argument('sheet', type=Path, help='Phiếu CSV Google Sheet đã điền')
            command.add_argument('--copy-to', type=Path, help='Thư mục bản sao có nhãn')
        if name == 'coverage':
            command.add_argument('--targets', type=Path, help='CSV mục tiêu độ phủ')
        if name == 'split':
            command.add_argument('--seed', type=int, default=28, help='Hạt giống ngẫu nhiên, mặc định 28')
            command.add_argument('--val-ratio', type=float, default=0.2, help='Tỉ lệ lượt val, mặc định 0.2')
    args = parser.parse_args(argv)
    if args.command is None:
        parser.error('Cần chọn check, sheet, apply, coverage hoặc split')
    try:
        if args.command == 'sheet':
            print(sheet(read_passes(args.root, require_end=False)), end='')
            return 0
        by_pass, events, errors = inspect_dataset(args.root)
        if errors:
            for error in errors:
                print('LỖI: ' + error, file=sys.stderr)
            return 1
        if args.command == 'check':
            print('ĐẠT: {} lượt, {} ảnh'.format(len(events), sum(map(len, by_pass.values()))))
        elif args.command == 'apply':
            count, copied = apply_sheet(args.root, args.sheet, by_pass, events, args.copy_to)
            if count:
                print('Đã ghi labels.csv: {} lượt; sao chép {} ảnh.'.format(count, copied))
            else:
                print('Không có dòng hợp lệ; không thay đổi labels.csv hay ảnh.')
        else:
            label_path = args.root / 'labels.csv'
            records = read_labels(label_path, events) if label_path.exists() else {}
            if args.command == 'coverage' and not label_path.exists():
                raise ValueError('Cần labels.csv; dùng sheet và apply trước coverage')
            if label_path.exists():
                warn_labels(records, events)
            good = usable_passes(by_pass, events, records)
            print('Bỏ {} lượt hỏng; còn {} lượt.'.format(len(events) - len(good), len(good)), file=sys.stderr)
            writer = csv.writer(sys.stdout, lineterminator='\n')
            if args.command == 'coverage':
                gaps = target_gaps(args.targets, good, records) if args.targets else []
                writer.writerow(('label', 'value', 'images', 'passes'))
                writer.writerows(coverage_rows(good, records))
                if args.targets:
                    writer.writerow(('cell', 'missing_images', 'missing_passes'))
                    writer.writerows(gaps)
            else:
                parts = split_passes(good, records, args.val_ratio, args.seed)
                writer.writerow(('split', 'file'))
                for part in ('train', 'val'):
                    writer.writerows((part, name) for name in parts[part])
                if not parts['train'] or not parts['val']:
                    print('LƯU Ý: một tập rỗng vì không đủ lượt độc lập.', file=sys.stderr)
        return 0
    except (OSError, ValueError, csv.Error) as exc:
        print('LỖI: {}'.format(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
