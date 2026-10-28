#!/usr/bin/env python3
# Kiểm hộp đen ảnh/thư mục, tỉ lệ, mã lỗi và ASCII.
import csv
import math
from pathlib import Path
import re
import statistics
import struct
import subprocess
import sys
import tempfile
import time
import zlib


HEADER = ['image', 'left_clusters', 'right_clusters', 'has_center', 'time_ms']
PNG = b'\x89PNG\r\n\x1a\n'


def check(condition, message):
    if not condition:
        raise AssertionError(message)


def chunk(kind, data):
    return (struct.pack('>I', len(data)) + kind + data +
            struct.pack('>I', zlib.crc32(kind + data) & 0xffffffff))


def write_png(path, rows):
    height, width = len(rows), len(rows[0])
    raw = b''.join(b'\0' + bytes(row) for row in rows)
    path.write_bytes(PNG + chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0)) +
                     chunk(b'IDAT', zlib.compress(raw)) + chunk(b'IEND', b''))


def read_png(path):
    data = path.read_bytes()
    check(data.startswith(PNG), 'Không phải PNG: ' + str(path))
    offset, compressed = 8, bytearray()
    while offset < len(data):
        size = struct.unpack_from('>I', data, offset)[0]
        kind = data[offset + 4:offset + 8]
        body = data[offset + 8:offset + 8 + size]
        check(zlib.crc32(kind + body) & 0xffffffff ==
              struct.unpack_from('>I', data, offset + 8 + size)[0], 'Sai CRC PNG')
        if kind == b'IHDR':
            width, height, depth, color, compression, filtering, interlace = struct.unpack('>IIBBBBB', body)
            check(depth == 8 and color in (0, 2, 4, 6) and
                  (compression, filtering, interlace) == (0, 0, 0), 'PNG cần 8 bit, không xen kẽ')
            channels = {0: 1, 2: 3, 4: 2, 6: 4}[color]
        elif kind == b'IDAT':
            compressed.extend(body)
        offset += size + 12
    raw = zlib.decompress(bytes(compressed))
    stride = width * channels
    check(len(raw) == height * (stride + 1), 'Sai kích thước dữ liệu PNG')
    previous, rows = bytearray(stride), []
    for y in range(height):
        start = y * (stride + 1)
        mode, row = raw[start], bytearray(raw[start + 1:start + 1 + stride])
        check(mode <= 4, 'Bộ lọc PNG không hợp lệ')
        for x in range(stride):
            left = row[x - channels] if x >= channels else 0
            up = previous[x]
            corner = previous[x - channels] if x >= channels else 0
            prediction = left + up - corner
            distances = [abs(prediction - v) for v in (left, up, corner)]
            paeth = (left, up, corner)[distances.index(min(distances))]
            row[x] = (row[x] + (0, left, up, (left + up) // 2, paeth)[mode]) & 255
        rows.append(row)
        previous = row
    return width, height, channels, rows


def lanes():
    rows = [bytearray(480) for _ in range(288)]
    for row in rows[144:276]:
        row[160:172] = b'\xff' * 12
        row[376:388] = b'\xff' * 12
    return rows


def enlarge(rows):
    # INTER_LINEAR: tâm điểm ảnh, chặn biên; làm tròn về mức xám 8 bit.
    result = []
    for y in range(384):
        sy = max(0, (y + 0.5) * 0.75 - 0.5)
        y0, fy = int(sy), sy - int(sy)
        upper, lower = rows[y0], rows[min(y0 + 1, 287)]
        row = bytearray()
        for x in range(640):
            sx = max(0, (x + 0.5) * 0.75 - 0.5)
            x0, fx = int(sx), sx - int(sx)
            x1 = min(x0 + 1, 479)
            value = ((1 - fy) * ((1 - fx) * upper[x0] + fx * upper[x1]) +
                     fy * ((1 - fx) * lower[x0] + fx * lower[x1]))
            row.append(int(value + 0.5))
        result.append(row)
    return result


def run(program, args, expected, deadline):
    remaining = deadline - time.monotonic()
    check(remaining > 0, 'Hết hạn chung của ca')
    # subprocess.run dừng và thu hồi tiến trình con khi hết hạn, kể cả lúc test hỏng.
    done = subprocess.run([str(program)] + [str(a) for a in args],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                          timeout=min(30, remaining))
    check(done.returncode == expected, 'Mã {}, cần {}; lệnh {}\nstdout={!r}\nstderr={!r}'.format(
        done.returncode, expected, args, done.stdout, done.stderr))
    return done.stdout


def summary(out, names):
    with (out / 'summary.csv').open(encoding='utf-8', newline='') as stream:
        reader = csv.DictReader(stream)
        check(reader.fieldnames == HEADER, 'Sai tên/thứ tự cột summary.csv')
        rows = list(reader)
    check([r['image'] for r in rows] == sorted(names), 'Sai số dòng/tên/thứ tự ảnh CSV')
    used = set()
    for row in rows:
        check(None not in row and all(v is not None for v in row.values()), 'CSV thừa/thiếu cột')
        check(int(row['left_clusters']) >= 0 and int(row['right_clusters']) >= 0, 'Số cụm âm')
        check(row['has_center'] in ('0', '1'), 'has_center phải là 0 hoặc 1')
        elapsed = float(row['time_ms'])
        check(math.isfinite(elapsed) and elapsed >= 0, 'time_ms không hữu hạn hoặc âm')
        stem = Path(row['image']).stem
        if stem in used:
            stem = row['image']
        used.add(stem)
        read_png(out / (stem + '_mask.png'))
        jpeg = (out / (stem + '_overlay.jpg')).read_bytes()
        check(jpeg.startswith(b'\xff\xd8') and jpeg.endswith(b'\xff\xd9'), 'Không phải JPEG hoàn chỉnh')
    return rows


def competing_runs(program, source, out, deadline):
    # khởi chạy cả hai trước khi chờ; luôn thu hồi cả khi ca hỏng.
    processes = []
    with tempfile.TemporaryFile() as log:
        try:
            for _ in range(2):
                processes.append(subprocess.Popen(
                    [str(program), '--in-dir', str(source), '--out-dir', str(out)],
                    stdout=log, stderr=log))
            for process in processes:
                remaining = deadline - time.monotonic()
                check(remaining > 0, 'Hết hạn ca tranh chấp thư mục')
                process.wait(timeout=min(30, remaining))
            codes = sorted(process.returncode for process in processes)
            log.seek(0)
            check(codes == [0, 4], 'Hai tiến trình phải trả [0, 4], nhận {}: {!r}'.format(codes, log.read()))
        finally:
            for process in processes:
                if process.poll() is None:
                    process.kill()
            for process in processes:
                process.wait(timeout=5)


def signature(row):
    return tuple(int(row[key]) for key in HEADER[1:4])


def check_stdout(stdout, names):
    text = stdout.decode('ascii')
    timing = r'(?:\d+(?:\.\d+)?(?:[eE][+-]?\d+)?\s*ms\b|\b\w*ms\s*[:=]\s*\d+(?:\.\d+)?)'
    count = r'(?:\b{}\s*(?:images?|anh)\b|\b(?:images?|anh|processed|count|total|so_anh)\s*[:=]?\s*{}\b)'.format(
        len(names), len(names))
    lines = text.splitlines()
    for name in names:
        check(any(name in line and re.search(timing, line, re.I) for line in lines),
              'Thiếu thời gian mỗi ảnh trên stdout: ' + name)
    check(lines and re.search(count, lines[-1], re.I) and
          re.search(r'average|avg|mean|trung\s*binh|\btb\b', lines[-1], re.I) and
          re.search(timing, lines[-1], re.I), 'Thiếu dòng cuối: số ảnh và trung bình ms')


def centers(path, size):
    width, height, channels, rows = read_png(path)
    check((width, height) == size and channels in (3, 4), 'Overlay không giữ kích thước RGB gốc')
    values = []
    for ratio in (0.6, 0.7, 0.8, 0.9):
        xs = []
        for y in range(int((ratio - 0.02) * height), int((ratio + 0.02) * height) + 1):
            for x in range(int(0.42 * width), int(0.72 * width)):
                r, g, b = rows[y][x * channels:x * channels + 3]
                if g >= 170 and r < 130 and b < 130:
                    xs.append(x)
        check(xs, 'Không thấy chấm đường giữa tại y/H={}'.format(ratio))
        values.append(statistics.median(xs) / width)
    return values


def geometry(program, root, image, size, deadline):
    out = root / ('single_' + image.stem)
    run(program, ['--in', image, '--out-dir', out, '--no-steps', '--min-left-components', '1'], 0, deadline)
    item = out / ('000001_' + image.name)
    check(read_png(item / 'normalized.png')[:2] == size, 'Ảnh xử lý bị đổi kích thước')
    return centers(item / 'overlay.png', size)


def batch(program, root, images, deadline, extra=()):
    source, out = root / 'input', root / 'output'
    source.mkdir()
    for name, rows in images:
        write_png(source / name, rows)
    stdout = run(program, ['--in-dir', source, '--out-dir', out] + list(extra), 0, deadline)
    rows = summary(out, [name for name, _ in images])
    check_stdout(stdout, [name for name, _ in images])
    return source, out, rows


def scenario(program, name, root, deadline):
    # Kỳ vọng từng ca đã nộp trước trong BẢNG CA; không gọi --self-test của sản phẩm.
    if name == 'directory':
        images = [('c_lane.png', lanes()), ('a_blank.png', [bytearray(480) for _ in range(288)]),
                  ('b_lane.png', lanes()), ('same.png', lanes()), ('same.jpg', lanes())]
        _, _, rows = batch(program, root, images, deadline)
        check(signature(rows[0]) == (0, 0, 0), 'Ảnh đen phải xử lý xong và không có làn')
    elif name in ('direct_640', 'scale'):
        large = enlarge(lanes())
        if name == 'direct_640':
            # Vẽ trực tiếp theo tỉ lệ, độc lập với đường nội suy của ca scale.
            large = [bytearray(640) for _ in range(384)]
            for row in large[192:368]:
                row[213:229] = b'\xff' * 16
                row[501:517] = b'\xff' * 16
            images = [('large.png', large)]
        else:
            images = [('base.png', lanes()), ('large.png', large)]
        source, out, rows = batch(program, root, images, deadline, ['--min-left-components', '1'])
        for row in rows:
            check(signature(row) == (1, 1, 1), 'Hai vạch phải có cụm 1/1 và đường giữa')
        check(read_png(out / 'large_mask.png')[:2] == (640, 384), 'Mask không giữ cỡ 640x384')
        scaled = geometry(program, root, source / 'large.png', (640, 384), deadline)
        check(all(abs(x - 273.5 / 480) <= 0.03 for x in scaled), 'Đường giữa lệch quá 3%W')
        if name == 'scale':
            original = geometry(program, root, source / 'base.png', (480, 288), deadline)
            check(all(abs(a - b) <= 0.03 for a, b in zip(original, scaled)), 'Phóng ảnh lệch quá 3%W')
    elif name == 'invalid':
        source = root / 'input'
        source.mkdir()
        image = source / 'lane.png'
        write_png(image, lanes())
        cases = [(['--unknown', '1'], False), (['--smax', 'nan'], False),
                 (['--vmin', '200junk'], False), (['--morph-kernel', '2'], False),
                 (['--fit', 'unknown'], False), (['--in', image], False),
                 (['--in-dir'], True), (['--out-dir'], True),
                 (['--in-dir', source], True), (['--in-dir', source, '--out-dir'], True)]
        for i, (args, bare) in enumerate(cases):
            folder = root / str(i)
            folder.mkdir()
            prefix = [] if bare else ['--in-dir', source, '--out-dir', folder / 'out']
            run(program, prefix + args, 1, deadline)
    elif name == 'empty':
        for label in ('empty', 'unreadable'):
            folder = root / label
            folder.mkdir()
            if label == 'unreadable':
                (folder / 'bad.png').write_bytes(b'not an image')
            run(program, ['--in-dir', folder, '--out-dir', root / (label + '_out')], 2, deadline)
    elif name == 'occupied':
        source = root / 'input'
        source.mkdir()
        write_png(source / 'lane.png', lanes())
        for label in ('file', 'subdir'):
            out = root / label
            out.mkdir()
            if label == 'file':
                (out / 'keep.txt').write_bytes(b'keep exactly\x00\xff')
            else:
                (out / 'keep').mkdir()
            before = {str(p.relative_to(out)): p.read_bytes() if p.is_file() else None
                      for p in out.rglob('*')}
            run(program, ['--in-dir', source, '--out-dir', out], 4, deadline)
            after = {str(p.relative_to(out)): p.read_bytes() if p.is_file() else None
                     for p in out.rglob('*')}
            check(after == before, 'Đích có sẵn bị thay đổi')
    elif name == 'concurrent_output':
        source = root / 'input'
        source.mkdir()
        names = ['{:02d}.png'.format(i) for i in range(12)]
        for image in names:
            write_png(source / image, lanes())
        for attempt in range(5):
            out = root / ('same_' + str(attempt))
            out.mkdir()
            competing_runs(program, source, out, deadline)
            summary(out, names)
    elif name == 'ascii':
        help_text = run(program, ['--help'], 0, deadline).decode('ascii')
        check('--in-dir' in help_text and '--out-dir' in help_text, 'Help thiếu chế độ thư mục')
        (root / 'output').mkdir()
        batch(program, root, [('lane.png', lanes())], deadline)
    else:
        raise AssertionError('Ca không tồn tại: ' + name)


def self_check(root):
    # Tự kiểm: phải Popen cả hai trước wait và dọn tiến trình khi hết hạn.
    from unittest.mock import Mock, patch
    for codes in ((0, 4), (4, 0), (0, 0), (4, 4), (0, 1), None):
        processes = [Mock(returncode=code) for code in (codes or (None, None))]
        for process in processes:
            process.poll.side_effect = lambda p=process: p.returncode
            process.kill.side_effect = lambda p=process: setattr(p, 'returncode', -9)
        with patch.object(subprocess, 'Popen', side_effect=processes) as launch:
            def wait(timeout):
                check(launch.call_count == 2, 'Đã wait trước Popen thứ hai')
                check(0 < timeout <= 30, 'Thiếu hạn giờ tiến trình con')
                if codes is None and any(p.returncode is None for p in processes):
                    raise subprocess.TimeoutExpired('binary', timeout)
            for process in processes:
                process.wait.side_effect = wait
            try:
                competing_runs('binary', root / 'input', root / 'output', time.monotonic() + 10)
            except (AssertionError, subprocess.TimeoutExpired):
                check(codes not in ((0, 4), (4, 0)), 'Từ chối nhầm mã [0, 4]')
            else:
                check(codes in ((0, 4), (4, 0)), 'Cho đạt mã thoát sai')
            check(all(p.poll() is not None for p in processes), 'Còn tiến trình sau khi ca hỏng')
    # Tự kiểm bộ sinh/giải PNG bằng mẫu lọc độc lập, cùng nhánh phát hiện đầu ra sai.
    image = root / 'lane.png'
    write_png(image, lanes())
    check(read_png(image)[:3] == (480, 288, 1) and read_png(image)[3] == lanes(), 'PNG không khứ hồi')
    large = enlarge(lanes())
    check(len(large) == 384 and len(large[0]) == 640 and large[250][220] == 255 and
          large[0][0] == 0 and any(0 < p < 255 for p in large[192]), 'Sai nội suy tuyến tính')
    # Mẫu mã hoá độc lập cho cả điểm trái, hàng trên và góc trên trái của 5 bộ lọc.
    first = [10, 20, 30, 40, 50, 60]
    filtered = [([*first], [15, 25, 35, 45, 55, 65]),
                ([10, 20, 30, 30, 30, 30], [15, 25, 35, 30, 30, 30]),
                ([*first], [5] * 6),
                ([10, 20, 30, 35, 40, 45], [10, 15, 20, 18, 18, 18]),
                ([10, 20, 30, 30, 30, 30], [5] * 6)]
    for mode, pair in enumerate(filtered):
        raw = b''.join(bytes([mode] + row) for row in pair)
        image.write_bytes(PNG + chunk(b'IHDR', struct.pack('>IIBBBBB', 2, 2, 8, 2, 0, 0, 0)) +
                          chunk(b'IDAT', zlib.compress(raw)) + chunk(b'IEND', b''))
        check(read_png(image)[3] == [bytes(first), bytes([15, 25, 35, 45, 55, 65])], 'Sai bộ lọc PNG')
    rgb = [bytearray(480 * 3) for _ in range(288)]
    for y in (172, 201, 230, 259):
        rgb[y][273 * 3:273 * 3 + 3] = b'\0\xff\0'
    raw = b''.join(b'\0' + row for row in rgb)
    image.write_bytes(PNG + chunk(b'IHDR', struct.pack('>IIBBBBB', 480, 288, 8, 2, 0, 0, 0)) +
                      chunk(b'IDAT', zlib.compress(raw)) + chunk(b'IEND', b''))
    check(centers(image, (480, 288)) == [273 / 480] * 4, 'Sai phép đo đường giữa')
    check_stdout(b'a.png 1.25 ms\nimages=1 average_ms=1.25\n', ['a.png'])
    out = root / 'summary'
    out.mkdir()
    names = ['a.png', 'same.jpg', 'same.png']
    with (out / 'summary.csv').open('w', encoding='utf-8', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(HEADER)
        writer.writerows((name, 1, 1, 1, 0.5) for name in names)
    for stem in ('a', 'same', 'same.png'):
        write_png(out / (stem + '_mask.png'), [bytearray(1)])
        (out / (stem + '_overlay.jpg')).write_bytes(b'\xff\xd8\xff\xd9')
    check(len(summary(out, names)) == 3, 'Sai tên theo stem hoặc xử lý trùng stem')
    for stem in ('a', 'same.png'):
        mask = out / (stem + '_mask.png')
        mask.rename(out / (stem + '.wrong_mask.png'))
        try:
            summary(out, names)
        except FileNotFoundError:
            pass
        else:
            raise AssertionError('Không phát hiện thiếu mask: ' + stem)
        (out / (stem + '.wrong_mask.png')).rename(mask)
    for bad in (b'a.png\nimages=1 average_ms=1\n', b'a.png 1 ms\n',
                b'a.png 1 ms\nimages=2 average_ms=1\n', b'\xff'):
        try:
            check_stdout(bad, ['a.png'])
        except (AssertionError, UnicodeDecodeError):
            continue
        raise AssertionError('Không phát hiện stdout sai')
    try:
        run(sys.executable, ['-c', 'import time; time.sleep(5)'], 0, time.monotonic() + 0.1)
    except subprocess.TimeoutExpired:
        pass
    else:
        raise AssertionError('Hạn tiến trình con không hoạt động')


def main():
    check(len(sys.argv) in (2, 3), 'Dùng: test_acceptance.py PROGRAM CASE | --self-check')
    with tempfile.TemporaryDirectory(prefix='ucr_lane_acceptance_') as folder:
        root = Path(folder)
        if sys.argv[1] == '--self-check':
            self_check(root)
        else:
            deadline = time.monotonic() + 90
            scenario(Path(sys.argv[1]).resolve(), sys.argv[2], root, deadline)
            check(time.monotonic() <= deadline, 'Hết hạn chung của ca')
    print('DAT: ' + sys.argv[-1])


if __name__ == '__main__':
    main()
