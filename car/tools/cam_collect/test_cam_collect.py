"""T32: test hộp đen từ phiếu và quyết định 31/10; Python 3.6.

Kỳ vọng không suy từ main.cpp; lúc chạy M1/M2 chỉ vá neo trên bản sao tạm.
Hook preview stream dùng lại từ test drive.
H1/H2/H3 tự kiểm bộ kiểm thử; M1/M2 dựng bản lỗi trong thư mục tạm.
"""
import csv
from contextlib import contextmanager
from datetime import datetime, timedelta
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import struct
import subprocess
import sys
import tempfile
import time


# Python 3.6 chốt mã hóa argv lúc khởi động; đổi locale phải chạy lại Python.
os.environ.update(LC_ALL='C.UTF-8', LANG='C.UTF-8')
if sys.getfilesystemencoding().lower().replace('-', '') != 'utf8':
    if os.environ.get('UCR_CAM_TEST_UTF8_REEXEC') == '1':
        raise RuntimeError('Không thể bật mã hóa UTF-8 sau khi chạy lại Python')
    os.environ['UCR_CAM_TEST_UTF8_REEXEC'] = '1'
    os.execv(sys.executable, [sys.executable] + sys.argv)


# Các trường CaptureColumns và filename/ts_ms được công bố ở pass_store.h.
# Mẫu cố định: pass_tools.py PASS_COLUMNS; data_audit.py dùng pass/file/state/
# cmd_speed/cmd_steer. Hai consumer không công bố đủ header; phần còn lại chốt từ
# drive_collect/pass_store.cpp:24-26 và test_pass_store.cpp:79,115 trước khi chuyển.
COLUMNS = ['frame', 'monotonic_ns', 'wall_time_unix_ms', 'filename', 'width',
           'height', 'mean_luma', 'lap_var', 'dropped_since_last']
CSV_HEADERS = {
    'frames.csv': COLUMNS + ['cmd_speed', 'cmd_steer', 'state', 'pass', 'file'],
    'passes.csv': ['pass', 'event', 'ts_ms', 'frames', 'end_reason'],
    'sessions.csv': ['ts_ms', 'git', 'argv', 'label', 'pipeline'],
}
EXIT_CODES = dict(eof=0, max_frames=0, max_seconds=0, esc=0, sigint=0, sigterm=0,
                  sighup=0, no_frame_timeout=3, stall_timeout=4, disk_low=5, error=1)
PARAMS = dict(interval_ms=1000, max_frames=0, max_seconds=0, warmup_frames=15,
              jpeg_quality=90, min_free_mb=1024, start_delay_s=0)
FLAGS = ['out-dir', 'out-root', 'label', 'who', 'weather', 'curtain', 'lights',
         'props', 'note', 'source', 'pipeline', 'interval-ms', 'max-frames',
         'max-seconds', 'start-delay-s', 'warmup-frames', 'jpeg-quality',
         'min-free-mb', 'help']
FORBIDDEN = r'Motor|Servo|IMU|ToF|Backend|OLED|app_core|I2C|UART|i2c|uart'
DEVICE_LIBS = r'(?i)motor|servo|imu|tof|backend|oled|app_core|wiringpi|pigpio|i2c|uart|gpiod|bno055|vl53|pca9685|sh1106'
REC_DEFAULT = 'rec_default: không P vẫn ghi ảnh/lượt'
REC_OFF = 'rec_off: ảnh/frames/lượt thay đổi khi đã tắt P'
REC_OFF_ERROR = 'rec_off: saveFrame ngoài lượt làm thoát error/1'
SAVE_WITHOUT_PASS = 'T32_SAVE_WITHOUT_PASS'


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8'))


def table(out, name, live=False):
    path = out / name
    if not path.exists():
        return []
    lines = path.read_text(encoding='utf-8').splitlines(True)
    # Khi đang ghi, chỉ quan sát các dòng đã hoàn chỉnh; kiểm chặt sau khi thoát.
    if live and lines and not lines[-1].endswith('\n'):
        lines.pop()
    if live and not lines:
        return []
    reader = csv.DictReader(lines)
    require(reader.fieldnames and len(set(reader.fieldnames)) == len(reader.fieldnames),
            'Thiếu/trùng header ' + name)
    rows = list(reader)
    require(all(None not in row and all(v is not None for v in row.values()) for row in rows),
            'CSV thiếu/thừa giá trị: ' + name)
    return rows


def csv_rows(out, live=False):
    rows = table(out, 'frames.csv', live)
    require(all(set(COLUMNS) <= set(row) for row in rows), 'frames.csv thiếu cột capture')
    return rows


def fixed_headers(out, names=CSV_HEADERS, header_only=False):
    for name in names:
        with (out / name).open(encoding='utf-8', newline='') as stream:
            reader = csv.reader(stream)
            require(next(reader, None) == CSV_HEADERS[name], 'Sai header cố định: ' + name)
            if header_only:
                require(next(reader, None) is None, 'Có dữ liệu dù chưa P: ' + name)


def poll(predicate, process, seconds, message):
    deadline = time.monotonic() + seconds
    while True:
        if predicate():
            return
        require(process.poll() is None, message + ': tiến trình đã thoát')
        remaining = deadline - time.monotonic()
        require(remaining > 0, message)
        time.sleep(min(0.02, remaining))


@contextmanager
def running(args, root, hook=None, timeout_ms=2000, preview=None, preview_mode=None):
    env = os.environ.copy()
    for key in list(env):
        if key.startswith(('UCR_CAM_TEST_', 'UCR_DRIVE_FAKE_')) or key in ('DISPLAY', 'WAYLAND_DISPLAY'):
            env.pop(key, None)
    if hook is not None:
        env.update(UCR_CAM_TEST_SOURCE=hook, UCR_CAM_TEST_TIMEOUT_MS=str(timeout_ms))
    if preview is not None:
        env['UCR_DRIVE_FAKE_PREVIEW'] = 'stream:' + str(preview)
    if preview_mode is not None:
        env['UCR_DRIVE_FAKE_PREVIEW'] = preview_mode
    with tempfile.TemporaryFile() as stdout, tempfile.TemporaryFile() as stderr:
        process = subprocess.Popen(args, cwd=str(root), env=env, stdout=stdout,
                                   stderr=stderr, stdin=subprocess.DEVNULL,
                                   start_new_session=True)
        try:
            yield process, stderr
        finally:
            # Dọn cả nhóm con khi assertion/hạn giờ thất bại, kể cả cha đã thoát.
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=5)


def stderr_text(stream):
    # pread không đổi vị trí ghi dùng chung với tiến trình con.
    return os.pread(stream.fileno(), os.fstat(stream.fileno()).st_size, 0).decode('utf-8', errors='replace')


def execute(args, root, code=0, hook=None, timeout=35, preview_mode=None):
    started = time.monotonic()
    with running(args, root, hook, preview_mode=preview_mode) as (process, stderr):
        process.wait(timeout=timeout)
        error = stderr_text(stderr)
        valid = process.returncode != 0 if code is None else process.returncode == code
        require(valid, 'Sai mã thoát {} (cần {}): {}'.format(process.returncode, code, error))
        children_reaped(process)
        return process.returncode, error, time.monotonic() - started


def make_source(root, count=100):
    source = root / 'input'
    source.mkdir()
    for index in range(count):
        width, height = (32, 24) if index % 2 == 0 else (40, 30)
        (source / ('{:03d}.ppm'.format(index))).write_bytes(
            'P6\n{} {}\n255\n'.format(width, height).encode('ascii') +
            bytes((40, 80, index % 256)) * (width * height))
    return source


def inspect_output(out, reason, code, expected=None):
    summary = read_json(out / 'summary.json')
    keys = {'total_images', 'average_fps', 'duration_seconds', 'empty_frames',
            'width', 'height', 'exit_code', 'frame_sizes', 'exit_reason', 'read_fps',
            'save_fps', 'total_bytes', 'warmup_dropped', 'free_mb_start', 'free_mb_end'}
    require(keys <= set(summary), 'Thiếu khóa summary: ' + repr(keys - set(summary)))
    require(summary['exit_reason'] == reason and summary['exit_code'] == code == EXIT_CODES[reason],
            'Sai exit_reason/mã thoát: ' + repr(summary))
    rows = csv_rows(out)
    images = sorted(out.glob('*.jpg'))
    require(len(rows) == len(images) == summary['total_images'], 'Số ảnh/CSV/summary lệch nhau')
    if expected is not None:
        require(len(rows) == expected, 'Sai số lượng ảnh: {}'.format(len(rows)))
    require(not list(out.rglob('*.tmp')), 'Còn file tạm sau khi thoát')
    for key in ('average_fps', 'duration_seconds', 'read_fps', 'save_fps', 'total_bytes',
                'empty_frames', 'warmup_dropped', 'free_mb_start', 'free_mb_end'):
        require(math.isfinite(float(summary[key])) and summary[key] >= 0, 'Thống kê không hợp lệ: ' + key)
    previous = -1
    filenames = set()
    for index, row in enumerate(rows, 1):
        require(None not in row and all(value is not None for value in row.values()), 'CSV thiếu/thừa giá trị')
        require(int(row['frame']) == index, 'Số frame không liên tục toàn phiên')
        stamp = int(row['monotonic_ns'])
        require(stamp > previous, 'Timestamp không tăng')
        previous = stamp
        require(int(row['wall_time_unix_ms']) > 0, 'Thiếu thời gian đồng hồ')
        require(Path(row['filename']).name == row['filename'], 'Tên ảnh không được thoát thư mục')
        require(row['filename'] not in filenames, 'Lặp tên ảnh')
        require(re.fullmatch(r'p[0-9]{4,}_[0-9]{4,}\.jpg', row['filename']),
                'Tên ảnh không có nhãn lượt: ' + row['filename'])
        filenames.add(row['filename'])
        data = (out / row['filename']).read_bytes()
        require(data[:2] == b'\xff\xd8' and data[-2:] == b'\xff\xd9', 'JPEG không hoàn chỉnh')
        marker = data.index(b'\xff\xc0')
        height, width = struct.unpack('>HH', data[marker + 5:marker + 9])
        require((width, height) == (int(row['width']), int(row['height'])), 'Cỡ JPEG lệch metadata')
        require(width > 0 and height > 0, 'Thiếu kích thước')
        require(0 <= float(row['mean_luma']) <= 255, 'Độ sáng ngoài 0–255')
        require(math.isfinite(float(row['lap_var'])) and float(row['lap_var']) >= 0, 'Độ nét không hợp lệ')
        require(int(row['dropped_since_last']) >= 0, 'Số khung bỏ âm')
    require(summary['total_bytes'] == sum(path.stat().st_size for path in images), 'Sai tổng byte JPEG')
    return summary, rows


def inspect_session(out, values=None):
    session = read_json(out / 'session.json')
    keys = {'label', 'who', 'weather', 'curtain', 'lights', 'props', 'note',
            'started_utc', 'ended_utc', 'hostname', 'pipeline_used', 'version', 'params'}
    require(keys <= set(session), 'Thiếu khóa session: ' + repr(keys - set(session)))
    require(set(PARAMS) <= set(session['params']), 'Thiếu tham số trong session')
    for key in keys - {'params'}:
        require(isinstance(session[key], str), 'Trường session không phải chuỗi: ' + key)
    require(session['started_utc'] and session['ended_utc'] and session['hostname'], 'Thiếu thời gian/tên máy')
    for key, value in (values or {}).items():
        require(session[key] == value, 'Không giữ nguyên trường session: ' + key)
    return session


def inspect_symbols(binary, nm, ldd, strings, root):
    # nm rỗng phải có bằng chứng phụ; kiểm strings cả khi còn ký hiệu.
    stripped = False
    for tool, args in ((nm, ['-C']), (strings, []), (ldd, [])):
        result = subprocess.run([tool] + args + [binary], cwd=str(root),
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=30)
        output = result.stdout.decode(errors='replace')
        error = result.stderr.decode(errors='replace')
        no_symbols = bool(re.search('no symbols', output + error, re.I))
        require(result.returncode == 0 or
                (tool == nm and result.returncode == 1 and no_symbols),
                tool + ' thất bại: ' + error)
        if tool == nm:
            stripped = not output.strip() or no_symbols
        else:
            require(output.strip(), tool + ' không có dữ liệu làm bằng chứng')
        names = '\n'.join(Path(line.split()[0]).name for line in output.splitlines() if line.split())
        require(not re.search(DEVICE_LIBS if tool == ldd else FORBIDDEN,
                              names if tool == ldd else output), 'Có mã/thư viện thiết bị: ' + output)
        require(tool != ldd or 'not found' not in output.lower(), 'Thiếu thư viện khi chạy ldd')
    print('nm {}; strings và ldd có dữ liệu, không có từ cấm'.format(
        'rỗng/no symbols: đã kiểm bằng chứng phụ' if stripped else 'có ký hiệu'))


def symbols_self_check(root):
    # Tự kiểm cổng: dữ liệu rỗng/lỗi và mọi từ cấm đều phải bị phát hiện.
    from unittest.mock import patch
    clean = [(0, b'0000 T main\n', b''), (0, b'main\n', b''),
             (0, b'libc.so.6 => /lib/libc.so.6 (0x123)\n', b'')]

    def verify(replies, expected):
        results = [subprocess.CompletedProcess([], code, out, err) for code, out, err in replies]
        with patch.object(subprocess, 'run', side_effect=results):
            try:
                inspect_symbols('binary', 'nm', 'ldd', 'strings', root)
            except AssertionError:
                require(not expected, 'Từ chối nhầm bằng chứng hợp lệ')
            else:
                require(expected, 'Cho đạt bằng chứng thiếu hoặc có mã thiết bị')

    for nm_result in (clean[0], (0, b'', b''), (0, b'  \n', b''),
                      (0, b'', b'nm: no symbols\n'), (1, b'', b'nm: no symbols\n')):
        verify([nm_result] + clean[1:], True)
        for index in (1, 2):
            bad = [nm_result] + clean[1:]
            bad[index] = (0, b'', b'')
            verify(bad, False)
        for word in FORBIDDEN.split('|'):
            verify([nm_result, (0, word.encode('ascii') + b'\n', b''), clean[2]], False)
    for index in range(3):
        for word in FORBIDDEN.split('|'):
            bad = list(clean)
            bad[index] = (0, word.encode('ascii') + b'\n', b'')
            verify(bad, False)
        bad = list(clean)
        bad[index] = (2, b'', b'tool failed')
        verify(bad, False)
    verify(clean[:2] + [(0, b'libc.so.6 => not found\n', b'')], False)


def pass_events(out, event, live=False):
    # Tên cột sự kiện không có trong header công khai; ca format so toàn schema.
    return [r for r in table(out, 'passes.csv', live) if event in r.values()]


def check_passes(out, labels):
    rows = table(out, 'passes.csv')
    require(len(rows) == 2 * len(labels), 'Mỗi lượt cần đúng hai dòng start/end')
    for index, label in enumerate(labels):
        start, end = rows[2 * index:2 * index + 2]
        require(start['event'] == 'start' and end['event'] == 'end', 'Thiếu start/end hoặc sai thứ tự')
        require(start['pass'] == end['pass'] == label[1:], 'Sai số lượt: ' + label)


def off_snapshot(out):
    return (table(out, 'frames.csv', True), table(out, 'passes.csv', True),
            {str(p.relative_to(out)): p.read_bytes() for p in out.rglob('*.jpg')})


def default_off(snapshot):
    frames, passes, images = snapshot
    require(not frames and not passes and not images, REC_DEFAULT)


def off_guard(before, after):
    require(before == after, REC_OFF)


def observe_off(process, stderr, out, before, seconds):
    deadline = time.monotonic() + seconds
    while True:
        off_guard(before, off_snapshot(out))
        if process.poll() is not None:
            # Chỉ dấu tại saveFrame của bản sao M2; lỗi khởi động không có dấu này.
            if process.returncode == 1 and SAVE_WITHOUT_PASS in stderr_text(stderr).splitlines():
                inspect_output(out, 'error', 1)
                require(len(table(out, 'sessions.csv')) == 1, 'Lỗi thiết lập phiên M2')
                children_reaped(process)
                raise AssertionError(REC_OFF_ERROR)
            raise AssertionError('Thoát sớm khi REC tắt: ' + stderr_text(stderr))
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return
        time.sleep(min(0.02, remaining))


def observe(process, seconds, assertion):
    deadline = time.monotonic() + seconds
    while True:
        assertion()
        require(process.poll() is None, 'Thoát sớm trong khoảng quan sát tắt ghi')
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return
        time.sleep(min(0.02, remaining))


def send_preview(keys, value):
    # stream: đọc nối tiếp tệp; không đặt lịch phím theo sleep hay sửa preview.
    with keys.open('ab') as stream:
        stream.write(value.encode('ascii'))


def clean_exit(process, stderr, code=0, timeout=15):
    require(process.wait(timeout=timeout) == code,
            'Sai mã thoát (cần {}): {}'.format(code, stderr_text(stderr)))
    children_reaped(process)


def children_reaped(process):
    # Kiểm TRƯỚC finally dọn nhóm: không cho cleanup của test che rò Capture/GStreamer.
    deadline = time.monotonic() + 2
    while True:
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            return
        require(time.monotonic() < deadline, 'Còn tiến trình con trong nhóm sau khi cha thoát')
        time.sleep(0.02)


@contextmanager
def collecting(binary, root, args=(), source=None, hook=None):
    root.mkdir(parents=True, exist_ok=True)
    source = source or make_source(root)
    out, keys = root / 'output', root / 'preview_keys'
    keys.write_bytes(b'')
    require(b'UCR_DRIVE_FAKE_PREVIEW' in Path(binary).read_bytes(),
            'Thiếu hook preview đối chứng trong bản cam hooks; xem T32_cases.md')
    argv = [binary, '--out-dir', str(out), '--min-free-mb', '0',
            '--interval-ms', '100', '--warmup-frames', '0']
    if hook is None:
        argv += ['--source', str(source)]
    argv += list(args)
    with running(argv, root, hook=hook, preview=keys) as (process, stderr):
        poll(lambda: (out / 'sessions.csv').exists(), process, 10,
             'Không khởi tạo PassStore/sessions.csv')
        yield process, stderr, out, keys


def start_pass(process, out, keys, key='p', minimum=1):
    send_preview(keys, key)
    poll(lambda: len(csv_rows(out, True)) >= minimum, process, 8, 'P không mở lượt/ghi ảnh')


def stop_pass(process, out, keys, count=1):
    send_preview(keys, 'P')
    poll(lambda: len(pass_events(out, 'end', True)) == count, process, 5,
         'P không đóng lượt vào passes.csv')


def finish_preview(process, stderr, keys, key='E'):
    send_preview(keys, key)
    clean_exit(process, stderr)


def format_reference(out, root, argv, label, pass_store_source, pass_store_include):
    """So định dạng qua API công khai; không đọc nội dung nguồn sản xuất."""
    common = Path(pass_store_include).resolve()
    source = Path(pass_store_source).resolve()
    require(common.name == 'collect_common' and source.parent == common and
            source.name == 'pass_store.cpp', 'PassStore phải nằm trong collect_common')
    require(source.is_file() and (common / 'pass_store.h').is_file(),
            'Chưa có PassStore dùng chung sau DEV')
    fixed_headers(out)
    probe, binary, reference = root / 'schema.cpp', root / 'schema_probe', root / 'reference'
    probe.write_text(
        '#include "pass_store.h"\n#include <vector>\n#include <string>\n'
        '#include <fstream>\n#include <iterator>\n'
        'int main(int n, char** a) {\n'
        ' drive_collect::SessionInfo s; s.ts_ms=1000; s.label=a[2];\n'
        ' for(int i=4;i<n;++i) s.argv.emplace_back(a[i]);\n'
        ' std::ifstream f(a[3],std::ios::binary);\n'
        ' std::vector<unsigned char> jpeg((std::istreambuf_iterator<char>(f)),{});\n'
        ' drive_collect::PassStore p(a[1],s); if(!p.status()) return 1;\n'
        ' if(!p.beginPass(1001)) return 2;\n'
        ' drive_collect::CaptureColumns c; c.frame=1; c.monotonic_ns=1000000000;\n'
        ' c.width=32; c.height=24; c.state="DUNG";\n'
        ' if(!p.saveFrame(jpeg,1002,c)) return 3;\n'
        ' if(!p.endPass("exit",1003)) return 4; return 0;\n}\n', encoding='utf-8')
    compiler = os.environ.get('CXX', 'c++')
    command = [compiler, '-std=c++17', '-O0', '-g0', '-I', str(common), str(probe),
               str(source), '-lstdc++fs', '-o', str(binary)]
    # Cả compiler và probe đều có nhóm riêng để dọn con khi hết hạn.
    execute(command, root, timeout=25)
    image = out / csv_rows(out)[0]['filename']
    execute([str(binary), str(reference), label, str(image)] + list(argv), root, timeout=5)
    for name in ('frames.csv', 'passes.csv', 'sessions.csv'):
        with (reference / name).open(encoding='utf-8', newline='') as stream:
            expected = next(csv.reader(stream))
        with (out / name).open(encoding='utf-8', newline='') as stream:
            actual = next(csv.reader(stream))
        require(actual == expected, 'Sai thứ tự/toàn bộ cột {}: {} != {}'.format(name, actual, expected))
    expected_pass = table(reference, 'passes.csv')
    actual_pass = table(out, 'passes.csv')
    # Suy cột nhãn/sự kiện từ mẫu API thay vì đoán tên cột trong .cpp.
    for row, expected in zip(actual_pass, expected_pass):
        for key, value in expected.items():
            if value in ('p1', 'start', 'end'):
                require(row[key] == value, 'Sai cột nhãn/sự kiện: ' + key)
    expected_session = table(reference, 'sessions.csv')[0]
    actual_session = table(out, 'sessions.csv')
    require(len(actual_session) == 1, 'Phải có đúng một session')
    checked = []
    for key, value in expected_session.items():
        if value == label or (value and argv[0] in value):
            require(actual_session[0][key] == value, 'Sai label/argv trong sessions.csv: ' + key)
            checked.append(key)
    require(len(checked) >= 2, 'Mẫu PassStore không cung cấp đủ label và argv để đối chiếu')


def harness_oracle(case, root):
    """H1/H2 tự kiểm phép khẳng định; không tính là kiểm ngược sản phẩm."""
    out = root / 'output'
    out.mkdir()
    before = off_snapshot(out)
    default_off(before)
    off_guard(before, off_snapshot(out))
    if case == 'H1':
        (out / 'p0001_0001.jpg').write_bytes(b'anh ghi ngay khi mo')
        checker = lambda: default_off(off_snapshot(out))
        tag = 'rec_default:'
    else:
        # Mẫu tự kiểm có p1 đã đóng; không phải dữ liệu hợp đồng sản phẩm.
        with (out / 'passes.csv').open('w', encoding='utf-8', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerows([['test_only'], ['start'], ['end']])
        (out / 'p0001_0001.jpg').write_bytes(b'anh trong luot')
        before = off_snapshot(out)
        off_guard(before, off_snapshot(out))
        (out / 'p0001_0002.jpg').write_bytes(b'anh khi da tat')
        checker = lambda: off_guard(before, off_snapshot(out))
        tag = 'rec_off:'
    try:
        checker()
    except AssertionError as error:
        require(str(error).startswith(tag), 'Harness hỏng sai phép kiểm')
    else:
        raise AssertionError('Oracle bỏ lọt ' + case)
    print(case + ': oracle phát hiện dữ liệu lỗi; CHƯA kiểm ngược binary')


def patch_mutant(case, folder):
    """Neo do leader cấp; chỉ đọc/vá bản sao khi chạy kiểm ngược trên Linux."""
    path = folder / 'main.cpp'
    source = path.read_text(encoding='utf-8')
    gate = 'if (!store->active()) {'
    save = 'requireStore(store->saveFrame(encoded, h.wallMs, columns));'
    if case == 'M1':
        changes = [('bool recording = false;', 'bool recording = true;'),
                   (gate, 'if (false) {'),
                   (save, 'if (!store->active()) requireStore(store->beginPass(h.wallMs));\n'
                    '            ' + save)]
    else:
        changes = [(gate, 'if (false) {'),
                   (save, 'if (!store->active()) std::fputs("\\n' + SAVE_WITHOUT_PASS +
                    '\\n", stderr);\n            ' + save)]
    for anchor, replacement in changes:
        require(source.count(anchor) == 1, 'Neo kiểm ngược không khớp đúng một lần: ' + anchor)
        source = source.replace(anchor, replacement, 1)
    if case == 'M2':
        source = '#include <cstdio>\n' + source
    path.write_text(source, encoding='utf-8')


def mutation(case, root, binary, nm, ldd, strings, hooks_binary,
             pass_store_source, pass_store_include, cmake, cxx):
    scenario = {'M1': 'default_off', 'M2': 'off_guard'}[case]
    expected = {'M1': (REC_DEFAULT,), 'M2': (REC_OFF, REC_OFF_ERROR)}[case]
    # Bản sạch phải đạt toàn ca; không bắt lỗi baseline/build thành phát hiện.
    run_test(binary, scenario, nm, ldd, strings, hooks_binary,
             pass_store_source, pass_store_include)
    folder = root / 'cam_collect'
    original = Path(__file__).resolve().parent
    ignored = shutil.ignore_patterns('build*', '__pycache__', '*.pyc')
    shutil.copytree(str(original), str(folder), ignore=ignored)
    shutil.copytree(str(original.parent / 'collect_common'), str(root / 'collect_common'),
                    ignore=ignored)
    patch_mutant(case, folder)
    build = root / 'mutant-build'
    build.mkdir()
    configure = [cmake, '-DBUILD_TESTING=ON', '-DCMAKE_BUILD_TYPE=Release',
                 '-DCMAKE_CXX_FLAGS=-O0 -g0', '-DCMAKE_CXX_FLAGS_RELEASE=-DNDEBUG']
    if cxx:
        configure.append('-DCMAKE_CXX_COMPILER=' + cxx)
    configure.append(str(folder))
    deadline = time.monotonic() + 360
    # CMake 3.10: đúng một --target, -j1; không dựng bản xe hoặc chạy CTest con.
    for command in (configure, [cmake, '--build', '.', '--target',
                               'ucr_collect_cam_hooks', '--', '-j1']):
        remaining = deadline - time.monotonic()
        require(remaining > 0, 'Hết hạn dựng bản lỗi ' + case)
        execute(command, build, timeout=remaining)
    mutant = str(build / 'ucr_collect_cam_hooks')
    require(Path(mutant).is_file(), 'Không có binary kiểm ngược ' + case)
    try:
        run_test(mutant, scenario, nm, ldd, strings, mutant,
                 pass_store_source, pass_store_include)
    except AssertionError as error:
        require(str(error) in expected, 'Kiểm ngược hỏng sai oracle: ' + str(error))
        print('{} PHÁT HIỆN: {}'.format(case, error))
    else:
        raise AssertionError('Kiểm ngược sống sót: ' + case)


def run_test(binary, scenario, nm, ldd, strings, hooks_binary,
             pass_store_source, pass_store_include, cmake='cmake', cxx=None):
    with tempfile.TemporaryDirectory(prefix='ucr_cam_') as directory:
        root = Path(directory)
        if scenario in ('M1', 'M2'):
            mutation(scenario, root, binary, nm, ldd, strings, hooks_binary,
                     pass_store_source, pass_store_include, cmake, cxx)
            return
        if scenario in ('H1', 'H2'):
            harness_oracle(scenario, root)
            return
        if scenario == 'H3':
            symbols_self_check(root)
            return
        if scenario == 'symbols':
            normal, hooks = Path(binary).read_bytes(), Path(hooks_binary).read_bytes()
            for name in (b'UCR_CAM_TEST_SOURCE', b'UCR_CAM_TEST_TIMEOUT_MS', b'UCR_DRIVE_FAKE_PREVIEW'):
                require(name not in normal, 'Hook lọt bản thường: ' + repr(name))
                require(name in hooks, 'Bản hooks thiếu đối chứng: ' + repr(name))
            inspect_symbols(binary, nm, ldd, strings, root)
            return

        source = make_source(root)
        out = root / 'output'
        # T32b: bản thường không DISPLAY thoát 1 vì cửa sổ; các ca này cần cửa sổ giả còn sống.
        if scenario in ('default_off', 'corrupt', 'out_root'):
            binary = hooks_binary
        alive = 'ok' if binary == hooks_binary else None
        base = [binary, '--source', str(source), '--out-dir', str(out), '--min-free-mb', '0']

        # Quyết định 31/10: lỗi đĩa giữ 5; lỗi chung 1, tham số 2.
        if scenario in ('disk', 'disk_off'):
            execute(base + ['--min-free-mb', '1000000000000'], root, 5)
            inspect_output(out, 'disk_low', 5, 0)
            require(not table(out, 'passes.csv'), 'Đĩa thấp lúc mở vẫn tạo lượt')
            default_off(off_snapshot(out))
            return
        if scenario == 'invalid':
            bad = [['--unknown'], ['--out-dir'], ['--jpeg-quality', '101'],
                   ['--jpeg-quality', '0'], ['--interval-ms', '-1'], ['--max-frames', '1x'],
                   ['--max-frames', '-1'], ['--warmup-frames', '-1'], ['--max-seconds', '-1'],
                   ['--start-delay-s', '-1'], ['--min-free-mb', '18446744073709551616'],
                   ['--pipeline', ''], ['--source', str(source), '--pipeline', 'videotestsrc ! appsink']]
            bad += [['--' + flag] for flag in FLAGS if flag != 'help']
            deadline = time.monotonic() + 85
            for index, args in enumerate(bad):
                folder = root / str(index)
                folder.mkdir()
                remaining = deadline - time.monotonic()
                require(remaining > 0, 'Quá hạn ca invalid')
                execute([binary] + args, folder, 2, timeout=min(10, remaining))
                require(not list(folder.iterdir()), 'Tham số lỗi tạo đầu ra: ' + repr(args))
            return
        if scenario == 'help':
            result = subprocess.run([binary, '--help'], stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE, cwd=str(root), timeout=10)
            text = (result.stdout + result.stderr).decode('utf-8', errors='replace')
            require(result.returncode == 0 and all('--' + flag in text for flag in FLAGS),
                    '--help thiếu tham số/không thoát 0')
            require(set(root.iterdir()) == {source}, '--help có tác dụng phụ')
            return
        if scenario == 'existing':
            out.mkdir()
            marker = out / 'keep'
            marker.write_bytes(b'du lieu cu')
            execute(base, root, code=None)
            require(set(out.iterdir()) == {marker} and marker.read_bytes() == b'du lieu cu', 'Ghi đè dữ liệu')
            return
        if scenario == 'pipeline':
            pipeline = 'ucr_missing_source ! videoconvert ! appsink'
            code, error, _ = execute([binary, '--out-dir', str(out), '--min-free-mb', '0',
                                      '--pipeline', pipeline], root, code=None, timeout=16)
            require(code in (1, 3) and pipeline in error, 'Sai lỗi/chẩn đoán pipeline')
            inspect_output(out, 'error' if code == 1 else 'no_frame_timeout', code, 0)
            return
        if scenario == 'hooks_ignored':
            # Bản thường không đọc UCR_DRIVE_FAKE_PREVIEW nên không DISPLAY thì luôn lỗi cửa sổ (T32b).
            # Kiểm hành vi: có/không có hook nguồn 'never' (hook sẽ cho mã 3) kết quả phải như nhau.
            results = []
            for index, hook in enumerate((None, 'never')):
                folder = root / ('run' + str(index))
                folder.mkdir()
                run_out = folder / 'output'
                _, error, _ = execute([binary, '--source', str(source), '--out-dir', str(run_out),
                                       '--min-free-mb', '0', '--max-seconds', '3'],
                                      folder, 1, hook=hook, timeout=10)
                require('cửa sổ ảnh' in error, 'Lỗi không phải lỗi cửa sổ ảnh: ' + error)
                summary, _ = inspect_output(run_out, 'error', 1, 0)
                default_off(off_snapshot(run_out))
                results.append((summary['exit_reason'], summary['total_images']))
            require(results[0] == results[1], 'Bản thường đổi hành vi theo hook: ' + repr(results))
            return
        if scenario == 'default_off':
            with running(base + ['--max-seconds', '3', '--interval-ms', '100'], root,
                         preview_mode=alive) as (process, stderr):
                poll(lambda: (out / 'sessions.csv').exists(), process, 8, 'Chưa có sessions.csv')
                observe(process, 0.4, lambda: default_off(off_snapshot(out)))
                clean_exit(process, stderr)
            inspect_output(out, 'max_seconds', 0, 0)
            default_off(off_snapshot(out))
            return
        if scenario == 'no_frame_timeout':
            args = [hooks_binary, '--out-dir', str(out), '--min-free-mb', '0', '--warmup-frames', '0']
            _, _, elapsed = execute(args, root, 3, hook='never', timeout=16, preview_mode='ok')
            require(1.8 <= elapsed <= 14, 'Sai hạn watchdog 2000 ms với dung sai tải')
            inspect_output(out, 'no_frame_timeout', 3, 0)
            default_off(off_snapshot(out))
            return
        if scenario == 'corrupt':
            # Lỗi đọc nguồn không cần mở lượt; có thể xảy ra trước P đầu tiên.
            (source / '000.ppm').write_bytes(b'khong phai anh')
            execute(base, root, 1, preview_mode=alive)
            summary, _ = inspect_output(out, 'error', 1, 0)
            require(summary['empty_frames'] >= 1, 'Không ghi nhận ảnh hỏng')
            default_off(off_snapshot(out))
            return
        if scenario == 'defaults':
            keys = root / 'preview_keys'
            keys.write_bytes(b'')
            require(b'UCR_DRIVE_FAKE_PREVIEW' in Path(hooks_binary).read_bytes(), 'Thiếu hook preview')
            argv = [hooks_binary, '--source', str(source), '--min-free-mb', '0', '--max-frames', '3']
            with running(argv, root, preview=keys) as (process, stderr):
                def session_ready():
                    return list((root / 'runs_cam').glob('*/sessions.csv'))
                poll(session_ready, process, 10, 'Không có phiên trong runs_cam mặc định')
                paths = session_ready()
                require(len(paths) == 1, 'Sai số phiên mặc định')
                out = paths[0].parent
                require(re.fullmatch(r'\d{8}_\d{6}_cam', out.name), 'Sai tên phiên mặc định')
                default_off(off_snapshot(out))
                start_pass(process, out, keys)
                clean_exit(process, stderr)
            _, rows = inspect_output(out, 'max_frames', 0, 3)
            check_passes(out, ['p1'])
            session = inspect_session(out)
            expected = dict(PARAMS, min_free_mb=0, max_frames=3)
            require(all(session['params'][k] == v for k, v in expected.items()), 'Sai params mặc định')
            for left, right in zip(rows, rows[1:]):
                gap = (int(right['monotonic_ns']) - int(left['monotonic_ns'])) / 1e6
                require(900 <= gap <= 2500, 'Nhịp mặc định ngoài 1000 ms + dung sai: ' + str(gap))
            return
        if scenario == 'out_root':
            parent = root / 'sessions'
            argv = [binary, '--source', str(source), '--out-root', str(parent),
                    '--label', 'L1c a/b', '--min-free-mb', '0', '--max-seconds', '1']
            execute(argv, root, preview_mode=alive)
            first = list(parent.iterdir())
            require(len(first) == 1 and re.fullmatch(r'\d{8}_\d{6}_L1c_a_b', first[0].name), 'Sai tên phiên')
            stamp = datetime.strptime(first[0].name[:15], '%Y%m%d_%H%M%S')
            markers = []
            for offset in range(-30, 31):
                name = (stamp + timedelta(seconds=offset)).strftime('%Y%m%d_%H%M%S') + '_L1c_a_b'
                for suffix in ('', '_2', '_3'):
                    folder = parent / (name + suffix)
                    folder.mkdir(exist_ok=True)
                    marker = folder / 'keep'
                    marker.write_bytes(b'cu')
                    markers.append(marker)
            before = set(parent.iterdir())
            execute(argv, root, preview_mode=alive)
            added = set(parent.iterdir()) - before
            require(len(added) == 1 and next(iter(added)).name.endswith('_4'), 'Không tránh trùng tên')
            require(all(p.read_bytes() == b'cu' for p in markers), 'Đã sửa phiên cũ')
            execute(base + ['--out-root', str(parent), '--max-seconds', '1'], root, preview_mode=alive)
            require(set(parent.iterdir()) == before | added and out.exists(), 'Sai ưu tiên out-dir')
            return

        # Những ca còn lại cần P từ preview. Không đặt ra CLI/hook mới.
        hook = {'stall_timeout': 'stall_after:100', 'warmup': 'stall_after:1000'}.get(scenario)
        args = []
        if scenario in ('frames', 'format'):
            args = ['--max-frames', '4' if scenario == 'frames' else '1', '--interval-ms', '40']
        elif scenario == 'max_frames_across_passes':
            args = ['--max-frames', '6', '--interval-ms', '400']
        elif scenario == 'warmup':
            args = ['--max-frames', '5', '--warmup-frames', '3', '--interval-ms', '0']
        elif scenario == 'eof':
            args = ['--interval-ms', '0']
        elif scenario == 'max_seconds':
            args = ['--max-seconds', '4']
        elif scenario == 'start_delay':
            args = ['--start-delay-s', '2', '--max-frames', '1']
        elif scenario == 'off_guard':
            args = ['--interval-ms', '100']
        elif scenario == 'pipeline_frames':
            args = ['--pipeline', 'videotestsrc ! video/x-raw,width=80,height=60,format=BGR ! '
                    'appsink drop=true max-buffers=1 sync=false', '--max-frames', '3']
        values = {}
        if scenario in ('session', 'format'):
            values = {key: 'Tiếng Việt "đèn" \\ rèm\nxuống dòng\t ' + key
                      for key in ('label', 'who', 'weather', 'curtain', 'lights', 'props', 'note')}
            args += ['--max-frames', '1']
            for key, value in values.items():
                args += ['--' + key, value]
        session_root = root / 'run'
        # pipeline có nguồn riêng; dùng context riêng để không truyền --source cùng pipeline.
        if scenario == 'pipeline_frames':
            session_root.mkdir()
            keys = session_root / 'keys'
            keys.write_bytes(b'')
            out = session_root / 'output'
            require(b'UCR_DRIVE_FAKE_PREVIEW' in Path(hooks_binary).read_bytes(), 'Thiếu hook preview')
            with running([hooks_binary, '--out-dir', str(out), '--min-free-mb', '0',
                          '--warmup-frames', '0'] + args, session_root, preview=keys) as (process, stderr):
                try:
                    poll(lambda: (out / 'sessions.csv').exists(), process, 8, 'Không mở phiên pipeline')
                    start_pass(process, out, keys)
                    clean_exit(process, stderr)
                except AssertionError:
                    if process.poll() is None:
                        raise
                    error = stderr_text(stderr)
                    require(process.returncode == 1 and 'Không mở/đọc được nguồn' in error,
                            'Pipeline trả kết quả ngoài dự kiến: ' + error)
                    require(args[1] in error, 'Thiếu chuỗi pipeline trong stderr: ' + error)
                    children_reaped(process)
                    inspect_output(out, 'error', 1, 0)
                    require(inspect_session(out)['pipeline_used'] == args[1], 'Sai pipeline thực tế')
                    raise AssertionError('CHƯA KIỂM pipeline_frames: cần backend GStreamer; '
                                         'thật sự chạy được ở Ubuntu/xe. stderr: ' + error)
            _, rows = inspect_output(out, 'max_frames', 0, 3)
            require(all((int(r['width']), int(r['height'])) == (80, 60) for r in rows),
                    'Cỡ pipeline bị đổi')
            require(inspect_session(out)['pipeline_used'] == args[1], 'Sai pipeline thực tế')
            check_passes(out, ['p1'])
            return

        started = time.monotonic()
        with collecting(hooks_binary, session_root, args, source=source, hook=hook) as run:
            process, stderr, out, keys = run
            if scenario == 'esc_no_pass':
                default_off(off_snapshot(out))
                finish_preview(process, stderr, keys)
                inspect_output(out, 'esc', 0, 0)
                default_off(off_snapshot(out))
                optional = [name for name in ('passes.csv', 'frames.csv') if (out / name).exists()]
                fixed_headers(out, optional, header_only=True)
                fixed_headers(out, ('sessions.csv',))
                require(len(table(out, 'sessions.csv')) == 1, 'Phải có đúng một session khi Esc chưa P')
                return
            if scenario in ('pass_toggle', 'off_guard', 'pass_two', 'max_frames_across_passes'):
                if scenario == 'off_guard':
                    observe_off(process, stderr, out, ([], [], {}), 0.45)
                else:
                    default_off(off_snapshot(out))
                start_pass(process, out, keys)
                stop_pass(process, out, keys)
                check_passes(out, ['p1'])
                before = off_snapshot(out)
                # Nhịp 100 ms thường; ca tổng ảnh dùng 400 ms và quan sát > 2 nhịp.
                observe_off(process, stderr, out, before,
                            0.9 if scenario == 'max_frames_across_passes' else 0.45)
                if scenario in ('pass_two', 'max_frames_across_passes'):
                    saved = len(csv_rows(out, True))
                    if scenario == 'max_frames_across_passes':
                        require(saved < 6, 'Thiết lập: lượt đầu đã chạm max-frames trước P tắt')
                    start_pass(process, out, keys, key='P', minimum=saved + 1)
                    if scenario == 'pass_two':
                        stop_pass(process, out, keys, 2)
                    else:
                        clean_exit(process, stderr)
                    check_passes(out, ['p1', 'p2'])
                    names = [r['filename'] for r in csv_rows(out, True)]
                    require('p0001_0001.jpg' in names and 'p0002_0001.jpg' in names,
                            'Không đặt lại số ảnh trong tên theo lượt mới')
                if scenario != 'max_frames_across_passes':
                    finish_preview(process, stderr, keys)
            else:
                if scenario == 'start_delay':
                    send_preview(keys, 'p')
                    # Khẳng định liên tục trước hạn; chờ theo đồng hồ đơn điệu.
                    until = started + 1.8
                    while time.monotonic() < until:
                        require(not list(out.glob('*.jpg')), 'Ghi trước hết start-delay')
                        require(process.poll() is None, 'Thoát trước start-delay')
                        time.sleep(0.02)
                else:
                    start_pass(process, out, keys)
                if scenario in ('escape', 'preview_ctrl_c'):
                    finish_preview(process, stderr, keys, 'E' if scenario == 'escape' else '3')
                elif scenario in ('signal_int', 'signal_term', 'signal_hup'):
                    sig = {'signal_int': signal.SIGINT, 'signal_term': signal.SIGTERM,
                           'signal_hup': signal.SIGHUP}[scenario]
                    process.send_signal(sig)
                    clean_exit(process, stderr)
                else:
                    clean_exit(process, stderr, 4 if scenario == 'stall_timeout' else 0)

        reason = {'escape': 'esc', 'preview_ctrl_c': 'sigint', 'signal_int': 'sigint',
                  'signal_term': 'sigterm', 'signal_hup': 'sighup', 'pass_toggle': 'esc',
                  'pass_two': 'esc', 'off_guard': 'esc', 'eof': 'eof',
                  'stall_timeout': 'stall_timeout', 'max_seconds': 'max_seconds'}.get(scenario, 'max_frames')
        count = {'frames': 4, 'format': 1, 'session': 1, 'warmup': 5,
                 'start_delay': 1, 'eof': 100, 'max_frames_across_passes': 6}.get(scenario)
        summary, rows = inspect_output(out, reason, EXIT_CODES[reason], count)
        check_passes(out, ['p1', 'p2'] if scenario in ('pass_two', 'max_frames_across_passes') else ['p1'])
        session = inspect_session(out, values)
        if scenario == 'frames':
            interval = 40
            require(session['params']['interval_ms'] == interval, 'Sai nhịp đã dùng')
            for left, right in zip(rows, rows[1:]):
                gap = (int(right['monotonic_ns']) - int(left['monotonic_ns'])) / 1e6
                require(interval - 20 <= gap <= interval + 2000, 'Nhịp lưu ngoài cận dung sai: ' + str(gap))
        if scenario == 'warmup':
            require(summary['warmup_dropped'] == 3, 'Sai số khung warmup')
        if scenario == 'start_delay':
            require(int(rows[0]['monotonic_ns']) / 1e9 >= started + 1.8, 'Lưu trước start-delay')
        if scenario in ('session', 'format'):
            # sessions.csv giữ label và argv; metadata khác vẫn ở session.json, không thêm cột.
            format_reference(out, root, [hooks_binary, '--out-dir', str(out), '--min-free-mb', '0',
                             '--interval-ms', '100', '--warmup-frames', '0', '--source', str(source)] + args,
                             values['label'], pass_store_source, pass_store_include)


def preview_failure(binary, scenario):
    # T32b mục 1/2: lỗi cửa sổ phải kết thúc phiên, kể cả khi chưa bấm P.
    modes = {'preview_no_display': None, 'preview_fail_start': 'fail_start',
             'preview_die_off': 'die_after_frame',
             'preview_close_off': 'close_after_frame'}
    with tempfile.TemporaryDirectory(prefix='ucr_cam_t32b_') as directory:
        root = Path(directory)
        source = make_source(root)
        out = root / 'output'
        args = [binary, '--out-dir', str(out), '--source', str(source),
                '--min-free-mb', '0', '--warmup-frames', '0',
                '--interval-ms', '100', '--max-seconds', '20']
        started = time.monotonic()
        with running(args, root, preview_mode=modes[scenario]) as (process, stderr):
            require(process.wait(timeout=5) == 1,
                    'Lỗi cửa sổ phải thoát mã 1: ' + stderr_text(stderr))
            require(time.monotonic() - started <= 5,
                    'Lỗi cửa sổ không kết thúc trong 5 giây')
            if scenario in ('preview_no_display', 'preview_fail_start'):
                require('cửa sổ ảnh' in stderr_text(stderr),
                        'Thiếu chẩn đoán tiếng Việt cửa sổ ảnh: ' + stderr_text(stderr))
            children_reaped(process)
        inspect_output(out, 'error', 1, 0)
        default_off(off_snapshot(out))
        inspect_session(out)


if __name__ == '__main__':
    require(os.name == 'posix', 'Ca runtime cần Linux/POSIX; Windows chỉ kiểm cú pháp')
    # Kiểm ngược: 360 s dựng + 200 s baseline/probe + 20 s dọn; CTest cho 600 s.
    limit = 580 if sys.argv[2] in ('M1', 'M2') else 100
    def expired(signum, frame):
        raise AssertionError('Ca vượt hạn tổng {} s'.format(limit))
    signal.signal(signal.SIGALRM, expired)
    signal.alarm(limit)
    try:
        if sys.argv[2] in ('preview_no_display', 'preview_fail_start',
                           'preview_die_off', 'preview_close_off'):
            preview_failure(str(Path(sys.argv[1]).resolve()), sys.argv[2])
        else:
            run_test(str(Path(sys.argv[1]).resolve()), sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5],
                     str(Path(sys.argv[6]).resolve()), sys.argv[7], sys.argv[8],
                     sys.argv[9] if len(sys.argv) > 9 else 'cmake',
                     sys.argv[10] if len(sys.argv) > 10 else None)
    finally:
        signal.alarm(0)
    print('PASS ' + sys.argv[2])
