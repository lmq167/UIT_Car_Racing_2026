"""Test hộp đen cho ucr_collect_cam; chỉ thư viện chuẩn Python 3.6."""
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


COLUMNS = ['frame', 'monotonic_ns', 'wall_time_unix_ms', 'filename', 'width',
           'height', 'mean_luma', 'lap_var', 'dropped_since_last']
EXIT_CODES = dict(eof=0, max_frames=0, max_seconds=0, sigint=0, sigterm=0,
                  sighup=0, no_frame_timeout=3, stall_timeout=4, disk_low=5, error=1)
PARAMS = dict(interval_ms=1000, max_frames=0, max_seconds=0, warmup_frames=15,
              jpeg_quality=90, min_free_mb=1024, start_delay_s=0)
FLAGS = ['out-dir', 'out-root', 'label', 'who', 'weather', 'curtain', 'lights',
         'props', 'note', 'source', 'pipeline', 'interval-ms', 'max-frames',
         'max-seconds', 'start-delay-s', 'warmup-frames', 'jpeg-quality',
         'min-free-mb', 'help']
FORBIDDEN = r'Motor|Servo|IMU|ToF|Backend|OLED|app_core|I2C|UART|i2c|uart'
DEVICE_LIBS = r'(?i)motor|servo|imu|tof|backend|oled|app_core|wiringpi|pigpio|i2c|uart|gpiod|bno055|vl53|pca9685|sh1106'


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8'))


def csv_rows(out, live=False):
    path = out / 'metadata.csv'
    if not path.exists():
        return []
    lines = path.read_text(encoding='utf-8').splitlines(True)
    # Khi đang ghi, chỉ quan sát các dòng đã hoàn chỉnh; kiểm chặt sau khi thoát.
    if live:
        lines = [line for line in lines if line.endswith('\n')]
    reader = csv.DictReader(lines)
    if live and reader.fieldnames != COLUMNS:
        return []
    require(reader.fieldnames == COLUMNS, 'Sai thứ tự cột CSV: ' + repr(reader.fieldnames))
    return list(reader)


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
def running(args, root, hook=None, timeout_ms=2000):
    env = os.environ.copy()
    for key in ('UCR_CAM_TEST_SOURCE', 'UCR_CAM_TEST_TIMEOUT_MS', 'DISPLAY', 'WAYLAND_DISPLAY'):
        env.pop(key, None)
    if hook is not None:
        env.update(UCR_CAM_TEST_SOURCE=hook, UCR_CAM_TEST_TIMEOUT_MS=str(timeout_ms))
    with tempfile.TemporaryFile() as stdout, tempfile.TemporaryFile() as stderr:
        process = subprocess.Popen(args, cwd=str(root), env=env, stdout=stdout,
                                   stderr=stderr, start_new_session=True)
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


def execute(args, root, code=0, hook=None, timeout=35):
    started = time.monotonic()
    with running(args, root, hook) as (process, stderr):
        process.wait(timeout=timeout)
        error = stderr_text(stderr)
        valid = process.returncode != 0 if code is None else process.returncode == code
        require(valid, 'Sai mã thoát {} (cần {}): {}'.format(process.returncode, code, error))
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
        require(int(row['frame']) == index, 'Số frame không liên tục')
        stamp = int(row['monotonic_ns'])
        require(stamp > previous, 'Timestamp không tăng')
        previous = stamp
        require(int(row['wall_time_unix_ms']) > 0, 'Thiếu thời gian đồng hồ')
        require(Path(row['filename']).name == row['filename'], 'Tên ảnh không được thoát thư mục')
        require(row['filename'] not in filenames, 'Lặp tên ảnh')
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


def run_test(binary, scenario, nm, ldd, strings, hooks_binary):
    root = Path(tempfile.mkdtemp(prefix='ucr_cam_'))
    try:
        require(not re.search(r'\s', str(root)), 'Đường dẫn tạm phải không có dấu cách')
        # B15: kiểm bản xe, không dùng bản hook; cấm cả thư viện thiết bị trực tiếp/gián tiếp.
        if scenario == 'symbols':
            # đọc byte thô, bản hooks phải làm đối chứng có đủ hai tên biến.
            normal_bytes = Path(binary).read_bytes()
            hooks_bytes = Path(hooks_binary).read_bytes()
            for name in (b'UCR_CAM_TEST_SOURCE', b'UCR_CAM_TEST_TIMEOUT_MS'):
                require(name not in normal_bytes, 'Bản thường chứa biến hook: ' + name.decode('ascii'))
                require(name in hooks_bytes, 'Bản hooks thiếu biến đối chứng: ' + name.decode('ascii'))
            inspect_symbols(binary, nm, ldd, strings, root)
            return
        if scenario == 'symbols_harness':
            symbols_self_check(root)
            return
        source = make_source(root)
        out = root / 'output'
        base = [binary, '--source', str(source), '--out-dir', str(out), '--min-free-mb', '0']

        # B6/B8/B11: ảnh đổi cỡ, cột CSV, số liệu và JPEG nguyên vẹn sau max_frames.
        if scenario == 'frames':
            execute(base + ['--max-frames', '4', '--interval-ms', '40', '--jpeg-quality', '85'], root)
            summary, rows = inspect_output(out, 'max_frames', 0, 4)
            require(summary['empty_frames'] == 0, 'Sai thống kê khung rỗng')
            require([(int(r['width']), int(r['height'])) for r in rows] == [(32, 24), (40, 30)] * 2, 'Ảnh bị đổi cỡ')
            require((summary['width'], summary['height']) == (40, 30), 'Sai cỡ cuối')
            require(sorted(summary['frame_sizes'], key=lambda s: s['width']) == [
                {'width': 32, 'height': 24, 'count': 2}, {'width': 40, 'height': 30, 'count': 2}], 'Sai thống kê kích thước')
            for left, right in zip(rows, rows[1:]):
                require(int(right['monotonic_ns']) - int(left['monotonic_ns']) >= 40000000, 'Lưu ảnh quá dày')

        # A1/B8/B9/B11: phải thấy ít nhất một dòng CSV đã flush rồi mới gửi tín hiệu.
        elif scenario in ('signal_int', 'signal_term', 'signal_hup'):
            sig, reason = {'signal_int': (signal.SIGINT, 'sigint'),
                           'signal_term': (signal.SIGTERM, 'sigterm'),
                           'signal_hup': (signal.SIGHUP, 'sighup')}[scenario]
            with running(base + ['--interval-ms', '200'], root) as (process, stderr):
                poll(lambda: len(csv_rows(out, live=True)) >= 1, process, 10, 'Không có CSV đã flush trước tín hiệu')
                initial = read_json(out / 'session.json')
                require(initial['started_utc'] and initial['params'], 'Thiếu session lúc bắt đầu')
                for row in csv_rows(out, live=True):
                    data = (out / row['filename']).read_bytes()
                    require(data[:2] == b'\xff\xd8' and data[-2:] == b'\xff\xd9', 'CSV trỏ ảnh chưa xong')
                process.send_signal(sig)
                require(process.wait(timeout=20) == 0, 'Tín hiệu phải kết thúc mã 0: ' + stderr_text(stderr))
            _, rows = inspect_output(out, reason, 0)
            inspect_session(out)
            require(1 <= len(rows) < 100, 'Không dừng giữa phiên')

        # A1/B10/B13: lỗi pipeline phải rõ lý do, có gợi ý và không treo quá 5+10 s.
        elif scenario == 'pipeline':
            pipeline = 'ucr_missing_source ! videoconvert ! appsink'
            code, error, elapsed = execute([binary, '--out-dir', str(out), '--min-free-mb', '0',
                                            '--pipeline', pipeline], root, code=None, timeout=16)
            require(elapsed <= 15, 'Pipeline lỗi vượt 5 s + 10 s dung sai')
            require(pipeline in error and re.search(r'(?i)error|fail|cannot|could not|khong|không|lỗi|loi|timeout', error),
                    'Thiếu chuỗi pipeline và lý do: ' + error)
            require('pgrep -a main' in error, 'Thiếu gợi ý chương trình đang giữ camera')
            inspect_output(out, 'no_frame_timeout' if code == 3 else 'error', code, 0)

        # Test cũ/B6/B8: giữ kiểm tra nguồn GStreamer hợp lệ, bỏ warmup rõ ràng.
        elif scenario == 'pipeline_frames':
            pipeline = ('videotestsrc num-buffers=8 ! video/x-raw,width=80,height=60,format=BGR ! '
                        'appsink drop=false max-buffers=1 sync=false')
            with running([binary, '--out-dir', str(out), '--min-free-mb', '0', '--interval-ms', '0',
                          '--warmup-frames', '0', '--max-frames', '3', '--pipeline', pipeline], root) as (process, stderr):
                code = process.wait(timeout=35)
                error = stderr_text(stderr)
            if code == 0:
                _, rows = inspect_output(out, 'max_frames', 0, 3)
                require(all((int(row['width']), int(row['height'])) == (80, 60) for row in rows), 'Cỡ pipeline bị đổi')
            else:
                # Container có thể thiếu GStreamer; đường GStreamer thật được kiểm ở bản Ubuntu.
                require(code == 1 and 'Không mở/đọc được nguồn' in error,
                        'Pipeline trả kết quả ngoài dự kiến (mã {}): {}'.format(code, error))
                require(pipeline in error, 'Thiếu chuỗi pipeline trong stderr: ' + error)
                inspect_output(out, 'error', 1, 0)
            require(inspect_session(out)['pipeline_used'] == pipeline, 'Sai pipeline thực tế trong session')

        # B10/B8/B5: hook không trả khung hoặc dừng sau N khung; warmup bỏ đúng N.
        elif scenario in ('no_frame_timeout', 'stall_timeout', 'warmup'):
            hook = {'no_frame_timeout': 'never', 'stall_timeout': 'stall_after:8',
                    'warmup': 'stall_after:1000'}[scenario]
            warmup = 3 if scenario == 'warmup' else 0
            args = [binary, '--out-dir', str(out), '--min-free-mb', '0', '--interval-ms', '0',
                    '--warmup-frames', str(warmup)]
            if scenario == 'warmup':
                args += ['--max-frames', '5']
            code = 0 if scenario == 'warmup' else EXIT_CODES[scenario]
            _, _, elapsed = execute(args, root, code, hook, timeout=16)
            require(elapsed <= 2 + 2 + 10, 'Watchdog vượt hạn + 2 s dọn + 10 s dung sai tải')
            reason = 'max_frames' if scenario == 'warmup' else scenario
            expected = 0 if scenario == 'no_frame_timeout' else (5 if scenario == 'warmup' else None)
            summary, rows = inspect_output(out, reason, code, expected)
            if scenario == 'stall_timeout':
                require(1 <= len(rows) <= 8, 'Nguồn stall phải lưu được khung trước khi mất ảnh')
            if scenario == 'no_frame_timeout':
                require(elapsed >= 1.8, 'Watchdog thoát trước hạn hook 2000 ms')
            require(summary['warmup_dropped'] == warmup, 'Sai số khung warmup đã bỏ')

        # A1/B5/B8: PPM trong thư mục, không AVI/codec; thư mục bỏ qua warmup mặc định.
        elif scenario == 'eof':
            execute(base + ['--interval-ms', '0'], root)
            summary, _ = inspect_output(out, 'eof', 0, 100)
            require(summary['warmup_dropped'] == 0, 'Thư mục không được bỏ warmup')

        # bản thường bỏ qua cả nguồn never và hạn thử 500 ms.
        elif scenario == 'hooks_ignored':
            with running(base + ['--interval-ms', '0'], root,
                         hook='never', timeout_ms=500) as (process, stderr):
                require(process.wait(timeout=35) == 0,
                        'Bản thường kích hoạt hook: ' + stderr_text(stderr))
            inspect_output(out, 'eof', 0, 100)

        # B1/B3: kiểm mặc định từ JSON, không suy từ thời lượng chạy.
        elif scenario == 'defaults':
            execute([binary, '--source', str(source), '--max-frames', '1'], root)
            sessions = list((root / 'runs_cam').iterdir())
            require(len(sessions) == 1 and re.fullmatch(r'\d{8}_\d{6}_cam', sessions[0].name), 'Sai out-root/label mặc định')
            out = sessions[0]
            session = inspect_session(out)
            expected = dict(PARAMS, max_frames=1)
            require(all(session['params'][key] == value for key, value in expected.items()), 'Sai tham số mặc định')
            inspect_output(out, 'max_frames', 0, 1)

        # B2: đặt sẵn tên trong cửa sổ rộng để buộc xử lý trùng, không đua đồng hồ giây.
        elif scenario == 'out_root':
            parent = root / 'sessions'
            args = [binary, '--source', str(source), '--out-root', str(parent), '--label', 'L1c a/b',
                    '--min-free-mb', '0', '--interval-ms', '0', '--max-frames', '1']
            execute(args, root)
            first = list(parent.iterdir())
            require(len(first) == 1 and re.fullmatch(r'\d{8}_\d{6}_L1c_a_b', first[0].name), 'Sai tên thư mục/nhãn')
            timestamp = datetime.strptime(first[0].name[:15], '%Y%m%d_%H%M%S')
            sentinels = []
            for offset in range(-60, 61):
                name = (timestamp + timedelta(seconds=offset)).strftime('%Y%m%d_%H%M%S') + '_L1c_a_b'
                for suffix in ('', '_2', '_3'):
                    directory = parent / (name + suffix)
                    directory.mkdir(exist_ok=True)
                    marker = directory / 'keep'
                    marker.write_bytes(b'du lieu cu')
                    sentinels.append(marker)
            before = set(parent.iterdir())
            execute(args, root)
            added = set(parent.iterdir()) - before
            require(len(added) == 1 and next(iter(added)).name.endswith('_4'), 'Không thêm hậu tố khi trùng tên')
            require(all(path.read_bytes() == b'du lieu cu' for path in sentinels), 'Ghi đè thư mục cũ')
            inspect_output(next(iter(added)), 'max_frames', 0, 1)
            execute(base + ['--out-root', str(parent), '--max-frames', '1', '--interval-ms', '0'], root)
            inspect_output(out, 'max_frames', 0, 1)
            require(set(parent.iterdir()) == before | added, 'out-dir không ghi đè lựa chọn out-root')

        # B3: đủ khóa và giữ nguyên Unicode/nháy/gạch chéo/ký tự điều khiển.
        elif scenario == 'session':
            values = {key: 'Tiếng Việt "đèn" \\ rèm\nxuống dòng\t\x01 ' + key
                      for key in ('label', 'who', 'weather', 'curtain', 'lights', 'props', 'note')}
            args = base + ['--interval-ms', '0', '--max-frames', '1', '--warmup-frames', '7', '--jpeg-quality', '85']
            for key, value in values.items():
                args += ['--' + key, value]
            execute(args, root)
            session = inspect_session(out, values)
            require(session['pipeline_used'] == 'source:' + str(source), 'Sai nguồn thực tế trong session')
            expected = dict(PARAMS, interval_ms=0, max_frames=1, warmup_frames=7, jpeg_quality=85, min_free_mb=0)
            require(all(session['params'][key] == value for key, value in expected.items()), 'Sai params đã dùng')
            inspect_output(out, 'max_frames', 0, 1)

        # B4/B8: tự dừng theo đồng hồ đơn điệu, cho phép 10 s dung sai tải.
        elif scenario == 'max_seconds':
            _, _, elapsed = execute(base + ['--interval-ms', '200', '--max-seconds', '1'], root)
            require(0.9 <= elapsed <= 11, 'max-seconds không đúng hạn có dung sai')
            _, rows = inspect_output(out, 'max_seconds', 0)
            require(1 <= len(rows) < 100, 'Không ghi/dừng giữa phiên max-seconds')

        # B4: đếm lùi xuất hiện trên stderr trước ảnh; không sleep chờ cố định.
        elif scenario == 'start_delay':
            started = time.monotonic()
            with running(base + ['--start-delay-s', '1', '--max-frames', '1'], root) as (process, stderr):
                poll(lambda: bool(re.search(r'\b1\b', stderr_text(stderr))), process, 10, 'Không thấy đếm lùi stderr')
                require(process.wait(timeout=20) == 0, 'Không kết thúc sau đếm lùi')
            _, rows = inspect_output(out, 'max_frames', 0, 1)
            require(int(rows[0]['monotonic_ns']) / 1e9 >= started + 0.9, 'Lưu ảnh trước hết start-delay')
            require(inspect_session(out)['params']['start_delay_s'] == 1, 'Thiếu start_delay_s đã dùng')

        # B12/B8/B11: ngưỡng hợp lệ nhưng lớn hơn dung lượng máy, phải là lỗi ổ đĩa mã 5.
        elif scenario == 'disk':
            _, error, _ = execute(base + ['--min-free-mb', '1000000000000'], root, 5)
            require(error.strip(), 'Thiếu thông báo dung lượng thấp')
            inspect_output(out, 'disk_low', 5, 0)

        # CLI: mỗi đầu vào sai có cwd riêng; không được tạo bất kỳ đầu ra nào.
        elif scenario == 'invalid':
            bad_args = [['--width', '640'], ['--height', '384'], ['--unknown'], ['--out-dir'],
                        ['--jpeg-quality', '101'], ['--jpeg-quality', '0'], ['--interval-ms', '-1'],
                        ['--max-frames', '1x'], ['--max-frames', '-1'], ['--warmup-frames', '-1'],
                        ['--max-seconds', '-1'], ['--start-delay-s', '-1'],
                        ['--min-free-mb', '18446744073709551616'], ['--pipeline', ''],
                        ['--source', str(source), '--pipeline', 'videotestsrc ! appsink']]
            bad_args += [['--' + key] for key in FLAGS if key != 'help']
            deadline = time.monotonic() + 95
            for args in bad_args:
                case_root = Path(tempfile.mkdtemp(prefix='invalid_', dir=str(root)))
                prefix = [] if args == ['--out-dir'] else ['--out-dir', str(case_root / 'output')]
                remaining = deadline - time.monotonic()
                require(remaining > 0, 'Ca invalid vượt tổng hạn 95 s')
                execute([binary] + prefix + args, case_root, 2, timeout=min(20, remaining))
                require(not list(case_root.iterdir()), 'Tham số lỗi vẫn tạo đầu ra: ' + repr(args))

        # CLI: tên tham số đầy đủ, không có tác dụng phụ khi gọi help.
        elif scenario == 'help':
            result = subprocess.run([binary, '--help'], cwd=str(root), stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE, timeout=30)
            output = (result.stdout + result.stderr).decode('utf-8', errors='replace')
            require(result.returncode == 0, '--help phải thoát 0')
            require(all(re.search(r'--' + flag + r'(?![\w-])', output) for flag in FLAGS), 'Help thiếu tham số: ' + output)
            require(set(root.iterdir()) == {source}, '--help tạo đầu ra')

        # B8/B11: ảnh hỏng phải error/1, ghi nhận khung rỗng, không còn ảnh/tmp.
        elif scenario == 'corrupt':
            (source / '000.ppm').write_bytes(b'khong phai anh')
            execute(base + ['--max-frames', '1'], root, 1)
            summary, _ = inspect_output(out, 'error', 1, 0)
            require(summary['empty_frames'] >= 1, 'Không ghi nhận ảnh hỏng')

        # B2: tuyệt đối không ghi vào out-dir đã tồn tại.
        elif scenario == 'existing':
            out.mkdir()
            sentinel = out / 'metadata.csv'
            sentinel.write_bytes(b'du lieu cu')
            execute(base, root, code=None)
            require(sentinel.read_bytes() == b'du lieu cu' and set(out.iterdir()) == {sentinel}, 'Ghi đè dữ liệu đã có')
        else:
            raise AssertionError('Ca kiểm thử không biết: ' + scenario)
    finally:
        shutil.rmtree(str(root))


if __name__ == '__main__':
    run_test(str(Path(sys.argv[1]).resolve()), sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5],
             str(Path(sys.argv[6]).resolve()))
    print('PASS ' + sys.argv[2])
