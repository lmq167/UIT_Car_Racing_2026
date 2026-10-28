"""Kiểm thử bằng ảnh PPM và video AVI tự sinh, chỉ cần thư viện chuẩn Python 3.6."""
import csv
import json
import os
from pathlib import Path
import re
import signal
import struct
import subprocess
import sys
import tempfile
import time


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def riff_chunk(tag, data):
    return tag + struct.pack('<I', len(data)) + data + (b'\0' if len(data) % 2 else b'')


def make_video(path):
    # AVI RGB 24 bit không nén, không cần codec hay gói Python ngoài.
    width, height, count = 32, 24, 12
    size = width * height * 3
    avih = struct.pack('<14I', 40000, size * 25, 0, 0, count, 0, 1, size,
                       width, height, 0, 0, 0, 0)
    strh = struct.pack('<4s4sIHH8I4h', b'vids', b'DIB ', 0, 0, 0,
                       0, 1, 25, 0, count, size, 0xffffffff, 0, 0, 0, width, height)
    strf = struct.pack('<IiiHHIIiiII', 40, width, height, 1, 24, 0, size, 0, 0, 0, 0)
    stream = riff_chunk(b'LIST', b'strl' + riff_chunk(b'strh', strh) + riff_chunk(b'strf', strf))
    header = riff_chunk(b'LIST', b'hdrl' + riff_chunk(b'avih', avih) + stream)
    frames = b''.join(riff_chunk(b'00db', bytes((i * 10, 80, 160)) * (width * height)) for i in range(count))
    path.write_bytes(riff_chunk(b'RIFF', b'AVI ' + header + riff_chunk(b'LIST', b'movi' + frames)))


def inspect_output(out, expected=None):
    summary = json.loads((out / 'summary.json').read_text())
    with (out / 'metadata.csv').open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    images = sorted(out.glob('*.jpg'))
    require(len(rows) == len(images) == summary['total_images'], 'Số ảnh/CSV/summary lệch nhau')
    if expected is not None:
        require(len(rows) == expected, 'Sai số lượng ảnh')
    require(summary['duration_seconds'] > 0, 'Thiếu thời gian chạy')
    require(summary['average_fps'] >= 0, 'FPS âm')
    previous = -1
    for index, row in enumerate(rows, 1):
        require(int(row['frame']) == index, 'Số frame không liên tục')
        stamp = int(row['monotonic_ns'])
        require(stamp > previous, 'Timestamp không tăng')
        previous = stamp
        require(int(row['wall_time_unix_ms']) > 0, 'Thiếu thời gian đồng hồ')
        data = (out / row['filename']).read_bytes()
        require(data[:2] == b'\xff\xd8' and data[-2:] == b'\xff\xd9', 'JPEG không hoàn chỉnh')
        require(int(row['width']) > 0 and int(row['height']) > 0, 'Thiếu kích thước')
        marker = data.index(b'\xff\xc0')
        height, width = struct.unpack('>HH', data[marker + 5:marker + 9])
        require((width, height) == (int(row['width']), int(row['height'])), 'Cỡ JPEG lệch metadata')
    return summary, rows


def run_test(binary, scenario, nm):
    if scenario == 'symbols':
        result = subprocess.run([nm, '-C', binary], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10)
        require(result.returncode == 0, 'nm thất bại: ' + result.stderr.decode(errors='replace'))
        require(not re.search(r'Motor|Servo|IMU|ToF|Backend', result.stdout.decode(errors='replace')), 'Có mã thiết bị trong binary')
        return
    with tempfile.TemporaryDirectory(prefix='cam collect ') as temp:
        root = Path(temp)
        source = root / 'input'
        source.mkdir()
        for index in range(100):
            width, height = (32, 24) if index % 2 == 0 else (40, 30)
            (source / ('{:03d}.ppm'.format(index))).write_bytes(
                'P6\n{} {}\n255\n'.format(width, height).encode() + bytes((40, 80, index)) * (width * height))
        out = root / 'output'
        base = [binary, '--source', str(source), '--out-dir', str(out), '--min-free-mb', '0']

        def execute(args, code=0):
            process = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
            try:
                stdout, stderr = process.communicate(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.communicate(timeout=2)
                raise
            result = subprocess.CompletedProcess(args, process.returncode, stdout, stderr)
            require((result.returncode == 0) == (code == 0),
                    'Sai mã thoát {}: {}'.format(result.returncode, result.stderr.decode(errors='replace')))
            return result

        if scenario == 'frames':
            execute(base + ['--max-frames', '4', '--interval-ms', '40', '--jpeg-quality', '85'])
            summary, rows = inspect_output(out, 4)
            require(summary['exit_code'] == 0 and summary['empty_frames'] == 0, 'Sai thống kê')
            require([(int(r['width']), int(r['height'])) for r in rows] == [(32, 24), (40, 30)] * 2, 'Ảnh bị đổi cỡ')
            require(summary['width'] == 40 and summary['height'] == 30, 'Sai cỡ cuối')
            require(sorted(summary['frame_sizes'], key=lambda s: s['width']) == [
                {'width': 32, 'height': 24, 'count': 2}, {'width': 40, 'height': 30, 'count': 2}], 'Sai thống kê kích thước')
            for left, right in zip(rows, rows[1:]):
                require(int(right['monotonic_ns']) - int(left['monotonic_ns']) >= 40000000, 'Lưu ảnh quá dày')
        elif scenario in ('signal_int', 'signal_term'):
            if scenario == 'signal_term':
                video = root / 'signal.avi'
                make_video(video)
                base[2] = str(video)
            with (root / 'process.log').open('wb') as log:
                process = subprocess.Popen(base + ['--interval-ms', '200'], stdout=log, stderr=log, start_new_session=True)
                try:
                    deadline = time.monotonic() + 8
                    ready = False
                    while time.monotonic() < deadline and process.poll() is None:
                        csv_path = out / 'metadata.csv'
                        if csv_path.exists() and len(csv_path.read_text().splitlines()) >= 3:
                            ready = True
                            break
                        time.sleep(0.02)
                    require(ready, 'Không có CSV đã flush trước tín hiệu')
                    process.send_signal(signal.SIGINT if scenario == 'signal_int' else signal.SIGTERM)
                    require(process.wait(timeout=4) == 0, 'Tín hiệu phải kết thúc mã 0')
                finally:
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=2)
            summary, rows = inspect_output(out)
            require(2 <= len(rows) < 100 and summary['exit_code'] == 0, 'Không dừng giữa phiên')
        elif scenario == 'disk':
            result = execute(base + ['--min-free-mb', '18446744073709551615'], 1)
            require('chỗ trống' in result.stderr.decode(), 'Thiếu lý do từ chối dung lượng')
            require(not list(out.glob('*.jpg')), 'Vẫn ghi ảnh khi ổ thiếu chỗ')
        elif scenario == 'invalid':
            for args in (['--width', '640'], ['--height', '384'], ['--unknown'], ['--out-dir'],
                         ['--jpeg-quality', '101'], ['--interval-ms', '-1'], ['--max-frames', '1x'],
                         ['--min-free-mb', '18446744073709551616'], ['--pipeline', ''],
                         ['--source', str(source), '--pipeline', 'videotestsrc ! appsink']):
                execute([binary] + args, 1)
            require(not out.exists(), 'CLI lỗi vẫn tạo đầu ra')
        elif scenario == 'pipeline':
            pipeline = 'gst-launch-1.0 -e ucr_missing_source ! fakesink'
            started = time.monotonic()
            result = execute([binary, '--out-dir', str(out), '--min-free-mb', '0', '--pipeline', pipeline], 1)
            error = result.stderr.decode(errors='replace')
            require(time.monotonic() - started < 7, 'Mở pipeline quá 5 giây cộng thời gian dọn')
            require('ucr_missing_source' in error and 'appsink' in error and 'gst-launch' not in error,
                    'Thiếu pipeline thực tế đã dựng')
            inspect_output(out, 0)
            # Không coi lỗi thiếu GStreamer là đã kiểm được thời hạn đọc khung đầu.
            blocked = root / 'blocked'
            started = time.monotonic()
            result = execute([binary, '--out-dir', str(blocked), '--min-free-mb', '0', '--pipeline',
                              'videotestsrc is-live=true ! valve drop=true ! videoconvert ! video/x-raw,format=BGR ! appsink'], 1)
            require(4.5 <= time.monotonic() - started < 7, 'Không kiểm được mốc hết hạn 5 giây')
            require('5 giây' in result.stderr.decode(errors='replace'), 'Nguồn bị chặn phải báo hết hạn')
            inspect_output(blocked, 0)
            camera_out = root / 'pipeline-output'
            execute([binary, '--out-dir', str(camera_out), '--min-free-mb', '0', '--interval-ms', '0',
                     '--max-frames', '3', '--pipeline',
                     'videotestsrc num-buffers=8 ! video/x-raw,width=80,height=60,format=BGR ! appsink drop=false max-buffers=1 sync=false'])
            _, rows = inspect_output(camera_out, 3)
            require(all((int(r['width']), int(r['height'])) == (80, 60) for r in rows), 'Cỡ pipeline bị đổi')
        elif scenario == 'eof':
            video = root / 'input.avi'
            make_video(video)
            video_out = root / 'video-output'
            execute([binary, '--source', str(video), '--out-dir', str(video_out), '--min-free-mb', '0',
                     '--interval-ms', '10', '--max-frames', '0'])
            summary, rows = inspect_output(video_out, 12)
            require(all((int(r['width']), int(r['height'])) == (32, 24) for r in rows), 'Sai cỡ video')
            # Thư mục hết ảnh: max-frames=0 kết thúc tự nhiên.
            execute(base + ['--interval-ms', '0'])
            inspect_output(out, 100)
        elif scenario == 'corrupt':
            (source / '000.ppm').write_bytes(b'khong phai anh')
            execute(base + ['--max-frames', '1'], 1)
            summary, _ = inspect_output(out, 0)
            require(summary['empty_frames'] == 1 and summary['exit_code'] != 0, 'Không ghi nhận ảnh hỏng')
        elif scenario == 'existing':
            out.mkdir()
            sentinel = out / 'metadata.csv'
            sentinel.write_text('du lieu cu')
            execute(base, 1)
            require(sentinel.read_text() == 'du lieu cu', 'Ghi đè dữ liệu đã có')
        else:
            raise AssertionError('Ca kiểm thử không biết: ' + scenario)


if __name__ == '__main__':
    run_test(str(Path(sys.argv[1]).resolve()), sys.argv[2], sys.argv[3])
    print('PASS ' + sys.argv[2])
