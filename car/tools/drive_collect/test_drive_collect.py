"""T21 §3/§4/§7–§13: kiểm hộp đen bằng PTY, chỉ chạy bản phần cứng giả.

Python 3.6; mỗi ca có khóa/vết/dữ liệu riêng. Không chạy binary xe thật.
Release (bản tối ưu) giữ trần 200 ms; sanitizer (dò lỗi bộ nhớ/luồng)
được cộng 300 ms theo thiết kế §8.
"""
import argparse
from contextlib import contextmanager
import csv
import io
import json
import math
import os
from pathlib import Path
import re
import select
import shlex
import shutil
import signal
import struct
import subprocess
import sys
import tempfile
import time

# Python 3.6 có thể dùng locale ASCII; cả thông báo lỗi cũng phải in được.
for _name in ('stdout', 'stderr'):
    _stream = getattr(sys, _name)
    if hasattr(_stream, 'buffer'):
        setattr(sys, _name, io.TextIOWrapper(_stream.detach(), encoding='utf-8',
                                           errors='replace', line_buffering=True))

if os.name == 'posix':
    import fcntl
    import pty
    import resource
    import termios


COLUMNS = ['frame', 'monotonic_ns', 'wall_time_unix_ms', 'filename', 'width',
           'height', 'mean_luma', 'lap_var', 'dropped_since_last',
           'cmd_speed', 'cmd_steer', 'state']
REASONS = dict(esc=0, args=2, no_accept_flag=2, not_tty=2, open_fail=3,
               lock_held=6, stdin_eof=4, camera_no_frame=4, camera_stall=4,
               write_error=4, disk_low=4, motor_fail=5, servo_fail=5,
               sigint=130, sigterm=143, sighup=129, sigtstp=148, motor_init_fail=3)
CUT = {4, 5, 42, 52, 54, 59, 61, 63, 64}
CASES = ['D{:02d}'.format(n) for n in range(1, 65) if n not in CUT]
CASES += ['D{:02d}'.format(n) for n in range(65, 114)]
CASES += ['M01', 'M02', 'M03', 'M04', 'M05', 'M06', 'M07', 'M08', 'M09', 'M10', 'M11', 'M12', 'M13']


class SafetyFailure(AssertionError):
    """Mã phép kiểm giúp mutation không nhận nhầm lỗi thiết lập là phát hiện."""
    def __init__(self, tag, message):
        self.tag = tag
        super().__init__('{}: {}'.format(tag, message))


def need(condition, message, tag='contract'):
    if not condition:
        raise SafetyFailure(tag, message)


def monotonic_ms():
    return time.monotonic() * 1000


def read_trace(path):
    if not path.exists():
        return []
    events = []
    for line in path.read_text(encoding='utf-8').splitlines(True):
        if not line.endswith('\n'):
            continue
        fields = line.rstrip('\r\n').split(' ')
        need(len(fields) >= 2, 'Vết không hợp lệ: ' + repr(line))
        stamp = float(fields[0])
        need(math.isfinite(stamp), 'Mốc vết không hữu hạn')
        kind = fields[1]
        need(kind in ('motor_init', 'motor_speed', 'motor_emergency', 'servo_angle', 'key', 'state', 'key_burst', 'input_pause'),
             'Sự kiện ngoài hợp đồng: ' + repr(line))
        value = ' '.join(fields[2:])
        failed = value.endswith(' fail') or value == 'fail'
        if failed:
            value = value[:-5] if value != 'fail' else ''
        if kind in ('motor_speed', 'servo_angle'):
            value = float(value)
            need(math.isfinite(value), 'Lệnh NaN/Inf')
        events.append((stamp, kind, value, failed))
    need(all(a[0] <= b[0] for a, b in zip(events, events[1:])), 'Vết đảo thời gian')
    return events


def matched(events, kind, value=None):
    return [(i, e) for i, e in enumerate(events)
            if e[1] == kind and (value is None or e[2] == value)]


def init_sequence(events, required=False, complete=False):
    """§12: kiểm toàn vết, kể cả lỗi muộn/thử dừng; lỗi trước Init được phép không gọi."""
    inits = matched(events, 'motor_init')
    required = required or any(e[1] in ('motor_speed', 'motor_emergency', 'key') for e in events)
    need(len(inits) == 1 if required else len(inits) <= 1,
         'Init phải đúng một lần sau khi bắt đầu dùng motor/phím', 'init_once')
    if not inits:
        return
    start = inits[0][0]
    need(not matched(events[:start], 'motor_speed'), 'Có lệnh tốc độ trước Init', 'init_order')
    states = matched(events[:start], 'state')
    need(states and states[-1][1][2] == 'DUNG', 'Init không ở trạng thái DUNG', 'init_order')
    zero_motor = None
    pair_done = None
    for index, event in enumerate(events):
        _, kind, value, failed = event
        if index > start and kind == 'motor_speed' and value == 0 and not failed:
            zero_motor = index
        if zero_motor is not None and index > zero_motor and kind == 'servo_angle' and value == 0 and not failed:
            pair_done = index
        if kind == 'key' or (kind == 'motor_speed' and value > 0):
            need(pair_done is not None and index > pair_done,
                 'Xử lý phím/lệnh tiến trước cặp dừng thành công sau Init', 'init_order')
    if complete:
        need(pair_done is not None, 'Thiếu motor 0 rồi servo 0 thành công sau Init', 'init_order')


def limits(events, moving=True):
    for _, kind, value, _ in events:
        if kind == 'motor_speed':
            need(0 <= value <= (5 if moving else 0), 'Tốc độ bị cấm: ' + str(value),
                 'startup_stopped' if not moving else 'speed_limit')
        if kind == 'servo_angle':
            need(abs(value) <= (10 if moving else 0), 'Góc bị cấm: ' + str(value),
                 'startup_stopped' if not moving else 'steer_limit')


def ordered_stop(events, tag='stop_order'):
    motors = matched(events, 'motor_speed', 0) + matched(events, 'motor_emergency')
    servos = matched(events, 'servo_angle', 0)
    need(motors and servos, 'Thiếu cặp dừng motor/servo', tag)
    # X kiểm cặp ĐẦU sau thao tác, không dùng cặp cleanup cuối để che lỗi.
    need(min(i for i, _ in motors) < servos[0][0], 'Servo về 0 trước motor', tag)


def final_stop(events):
    hardware = [e for e in events if e[1] in ('motor_speed', 'motor_emergency', 'servo_angle')]
    need(hardware, 'Không có vết phần cứng khi thoát')
    motors = [i for i, e in enumerate(hardware) if e[1] in ('motor_speed', 'motor_emergency')]
    servos = [i for i, e in enumerate(hardware) if e[1] == 'servo_angle']
    need(motors and servos and motors[-1] < servos[-1], 'Thứ tự dừng cuối sai')
    need(hardware[motors[-1]][1] == 'motor_emergency' or hardware[motors[-1]][2] == 0,
         'Lệnh motor cuối khác 0')
    need(hardware[servos[-1]][2] == 0, 'Lệnh servo cuối khác 0')


def jpeg_info(data):
    """Đọc marker JPEG, không phụ thuộc cv2; trả cỡ và bảng lượng tử."""
    need(data[:2] == b'\xff\xd8' and data[-2:] == b'\xff\xd9', 'JPEG dở dang')
    pos, size, tables = 2, None, {}
    while pos < len(data) - 2:
        need(data[pos] == 255, 'Marker JPEG sai')
        while data[pos] == 255:
            pos += 1
        marker = data[pos]
        pos += 1
        if marker == 0xda:
            break
        if marker in (0xd8, 0xd9) or 0xd0 <= marker <= 0xd7:
            continue
        length = struct.unpack('>H', data[pos:pos + 2])[0]
        need(length >= 2 and pos + length <= len(data), 'Độ dài marker sai')
        payload = data[pos + 2:pos + length]
        if marker in (0xc0, 0xc1, 0xc2):
            height, width = struct.unpack('>HH', payload[1:5])
            size = (width, height)
        if marker == 0xdb:
            offset = 0
            while offset < len(payload):
                spec = payload[offset]
                count = 64 * (2 if spec >> 4 else 1)
                table = payload[offset + 1:offset + 1 + count]
                need(len(table) == count, 'Bảng lượng tử JPEG thiếu byte')
                tables[spec & 15] = (spec >> 4, tuple(table))
                offset += count + 1
        pos += length
    need(size is not None and all(size), 'Không có cỡ JPEG')
    return size, tables


def quality90(tables):
    # Bảng JPEG chuẩn ở thứ tự zigzag; Q=90 dùng scale=20.
    luma = [16,11,10,16,24,40,51,61,12,12,14,19,26,58,60,55,
            14,13,16,24,40,57,69,56,14,17,22,29,51,87,80,62,
            18,22,37,56,68,109,103,77,24,35,55,64,81,104,113,92,
            49,64,78,87,103,121,120,101,72,92,95,98,112,100,103,99]
    chroma = [17,18,24,47,99,99,99,99,18,21,26,66,99,99,99,99,
              24,26,56,99,99,99,99,99,47,66,99,99,99,99,99,99] + [99] * 32
    zigzag = [0,1,8,16,9,2,3,10,17,24,32,25,18,11,4,5,12,19,26,33,40,
              48,41,34,27,20,13,6,7,14,21,28,35,42,49,56,57,50,43,36,29,
              22,15,23,30,37,44,51,58,59,52,45,38,31,39,46,53,60,61,54,47,55,62,63]
    for index, base in enumerate((luma, chroma)):
        expected = tuple(max(1, min(255, (base[i] * 20 + 50) // 100)) for i in zigzag)
        need(tables.get(index) == (0, expected), 'JPEG không dùng chất lượng 90')


def read_jpeg(path):
    info = jpeg_info(path.read_bytes())
    try:
        import cv2
    except ImportError:
        return info
    frame = cv2.imread(str(path), cv2.IMREAD_COLOR)
    need(frame is not None and frame.size > 0, 'cv2 không giải mã được JPEG')
    need((frame.shape[1], frame.shape[0]) == info[0], 'Cỡ giải mã khác header JPEG')
    return info


class Run:
    def __init__(self, cfg, root, args=(), env=None, pipe=False, lock=None,
                 accepted=True, command=None, out_root=None, stderr=None, rec_on_start=True):
        self.cfg, self.root = cfg, root
        root.mkdir(parents=True, exist_ok=True)
        self.out = Path(out_root) if out_root is not None else root / 'out'
        self.trace = root / 'trace'
        self.ctrl = root / 'ctrl'
        self.ctrl.touch()
        self.master, self.slave = pty.openpty()
        self.before = termios.tcgetattr(self.slave)
        self.log = tempfile.TemporaryFile()
        self.next_w = 0
        self.pump = False
        self.command_speed = 5
        self.hold_timeout = 800
        for index, value in enumerate(args[:-1]):
            if value == '--hold-timeout-ms':
                try:
                    self.hold_timeout = int(args[index + 1])
                except ValueError:
                    pass  # Ca đầu vào hỏng không gọi các helper điều khiển.
            if value == '--speed-cms':
                try:
                    self.command_speed = float(args[index + 1])
                except ValueError:
                    pass  # Ca đầu vào hỏng không gọi drive().
        child_env = {k: v for k, v in os.environ.items() if not k.startswith('UCR_DRIVE_FAKE_')}
        child_env.update(LC_ALL='C.UTF-8', LANG='C.UTF-8',
                         UCR_DRIVE_FAKE_TRACE=str(self.trace),
                         UCR_DRIVE_FAKE_CTRL=str(self.ctrl),
                         UCR_DRIVE_FAKE_CAMERA='ok', UCR_DRIVE_FAKE_FREE_MB='4096',
                         UCR_DRIVE_FAKE_LOCK=str(lock or root / 'vehicle.lock'))
        child_env.update(env or {})
        # Race giả nằm trong OpenCV/TBB chưa instrument (đếm tham chiếu cv::Mat); xem tsan.supp.
        child_env['TSAN_OPTIONS'] = ':'.join(filter(None, (child_env.get('TSAN_OPTIONS', ''),
            'suppressions=' + str(Path(__file__).resolve().parent / 'tsan.supp'))))
        argv = [cfg.hooks, '--out-root', str(self.out), '--label', 'test']
        if accepted:
            argv.append('--accept-no-obstacle-brake')
        if rec_on_start:
            argv.append('--rec-on-start')
        argv += list(args)
        self.started = monotonic_ms()

        def child_setup():
            os.setsid()
            resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
            if not pipe:
                fcntl.ioctl(0, termios.TIOCSCTTY, 0)
                os.tcsetpgrp(0, os.getpgrp())

        try:
            self.process = subprocess.Popen(command or argv, cwd=str(root), env=child_env,
                                            stdin=subprocess.PIPE if pipe else self.slave,
                                            stdout=self.log, stderr=self.log if stderr is None else stderr,
                                            preexec_fn=child_setup)
        except BaseException:
            os.close(self.master)
            os.close(self.slave)
            self.log.close()
            raise

    def events(self):
        return read_trace(self.trace)

    def text(self):
        return os.pread(self.log.fileno(), os.fstat(self.log.fileno()).st_size, 0).decode(
            'utf-8', errors='replace')

    def wait(self, predicate, seconds=5, message='Hết giờ đợi điều kiện', alive=True):
        deadline = time.monotonic() + seconds
        while True:
            value = predicate()
            if value:
                return value
            if alive:
                need(self.process.poll() is None, message + ': con đã thoát; ' + self.text())
            remaining = deadline - time.monotonic()
            need(remaining > 0, message)
            self.pulse()
            select.select([], [], [], min(0.01, remaining))

    def observe(self, seconds, assertion=lambda: None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            need(self.process.poll() is None, 'Tiến trình thoát trong cửa sổ quan sát: ' + self.text())
            assertion()
            self.pulse()
            select.select([], [], [], min(0.01, max(0, end - time.monotonic())))
        assertion()

    def send(self, data):
        if isinstance(data, str):
            data = data.encode('ascii')
        need(self.master is not None, 'PTY đã đóng')
        os.write(self.master, data)

    def pulse(self):
        if self.pump and self.process.poll() is None and time.monotonic() >= self.next_w:
            self.send('W')
            self.next_w = time.monotonic() + 0.10

    def key(self, key):
        mark = len(self.events())
        self.send(key)
        wanted = 'ESC' if key == '\x1b' else key.upper()
        return self.wait(lambda: next((e for e in self.events()[mark:]
                                      if e[1] == 'key' and str(e[2]).upper() == wanted), None),
                         message='Không thấy phím ' + repr(key), alive=key != '\x1b')

    def event(self, kind, value=None, mark=0, seconds=5):
        return self.wait(lambda: next((e for e in self.events()[mark:]
                                      if e[1] == kind and (value is None or e[2] == value)), None),
                         seconds, 'Không thấy {} {}'.format(kind, value))

    def boot(self, images=True, settle=True):
        first = self.event('state', 'DUNG', seconds=10)
        need(self.started - 5 <= first[0] <= monotonic_ms() + 5,
             'Vết không cùng miền CLOCK_MONOTONIC với test', 'setup')
        need(self.events()[0][1:] == ('state', 'DUNG', False) or
             matched(self.events(), 'state')[0][1][2] == 'DUNG',
             'Trạng thái đầu khác DUNG', 'startup_stopped')
        self.wait(lambda: not (termios.tcgetattr(self.slave)[3] & (termios.ICANON | termios.ECHO)),
                  message='Terminal chưa raw/no-echo')
        if images:
            self.wait(lambda: self.rows(), seconds=10, message='Không có ảnh đầu')
        init_sequence(self.events(), required=True, complete=True)
        # Sau Init chương trình yêu cầu lần nhấn mới: W trong vòng một hạn giữ kể từ lúc khởi động bị bỏ.
        init_at = min(e[0] for e in self.events() if e[1] == 'servo_angle')  # cuối Init, ngay trước mốc yêu cầu nhấn mới
        left = init_at + self.hold_timeout + 250 - monotonic_ms()
        if settle and left > 0:
            pump, self.pump = self.pump, False
            self.observe(left / 1000)
            self.pump = pump

    def ready(self):
        # Chỉ khi phục hồi: đợi im phím đủ H từ phím cũ cuối, không chặn W đầu.
        events = self.events()
        states = matched(events, 'state')
        stopped = states and states[-1][1][2] == 'DUNG'
        resumed = any(e[1] == 'state' and e[2] == 'CHAY' for e in events) or any(
            e[1] == 'key' and str(e[2]).upper() in ('X', 'S', ' ', 'SPACE') for e in events)
        if stopped and resumed:
            self.pump = False
            relevant = [e[0] for e in events if (e[1] == 'motor_speed' and e[2] == 0) or
                        (e[1] == 'key' and str(e[2]).upper() in ('W', 'A', 'D', 'C'))]
            end = max(relevant) + self.hold_timeout + 40
            self.observe(max(0, (end - monotonic_ms()) / 1000))

    def drive(self):
        self.ready()
        mark = len(self.events())
        self.key('W')
        event = self.event('motor_speed', self.speed(), mark)
        self.wait(lambda: matched(self.events(), 'state')[-1][1][2] == 'CHAY',
                  message='Trạng thái gần nhất không phải CHAY')
        return event

    def speed(self):
        return self.command_speed

    def keep_driving(self):
        self.drive()
        self.pump = True
        self.next_w = time.monotonic() + 0.1

    def inject(self, command):
        with self.ctrl.open('a', encoding='utf-8') as stream:
            stream.write(command + '\n')
            stream.flush()
        return monotonic_ms()

    def rows(self):
        paths = list(self.out.glob('frames.csv'))
        if not paths:
            return []
        lines = paths[0].read_text(encoding='utf-8').splitlines(True)
        reader = csv.DictReader([line for line in lines if line.endswith('\n')])
        if reader.fieldnames is None:
            return []
        need(reader.fieldnames == COLUMNS + ['pass', 'file'], 'Sai cột/thứ tự CSV: ' + repr(reader.fieldnames))
        rows = list(reader)
        need(all(None not in row and None not in row.values() for row in rows), 'CSV thiếu/thừa cột')
        return [row for row in rows if int(row['monotonic_ns']) >= self.started * 1000000]

    def session_dir(self):
        paths = list(self.out.glob('*/session.json'))
        need(len(paths) == 1, 'Cần đúng một session.json: ' + repr(paths))
        return paths[0].parent

    def finish(self, reason='esc', codes=None, cleanup=True, summary=True, esc_mark=None):
        if reason == 'esc':
            self.pump = False
            mark = len(self.events()) if esc_mark is None else esc_mark
            # Ca đã gửi Esc phải kiểm chính phím đó, không gửi thêm lúc con đang thoát.
            key = (self.key('\x1b') if esc_mark is None else
                   self.wait(lambda: matched(self.events()[mark:], 'key', 'ESC'),
                             alive=False, message='Không thấy Esc đã gửi')[0][1])
            stop_by(self, mark, key[0])
        self.wait(lambda: self.process.poll() is not None, seconds=8,
                  message='Không thoát: ' + reason, alive=False)
        code = self.process.returncode
        expected = {REASONS[reason]} if codes is None else set(codes)
        need(code in expected, 'Mã thoát {} cần {}: {}'.format(code, expected, self.text()))
        events = self.events()
        limits(events)
        if cleanup:
            final_stop(events)
        if reason != 'esc' and hasattr(self, 'stop_mark'):
            no_restart(events[self.stop_mark:])
        if summary:
            self.check_output(reason, code)
        return events

    def check_output(self, reason, code):
        folder = self.session_dir()
        session = json.loads((folder / 'session.json').read_text(encoding='utf-8'))
        summary = json.loads((folder / 'summary.json').read_text(encoding='utf-8'))
        need({'started_utc', 'argv', 'width', 'height', 'speed_cms', 'max_steer_deg',
              'hold_timeout_ms'} <= set(session), 'Thiếu khóa session theo G5')
        need({'started_utc', 'ended_utc', 'images', 'exit_reason', 'exit_code',
              'emergency_stops', 'max_speed_cmd', 'max_steer_abs_cmd'} <= set(summary),
             'Thiếu khóa summary theo G5')
        need(summary['started_utc'] and summary['ended_utc'], 'Thiếu thời gian phiên')
        need(summary['exit_reason'] == reason and summary['exit_code'] == code,
             'Sai lý do/mã trong summary: ' + repr(summary))
        events = self.events()
        xs = sum(1 for e in events if e[1] == 'key' and str(e[2]).upper() == 'X')
        need(summary['emergency_stops'] == xs, 'Sai bộ đếm X')
        if reason not in ('motor_fail', 'servo_fail'):
            max_speed = max([0] + [e[2] for e in events if e[1] == 'motor_speed' and not e[3]])
            max_steer = max([0] + [abs(e[2]) for e in events if e[1] == 'servo_angle' and not e[3]])
            need(summary['max_speed_cmd'] == max_speed and summary['max_steer_abs_cmd'] == max_steer,
                 'Sai cực đại tốc độ/góc: ' + repr(summary))
        rows = self.rows()
        images = [self.out / row['filename'] for row in rows]
        need(set(images) == set(self.out.glob('*.jpg')), 'Ảnh ngoài frames.csv')
        need(summary['images'] == len(rows) == len(images), 'Lệch số ảnh/CSV/summary')
        need(not list(self.out.rglob('*.tmp')), 'Còn tệp ảnh tạm sau thoát')
        previous, filenames = -1, set()
        for index, row in enumerate(rows, 1):
            need(int(row['frame']) == index, 'Số khung không liên tục')
            stamp = int(row['monotonic_ns'])
            need(stamp > previous, 'Mốc ảnh không tăng')
            previous = stamp
            name = row['filename']
            need(Path(name).name == name and name not in filenames, 'Tên ảnh sai/lặp')
            filenames.add(name)
            size, _ = read_jpeg(self.out / name)
            need(size == (int(row['width']), int(row['height'])), 'Cỡ ảnh khác CSV')
            need(0 <= float(row['cmd_speed']) <= 5 and abs(float(row['cmd_steer'])) <= 10,
                 'Lệnh CSV ngoài giới hạn')
            need(row['state'] in ('DUNG', 'CHAY'), 'Trạng thái CSV sai')
        return summary, rows

    def close(self):
        self.pump = False
        try:
            os.killpg(self.process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        self.process.wait(timeout=5)
        if self.process.stdin:
            self.process.stdin.close()
        try:
            if self.master is not None:
                termios.tcsetattr(self.slave, termios.TCSANOW, self.before)
        finally:
            if self.master is not None:
                os.close(self.master)
            os.close(self.slave)
            self.log.close()


@contextmanager
def running(cfg, root, **kwargs):
    run = Run(cfg, root, **kwargs)
    try:
        yield run
        if kwargs.get('command') is None:
            init_sequence(run.events())
    except BaseException:
        print('LOG CON:\n' + run.text()[-6000:], file=sys.stderr)
        print('VẾT CUỐI: ' + repr(run.events()[-25:]), file=sys.stderr)
        raise
    finally:
        run.close()


def steer(run, key, target):
    mark = len(run.events())
    run.key(key)
    run.event('servo_angle', target, mark)


def moving(run):
    run.boot()
    run.ready()
    run.keep_driving()
    steer(run, 'A', 5)


def stop_by(run, mark, start_ms, allowance=0, tag='fault_stop'):
    run.stop_mark = mark
    deadline = start_ms + allowance + 200 + run.cfg.slack
    try:
        run.wait(lambda: matched(run.events()[mark:], 'motor_speed', 0) or
                 matched(run.events()[mark:], 'motor_emergency'),
                 max(0.01, (deadline - monotonic_ms()) / 1000), 'Không dừng trước hạn', alive=False)
    except SafetyFailure as error:
        raise SafetyFailure(tag, str(error))
    stops = [e for e in run.events()[mark:] if e[1] == 'motor_emergency' or
             (e[1] == 'motor_speed' and e[2] == 0)]
    need(stops and stops[0][0] <= deadline, 'Dừng quá hạn {:.1f} ms'.format(deadline), tag)
    after = run.events()[mark:]
    first = next(i for i, e in enumerate(after) if e == stops[0])
    need(not any(e[1] == 'motor_speed' and e[2] > 0 for e in after[first + 1:]),
         'Tự chạy lại sau lỗi')
    return stops[0]


def rejected(cfg, root, args, code=2, accepted=True, env=None, pipe=False):
    with running(cfg, root, args=args, accepted=accepted, env=env, pipe=pipe) as run:
        try:
            run.wait(lambda: run.process.poll() is not None, 10, 'Không từ chối đầu vào', alive=False)
        except SafetyFailure as error:
            if '--speed-cms' in args:
                raise SafetyFailure('speed_reject', str(error))
            raise
        need(run.process.returncode == code, 'Cần mã {} thay vì {}: {}'.format(
            code, run.process.returncode, run.text()), 'speed_reject' if '--speed-cms' in args else 'contract')
        need(run.text().strip(), 'Không giải thích lý do từ chối')
        limits(run.events(), moving=False)


def motion_quiet(events, tag='fresh_press'):
    need(not any((e[1] == 'motor_speed' and e[2] > 0) or
                 (e[1] == 'servo_angle' and e[2] != 0) for e in events),
         'Phím giữ cũ làm xe chạy/lái lại trước khi im đủ H', tag)


def startup_quiet(events, tag):
    motion_quiet(events, tag)
    need(not matched(events, 'key'),
         'Đã diễn giải phím tồn trước khi nhận W mới sau Init', tag)


def held_then_fresh(run, mark):
    """T24/A: phím lặp cũ kéo dài quá H kể từ sự cố vẫn không được chạy."""
    run.pump = False
    for _ in range(6):
        run.key('W')
        run.key('A')
        run.observe(run.hold_timeout * 0.30 / 1000,
                    lambda: motion_quiet(run.events()[mark:]))
    # Đã xa sự cố >H nhưng chỉ cách phím bị từ chối cuối <H.
    run.observe(run.hold_timeout * 0.30 / 1000,
                lambda: motion_quiet(run.events()[mark:]))
    run.key('W')
    run.observe(0.05, lambda: motion_quiet(run.events()[mark:]))
    run.observe((run.hold_timeout + 50) / 1000,
                lambda: motion_quiet(run.events()[mark:]))
    fresh = len(run.events())
    run.key('W')
    run.event('motor_speed', run.speed(), fresh)


def unlock_case(n, cfg, root):
    if n == 97:
        # W giữ liên tục xuyên khởi động (tự lặp) không được chạy xe; phải nhả đủ H rồi nhấn mới.
        with running(cfg, root / 'held_start') as run:
            run.boot(settle=False)
            run.pump = True
            run.observe(run.hold_timeout * 1.5 / 1000, lambda: motion_quiet(run.events(), 'startup_held'))
            run.pump = False
            run.observe((run.hold_timeout + 100) / 1000)
            mark = len(run.events())
            run.key('W')
            run.event('motor_speed', run.speed(), mark)
            run.finish()
        return
    if n in (94, 95):
        environment = {'UCR_DRIVE_FAKE_INIT_MS': '1500'}
        if n == 95:
            environment['UCR_DRIVE_FAKE_PREVIEW'] = 'keys:' + '?' * 10 + 'A' + 'W' * 20
        tag = 'startup_stdin' if n == 94 else 'startup_preview'
        with running(cfg, root / tag, env=environment) as run:
            init = run.event('motor_init', seconds=10)
            if n == 94:
                need(os.write(run.master, b'AW' * 20) == 40,
                     'PTY chưa nhận đủ phím tồn trong Init', 'setup')
            need(monotonic_ms() - init[0] < 1000,
                 'Không còn đủ cửa sổ đặt phím trước cuối Init', 'setup')
            if n == 95:
                # Con giả gửi mỗi 20 ms, chuỗi 31 ký tự kết thúc trước 900 ms.
                run.wait(lambda: monotonic_ms() >= init[0] + 900, 2,
                         'Chưa hết cửa sổ tạo hàng phím preview trong Init')
                need(monotonic_ms() < init[0] + 1400,
                     'Đặt hàng preview không còn nằm trong Init 1500 ms', 'setup')
            run.boot()
            run.observe(0.3, lambda: startup_quiet(run.events(), tag))
            mark = len(run.events())
            run.key('W')
            run.event('motor_speed', 5, mark)  # Không để xả nhầm W đầu mới.
            run.finish()
        return
    with running(cfg, root / 'held_x', args=['--hold-timeout-ms', '700']) as run:
        moving(run)
        run.pump = False
        mark = len(run.events())
        key = run.key('X')
        stop_by(run, mark, key[0])
        run.event('servo_angle', 0, mark)
        ordered_stop(run.events()[mark:])
        need(matched(run.events(), 'state')[-1][1][2] == 'DUNG',
             'X không về DUNG', 'fresh_press')
        held_then_fresh(run, len(run.events()))
        run.finish()


def no_selftest_hardware(events):
    need(not any(e[1].startswith(('motor_', 'servo_')) for e in events),
         '--selftest đã chạm motor/servo', 'selftest_hardware')


def burst_case(n, cfg, root):
    if n == 101:
        # C qua bộ lọc preview mà không xả hàng như Space; '?' bị bỏ ngay ở con.
        # Con preview tiếp tục gửi khi cha bị dừng: tạo một lần đọc >=4 phím, không sửa preview.cpp.
        for end in ('', '3', 'E'):
            with running(cfg, root / (end or 'continue'),
                         env={'UCR_DRIVE_FAKE_INIT_MS': '0',
                              'UCR_DRIVE_FAKE_PREVIEW': 'keys:' + 'c' * 120 + 'wAdC' + end}) as run:
                try:
                    run.boot()
                    run.wait(lambda: len(matched(run.events(), 'key', 'c')) >= 100,
                             message='Chưa tới cửa sổ tạo hàng preview')
                    mark = len(run.events())
                    os.kill(run.process.pid, signal.SIGSTOP)
                    until = time.monotonic() + 0.65
                    run.wait(lambda: time.monotonic() >= until, 2, 'Chưa tạo đủ hàng preview')
                finally:
                    if run.process.poll() is None:
                        os.kill(run.process.pid, signal.SIGCONT)
                run.wait(lambda: [str(e[2]).lower() for e in run.events()[mark:]
                                  if e[1] == 'key' and str(e[2]).lower() in 'wadc'][-4:] == list('wadc'),
                         message='Không đọc được lô preview', alive=False)
                if end:
                    run.wait(lambda: run.process.poll() is not None, 8, 'Phím thoát preview bị mất', alive=False)
                    need(run.process.returncode == (130 if end == '3' else 0), 'Sai mã thoát preview')
                    final_stop(run.events())
                    run.check_output('sigint' if end == '3' else 'esc', run.process.returncode)
                    motion_quiet(run.events()[mark:], 'key_burst')
                else:
                    run.observe(0.1, lambda: motion_quiet(run.events()[mark:], 'key_burst'))
                    run.observe(0.9, lambda: motion_quiet(run.events()[mark:], 'key_burst'))
                    run.keep_driving()  # Nhấn đơn rồi lặp W mỗi 100 ms vẫn hoạt động.
                    run.observe(1.8)
                    run.finish()
                need(matched(run.events()[mark:], 'key_burst'), 'Không ghi vết lô preview', 'key_burst')
        return
    if n == 100:
        for index, (key, reason) in enumerate((('X', 'key_x'), (' ', 'key_space'),
                                               ('S', 'key_s'), ('\x1b', 'esc'), ('\x03', 'sigint'))):
            with running(cfg, root / str(index)) as run:
                run.boot()
                run.drive()
                run.observe(0.45)
                mark = len(run.events())
                run.send('wAdC' + key + 'WWWW')
                if reason in ('esc', 'sigint'):
                    run.wait(lambda: run.process.poll() is not None, 8, 'Phím thoát trong lô bị mất', alive=False)
                    need(run.process.returncode == (0 if reason == 'esc' else 130), 'Sai mã phím thoát')
                    final_stop(run.events())
                    run.check_output(reason, run.process.returncode)
                else:
                    run.event('motor_speed', 0, mark)
                    run.wait(lambda: ('key_' + ('space' if key == ' ' else key.lower())) in run.text(),
                             message='Phím dừng trong lô không được xử lý')
                    run.observe(0.15)
                motion_quiet(run.events()[mark:], 'key_burst')
                ordered_stop(run.events()[mark:])
                if reason not in ('esc', 'sigint'):
                    run.finish()
        return
    with running(cfg, root, args=['--no-preview', '--hold-timeout-ms', '2000'] if n == 99 else ['--no-preview']) as run:
        run.boot()
        run.key('?')  # Lần đọc trước có phím, không phụ thuộc thời gian Init.
        if n == 99:
            run.drive()
            run.key('A')
            run.observe(1.1)  # H/2 < im < H: vẫn CHAY, không nhầm với deadman.
        else:
            run.observe(0.9)  # Im > H: lô cũ từng vượt qua chốt nhấn mới.
        mark = len(run.events())
        run.send('wAdC')
        run.wait(lambda: len(matched(run.events()[mark:], 'key')) >= 4,
                 message='Chưa đọc đủ lô tty')
        run.observe(0.15, lambda: motion_quiet(run.events()[mark:], 'key_burst'))
        need(matched(run.events()[mark:], 'key_burst'), 'Thiếu vết key_burst', 'key_burst')
        if n == 99:
            ordered_stop(run.events()[mark:])
            run.wait(lambda: 'key_burst' in run.text(), message='Thiếu nhật ký dừng lô')
            paths = list(run.out.glob('*/stops.csv'))
            run.wait(lambda: paths and 'key_burst' in paths[0].read_text(encoding='utf-8'),
                     message='stops.csv thiếu key_burst')
        else:
            need(not matched(run.events()[mark:], 'motor_speed', 0),
                 'Lô khi DUNG bị ghi thành sự kiện dừng')
        held_then_fresh(run, len(run.events()))
        run.finish()
        records = json.loads((run.session_dir() / 'summary.json').read_text(encoding='utf-8'))['stops']
        need(sum(r['reason'] == 'key_burst' for r in records) == (1 if n == 99 else 0),
             'Lô DUNG/CHAY ghi sai số sự kiện dừng')


def selftest_case(n, cfg, root):
    modes = ('ok',) if n == 102 else ('camera', 'hang', 'jpeg', 'display', 'xauthority', 'disk',
                                     'busy', 'output', 'stop_missing', 'stop_permission', 'args', 'git')
    for mode in modes:
        folder = root / mode
        folder.mkdir(parents=True)
        binary = folder / 'ucr_drive_collect'
        shutil.copy2(cfg.hooks, str(binary))
        binary.chmod(0o700)
        stop_called = folder / 'stop_called'
        if mode != 'stop_missing':
            executable(folder / 'ucr_stop', '#!/bin/sh\nprintf called > ' + shlex.quote(str(stop_called)) + '\n')
            if mode == 'stop_permission':
                (folder / 'ucr_stop').chmod(0o600)
        out = folder / 'out'
        if mode == 'output':
            out.write_text('giữ nguyên tệp này', encoding='utf-8')
        environment = dict(DISPLAY=':99', XAUTHORITY='/tmp/selftest-authority', PATH='')
        if mode == 'camera': environment['UCR_DRIVE_FAKE_OPEN_FAIL'] = 'camera'
        if mode == 'hang': environment['UCR_DRIVE_FAKE_CAMERA'] = 'none'
        if mode == 'jpeg': environment['UCR_DRIVE_FAKE_SELFTEST_JPEG_FAIL'] = '1'
        if mode == 'display': environment['DISPLAY'] = ''
        if mode == 'xauthority': environment['XAUTHORITY'] = ''
        if mode == 'disk': environment['UCR_DRIVE_FAKE_FREE_MB'] = '0'
        command = [str(binary), '--selftest', '--out-root', str(out), '--min-free-mb', '4096']
        if mode not in ('display', 'xauthority'):
            command.append('--no-preview')
        if mode == 'args': command += ['--pipeline']
        lock = folder / 'vehicle.lock'
        with lock.open('w') as held:
            if mode == 'busy': fcntl.flock(held.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            with running(cfg, folder / 'run', env=environment, command=command, pipe=True,
                         accepted=False, lock=lock) as run:
                started = time.monotonic()
                run.wait(lambda: run.process.poll() is not None, 14, 'Selftest quá hạn', alive=False)
                no_selftest_hardware(run.events())
                need(not stop_called.exists(), 'Selftest đã chạy ucr_stop', 'selftest_hardware')
                expected = 0 if mode in ('ok', 'git') else 2 if mode == 'args' else 1
                need(run.process.returncode == expected,
                     'Selftest {}: sai mã {}: {}'.format(mode, run.process.returncode, run.text()))
                text = run.text()
                if mode == 'ok':
                    for item in ('camera', 'jpeg', 'DISPLAY', 'XAUTHORITY', 'disk', 'lock', 'out_root', 'ucr_stop', 'total'):
                        need(re.search(r'^OK ' + item + ':', text, re.MULTILINE), 'Thiếu OK ' + item)
                elif mode != 'args' and mode != 'git':
                    item = dict(camera='camera', hang='camera', jpeg='jpeg', display='DISPLAY', xauthority='XAUTHORITY',
                                disk='disk', busy='lock', output='out_root', stop_missing='ucr_stop',
                                stop_permission='ucr_stop')[mode]
                    need(re.search(r'^FAIL ' + item + ':', text, re.MULTILINE), 'Thiếu FAIL ' + item)
                    need(re.search(r'^FAIL total:', text, re.MULTILINE), 'Tổng không ghi FAIL')
                if mode == 'busy': need('xe đang bận' in text, 'Thiếu thông báo xe đang bận')
                if mode == 'hang':
                    need(time.monotonic() - started <= 10.5 + cfg.slack/1000 + cfg.tsan_slack/1000,
                         'Selftest camera không chốt hạn 10 s')
                if mode in ('ok', 'git'): need('WARN git: không rõ' in text, 'Git thiếu không có WARN')
                need(not list(folder.glob('out/.ucr_selftest_*')), 'Còn thư mục/JPEG tạm sau tự kiểm')
                if mode == 'output': need(out.read_text(encoding='utf-8') == 'giữ nguyên tệp này', 'Đã hỏng tệp có sẵn')
            fcntl.flock(held.fileno(), fcntl.LOCK_UN)
        with lock.open('r+') as available:
            fcntl.flock(available.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)


def unread_safety_case(n, cfg, root):
    if n == 104:
        for end, reason, code in (('E', 'esc', 0), ('3', 'sigint', 130),
                                  ('x', 'key_x', None), ('s', 'key_s', None), (' ', 'key_space', None)):
            folder = root / str(ord(end))
            batch = folder / 'batch'
            with running(cfg, folder, env={'UCR_DRIVE_FAKE_PREVIEW': 'batch:' + str(batch)}) as run:
                run.boot()
                run.key('?')
                run.observe(0.45)
                mark = len(run.events())
                # Một write <= PIPE_BUF: 64 chuyển động rồi phím an toàn thứ 65.
                batch.write_bytes(('c' * 64 + end + 'wAdC').encode('ascii'))
                if code is not None:
                    run.wait(lambda: run.process.poll() is not None, 8, 'Mất phím thoát thứ 65', alive=False)
                    need(run.process.returncode == code, 'Sai mã thoát phím thứ 65')
                    run.check_output(reason, code)
                    final_stop(run.events())
                else:
                    run.wait(lambda: reason in run.text(), message='Mất lý do dừng phím thứ 65')
                    run.observe(0.15)
                need(matched(run.events()[mark:], 'key_burst'), 'Chưa dựng được lô 64 phím', 'setup')
                motion_quiet(run.events()[mark:], 'unread_safety')
                if code is None:
                    run.finish()
        return
    with running(cfg, root, args=['--no-preview']) as run:
        run.boot()
        run.drive()
        mark = len(run.events())
        # Chỉ giữ luồng nhập; camera vẫn tươi, lượt tiếp theo bắt buộc tick deadman.
        with run.ctrl.open('a') as control:
            control.write('pause_input_900ms\n')
        run.event('input_pause', mark=mark)
        run.send('\x1b')
        run.wait(lambda: run.process.poll() is not None, 8, 'Esc đang chờ tại deadman bị mất', alive=False)
        need(run.process.returncode == 0, 'Esc tại deadman sai mã thoát')
        run.check_output('esc', 0)
        records = json.loads((run.session_dir() / 'summary.json').read_text(encoding='utf-8'))['stops']
        reasons = [record['reason'] for record in records]
        need('deadman' in reasons and 'key_esc' in reasons and
             reasons.index('deadman') < reasons.index('key_esc'), 'Chưa xử lý Esc đúng lượt deadman', 'unread_safety')
        no_restart(run.events()[mark:])
        final_stop(run.events())



def pass_rows(run):
    path = run.out / 'passes.csv'
    if not path.exists():
        return []
    with path.open(encoding='utf-8', newline='') as stream:
        return list(csv.DictReader(stream))


def recording_case(n, cfg, root):
    if n == 109:
        out = root / 'shared'
        out.mkdir(parents=True)
        (out / '.pass_counter').write_text('1\n', encoding='utf-8')
        (out / 'passes.csv').write_text('pass,event,ts_ms,frames,end_reason\n77,start,1,0,\n77,end,2,0,exit\n', encoding='utf-8')
        sentinel = out / 'p0122_0001.jpg'
        sentinel.write_bytes(b'GIU ANH CU')
        with running(cfg, root / 'killed', out_root=out) as run:
            run.boot()
            rows = run.rows()
            need(rows and int(rows[0]['pass']) == 123 and rows[0]['filename'] == 'p0123_0001.jpg',
                 'Không lấy max ba nguồn + 1', 'rec_counter')
            run.process.kill()  # Chỉ binary hooks; giữ lượt dở dang để phục hồi.
            run.process.wait(timeout=5)
        (out / '.pass_counter').write_text('1\n', encoding='utf-8')
        with running(cfg, root / 'again', out_root=out) as run:
            run.boot()
            need(int(run.rows()[0]['pass']) == 124, 'Kill làm trùng lượt', 'rec_counter')
            need(sentinel.read_bytes() == b'GIU ANH CU', 'Ghi đè ảnh cũ', 'rec_counter')
            need(all(re.match(r'^p[0-9]{4,}_[0-9]{4,}\.jpg$', row['filename']) for row in run.rows()),
                 'Tên ảnh sai', 'rec_counter')
            run.finish(summary=False)
        return
    if n == 112:
        for source in ('tty', 'preview'):
            for key, reason, ended in (('x', 'esc', 'stop'), ('E', 'esc', 'exit'), ('3', 'sigint', 'stop')):
                folder = root / (source + key)
                folder.mkdir(parents=True)
                batch = folder / 'keys'
                env = {'UCR_DRIVE_FAKE_PREVIEW': 'batch:' + str(batch)} if source == 'preview' else {}
                with running(cfg, folder, env=env) as run:
                    run.boot()
                    run.drive()
                    mark = len(run.events())
                    if source == 'preview':
                        batch.write_bytes(key.encode('ascii'))
                    elif key == '3':
                        run.process.send_signal(signal.SIGINT)
                    else:
                        run.key('\x1b' if key == 'E' else key)
                    if key == 'x':
                        run.event('motor_speed', 0, mark)
                        run.wait(lambda: any(row['event'] == 'end' for row in pass_rows(run)))
                    run.finish(reason, esc_mark=mark if key == 'E' else None)
                    ends = [row for row in pass_rows(run) if row['event'] == 'end']
                    need(len(ends) == 1 and ends[0]['end_reason'] == ended,
                         'Phím dừng không đóng lượt đúng: ' + repr(ends), 'rec_stop')
                    ordered_stop(run.events()[mark:])
        for fault in ('jpeg_fail', 'csv_fail'):
            with running(cfg, root / fault) as run:
                run.boot()
                run.keep_driving()
                run.inject(fault)
                run.finish('write_error')
                ends = [row for row in pass_rows(run) if row['event'] == 'end']
                need(len(ends) == 1 and ends[0]['end_reason'] == 'error',
                     'Lỗi ghi không đóng lượt error: ' + repr(ends), 'rec_error')
        return
    sources = ('tty', 'preview') if n in (108, 113) else ('tty',)
    scenarios = [(source, hold) for source in sources
                 for hold in ((800, 1400) if n == 113 else (800,))]
    for source, hold in scenarios:
        folder = root / source
        if n == 113:
            folder = folder / str(hold)
        folder.mkdir(parents=True)
        batch = folder / 'keys'
        env = {'UCR_DRIVE_FAKE_PREVIEW': 'batch:' + str(batch)} if source == 'preview' else {}
        args = ['--interval-ms', '100']
        if n == 113:
            args += ['--hold-timeout-ms', str(hold)]
        with running(cfg, folder, rec_on_start=False, args=args, env=env) as run:
            run.boot(images=False)
            run.wait(lambda: (run.out / 'sessions.csv').exists())
            if n == 106:
                run.observe(2.3)
                need(not run.rows() and not list(run.out.glob('*.jpg')) and not pass_rows(run),
                     'REC mặc định tắt vẫn ghi ảnh/lượt', 'rec_default')
            elif n == 107:
                run.keep_driving()
                mark = len(run.events())
                run.key('P')
                run.observe(0.3)
                need(not pass_rows(run) and not run.rows(), 'P khi chạy mở lượt/ghi ảnh', 'rec_moving')
                need(not any(e[1] in ('motor_speed', 'servo_angle') and (e[1] != 'motor_speed' or e[2] == 0)
                             for e in run.events()[mark:]), 'P làm đổi lệnh dừng/lái', 'rec_moving')
            elif n in (108, 110, 113):
                mark = len(run.events())
                if source == 'preview':
                    batch.write_bytes(b'P')
                    first_p = run.event('key', 'P', mark)
                else:
                    first_p = run.key('P')
                run.wait(lambda: run.rows(), message='P không bật ghi ảnh')
                need([row['event'] for row in pass_rows(run)] == ['start'], 'Thiếu start lượt', 'rec_toggle')
                rec_window = max(run.hold_timeout, 1000)
                if n == 113:
                    run.observe(max(0, (first_p[0] + rec_window - 100 - monotonic_ms()) / 1000))
                    if source == 'preview':
                        repeat_mark = len(run.events())
                        batch.write_bytes(b'P')
                        repeat_p = run.event('key', 'P', repeat_mark)
                    else:
                        repeat_p = run.key('p')
                    need(0 < repeat_p[0] - first_p[0] < rec_window,
                         'Chưa dựng được P lặp trong cửa sổ', 'setup')

                    def still_recording():
                        need([row['event'] for row in pass_rows(run)] == ['start'],
                             'P lặp đóng/mở thêm lượt', 'rec_debounce')

                    run.observe(max(0, (first_p[0] + rec_window + 150 - monotonic_ms()) / 1000),
                                still_recording)
                if n == 110:
                    run.wait(lambda: len(run.rows()) >= 3, seconds=8)
                    stamps = [int(row['monotonic_ns']) for row in run.rows()]
                    need(all(b - a >= 2000000000 for a, b in zip(stamps, stamps[1:])),
                         'Xe đứng ghi nhanh hơn 1 ảnh/2 s', 'rec_cadence')
                # Chờ theo mốc P được nhận và hạn giữ của ca; không đổi ngưỡng an toàn.
                run.observe(max(0, (first_p[0] + rec_window + 150 - monotonic_ms()) / 1000))
                if source == 'preview':
                    off_mark = len(run.events())
                    batch.write_bytes(b'P')
                    off_p = run.event('key', 'P', off_mark)
                else:
                    off_p = run.key('p')
                need(off_p[0] - first_p[0] >= rec_window + 150,
                     'P tắt chưa qua cửa sổ chống lặp và biên', 'setup')
                run.wait(lambda: any(row['event'] == 'end' for row in pass_rows(run)))
                count = len(run.rows())
                run.observe(2.2)
                need(len(run.rows()) == count, 'Ghi ảnh sau REC tắt', 'rec_toggle')
                rows = pass_rows(run)
                need(len(rows) == 2 and rows[1]['end_reason'] == 'user' and int(rows[1]['frames']) == count,
                     'Sai dòng end/user/số ảnh', 'rec_toggle')
                if n == 113:
                    need(count > 0, 'P lặp tạo lượt rỗng', 'rec_debounce')
                need(not any(e[1] in ('motor_speed', 'servo_angle') for e in run.events()[mark:]),
                     'P ghi lệnh phần cứng', 'rec_toggle')
            elif n == 111:
                run.send(b'wwwwP')
                run.observe(0.3)
                need(not pass_rows(run) and not run.rows(), 'Lô dồn vẫn nhận P', 'rec_burst')
                limits(run.events(), moving=False)
            run.finish()


def test_case(case, cfg, root):
    """Mỗi nhánh đối chiếu hàng cùng mã trong T21_test_cases_report, §8 ưu tiên."""
    n = int(case[1:])
    if case.startswith('M'):
        return mutation(case, cfg, root)
    if n >= 106:
        return recording_case(n, cfg, root)
    if n in (104, 105):
        return unread_safety_case(n, cfg, root)
    if n in (98, 99, 100, 101):
        return burst_case(n, cfg, root)
    if n in (102, 103):
        return selftest_case(n, cfg, root)
    if n in (94, 95, 96, 97):
        return unlock_case(n, cfg, root)
    if n in (92, 93):
        return init_write_failure(n, cfg, root)
    if n >= 85:
        return fix4_case(n, cfg, root)
    if n >= 80:
        return init_case(n, cfg, root)
    if n >= 65:
        return added_case(n, cfg, root)
    if n in (7, 8, 10, 38, 39, 60):
        if n == 7:
            variants = [('--speed-cms', '20'), ('--speed-cms', '5.01')]
        elif n == 8:
            variants = [('--speed-cms', v) for v in ('-1', 'nan', 'inf', '-inf')]
        elif n == 10:
            variants = [('--max-steer-deg', '30'), ('--max-steer-deg', '10.01')]
        elif n == 60:
            variants = [('--unknown',), ('--speed-cms',), ('--speed-cms', 'rác'),
                        ('--hold-timeout-ms', '0'), ('--interval-ms', '-1'),
                        ('--hold-timeout-ms', '99'), ('--hold-timeout-ms', '699'), ('--hold-timeout-ms', '2001'),
                        ('--steer-step-deg', '0'),
                        ('--hold-timeout-ms', '999999999999999999999999999999')]
        else:
            variants = [()]
        for index, args in enumerate(variants):
            rejected(cfg, root / str(index), args, accepted=n != 38, pipe=n == 39)
        if n == 60:
            with running(cfg, root / 'step11', args=['--steer-step-deg', '11']) as run:
                run.boot()
                run.ready()
                run.keep_driving()
                for key, target in (('A', 10), ('A', 10), ('D', -1), ('D', -10), ('D', -10)):
                    steer(run, key, target)
                run.finish()
        if n == 10:
            with running(cfg, root / 'valid', args=['--max-steer-deg', '7', '--steer-step-deg', '3']) as run:
                run.boot()
                run.ready()
                for target in (3, 6, 7, 7):
                    steer(run, 'A', target)
                for target in (4, 1, -2, -5, -7, -7):
                    steer(run, 'D', target)
                run.finish()
        if n == 8:
            # Dấu đọc phím có trước dừng/xả hàng; mỗi phím cần phiên riêng.
            for index, key in enumerate(('W', 'S', ' ', 'C', 'A', 'D', 'X', 'S')):
                with running(cfg, root / ('key_' + str(index))) as run:
                    moving(run)
                    run.pump = False
                    mark = len(run.events())
                    pressed = run.key(key)
                    if key in ('S', ' ', 'X'):
                        stop_by(run, mark, pressed[0])
                        run.event('state', 'DUNG', mark)
                        if key == 'X':
                            run.event('servo_angle', 0, mark)
                            ordered_stop(run.events()[mark:])
                    elif key == 'W':
                        run.event('motor_speed', run.speed(), mark)
                    else:
                        run.event('servo_angle', {'C': 0, 'A': 10, 'D': 0}[key], mark)
                    # Không gửi Esc vào đúng cửa sổ xóa phím sau S/Space/X.
                    run.process.send_signal(signal.SIGTERM)
                    run.finish('sigterm')
        return

    if n in (21, 22, 23, 24):
        signals = [(signal.SIGINT, 'sigint'), (signal.SIGTERM, 'sigterm'), (signal.SIGHUP, 'sighup')]
        for state in (('DUNG',) if n == 24 else ('CHAY',)):
            for sig, reason in signals if n == 24 else [signals[n - 21]]:
                with running(cfg, root / (state + reason)) as run:
                    run.boot()
                    if state != 'DUNG':
                        run.ready()
                    if state == 'CHAY':
                        run.keep_driving()
                        steer(run, 'A', 5)
                    mark, stamp = len(run.events()), monotonic_ms()
                    run.process.send_signal(sig)
                    stop_by(run, mark, stamp)
                    run.finish(reason)
        return

    if n in (29, 30, 31, 34, 35):
        faults = {29: [('jpeg_fail', 'write_error')], 30: [('csv_fail', 'write_error')],
                  31: [('free_mb 1023', 'disk_low')],
                  34: [('motor_fail', 'esc'), ('motor_fail', 'sigterm'),
                       ('motor_fail', 'camera_stall'), ('motor_fail', 'disk_low')],
                  35: [('throw', 'exception')]}
        for index, (fault, reason) in enumerate(faults[n]):
            with running(cfg, root / str(index), args=['--interval-ms', '100']) as run:
                moving(run)
                if n == 34:
                    run.pump = False
                mark = len(run.events())
                stamp = run.inject(fault)
                if n == 34:
                    # Hai phím qua hai vòng: motor_fail đã được poll trước
                    # khi tạo sự kiện Esc/tín hiệu/lỗi dữ liệu kết hợp.
                    run.key('C')
                    run.key('C')
                if reason == 'esc':
                    run.pump = False
                    run.key('\x1b')
                elif reason == 'sigterm':
                    run.process.send_signal(signal.SIGTERM)
                elif n == 34:
                    run.inject('camera_stall' if reason == 'camera_stall' else 'free_mb 1')
                if n == 35:
                    run.wait(lambda: run.process.poll() is not None, 5, 'terminate không abort', alive=False)
                    need(run.process.returncode == -signal.SIGABRT, 'Ngoại lệ phải kết thúc SIGABRT')
                    need(matched(run.events()[mark:], 'motor_emergency'), 'terminate thiếu EmergencyStop')
                else:
                    stop_by(run, mark, stamp, allowance=500 if reason == 'camera_stall' else
                            (100 if n in (29, 30) else 0))
                    run.finish('motor_fail' if n == 34 else reason)
                    no_restart(run.events()[mark:])
                    if n == 31:
                        with (run.out / 'passes.csv').open(encoding='utf-8', newline='') as stream:
                            passes = list(csv.DictReader(stream))
                        need(len(passes) == 2 and passes[0]['event'] == 'start' and
                             passes[1]['event'] == 'end' and
                             passes[1]['pass'] == passes[0]['pass'] and
                             passes[1]['end_reason'] == 'disk_low',
                             'Hết dung lượng phải đóng lượt bằng end,disk_low')
                    if n == 34:
                        motor_failed(run.events()[mark:], run.text())
        if n == 31:
            with running(cfg, root / 'startup', env={'UCR_DRIVE_FAKE_FREE_MB': '1023'}) as run:
                run.finish('disk_low')
                limits(run.events(), moving=False)
        return

    if n == 37:
        for device in ('camera', 'motor', 'servo'):
            with running(cfg, root / device, env={'UCR_DRIVE_FAKE_OPEN_FAIL': device}) as run:
                run.finish('motor_init_fail' if device == 'motor' else 'open_fail',
                           cleanup=False, summary=False)
                limits(run.events(), moving=False)
                if device == 'motor':
                    need('KIEM TRA XE, DUNG CONG TAC NGUON' in run.text(), 'Lỗi Init thiếu cảnh báo')
                elif device == 'camera':
                    need('test_motor/test_servo/main/ucr_collect_cam' in run.text(), 'Thiếu lời nhắc đóng chương trình')
                if matched(run.events(), 'servo_angle'):
                    final_stop(run.events())
        return

    if n == 40:
        for reason in ('esc', 'sigterm', 'write_error'):
            with running(cfg, root / reason, args=['--interval-ms', '100']) as run:
                moving(run)
                if reason == 'sigterm':
                    run.process.send_signal(signal.SIGTERM)
                elif reason == 'write_error':
                    run.inject('csv_fail')
                run.finish(reason)
                need(termios.tcgetattr(run.slave) == run.before, 'Không khôi phục termios: ' + reason)
        return

    if n in (49, 50, 51, 55):
        return artifacts(n, cfg)
    if n in (56, 57, 58):
        return wrapper_case(n, cfg, root)

    if n in (16, 17, 53):
        return key_variants(n, cfg, root)

    if n == 36:
        lock = root / 'shared.lock'
        with running(cfg, root / 'owner', lock=lock) as owner:
            owner.boot()
            with running(cfg, root / 'contender', lock=lock) as other:
                other.finish('lock_held', cleanup=False, summary=False)
                need(not any(e[1] in ('motor_init', 'motor_speed', 'motor_emergency', 'servo_angle')
                             for e in other.events()), 'Tiến trình tranh khóa can thiệp phần cứng')
                need('test_motor/test_servo/main/ucr_collect_cam' in other.text(), 'Thiếu cảnh báo khóa')
            owner.ready()
            owner.drive()
            owner.finish()
        with running(cfg, root / 'successor', lock=lock) as run:
            run.boot()
            run.finish()
        return

    if n == 62:
        for fault, reason in (('camera_stall', 'camera_stall'), ('motor_fail', 'motor_fail')):
            lock = root / (fault + '.lock')
            with running(cfg, root / (fault + '1'), lock=lock) as run:
                moving(run)
                run.inject(fault)
                if fault == 'motor_fail':
                    run.inject('free_mb 1')
                run.finish(reason)
            with running(cfg, root / (fault + '2'), lock=lock) as run:
                run.boot()
                run.observe(0.2, lambda: limits(run.events(), moving=False))
                run.finish()
        return

    if n == 6:
        for speed in (5, 3):
            with running(cfg, root / str(speed), args=['--speed-cms', str(speed)]) as run:
                run.boot()
                run.ready()
                for _ in range(5):
                    run.drive()
                need(all(e[2] in (0, speed) for e in run.events() if e[1] == 'motor_speed'),
                     'W tích lũy tốc độ')
                run.finish()
        return

    if n == 28:
        blocked = root / 'blocked'
        blocked.mkdir(mode=0o500)
        try:
            try:
                (blocked / 'probe').write_text('x', encoding='utf-8')
            except PermissionError:
                pass
            else:
                raise SafetyFailure('setup', 'Tài khoản vẫn ghi được thư mục cấm; chạy không dùng root')
            with running(cfg, root / 'run', out_root=blocked) as run:
                run.finish('write_error', cleanup=False, summary=False)
                limits(run.events(), moving=False)
                if matched(run.events(), 'servo_angle'):
                    final_stop(run.events())
        finally:
            blocked.chmod(0o700)
        return

    if n == 32:
        for free, minimum in ((1024, 1024), (1025, 1024), (2000, 2000)):
            with running(cfg, root / str(free), env={'UCR_DRIVE_FAKE_FREE_MB': str(free)},
                         args=['--min-free-mb', str(minimum)]) as run:
                moving(run)
                run.finish()
        return

    if n == 19:
        for hold in (800, 700):
            with running(cfg, root / str(hold), args=['--hold-timeout-ms', str(hold)]) as run:
                run.boot()
                run.ready()
                run.keep_driving()
                run.observe((hold + 300) / 1000, lambda: need(
                    matched(run.events(), 'state')[-1][1][2] == 'CHAY', 'Dừng khi W vẫn còn được cấp'))
                run.pump = False
                mark = len(run.events())
                key = [e for e in run.events() if e[1] == 'key' and str(e[2]).upper() == 'W'][-1]
                stop_by(run, mark, key[0], hold, 'deadman')
                run.event('state', 'DUNG', mark)
                run.drive()
                run.finish()
        return

    if n == 48:
        # Mọi finish() kiểm G5; ca này tạo đủ cực đại âm/dương và bộ đếm X.
        with running(cfg, root / 'summary', args=['--interval-ms', '100']) as run:
            moving(run)
            for target in (0, -5, -10):
                steer(run, 'D', target)
            run.key('X')
            run.pump = False
            run.finish()
        return

    env = {'UCR_DRIVE_FAKE_CAMERA': 'none'} if n == 26 else (
          {'UCR_DRIVE_FAKE_MOTOR_FAIL': '1'} if n == 33 else None)
    args = ['--interval-ms', '100'] if n in (43, 44, 45, 47) else []
    with running(cfg, root / 'run', env=env, args=args) as run:
        if n == 26:
            run.boot(images=False)
            camera_start = matched(run.events(), 'servo_angle', 0)[0][1][0]
            # R/Y bị bỏ khi chưa từng có khung, kể cả liên tục cấp phím.
            for _ in range(10):
                run.send('RYW')
                run.observe(0.3, lambda: limits(run.events(), moving=False))
            run.finish('camera_no_frame')
            limits(run.events(), moving=False)
            need(monotonic_ms() - camera_start <= 5500 + cfg.slack + cfg.tsan_slack,
                 'Khung đầu chờ quá hạn 5 s sau Init')
            return
        if n == 33:
            # §12.4: ghi lệnh 0 ngay sau Init hỏng là lỗi khởi tạo, không còn mã 5.
            run.finish('motor_init_fail', summary=False)
            motor_failed(run.events(), run.text())
            return
        if n in (1, 2):
            run.boot()
            need(matched(run.events(), 'state')[0][1][2] == 'DUNG',
                 'Không khởi động DUNG', 'startup_stopped')
            run.observe(0.2, lambda: limits(run.events(), moving=False))
            if n == 2:
                # Bỏ qua helper phục hồi: W đầu không được chịu khoảng chờ H.
                mark = len(run.events())
                key = run.key('W')
                event = run.event('motor_speed', 5, mark)
                need(event[0] - key[0] <= 200 + cfg.slack,
                     'W đầu không chạy ngay', 'initial_w')
            run.finish()
            return
        run.boot()
        if n == 14:
            for _ in range(2):
                mark = len(run.events())
                run.key('X')
                run.event('servo_angle', 0, mark)
                ordered_stop(run.events()[mark:])
                run.key('W')
            limits(run.events(), moving=False)
        elif n == 25:
            run.ready()
            run.keep_driving()
            mark, stamp = len(run.events()), monotonic_ms()
            run.pump = False
            os.close(run.master)
            run.master = None
            stop_by(run, mark, stamp)
            run.wait(lambda: run.process.poll() is not None, 5, alive=False)
            reason = 'sighup' if run.process.returncode == 129 else 'stdin_eof'
            run.finish(reason, codes={4, 129})
            # Lỗi đọc riêng có tên điều khiển, không bị nhầm với HUP.
            with running(cfg, root / 'stdin_error') as other:
                moving(other)
                mark = len(other.events())
                stamp = other.inject('stdin_error')
                stop_by(other, mark, stamp)
                other.finish('stdin_eof')
            return
        elif n == 27:
            run.ready()
            run.keep_driving()
            mark = len(run.events())
            stamp = run.inject('camera_stall')
            stop_by(run, mark, stamp, 500)
            run.finish('camera_stall')
            no_restart(run.events()[mark:])
            return
        elif n == 41:
            text = run.text().lower()
            need('tof' in text and ('không' in text or 'khong' in text), 'Thiếu cảnh báo không phanh ToF')
            need('giám sát' in text or 'giam sat' in text, 'Thiếu cảnh báo người giám sát')
            need('ctrl+c' in text and ('một' in text or 'mot' in text), 'Thiếu cảnh báo Ctrl+C')
            run.ready()
            run.drive()
        elif n == 47:
            folder = run.session_dir()
            sentinel = folder / 'sentinel'
            sentinel.write_bytes(b'GIU NGUYEN')
            run.finish()
            with running(cfg, root / 'again', out_root=run.out) as other:
                other.wait(lambda: len(list(run.out.glob('*/session.json'))) == 2, 10)
                other.key('\x1b')
                other.wait(lambda: other.process.poll() is not None, 5, alive=False)
                need(other.process.returncode == 0, 'Phiên sau không thoát sạch')
                final_stop(other.events())
                need(sentinel.read_bytes() == b'GIU NGUYEN', 'Ghi đè phiên cũ')
            return
        elif n in (44, 45, 46):
            count = 3 if n == 46 else 2
            run.wait(lambda: len(run.rows()) >= count, seconds=10)
            rows = run.rows()
            for row in rows:
                size, tables = read_jpeg(run.out / row['filename'])
                need(size == (1280, 720), 'Ảnh giả sai cỡ nguồn')
                if n == 45:
                    quality90(tables)
            if n == 45:
                def check_committed():
                    for path in run.out.glob('*.jpg'):
                        read_jpeg(path)
                    for row in run.rows():
                        read_jpeg(run.out / row['filename'])
                run.observe(0.4, check_committed)
            if n == 46:
                times = [int(row['monotonic_ns']) for row in rows]
                need(all(b - a >= 1000000000 for a, b in zip(times, times[1:])),
                     'Chu kỳ mặc định ngắn hơn 1000 ms')
                session = json.loads((run.session_dir() / 'session.json').read_text(encoding='utf-8'))
                # G5 không hứa trường pipeline_used; chuỗi mặc định kiểm ở hiện vật.
                data = Path(cfg.real).read_bytes()
                for part in (b'sensor-mode=5', b'wbmode=0', b'width=1280,height=720', b'framerate=60/1'):
                    need(part in data, 'Thiếu thành phần pipeline mặc định: ' + repr(part))
                need(session['width'] == 1280 and session['height'] == 720, 'Session sai cỡ')
        else:
            run.ready()
            if n == 3:
                mark = len(run.events())
                run.observe(1, lambda: limits(run.events()[mark:], moving=False))
            elif n in (9, 11):
                if n == 11:
                    for key, target in (('A', 5), ('C', 0), ('D', -5), ('C', 0)):
                        steer(run, key, target)
                run.keep_driving()
                for target in (5, 10, 10, 10):
                    steer(run, 'A', target)
                if n == 11:
                    steer(run, 'C', 0)
                for target in ((-5, -10, -10) if n == 11 else (5, 0, -5, -10, -10)):
                    steer(run, 'D', target)
                if n == 11:
                    steer(run, 'C', 0)
            elif n == 12:
                for stop_key, turn, angle in (('S', 'A', 5), (' ', 'D', -5)):
                    run.keep_driving()
                    steer(run, turn, angle)
                    run.pump = False
                    mark = len(run.events())
                    key = run.key(stop_key)
                    stop_by(run, mark, key[0])
                    run.observe(1, lambda: need(not any(e[1] == 'servo_angle' and e[2] != angle
                                                                     for e in run.events()[mark:]),
                                               'S/Space đổi góc lái'))
                    steer(run, 'C', 0)
            elif n == 13:
                run.keep_driving()
                steer(run, 'A', 5)
                run.pump = False
                mark = len(run.events())
                key = run.key('X')
                stop_by(run, mark, key[0])
                run.event('servo_angle', 0, mark)
                ordered_stop(run.events()[mark:], 'x_order')
                run.event('state', 'DUNG', mark)
                after = len(run.events())
                run.key('W')
                run.observe(0.3, lambda: limits(run.events()[after:], moving=False))
            elif n == 15:
                run.keep_driving()
                steer(run, 'A', 5)
            elif n in (18, 20):
                mark = len(run.events())
                key = run.key('W')
                run.event('motor_speed', 5, mark)
                stop_mark = len(run.events())
                if n == 20:
                    for key_name in ('A', 'D', 'C', '?'):
                        run.key(key_name)
                        run.observe(0.15)
                stop_by(run, stop_mark, key[0], 800, 'deadman')
                run.event('state', 'DUNG', stop_mark)
            elif n == 43:
                run.wait(lambda: any(r['state'] == 'DUNG' for r in run.rows()))
                run.keep_driving()
                steer(run, 'A', 5)
                run.wait(lambda: any(r['state'] == 'CHAY' and float(r['cmd_speed']) == 5 and
                                     float(r['cmd_steer']) == 5 for r in run.rows()))
                check_capture_states(run)
            else:
                raise AssertionError('Chưa cài đặt ' + case)
        run.finish()
        if n == 46:
            pipeline = 'videotestsrc is-live=true ! video/x-raw,width=320,height=240 ! videoconvert ! appsink'
            with running(cfg, root / 'override', args=['--interval-ms', '250', '--pipeline', pipeline]) as other:
                other.boot()
                other.keep_driving()
                other.wait(lambda: len([row for row in other.rows() if row['state'] == 'CHAY']) >= 3, 5)
                stamps = [int(row['monotonic_ns']) for row in other.rows() if row['state'] == 'CHAY']
                need(all(b - a >= 250000000 for a, b in zip(stamps, stamps[1:])), 'Không giữ interval 250 ms')
                need(stamps[2] - stamps[0] < 1800000000, 'Bỏ qua interval ghi đè, vẫn dùng 1000 ms')
                other.finish()
                session = json.loads((other.session_dir() / 'session.json').read_text(encoding='utf-8'))
                need(pipeline in str(session['argv']), 'Không lưu argv pipeline ghi đè')


def init_case(n, cfg, root):
    """§12–§13: giữ cận Init; chỉ chờ thoát sau tín hiệu có dung sai TSan."""
    if n == 84:
        return wrapper_preflight(cfg, root)
    if n == 80:
        with running(cfg, root / 'sequence') as run:
            run.boot()
            for _ in range(2):
                run.ready()
                run.drive()
                mark = len(run.events())
                run.key('X')
                run.pump = False
                run.event('state', 'DUNG', mark)
            run.finish()
            init_sequence(run.events(), required=True, complete=True)
        return
    if n == 81:
        for sig, reason in ((signal.SIGTERM, 'sigterm'), (signal.SIGINT, 'sigint'),
                            (signal.SIGHUP, 'sighup'), (signal.SIGTSTP, 'sigtstp')):
            with running(cfg, root / reason, env={'UCR_DRIVE_FAKE_INIT_MS': '1500'}) as run:
                event = run.event('motor_init', seconds=10)
                stamp = monotonic_ms()
                need(stamp - event[0] < 1000, 'Không còn đủ cửa sổ gửi tín hiệu trong Init', 'setup')
                run.process.send_signal(sig)
                run.send('RYW')
                deadline = event[0] + 1500 + 300 + cfg.tsan_slack
                run.wait(lambda: run.process.poll() is not None,
                         max(0.01, (deadline - monotonic_ms()) / 1000),
                         'Tín hiệu trong Init thoát quá hạn', alive=False)
                need(monotonic_ms() <= deadline, 'Quá Init còn lại +300 ms + dung sai TSan')
                need(run.process.returncode == REASONS[reason], 'Sai mã tín hiệu trong Init: ' + run.text())
                events = run.events()
                init_sequence(events, required=True, complete=True)
                first_zero = matched(events, 'motor_speed', 0)[0][1]
                need(first_zero[0] >= event[0] + 1495,
                     'Tín hiệu đã bỏ dở Init trước 1500 ms (dung sai mốc vết 5 ms)')
                need(matched(events, 'servo_angle', 0)[0][1][0] <= event[0] + 1500 + 300,
                     'Cặp dừng sau Init quá hạn; không áp dung sai TSan')
                need(not matched(events, 'key'), 'Đã xử lý phím dù có tín hiệu trong Init')
                limits(events, moving=False)
                final_stop(events)
        return
    env = {'UCR_DRIVE_FAKE_OPEN_FAIL': 'motor_init'} if n == 82 else {
        'UCR_DRIVE_FAKE_INIT_MS': '8000'}
    with running(cfg, root / 'failure', env=env) as run:
        event = run.event('motor_init', seconds=10)
        # Pha Init không thuộc mốc 200 ms; TSan thoát chậm hơn nên thêm 1 s riêng cho sanitizer.
        deadline = event[0] + 5500 + (1000 if cfg.slack else 0)
        run.wait(lambda: run.process.poll() is not None,
                 max(0.01, (deadline - monotonic_ms()) / 1000), 'Init không thoát có hạn', alive=False)
        need(monotonic_ms() <= deadline, 'Init quá 5,5 s')
        need(run.process.returncode == 3, 'Lỗi/watchdog Init phải mã 3: ' + run.text())
        need('KIEM TRA XE, DUNG CONG TAC NGUON' in run.text(), 'Lỗi/watchdog Init thiếu cảnh báo')
        events = run.events()
        init_sequence(events, required=True)
        limits(events, moving=False)
        need(not matched(events, 'key'), 'Xử lý phím khi Init hỏng/treo')


def burst_deadman(events, hold, slack):
    """fix4 H2: đo từ W ĐẦU của lượt gửi duy nhất, tránh hàng tồn tự gia hạn."""
    keys = matched(events, 'key', 'W')
    need(keys and matched(events, 'motor_speed', 5), 'Lượt W chưa làm xe chạy', 'setup')
    tail = events[keys[0][0]:]
    stops = matched(tail, 'motor_speed', 0)
    need(stops and stops[0][1][0] <= keys[0][1][0] + hold + 200 + slack,
         'Hàng W tồn kéo dài deadman quá hold+200 ms từ lượt đọc đầu', 'burst_deadman')
    no_restart(tail)


@contextmanager
def stderr_pipe():
    """Giữ bên đọc mở nhưng không đọc: write bị chặn thay vì nhận EPIPE."""
    reader, writer = os.pipe()
    try:
        yield reader, writer
    finally:
        os.close(reader)
        os.close(writer)


def fill_pipe(writer):
    flags = fcntl.fcntl(writer, fcntl.F_GETFL)
    fcntl.fcntl(writer, fcntl.F_SETFL, flags | os.O_NONBLOCK)
    try:
        for size in (4096, 1):
            while True:
                try:
                    os.write(writer, b'!' * size)
                except BlockingIOError:
                    break
    finally:
        fcntl.fcntl(writer, fcntl.F_SETFL, flags)


def drain_pipe(reader):
    # Chỉ tháo nghẽn SAU khi con đã thoát; trình bao còn cần in cảnh báo.
    flags = fcntl.fcntl(reader, fcntl.F_GETFL)
    fcntl.fcntl(reader, fcntl.F_SETFL, flags | os.O_NONBLOCK)
    try:
        while True:
            try:
                if not os.read(reader, 65536):
                    break
            except BlockingIOError:
                break
    finally:
        fcntl.fcntl(reader, fcntl.F_SETFL, flags)


def init_write_failure(n, cfg, root):
    """N3/§12.4–6: lỗi ghi đầu/giữa Init không bị lệnh thành công sau đó che mất."""
    env = {'UCR_DRIVE_FAKE_INIT_WRITE_FAIL': str(n - 91)}
    with running(cfg, root / 'init_write_failure', env=env) as run:
        deadline = run.started + 5500
        run.wait(lambda: run.process.poll() is not None,
                 max(0.01, (deadline - monotonic_ms()) / 1000),
                 'Lỗi ghi Init không thoát trong 5,5 s', alive=False)
        need(monotonic_ms() <= deadline, 'Lỗi ghi Init quá 5,5 s')
        events = run.finish('motor_init_fail', cleanup=False, summary=False)
        need('KIEM TRA XE, DUNG CONG TAC NGUON' in run.text(), 'Lỗi ghi Init thiếu cảnh báo')
        init_sequence(events, required=True)
        need(not any(e[1] == 'motor_speed' and e[2] > 0 for e in events),
             'Có lệnh tốc độ dương khi ghi Init thất bại')
        for path in run.out.glob('*/summary.json'):
            summary = json.loads(path.read_text(encoding='utf-8'))
            need(summary.get('exit_reason') == 'motor_init_fail',
                 'Sai lý do trong summary khi ghi Init thất bại: ' + repr(summary))


def fix4_case(n, cfg, root):
    """T21_test_fix4 H2/N1/N2/N4; N3 cần giao diện tiêm lỗi ghi giữa Init."""
    if n == 85:
        for hold in (700, 800):
            with running(cfg, root / str(hold), args=['--hold-timeout-ms', str(hold)]) as run:
                run.boot()
                run.ready()
                run.key('?')  # D85 kiểm tuổi hàng đang đọc, không phải lô cũ sau khoảng im (D98).
                mark = len(run.events())
                need(os.write(run.master, b'W' * 4000) == 4000, 'PTY chưa nhận trọn một lượt 4.000 W', 'setup')
                key = run.event('key', 'W', mark)
                stop_by(run, mark, key[0], hold, 'burst_deadman')
                run.observe(1.2, lambda: burst_deadman(run.events()[mark:], hold, cfg.slack))
                run.finish()
        return
    if n in (86, 87, 88):
        return cleanup_deadline(n, cfg, root)
    if n in (89, 90):
        return wrapper_lock_order(n, cfg, root)
    need(n == 91, 'Ca fix4 không biết')
    # N4/§12.3: tín hiệu thật ở GIỮA Init; đủ cả SIGQUIT được bổ sung ở fix4.
    for sig, reason in ((signal.SIGINT, 'sigint'), (signal.SIGTERM, 'sigterm'),
                        (signal.SIGHUP, 'sighup'), (signal.SIGQUIT, 'sigterm'),
                        (signal.SIGTSTP, 'sigtstp')):
        with running(cfg, root / str(sig), env={'UCR_DRIVE_FAKE_INIT_MS': '1500'}) as run:
            init = run.event('motor_init', seconds=10)
            run.wait(lambda: monotonic_ms() >= init[0] + 650, 2, 'Không tới giữa Init')
            need(monotonic_ms() < init[0] + 1000, 'Mất cửa sổ giữa Init', 'setup')
            run.process.send_signal(sig)
            run.send('RYW')
            deadline = init[0] + 1500 + 300 + cfg.tsan_slack
            run.wait(lambda: run.process.poll() is not None,
                     max(0.01, (deadline - monotonic_ms()) / 1000),
                     'Tín hiệu giữa Init thoát quá Init còn lại +300 ms + dung sai TSan', alive=False)
            need(monotonic_ms() <= deadline, 'Quá hạn tín hiệu giữa Init')
            need(run.process.returncode == REASONS[reason], 'Sai mã tín hiệu: ' + run.text())
            events = run.events()
            init_sequence(events, required=True, complete=True)
            need(matched(events, 'motor_speed', 0)[0][1][0] >= init[0] + 1495,
                 'Tín hiệu cắt ngắn INIT_MS (dung sai vết 5 ms)', 'init_interrupted')
            need(matched(events, 'servo_angle', 0)[0][1][0] <= init[0] + 1500 + 300,
                 'Cặp dừng sau Init quá hạn; không áp dung sai TSan')
            need(not matched(events, 'key'), 'Xử lý phím dù đang chờ tín hiệu trong Init')
            limits(events, moving=False)
            final_stop(events)


def check_tsan_logs(folder):
    for path in folder.glob('tsan.*'):
        need(b'WARNING: ThreadSanitizer' not in path.read_bytes(),
             'TSan báo lỗi luồng trong ' + str(path), 'tsan_warning')


def cleanup_deadline(n, cfg, root):
    """N1: hạn 3 s sau cặp dừng; +300 ms lịch, +300 ms sanitizer, dung sai TSan theo ca."""
    modes = ('stderr', 'json') if n == 88 else (('stderr',) if n == 86 else ('json',))
    reasons = ('disk_low',) if n == 88 else ('disk_low', 'motor_fail', 'sigterm')
    for mode in modes:
        for reason in reasons:
            folder = root / (mode + reason)
            folder.mkdir(parents=True)
            env, command, child_mark, journal = {}, None, folder / 'child', folder / 'journal'
            if mode == 'stderr' and cfg.tsan_slack:
                # TSan ghi ra tệp riêng để không bị chặn bởi pipe stderr cố ý làm đầy.
                env['TSAN_OPTIONS'] = ':'.join(filter(None, (
                    os.environ.get('TSAN_OPTIONS', ''), 'log_path="{}"'.format(folder / 'tsan'))))
            if n == 88:
                build = folder / 'build with spaces'
                build.mkdir()
                fake = folder / 'fake.py'
                fake.write_text(
                    'import json, os, sys\nfrom pathlib import Path\n'
                    'kind = sys.argv[1]\n'
                    'with open(os.environ["JOURNAL"], "a", encoding="utf-8") as f:\n'
                    ' f.write(json.dumps([kind]) + "\\n")\n'
                    'if kind == "child":\n'
                    ' Path(os.environ["CHILD_MARK"]).write_text(str(os.getpid()), encoding="utf-8")\n'
                    ' os.execv(os.environ["HOOKS"], [os.environ["HOOKS"]] + sys.argv[2:])\n',
                    encoding='utf-8')
                prefix = '#!/bin/sh\nexec {} {} '.format(shlex.quote(sys.executable), shlex.quote(str(fake)))
                executable(build / 'ucr_stop', prefix + 'stop "$@"\n')
                executable(build / 'ucr_drive_collect', prefix + 'child "$@"\n')
                env.update(UCR_BUILD_DIR=str(build), JOURNAL=str(journal), CHILD_MARK=str(child_mark), HOOKS=cfg.hooks)
                command = [cfg.bash, isolated_wrapper(cfg, folder), '--accept-no-obstacle-brake',
                           '--out-root', str(folder / 'run' / 'out'), '--interval-ms', '100', '--rec-on-start']
            with stderr_pipe() as (reader, writer):
                with running(cfg, folder / 'run', env=env, command=command,
                             args=['--interval-ms', '100'], stderr=writer if mode == 'stderr' else None) as run:
                    moving(run)
                    if mode == 'stderr':
                        fill_pipe(writer)
                    else:
                        summary = run.session_dir() / 'summary.json'
                        need(not summary.exists(), 'Summary đã tồn tại trước khi thoát', 'setup')
                        os.mkfifo(str(summary))  # Không có bên đọc, open(ghi) phải đợi.
                    run.pump = False
                    mark = len(run.events())
                    stamp = monotonic_ms()
                    if reason == 'sigterm':
                        run.process.send_signal(signal.SIGTERM)
                    else:
                        run.inject('free_mb 1' if reason == 'disk_low' else 'motor_fail')
                        if reason == 'motor_fail':
                            run.send('S')
                    stop_by(run, mark, stamp)
                    run.event('servo_angle', 0, mark)
                    stopped = matched(run.events()[mark:], 'servo_angle', 0)[0][1][0]
                    deadline = stopped + 3000 + 300 + cfg.slack + cfg.tsan_slack
                    pid = int(child_mark.read_text(encoding='utf-8')) if n == 88 else run.process.pid
                    exited = (lambda: not Path('/proc/{}/stat'.format(pid)).exists()) if n == 88 else (
                        lambda: run.process.poll() is not None)
                    run.wait(exited, max(0.01, (deadline - monotonic_ms()) / 1000),
                             'Cleanup nghẽn không thoát trong 3 s + dung sai', alive=False)
                    need(monotonic_ms() <= deadline, 'Cleanup quá hạn 3 s + dung sai')
                    if n == 88:
                        run.wait(lambda: wrapper_records(journal) == [['stop'], ['child'], ['stop']],
                                 5, 'Không gọi stop sau con khi stderr/JSON còn nghẽn', alive=False)
                        if mode == 'stderr':
                            drain_pipe(reader)
                        run.wait(lambda: run.process.poll() is not None, 5,
                                 'Trình bao không thoát sau con', alive=False)
                        need(wrapper_records(journal) == [['stop'], ['child'], ['stop']],
                             'Cleanup nghẽn đã bỏ stop sau con hoặc sai thứ tự')
                    if mode == 'stderr' and cfg.tsan_slack:
                        check_tsan_logs(folder)
                    need(run.process.returncode == REASONS[reason], 'Cleanup đổi mã đang thoát: ' + run.text())
                    events = run.events()
                    init_sequence(events, required=True, complete=True)
                    final_stop(events)
                    no_restart(events[mark:])
                    if reason == 'motor_fail':
                        need(len(matched(events[mark:], 'motor_speed', 0) +
                                 matched(events[mark:], 'motor_emergency')) >= 4, 'Thiếu 4 lần dừng lỗi')


def wrapper_lock_order(n, cfg, root):
    """N2: hàng rào bằng tệp, khóa thử thật, stub (chương trình giả) ghi thứ tự."""
    for code in ((0,) if n == 89 else (0, 7)):
        folder = root / str(code)
        folder.mkdir(parents=True)
        build = folder / 'build'
        build.mkdir()
        lock, journal = folder / 'vehicle.lock', folder / 'journal'
        fake = folder / 'fake.py'
        fake.write_text(
            'import json, os, sys, time\nfrom pathlib import Path\n'
            'root = Path(os.environ["GATES"])\n'
            'kind = sys.argv[1]\n'
            'stage = "child" if kind == "child" else ("post" if (root / "child_seen").exists() else "pre")\n'
            'def record(value):\n'
            ' with open(os.environ["JOURNAL"], "a", encoding="utf-8") as f: f.write(json.dumps([value]) + "\\n")\n'
            'record(stage + "_enter")\n'
            'if stage == "child": (root / "child_seen").touch()\n'
            'end = time.monotonic() + 8\n'
            'while not (root / (stage + "_release")).exists():\n'
            ' if time.monotonic() >= end: sys.exit(99)\n'
            ' time.sleep(.005)\n'
            'record(stage + "_exit")\n'
            'sys.exit(int(os.environ["CHILD_CODE"]) if stage == "child" else 0)\n', encoding='utf-8')
        prefix = '#!/bin/sh\nexec {} {} '.format(shlex.quote(sys.executable), shlex.quote(str(fake)))
        executable(build / 'ucr_drive_collect', prefix + 'child\n')
        executable(build / 'ucr_stop', prefix + 'stop\n')
        env = dict(UCR_BUILD_DIR=str(build), UCR_DRIVE_FAKE_LOCK=str(lock),
                   JOURNAL=str(journal), GATES=str(folder), CHILD_CODE=str(code))
        with running(cfg, folder / 'run', env=env,
                     command=[cfg.bash, isolated_wrapper(cfg, folder)]) as run:
            run.wait(lambda: ['pre_enter'] in wrapper_records(journal), 5, 'Không tới stop trước')
            with lock.open('a', encoding='utf-8') as probe:
                def locked():
                    try:
                        fcntl.flock(probe, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    except BlockingIOError:
                        return
                    fcntl.flock(probe, fcntl.LOCK_UN)
                    need(False, 'Trình bao nhả khóa trong khi ucr_stop còn chạy', 'wrapper_lock')
                run.observe(0.2, locked)
            need(wrapper_records(journal) == [['pre_enter']], 'Chạy con trước khi stop trước xong')
            (folder / 'pre_release').touch()
            run.wait(lambda: ['child_enter'] in wrapper_records(journal), 5, 'Không chạy con sau stop trước')
            with lock.open('a', encoding='utf-8') as owner:
                # Lấy được khóa cũng chứng minh đã nhả khóa trước khi chạy con.
                fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
                if n == 89:
                    fcntl.flock(owner, fcntl.LOCK_UN)
                    (folder / 'post_release').touch()
                (folder / 'child_release').touch()
                run.wait(lambda: run.process.poll() is not None, 5, 'Cleanup không tranh khóa không chờ', alive=False)
                expected = [['pre_enter'], ['pre_exit'], ['child_enter'], ['child_exit']]
                if n == 89:
                    expected += [['post_enter'], ['post_exit']]
                need(wrapper_records(journal) == expected, 'Sai thứ tự hoặc vẫn stop khi khóa cleanup bận')
                need(run.process.returncode == (0 if n == 89 else (6 if code == 0 else code)),
                     'Sai mã khi cleanup bận: ' + run.text())
                if n == 90:
                    need(run.text().strip(), 'Cleanup bận không có lời nhắc')


def motor_failed(events, text):
    attempts = [e for e in events if e[1] == 'motor_emergency' or (e[1] == 'motor_speed' and e[2] == 0)]
    need(len(attempts) >= 4, 'Cần lần đầu + 3 thử lại')
    need(all(e[3] for e in attempts), 'Vết motor lỗi thiếu đuôi fail')
    need(20 <= attempts[3][0] - attempts[0][0] <= 400,
         'Ba lần thử lại không có khoảng cách xấp xỉ 20 ms (dung sai tổng 20–400 ms)')
    need('KIEM TRA XE, DUNG CONG TAC NGUON' in text, 'Thiếu cảnh báo lớn motor')
    final_stop(events)


def check_capture_states(run):
    events = run.events()
    changes, latest = [], {}
    for event in events:
        if event[1] in ('state', 'motor_speed', 'servo_angle') and latest.get(event[1]) != event[2]:
            changes.append(event)
            latest[event[1]] = event[2]
    checked = set()
    for row in run.rows():
        ms = int(row['monotonic_ns']) / 1000000
        # Bỏ ảnh cùng ms chuyển trạng thái: vết chỉ có độ phân giải ms.
        if any(abs(e[0] - ms) <= 1 for e in changes):
            continue
        prior = [e for e in events if e[0] < ms]
        for kind, column in (('motor_speed', 'cmd_speed'), ('servo_angle', 'cmd_steer'), ('state', 'state')):
            values = matched(prior, kind)
            if values:
                expected = values[-1][1][2]
                actual = row[column] if kind == 'state' else float(row[column])
                need(actual == expected, 'CSV khác lệnh/trạng thái tại lúc chụp: ' + column)
        checked.add(row['state'])
    need({'DUNG', 'CHAY'} <= checked, 'Thiếu ảnh có mốc ổn định ở đủ hai trạng thái')


def no_restart(events):
    stops = matched(events, 'motor_speed', 0) + matched(events, 'motor_emergency')
    need(stops, 'Không có dừng để kiểm chốt lỗi')
    first = min(i for i, _ in stops)
    need(not any(e[1] == 'motor_speed' and e[2] > 0 for e in events[first + 1:]),
         'Lệnh tiến xuất hiện sau khi đã dừng lỗi')


def added_case(n, cfg, root):
    """Ca bổ sung từ thiết kế §9–§11; không suy kỳ vọng từ triển khai."""
    if n == 65:
        with running(cfg, root / 'before_camera', env={'UCR_DRIVE_FAKE_OPEN_FAIL': 'camera'}) as run:
            run.finish('open_fail', summary=False)
            ordered_stop(run.events())
            limits(run.events(), moving=False)
        return
    if n == 66:
        for hold in (700, 2000):
            with running(cfg, root / str(hold), args=['--hold-timeout-ms', str(hold)]) as run:
                run.boot()
                run.ready()
                run.key('?')  # D66 kiểm deadman của hàng liên tục; lô sau im được kiểm riêng ở D98.
                mark = len(run.events())
                run.send('W' * 64)
                run.wait(lambda: len([e for e in run.events()[mark:] if e[1] == 'key' and
                                      str(e[2]).upper() == 'W']) >= 64)
                run.event('motor_speed', 5, mark)
                last_w = [e for e in run.events() if e[1] == 'key' and str(e[2]).upper() == 'W'][-1]
                stop_by(run, len(run.events()), last_w[0], hold, 'deadman')
                run.finish()
        return
    if n == 67:
        for key in ('S', ' ', 'X'):
            with running(cfg, root / str(ord(key))) as run:
                moving(run)
                run.pump = False
                mark = len(run.events())
                run.send(key + 'W' * 40)
                run.event('motor_speed', 0, mark)
                run.observe(1.1, lambda: no_restart(run.events()[mark:]))
                run.finish()
        return
    if n in (68, 74):
        for fault in ('jpeg_hang', 'csv_hang'):
            signals = [(None, 'camera_stall')] if n == 68 else [
                (signal.SIGINT, 'sigint'), (signal.SIGTERM, 'sigterm'),
                (signal.SIGHUP, 'sighup'), (signal.SIGQUIT, 'sigterm'),
                (signal.SIGTSTP, 'sigtstp')]
            for sig, reason in signals:
                folder = fault + reason if sig is None else fault + '_' + str(sig)
                with running(cfg, root / folder, args=['--interval-ms', '100']) as run:
                    moving(run)
                    mark = len(run.events())
                    stamp = run.inject(fault)
                    if sig is not None:
                        # Phím C qua hai vòng là hàng rào: tệp CTRL đã được poll
                        # trong ít nhất một vòng trước lúc gửi tín hiệu.
                        run.key('C')
                        run.key('C')
                        run.observe(0.3, lambda: need(not matched(run.events()[mark:], 'motor_speed', 0),
                                                    'Đã dừng trước khi tiêm tín hiệu'))
                        stamp = monotonic_ms()
                        run.process.send_signal(sig)
                    stop_by(run, mark, stamp, 2000 if sig is None else 0)
                    run.finish(reason, summary=False)
                    no_restart(run.events()[mark:])
                    summaries = list(run.out.glob('*/summary.json'))
                    if summaries:
                        run.check_output(reason, REASONS[reason])
        return
    if n == 69:
        for byte, reason in ((b'\x03', 'sigint'), (b'\x1c', 'sigterm'), (b'\x1a', 'sigtstp')):
            with running(cfg, root / reason) as run:
                moving(run)
                need(termios.tcgetattr(run.slave)[3] & termios.ISIG, 'Terminal đã tắt ISIG')
                mark, stamp = len(run.events()), monotonic_ms()
                run.send(byte)
                stop_by(run, mark, stamp)
                run.finish(reason)
                need(termios.tcgetattr(run.slave) == run.before, 'Không khôi phục terminal sau Ctrl')
        return
    if n == 70:
        with running(cfg, root / 'servo') as run:
            moving(run)
            mark = len(run.events())
            stamp = run.inject('servo_fail')
            # Đổi góc khác 0 để phép kiểm không phụ thuộc tối ưu bỏ lệnh lặp.
            run.key('D')
            stop_by(run, mark, stamp)
            run.finish('servo_fail')
            events = run.events()[mark:]
            attempts = matched(events, 'motor_speed', 0) + matched(events, 'motor_emergency')
            need(len(attempts) >= 4, 'Servo lỗi thiếu 4 lần yêu cầu dừng motor')
            need('KIEM TRA XE, DUNG CONG TAC NGUON' in run.text(), 'Servo lỗi thiếu cảnh báo nguồn')
            no_restart(events)
        return
    if n == 71:
        return test_case('D27', cfg, root)
    if n == 72:
        with running(cfg, root / 'slow', args=['--interval-ms', '100']) as run:
            moving(run)
            mark = len(run.events())
            run.inject('jpeg_slow')
            run.observe(3.4, lambda: need(not matched(run.events()[mark:], 'motor_speed', 0),
                                         'JPEG chậm làm báo nhầm mất khung'))
            need(matched(run.events()[mark:], 'motor_speed', 5), 'Không duy trì W khi JPEG chậm')
            run.finish()
        return
    if n == 73:
        with running(cfg, root / 'pause') as run:
            moving(run)  # Đã chạy và lái 5°; nhấn A lần nữa sẽ thành 10°.
            mark = len(run.events())
            stamp = run.inject('camera_pause_600ms')
            stop_by(run, mark, stamp, 500)
            run.event('servo_angle', 0, mark)
            ordered_stop(run.events()[mark:])
            run.event('state', 'DUNG', mark)
            # Dừng cấp W ngay khi dừng; W đã gửi lúc mất khung không được lưu lại.
            run.pump = False
            held_then_fresh(run, len(run.events()))
            run.finish()
        return
    if n == 75:
        with running(cfg, root / 'motor_hang') as run:
            moving(run)
            mark = len(run.events())
            stamp = run.inject('motor_hang')
            run.wait(lambda: run.process.poll() is not None, (1500 + cfg.slack) / 1000,
                     'Motor ghi treo không thoát trước 1,5 s', alive=False)
            need(monotonic_ms() - stamp <= 1500 + cfg.slack, 'Motor treo quá hạn')
            run.finish('motor_fail', summary=False)
            need('KIEM TRA XE, DUNG CONG TAC NGUON' in run.text(), 'Motor treo thiếu cảnh báo')
            no_restart(run.events()[mark:])
        return
    if n in (76, 77):
        return wrapper_guards(n, cfg, root)
    if n == 78:
        return stop_journal(cfg, root)
    if n == 79:
        rejected(cfg, root / 'cli', ['--lock-file', str(root / 'other.lock')])
        data = Path(cfg.real).read_bytes()
        need(b'--lock-file' not in data, 'Bản xe còn cờ đổi khóa')
        need(b'/tmp/ucr_vehicle.lock' in data, 'Không có khóa xe cố định trong hiện vật')
        return
    raise AssertionError('Chưa cài đặt D' + str(n))


def stop_journal(cfg, root):
    with running(cfg, root / 'journal', args=['--interval-ms', '100']) as run:
        moving(run)
        run.pump = False
        run.key('S')
        run.event('state', 'DUNG')
        run.drive()
        mark = len(run.events())
        key = run.key('W')
        stop_by(run, mark, key[0], 800, 'deadman')
        run.keep_driving()
        run.key('X')
        run.pump = False
        run.ready()
        run.keep_driving()
        mark, stamp = len(run.events()), monotonic_ms()
        run.process.send_signal(signal.SIGTERM)
        stop_by(run, mark, stamp)
        run.finish('sigterm')
        folder = run.session_dir()
        with (folder / 'stops.csv').open(encoding='utf-8', newline='') as stream:
            rows = list(csv.DictReader(stream))
        required = {'reason', 't_trigger_ms', 't_motor0_done_ms', 'latency_ms'}
        need(rows and all(required <= set(row) for row in rows), 'Nhật ký dừng thiếu cột')
        for reason in ('key_s', 'key_x', 'deadman', 'sigterm'):
            selected = [row for row in rows if row['reason'] == reason]
            need(len(selected) == 1, 'Cần đúng 1 dòng dừng cho ' + reason)
            row = selected[0]
            trigger, done, latency = (float(row[key]) for key in
                                      ('t_trigger_ms', 't_motor0_done_ms', 'latency_ms'))
            need(all(math.isfinite(value) for value in (trigger, done, latency)) and latency >= 0,
                 'Độ trễ dừng không hữu hạn/âm')
            need(abs(done - trigger - latency) <= 5, 'Độ trễ không khớp hai mốc')
            zeros = matched(run.events(), 'motor_speed', 0)
            need(any(abs(event[0] - done) <= 5 for _, event in zeros), 'Nhật ký lệch vết >5 ms')
            need('DUNG ' + reason in run.text(), 'Thiếu dòng độ trễ trên terminal: ' + reason)
        summary = json.loads((folder / 'summary.json').read_text(encoding='utf-8'))
        need(isinstance(summary.get('stops'), list) and len(summary['stops']) <= 100,
             'Summary thiếu/sai mảng stops')
        for reason in ('key_s', 'key_x', 'deadman', 'sigterm'):
            need(sum(row.get('reason') == reason for row in summary['stops']) == 1,
                 'Summary thiếu/lặp lý do dừng ' + reason)


def key_variants(n, cfg, root):
    if n in (16, 17):
        for state in ('DUNG', 'CHAY'):
            with running(cfg, root / state) as run:
                run.boot()
                if state == 'CHAY':
                    run.ready()
                    run.keep_driving()
                for seq in (b'\x1b[A', b'\x1b[B', b'\x1b[C', b'\x1b[D',
                            b'\x1bOA', b'\x1bOB', b'\x1bOC', b'\x1bOD'):
                    mark = len(run.events())
                    if n == 17:
                        run.send(seq[:1])
                        begin = time.monotonic()
                        run.observe(0.01)
                        need(time.monotonic() - begin < 0.04, 'Thiết lập escape phân mảnh quá trễ', 'setup')
                        run.send(seq[1:])
                    else:
                        run.send(seq)
                    run.observe(0.15, lambda: need(not matched(run.events()[mark:], 'servo_angle'),
                                                  'Đuôi mũi tên biến thành phím lái'))
                for truncated in (b'\x1b[', b'\x1bO', b'\xff', b'?'):
                    mark = len(run.events())
                    run.send(truncated)
                    run.observe(0.3, lambda: need(not matched(run.events()[mark:], 'servo_angle'),
                                                 'Chuỗi cụt/byte lạ thành lái'))
                if state == 'DUNG':
                    run.ready()
                    run.drive()
                else:
                    run.drive()
                # Esc đơn có cửa sổ rộng; không kiểm sát biên 50 ms.
                run.pump = False
                run.send(b'\x1b')
                run.wait(lambda: run.process.poll() is not None, 5, alive=False)
                need(matched(run.events(), 'key', 'ESC'), 'Esc đơn không được ghi nhận')
                need(run.process.returncode == 0, 'Esc đơn sai mã thoát')
                final_stop(run.events())
                run.check_output('esc', 0)
    elif n == 53:
        for lower in (False, True):
            with running(cfg, root / str(lower)) as run:
                run.boot()
                keys = 'wadcsx' if lower else 'WADCSX'
                for key in keys:
                    run.key(key)
                    if key.upper() == 'W':
                        run.event('motor_speed', 5)
                    elif key.upper() == 'A':
                        run.event('servo_angle', 5)
                    elif key.upper() == 'D':
                        run.event('servo_angle', 0)
                # X xóa hàng đợi phím; Esc gửi ngay sau X có thể bị xóa theo (như D08). Chờ X xử lý xong rồi mới thoát.
                run.pump = False
                run.observe(0.5, lambda: None)
                run.finish()


def tool_output(argv):
    result = subprocess.run(argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=25)
    need(result.returncode == 0, 'Lỗi công cụ {}: {}'.format(argv[0], result.stderr.decode(errors='replace')))
    text = result.stdout.decode(errors='replace')
    need(text.strip(), 'Công cụ trả dữ liệu rỗng: ' + argv[0])
    return text


def artifacts(n, cfg):
    binary = cfg.cam if n == 49 else cfg.real
    need(binary and Path(binary).is_file(), 'Thiếu hiện vật binary cần kiểm')
    data = Path(binary).read_bytes()
    need(data[:4] == b'\x7fELF', 'Hiện vật phải là ELF Linux')
    if n == 51:
        hooks = Path(cfg.hooks).read_bytes()
        need(data != hooks, 'Bản xe và hooks giống nhau')
        need(b'UCR_DRIVE_FAKE_' not in data, 'Hook lọt vào bản xe')
        for name in ('TRACE', 'CTRL', 'CAMERA', 'MOTOR_FAIL', 'FREE_MB', 'OPEN_FAIL', 'LOCK', 'INIT_MS'):
            need(('UCR_DRIVE_FAKE_' + name).encode('ascii') in hooks, 'Thiếu hook đối chứng ' + name)
        return
    symbols = tool_output([cfg.nm, '-C', binary])
    strings = tool_output([cfg.strings, binary])
    deps = tool_output([cfg.ldd, binary])
    need('not found' not in deps.lower(), 'Thiếu thư viện động')
    forbidden = (r'\b(?:Motor|Servo)::' if n == 49 else
                 r'\b(?:ToF|TOF|IMU|BNO055|SH1106|Backend|App|Safety)::|app_core|tof_backend')
    need(not re.search(forbidden, symbols), 'Binary chứa lớp/thư viện bị cấm')
    libraries = '\n'.join(line.strip().split()[0] for line in deps.splitlines() if line.strip())
    need(not re.search(r'(?i)bno055|vl53|sh1106|app_core|tof_backend', libraries),
         'Kéo thư viện cảm biến/app')
    if n == 49:
        need(not re.search(r'(?i)motor|servo|libi2c|uart', libraries), 'Cam kéo thư viện cơ cấu')
    if n in (50, 55):
        # §12 cho phép chuỗi Init nguyên bản; chỉ AutoTunePid vẫn bị cấm.
        for name in ('AutoTunePid',):
            need(name not in symbols and name not in strings,
                 'Có ký hiệu cấu hình driver bị cấm: ' + name)


def executable(path, source):
    path.write_text(source, encoding='utf-8')
    path.chmod(0o700)


def isolated_wrapper(cfg, folder):
    # Bản sao nguyên byte có gốc tạm: bỏ qua UCR_BUILD_DIR cũng không thể
    # rơi về build/ucr_stop thật trong cây sản phẩm.
    root = folder / 'isolated_vehicle'
    (root / 'scripts').mkdir(parents=True)
    (root / 'config.yaml').write_text('MO PHONG', encoding='utf-8')
    copied = root / 'scripts' / 'run_drive.sh'
    text = Path(cfg.wrapper).read_text(encoding='utf-8')
    # Bản xe có đúng một dòng khóa cố định; chỉ bản sao test mới đổi sang khóa riêng của ca.
    need(text.count('lock_file=/tmp/ucr_vehicle.lock') == 1, 'run_drive.sh phải có đúng một dòng lock_file=/tmp/ucr_vehicle.lock')
    copied.write_bytes(text.replace('lock_file=/tmp/ucr_vehicle.lock', 'lock_file="$UCR_DRIVE_FAKE_LOCK"').encode('utf-8'))
    return str(copied)


def wrapper_records(path):
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines(True)
            if line.endswith('\n')]


def wrapper_preflight(cfg, root):
    """§13.3: giữ khóa thật trong ca bận; chỉ chạy bản sao script và executable giả."""
    for mode in ('free', 'busy', 'pre_fail'):
        folder = root / mode
        folder.mkdir()
        build = folder / 'build with spaces'
        build.mkdir()
        config = folder / 'config with spaces.yaml'
        config.write_text('MO PHONG', encoding='utf-8')
        journal, lock = folder / 'journal', folder / 'vehicle.lock'
        fake = folder / 'fake.py'
        fake.write_text(
            'import json, os, sys\n'
            'kind = sys.argv[1]\n'
            'with open(os.environ["JOURNAL"], "a", encoding="utf-8") as f:\n'
            ' f.write(json.dumps([kind, sys.argv[2:]]) + "\\n")\n'
            'sys.exit(9 if kind == "stop" and os.environ["MODE"] == "pre_fail" else 0)\n',
            encoding='utf-8')
        prefix = '#!/bin/sh\nexec {} {} '.format(shlex.quote(sys.executable), shlex.quote(str(fake)))
        executable(build / 'ucr_stop', prefix + 'stop "$@"\n')
        executable(build / 'ucr_drive_collect', prefix + 'child "$@"\n')
        with lock.open('w', encoding='utf-8') as owner:
            if mode == 'busy':
                fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with running(cfg, folder / 'run', env=dict(UCR_BUILD_DIR=str(build),
                         UCR_DRIVE_FAKE_LOCK=str(lock), JOURNAL=str(journal), MODE=mode),
                         command=[cfg.bash, isolated_wrapper(cfg, folder), '--config', str(config)]) as run:
                run.wait(lambda: run.process.poll() is not None, 5, 'Trình bao không kết thúc', alive=False)
                records = wrapper_records(journal)
                kinds = [record[0] for record in records]
                expected = {'free': ['stop', 'child', 'stop'], 'busy': [], 'pre_fail': ['stop']}[mode]
                need(kinds == expected, 'Sai thứ tự/số lần gọi nhánh {}: {}'.format(mode, kinds))
                need(run.process.returncode == {'free': 0, 'busy': 6, 'pre_fail': 1}[mode],
                     'Sai mã nhánh ' + mode + ': ' + run.text())
                for kind, argv in records:
                    if kind == 'stop':
                        need(argv == ['--config', str(config)], 'Stop trước/sau gọi sai config')
                if mode != 'free':
                    need(run.text().strip(), 'Thiếu cảnh báo khóa hoặc dừng trước thất bại')


def wrapper_case(n, cfg, root):
    """G6: chỉ hai executable giả trong UCR_BUILD_DIR, không có đường tới ucr_stop thật."""
    variants = [('kill', 0)] if n == 56 else ([('ctrlc', 0)] if n == 57 else
               [('clean', 0), ('error', 0), ('term', 0), ('clean', 9)])
    for index, (mode, stop_status) in enumerate(variants):
        folder = root / str(index)
        folder.mkdir()
        build = folder / 'build with spaces'
        build.mkdir()
        config = folder / 'config with spaces.yaml'
        config.write_text('MO PHONG', encoding='utf-8')
        child_trace, stop_trace = folder / 'child.json', folder / 'stop.json'
        journal = folder / 'journal'
        fake = folder / 'fake.py'
        fake.write_text(
            'import json, os, signal, sys, termios, time\n'
            'from pathlib import Path\n'
            'kind = sys.argv[1]\n'
            'alive = False\n'
            'if kind == "stop" and Path(os.environ["CHILD_TRACE"]).exists():\n'
            ' try:\n'
            '  os.kill(json.loads(Path(os.environ["CHILD_TRACE"]).read_text(encoding="utf-8"))["pid"], 0)\n'
            '  alive = True\n'
            ' except ProcessLookupError: pass\n'
            'with open(os.environ["JOURNAL"], "a", encoding="utf-8") as f:\n'
            ' f.write(json.dumps([kind, sys.argv[2:], alive]) + "\\n")\n'
            'if kind == "stop":\n'
            ' Path(os.environ["STOP_TRACE"]).write_text(json.dumps(sys.argv[2:]), encoding="utf-8")\n'
            ' sys.exit(int(os.environ["STOP_STATUS"]) if Path(os.environ["CHILD_TRACE"]).exists() else 0)\n'
            'fd = sys.stdin.fileno()\n'
            'attrs = termios.tcgetattr(fd)\n'
            'attrs[3] &= ~(termios.ICANON | termios.ECHO)\n'
            'termios.tcsetattr(fd, termios.TCSANOW, attrs)\n'
            'mark = Path(os.environ["CHILD_TRACE"])\n'
            'pending = mark.with_suffix(".tmp")\n'
            'pending.write_text(json.dumps(dict(\n'
            ' pid=os.getpid(), pgid=os.getpgrp(), fg=os.tcgetpgrp(fd), tty=os.isatty(fd))), encoding="utf-8")\n'
            'pending.rename(mark)\n'
            'signal.signal(signal.SIGINT, lambda s,f: sys.exit(130))\n'
            'signal.signal(signal.SIGTERM, lambda s,f: sys.exit(143))\n'
            'while True:\n'
            ' key = os.read(fd, 1)\n'
            ' if key == b"q": sys.exit(0)\n'
            ' if key == b"e": sys.exit(7)\n', encoding='utf-8')
        prefix = '#!/bin/sh\nexec {} {} '.format(shlex.quote(sys.executable), shlex.quote(str(fake)))
        executable(build / 'ucr_drive_collect', prefix + 'drive "$@"\n')
        executable(build / 'ucr_stop', prefix + 'stop "$@"\n')
        env = dict(UCR_BUILD_DIR=str(build), CHILD_TRACE=str(child_trace), STOP_TRACE=str(stop_trace),
                   STOP_STATUS=str(stop_status), JOURNAL=str(journal))
        with running(cfg, folder / 'run', env=env,
                     command=[cfg.bash, isolated_wrapper(cfg, folder), '--config', str(config)]) as run:
            run.wait(lambda: child_trace.exists(), 5, 'Trình bao không chạy con từ UCR_BUILD_DIR')
            need([record[0] for record in wrapper_records(journal)] == ['stop', 'drive'],
                 'Cần đúng một stop trước con')
            child = json.loads(child_trace.read_text(encoding='utf-8'))
            need(child['tty'] and child['pgid'] == child['fg'], 'Con mất TTY/nhóm tiền cảnh')
            if mode == 'kill':
                os.kill(child['pid'], signal.SIGKILL)
            elif mode == 'term':
                os.kill(child['pid'], signal.SIGTERM)
            elif mode == 'ctrlc':
                run.send(b'\x03')
            else:
                run.send('e' if mode == 'error' else 'q')
            run.wait(lambda: run.process.poll() is not None, 5, 'Trình bao không kết thúc', alive=False)
            need(stop_trace.exists(), 'Không gọi ucr_stop giả')
            need([record[0] for record in wrapper_records(journal)] == ['stop', 'drive', 'stop'],
                 'Cần đúng một stop sau con, ngoài stop trước con')
            need(wrapper_records(journal)[-1][2] is False, 'Stop sau gọi khi con vẫn còn sống')
            need(all(record[1] == ['--config', str(config)] for record in wrapper_records(journal)
                     if record[0] == 'stop'), 'Một lần gọi stop dùng sai config')
            need(json.loads(stop_trace.read_text(encoding='utf-8')) == ['--config', str(config)],
                 'Gọi stop sai config')
            if stop_status or mode != 'clean':
                need(run.process.returncode != 0, 'Trình bao che lỗi con/stop')
            else:
                need(run.process.returncode == 0, 'Trình bao thoát sạch sai mã')


def wrapper_guards(n, cfg, root):
    modes = ('lock', 'open') if n == 76 else ('missing', 'not_executable', 'pre_hang', 'post_hang')
    for mode in modes:
        folder = root / mode
        folder.mkdir()
        build = folder / 'build'
        build.mkdir()
        child_mark, stop_mark = folder / 'child', folder / 'stop'
        journal = folder / 'journal'
        fake = folder / 'fake.py'
        fake.write_text(
            'import json, os, signal, sys\nfrom pathlib import Path\n'
            'kind, mode = sys.argv[1:3]\n'
            'with open(os.environ["JOURNAL"], "a", encoding="utf-8") as f:\n'
            ' f.write(json.dumps([kind]) + "\\n")\n'
            'Path(os.environ["CHILD_MARK"] if kind == "child" else os.environ["STOP_MARK"]).write_text(\n'
            ' json.dumps(dict(pid=os.getpid(), pgid=os.getpgrp())), encoding="utf-8")\n'
            'if kind == "child": sys.exit(6 if mode == "lock" else 3 if mode == "open" else 0)\n'
            'if mode == "pre_hang" or (mode == "post_hang" and Path(os.environ["CHILD_MARK"]).exists()):\n'
            ' while True: signal.pause()\n', encoding='utf-8')
        prefix = '#!/bin/sh\nexec {} {} '.format(shlex.quote(sys.executable), shlex.quote(str(fake)))
        executable(build / 'ucr_drive_collect', prefix + 'child ' + mode + '\n')
        if mode != 'missing':
            executable(build / 'ucr_stop', prefix + 'stop ' + mode + '\n')
            if mode == 'not_executable':
                (build / 'ucr_stop').chmod(0o600)
        with running(cfg, folder / 'run', env=dict(UCR_BUILD_DIR=str(build),
                     CHILD_MARK=str(child_mark), STOP_MARK=str(stop_mark), JOURNAL=str(journal)),
                     command=[cfg.bash, isolated_wrapper(cfg, folder)]) as run:
            try:
                run.wait(lambda: run.process.poll() is not None, 14 if mode.endswith('hang') else 5,
                         'Trình bao không giữ hạn stop', alive=False)
                if mode in ('missing', 'not_executable'):
                    need(run.process.returncode == 1 and not child_mark.exists(),
                         'Vẫn chạy con khi thiếu ucr_stop thực thi được')
                elif mode == 'lock':
                    need(run.process.returncode == 6 and wrapper_records(journal) == [['stop'], ['child']],
                         'Con trả 6: đã stop trước khi con tranh khóa, không được stop sau')
                    need(run.text().strip(), 'Thiếu lời nhắc tranh khóa')
                elif mode == 'pre_hang':
                    need(run.process.returncode == 1 and not child_mark.exists() and
                         wrapper_records(journal) == [['stop']], 'Stop trước treo vẫn chạy con hoặc sai mã')
                else:
                    need(child_mark.exists() and stop_mark.exists(), 'Không chạy con/stop giả')
                    need(wrapper_records(journal) == [['stop'], ['child'], ['stop']], 'Sai thứ tự stop/con/stop')
                    need(run.process.returncode != 0, 'Che lỗi mở hoặc stop treo')
            finally:
                # timeout có thể tạo nhóm riêng; dọn đúng PID/nhóm stub đã ghi.
                if stop_mark.exists():
                    record = json.loads(stop_mark.read_text(encoding='utf-8'))
                    try:
                        if os.getpgid(record['pid']) == record['pgid']:
                            if record['pgid'] not in (os.getpgrp(), run.process.pid):
                                os.killpg(record['pgid'], signal.SIGKILL)
                            else:
                                os.kill(record['pid'], signal.SIGKILL)
                    except ProcessLookupError:
                        pass


def mutant_source(case):
    if case == 'M12':
        return 'pass_store.cpp'
    return 'main.cpp' if case in ('M05', 'M06', 'M07', 'M08', 'M09', 'M13') else 'options.h'


def patch_mutant(case, folder):
    """Chỉ đọc/vá BẢN SAO để đặt neo; kỳ vọng lấy từ yêu cầu của phiếu."""
    path = folder / mutant_source(case)
    source = path.read_text(encoding='utf-8')
    if case == 'M01':
        pattern, replacement = r'\bstop\("deadman",\s*deadmanAt\(\)\);', '(void)deadmanAt();'
    elif case == 'M02':
        pattern, replacement = r'\|\|\s*o\.speed\s*>\s*5\b', ''
    elif case == 'M03':
        # X nay dùng stop(), không dùng halt(); chỉ làm sai X, giữ đường Init.
        pattern = (r'(void stop\([^\n]+\)\s*\{.*?std::exception_ptr failure;\s*)'
                   r'(try\s*\{\s*setSpeed\(0\);\s*\}\s*catch\s*\(\.\.\.\)\s*'
                   r'\{\s*failure = std::current_exception\(\);\s*\})')
        replacement = (r'\1if (reason == "key_x" && center) setAngle(0);\n        \2')
    elif case == 'M04':
        pattern, replacement = r'if \(awaitingPress_\) \{', 'if (false && awaitingPress_) {'
    elif case == 'M06':
        pattern = r'terminal->discard\(\); // Bỏ phím gõ trước và trong Init, W mới sau đó chạy ngay khi có ảnh\.'
        replacement = '// Mutation: cố ý giữ phím terminal cũ.'
    elif case == 'M07':
        pattern = (r'(terminal->discard\(\); // Bỏ phím gõ trước và trong Init, W mới sau đó chạy ngay khi có ảnh\.\n)'
                   r'        preview.discardKeys\(\);')
        replacement = r'\1        // Mutation: cố ý giữ phím preview cũ.'
    elif case == 'M05':
        # §12: gọi Init hai lần trên bản sao, giữ đường khởi động thành công.
        pattern = r'if\s*\(!motor\.init\(\)\)'
        replacement = 'if (!(motor.init() && motor.init()))'
    elif case == 'M08':
        pattern = r'const bool burst = control\.keyBurst\(keys, previewKeys, readAt\);'
        replacement = 'const bool burst = false;'
    elif case == 'M09':
        pattern = r'if \(options\.selftest\) \{\n        try \{ return selftest\(options\); \}'
        replacement = ('if (options.selftest) {\n'
                       '        DriveMotor forbidden; forbidden.init();\n'
                       '        try { return selftest(options); }')
    elif case == 'M10':
        pattern, replacement = r'recOnStart = false', 'recOnStart = true'
    elif case == 'M11':
        pattern = r'return state == State::DUNG && speed == 0 && !latched_error && !exiting &&'
        replacement = 'return true ||'
    elif case == 'M12':
        pattern, replacement = r'std::max\(\{counter, csvMax, imageMax\}\)', 'counter'
    elif case == 'M13':
        pattern = r'(?m)^            nextPass_ = store.nextPass\(\);$'
        replacement = '            nextPass_ = store.nextPass(); requireStore(store.beginPass(wallMs()));'
        need(source.count('!recording_ || ') == 2, 'Sai hai cổng REC', 'mutation_setup')
        source = source.replace('!recording_ || ', '')
    else:
        raise SafetyFailure('mutation_setup', 'Mã mutation không biết: ' + case)
    patched, count = re.subn(pattern, replacement, source, flags=re.DOTALL if case == 'M03' else 0)
    need(count == 1 and patched != source, 'Điểm vá mutation không khớp duy nhất: ' + case, 'mutation_setup')
    path.write_text(patched, encoding='utf-8')


def mutation(case, cfg, root):
    original = dict(M01='D18', M02='D07', M03='D13', M04='D96', M05='D80', M06='D94', M07='D95',
                    M08='D98', M09='D102', M10='D106', M11='D107', M12='D109', M13='D106')[case]
    expected = dict(M01='deadman', M02='speed_reject', M03='x_order', M04='fresh_press', M05='init_once',
                    M06='startup_stdin', M07='startup_preview', M08='key_burst', M09='selftest_hardware', M10='rec_default', M11='rec_moving',
                     M12='rec_counter', M13='rec_default')[case]
    # Không đếm lỗi build/khởi động/bản gốc thành bằng chứng test phát hiện lỗi.
    test_case(original, cfg, root / 'baseline')
    folder = root / 'source'
    shutil.copytree(cfg.source, str(folder), ignore=shutil.ignore_patterns(
        'build*', '__pycache__', '*.pyc', 'test_drive_collect.py', 'tests.cmake'))
    patch_mutant(case, folder)
    build = root / 'mutant-build'
    build.mkdir()
    # CMake độc lập chỉ biên dịch bản hooks, không tạo/chạy phần cứng thật.
    # main.cpp đã nối PreviewHost; bản hooks cần cả đơn vị preview để liên kết.
    (build / 'CMakeLists.txt').write_text(
        'cmake_minimum_required(VERSION 3.10)\nproject(Mutant LANGUAGES CXX)\n'
        'set(CMAKE_CXX_STANDARD 17)\nset(CMAKE_CXX_STANDARD_REQUIRED ON)\n'
        'find_package(OpenCV 4.1 REQUIRED COMPONENTS core imgcodecs videoio imgproc OPTIONAL_COMPONENTS highgui)\n'
        'find_package(Threads REQUIRED)\n'
        'add_executable(mutant "${SOURCE}/main.cpp" "${SOURCE}/preview.cpp" "${SOURCE}/pass_store.cpp")\n'
        'target_compile_definitions(mutant PRIVATE UCR_DRIVE_TEST_HOOKS=1)\n'
        'if(OpenCV_highgui_FOUND)\n'
        ' target_compile_definitions(mutant PRIVATE UCR_DRIVE_HAVE_HIGHGUI=1)\nendif()\n'
        'target_include_directories(mutant PRIVATE ${OpenCV_INCLUDE_DIRS})\n'
        'target_link_libraries(mutant PRIVATE ${OpenCV_LIBS} Threads::Threads)\n'
        'if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9)\n'
        ' target_link_libraries(mutant PRIVATE stdc++fs)\nendif()\n', encoding='utf-8')
    # Tổng hạn build 65 s + baseline/probe nằm dưới TIMEOUT=120.
    started = time.monotonic()
    for argv in ([cfg.cmake, '-DSOURCE=' + str(folder), '-DCMAKE_BUILD_TYPE=Release',
                  '-DCMAKE_CXX_FLAGS=-O0 -g0', '-DCMAKE_CXX_FLAGS_RELEASE=-DNDEBUG',
                  '-DCMAKE_CXX_COMPILER=' + cfg.cxx, '.'],
                 [cfg.cmake, '--build', '.', '--', '-j1']):
        remaining = 65 - (time.monotonic() - started)
        need(remaining > 0, 'Hết hạn build mutation', 'mutation_setup')
        process = subprocess.Popen(argv, cwd=str(build), stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        try:
            output, _ = process.communicate(timeout=remaining)
            result = subprocess.CompletedProcess(argv, process.returncode, output)
        finally:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=5)
        need(result.returncode == 0, 'Build mutation lỗi: ' + result.stdout.decode(errors='replace')[-4000:],
             'mutation_setup')
    mutant = argparse.Namespace(**vars(cfg))
    mutant.hooks = str(build / 'mutant')
    mutant.slack = 0  # Bản sao vừa biên dịch Release, không áp dung sai sanitizer.
    mutant.tsan_slack = 0
    if case == 'M02':
        with running(mutant, root / 'precondition', args=['--speed-cms', '20']) as run:
            run.boot()
            run.ready()
            run.drive()
            need(matched(run.events(), 'motor_speed', 20), 'Bản vá chưa bỏ trần tốc độ', 'mutation_setup')
    try:
        test_case(original, mutant, root / 'probe')
    except SafetyFailure as error:
        need(error.tag == expected, 'Mutation hỏng sai phép kiểm: ' + str(error), 'mutation_setup')
        print('{} PHÁT HIỆN: {}'.format(case, error))
    else:
        raise SafetyFailure('mutation_survived', case + ' không bị test phát hiện')


def self_check():
    """Kiểm bộ phân tích và cổng mutation; chạy được trên Windows, không giả runtime."""
    with tempfile.TemporaryDirectory(prefix='drive_self_') as directory:
        root = Path(directory)
        check_tsan_logs(root)  # TSan không tạo tệp nếu không có báo cáo.
        (root / 'tsan.1').write_text('ThreadSanitizer: không có cảnh báo\n', encoding='utf-8')
        check_tsan_logs(root)
        (root / 'tsan.2').write_bytes(b'\xffWARNING: ThreadSanitizer: data race\n')
        try:
            check_tsan_logs(root)
        except SafetyFailure as error:
            need(error.tag == 'tsan_warning' and 'tsan.2' in str(error), 'Sai lỗi nhật ký TSan')
        else:
            raise AssertionError('Bộ kiểm bỏ lọt cảnh báo TSan trong tệp')
        trace = root / 'trace'
        trace.write_text('10 state DUNG\n10 motor_init\n11 key  \n12 motor_speed 0 fail\n'
                         '13 motor_emergency fail\n14 servo_angle 0\n15 key', encoding='utf-8')
        events = read_trace(trace)
        need(len(events) == 6 and events[1][1] == 'motor_init' and events[2][2] == ' ' and events[3][3],
             'Bộ đọc vết sai')
        final_stop(events)
        ordered_stop(events)
        valid_init = [(1, 'state', 'DUNG', False), (2, 'motor_init', '', False),
                      (3, 'motor_speed', 0, False), (4, 'servo_angle', 0, False),
                      (5, 'key', 'R', False), (6, 'motor_speed', 5, False)]
        init_sequence(valid_init, required=True, complete=True)
        init_sequence([], required=False)
        init_sequence(valid_init[:2], required=True)  # Watchdog không buộc có lệnh dừng.
        burst = [(10, 'key', 'W', False), (10, 'motor_speed', 5, False),
                 (12, 'key', 'W', False), (310, 'motor_speed', 0, False)]
        burst_deadman(burst, 100, 0)
        for bad in (burst[:-1] + [(311, 'motor_speed', 0, False)],
                    burst + [(400, 'motor_speed', 5, False)]):
            try:
                burst_deadman(bad, 100, 0)
            except SafetyFailure:
                pass
            else:
                raise AssertionError('Bộ kiểm hàng W bỏ lọt trễ/chạy lại')
        cmake = Path(__file__).with_name('tests.cmake').read_text(encoding='utf-8')
        motion_quiet([(1, 'motor_speed', 0, False)])
        try:
            motion_quiet([(1, 'motor_speed', 5, False)])
        except SafetyFailure as error:
            need(error.tag == 'fresh_press', 'Sai mã phép kiểm nhấn mới')
        else:
            raise AssertionError('Bộ kiểm nhấn mới bỏ lọt tốc độ dương')
        startup_quiet([(1, 'motor_speed', 0, False)], 'startup_stdin')
        for events in ([(1, 'key', 'W', False)], [(1, 'servo_angle', 5, False)]):
            try:
                startup_quiet(events, 'startup_preview')
            except SafetyFailure as error:
                need(error.tag == 'startup_preview', 'Sai mã kiểm xả startup')
            else:
                raise AssertionError('Bộ kiểm xả startup bỏ lọt phím/lệnh cũ')
        registered = re.findall(r'\b[DM]\d{2,3}\b', cmake.split('set(_drive_cases', 1)[1].split(')', 1)[0])
        need(len(registered) == len(set(registered)) and set(registered) == set(CASES),
             'Danh sách CLI/CMake lệch hoặc trùng ca')
        for bad, tag, checker in (
                ([(1, 'servo_angle', 0, False), (1, 'motor_speed', 0, False)], 'x_order',
                 lambda es: ordered_stop(es, 'x_order')),
                ([(1, 'motor_speed', 6, False)], 'speed_limit', limits),
                ([(1, 'motor_speed', -1, False)], 'speed_limit', limits),
                ([(1, 'motor_speed', 5, False)], 'startup_stopped', lambda es: limits(es, False)),
                (valid_init + [(7, 'motor_init', '', False)], 'init_once', init_sequence),
                (valid_init[:1] + valid_init[2:], 'init_once', init_sequence),
                (valid_init[:1] + [valid_init[2]] + valid_init[1:2], 'init_order', init_sequence),
                (valid_init[:2] + valid_init[3:], 'init_order', init_sequence),
                (valid_init[:3] + valid_init[4:], 'init_order', init_sequence),
                (valid_init[:2] + [(3, 'motor_speed', 0, True)] + valid_init[3:],
                 'init_order', init_sequence),
                (valid_init[:2] + [valid_init[4]], 'init_order', init_sequence),
                (valid_init[:2], 'init_order', lambda es: init_sequence(es, complete=True))):
            try:
                checker(bad)
            except SafetyFailure as error:
                need(error.tag == tag, 'Sai mã phép kiểm tự kiểm')
            else:
                raise AssertionError('Bộ kiểm bỏ lọt vết lỗi')
        data = b'\xff\xd8\xff\xc0\x00\x0b\x08\x02\xd0\x05\x00\x01\x01\x11\x00\xff\xd9'
        need(jpeg_info(data)[0] == (1280, 720), 'Đọc cỡ JPEG sai')
        fixtures = {
            'M01': 'if (state == State::CHAY && lastW_ == received) return;\n'
                   'if (state == State::CHAY && now >= deadmanAt()) { stop("deadman", deadmanAt()); }',
            'M02': 'if (!finite || o.speed < 0 || o.speed > 5) throw error;',
            'M03': 'void stop(const std::string& reason, int64_t trigger, bool center = false) {\n'
                   ' std::exception_ptr failure;\n try { setSpeed(0); }\n'
                   ' catch (...) { failure = std::current_exception(); }\n}',
            'M04': 'if (awaitingPress_) { reject(); }',
            'M06': 'terminal->discard(); // Bỏ phím gõ trước và trong Init, W mới sau đó chạy ngay khi có ảnh.',
            'M07': 'terminal->discard(); // Bỏ phím gõ trước và trong Init, W mới sau đó chạy ngay khi có ảnh.\n        preview.discardKeys();',
            'M05': 'if (!motor.init()) throw Failure(3, "motor_init_fail", "Init hỏng");',
            'M08': 'const bool burst = control.keyBurst(keys, previewKeys, readAt);',
            'M09': 'if (options.selftest) {\n        try { return selftest(options); }'}
        for case, source in fixtures.items():
            folder = root / case
            folder.mkdir()
            path = folder / mutant_source(case)
            path.write_text(source, encoding='utf-8')
            patch_mutant(case, folder)
            need(path.read_text(encoding='utf-8') != source, 'Mutation không đổi bản sao')
            # Kiểm neo duy nhất trên mã hiện tại, chỉ vá bản sao tạm.
            path.write_bytes((Path(__file__).resolve().parent / path.name).read_bytes())
            patch_mutant(case, folder)
        for case in ('M10', 'M11', 'M12', 'M13'):
            folder = root / case
            folder.mkdir()
            path = folder / mutant_source(case)
            path.write_bytes((Path(__file__).resolve().parent / path.name).read_bytes())
            patch_mutant(case, folder)
        no_selftest_hardware([])
        for event in ('motor_init', 'motor_speed', 'motor_emergency', 'servo_angle'):
            try:
                no_selftest_hardware([(0, event, 0, False)])
            except SafetyFailure as error:
                need(error.tag == 'selftest_hardware', 'Sai mã phép kiểm tự kiểm không động cơ')
            else:
                raise AssertionError('Bộ kiểm tự kiểm bỏ lọt ' + event)
    print('ĐẠT self-check: nhật ký TSan, vết/Init, hàng W, đăng ký ca, JPEG, không động cơ và 13 neo hiện tại; chưa chạy binary sản phẩm')


def main():
    # Python 3.6 chốt mã hóa đường dẫn khi khởi động; cần UTF-8 cho CLI tiếng Việt.
    if os.name == 'posix' and sys.getfilesystemencoding().lower().replace('-', '') != 'utf8':
        need(os.environ.get('UCR_DRIVE_TEST_UTF8') != '1', 'Không bật được UTF-8')
        os.environ.update(LC_ALL='C.UTF-8', LANG='C.UTF-8', UCR_DRIVE_TEST_UTF8='1')
        os.execv(sys.executable, [sys.executable] + sys.argv)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=CASES)
    parser.add_argument('--self-check', action='store_true')
    parser.add_argument('--hooks')
    parser.add_argument('--real')
    parser.add_argument('--cam')
    parser.add_argument('--source', default=str(Path(__file__).resolve().parent))
    parser.add_argument('--wrapper', default=str(Path(__file__).resolve().parents[2] / 'scripts' / 'run_drive.sh'))
    parser.add_argument('--slack', type=int, choices=(0, 300), default=0)
    # Chỉ chờ thoát/cleanup; không cộng vào stop_by, deadman hoặc cặp dừng sau Init.
    parser.add_argument('--tsan-slack', type=int, choices=(0, 1000, 2500), default=0)
    parser.add_argument('--nm', default='nm')
    parser.add_argument('--strings', default='strings')
    parser.add_argument('--ldd', default='ldd')
    parser.add_argument('--bash', default='bash')
    parser.add_argument('--cmake', default='cmake')
    parser.add_argument('--cxx', default='g++')
    cfg = parser.parse_args()
    if cfg.self_check:
        self_check()
        return
    need(os.name == 'posix', 'Ca PTY cần Linux/POSIX; không kết luận đạt trên Windows')
    need(cfg.case and cfg.hooks and cfg.real, 'Thiếu --case/--hooks/--real')
    for key in ('hooks', 'real', 'cam', 'source', 'wrapper'):
        if getattr(cfg, key):
            setattr(cfg, key, str(Path(getattr(cfg, key)).resolve()))
    need(Path(cfg.hooks).is_file(), 'Thiếu binary hooks')
    need(b'UCR_DRIVE_FAKE_TRACE' in Path(cfg.hooks).read_bytes(), 'Từ chối chạy binary thiếu hook đối chứng')
    print('{}: dung sai sanitizer +{} ms, khung đầu/cleanup TSan +{} ms'.format(
        cfg.case, cfg.slack, cfg.tsan_slack))
    with tempfile.TemporaryDirectory(prefix='ucr_drive_' + cfg.case + '_') as directory:
        test_case(cfg.case, cfg, Path(directory))
    print(cfg.case + ' ĐẠT')


if __name__ == '__main__':
    main()
