"""T33: kỳ vọng chỉ từ phiếu/hợp đồng; Python 3.6, chỉ chạy binary hooks.

Không nhập test cũ: thay đổi song song của DEV không được đổi kỳ vọng T33.
M1–M5 chỉ đọc/vá BẢN SAO nguồn khi chạy Linux, không sửa cây làm việc.
CTRL và lệnh lỗi kế thừa mẫu test của công cụ thu cũ (đã xóa), không thêm biến giả mới.
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
import shutil
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time

# Python 3.6 trong container có thể dùng locale ASCII; thông báo lỗi tiếng Việt vẫn phải in được.
for _name in ('stdout', 'stderr'):
    _stream = getattr(sys, _name)
    if hasattr(_stream, 'buffer'):
        setattr(sys, _name, io.TextIOWrapper(_stream.detach(), encoding='utf-8',
                                           errors='replace', line_buffering=True))

if os.name == 'posix':
    import fcntl

# Như công cụ thu cũ: race giả trong OpenCV/TBB chưa instrument.
TSAN_SUPP = Path(__file__).resolve().parent.parent / 'collect_common' / 'tsan.supp'


HEADERS = {
    'frames.csv': ['frame', 'monotonic_ns', 'wall_time_unix_ms', 'filename',
                   'width', 'height', 'mean_luma', 'lap_var', 'dropped_since_last',
                   'cmd_speed', 'cmd_steer', 'state', 'pass', 'file'],
    'passes.csv': ['pass', 'event', 'ts_ms', 'frames', 'end_reason'],
    'sessions.csv': ['ts_ms', 'git', 'argv', 'label', 'pipeline'],
}
CASES = ('K01 K02 K03 K04 K05 S01 E01 E02 E02b E03 E03b E04 E05 E05b E06 E06b E07 E09 E10 '
         'P01 P02 P03 F01 F02 F03 F03b F04 M1 M1b M2 M2s M3 M4 M5 SC').split()
FORBIDDEN = r'(?i)\b(?:ToF|IMU|OLED)(?:::|\b)|VL53|BNO055|SH1106'


class Failure(AssertionError):
    def __init__(self, tag, message):
        self.tag = tag
        super().__init__('{}: {}'.format(tag, message))


def need(ok, message, tag='contract'):
    if not ok:
        raise Failure(tag, message)


def now_ms():
    return time.monotonic() * 1000


def trace(path):
    if not path.exists():
        return []
    events = []
    for line in path.read_text(encoding='utf-8').splitlines(True):
        if not line.endswith('\n'):
            continue
        fields = line.strip('\r\n').split(' ', 2)
        need(len(fields) >= 2, 'Vết thiếu trường', 'interface')
        stamp, kind = float(fields[0]), fields[1]
        value = fields[2] if len(fields) == 3 else ''
        failed = value == 'fail' or value.endswith(' fail')
        if failed:
            value = value[:-5] if value != 'fail' else ''
        if kind in ('motor_speed', 'servo_angle'):
            value = float(value)
            need(math.isfinite(value), 'Lệnh NaN/Inf')
        need(math.isfinite(stamp), 'Mốc vết NaN/Inf', 'interface')
        events.append((stamp, kind, value, failed))
    need(all(a[0] <= b[0] for a, b in zip(events, events[1:])), 'Vết đảo thời gian', 'interface')
    return events


def last_commands(events):
    speed = angle = None
    for _, kind, value, failed in events:
        if failed:
            continue
        if kind in ('motor_speed', 'motor_emergency'):
            speed = 0 if kind == 'motor_emergency' else value
        elif kind == 'servo_angle':
            angle = value
    return speed, angle


def safe_range(events):
    for _, kind, value, failed in events:
        if kind == 'motor_speed':
            need(abs(value) <= 40, 'Lệnh tốc độ vượt ±40: ' + str(value), 'key_speed')
            need(value != 0 or math.copysign(1, value) > 0,
                 'Motor nhận -0.0', 'positive_zero')
        elif kind == 'servo_angle':
            need(abs(value) <= 40, 'Lệnh góc vượt ±40: ' + str(value), 'key_steer')


def stop_pair(events, tag='stop_order'):
    hardware = [e for e in events if e[1] in ('motor_speed', 'motor_emergency', 'servo_angle')]
    need(hardware, 'Không có lệnh dừng sau tác nhân', tag)
    motors = [i for i, e in enumerate(hardware)
              if not e[3] and (e[1] == 'motor_emergency' or (e[1] == 'motor_speed' and e[2] == 0))]
    servos = [i for i, e in enumerate(hardware) if e[1] == 'servo_angle' and e[2] == 0 and not e[3]]
    need(motors and servos, 'Thiếu motor 0/servo 0 sau tác nhân', tag)
    need(motors[0] < servos[0], 'Servo 0 trước motor 0', tag)
    need(last_commands(hardware) == (0, 0), 'Lệnh cuối không phải 0/0', tag)
    for _, kind, value, failed in hardware[motors[0]:]:
        need(kind == 'motor_emergency' or value == 0, 'Chạy/rẽ lại sau dừng', tag)
    return hardware[motors[0]][0]


def table(out, name, live=True):
    path = out / name
    if not path.exists():
        return []
    lines = path.read_text(encoding='utf-8').splitlines(True)
    if live:
        lines = [line for line in lines if line.endswith('\n')]
    if not lines:
        return []
    reader = csv.DictReader(lines)
    need(reader.fieldnames == HEADERS[name], 'Sai header ' + name, 'record_format')
    rows = list(reader)
    need(all(None not in row and None not in row.values() for row in rows),
         'CSV thiếu/thừa cột: ' + name, 'record_format')
    return rows


def tool(argv, root, timeout=20):
    """Giới hạn cả nhóm build/process, không để con chạy sau timeout."""
    with tempfile.TemporaryFile() as log:
        process = subprocess.Popen(argv, cwd=str(root), stdout=log, stderr=log,
                                   stdin=subprocess.DEVNULL, start_new_session=True)
        try:
            process.wait(timeout=timeout)
            log.seek(0)
            return process.returncode, log.read().decode('utf-8', errors='replace')
        finally:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=3)


class Run:
    def __init__(self, cfg, root, args=(), env=None, lock=None, default=False, config=True):
        self.root = root
        self.slack = getattr(cfg, 'slack', 0)
        # An toàn: không bao giờ chạy binary thiếu điểm cắm giả (có thể là bản xe thật).
        need(b'UCR_DRIVE_FAKE_TRACE' in Path(cfg.hooks).read_bytes(),
             'Từ chối chạy binary không có hook giả: ' + str(cfg.hooks), 'environment')
        root.mkdir(parents=True)
        self.project = root / 'project'
        bindir = self.project / 'bin'
        bindir.mkdir(parents=True)
        if config:
            (self.project / 'config.yaml').write_text('{}\n', encoding='utf-8')
        # Chỉ sao binary hooks; helper cạnh binary là giả, không chạm thiết bị.
        self.binary = bindir / 'ucr_collect_drive_hooks'
        shutil.copy2(cfg.hooks, str(self.binary))
        self.path = root / 'trace'
        self.keys = root / 'keys'
        self.ctrl = root / 'control'
        self.keys.touch()
        self.ctrl.touch()
        helper = bindir / 'ucr_stop'
        helper.write_text('#!' + sys.executable + '\n'
                          'import os,time\n'
                          'with open(os.environ["UCR_DRIVE_FAKE_TRACE"],"a") as f:\n'
                          ' f.write("%d test_ucr_stop\\n" % int(time.monotonic()*1000))\n',
                          encoding='utf-8')
        helper.chmod(0o755)
        self.out = self.project / 'data_raw' if default else root / 'out'
        child_env = {k: v for k, v in os.environ.items()
                     if not k.startswith(('UCR_DRIVE_FAKE_', 'UCR_CAM_TEST_'))
                     and k not in ('DISPLAY', 'WAYLAND_DISPLAY')}
        child_env.update(LC_ALL='C.UTF-8', LANG='C.UTF-8',
                         UCR_DRIVE_FAKE_TRACE=str(self.path),
                         UCR_DRIVE_FAKE_PREVIEW='stream:' + str(self.keys),
                         UCR_DRIVE_FAKE_CAMERA='ok', UCR_DRIVE_FAKE_CTRL=str(self.ctrl),
                         UCR_DRIVE_FAKE_FREE_MB='4096',
                         UCR_DRIVE_FAKE_LOCK=str(lock or root / 'vehicle.lock'),
                         # Như mẫu preview: DISPLAY giả để lỗi (nếu có) do con giả, không do thiếu X.
                         DISPLAY=':ucr_fake_only')
        child_env['TSAN_OPTIONS'] = ':'.join(filter(None, (child_env.get('TSAN_OPTIONS', ''),
                                                           'suppressions=' + str(TSAN_SUPP))))
        child_env.update(env or {})
        argv = [str(self.binary), '--label', 'test']
        if not default:
            argv += ['--out-root', str(self.out)]
        argv += list(args)
        self.log = tempfile.TemporaryFile()
        self.process = subprocess.Popen(argv, cwd=str(root), env=child_env,
                                        stdin=subprocess.DEVNULL, stdout=self.log,
                                        stderr=self.log, start_new_session=True)

    def events(self):
        return trace(self.path)

    def text(self):
        return os.pread(self.log.fileno(), os.fstat(self.log.fileno()).st_size, 0).decode(
            'utf-8', errors='replace')

    def wait(self, predicate, seconds=5, message='Hết hạn', alive=True, tag='interface'):
        deadline = time.monotonic() + seconds
        while True:
            result = predicate()
            if result:
                return result
            need(not alive or self.process.poll() is None,
                 message + ': tiến trình thoát; ' + self.text()[-2000:], tag)
            left = deadline - time.monotonic()
            need(left > 0, message + '; ' + self.text()[-2000:], tag)
            time.sleep(min(0.01, left))

    def observe(self, seconds, check=lambda: None):
        deadline = time.monotonic() + seconds
        while True:
            need(self.process.poll() is None, 'Thoát trong lúc quan sát: ' + self.text())
            check()
            left = deadline - time.monotonic()
            if left <= 0:
                return
            time.sleep(min(0.01, left))

    def boot(self):
        self.wait(lambda: last_commands(self.events()) == (0, 0), 10, 'Thiếu dừng khởi động')
        self.wait(lambda: (self.out / 'sessions.csv').exists(), 10, 'Chưa có phiên ghi')
        self.wait(lambda: self.children(), 5, 'Không có cửa sổ giả')
        self.wait(lambda: any(e[1] == 'ready' for e in self.events()), 10, 'Chưa sẵn sàng nhận phím (ảnh đầu, cửa sổ, luồng ghi)')
        # Khởi động: motor 0 TRƯỚC servo 0, chưa có lệnh tiến/lái nào (phiếu: tốc độ/góc khởi đầu 0).
        stop_pair(self.events(), 'startup_stop')
        safe_range(self.events())

    def children(self):
        children = set()
        for path in Path('/proc/{}/task'.format(self.process.pid)).glob('*/children'):
            try:
                children.update(int(v) for v in path.read_text().split())
            except FileNotFoundError:
                pass
        return children

    def send(self, key):
        with self.keys.open('ab') as stream:
            stream.write(key.encode('ascii'))

    def ack(self, key, mark):
        aliases = {'E': ('E', 'ESC', '\x1b'), '3': ('3', '\x03', 'CTRL+C'),
                   ' ': (' ', 'SPACE')}.get(key, (key.upper(),))
        return self.wait(lambda: next((e for e in self.events()[mark:]
                                      if e[1] == 'key' and str(e[2]).upper() in aliases), None),
                         message='Thiếu vết phím ' + repr(key), alive=key not in ('E', '3'))

    def act(self, key, speed, angle, tag='key_table', latency=False):
        """Gửi một phím, đợi lệnh chốt; latency=True trả độ trễ vết phím → lệnh motor (ms)."""
        mark = len(self.events())
        self.send(key)
        event = self.ack(key, mark)
        self.wait(lambda: last_commands(self.events()) == (speed, angle),
                  1, 'Sai lệnh sau ' + repr(key), tag=tag)
        events = self.events()
        safe_range(events)
        if latency:
            got = [e for e in events[mark:] if e[1] == 'motor_speed' and e[2] == speed and not e[3]]
            need(got, 'Thiếu lệnh motor sau ' + repr(key), tag)
            return got[0][0] - event[0]
        return mark

    def inject(self, command):
        stamp = now_ms()
        with self.ctrl.open('a', encoding='utf-8') as stream:
            stream.write(command + '\n')
        return stamp

    def rows(self):
        return table(self.out, 'frames.csv')

    def finish(self, code=0, key='E', mark=None, tag='stop_order'):
        if mark is None:
            mark = len(self.events())
            self.send(key)
        self.wait(lambda: self.process.poll() is not None, 12, 'Không thoát', alive=False)
        need(self.process.returncode == code,
             'Mã thoát {} cần {}: {}'.format(self.process.returncode, code, self.text()), 'exit_code')
        events = self.events()
        stop_pair(events[mark:], tag)
        safe_range(events)
        return events

    def close(self):
        try:
            os.killpg(self.process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        self.process.wait(timeout=3)
        self.log.close()


@contextmanager
def running(cfg, root, **kwargs):
    run = Run(cfg, root, **kwargs)
    try:
        yield run
    except BaseException:
        print('LOG: ' + run.text()[-3000:], file=sys.stderr)
        print('VẾT: ' + repr(run.events()[-20:]), file=sys.stderr)
        raise
    finally:
        run.close()


def moving(run, reverse=False):
    run.boot()
    run.act('W', -5 if reverse else 5, 0, tag='motor_sign')
    run.act('A', -5 if reverse else 5, 5)


def off_snapshot(run):
    return (run.rows(), table(run.out, 'passes.csv'),
            {str(p.relative_to(run.out)): p.read_bytes() for p in run.out.rglob('*.jpg')})


def recording(run, count):
    run.send('P')
    run.wait(lambda: len([r for r in table(run.out, 'passes.csv') if r['event'] == 'start']) == count,
             message='P không mở lượt', tag='record_toggle')


def recording_off(run, count):
    run.send('P')
    run.wait(lambda: len([r for r in table(run.out, 'passes.csv') if r['event'] == 'end']) == count,
             message='P không đóng lượt', tag='record_toggle')
    snapshot = off_snapshot(run)
    run.observe(1.2, lambda: need(off_snapshot(run) == snapshot,
                                'Vẫn ghi khi P tắt', 'record_off'))


def inspect_data(run):
    rows = table(run.out, 'frames.csv', False)
    passes = table(run.out, 'passes.csv', False)
    sessions = table(run.out, 'sessions.csv', False)
    need(rows and len(sessions) == 1, 'Thiếu frames/phiên', 'record_format')
    need(sessions[0]['git'] and sessions[0]['label'] == 'test', 'Thiếu git/nhãn CSV', 'record_format')
    need([(int(r['pass']), r['event']) for r in passes] ==
         [(1, 'start'), (1, 'end'), (2, 'start'), (2, 'end')], 'Sai lượt 1/2', 'record_counter')
    names, previous = set(), -1
    for index, row in enumerate(rows, 1):
        name = row['filename']
        need(int(row['frame']) == index and int(row['monotonic_ns']) > previous,
             'Số khung/mốc không tăng', 'record_format')
        previous = int(row['monotonic_ns'])
        need(Path(name).name == name and name not in names and row['file'] == name,
             'Tên file sai/lặp', 'record_format')
        names.add(name)
        need(re.fullmatch(r'p[0-9]{4,}_[0-9]{4,}\.jpg', name), 'Sai tên ảnh', 'record_format')
        need(int(name[1:].split('_')[0]) == int(row['pass']), 'Tên ảnh lệch lượt', 'record_counter')
        data = (run.out / name).read_bytes()
        need(data[:2] == b'\xff\xd8' and data[-2:] == b'\xff\xd9', 'JPEG dở dang', 'record_format')
        marker = data.find(b'\xff\xc0')
        need(marker >= 0, 'Thiếu cỡ JPEG', 'record_format')
        height, width = struct.unpack('>HH', data[marker + 5:marker + 9])
        need((width, height) == (int(row['width']), int(row['height'])) == (1280, 720),
             'Ép sai cỡ camera giả mặc định', 'record_format')
        need(float(row['cmd_speed']) == 5 and float(row['cmd_steer']) == 5,
             'Metadata lệch lệnh chốt W/A', 'record_format')
    need(names == {p.name for p in run.out.glob('*.jpg')}, 'Ảnh ngoài CSV', 'record_format')
    for end in (r for r in passes if r['event'] == 'end'):
        need(int(end['frames']) == sum(r['pass'] == end['pass'] for r in rows),
             'Bộ đếm ảnh lượt sai', 'record_counter')
    need(not list(run.out.rglob('*.tmp')), 'Còn ảnh tạm', 'record_format')


def symbols(cfg, root):
    # Không chạy binary thật. nm rỗng/lỗi là lỗi bằng chứng, không phải đạt.
    code, output = tool([cfg.nm, '-C', cfg.real], root)
    need(code == 0 and output.strip() and 'no symbols' not in output.lower(),
         'nm thiếu bằng chứng: ' + output, 'symbols_setup')
    need(not re.search(FORBIDDEN, output), 'Có ToF/IMU/OLED trong binary', 'no_tof')
    code, output = tool(['ldd', cfg.real], root)
    need(code == 0 and output.strip() and 'not found' not in output.lower(),
         'ldd thiếu bằng chứng: ' + output, 'symbols_setup')
    need(not re.search(FORBIDDEN, output), 'Liên kết cảm biến bị cấm', 'no_tof')


def nice(pid):
    """Trường nice (thứ 19) của /proc/<pid>/stat; tên lệnh có thể chứa dấu cách nên tách sau ')'."""
    try:
        return int(Path('/proc/{}/stat'.format(pid)).read_text().rsplit(')', 1)[1].split()[16])
    except (FileNotFoundError, IndexError, ValueError):
        return None


def stop_within(run, mark, stamp, allowance, tag):
    """Lệnh motor 0/emergency đầu tiên sau mark phải tới trước stamp + allowance + slack sanitizer."""
    deadline = stamp + allowance + run.slack

    def first_stop():
        return next((e for e in run.events()[mark:] if not e[3] and
                     (e[1] == 'motor_emergency' or (e[1] == 'motor_speed' and e[2] == 0))), None)

    stop = run.wait(first_stop, max(0.05, (deadline - now_ms()) / 1000) + 0.05,
                    'Không dừng trước hạn', alive=False, tag=tag)
    need(stop[0] <= deadline, 'Dừng trễ {:.0f} ms (hạn {} ms)'.format(stop[0] - stamp, allowance + run.slack), tag)
    return stop


def closed_data(run, tag='record_close'):
    """E10: thoát lúc P bật → lượt được đóng (dòng end), không ảnh tạm/dở, bộ đếm khớp frames.csv."""
    rows = table(run.out, 'frames.csv', False)
    passes = table(run.out, 'passes.csv', False)
    need(rows, 'Không có ảnh trước khi thoát', 'setup')
    starts = [r for r in passes if r['event'] == 'start']
    ends = [r for r in passes if r['event'] == 'end']
    need(len(starts) == 1 and len(ends) == 1 and ends[0]['pass'] == starts[0]['pass'],
         'passes.csv thiếu dòng end khi thoát lúc P bật: ' + repr(passes), tag)
    need(int(ends[0]['frames']) == sum(r['pass'] == ends[0]['pass'] for r in rows),
         'Bộ đếm frames của lượt lệch frames.csv', tag)
    names = [r['filename'] for r in rows]
    need(set(names) == {p.name for p in run.out.glob('*.jpg')} and len(set(names)) == len(names),
         'JPEG không khớp từng dòng frames.csv', tag)
    for name in names:
        data = (run.out / name).read_bytes()
        need(data[:2] == b'\xff\xd8' and data[-2:] == b'\xff\xd9', 'JPEG dở dang: ' + name, tag)
    need(not list(run.out.rglob('*.tmp')), 'Còn tệp .tmp sau thoát', tag)


def failed_motor_stop(events):
    """E07: motor giả lỗi (vết đuôi 'fail', như mẫu công cụ thu cũ) — chấp nhận lệnh dừng bị lỗi,
    nhưng phải CÓ thử dừng, không có lệnh tiến, servo cuối (nếu có) là 0 và sau lần thử dừng đầu."""
    need(not any(e[1] == 'motor_speed' and e[2] != 0 for e in events), 'Lệnh tiến khi motor hỏng', 'motor_fail')
    attempts = [i for i, e in enumerate(events)
                if e[1] == 'motor_emergency' or (e[1] == 'motor_speed' and e[2] == 0)]
    need(attempts, 'Không thử dừng motor khi motor hỏng', 'motor_fail')
    servos = [i for i, e in enumerate(events) if e[1] == 'servo_angle']
    if servos:
        need(events[servos[-1]][2] == 0 and servos[-1] > attempts[0], 'Servo cuối khác 0/trước motor', 'motor_fail')


def no_forward(events, tag):
    need(not any(e[1] == 'motor_speed' and e[2] != 0 for e in events), 'Có lệnh tiến', tag)
    if any(e[1] in ('motor_speed', 'motor_emergency', 'servo_angle') and not e[3] for e in events):
        stop_pair(events, tag)


def ucr_stop_order(events):
    """Phiếu/HĐ §1: ucr_stop (giả cạnh binary) trước khi dùng motor và sau servo 0 cuối."""
    helpers = [i for i, e in enumerate(events) if e[1] == 'test_ucr_stop']
    motors = [i for i, e in enumerate(events) if e[1].startswith('motor_')]
    servos = [i for i, e in enumerate(events) if e[1] == 'servo_angle']
    need(len(helpers) == 2 and motors and servos and helpers[0] < motors[0] and helpers[1] > servos[-1],
         'Thiếu ucr_stop trước/sau hoặc sai thứ tự', 'stop_helper')


def hw_probe(cfg, root, kind):
    """M2/M2s: với trần phím đã nới lên 100, W/A ×13 chỉ được chạm đúng 60, không vượt (kẹp phần cứng)."""
    key, index = ('W', 0) if kind == 'speed' else ('A', 1)
    event_kind = 'motor_speed' if kind == 'speed' else 'servo_angle'
    tag = 'hw_' + kind
    with running(cfg, root) as run:
        run.boot()
        for i in range(1, 14):
            expected = min(5 * i, 60)
            mark = len(run.events())
            run.send(key)
            run.ack(key, mark)
            run.wait(lambda: last_commands(run.events())[index] == expected, 1,
                     'Sai lệnh sau {} lần {}'.format(key, i), tag=tag)
            values = [e[2] for e in run.events() if e[1] == event_kind and not e[3]]
            need(all(abs(v) <= 60 for v in values), 'Lệnh vượt phần cứng 60: ' + repr(max(values)), tag)
        need(max(abs(v) for v in values) == 60, 'Đối chứng không chạm đúng 60', tag)
        mark = len(run.events())
        run.send('E')
        run.wait(lambda: run.process.poll() is not None, 12, 'Không thoát', alive=False)
        stop_pair(run.events()[mark:], 'exit_stop')


def test_case(case, cfg, root):
    """Mỗi nhánh tương ứng đúng hàng cùng mã trong T33_cases.md."""
    if case == 'F03':
        root.mkdir(parents=True)
        return symbols(cfg, root)
    if case == 'F03b':
        data = Path(cfg.real).read_bytes()
        need(data[:4] == b'\x7fELF', 'Bản xe không phải ELF', 'symbols_setup')
        need(b'UCR_DRIVE_FAKE_' not in data, 'Bản xe chứa điểm cắm giả UCR_DRIVE_FAKE_', 'no_hooks_in_car')
        return
    if case in ('K01', 'K02', 'K03', 'K04', 'K05'):
        with running(cfg, root) as run:
            run.boot()
            if case == 'K01':
                speed = 0
                for key, delta in [('W', 5)] * 12 + [('S', -5)] * 24 + [('W', 5)] * 24:
                    speed = max(-40, min(40, speed + delta))
                    run.act(key, speed, 0, tag='key_speed')
                run.observe(1.2, lambda: need(last_commands(run.events()) == (40, 0),
                                            'Có deadman: lệnh không chốt', 'key_latch'))
            elif case == 'K02':
                # Mỗi bước qua/tới 0 lặp ba lần, lấy min ≤ 250 ms: nghỉ 300 ms bắt buộc sẽ luôn vượt.
                latencies = {}
                run.act('W', 5, 0)
                for _ in range(3):
                    for label, key, speed in (('5->0', 'S', 0), ('0->-5', 'S', -5),
                                              ('-5->0', 'W', 0), ('0->5', 'W', 5)):
                        latencies.setdefault(label, []).append(
                            run.act(key, speed, 0, tag='zero_cross', latency=True))
                for label, values in sorted(latencies.items()):
                    need(min(values) <= 250, 'Có nghỉ khi {} (min {:.0f} ms)'.format(label, min(values)),
                         'zero_cross')
            elif case == 'K03':
                run.act('W', 5, 0)
                angle = 0
                for key, delta in [('A', 5)] * 12 + [('D', -5)] * 24:
                    angle = max(-40, min(40, angle + delta))
                    run.act(key, 5, angle, tag='key_steer')
                run.act('C', 5, 0, tag='key_steer')
            elif case == 'K04':
                for key in (' ', 'X'):
                    run.act('W', 5, 0)
                    run.act('A', 5, 5)
                    mark = run.act(key, 0, 0)
                    # Leader chốt: motor 0 TRƯỚC servo 0 cả với Space/X.
                    stop_pair(run.events()[mark:], 'space_x')
                    run.observe(0.8, lambda: need(last_commands(run.events()) == (0, 0), 'Chạy lại sau X/Space'))
            else:
                # Không chặn lô phím (phiếu): 20 W dồn một lần → cuối 40, không lệnh nào vượt 40.
                run.send('W' * 20)
                run.wait(lambda: last_commands(run.events()) == (40, 0), 3, 'Lô 20 W không chốt 40',
                         tag='key_burst_cap')
                run.observe(0.5, lambda: safe_range(run.events()))
                mark = len(run.events())
                run.send('QT')
                run.send('A')  # hàng rào: Q/T đã được đọc trước A
                run.ack('A', mark)
                run.wait(lambda: last_commands(run.events()) == (40, 5), 1, 'Sai lệnh sau Q/T/A',
                         tag='unknown_key')
                run.observe(0.5, lambda: None)
                need(all(e[2] == 40 for e in run.events()[mark:] if e[1] == 'motor_speed') and
                     all(e[2] == 5 for e in run.events()[mark:] if e[1] == 'servo_angle') and
                     not any(e[1] == 'motor_emergency' for e in run.events()[mark:]),
                     'Phím lạ Q/T đổi lệnh', 'unknown_key')
            run.finish()
        return
    if case == 'S01':
        for reverse in (False, True):
            with running(cfg, root / str(reverse), args=['--reverse-motor'] if reverse else []) as run:
                run.boot()
                factor = -1 if reverse else 1
                for key, logical in [('W', 5), ('S', 0), ('S', -5), ('W', 0)]:
                    run.act(key, logical * factor, 0, tag='motor_sign')
                run.act('W', 5 * factor, 0, tag='motor_sign')
                run.act('A', 5 * factor, 5, tag='motor_sign')
                run.act('D', 5 * factor, 0, tag='motor_sign')
                mark = run.act('X', 0, 0, tag='motor_sign')
                stop_pair(run.events()[mark:], 'motor_sign')
                run.finish()
        return
    if case in ('E01', 'E02'):
        # E01 gộp ca ucr_stop trước/sau (E08 cũ) vì cùng kịch bản thoát bằng phím.
        triggers = ['E', '3'] if case == 'E01' else [signal.SIGINT, signal.SIGTERM, signal.SIGHUP]
        for trigger in triggers:
            with running(cfg, root / str(trigger), args=['--reverse-motor']) as run:
                moving(run, True)
                mark = len(run.events())
                if isinstance(trigger, int):
                    run.process.send_signal(trigger)
                else:
                    run.send(trigger)
                events = run.finish(mark=mark, tag='exit_stop')
                if case == 'E01':
                    ucr_stop_order(events)
        return
    if case == 'E02b':
        # SIGTERM khi ghi JPEG đang treo (hook jpeg_hang của mẫu drive): dừng ≤ 1 s, thoát 0.
        with running(cfg, root, args=['--interval-ms', '100']) as run:
            moving(run)
            recording(run, 1)
            run.wait(lambda: len(run.rows()) >= 2, message='Chưa có ảnh trước khi treo')
            mark = len(run.events())
            run.inject('jpeg_hang')
            run.act('C', 5, 0)  # hàng rào: vòng điều khiển đã quay sau lệnh CTRL
            run.act('A', 5, 5)
            run.observe(0.3, lambda: need(last_commands(run.events()) == (5, 5), 'Dừng trước tín hiệu', 'setup'))
            mark = len(run.events())
            stamp = now_ms()
            run.process.send_signal(signal.SIGTERM)
            stop_within(run, mark, stamp, 1000, 'hang_signal')
            run.finish(code=0, mark=mark, tag='hang_signal')
        return
    if case in ('E03', 'E03b'):
        with running(cfg, root, args=['--interval-ms', '100']) as run:
            moving(run)
            if case == 'E03':
                recording(run, 1)
                run.wait(lambda: len(run.rows()) >= 2, message='Chưa có ảnh trước khoảng mất')
            mark = len(run.events())
            stamp = run.inject('camera_pause_600ms')
            # Cận trên như stop_by của công cụ thu cũ: 500 ms + 200 ms (chu kỳ + dung sai) + slack.
            stop_within(run, mark, stamp, 500 + 200, 'stale_frame')
            run.wait(lambda: last_commands(run.events()) == (0, 0), 1, 'Góc chưa về 0 sau mất ảnh',
                     tag='stale_frame')
            stop_pair(run.events()[mark:], 'stale_frame')
            if case == 'E03':
                # Đợi bằng chứng ảnh MỚI sau phục hồi, không dùng sleep để đoán.
                run.wait(lambda: any(int(r['monotonic_ns']) / 1000000 > stamp + 700 for r in run.rows()),
                         3, 'Thiếu ảnh phục hồi: cần hook camera_pause_600ms', tag='interface')
                hold = 1.0
            else:
                hold = 1.5  # P tắt: không thấy ảnh, quan sát quá hết khoảng tạm dừng 600 ms
            run.observe(hold, lambda: need(last_commands(run.events()) == (0, 0),
                                         'Tự chạy/rẽ lại khi có ảnh', 'stale_frame'))
            run.act('W', 5, 0, tag='stale_frame')
            run.finish()
        return
    if case == 'E04':
        # HĐ §2: cửa sổ ảnh không chạy/đóng/chết → mã 1; chế độ giả lấy đúng mẫu test_preview.py.
        for mode in ('kill', 'close_after_frame', 'die_after_frame', 'fail_start'):
            env = {} if mode == 'kill' else {'UCR_DRIVE_FAKE_PREVIEW': mode}
            with running(cfg, root / mode, env=env) as run:
                if mode == 'kill':
                    moving(run)
                    # HĐ không quy định số tiến trình con (mã có thêm con camera); con cửa sổ của
                    # collect_common/preview.cpp tự hạ ưu tiên nice 10 → chọn đúng con đó.
                    children = {pid for pid in run.children() if nice(pid) == 10}
                    need(len(children) == 1, 'Cần đúng một cửa sổ giả', 'interface')
                    mark = len(run.events())
                    os.kill(next(iter(children)), signal.SIGKILL)
                    run.finish(code=1, mark=mark)
                else:
                    run.wait(lambda: run.process.poll() is not None, 12, 'Không thoát khi cửa sổ hỏng',
                             alive=False)
                    need(run.process.returncode == 1, 'Cửa sổ {} cần mã 1: {}'.format(mode, run.text()),
                         'exit_code')
                    no_forward(run.events(), 'window_dead')
        return
    if case in ('E05', 'E09'):
        # E09: mẫu công cụ thu cũ coi jpeg_hang và camera_stall là "treo" (mã 4 theo HĐ §2).
        for fault in (['free_mb 0', 'jpeg_fail', 'csv_fail'] if case == 'E05' else ['jpeg_hang', 'camera_stall']):
            with running(cfg, root / fault.replace(' ', '_'), args=['--interval-ms', '100']) as run:
                moving(run)
                recording(run, 1)
                run.wait(lambda: len(run.rows()) >= 2, message='Thiếu ảnh trước lỗi')
                mark = len(run.events())
                run.inject(fault)
                run.finish(code=5 if case == 'E05' else 4, mark=mark)
        return
    if case == 'E05b':
        # Leader chốt: đĩa dưới ngưỡng → mã 5 BẤT KỂ P (cả lúc khởi động).
        with running(cfg, root / 'startup', env={'UCR_DRIVE_FAKE_FREE_MB': '0'}) as run:
            run.wait(lambda: run.process.poll() is not None, 12, 'Không thoát khi đĩa đầy lúc khởi động',
                     alive=False)
            need(run.process.returncode == 5, 'Đĩa đầy lúc khởi động cần mã 5: ' + run.text(), 'exit_code')
            no_forward(run.events(), 'disk_low')
        with running(cfg, root / 'p_off') as run:
            moving(run)
            mark = len(run.events())
            run.inject('free_mb 0')
            run.finish(code=5, mark=mark, tag='disk_low')
        return
    if case == 'E06':
        root.mkdir(parents=True)
        lock = root / 'shared.lock'
        with lock.open('w') as owner:
            fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with running(cfg, root / 'busy', lock=lock) as run:
                run.wait(lambda: run.process.poll() is not None, alive=False)
                need(run.process.returncode == 6, 'Bận khóa không trả 6', 'lock_busy')
                need(not any(e[1].startswith('motor_') or e[1] == 'test_ucr_stop' for e in run.events()),
                     'Bận khóa vẫn chạm motor/helper', 'lock_busy')
        with running(cfg, root / 'released', lock=lock) as run:
            moving(run)
            run.finish()
        return
    if case == 'E06b':
        # Hai bản cùng lúc: B thoát 6 không chạm motor, A không bị ảnh hưởng; A thoát xong B chạy được.
        root.mkdir(parents=True)
        lock = root / 'shared.lock'
        with running(cfg, root / 'A', lock=lock) as first:
            moving(first)
            with running(cfg, root / 'B', lock=lock) as second:
                second.wait(lambda: second.process.poll() is not None, 12, 'B không thoát', alive=False)
                need(second.process.returncode == 6, 'B cần mã 6: ' + second.text(), 'lock_busy')
                need(not any(e[1].startswith('motor_') or e[1].startswith('servo_') or e[1] == 'test_ucr_stop'
                             for e in second.events()), 'B chạm motor/servo/ucr_stop', 'lock_busy')
            first.observe(0.5, lambda: need(last_commands(first.events()) == (5, 5), 'A bị B làm đổi lệnh',
                                            'lock_busy'))
            first.finish()
        with running(cfg, root / 'B2', lock=lock) as run:
            moving(run)
            run.finish()
        return
    if case == 'E07':
        scenarios = [('open', {'UCR_DRIVE_FAKE_OPEN_FAIL': 'camera'}, 3),
                     ('none', {'UCR_DRIVE_FAKE_CAMERA': 'none'}, 3),
                     ('motor', {'UCR_DRIVE_FAKE_MOTOR_FAIL': '1'}, 1)]
        for name, env, code in scenarios:
            with running(cfg, root / name, env=env) as run:
                run.wait(lambda: run.process.poll() is not None, 12, 'Không thoát lỗi khởi động', alive=False)
                need(run.process.returncode == code, 'Sai mã lỗi ' + name + ': ' + run.text(), 'exit_code')
                if name == 'motor':
                    failed_motor_stop(run.events())
                else:
                    no_forward(run.events(), 'startup_fail')
        return
    if case == 'E10':
        # Thoát lúc P đang bật (Esc, SIGTERM, lỗi đĩa mã 5): lượt được đóng gọn.
        for trigger, code in (('E', 0), (signal.SIGTERM, 0), ('free_mb 0', 5)):
            name = trigger if isinstance(trigger, str) else 'sigterm'
            with running(cfg, root / name.replace(' ', '_'), args=['--interval-ms', '100']) as run:
                moving(run)
                recording(run, 1)
                run.wait(lambda: len(run.rows()) >= 3, message='Thiếu ảnh trước khi thoát')
                mark = len(run.events())
                if trigger == 'E':
                    run.send('E')
                elif isinstance(trigger, int):
                    run.process.send_signal(trigger)
                else:
                    run.inject(trigger)
                run.finish(code=code, mark=mark)
                closed_data(run)
        return
    if case in ('P01', 'P02', 'P03'):
        with running(cfg, root, args=['--interval-ms', '100'] if case == 'P02' else []) as run:
            moving(run)
            # shortcut: chưa có vết khung trước P, dùng mốc khung khi hook cung cấp.
            # A đã chốt trong vết; chờ 5 chu kỳ camera giả (2*kPollMs = 20 ms) và dự phòng sanitizer.
            run.observe(0.1 + cfg.slack / 1000)
            if case == 'P01':
                initial = off_snapshot(run)
                need(not initial[0] and not initial[1] and not initial[2], 'P mặc định bật', 'record_default')
                run.observe(1.3, lambda: need(off_snapshot(run) == initial, 'Ghi khi chưa P', 'record_default'))
            elif case == 'P02':
                for count in (1, 2):
                    before = len(run.rows())
                    recording(run, count)
                    run.wait(lambda: len(run.rows()) >= before + 3, message='Không ghi khi P bật')
                    recording_off(run, count)
            else:
                recording(run, 1)
                run.wait(lambda: len(run.rows()) >= 4, 8, 'Nhịp mặc định không ghi 1 ảnh/s')
                stamps = [int(r['monotonic_ns']) / 1000000 for r in run.rows()]
                gaps = sorted(b - a for a, b in zip(stamps, stamps[1:]))
                median = gaps[len(gaps) // 2]
                need(min(gaps) >= 800 and 900 <= median <= 1200 + cfg.slack,
                     'Nhịp mặc định khác 1000 ms: ' + repr(gaps), 'record_interval')
            run.finish()
            if case == 'P02':
                inspect_data(run)
                stamps = [int(r['monotonic_ns']) for r in run.rows() if int(r['pass']) == 1]
                # Nhịp 100 ms: khoảng trung bình phải nhỏ rõ so với mặc định 1000 ms.
                need(len(stamps) >= 3 and (stamps[-1] - stamps[0]) / (len(stamps) - 1) <= 500000000,
                     'Không dùng --interval-ms 100', 'record_interval')
        return
    if case == 'F01':
        bad = [['--unknown'], ['--interval-ms', '0'], ['--interval-ms', '-1'],
               ['--interval-ms', 'abc'], ['--interval-ms', '1x'], ['--interval-ms'],
               ['--reverse-motor=abc'], ['--accept-no-obstacle-brake']]
        bad += [[flag] for flag in ('--label', '--out-root', '--who', '--weather',
                                   '--curtain', '--lights', '--props', '--note')]
        for index, args in enumerate([['--help']] + bad):
            with running(cfg, root / str(index), args=args) as run:
                run.wait(lambda: run.process.poll() is not None, alive=False)
                need(run.process.returncode == (0 if index == 0 else 2), 'Sai mã cờ: ' + repr(args))
                need(not any(e[1].startswith('motor_') for e in run.events()), 'Cờ/help chạm motor')
                if index == 0:
                    for flag in ('label', 'out-root', 'interval-ms', 'reverse-motor', 'who',
                                 'weather', 'curtain', 'lights', 'props', 'note', 'help'):
                        need('--' + flag in run.text(), 'Help thiếu --' + flag)
                    need('--accept-no-obstacle-brake' not in run.text(), 'Help giữ cờ cũ')
        metadata = dict(who='tester', weather='sunny', curtain='open', lights='on', props='none', note='T33')
        args = [item for pair in metadata.items() for item in ('--' + pair[0], pair[1])]
        with running(cfg, root / 'metadata', args=args) as run:
            moving(run)
            run.finish()
            sessions = list(run.out.rglob('session.json'))
            need(len(sessions) == 1, 'Cần đúng một session.json')
            session = json.loads(sessions[0].read_text(encoding='utf-8'))
            for key, value in metadata.items():
                need(session.get(key) == value, 'Không giữ metadata ' + key)
            values = json.dumps(session, ensure_ascii=False)
            need(socket.gethostname() in values and cfg.revision in values, 'Thiếu hostname/git trong session')
        return
    if case == 'F02':
        with running(cfg, root / 'found', default=True) as run:
            moving(run)
            recording(run, 1)
            run.wait(lambda: run.rows(), message='Không ghi trong gốc/data_raw')
            run.finish()
            need(not (run.root / 'data_raw').exists(), 'Lấy thư mục hiện hành làm gốc')
        with running(cfg, root / 'missing', default=True, config=False) as run:
            run.wait(lambda: run.process.poll() is not None, alive=False)
            need(run.process.returncode == 1 and 'config.yaml' in run.text(), 'Thiếu gốc không báo lỗi rõ/mã 1')
        return
    if case == 'F04':
        for reverse in (False, True):
            with running(cfg, root / str(reverse), args=['--reverse-motor'] if reverse else []) as run:
                moving(run, reverse)
                run.finish()
                banner(run.text(), cfg, run.out)
        return
    raise Failure('setup', 'Chưa đăng ký ca ' + case)


def banner(text, cfg, out):
    """F04: khung cảnh báo + thông tin khởi động theo phiếu (khớp có dấu hoặc không dấu)."""
    low = text.lower()
    need(re.search(r'phanh chống vật cản|phanh chong vat can', low), 'Thiếu "không có phanh chống vật cản"',
         'startup_banner')
    need(re.search(r'người giám sát đứng sát|nguoi giam sat dung sat', low), 'Thiếu câu người giám sát',
         'startup_banner')
    need('space/x' in low, 'Thiếu "Space/X dừng"', 'startup_banner')
    # Chấp nhận "5 cm/s" hoặc dạng của mã "Tốc độ: bước=5 trần=40 phần cứng=60 cm/s" (cùng dòng).
    need(re.search(r'\b5(?:[.,]0+)?\s*cm/s|(?:tốc độ|toc do)[^\n]*(?:bước|buoc)\s*[=:]\s*5(?:[.,]0+)?\b[^\n]*cm/s',
                   low), 'Không in bước 5 cm/s', 'startup_banner')
    for number in (40, 60):
        need(re.search(r'\b{}(?:[.,]0+)?\b'.format(number), low), 'Không in hằng ' + str(number), 'startup_banner')
    need(re.search(r'\b500\b|\b0[.,]5\s*s\b', low), 'Không in ngưỡng mất ảnh 500 ms/0,5 s', 'startup_banner')
    need(re.search(r'\b1000\b|\b1(?:[.,]0+)?\s*(?:s|giây)\b|1 ảnh/(?:s|giây)', low), 'Không in nhịp 1000 ms',
         'startup_banner')
    need(socket.gethostname() in text and cfg.revision in text and str(out) in text,
         'Thiếu hostname/git/thư mục khởi động', 'startup_banner')
    need('reverse' in low or 'đảo' in low or 'dao' in low, 'Không in trạng thái đảo chiều', 'startup_banner')


def mask(source):
    """Che comment/chuỗi (giữ nguyên độ dài) để neo không khớp nhầm vào chú thích."""
    return re.sub(r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'',
                  lambda m: ' ' * len(m.group()), source, flags=re.S)


def function_spans(source):
    """Tìm thân hàm trên bản sao, che comment/chuỗi trước khi đếm ngoặc."""
    masked = mask(source)
    pattern = (r'\b(void|bool|float|double|int)\s+([\w:]+)\s*\(([^;{}]*)\)\s*'
               r'(?:(?:const|noexcept|override|final)\s*)*\{')
    for match in re.finditer(pattern, masked):
        start = match.end()
        depth, end = 1, start
        while depth and end < len(masked):
            depth += (masked[end] == '{') - (masked[end] == '}')
            end += 1
        need(depth == 0, 'Ngoặc mutation không cân', 'mutation_setup')
        yield match, start, end - 1


def patch_function(path, name, replacement):
    """Neo theo TÊN hàm trong hợp đồng; tham số float/int/double; không đúng một neo → mutation_setup."""
    source = path.read_text(encoding='utf-8')
    found = [(m, start, end) for m, start, end in function_spans(source) if m.group(2) == name]
    need(len(found) == 1, 'Không có đúng một neo {} trong {}'.format(name, path.name), 'mutation_setup')
    match, start, end = found[0]
    param = re.search(r'\b(?:float|int|double)\s+(\w+)', match.group(3))
    need(param is not None, 'Hàm thiếu tham số số ' + name, 'mutation_setup')
    path.write_text(source[:start] + replacement(param.group(1), source[start:end]) + source[end:], encoding='utf-8')


def identity(value, body):
    return '\n return ' + value + ';\n'


def remove_static_asserts(source, only=None):
    """Gỡ các static_assert (trong limits.h: tất cả; file khác: chỉ cái nhắc tên `only`)."""
    masked = mask(source)
    spans = []
    for match in re.finditer(r'\bstatic_assert\s*\(', masked):
        depth, end = 1, match.end()
        while depth and end < len(masked):
            depth += (masked[end] == '(') - (masked[end] == ')')
            end += 1
        semicolon = masked.find(';', end - 1)
        need(depth == 0 and semicolon >= 0, 'static_assert không cân ngoặc', 'mutation_setup')
        if only is None or only in masked[match.start():semicolon]:
            spans.append((match.start(), semicolon + 1))
    for start, end in reversed(spans):
        source = source[:start] + source[end:]
    return source


def widen_limit(folder, constant):
    """M2a: nới trần phím `constant` lên 100 và gỡ static_assert liên quan; giữ nguyên kẹp phần cứng."""
    limits = folder / 'limits.h'
    source = limits.read_text(encoding='utf-8')
    found = list(re.finditer(r'\b' + constant + r'\s*(?:=\s*|\{\s*)([0-9]+(?:\.[0-9]*)?f?)', mask(source)))
    need(len(found) == 1, 'Không có đúng một định nghĩa ' + constant, 'mutation_setup')
    value = found[0]
    widened = '100.0f' if 'Speed' in constant else '100'
    source = source[:value.start(1)] + widened + source[value.end(1):]
    limits.write_text(remove_static_asserts(source), encoding='utf-8')
    for path in folder.iterdir():
        if path.suffix in ('.cpp', '.h') and path.name != 'limits.h' and not path.name.startswith('test_'):
            text = path.read_text(encoding='utf-8')
            changed = remove_static_asserts(text, constant)
            if changed != text:
                path.write_text(changed, encoding='utf-8')


def patch_mutant(case, folder):
    # Chỉ hàm này đọc .cpp/.h, và CHỈ dưới bản sao tạm đã tạo ở mutation().
    limits = folder / 'limits.h'
    if case in ('M1', 'M1b'):
        patch_function(limits, 'clampKeySpeed' if case == 'M1' else 'clampKeySteer', identity)
    elif case in ('M2', 'M2s'):
        patch_function(limits, 'clampHwSpeed' if case == 'M2' else 'clampHwSteer', identity)
    elif case == 'M3':
        source = limits.read_text(encoding='utf-8')
        # Neo là ĐỊNH NGHĨA (tên HĐ), không khớp chú thích; giữ nguyên specifier (inline/constexpr).
        found = list(re.finditer(r'((?:(?:static|inline|constexpr)\s+)*)float\s+applyMotorSign\s*\(',
                                 mask(source)))
        need(len(found) == 1, 'Neo định nghĩa applyMotorSign không duy nhất/không có trong limits.h',
             'mutation_setup')
        match = found[0]
        name_at = source.index('applyMotorSign', match.start())
        changed = source[:name_at] + 't33SignOnce' + source[name_at + len('applyMotorSign'):]
        spans = [(m, start, end) for m, start, end in function_spans(changed) if m.group(2) == 't33SignOnce']
        need(len(spans) == 1, 'Không thấy thân applyMotorSign', 'mutation_setup')
        _, _, end = spans[0]
        # Giữ nguyên thân lần 1; gọi chính phép biến đổi đó lần 2 trên kết quả (đảo dấu hai lần).
        changed = changed[:end + 1] + ('\n' + (match.group(1) or 'inline ') +
                                      'float applyMotorSign(float v, bool reverse) {\n'
                                      ' return t33SignOnce(t33SignOnce(v, reverse), reverse);\n}\n') + changed[end + 1:]
        limits.write_text(changed, encoding='utf-8')
    elif case == 'M4':
        # Neo: hàm ghi vết "motor_speed"/"motor_emergency" (định dạng vết HĐ) ở bất kỳ file nguồn nào
        # của collect_drive; phải đúng MỘT neo motor_speed, không thì báo lỗi thiết lập (không bao giờ "đạt").
        files = sorted(p for p in folder.iterdir() if p.suffix in ('.cpp', '.h') and not p.name.startswith('test_'))
        speed_count = emergency_count = 0
        touched = []
        for path in files:
            source = path.read_text(encoding='utf-8')
            edits = []
            for match, start, end in function_spans(source):
                body = source[start:end]
                # Giả vờ gửi thành công để chương trình thoát như thường (lỗi phải là exit_stop, không phải mã thoát).
                returned = {'void': 'return;', 'bool': 'return true;'}.get(match.group(1), 'return {};')
                if re.search(r'"motor_speed(?:\s|"|\\)', body):
                    param = re.search(r'\b(?:float|double)\s+(\w+)', match.group(3))
                    need(param is not None, 'Neo motor_speed không có tham số số thực: ' + path.name,
                         'mutation_setup')
                    v = param.group(1)
                    edits.append((start, '\n if (' + v + ' != 0) t33Moved() = true;\n'
                                  ' else if (t33Moved()) ' + returned + '\n'))
                    speed_count += 1
                elif re.search(r'"motor_emergency(?:\s|"|\\)', body):
                    edits.append((start, '\n if (t33Moved()) ' + returned + '\n'))
                    emergency_count += 1
            if edits:
                for start, addition in sorted(edits, reverse=True):
                    source = source[:start] + addition + source[start:]
                touched.append((path, source))
        need(speed_count == 1 and emergency_count <= 1,
             'Cần đúng một neo ghi vết motor_speed (và tối đa một motor_emergency); thấy {} / {}'.format(
                 speed_count, emergency_count), 'mutation_setup')
        for path, source in touched:
            # Cờ dùng chung qua hàm inline: header và .cpp cùng thấy một biến.
            path.write_text('#ifndef T33_MOVED_DEFINED\n#define T33_MOVED_DEFINED\n'
                            'inline bool& t33Moved() { static bool v = false; return v; }\n#endif\n' + source,
                            encoding='utf-8')
    elif case == 'M5':
        # Gọi hàm ToF giả có ký hiệu thật, không chỉ thêm chuỗi để lừa nm.
        path = folder / 'main.cpp'
        source = path.read_text(encoding='utf-8')
        path.write_text('namespace ToF { __attribute__((noinline)) void Init() {\n'
                        ' asm volatile("" ::: "memory");\n} }\n'
                        'struct T33ToFCall { T33ToFCall() { ToF::Init(); } };\n'
                        'static T33ToFCall t33ToFCall;\n' + source, encoding='utf-8')
    else:
        raise Failure('mutation_setup', 'Mã mutation không biết: ' + case)


def copy_source(cfg, dest):
    folder = dest / 'collect_drive'
    shutil.copytree(cfg.source, str(folder), ignore=shutil.ignore_patterns('build*', '__pycache__', '*.pyc'))
    shutil.copytree(str(Path(cfg.source).parent / 'collect_common'), str(dest / 'collect_common'),
                    ignore=shutil.ignore_patterns('build*', '__pycache__', '*.pyc'))
    return folder


def build_targets(cfg, root, targets, budget):
    """targets: [(tên, thư mục nguồn, 'hooks'|'limits')]; một --target mỗi lệnh, -j1, có tổng hạn."""
    project = root / 'mutation-project'
    project.mkdir()
    lines = ['cmake_minimum_required(VERSION 3.10)', 'project(T33Mutation LANGUAGES CXX)',
             'set(CMAKE_CXX_STANDARD 17)', 'set(CMAKE_CXX_STANDARD_REQUIRED ON)',
             'find_package(OpenCV 4.1 REQUIRED COMPONENTS core imgcodecs videoio imgproc OPTIONAL_COMPONENTS highgui)',
             'find_package(Threads REQUIRED)']
    for name, src, kind in targets:
        src = Path(src).as_posix()
        if kind == 'limits':
            lines.append('add_executable({} "{}/test_limits.cpp")'.format(name, src))
            continue
        lines += ['add_executable({0} "{1}/main.cpp" "{1}/../collect_common/preview.cpp" '
                  '"{1}/../collect_common/pass_store.cpp")'.format(name, src),
                  'target_compile_definitions({} PRIVATE UCR_DRIVE_TEST_HOOKS=1 '
                  'UCR_DRIVE_GIT_REVISION="${{REVISION}}")'.format(name),
                  'if(OpenCV_highgui_FOUND)',
                  ' target_compile_definitions({} PRIVATE UCR_DRIVE_HAVE_HIGHGUI=1)'.format(name), 'endif()',
                  'target_include_directories({} PRIVATE ${{OpenCV_INCLUDE_DIRS}})'.format(name),
                  'target_link_libraries({} PRIVATE ${{OpenCV_LIBS}} Threads::Threads)'.format(name),
                  'if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9)',
                  ' target_link_libraries({} PRIVATE stdc++fs)'.format(name), 'endif()']
    (project / 'CMakeLists.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    build = root / 'build'
    build.mkdir()
    deadline = time.monotonic() + budget
    commands = [[cfg.cmake, str(project), '-DREVISION=' + cfg.revision, '-DCMAKE_CXX_COMPILER=' + cfg.cxx,
                 '-DCMAKE_BUILD_TYPE=Release', '-DCMAKE_CXX_FLAGS=-O0 -g0']]
    commands += [[cfg.cmake, '--build', '.', '--target', name, '--', '-j1'] for name, _, _ in targets]
    for command in commands:
        remaining = deadline - time.monotonic()
        need(remaining > 0, 'Hết hạn build mutation', 'mutation_setup')
        code, output = tool(command, build, remaining)
        need(code == 0, 'Build mutation hỏng: ' + output[-4000:], 'mutation_setup')
    return build


def mutant_cfg(cfg, binary):
    mutant = argparse.Namespace(**vars(cfg))
    mutant.hooks = str(binary)
    mutant.slack = 0  # Bản sao Release, không sanitizer.
    return mutant


PROBES = {'M1': ('K01', 'key_speed'), 'M1b': ('K03', 'key_steer'), 'M3': ('S01', 'motor_sign'),
          'M4': ('E01', 'exit_stop'), 'M5': ('F03', 'no_tof')}


def mutation(case, cfg, root):
    root.mkdir(parents=True)
    if case in ('M2', 'M2s'):
        # M2a (đối chứng): nới trần phím lên 100, GIỮ kẹp phần cứng → phải chạm đúng 60, không vượt.
        # M2b (lỗi): M2a + kẹp phần cứng trả nguyên đầu vào → vết > 60 → phải HỎNG tag hw_*.
        kind = 'speed' if case == 'M2' else 'steer'
        constant = 'kMaxSpeedCms' if kind == 'speed' else 'kMaxSteerDeg'
        code, output = tool([cfg.limits], root)
        need(code == 0 and 'OK limits T33' in output, 'Baseline limits lỗi: ' + output, 'mutation_setup')
        control = copy_source(cfg, root / 'control')
        widen_limit(control, constant)
        broken = copy_source(cfg, root / 'broken')
        widen_limit(broken, constant)
        patch_mutant(case, broken)
        unit = copy_source(cfg, root / 'unit')
        patch_mutant(case, unit)
        build = build_targets(cfg, root, [('control', control, 'hooks'), ('broken', broken, 'hooks'),
                                          ('unit', unit, 'limits')], 200)
        try:
            hw_probe(mutant_cfg(cfg, build / 'control'), root / 'probe_control', kind)
        except Failure as error:
            raise Failure('mutation_setup', 'Đối chứng M2a phải đạt (≤60 và chạm 60): ' + str(error))
        try:
            hw_probe(mutant_cfg(cfg, build / 'broken'), root / 'probe_broken', kind)
        except Failure as error:
            need(error.tag == 'hw_' + kind, 'Hỏng sai phép kiểm: ' + str(error), 'mutation_setup')
        else:
            raise Failure('mutation_survived', case + ' (bỏ kẹp phần cứng) không bị phát hiện')
        code, output = tool([str(build / 'unit')], root)
        need(code != 0 and 'limits_hw_' + kind in output,
             case + ' sống sót/hỏng sai phép kiểm ở test_limits: ' + output, 'mutation_survived')
        print(case + ': đối chứng đạt (chạm 60), bản lỗi HỎNG đúng hw_' + kind + ' và limits_hw_' + kind)
        return
    probe, expected = PROBES[case]
    # Baseline phải đạt trước; lỗi build/neo/khởi động không phải phát hiện lỗi.
    test_case(probe, cfg, root / 'baseline')
    folder = copy_source(cfg, root / 'source')
    patch_mutant(case, folder)
    build = build_targets(cfg, root, [('mutant', folder, 'hooks')], 150)
    mutant = mutant_cfg(cfg, build / 'mutant')
    if case == 'M5':
        mutant.real = mutant.hooks
    try:
        test_case(probe, mutant, root / 'probe')
    except Failure as error:
        need(error.tag == expected, 'Hỏng sai phép kiểm: ' + str(error), 'mutation_setup')
    else:
        raise Failure('mutation_survived', case + ' không bị phát hiện')
    print(case + ': baseline đạt, bản lỗi HỎNG đúng phép kiểm ' + expected)


def expect_failure(tag, function, *args):
    try:
        function(*args)
    except Failure as error:
        need(error.tag == tag, 'Tự kiểm: sai mã phép kiểm {} / {}'.format(error.tag, tag), 'self_check')
    else:
        raise Failure('self_check', 'Tự kiểm: bộ kiểm bỏ lọt lỗi ' + tag)


def self_check():
    """Kiểm bộ đọc vết, phép kiểm dừng và bộ vá M1–M5 trên đoạn mã mẫu do test tự viết.

    Chạy được cả Windows; KHÔNG thay ca Linux và không đọc nguồn DEV.
    """
    stop = [(1, 'motor_speed', 5.0, False), (2, 'motor_speed', 0.0, False), (3, 'servo_angle', 0.0, False)]
    stop_pair(stop)
    expect_failure('stop_order', stop_pair, [stop[0], stop[2], stop[1]])
    expect_failure('stop_order', stop_pair, stop[:2])
    expect_failure('stop_order', stop_pair, stop + [(4, 'motor_speed', 5.0, False)])
    expect_failure('key_speed', safe_range, [(1, 'motor_speed', 45.0, False)])
    expect_failure('key_steer', safe_range, [(1, 'servo_angle', -41.0, False)])
    expect_failure('positive_zero', safe_range, [(1, 'motor_speed', -0.0, False)])
    need(last_commands([(1, 'motor_speed', 5.0, False), (2, 'motor_emergency', '', False),
                        (3, 'servo_angle', 0.0, False)]) == (0, 0), 'Tự kiểm last_commands', 'self_check')
    with tempfile.TemporaryDirectory(prefix='ucr-t33-self-') as tmp:
        folder = Path(tmp)
        (folder / 'trace').write_text('1 motor_speed 5\n2 key  \n3 motor_speed 0 fail\n4 key', encoding='utf-8')
        events = trace(folder / 'trace')
        need(len(events) == 3 and events[1][2] == ' ' and events[2][3], 'Tự kiểm bộ đọc vết', 'self_check')
        limits_h = ('namespace collect_drive {\n'
                    '// applyMotorSign(v, reverse): chú thích nhắc tên, không phải neo\n'
                    'constexpr float kMaxSpeedCms = 40.0f;\n'
                    'constexpr int kMaxSteerDeg{40};\n'
                    'static_assert(kMaxSpeedCms <= 60.0f, "trần (không phải ngoặc)");\n'
                    'constexpr float clampKeySpeed(float v) { return v < -40 ? -40 : (v > 40 ? 40 : v); }\n'
                    'inline float clampHwSpeed(const float v) noexcept { return v < -60 ? -60 : (v > 60 ? 60 : v); }\n'
                    'constexpr int clampKeySteer(int a) { return a < -40 ? -40 : (a > 40 ? 40 : a); }\n'
                    'constexpr int clampHwSteer(int deg) { return deg < -60 ? -60 : (deg > 60 ? 60 : deg); }\n'
                    'constexpr float applyMotorSign(float v, bool reverse) {\n'
                    '    return v == 0 ? +0.0f : (reverse ? -v : v);\n}\n}\n')
        main_cpp = ('#include "limits.h"\nstruct FakeMotor {\n'
                    '  bool SetSpeedCms(float cms) { trace("motor_speed %.3f", cms); return true; }\n'
                    '  void EmergencyStop() { trace("motor_emergency"); }\n};\n'
                    'static_assert(collect_drive::kMaxSteerDeg < 60, "x");\n'
                    'static_assert(sizeof(int) == 4, "khác");\n'
                    'int main(int argc, char** argv) { const char* s = "{ không phải ngoặc"; return 0; }\n')
        checks = {
            'M1': lambda l, m: 'clampKeySpeed(float v) {\n return v;' in l,
            'M1b': lambda l, m: 'clampKeySteer(int a) {\n return a;' in l,
            'M2': lambda l, m: 'noexcept {\n return v;' in l,
            'M2s': lambda l, m: 'clampHwSteer(int deg) {\n return deg;' in l,
            'M3': lambda l, m: ('constexpr float t33SignOnce(' in l and
                                'constexpr float applyMotorSign(float v, bool reverse) {\n return t33SignOnce(t33SignOnce('
                                in l and '// applyMotorSign(v, reverse)' in l),
            'M4': lambda l, m: ('if (cms != 0) t33Moved() = true;' in m and
                                'else if (t33Moved()) return true;' in m and '{\n if (t33Moved()) return;' in m),
            'M5': lambda l, m: 'ToF::Init();' in m}
        for case in ('M1', 'M1b', 'M2', 'M2s', 'M3', 'M4', 'M5', 'W-speed', 'W-steer'):
            work = folder / case
            work.mkdir()
            (work / 'limits.h').write_text(limits_h, encoding='utf-8')
            (work / 'main.cpp').write_text(main_cpp, encoding='utf-8')
            if case.startswith('W-'):
                widen_limit(work, 'kMaxSpeedCms' if case == 'W-speed' else 'kMaxSteerDeg')
            else:
                patch_mutant(case, work)
            new_limits = (work / 'limits.h').read_text(encoding='utf-8')
            new_main = (work / 'main.cpp').read_text(encoding='utf-8')
            if case == 'W-speed':
                ok = ('kMaxSpeedCms = 100.0f;' in new_limits and 'static_assert' not in new_limits and
                      'clampHwSpeed(const float v) noexcept { return v < -60' in new_limits and
                      new_main.count('static_assert') == 2)
            elif case == 'W-steer':
                ok = ('kMaxSteerDeg{100}' in new_limits and 'static_assert' not in new_limits and
                      new_main.count('static_assert') == 1 and 'sizeof(int)' in new_main)
            else:
                ok = checks[case](new_limits, new_main)
            need(ok, 'Tự kiểm bộ vá ' + case, 'self_check')
        # Không có neo → lỗi thiết lập rõ ràng, tuyệt đối không coi là phát hiện.
        empty = folder / 'empty'
        empty.mkdir()
        (empty / 'limits.h').write_text('namespace collect_drive {}\n', encoding='utf-8')
        (empty / 'main.cpp').write_text('int main() { return 0; }\n', encoding='utf-8')
        for case in ('M1', 'M1b', 'M2', 'M2s', 'M3', 'M4'):
            expect_failure('mutation_setup', patch_mutant, case, empty)
        for constant in ('kMaxSpeedCms', 'kMaxSteerDeg'):
            expect_failure('mutation_setup', widen_limit, empty, constant)
        # Bộ kiểm motor lỗi / ucr_stop / khung cảnh báo.
        failed_motor_stop([(1, 'motor_speed', 0.0, True), (2, 'motor_emergency', '', True),
                           (3, 'servo_angle', 0.0, False)])
        expect_failure('motor_fail', failed_motor_stop, [(1, 'servo_angle', 0.0, False)])
        expect_failure('motor_fail', failed_motor_stop, [(1, 'motor_speed', 5.0, True)])
        ucr_stop_order([(0, 'test_ucr_stop', '', False), (1, 'motor_init', '', False),
                        (2, 'motor_speed', 0.0, False), (3, 'servo_angle', 0.0, False),
                        (4, 'test_ucr_stop', '', False)])
        expect_failure('stop_helper', ucr_stop_order, [(1, 'motor_speed', 0.0, False),
                                                       (2, 'test_ucr_stop', '', False),
                                                       (3, 'servo_angle', 0.0, False),
                                                       (4, 'test_ucr_stop', '', False)])
        banner('KHÔNG CÓ PHANH CHỐNG VẬT CẢN; người giám sát đứng sát xe; Space/X dừng\n'
               'host {} git abc1234 out /o bước 5 cm/s tối đa 40 phần cứng 60 mất ảnh 0,5 s nhịp 1000 ms '
               'đảo chiều: không'.format(socket.gethostname()), argparse.Namespace(revision='abc1234'), '/o')
        expect_failure('startup_banner', banner, 'không phanh; Space dừng', argparse.Namespace(revision='x'), '/o')
    cmake = Path(__file__).with_name('tests.cmake').read_text(encoding='utf-8')
    registered = re.search(r'set\(_collect_cases\s+([^)]*)\)', cmake).group(1).split()
    need(registered == [c for c in CASES if c != 'SC'], 'Danh sách ca CMake lệch CASES', 'self_check')
    print('ĐẠT tự kiểm: vết, cặp dừng, trần ±40, +0.0, bộ vá M1/M1b/M2/M2s/M3/M4/M5 + nới trần, motor lỗi, ucr_stop, khung cảnh báo, đăng ký ca; chưa chạy binary')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', required=True, choices=CASES)
    for name in ('hooks', 'real', 'limits', 'source', 'cmake', 'cxx', 'nm', 'revision'):
        parser.add_argument('--' + name)
    # Như công cụ thu cũ: +300 ms chỉ khi build sanitizer; không cộng vào phép kiểm "qua 0 không nghỉ".
    parser.add_argument('--slack', type=int, choices=(0, 300), default=0)
    cfg = parser.parse_args()
    if cfg.case == 'SC':
        return self_check()
    need(all(getattr(cfg, n) for n in ('hooks', 'real', 'limits', 'source', 'cmake', 'cxx', 'nm', 'revision')),
         'Thiếu tham số đường dẫn cho ca ' + cfg.case, 'environment')
    need(os.name == 'posix', 'Chỉ chạy chấp nhận trên Linux; Windows chỉ kiểm cú pháp', 'environment')
    with tempfile.TemporaryDirectory(prefix='ucr-t33-') as tmp:
        if cfg.case.startswith('M'):
            mutation(cfg.case, cfg, Path(tmp) / cfg.case)
        else:
            test_case(cfg.case, cfg, Path(tmp) / cfg.case)
    print('PASS ' + cfg.case)


if __name__ == '__main__':
    main()
