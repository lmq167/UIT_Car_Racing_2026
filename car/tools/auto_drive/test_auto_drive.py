"""T47 P1: chấp nhận hộp đen theo leader L1–L7 và bản duyệt; Python 3.6/Linux.

Chỉ chạy hooks, không đọc nguồn DEV. A27/A33 là kiểm chung và có ca riêng.
Tên trường CSV/session bổ sung được công bố trong T47t_report.md cho DEV.
"""
import argparse
from contextlib import contextmanager
import csv
import hashlib
import importlib.util
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
import zlib

HERE = Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location('t33', str(HERE.parent / 'collect_drive' / 'test_collect_drive.py'))
td = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(td)
need, Failure = td.need, td.Failure
_spec_info = importlib.util.spec_from_file_location('road_fixture', str(HERE.parent / 'test_model' / 'make_fake_road_onnx.py'))
road_fixture = importlib.util.module_from_spec(_spec_info)
_spec_info.loader.exec_module(road_fixture)

CASES = ['A{:02d}'.format(i) for i in range(1, 36) if i != 30] + ['A60']
BOUNDS = {
    'control_period_ms': (50, 20, 100, 'ms'), 'mask_max_age_ms': (300, 50, 1000, 'ms'),
    'cmd_max_age_ms': (200, 50, 500, 'ms'), 'det_max_age_ms': (500, 50, 2000, 'ms'),
    'max_speed_cms': (10, 0, 10, 'cm/s'), 'cruise_cms': (8, 0, 10, 'cm/s'),
    'min_cms': (4, 0, 10, 'cm/s'), 'lost_cms': (4, 0, 10, 'cm/s'),
    'slow_per_deg': (0.1, 0, 1, 'cm/s/'), 'steer_max_deg': (30, 0, 40, ''),
    'steer_rate_dps': (200, 10, 1000, '/s'), 'kp': (30, 0, 200, ''),
    'kd': (2, 0, 50, ''), 'lane_width_min_ratio': (0.6, 0.1, 1, ''),
    'lane_width_max_ratio': (1.4, 1, 3, ''), 'center_offset_frac': (0, -0.5, 0.5, ''),
    'min_valid_rows': (1, 1, 5, ''), 'lost_hold_ms': (300, 0, 2000, 'ms'),
    'lost_stop_ms': (800, 0, 3000, 'ms'), 'regain_frames': (3, 1, 20, ''),
    'record_every_n': (5, 1, 40, ''), 'yolo_every_n': (3, 1, 10, '')}
VECTORS = {'scan_rows': [0.85, 0.65, 0.45], 'row_weights': [0.5, 0.3, 0.2],
           'lane_width_frac': [0.70, 0.55, 0.40]}
CSV_FIELDS = ('t_ms frame_id image_age_ms mask_age_ms det_age_ms planner_state '
              'scan_rows e p d requested_steer_deg requested_speed_cms sent_steer_deg '
              'sent_speed_cms reason road_ms yolo_ms loop_ms dropped csv_dropped').split()
STATS = 'camera_fps road_ms yolo_ms loop_p95_ms rss_mb dropped free_mb'.split()


def yaml_text(values):
    return '{}\n' if not values else ''.join('{}: {}\n'.format(k, json.dumps(v, ensure_ascii=False))
                                            for k, v in sorted(values.items()))


def png_bytes(rows):
    def chunk(kind, data):
        return struct.pack('>I', len(data)) + kind + data + struct.pack('>I', zlib.crc32(kind + data) & 0xffffffff)
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', 160, 96, 8, 0, 0, 0, 0)) +
            chunk(b'IDAT', zlib.compress(b''.join(b'\0' + bytes(row) for row in rows))) + chunk(b'IEND', b''))


def mask_rows(mode):
    rows = []
    offset = {'straight': 0, 'left': -12, 'right': 12, 'empty': 0}[mode]
    for y in range(96):
        frac = y / 96.0
        width = 160 * (0.40 + (frac - 0.45) * 0.75)
        left, right = int(80 + offset - width / 2), int(80 + offset + width / 2)
        row = bytearray(160)
        if mode != 'empty':
            for x in range(max(0, left), min(160, right + 1)):
                row[x] = 2 if x in (left, right) else 1
        rows.append(row)
    return rows


def fixtures(root, schedule):
    for mode in ('straight', 'left', 'right', 'empty'):
        rows = mask_rows(mode)
        need(len(rows) == 96 and all(len(r) == 160 and set(r) <= {0, 1, 2} for r in rows),
             'Fixture sai cỡ/lớp', 'setup')
        (root / (mode + '.png')).write_bytes(png_bytes(rows))
    (root / 'list.csv').write_text('frame_id,mask\n' + ''.join(
        '{},{}.png\n'.format(frame, mode) for frame, mode in schedule), encoding='utf-8')
    (root / 'road_info.json').write_text(json.dumps(road_fixture.info(96, 160)), encoding='utf-8')


def cap(events):
    # A33: kiểm cả lệnh thử gửi bị fail; không cho lọt giá trị ngoài lớp kẹp.
    for _, kind, value, _ in events:
        if kind in ('motor_speed', 'servo_angle'):
            limit = 10 if kind == 'motor_speed' else 40
            need(math.isfinite(value) and abs(value) <= limit, 'Vượt trần cứng: {} {}'.format(kind, value), 'invariant_cap')


class Run(td.Run):
    """Dùng lại wait/observe/text/send/ack/children/close của T33; chỉ đổi khởi chạy auto."""
    def __init__(self, cfg, root, values=None, env=None, schedule=((1, 'straight'),),
                 args=None, lock=None, setup=None, no_yolo=True, info=True, stderr=None):
        need(b'UCR_AUTO_FAKE_TRACE' in Path(cfg.hooks).read_bytes(),
             'Từ chối binary không có hooks auto', 'environment')
        self.root, self.slack, self.seq = root, cfg.slack, 0
        root.mkdir(parents=True)
        self.project = root / 'project'
        bindir = self.project / 'bin'
        bindir.mkdir(parents=True)
        self.binary = bindir / 'ucr_auto_hooks'
        shutil.copy2(cfg.hooks, str(self.binary))
        self.path, self.keys, self.ctrl = root / 'trace', root / 'keys', root / 'ctrl'
        self.keys.touch()
        self.ctrl.touch()
        self.out, self.control = root / 'out', self.project / 'control.yaml'
        self.values = {} if values is None else dict(values)
        self.control.write_text(yaml_text(self.values), encoding='utf-8')
        fixtures(root, schedule)
        if setup:
            setup(self)
        helper = bindir / 'ucr_stop'
        helper.write_text('#!' + sys.executable + '\nimport os,time\n'
                          'with open(os.environ["UCR_AUTO_FAKE_TRACE"],"a") as f:\n'
                          ' f.write("%d test_ucr_stop\\n" % int(time.monotonic()*1000))\n', encoding='utf-8')
        helper.chmod(0o755)
        child_env = {k: v for k, v in os.environ.items()
                     if not k.startswith(('UCR_AUTO_FAKE_', 'UCR_DRIVE_FAKE_', 'UCR_CAM_TEST_'))
                     and k not in ('DISPLAY', 'WAYLAND_DISPLAY')}
        child_env.update(LC_ALL='C.UTF-8', LANG='C.UTF-8', DISPLAY=':ucr_fake_only')
        fake = dict(TRACE=str(self.path), LOCK=str(lock or root / 'vehicle.lock'), CTRL=str(self.ctrl),
                    PREVIEW='stream:' + str(self.keys), CAMERA='ok', FREE_MB='4096',
                    ROAD_MASKS=str(root / 'list.csv'))
        fake.update(env or {})
        child_env.update(('UCR_AUTO_FAKE_' + k, v) for k, v in fake.items())
        child_env['TSAN_OPTIONS'] = ':'.join(filter(None, (child_env.get('TSAN_OPTIONS', ''),
                                                        'suppressions=' + str(td.TSAN_SUPP))))
        argv = [str(self.binary), '--control', str(self.control), '--out-root', str(self.out)]
        if info:
            argv.extend(['--info', str(root / 'road_info.json')])
        if no_yolo:
            argv.append('--no-yolo')
        argv.extend(args or [])
        self.log = tempfile.TemporaryFile()
        try:
            self.process = subprocess.Popen(argv, cwd=str(root), env=child_env, stdin=subprocess.DEVNULL,
                                            stdout=self.log, stderr=self.log if stderr is None else stderr,
                                            start_new_session=True)
        except BaseException:
            self.log.close()
            raise

    def events(self):
        events = td.trace(self.path)
        cap(events)
        return events

    def of(self, kind, mark=0):
        return [e for e in self.events()[mark:] if e[1] == kind]

    def boot(self):
        self.wait(lambda: self.of('ready'), 10, 'Thiếu ready auto')
        events = self.events()
        need(len(self.of('ready')) == 1, 'ready không đúng một lần')
        need(self.of('frame') and self.of('mask') and self.session_files(), 'ready trước ảnh/mask/log')
        td.stop_pair(events, 'startup_stop')
        helper = next((i for i, e in enumerate(events) if e[1] == 'test_ucr_stop'), None)
        init = next((i for i, e in enumerate(events) if e[1] == 'motor_init'), None)
        need(helper is not None and init is not None and helper < init, 'Helper đầu phải trước motor_init')
        need(any(str(e[2]) == 'CHO_SAN_SANG' for e in self.of('state')), 'Thiếu trạng thái chờ')

    def session_files(self):
        return list(self.out.glob('*/session.json'))

    def session(self):
        files = self.session_files()
        need(len(files) == 1, 'Phải có đúng một thư mục phiên', 'record_format')
        return files[0].parent

    def rows(self):
        files = list(self.out.glob('*/control_log.csv'))
        if not files:
            return []
        lines = [s for s in files[0].read_text(encoding='utf-8').splitlines(True) if s.endswith('\n')]
        if not lines:
            return []
        reader = csv.DictReader(lines)
        need(set(CSV_FIELDS) <= set(reader.fieldnames or []), 'CSV thiếu cột B5: ' + repr(reader.fieldnames), 'record_format')
        rows = list(reader)
        need(all(None not in row and None not in row.values() for row in rows), 'CSV thiếu/thừa ô', 'record_format')
        return rows

    def inject(self, command):
        self.seq += 1
        mark = len(self.events())
        value = '{} {}'.format(self.seq, command)
        with self.ctrl.open('a', encoding='utf-8') as stream:
            stream.write(value + '\n')
        return self.wait(lambda: next((e for e in self.of('inject', mark) if e[2] == value), None),
                         message='Không xác nhận CTRL ' + value)

    def ack(self, key, mark):
        return super().ack({'\x03': '3', '\x1b': 'E'}.get(key, key), mark)

    def key(self, key):
        mark = len(self.events())
        self.send(key)
        return self.ack(key, mark)

    def go(self):
        mark = len(self.events())
        self.key('G')
        return self.wait(lambda: next((e for e in self.events()[mark:]
                                      if e[1] == 'motor_speed' and e[2] > 0 and not e[3]), None),
                         message='G không tạo chuyển động')

    def frames(self, count=5, check=lambda: None):
        start = len(self.of('frame'))
        def enough():
            check()
            return len(self.of('frame')) >= start + count
        self.wait(enough, 10, 'Không có thêm frame')

    def finish(self, code=0, key='E'):
        event = self.key(key)
        stopped(self, event[0], 150, 'exit_stop')
        self.exited(code)

    def exited(self, code=0):
        self.wait(lambda: self.process.poll() is not None, 12, 'Không thoát hữu hạn', alive=False)
        need(self.process.returncode == code, 'Sai exit {} != {}: {}'.format(self.process.returncode, code, self.text()), 'exit_code')
        events = self.events()
        helpers = self.of('test_ucr_stop')
        need(len(helpers) >= 2 and td.last_commands(events) == (0, 0), 'Thiếu helper cuối/dừng cuối')
        last_helper = max(i for i, e in enumerate(events) if e[1] == 'test_ucr_stop')
        last_motor = max(i for i, e in enumerate(events) if motor_zero(e))
        last_servo = max(i for i, e in enumerate(events) if e[1] == 'servo_angle' and e[2] == 0 and not e[3])
        need(last_motor < last_servo < last_helper, 'Thứ tự cuối phải motor0/servo0/helper')


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
        try:
            run.events()  # A33 áp dụng cả ca lỗi nạp/thoát sớm.
        finally:
            run.close()


def external_stamp_ms():
    # Vết hooks cắt phần lẻ ms; mốc tác nhân ngoài phải cùng độ phân giải.
    return math.floor(td.now_ms())


def motor_zero(e):
    return not e[3] and (e[1] == 'motor_emergency' or (e[1] == 'motor_speed' and e[2] == 0))


def no_motion(run, stamp, tag='latched'):
    need(not any(e[0] >= stamp and e[1] in ('motor_speed', 'servo_angle') and e[2] != 0 and not e[3]
                 for e in run.events()), 'Có motor/servo khác0 sau mốc dừng', tag)


def stopping_pair(events, stamp, tag='stop_order'):
    es = [e for e in events if e[1] in ('motor_speed', 'motor_emergency', 'servo_angle')]
    motors = [i for i, e in enumerate(es) if e[0] >= stamp and motor_zero(e)]
    if not motors:
        return None
    first = motors[0]
    # Giữ lệnh trước mốc để không nhầm servo 0 của cặp chạy thẳng với lệnh dừng.
    zeros = [i for i, e in enumerate(es) if e[0] >= stamp and e[1] == 'servo_angle' and e[2] == 0 and not e[3]
             and not (i > 0 and es[i - 1][1] == 'motor_speed' and es[i - 1][2] != 0)]
    if not zeros:
        return None
    need(first < zeros[0], 'Servo0 trước motor0 sau tác nhân', tag)
    return es[first], es[zeros[0]]


def stopped(run, stamp, limit, tag='stop_order', after=None):
    """A27: cắt sau tác nhân, kiểm motor thực/thứ tự; S chỉ cộng đúng một lần."""
    def pair():
        return stopping_pair(run.events(), stamp if after is None else after, tag)
    result = run.wait(pair, (limit + run.slack) / 1000.0 + 1, 'Thiếu cặp dừng', alive=False, tag=tag)
    need(0 <= result[0][0] - stamp <= limit + run.slack,
         'Motor0 trễ {} ms, trần {}'.format(result[0][0] - stamp, limit + run.slack), tag)
    return result[1][0]


def space(run):
    event = run.key(' ')
    return stopped(run, event[0], 150)


def fresh_recovery(run, stamp, kind, limit, age, reason=None):
    end = stopped(run, stamp, limit, after=stamp + age)
    if reason:
        run.wait(lambda: any(e[0] >= stamp and e[2] == reason for e in run.of('reason')),
                 message='Thiếu reason ' + reason)
    before = len(run.of(kind))
    masks_before = len(run.of('mask'))
    run.frames(4, lambda: no_motion(run, end, 'stale_motion'))
    need(len(run.of(kind)) == before, 'Thiết lập stall vẫn sinh dữ liệu mới', 'setup')
    if kind == 'command':
        need(len(run.of('mask')) > masks_before, 'Model bị chặn cùng controller', 'setup')
    mark = len(run.events())
    run.inject('road_resume' if kind == 'mask' else 'controller_resume')
    new = run.wait(lambda: run.of(kind, mark), message='Không có dữ liệu phục hồi')[0]
    need(float(str(new[2]).split()[1]) > stamp, 'Phục hồi dùng timestamp dữ liệu cũ', 'stale_motion')
    no_motion_until = [e for e in run.events() if end <= e[0] < new[0]]
    need(not any(e[1] == 'motor_speed' and e[2] != 0 and not e[3] for e in no_motion_until),
         'Chạy trước dữ liệu mới', 'stale_motion')
    run.wait(lambda: any(e[0] >= new[0] and e[1] == 'motor_speed' and e[2] > 0 and not e[3]
                         for e in run.events()), message='Không tự phục hồi (L7)')


def reject(run, code=None, before_init=True):
    run.wait(lambda: run.process.poll() is not None, 12, 'Không từ chối đầu vào lỗi', alive=False)
    need(run.process.returncode != 0 if code is None else run.process.returncode == code,
         'Sai mã từ chối: ' + str(run.process.returncode))
    no_motion(run, 0, 'reject_motion')
    need(not run.of('ready'), 'ready khi boot lỗi')
    if before_init:
        need(not run.of('motor_init'), 'Lỗi đầu vào phải trước motor_init')
    need(run.text().strip(), 'Từ chối không nêu lỗi')


def patch_yaml(run, values):
    run.control.write_text(values if isinstance(values, str) else yaml_text(values), encoding='utf-8')


def reload(run, expected):
    mark = len(run.events())
    event = run.key('R')
    run.wait(lambda: any(e[2] == expected for e in run.of('reload', mark)), message='R thiếu ' + expected)
    return event[0]


def speed_is(run, speed):
    mark = len(run.events())
    run.frames(6)
    sent = [e[2] for e in run.of('motor_speed', mark) if not e[3]]
    need(sent and all(abs(v - speed) < 1e-6 for v in sent), 'Tốc độ không dùng cruise đã chọn: ' + repr(sent))


def record(run):
    rows = run.of('record')
    need(rows, 'Thiếu trace record')
    values = str(rows[-1][2]).split()
    need(len(values) >= 3, 'record thiếu every_n dropped free_mb')
    return int(values[0]), int(values[1]), float(values[2])


def counter(run, name):
    values = re.findall(r'\b' + re.escape(name) + r'\s*[=:]\s*(\d+)', run.text())
    if name == 'csv_dropped':
        values += [r[name] for r in run.rows()]
    return max([int(v) for v in values] or [0])


def inspect_session(run, sha256_failed=False):
    folder = run.session()
    data = json.loads((folder / 'session.json').read_text(encoding='utf-8'))
    for name in ('git_hash', 'git_branch', 'control_yaml', 'control_sha256', 'info_json', 'engine_name', 'engine_sha256'):
        need(name in data, 'session thiếu ' + name, 'record_format')
    if any((parent / '.git').exists() for parent in (HERE,) + tuple(HERE.parents)):
        need(re.fullmatch(r'[0-9a-fA-F]{7,40}', data['git_hash']) and data['git_branch']
             and data['git_branch'] != 'unknown', 'Thiếu revision/nhánh', 'record_format')
    else:
        need(data['git_hash'] == data['git_branch'] == 'unknown',
             'Bản không có .git phải ghi unknown cho revision/nhánh', 'record_format')
    raw = run.control.read_bytes()
    need(data['control_yaml'].encode('utf-8') == raw, 'Bản chép yaml sai', 'record_format')
    if sha256_failed:
        need(data['control_sha256'] in ('loi', ''), 'Băm lỗi phải là loi hoặc rỗng', 'record_format')
    else:
        need(data['control_sha256'] == hashlib.sha256(raw).hexdigest(), 'Băm yaml sai', 'record_format')
    need(data['info_json'] == json.loads((run.root / 'road_info.json').read_text(encoding='utf-8')),
         'Metadata session sai', 'record_format')
    rows = run.rows()
    need(rows, 'CSV không có dữ liệu', 'record_format')
    need(all(float(a['t_ms']) <= float(b['t_ms']) for a, b in zip(rows, rows[1:])), 'CSV đảo thời gian')
    for row in rows:
        for name in ('t_ms', 'e', 'p', 'd', 'requested_steer_deg', 'requested_speed_cms',
                     'sent_steer_deg', 'sent_speed_cms', 'road_ms', 'yolo_ms', 'loop_ms', 'dropped', 'csv_dropped'):
            need(math.isfinite(float(row[name])), 'CSV NaN/Inf: ' + name)
        for name in ('image_age_ms', 'mask_age_ms', 'det_age_ms'):
            need(math.isfinite(float(row[name])), 'Tuổi CSV không hữu hạn')
        scan = json.loads(row['scan_rows'])
        if int(row['frame_id']) > 0:
            need(len(scan) == len(run.values.get('scan_rows', VECTORS['scan_rows'])), 'CSV thiếu hàng quét')
            need(all(set(('y_frac', 'left_frac', 'right_frac', 'center_frac', 'valid')) <= set(s) for s in scan),
                 'CSV thiếu biên/tâm từng hàng')
            for s in scan:
                need(isinstance(s['valid'], bool), 'Cờ hàng quét phải là boolean JSON')
                for name in ('y_frac', 'left_frac', 'right_frac', 'center_frac'):
                    if s[name] is not None:
                        need(math.isfinite(float(s[name])) and 0 <= float(s[name]) <= 1, 'Tỉ lệ hàng quét sai')
        need(abs(float(row['sent_speed_cms'])) <= 10 and abs(float(row['sent_steer_deg'])) <= 40, 'CSV vượt trần')
        frame_id = int(row['frame_id'])
        if frame_id > 0:
            need(any(int(str(e[2]).split()[0]) == frame_id for e in run.of('frame')), 'CSV frame không có trace')
    return rows


def inspect_images(run, every=5):
    folder = run.session()
    jpgs, masks = list(folder.rglob('*.jpg')), list(folder.rglob('*.png'))
    need(jpgs and masks, 'Thiếu JPEG/mặt nạ', 'record_format')
    # Tên ảnh là frame_id, cùng stem cho JPEG/PNG: hợp đồng bổ sung ghi trong báo cáo.
    need({p.stem for p in jpgs} == {p.stem for p in masks}, 'JPEG/mask không cùng khung')
    ids = sorted(int(p.stem) for p in masks)
    need(all(i % every == 0 for i in ids), 'Sai nhịp record_every_n')
    for p in jpgs:
        data = p.read_bytes()
        marker = data.find(b'\xff\xc0')
        need(data.startswith(b'\xff\xd8') and data.endswith(b'\xff\xd9') and marker >= 0, 'JPEG hỏng')
        need(struct.unpack('>HH', data[marker + 5:marker + 9]) == (720, 1280), 'JPEG sai cỡ')
    # Giải mã cả 5 bộ lọc PNG để kiểm ảnh OpenCV ghi, không thêm dependency.
    for p in masks:
        pixels = read_png(p.read_bytes())
        need(set(pixels) <= {0, 1, 2}, 'PNG không phải chỉ số lớp')
        original = mask_rows('straight')
        need(pixels == b''.join(bytes(r) for r in original), 'Mặt nạ ghi khác fixture')
    need(not list(folder.rglob('*.tmp')), 'Còn file ghi dở')


def read_png(data):
    need(data[:8] == b'\x89PNG\r\n\x1a\n', 'PNG sai chữ ký')
    pos, payload, header = 8, b'', None
    while pos < len(data):
        n = struct.unpack('>I', data[pos:pos + 4])[0]
        kind, body = data[pos + 4:pos + 8], data[pos + 8:pos + 8 + n]
        need(zlib.crc32(kind + body) & 0xffffffff == struct.unpack('>I', data[pos + 8 + n:pos + 12 + n])[0], 'PNG sai CRC')
        if kind == b'IHDR':
            header = struct.unpack('>IIBBBBB', body)
        if kind == b'IDAT':
            payload += body
        pos += 12 + n
    need(header == (160, 96, 8, 0, 0, 0, 0), 'PNG phải CV_8UC1 96x160 không interlace')
    raw, out, prev = zlib.decompress(payload), bytearray(), bytearray(160)
    need(len(raw) == 96 * 161, 'PNG thiếu/thừa dữ liệu')
    for y in range(96):
        filt, row = raw[y * 161], bytearray(raw[y * 161 + 1:(y + 1) * 161])
        need(filt <= 4, 'PNG filter không hợp lệ')
        for x in range(160):
            a, b, c = row[x - 1] if x else 0, prev[x], prev[x - 1] if x else 0
            p = a + b - c
            pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
            paeth = a if pa <= pb and pa <= pc else b if pb <= pc else c
            row[x] = (row[x] + (0, a, b, (a + b) // 2, paeth)[filt]) & 255
        out.extend(row)
        prev = row
    return bytes(out)


def check_gitignore():
    # T47m LOW-1: bản rsync không có kho Git; cô lập exclude cá nhân trong kho tạm.
    git = shutil.which('git')
    need(git is not None, 'Không tìm được lệnh git', 'environment')
    with tempfile.TemporaryDirectory(prefix='ucr-t47-gitignore-') as tmp:
        repo_root = Path(tmp)
        env = dict(os.environ, GIT_CONFIG_NOSYSTEM='1', HOME=tmp)
        result = subprocess.run([git, 'init', '-q'], cwd=tmp, env=env,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=10)
        need(result.returncode == 0, 'Không tạo được kho Git tạm: ' + result.stderr, 'environment')
        shutil.copy2(str(HERE.parents[1] / '.gitignore'), str(repo_root / '.gitignore'))
        result = subprocess.run([git, '-c', 'core.excludesFile=/dev/null', 'check-ignore',
                                 '--no-index', '--verbose', '--', 'runs_auto/T47m/session.json'], cwd=tmp, env=env,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=10)
        need(result.returncode == 0, 'runs_auto chưa được bỏ qua: ' + result.stderr, 'gitignore')
        source, _, pattern = result.stdout.split('\t', 1)[0].rsplit(':', 2)
        need((repo_root / source).resolve() == (repo_root / '.gitignore').resolve() and not pattern.startswith('!'),
             'Quy tắc phải bỏ qua đường dẫn và thuộc bản chép ucr_car/.gitignore: ' + result.stdout, 'gitignore')


def test_case(case, cfg, root):
    # A01 — H3/N2: chờ G, ready duy nhất, không chuyển động khi chưa có phím.
    if case == 'A01':
        with running(cfg, root) as r:
            r.boot()
            r.frames(12, lambda: no_motion(r, 0))
            need(len(r.of('ready')) == 1, 'ready lặp')
            r.finish()
        return
    # A02 — B4/L5: Space chốt dừng trong 150+S ms, G mới cho chạy lại.
    if case == 'A02':
        with running(cfg, root) as r:
            r.boot(); r.go()
            end = space(r)
            r.frames(8, lambda: no_motion(r, end))
            r.go(); r.finish()
        return
    # A03/A04/A05 — B1/H3: nạp nguyên tử, sai giữ bản tốt, đang chạy bỏ R.
    if case in ('A03', 'A04', 'A05'):
        old, new = {'cruise_cms': 6, 'slow_per_deg': 0}, {'cruise_cms': 9, 'slow_per_deg': 0}
        with running(cfg, root, values=old) as r:
            r.boot(); r.go(); speed_is(r, 6)
            if case == 'A03':
                patch_yaml(r, new); reload(r, 'ignored')
                space(r); r.go(); speed_is(r, 6)
            else:
                space(r)
                patch_yaml(r, new); stamp = reload(r, 'accepted')
                r.frames(5, lambda: no_motion(r, stamp))
                r.go(); speed_is(r, 9); space(r)
                if case == 'A05':
                    for bad in ('cruise_cms: [\n', {'unknown': 1}, {'kp': 201},
                                {'min_cms': 9, 'cruise_cms': 8}, {'lost_hold_ms': 900, 'lost_stop_ms': 800}):
                        patch_yaml(r, bad); stamp = reload(r, 'error')
                        r.frames(4, lambda: no_motion(r, stamp))
                        r.go(); speed_is(r, 9); space(r)
            r.finish()
        return
    # A06 — L1: thiếu --no-yolo phải exit2 trước thiết bị.
    if case == 'A06':
        with running(cfg, root, no_yolo=False) as r:
            reject(r, 2)
            need('YOLO chưa hỗ trợ, dùng --no-yolo' in r.text(), 'Thiếu hướng dẫn YOLO P1')
            need(not r.of('motor_speed') and not r.of('servo_angle'), 'Mở thiết bị trước kiểm CLI')
        return
    # A07 — L2: tất cả khóa scalar có sai kiểu/ngoài hai biên; vector và quan hệ chéo.
    if case == 'A07':
        bad = []
        for key, (_, lo, hi, _) in BOUNDS.items():
            bad.extend({key: v} for v in ('sai_kieu', lo - 1, hi + 1))
        for key in VECTORS:
            bad.extend(({key: []}, {key: [0.5] * 6}, {key: 'sai_kieu'}, {key: [-0.1, 0.5, 0.5]}))
        bad += [{'scan_rows': [0, 0.65, 0.45]}, {'scan_rows': [1, 0.65, 0.45]},
                {'lane_width_frac': [0, 0.55, 0.4]}, {'lane_width_frac': [1.01, 0.55, 0.4]},
                {'row_weights': [0, 0, 0]}, {'row_weights': [1]}, {'lane_width_frac': [0.5]},
                {'scan_rows': [0.5]}, {'ki': 1}, {'lane_width_px': 100}, {'center_offset_px': 0},
                {'unknown': 1}, {'max_speed_cms': 7}, {'min_cms': 9}, {'lost_cms': 9, 'max_speed_cms': 8},
                {'lost_hold_ms': 900}, {'min_valid_rows': 4}]
        for i, values in enumerate(bad):
            with running(cfg, root / str(i), values=values) as r:
                reject(r)
        with running(cfg, root / 'syntax', setup=lambda r: patch_yaml(r, 'kp: [\n')) as r:
            reject(r)
        return
    # A08 — B1/L2/B5: mặc định, biên hợp lệ và bản chép/băm cấu hình.
    if case == 'A08':
        variants = [{}]
        for key, (_, lo, hi, _) in BOUNDS.items():
            for bound in (lo, hi):
                values = {key: bound}
                if key in ('max_speed_cms', 'cruise_cms', 'min_cms', 'lost_cms'):
                    values.update(max_speed_cms=10, cruise_cms=bound, min_cms=0, lost_cms=0)
                    values[key] = bound
                    if key == 'max_speed_cms':
                        values.update(cruise_cms=0)
                if key == 'lost_hold_ms':
                    values['lost_stop_ms'] = max(800, bound)
                if key == 'lost_stop_ms':
                    values['lost_hold_ms'] = 0
                if key == 'min_valid_rows':
                    values.update(scan_rows=[0.2, 0.35, 0.5, 0.65, 0.85], row_weights=[1] * 5, lane_width_frac=[0.5] * 5)
                variants.append(values)
        variants += [dict(max_speed_cms=10, steer_max_deg=40),
                     dict(scan_rows=[0.5], row_weights=[1], lane_width_frac=[1]),
                     dict(scan_rows=[0.1, 0.3, 0.5, 0.7, 0.9], row_weights=[0, 0, 0, 0, 1], lane_width_frac=[1] * 5)]
        for i, values in enumerate(variants):
            with running(cfg, root / str(i), values=values) as r:
                r.boot(); r.frames(3); r.finish(); inspect_session(r)
                if not values:
                    for key, (default, _, _, unit) in BOUNDS.items():
                        lines = [l for l in r.text().splitlines() if re.search(r'\b' + key + r'\b', l)]
                        need(any('(mặc định)' in l and re.search(r'\b' + key + r'\b\s*[:=|]?\s*' + re.escape(str(default)) + r'(?![\d.])', l)
                                 and unit in l for l in lines), 'Bảng mặc định thiếu/sai ' + key)
                        if key in ('steer_max_deg', 'kp', 'kd'):
                            need(any('°' in l or 'độ' in l for l in lines), 'Thiếu đơn vị góc ' + key)
                    for key, value in VECTORS.items():
                        lines = [l for l in r.text().splitlines() if key in l and '(mặc định)' in l]
                        need(any(all(str(v) in l for v in value) for l in lines), 'Thiếu mặc định vector ' + key)
        return
    # A09 — H7: lỗi nạp model không sẵn sàng, không quay motor.
    if case == 'A09':
        with running(cfg, root, env={'MODEL_LOAD_FAIL': 'road'}) as r:
            reject(r, before_init=False)
        return
    # A10 — N2/L1: hỏng fixture/metadata phải từ chối trước motor_init.
    if case == 'A10':
        setups = [lambda r: (r.root / 'straight.png').unlink(),
                  lambda r: (r.root / 'straight.png').write_bytes(b'broken'),
                  lambda r: (r.root / 'list.csv').unlink(),
                  lambda r: (r.root / 'list.csv').write_text('frame_id,mask\n2,straight.png\n', encoding='utf-8'),
                  lambda r: (r.root / 'road_info.json').unlink(),
                  lambda r: (r.root / 'road_info.json').write_text('{}', encoding='utf-8'),
                  lambda r: (r.root / 'road_info.json').write_text('{', encoding='utf-8'),
                  lambda r: (r.root / 'straight.png').write_bytes(png_bytes([bytearray([3] * 160)] * 96))]
        for i, setup in enumerate(setups):
            with running(cfg, root / str(i), setup=setup) as r:
                reject(r)
        with running(cfg, root / 'no_info_option', info=False) as r:
            reject(r)
        return
    # A11 — B4/L5/L7, M6: ảnh về vẫn chốt tới G mới.
    if case == 'A11':
        with running(cfg, root, values={'mask_max_age_ms': 1000}) as r:
            r.boot(); r.go()
            ev = r.inject('camera_pause_600ms')
            end = stopped(r, ev[0], 700)
            r.wait(lambda: any(e[0] >= ev[0] and e[2] == 'MAT_ANH' for e in r.of('reason')), message='Thiếu MAT_ANH')
            r.wait(lambda: any(e[0] > end and float(str(e[2]).split()[1]) > ev[0] + 500 for e in r.of('mask')),
                   message='Chưa có mask mới sau pause')
            r.frames(8, lambda: no_motion(r, end, 'image_latch'))
            r.go(); r.finish()
        return
    # A12/A13 — B4/L4/L7, M2: mask cũ dừng; chỉ tự chạy SAU mask mới.
    if case in ('A12', 'A13'):
        with running(cfg, root, values={'mask_max_age_ms': 100, 'control_period_ms': 20}) as r:
            r.boot(); r.go()
            if case == 'A13':
                r.inject('road_throw')
            r.inject('road_stall')
            last = r.of('mask')[-1]
            captured = float(str(last[2]).split()[1])
            fresh_recovery(r, captured, 'mask', 100 + 20 + 200, 100, 'MAT_NAN_CU')
            r.finish()
        return
    # A14 — B4/L5/L7, M5: độc lập controller treo, tự chạy lại khi lệnh mới.
    if case == 'A14':
        with running(cfg, root, values={'cmd_max_age_ms': 50}) as r:
            r.boot(); r.go(); r.inject('controller_stall')
            created = float(str(r.of('command')[-1][2]).split()[1])
            masks = len(r.of('mask'))
            fresh_recovery(r, created, 'command', 50 + 200, 50)
            need(len(r.of('mask')) > masks, 'Model không còn chạy trong controller_stall')
            r.finish()
        return
    # A15 — B4/L5/L7, M3: ngoại lệ chốt; không lấy reason thay bằng chứng motor.
    if case == 'A15':
        with running(cfg, root) as r:
            r.boot(); r.go(); ev = r.inject('controller_throw')
            end = stopped(r, ev[0], 150, 'exception_stop')
            r.wait(lambda: any(e[0] >= ev[0] and e[2] == 'LOI_DIEU_KHIEN' for e in r.of('reason')),
                   message='Thiếu LOI_DIEU_KHIEN')
            r.frames(8, lambda: no_motion(r, end))
            r.go(); r.finish()
        return
    # A16/A35 — L6/bản duyệt: đúng ±10/±40 cả khi yaml thấp hơn; CSV giữ yêu cầu trước kẹp.
    if case in ('A16', 'A35'):
        for low in (False, True):
            values = dict(max_speed_cms=5, cruise_cms=5, min_cms=0, steer_max_deg=10) if low else {}
            with running(cfg, root / str(low), values=values) as r:
                r.boot(); r.go()
                for sign in (1, -1):
                    ev = r.inject('controller_command {} {}'.format(sign * 50, sign * 90))
                    def sent():
                        es = [e for e in r.events() if e[0] >= ev[0] and not e[3]]
                        return (any(e[1] == 'motor_speed' and e[2] == sign * 10 for e in es) and
                                any(e[1] == 'servo_angle' and e[2] == sign * 40 for e in es))
                    r.wait(sent, message='Không gửi đúng biên kẹp từ lệnh tiêm', tag='hard_cap')
                    r.wait(lambda: any(float(row['requested_speed_cms']) == sign * 50 and
                                       float(row['requested_steer_deg']) == sign * 90 and
                                       float(row['sent_speed_cms']) == sign * 10 and
                                       float(row['sent_steer_deg']) == sign * 40 for row in r.rows()),
                           message='CSV không giữ yêu cầu 50/90 và đã gửi 10/40')
                r.finish()
        return
    # A17 — B5/L4/L5, M4: bão hoà 8 ảnh và 256 CSV, Space vẫn đúng hạn.
    if case == 'A17':
        with running(cfg, root, values={'record_every_n': 1, 'control_period_ms': 20}) as r:
            r.boot(); r.go(); ev = r.inject('writer_stall')
            r.wait(lambda: any(e[0] >= ev[0] for e in r.of('writer_stalled')), message='Writer chưa treo')
            mark = len(r.events())
            r.wait(lambda: len(r.of('frame', mark)) >= 9 and len(r.of('command', mark)) > 256,
                   25, 'Chưa bão hoà hai hàng đợi')
            r.wait(lambda: record(r)[1] > 0 and counter(r, 'csv_dropped') > 0,
                   5, 'Không đếm ảnh/CSV bỏ khi hàng đợi đầy')
            need(any(e[1] == 'motor_speed' and e[2] > 0 and not e[3] for e in r.events()[mark:]),
                 'Không điều khiển khi writer treo')
            end = space(r)
            r.frames(4, lambda: no_motion(r, end))
            r.inject('writer_resume'); r.finish()
        return
    # A18/A19 — B5/L4: ranh 1024 MiB; CSV còn chạy khi ngừng ảnh.
    if case in ('A18', 'A19'):
        for free in ((1023, 1024) if case == 'A18' else (4096,)):
            with running(cfg, root / str(free), env={'FREE_MB': str(free)}, values={'record_every_n': 1}) as r:
                r.boot(); r.go()
                if case == 'A19':
                    r.wait(lambda: list(r.out.rglob('*.jpg')) and list(r.out.rglob('*.png')), message='Chưa ghi ảnh đầu')
                    ev = r.inject('free_mb 1023')
                    r.wait(lambda: any(e[0] >= ev[0] and float(str(e[2]).split()[2]) == 1023 for e in r.of('record')),
                           12, 'Không kiểm lại đĩa mỗi 10 s')
                    # Xả tối đa 8 ảnh đã xếp trước mốc; không cho ảnh của frame sau mốc kiểm đĩa.
                    cutoff = int(str(r.of('frame')[-1][2]).split()[0])
                    rows_before = len(r.rows())
                    r.frames(40)
                    need(all(int(p.stem) <= cutoff for pattern in ('*.jpg', '*.png') for p in r.out.rglob(pattern)),
                         'Vẫn nhận ảnh sau đĩa thấp')
                    need(len(r.rows()) > rows_before, 'Đĩa thấp làm dừng CSV')
                    need(td.last_commands(r.events())[0] > 0, 'Đĩa thấp làm dừng xe')
                else:
                    r.frames(20)
                if case == 'A19':
                    space(r)
                r.finish()
                need(r.rows(), 'Đĩa thấp mất CSV')
                if free == 1023:
                    need(not list(r.out.rglob('*.jpg')) and not list(r.out.rglob('*.png')), '1023 MiB vẫn ghi ảnh')
                    need(re.search(r'(?i)warn|canh bao|cảnh báo', r.text()), 'Thiếu cảnh báo đĩa thấp')
                elif free == 1024:
                    inspect_images(r, 1)
        return
    # A20 — H10/L4: lỗi ghi một lần không dừng, có đếm/cảnh báo và phục hồi ghi.
    if case == 'A20':
        for command in ('jpeg_fail', 'csv_fail'):
            with running(cfg, root / command, values={'record_every_n': 1}) as r:
                r.boot(); r.go(); r.wait(lambda: r.rows(), message='Chưa ghi trước lỗi')
                before = counter(r, command)
                ev = r.inject(command)
                r.wait(lambda: counter(r, command) > before, message='Không đếm lỗi ' + command)
                need(re.search(r'(?i)warn|canh bao|cảnh báo', r.text()), 'Không cảnh báo lỗi ghi')
                nrows, nimages = len(r.rows()), len(list(r.out.rglob('*.jpg')))
                r.frames(12)
                need(len(r.rows()) > nrows and len(list(r.out.rglob('*.jpg'))) > nimages, 'Ghi không phục hồi sau lỗi một lần')
                need(any(e[0] >= ev[0] and e[1] == 'motor_speed' and e[2] > 0 and not e[3] for e in r.events()), 'Lỗi ghi làm ngừng xe')
                space(r); r.finish()
        return
    # A21 — B5/L4: hồ sơ, CSV đối chiếu trace, JPEG và mặt nạ cùng nhịp.
    if case == 'A21':
        with running(cfg, root, values={'record_every_n': 5, 'slow_per_deg': 0}) as r:
            r.boot(); r.go(); r.frames(25); space(r); r.finish()
            rows = inspect_session(r)
            inspect_images(r)
            for row in rows:
                if float(row['sent_speed_cms']) > 0:
                    need(any(e[1] == 'motor_speed' and not e[3] and e[2] == float(row['sent_speed_cms'])
                             and abs(e[0] - float(row['t_ms'])) <= 50 for e in r.events()), 'CSV tốc độ không có lệnh gửi tương ứng')
                    need(any(e[1] == 'servo_angle' and not e[3] and abs(e[2] - float(row['sent_steer_deg'])) < 1e-6
                             and abs(e[0] - float(row['t_ms'])) <= 50 for e in r.events()), 'CSV góc không có lệnh gửi tương ứng')
        return
    # A22 — N2: chủ khoá không bị helper/motor của tiến trình thứ hai tác động.
    if case == 'A22':
        import fcntl
        root.mkdir(parents=True)
        with (root / 'vehicle.lock').open('w') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with running(cfg, root / 'contender', lock=root / 'vehicle.lock') as r:
                reject(r, 6)
                need(not any(e[1] in ('test_ucr_stop', 'motor_speed', 'motor_emergency', 'servo_angle') for e in r.events()),
                     'Tiến trình tranh khoá tác động phần cứng')
        return
    # A23/A25 — L5/L7: tín hiệu thật 1000+S ms, byte Ctrl+C 150+S ms.
    if case in ('A23', 'A25'):
        modes = ('term', 'term_writer') if case == 'A23' else ('byte', 'int')
        for mode in modes:
            with running(cfg, root / mode) as r:
                r.boot(); r.go()
                if mode == 'term_writer':
                    r.inject('writer_stall')
                    r.wait(lambda: r.of('writer_stalled'), message='Writer chưa treo')
                if mode == 'byte':
                    ev = r.key('\x03')
                    stamp, limit = ev[0], 150
                else:
                    stamp, limit = external_stamp_ms(), 1000
                    os.kill(r.process.pid, signal.SIGINT if mode == 'int' else signal.SIGTERM)
                stopped(r, stamp, limit)
                r.exited()
        return
    # A24 — H3/L5: Esc/E dừng trước helper rồi thoát.
    if case == 'A24':
        for key in ('E', '\x1b'):
            with running(cfg, root / str(ord(key))) as r:
                r.boot(); r.go(); r.finish(key=key)
        return
    # A26 — L1/L7: giết đúng preview chính sau khi xe đã chạy, exit1 trong hạn motor.
    if case == 'A26':
        with running(cfg, root) as r:
            r.boot(); r.go()
            windows = sorted(pid for pid in r.children() if td.nice(pid) == 10)
            need(windows, 'Không nhận diện được preview chính qua nice10', 'interface')
            stamp = external_stamp_ms()
            os.kill(windows[0], signal.SIGKILL)  # Như T42: preview chính fork trước các cửa sổ phụ.
            stopped(r, stamp, 1000)
            r.exited(1)
        return
    # A27 — B4/L5, M7: cùng kiểm thứ tự trên các đường dừng đại diện, có mốc mới.
    if case == 'A27':
        for probe in ('A02', 'A11', 'A15', 'A23', 'A24', 'A25', 'A26'):
            test_case(probe, cfg, root / probe)
        return
    # A28 — L3: phân đoạn theo thời gian gốc, không cộng S vào hold/stop/regain.
    if case == 'A28':
        values = dict(lost_hold_ms=500, lost_stop_ms=1400, regain_frames=3, slow_per_deg=0,
                      min_cms=0, kd=0, control_period_ms=20)
        with running(cfg, root, values=values, schedule=((1, 'left'), (120, 'empty'), (300, 'right'))) as r:
            r.boot(); r.go()
            r.wait(lambda: any(int(str(e[2]).split()[0]) >= 119 for e in r.of('mask')), 15, 'Chưa đến trước mất làn')
            r.wait(lambda: any(e[2] == 'MAT_LAN' for e in r.of('reason')), 15, 'Không vào MAT_LAN trước phục hồi')
            need(int(str(r.of('mask')[-1][2]).split()[0]) < 300, 'Fixture không đủ thời gian mất làn', 'setup')
            r.wait(lambda: any(int(str(e[2]).split()[0]) >= 305 for e in r.of('mask')), 20, 'Chưa phục hồi làn')
            rows = r.rows()
            valid = [e for e in r.of('mask') if int(str(e[2]).split()[0]) < 120]
            need(valid, 'Thiếu mask hợp lệ cuối')
            last_valid = float(str(valid[-1][2]).split()[1])
            previous = [row for row in rows if 0 < int(row['frame_id']) < 120 and float(row['sent_speed_cms']) > 0]
            need(previous, 'Chưa chạy lệch tâm trước mất làn')
            angle = float(previous[-1]['sent_steer_deg'])
            need(angle > 0, 'Tiền đề cần góc trái khác 0')
            seen = set()
            for row in rows:
                frame = int(row['frame_id'])
                if not 120 <= frame < 300:
                    continue
                age = float(row['t_ms']) - last_valid
                speed, steer = float(row['sent_speed_cms']), float(row['sent_steer_deg'])
                if age < 500:
                    seen.add('hold'); need(speed == 4 and abs(steer - angle) < 1e-6, 'Sai giai đoạn giữ lost_cms')
                elif age < 1400:
                    seen.add('zero'); need(speed == 0 and abs(steer - angle) < 1e-6, 'Sai giai đoạn giữ góc/tốc độ0')
                else:
                    seen.add('lost'); need(speed == 0 and steer == 0 and row['planner_state'] == 'MAT_LAN', 'Sai MAT_LAN')
            need(seen == {'hold', 'zero', 'lost'}, 'Chưa quan sát đủ ba miền L3', 'setup')
            new = [e for e in r.of('mask') if int(str(e[2]).split()[0]) >= 300]
            need(len(new) >= 3, 'Thiếu 3 mask phục hồi')
            first, third = new[0][0], new[2][0]
            need(not any(e[1] == 'motor_speed' and e[2] != 0 and not e[3] and first <= e[0] < third
                         for e in r.events()), 'Chạy trước regain_frames liên tiếp')
            r.wait(lambda: any(e[1] == 'motor_speed' and e[2] > 0 and not e[3] and e[0] >= third for e in r.events()),
                   message='MAT_LAN không tự phục hồi')
            # Chuyển sang MAT_LAN: kiểm thứ tự từ mốc hết lost_stop, không dùng số0 boot.
            stopped(r, last_valid + 1400, 20 + 200)
            r.finish()
        return
    # A29 — B5/bản duyệt: chỉ kiểm định dạng/thông số hữu hạn, không đặt dung sai nhịp 2s.
    if case == 'A29':
        with running(cfg, root) as r:
            r.boot(); r.go()
            def stats():
                return [l for l in r.text().splitlines() if all(re.search(r'\b' + k + r'\s*[=:]', l) for k in STATS)]
            r.wait(lambda: len(stats()) >= 3, 10, 'Thiếu 3 bản thống kê định kỳ')
            for line in stats():
                for key in STATS:
                    value = float(re.search(r'\b' + key + r'\s*[=:]\s*([^\s,]+)', line).group(1))
                    need(math.isfinite(value) and value >= 0, 'Thống kê sai ' + key)
            r.wait(lambda: r.rows() and all(k in r.rows()[-1] for k in STATS),
                   message='Thống kê không được ghi vào CSV')
            for key in STATS:
                need(math.isfinite(float(r.rows()[-1][key])) and float(r.rows()[-1][key]) >= 0,
                     'Thống kê CSV sai ' + key)
            r.finish()
        return
    # A31 — bản duyệt: lỗi mở/khởi tạo thiết bị, không ready khi motor_init lỗi.
    if case == 'A31':
        variants = [dict(OPEN_FAIL=x) for x in ('motor', 'motor_init', 'servo', 'camera')]
        variants += [dict(CAMERA='none'), dict(PREVIEW='fail_start'), dict(MOTOR_FAIL='1')]
        for i, env in enumerate(variants):
            with running(cfg, root / str(i), env=env) as r:
                reject(r, before_init=False)
                if 'MOTOR_FAIL' in env:
                    events = r.events()
                    failed = [j for j, e in enumerate(events) if e[1] == 'motor_emergency' and e[3]]
                    need(failed and any(j > failed[0] and e[1] == 'servo_angle' and e[2] == 0
                                        for j, e in enumerate(events)), 'Motor fail vẫn phải thử servo0')
        return
    # A32 — L7: G lúc dữ liệu cũ bị bỏ; dữ liệu về không dùng lại phím cũ.
    if case == 'A32':
        for kind in ('camera', 'road'):
            with running(cfg, root / kind, values={'mask_max_age_ms': 100}) as r:
                r.boot(); r.inject(kind + '_stall')
                age = 500 if kind == 'camera' else 100
                r.wait(lambda: td.now_ms() - float(str(
                    r.of('frame' if kind == 'camera' else 'mask')[-1][2]).split()[1]) > age,
                    2, 'Chưa đủ tuổi cũ')
                mark = len(r.events()); ev = r.key('G')
                r.wait(lambda: any(str(e[2]).startswith('ignored ') for e in r.of('go', mark)), message='G cũ không bị bỏ')
                no_motion(r, ev[0])
                r.inject(kind + '_resume')
                r.frames(8, lambda: no_motion(r, ev[0], 'go_queued'))
                r.go(); r.finish()
        return
    # A33 — bản duyệt: checker chung bắt cả hai trần và không bỏ lệnh fail.
    if case == 'A33':
        for kind, value in (('motor_speed', 10.01), ('servo_angle', -40.01)):
            try:
                cap([(0, kind, value, False)])
            except Failure as error:
                need(error.tag == 'invariant_cap', 'Checker hỏng sai lý do')
            else:
                raise Failure('self_check', 'Checker bỏ lọt trần')
        cap([(0, 'motor_speed', -10, False), (1, 'servo_angle', 40, False)])
        # Tự kiểm encode/decode fixture khi ca thực chạy, phiên này chỉ py_compile.
        need(read_png(png_bytes(mask_rows('left'))) == b''.join(bytes(r) for r in mask_rows('left')), 'PNG roundtrip')
        test_case('A16', cfg, root)
        return
    # A34 — H4/B2/bản duyệt: làn trái → góc dương, phải → góc âm.
    if case == 'A34':
        for mode, sign in (('left', 1), ('right', -1)):
            with running(cfg, root / mode, values={'kd': 0}, schedule=((1, mode),)) as r:
                r.boot(); r.go()
                mark = len(r.events()); r.frames(12)
                angles = [e[2] for e in r.of('servo_angle', mark) if not e[3]]
                need(angles and all(v * sign > 0 for v in angles), 'Sai dấu +trái/-phải: ' + repr(angles), 'steer_sign')
                r.finish()
        return
    # A60 — T47m LOW-4: lỗi sha256sum vẫn đủ log như A21 và cảnh báo trên stderr.
    if case == 'A60':
        from unittest.mock import patch
        fake_bin = root / 'fake_bin'
        fake_bin.mkdir(parents=True)
        sha256 = fake_bin / 'sha256sum'
        sha256.write_text('#!' + sys.executable + '\nfrom pathlib import Path\n'
                          'Path(__file__).with_name("called").touch()\nraise SystemExit(1)\n', encoding='utf-8')
        sha256.chmod(0o755)
        child_path = str(fake_bin) + os.pathsep + os.environ.get('PATH', '')
        with patch.dict(os.environ, {'PATH': child_path}), tempfile.TemporaryFile() as error_log:
            with running(cfg, root / 'run', values={'record_every_n': 5, 'slow_per_deg': 0}, stderr=error_log) as r:
                r.boot(); r.go(); r.frames(25); r.finish(key='\x1b')
                need((fake_bin / 'called').is_file(), 'Chưa gọi sha256sum giả', 'setup')
                inspect_session(r, sha256_failed=True)
                inspect_images(r)
                error_log.seek(0)
                errors = error_log.read().decode('utf-8', errors='replace')
                warnings = [line for line in errors.splitlines()
                            if re.search(r'(?i)\bwarn(?:ing)?\b|canh bao|cảnh báo', line)
                            and re.search(r'(?i)sha|bam|băm|hash', line)]
                need(len(warnings) == 1,
                     'Phải có đúng một dòng cảnh báo stderr về băm: ' + errors, 'sha256_warning')
        return
    raise Failure('setup', 'Chưa có ca ' + case)


def selftest():
    from unittest.mock import patch
    with patch.object(td, 'now_ms', return_value=11975221.6):
        stamp = external_stamp_ms()
    assert stamp == 11975221
    same_ms = [(11975221.0, 'motor_speed', 0, False), (11975221.0, 'servo_angle', 0, False)]
    assert stopping_pair(same_ms, stamp) == tuple(same_ms)
    straight = [(0, 'motor_speed', 7.98, False), (1, 'reason', 'BAM_LAN', False),
                (2, 'servo_angle', 0, False), (3, 'motor_speed', 0, False),
                (4, 'servo_angle', 0, False)]
    assert stopping_pair(straight, 1) == (straight[3], straight[4])
    reversed_stop = [(0, 'servo_angle', 0, False), (1, 'motor_speed', 0, False),
                     (2, 'servo_angle', 0, False)]
    try:
        stopping_pair(reversed_stop, 0)
    except Failure as error:
        assert error.tag == 'stop_order'
    else:
        assert False, 'M7 lọt: servo 0 trước motor 0'
    assert stopping_pair(straight[:-1], 1) is None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--selftest', action='store_true')
    parser.add_argument('--case', choices=CASES + ['B5', 'gitignore'])
    parser.add_argument('--hooks')
    parser.add_argument('--slack', type=int, choices=(0, 300), default=0)
    cfg = parser.parse_args()
    if cfg.selftest:
        selftest()
        print('PASS selftest: 4 vết tổng hợp, bắt M7 và mốc cùng ms')
        return
    if cfg.case == 'gitignore':
        check_gitignore()
        print('PASS gitignore')
        return
    if not cfg.case or not cfg.hooks:
        parser.error('--case và --hooks là bắt buộc nếu không dùng --selftest')
    need(os.name == 'posix', 'Chỉ chạy hooks trên Linux', 'environment')
    with tempfile.TemporaryDirectory(prefix='ucr-t47-') as tmp:
        root = Path(tmp) / cfg.case
        if cfg.case == 'B5':
            # L7 thay S03: tạo trễ xác định, chấp nhận dừng do tuổi lệnh trong ca tải.
            with running(cfg, root, values={'control_period_ms': 20, 'cmd_max_age_ms': 500, 'record_every_n': 5}) as r:
                r.boot(); r.go(); ev = r.inject('control_delay 80')
                r.wait(lambda: r.of('record'), 3, 'Thiếu trace record đầu tiên')
                need(record(r)[0] == 5, 'Tiền đề every_n phải bằng5')
                r.wait(lambda: record(r)[0] >= 10, 7, 'Không giảm ghi trong 5s+2s')
                r.wait(lambda: record(r)[0] == 40, 14, 'Không tiếp tục nhân đôi tới40')
                r.observe(7, lambda: need(record(r)[0] <= 40, 'Tự giảm ghi vượt trần40'))
                values = [int(str(e[2]).split()[0]) for e in r.of('record') if e[0] >= ev[0]]
                changes = [5]
                for value in values:
                    if changes[-1] != value:
                        changes.append(value)
                need(changes == [5, 10, 20, 40], 'Nhân đôi sai hoặc vượt40: ' + repr(changes))
                need(re.search(r'\brecord_reason\s*[=:]\s*p95\b', r.text()), 'Không ghi lý do giảm ghi')
                r.inject('control_delay 0'); space(r); r.finish()
        else:
            test_case(cfg.case, cfg, root)
    print('PASS ' + cfg.case)


if __name__ == '__main__':
    main()
