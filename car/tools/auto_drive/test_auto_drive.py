"""T47 P1/P2: chấp nhận hộp đen theo hợp đồng leader; Python 3.6/Linux.

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

CASES = ['A{:02d}'.format(i) for i in list(range(1, 36)) + list(range(40, 49))]
CASES += ['A07b', 'A08b', 'A30c', 'A30d', 'A30e', 'A30f', 'A30g', 'A30h',
          'A40b', 'A42b', 'A43b', 'A49', 'A50', 'A51']
YOLO_CLASSES = ['red', 'green', 'off', 'left', 'right', 'no_left', 'no_right',
                'round_barrier', 'work_barrier']
YOLO_VALUES = dict(start_wait_green=1, vote_window=5, vote_min=3,
                   green_frames=3, approach_cms=5, red_stop_x_frac=0.90,
                   det_max_age_ms=500, yolo_every_n=3,
                   control_period_ms=20, slow_per_deg=0, kd=0)
P2_BOUNDS = dict(start_wait_green=(1, 0, 1), vote_window=(5, 1, 30),
                 vote_min=(3, 1, 5), green_frames=(3, 1, 5),
                 approach_cms=(5, 0, 10), red_stop_x_frac=(0.90, 0, 1))
for _light in ('red', 'green', 'off'):
    P2_BOUNDS[_light + '_conf_min'] = (0.5, 0, 1)
    P2_BOUNDS[_light + '_box_min_area_frac'] = (0.0001, 0, 1)
    YOLO_VALUES[_light + '_conf_min'] = 0.5
    YOLO_VALUES[_light + '_box_min_area_frac'] = 0.0001
YOLO_BOXES = dict(empty=[], green=[('green', 800)], off=[('off', 800)],
                  red_mid=[('red', 800)], red_edge=[('red', 1152)],
                  red_089=[('red', 0.89 * 1280)], red_095=[('red', 0.95 * 1280)],
                  red_050=[('red', 0.50 * 1280)], red_085=[('red', 0.85 * 1280)],
                  red_green=[('red', 1152), ('green', 800)])
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


def yolo_fixtures(root, schedule):
    """Theo make_fake_yolo.py: tensor cxcywh sau letterbox, không cần numpy/onnx."""
    info = dict(task='detection', architecture='fake_conv_random', class_names=YOLO_CLASSES,
                num_classes=9, source_image_hw=[720, 1280], input_name='images',
                input_shape=[1, 3, 320, 320], input_hw=[320, 320], input_dtype='float32',
                color_order='RGB', layout='NCHW', preprocessing_in_model=False,
                normalization=dict(scale=1 / 255, mean=[0, 0, 0], std=[1, 1, 1]),
                resize='letterbox', letterbox=dict(auto=False, center=True, padding_value=114,
                                                  scaleup=True, stride=32),
                letterbox_1280x720=dict(scale=0.25, resized_wh=[320, 180], pad_left=0, pad_top=70),
                output_name='output0', output_shape=[1, 13, 100],
                output_type='cxcywh_pixels_and_class_probabilities', box_format='cxcywh',
                box_units='input_pixels', scores_are_probabilities=True, objectness=False,
                nms_in_model=False, nms_class_aware=True, conf_threshold=0.25, nms_iou=0.7,
                opset=12, dynamic=False, seed=0, fake_data=True, recognition_validated=False)
    (root / 'yolo_info.json').write_text(json.dumps(info), encoding='utf-8')
    (root / 'yolo.engine').write_bytes(b'fake-yolo-engine\n')
    header = repr(dict(descr='<f4', fortran_order=False, shape=(1, 13, 100))).encode('ascii')
    header += b' ' * ((-(10 + len(header) + 1)) % 16) + b'\n'
    prefix = b'\x93NUMPY\x01\x00' + struct.pack('<H', len(header)) + header
    for mode, boxes in YOLO_BOXES.items():
        values = [0.0] * 1300
        for proposal, (cls, x2) in enumerate(boxes):
            x1 = x2 - 152
            box = [(x1 + x2) / 2 * 0.25, 300 * 0.25 + 70, (x2 - x1) * 0.25, 200 * 0.25]
            for channel, value in enumerate(box):
                values[channel * 100 + proposal] = value
            values[(4 + YOLO_CLASSES.index(cls)) * 100 + proposal] = 0.9
        (root / (mode + '.npy')).write_bytes(prefix + struct.pack('<1300f', *values))
    (root / 'yolo.csv').write_text('frame_id,tensor\n' + ''.join(
        '{},{}.npy\n'.format(frame, mode) for frame, mode in schedule), encoding='utf-8')


def cap(events):
    # A33: kiểm cả lệnh thử gửi bị fail; không cho lọt giá trị ngoài lớp kẹp.
    for _, kind, value, _ in events:
        if kind in ('motor_speed', 'servo_angle'):
            limit = 10 if kind == 'motor_speed' else 40
            need(math.isfinite(value) and abs(value) <= limit, 'Vượt trần cứng: {} {}'.format(kind, value), 'invariant_cap')


class Run(td.Run):
    """Dùng lại wait/observe/text/send/ack/children/close của T33; chỉ đổi khởi chạy auto."""
    def __init__(self, cfg, root, values=None, env=None, schedule=((1, 'straight'),),
                 args=None, lock=None, setup=None, no_yolo=True, info=True,
                 yolo_schedule=None, model_load_fail=None):
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
        if yolo_schedule is not None:
            yolo_fixtures(root, yolo_schedule)
        if setup:
            setup(self)
        helper = bindir / 'ucr_stop'
        helper.write_text('#!' + sys.executable + '\nimport os,time\n'
                          'with open(os.environ["UCR_AUTO_FAKE_TRACE"],"a") as f:\n'
                          ' f.write("%d test_ucr_stop\\n" % int(time.monotonic()*1000))\n', encoding='utf-8')
        helper.chmod(0o755)
        child_env = {k: v for k, v in os.environ.items()
                     if not k.startswith(('UCR_AUTO_FAKE_', 'UCR_DRIVE_FAKE_', 'UCR_CAM_TEST_'))
                     and k not in ('DISPLAY', 'WAYLAND_DISPLAY', 'UCR_AUTO_MODEL_LOAD_FAIL')}
        child_env.update(LC_ALL='C.UTF-8', LANG='C.UTF-8', DISPLAY=':ucr_fake_only')
        fake = dict(TRACE=str(self.path), LOCK=str(lock or root / 'vehicle.lock'), CTRL=str(self.ctrl),
                    PREVIEW='stream:' + str(self.keys), CAMERA='ok', FREE_MB='4096',
                    ROAD_MASKS=str(root / 'list.csv'))
        if yolo_schedule is not None:
            fake['YOLO_TENSORS'] = str(root / 'yolo.csv')
        fake.update(env or {})
        child_env.update(('UCR_AUTO_FAKE_' + k, v) for k, v in fake.items())
        if model_load_fail is not None:
            child_env['UCR_AUTO_MODEL_LOAD_FAIL'] = model_load_fail
        child_env['TSAN_OPTIONS'] = ':'.join(filter(None, (child_env.get('TSAN_OPTIONS', ''),
                                                        'suppressions=' + str(td.TSAN_SUPP))))
        argv = [str(self.binary), '--control', str(self.control), '--out-root', str(self.out)]
        if info:
            argv.extend(['--info', str(root / 'road_info.json')])
        if no_yolo:
            argv.append('--no-yolo')
        if yolo_schedule is not None:
            argv.extend(['--yolo-engine', str(root / 'yolo.engine'), '--yolo-info', str(root / 'yolo_info.json')])
        argv.extend(args or [])
        self.log = tempfile.TemporaryFile()
        try:
            self.process = subprocess.Popen(argv, cwd=str(root), env=child_env, stdin=subprocess.DEVNULL,
                                            stdout=self.log, stderr=self.log, start_new_session=True)
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


def stopped(run, stamp, limit, tag='stop_order', after=None, mark=0):
    """A27: cắt sau tác nhân, kiểm motor thực/thứ tự; S chỉ cộng đúng một lần."""
    def pair():
        return stopping_pair(run.events()[mark:], stamp if after is None else after, tag)
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


def yolo_running(cfg, root, values=None, **kwargs):
    control = dict(YOLO_VALUES)
    control.update(values or {})
    kwargs.setdefault('yolo_schedule', ((1, 'empty'),))
    return running(cfg, root, values=control, no_yolo=False, **kwargs)


def detection_batches(events):
    """Giữ vị trí trace để không nhận lô/lệnh trước tác nhân trong cùng ms."""
    batches = []
    for index, event in enumerate(events):
        if event[1] != 'detections':
            continue
        parts = str(event[2]).split()
        need(len(parts) == 2 and not event[3], 'detections phải có frame_id captured_ms', 'interface')
        frame, captured = int(parts[0]), float(parts[1])
        need(frame > 0 and math.isfinite(captured) and 0 <= captured <= event[0],
             'Mốc detections không hợp lệ', 'interface')
        need(not batches or frame > batches[-1][1], 'Đếm lại cùng lô YOLO/đảo frame', 'interface')
        batches.append((index, frame, captured, event[0]))
    return batches


def last_detection_frame(events):
    # Frame hợp lệ luôn dương; chưa có lô thì chưa vượt mốc nào.
    batches = detection_batches(events)
    return batches[-1][1] if batches else 0


def held(run, mark):
    events = run.events()
    need(td.last_commands(events) == (0, 0), 'Không giữ lệnh cuối motor/servo0', 'light_latch')
    need(not any(e[1] in ('motor_speed', 'servo_angle') and e[2] != 0 and not e[3]
                 for e in events[mark:]), 'Có chuyển động khi đang chờ đèn', 'light_latch')


def stop_tail(run, stamp, mark):
    events = run.events()
    pair = stopping_pair(events[mark:], stamp)
    need(pair is not None, 'Thiếu cặp dừng trong đoạn trace mới')
    return next(i for i in range(mark, len(events)) if events[i] is pair[1]) + 1


def phase_reached(run, first_frame, mark):
    def reached():
        held(run, mark)
        batches = detection_batches(run.events())
        return batches and batches[-1][1] >= first_frame
    return run.wait(reached, 25, 'Chưa quan sát pha đèn frame ' + str(first_frame))


def green_release(run, first_frame, mark, since=None):
    def enough():
        events = run.events()
        batches = [b for b in detection_batches(events) if b[0] >= mark and b[1] >= first_frame
                   and (since is None or b[2] > since)]
        cutoff = batches[2][0] if len(batches) >= 3 else len(events)
        need(not any(e[1] in ('motor_speed', 'servo_angle') and e[2] != 0 and not e[3]
                     for e in events[mark:cutoff]), 'Chạy trước 3 lô xanh mới', 'green_frames')
        return batches[2] if len(batches) >= 3 else None
    third = run.wait(enough, 25, 'Thiếu 3 lô xanh mới')
    run.wait(lambda: any(e[1] == 'motor_speed' and e[2] > 0 and not e[3]
                         for e in run.events()[third[0] + 1:]), message='Đủ xanh không tự chạy')
    run.wait(lambda: any(e[2] == 'DEN_XANH' for e in run.of('reason', mark)),
             message='Thiếu reason DEN_XANH')
    return third


def red_stop(run, first_frame, mark):
    batches = run.wait(lambda: [b for b in detection_batches(run.events())
                               if b[0] >= mark and b[1] >= first_frame][2:],
                       25, 'Thiếu 3 lô đỏ mới')
    third = batches[0]
    first = next(b for b in detection_batches(run.events()) if b[0] >= mark and b[1] >= first_frame)
    need(not any(motor_zero(e) for e in run.events()[first[0]:third[0]]),
         'Dừng đỏ trước khi đủ 3 phiếu mới', 'red_votes')
    stopped(run, third[3], run.values['control_period_ms'] + 200, mark=third[0] + 1)
    run.wait(lambda: any(e[2] == 'DUNG_DEN_DO' for e in run.of('reason', mark)),
             message='Thiếu reason DUNG_DEN_DO')
    # Bắt luôn chuyển động lọt giữa cặp dừng và lần đọc trace này.
    end = stop_tail(run, third[3], third[0] + 1)
    held(run, end)
    return end


def phase_batches(run, lo, hi, count):
    batches = [b for b in detection_batches(run.events()) if lo <= b[1] < hi]
    need(len(batches) == count, 'Lịch cần {} lô trong [{},{}), thực tế {}'.format(
        count, lo, hi, [b[1] for b in batches]), 'setup')
    return batches


def moving(run, mark):
    speeds = [e[2] for e in run.of('motor_speed', mark) if not e[3]]
    need(not any(motor_zero(e) for e in run.events()[mark:]) and all(0 < v <= 10 for v in speeds),
         'Dừng đỏ khi chưa đủ phiếu/vị trí hoặc vượt trần10', 'red_boundary')


def inspect_detections(run):
    path = run.session() / 'detections.csv'
    with path.open(encoding='utf-8', newline='') as stream:
        reader = csv.DictReader(stream)
        need(reader.fieldnames == 'frame_id captured_ms class conf x1 y1 x2 y2'.split(),
             'detections.csv sai cột', 'record_format')
        rows = list(reader)
    batches = {b[1]: b[2] for b in detection_batches(run.events())}
    seen, previous = set(), 0
    for row in rows:
        need(None not in row and None not in row.values(), 'CSV thiếu/thừa ô', 'record_format')
        frame, captured = int(row['frame_id']), float(row['captured_ms'])
        need(frame >= previous and frame in batches and captured == batches[frame],
             'CSV sai ID/thời điểm lô', 'record_format')
        previous = frame
        cls = row['class']
        expected = '' if frame < 90 else ('green' if frame < 180 else 'red')
        need(cls == expected, 'Lớp CSV không khớp tensor', 'record_format')
        seen.add(cls)
        if not cls:
            need(all(row[k] == '' for k in ('class', 'conf', 'x1', 'y1', 'x2', 'y2')),
                 'Lô rỗng cần sáu ô rỗng', 'record_format')
        else:
            numbers = [float(row[k]) for k in ('conf', 'x1', 'y1', 'x2', 'y2')]
            need(all(math.isfinite(v) for v in numbers), 'CSV NaN/Inf', 'record_format')
            x2 = 800 if cls == 'green' else 1152
            expected_box = (0.9, x2 - 152, 200, x2, 400)
            need(all(abs(a - b) < 0.001 for a, b in zip(numbers, expected_box)),
                 'CSV sai confidence/hộp ảnh gốc', 'record_format')
    need(seen == {'', 'green', 'red'}, 'Thiếu lô rỗng/xanh/đỏ trong CSV', 'record_format')


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


def inspect_session(run):
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
    need(data['control_yaml'].encode('utf-8') == raw and data['control_sha256'] == hashlib.sha256(raw).hexdigest(),
         'Bản chép/băm yaml sai', 'record_format')
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
    # A06 — hợp đồng P2 thay L1/P1: đủ YOLO chạy; thiếu/hỏng từ chối trước motor_init.
    if case == 'A06':
        with yolo_running(cfg, root / 'valid') as r:
            r.boot(); mark = len(r.events()); r.key('G')
            r.wait(lambda: any(e[2] == 'CHO_DEN_XUAT_PHAT' for e in r.of('reason', mark)),
                   message='YOLO hợp lệ không vào chờ xanh')
            r.frames(6, lambda: held(r, mark)); r.finish()
        for flag in (None, '--yolo-engine', '--yolo-info'):
            folder = root / ('missing_' + str(flag))
            args = [] if flag is None else [flag, str(folder / ('yolo.engine' if flag.endswith('engine') else 'yolo_info.json'))]
            with running(cfg, folder, no_yolo=False, args=args,
                         setup=lambda r: yolo_fixtures(r.root, ((1, 'empty'),))) as r:
                reject(r)
        bad = [lambda r: (r.root / 'yolo.engine').unlink(),
               lambda r: (r.root / 'yolo.engine').write_bytes(b''),
               lambda r: (r.root / 'yolo_info.json').unlink(),
               lambda r: (r.root / 'yolo_info.json').write_text('{', encoding='utf-8'),
               lambda r: (r.root / 'yolo_info.json').write_text('{}', encoding='utf-8')]
        for i, setup in enumerate(bad):
            with yolo_running(cfg, root / ('bad_' + str(i)), setup=setup) as r:
                reject(r)
        for index in range(9):
            def wrong_class(r, index=index):
                path = r.root / 'yolo_info.json'
                data = json.loads(path.read_text(encoding='utf-8'))
                data['class_names'][index] = 'ten_lop_khong_khop'
                path.write_text(json.dumps(data), encoding='utf-8')
            with yolo_running(cfg, root / ('class_' + str(index)), setup=wrong_class) as r:
                reject(r)
        return
    # A07b — B1/P2: ca bổ sung, không thay bộ sai cấu hình P1.
    if case == 'A07b':
        bad = []
        for key, (_, lo, hi) in P2_BOUNDS.items():
            bad.extend({key: v} for v in ('sai_kieu', lo - 1, hi + 1))
        bad += [{key: 1.5} for key in ('start_wait_green', 'vote_window', 'vote_min', 'green_frames')]
        bad += [dict(vote_window=2, vote_min=3, green_frames=1),
                dict(vote_window=2, vote_min=1, green_frames=3)]
        bad += [{key: 1} for key in ('box_min_area_px', 'box_min_area_frac', 'conf_min')]
        for i, values in enumerate(bad):
            with running(cfg, root / str(i), values=values) as r:
                reject(r)
        for light in ('red', 'green', 'off'):
            with yolo_running(cfg, root / light, values={light + '_conf_min': 0.1}) as r:
                reject(r)
        return
    # A08b — B1/P2: bảng mặc định và miền đúng; approach được kẹp khi chạy.
    if case == 'A08b':
        variants = [{}]
        for key, (_, lo, hi) in P2_BOUNDS.items():
            for value in (lo, hi):
                values = {key: value}
                if key == 'vote_window':
                    values.update(vote_min=1, green_frames=1)
                variants.append(values)
        variants.append(dict(max_speed_cms=0, cruise_cms=0, min_cms=0, lost_cms=0, approach_cms=10))
        for i, values in enumerate(variants):
            with running(cfg, root / str(i), values=values) as r:
                r.boot(); r.finish()
                if not values:
                    for key, (default, _, _) in P2_BOUNDS.items():
                        pattern = r'\b' + key + r'\b\s*[:=|]?\s*([-+\d.eE]+)'
                        lines = [l for l in r.text().splitlines() if '(mặc định)' in l]
                        need(any(re.search(pattern, l) and float(re.search(pattern, l).group(1)) == default
                                 for l in lines), 'Bảng mặc định P2 thiếu/sai ' + key)
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
    # A30 — B4/P1: YOLO giữ cả model; mask cũ dừng, tự hồi phục; Space/TERM độc lập.
    if case == 'A30':
        for action in ('resume', 'space', 'term'):
            with yolo_running(cfg, root / action, values=dict(start_wait_green=0, cmd_max_age_ms=500),
                              yolo_schedule=((1, 'red_mid'),)) as r:
                r.boot(); r.go()
                r.wait(lambda: any(e[2] == 'TIEP_CAN_DEN_DO' for e in r.of('reason')),
                       message='Chưa tiếp cận đỏ trước stall')
                mark = len(r.events()); ev = r.inject('yolo_stall')
                age = r.values.get('mask_max_age_ms', 300)
                r.wait(lambda: td.now_ms() - float(str(r.of('mask')[-1][2]).split()[1]) > age,
                       3, 'Mask không cũ khi YOLO stall')
                last = r.of('mask')[-1]
                captured = float(str(last[2]).split()[1])
                stopped(r, captured, age + r.values['control_period_ms'] + 200,
                        after=captured + age, mark=mark)
                r.wait(lambda: any(e[2] == 'MAT_NAN_CU' for e in r.of('reason', mark)),
                       message='Thiếu MAT_NAN_CU khi YOLO stall')
                end = stop_tail(r, captured + age, mark)
                count = len(r.of('detections'))
                r.frames(4, lambda: held(r, end))
                need(r.of('mask')[-1] == last and len(r.of('detections')) == count,
                     'Model vẫn công bố dữ liệu khi stall', 'setup')
                if action == 'term':
                    mark = len(r.events()); stamp = external_stamp_ms()
                    os.kill(r.process.pid, signal.SIGTERM)
                    stopped(r, stamp, 1000, mark=mark); r.exited()
                    continue
                if action == 'space':
                    mark = len(r.events()); key = r.key(' ')
                    stopped(r, key[0], 150, mark=mark)
                resume_mark = len(r.events()); r.inject('yolo_resume')
                new = r.wait(lambda: next((e for e in r.of('mask', resume_mark)
                                           if float(str(e[2]).split()[1]) > ev[0]), None),
                             message='Không có mask mới sau resume')
                need(not any(e[1] == 'motor_speed' and e[2] != 0 and not e[3]
                             for e in r.events()[end:r.events().index(new)]), 'Chạy trước mask mới')
                if action == 'space':
                    r.frames(6, lambda: held(r, end))
                else:
                    r.wait(lambda: td.last_commands(r.events())[0] > 0,
                           message='Không tự chạy sau resume khi chưa có chốt')
                r.finish()
        return
    # A30c..h — T47k bổ sung 03/11 13:50 a–f: lỗi YOLO liên tục, mask vẫn tươi.
    if case in ('A30c', 'A30d', 'A30e', 'A30f', 'A30g', 'A30h'):
        for action in (('space', 'term') if case == 'A30f' else ('normal',)):
            no_yolo = case == 'A30h'
            values = dict(YOLO_VALUES, start_wait_green=0, cmd_max_age_ms=500,
                          mask_max_age_ms=300, cruise_cms=8, max_speed_cms=10, kp=0)

            def before_launch(run):
                if case == 'A30d':
                    # Tiêm trước Popen để chưa có lô YOLO nào, không chỉ trước phím G.
                    run.seq = 1
                    run.ctrl.write_text('1 yolo_throw_on\n', encoding='utf-8')

            schedule = ((1, 'empty'), (120, 'red_edge')) if case == 'A30e' else ((1, 'empty'),)
            with running(cfg, root / action, values=values, no_yolo=no_yolo,
                         yolo_schedule=schedule, setup=before_launch) as r:
                r.boot()
                approach = r.values['approach_cms']
                age = r.values['det_max_age_ms']

                def wait_cruise(since=0):
                    r.wait(lambda: any(float(row['t_ms']) >= since
                                       and row['planner_state'] == row['reason'] == 'BAM_LAN'
                                       and abs(float(row['sent_speed_cms']) - 8) < 1e-6
                                       and (no_yolo or 0 <= float(row['det_age_ms']) <= age)
                                       and 0 <= float(row['mask_age_ms']) <= 300 + r.slack
                                       and 0 <= float(row['image_age_ms']) <= 500 + r.slack
                                       for row in r.rows()), 15, 'Chưa bám làn 8 với dữ liệu tươi')
                    speed_is(r, 8)

                if case == 'A30d':
                    r.wait(lambda: any(e[2] == '1 yolo_throw_on' for e in r.of('inject')),
                           message='Thiếu ACK lỗi YOLO trước G')
                    need(not detection_batches(r.events()), 'Đã có lô YOLO trước G', 'setup')
                    mark = len(r.events()); ev = r.key('G')
                else:
                    r.go(); wait_cruise()
                    if case == 'A30e':
                        need(last_detection_frame(r.events()) < 120, 'Bỏ lỡ pha trước đỏ', 'setup')
                        red_end = red_stop(r, 120, len(r.events()))
                    mark = len(r.events()); ev = r.inject('yolo_throw_on')
                limit = age + r.values['control_period_ms'] + 200 + r.slack
                deadline = ev[0] + limit
                r.wait(lambda: td.now_ms() >= deadline, limit / 1000.0 + 1,
                       'Chưa tới hạn phát hiện cũ')

                def capped():
                    cap_from = ev[0] if case == 'A30d' else deadline
                    motors = [e for e in r.of('motor_speed', mark) if e[0] >= cap_from]
                    need(all(abs(e[2]) <= approach for e in motors),
                         'Motor vượt approach sau hạn YOLO cũ', 'yolo_stale_cap')

                if case == 'A30e':
                    check = lambda: held(r, red_end)
                else:
                    check = (lambda: None) if no_yolo else capped
                r.frames(12, check)
                check()
                for kind in ('frame', 'mask'):
                    need(any(float(str(e[2]).split()[1]) > deadline for e in r.of(kind, mark)),
                         'Không có ảnh/mask mới sau lỗi YOLO', 'setup')
                motors = [e for e in r.of('motor_speed', mark) if e[0] >= deadline]
                if no_yolo:
                    need(motors and all(abs(e[2] - 8) < 1e-6 for e in motors)
                         and any(not e[3] for e in motors),
                         '--no-yolo bị đổi tốc độ bởi yolo_throw_on', 'yolo_disabled')
                elif case != 'A30e':
                    need(any(0 < e[2] <= approach and not e[3] for e in motors),
                         'YOLO cũ làm dừng hẳn thay vì hạ trần', 'yolo_stale_motion')
                    reason_mark = 0 if case == 'A30d' else mark
                    need(any(e[2] == 'MAT_YOLO' and e[0] <= deadline for e in r.of('reason', reason_mark)),
                         'Thiếu MAT_YOLO trong hạn', 'yolo_stale_reason')
                    r.wait(lambda: td.last_commands(r.events())[0] > 0,
                           3 + r.slack / 1000.0, 'Cần đang chạy MAT_YOLO trước tác nhân', tag='setup')
                need(not any(e[0] >= deadline for e in r.of('detections', mark)),
                     'Lỗi YOLO liên tục vẫn công bố lô mới sau hạn', 'setup')
                if case == 'A30d':
                    need(not detection_batches(r.events()), 'Lượt chưa có lô đã nhận detections', 'setup')
                stale_until = external_stamp_ms()
                recovery_start = recovery_until = None

                if case == 'A30c':
                    resume_mark = len(r.events()); off = r.inject('yolo_throw_off')
                    new = r.wait(lambda: next((b for b in detection_batches(r.events())
                                               if b[0] >= resume_mark and b[2] > off[0]), None),
                                 15, 'Không có lô chụp mới sau yolo_throw_off')
                    wait_cruise(new[3])
                    recovery_start = external_stamp_ms()
                    speed_is(r, 8)
                    recovery_until = external_stamp_ms()
                elif case == 'A30f':
                    stop_mark = len(r.events())
                    if action == 'space':
                        stamp, stop_limit = r.key(' ')[0], 150
                    else:
                        stamp, stop_limit = external_stamp_ms(), 1000
                        os.kill(r.process.pid, signal.SIGTERM)
                    servo_stamp = stopped(r, stamp, stop_limit, mark=stop_mark)
                    need(0 <= servo_stamp - stamp <= stop_limit + r.slack,
                         'Servo0 quá hạn sau motor0', 'stop_order')
                    end = stop_tail(r, stamp, stop_mark)
                    if action == 'term':
                        r.exited(); held(r, end)
                    else:
                        r.frames(6, lambda: held(r, end))
                elif case == 'A30g':
                    stall_mark = len(r.events()); r.inject('road_stall')
                    mask_age = r.values['mask_max_age_ms']
                    r.wait(lambda: td.now_ms() - float(str(r.of('mask')[-1][2]).split()[1]) > mask_age,
                           3, 'Mask chưa cũ sau road_stall')
                    last = r.of('mask')[-1]
                    captured = float(str(last[2]).split()[1])
                    stopped(r, captured, mask_age + r.values['control_period_ms'] + 200,
                            after=captured + mask_age, mark=stall_mark)
                    r.wait(lambda: any(e[2] == 'MAT_NAN_CU' for e in r.of('reason', stall_mark)),
                           message='Thiếu MAT_NAN_CU sau road_stall')
                    end = stop_tail(r, captured + mask_age, stall_mark)
                    r.frames(6, lambda: held(r, end))
                    need(r.of('mask')[-1] == last, 'road_stall vẫn sinh mask', 'setup')

                if not (case == 'A30f' and action == 'term'):
                    r.finish()
                rows = r.rows()
                stale_rows = [row for row in rows if deadline <= float(row['t_ms']) < stale_until]
                need(stale_rows, 'CSV thiếu đoạn quan sát sau hạn', 'record_format')
                if no_yolo:
                    need(not any(row['reason'] == 'MAT_YOLO' for row in rows)
                         and not any(e[2] == 'MAT_YOLO' for e in r.of('reason')),
                         '--no-yolo có MAT_YOLO', 'yolo_disabled')
                    need(all(abs(float(row['sent_speed_cms']) - 8) < 1e-6 for row in stale_rows),
                         '--no-yolo không giữ 8 trong CSV', 'yolo_disabled')
                elif case == 'A30e':
                    need(all(row['reason'] == 'DUNG_DEN_DO' and float(row['sent_speed_cms']) == 0
                             for row in stale_rows)
                         and not any(e[2] == 'MAT_YOLO' for e in r.of('reason', mark)),
                         'Mất YOLO đổi lý do/mở chốt đỏ', 'light_latch')
                else:
                    need(any(row['reason'] == 'MAT_YOLO' and row['planner_state'] == 'BAM_LAN'
                             and (case == 'A30d' or float(row['det_age_ms']) > age)
                             and 0 <= float(row['mask_age_ms']) <= 300 + r.slack
                             and 0 <= float(row['image_age_ms']) <= 500 + r.slack
                             and 0 < float(row['sent_speed_cms']) <= approach for row in stale_rows),
                         'CSV thiếu MAT_YOLO khi ảnh/mask tươi và xe vẫn chạy', 'yolo_stale_reason')
                    if case == 'A30d':
                        need(not detection_batches(r.events()), 'Lượt chưa có lô nhận detections', 'setup')
                    if recovery_start is not None:
                        recovered = [row for row in rows
                                     if recovery_start <= float(row['t_ms']) < recovery_until
                                     and 0 <= float(row['mask_age_ms']) <= r.values['mask_max_age_ms'] + r.slack
                                     and float(row['image_age_ms']) <= 500 + r.slack]
                        need(len(recovered) >= 3 and all(row['reason'] == row['planner_state'] == 'BAM_LAN'
                                               and abs(float(row['sent_speed_cms']) - 8) < 1e-6
                                               for row in recovered),
                             'Sau lô tươi chưa hết MAT_YOLO/về 8', 'yolo_recovery')
                        need(not any(e[2] == 'MAT_YOLO' and recovery_start <= e[0] < recovery_until
                                     for e in r.of('reason')), 'MAT_YOLO còn sau phục hồi', 'yolo_recovery')
        return
    # A40 — P2/L1: G không đèn giữ0; không đếm lặp cùng lô để đủ xanh.
    if case == 'A40':
        with yolo_running(cfg, root, yolo_schedule=((1, 'empty'), (120, 'green'))) as r:
            r.boot(); mark = len(r.events()); r.key('G')
            r.wait(lambda: any(e[2] == 'CHO_DEN_XUAT_PHAT' for e in r.of('reason', mark)),
                   message='G không đèn không chờ xanh')
            phase_reached(r, 90, mark)
            need(last_detection_frame(r.events()) < 120, 'Bỏ lỡ pha không đèn', 'setup')
            green_release(r, 120, mark)
            space(r); r.finish()
        return
    # A40b — xanh trước G không tính; rỗng/off ngắt hai lô xanh, phải đếm lại.
    if case == 'A40b':
        with yolo_running(cfg, root / 'before_g', yolo_schedule=((1, 'green'),)) as r:
            r.boot()
            r.wait(lambda: len(detection_batches(r.events())) >= 3, message='Thiếu xanh trước G')
            no_motion(r, 0)
            mark = len(r.events()); ev = r.key('G')
            green_release(r, 1, mark, since=ev[0])
            r.finish()
        for breaker in ('empty', 'off'):
            schedule = ((1, 'empty'), (120, 'green'), (126, breaker), (156, 'green'))
            with yolo_running(cfg, root / breaker, yolo_schedule=schedule) as r:
                r.boot(); mark = len(r.events()); r.key('G')
                phase_reached(r, 150, mark)
                need(last_detection_frame(r.events()) < 156, 'Bỏ lỡ pha ngắt xanh', 'setup')
                phase_batches(r, 120, 126, 2)
                green_release(r, 156, mark)
                r.finish()
        return
    # A41 — P2/L1: đỏ giữa ảnh đủ phiếu chỉ giảm tốc, không lấy motor0 làm PASS.
    if case == 'A41':
        with yolo_running(cfg, root, values={'start_wait_green': 0, 'kp': 0}, schedule=((1, 'straight'),),
                          yolo_schedule=((1, 'empty'), (120, 'red_mid'))) as r:
            r.boot(); r.go(); speed_is(r, 8)
            need(last_detection_frame(r.events()) < 120, 'Đỏ đến trước tiền đề cruise', 'setup')
            mark = len(r.events())
            ev = r.wait(lambda: next((e for e in r.of('reason', mark) if e[2] == 'TIEP_CAN_DEN_DO'), None),
                        25, 'Không tiếp cận đèn đỏ')
            r.wait(lambda: any(e[1] == 'motor_speed' and 0 < e[2] <= 5 and not e[3]
                               and e[0] >= ev[0] for e in r.events()), message='Không giảm về approach')
            events = r.events()
            sent_mark = next(i for i in range(mark, len(events)) if events[i][0] >= ev[0]
                             and events[i][1] == 'motor_speed' and 0 < events[i][2] <= 5 and not events[i][3])
            def approaching():
                speeds = [e[2] for e in r.of('motor_speed', sent_mark) if not e[3]]
                need(speeds and all(0 < v <= 5 for v in speeds), 'Đỏ giữa ảnh không giữ tốc độ (0,approach]')
            r.frames(12, approaching)
            space(r); r.finish()
        return
    # A42/A43/A44 — P2/L1: mép x2/W=0,90; chốt đỏ qua mất/off/Space/G.
    if case in ('A42', 'A43', 'A44'):
        schedule = ((1, 'empty'), (120, 'red_edge'))
        if case == 'A43':
            schedule += ((210, 'empty'), (270, 'off'), (360, 'green'))
        elif case == 'A44':
            schedule += ((360, 'green'),)
        with yolo_running(cfg, root, values={'start_wait_green': 0}, yolo_schedule=schedule) as r:
            r.boot(); r.go(); mark = len(r.events())
            need(last_detection_frame(r.events()) < 120, 'Bỏ lỡ tiền đề trước đỏ', 'setup')
            end = red_stop(r, 120, mark)
            if case == 'A42':
                r.frames(12, lambda: held(r, end))
            elif case == 'A43':
                phase_reached(r, 240, end)
                need(last_detection_frame(r.events()) < 270, 'Bỏ lỡ pha rỗng', 'setup')
                reds = [b for b in detection_batches(r.events()) if 120 <= b[1] < 210]
                need(reds, 'Thiếu lô đỏ trước pha rỗng', 'setup')
                red = reds[-1]
                r.wait(lambda: td.now_ms() - red[2] > r.values['det_max_age_ms'],
                       3, 'Đỏ chưa quá tuổi')
                held(r, end)
                phase_reached(r, 300, end)
                need(last_detection_frame(r.events()) < 360, 'Bỏ lỡ pha off', 'setup')
                green_release(r, 360, end)
            else:
                space(r)
                for _ in range(4):
                    r.key('G'); r.frames(5, lambda: held(r, end))
                need(last_detection_frame(r.events()) < 360, 'G gửi sau khi đã xanh', 'setup')
                phase_reached(r, 300, end)
                green_release(r, 360, end)
            r.finish()
        return
    # A42b — đủ vote mới xét x2 của hộp MỚI NHẤT, không lấy hộp cũ sát mép.
    if case == 'A42b':
        variants = dict(below=((1, 'empty'), (120, 'red_089')),
                        two_votes=((1, 'empty'), (120, 'red_095'), (126, 'empty')),
                        latest_edge=((1, 'empty'), (120, 'red_050'), (126, 'red_095')),
                        latest_back=((1, 'empty'), (120, 'red_095'), (123, 'red_085')))
        for name, schedule in variants.items():
            with yolo_running(cfg, root / name, values={'start_wait_green': 0}, yolo_schedule=schedule) as r:
                r.boot(); r.go(); mark = len(r.events())
                need(last_detection_frame(r.events()) < 120, 'Bỏ lỡ trước biên đỏ', 'setup')
                if name == 'latest_edge':
                    end = red_stop(r, 120, mark)
                    old = phase_batches(r, 120, 126, 2)
                    third = [b for b in detection_batches(r.events()) if b[1] >= 126][0]
                    need(third[2] - old[0][2] <= r.values['det_max_age_ms'],
                         'Phiếu đỏ cũ đã hết tuổi trước lô mép', 'setup')
                    r.frames(6, lambda: held(r, end))
                else:
                    def reached():
                        moving(r, mark)
                        return last_detection_frame(r.events()) >= 156
                    r.wait(reached, 25, 'Chưa qua pha biên đỏ')
                    if name == 'two_votes':
                        phase_batches(r, 120, 126, 2)
                    elif name == 'latest_back':
                        phase_batches(r, 120, 123, 1)
                    else:
                        need(len([b for b in detection_batches(r.events()) if b[1] >= 120]) >= 3,
                             'Chưa đủ vote cho đỏ0.89', 'setup')
                    if name != 'two_votes':
                        need(any(e[2] == 'TIEP_CAN_DEN_DO' for e in r.of('reason', mark)),
                             'Chưa có bằng chứng xử lý đủ phiếu đỏ', 'setup')
                    moving(r, mark)
                r.finish()
        return
    # A43b — đỏ và xanh trong cùng tensor: đỏ thắng khi đủ phiếu.
    if case == 'A43b':
        with yolo_running(cfg, root, values={'start_wait_green': 0},
                          yolo_schedule=((1, 'empty'), (120, 'red_green'))) as r:
            r.boot(); r.go(); mark = len(r.events())
            need(last_detection_frame(r.events()) < 120, 'Bỏ lỡ trước đỏ+xanh', 'setup')
            end = red_stop(r, 120, mark)
            r.frames(12, lambda: held(r, end)); r.finish()
        return
    # A49 — leader: chờ xuất phát/chốt đỏ vẫn enabled, R bị bỏ như A03.
    if case == 'A49':
        for state in ('CHO_DEN_XUAT_PHAT', 'DUNG_DEN_DO'):
            schedule = ((1, 'empty'),) if state == 'CHO_DEN_XUAT_PHAT' else ((1, 'empty'), (120, 'red_edge'))
            with yolo_running(cfg, root / state, values={'start_wait_green': int(state == 'CHO_DEN_XUAT_PHAT')},
                              yolo_schedule=schedule) as r:
                r.boot(); mark = len(r.events())
                if state == 'DUNG_DEN_DO':
                    r.go()
                    need(last_detection_frame(r.events()) < 120, 'Bỏ lỡ trước đỏ/R', 'setup')
                    end = red_stop(r, 120, mark)
                else:
                    r.key('G')
                    r.wait(lambda: any(e[2] == state for e in r.of('reason', mark)), message='Chưa chờ xanh')
                    end = mark
                values = dict(r.values); values.update(start_wait_green=0, cruise_cms=9)
                patch_yaml(r, values); reload(r, 'ignored')
                for _ in range(3):
                    r.key('G'); r.frames(4, lambda: held(r, end))
                held(r, end); r.finish()
        return
    # A50 — yolo_throw một lần không làm chết model/tiến trình, không mở đỏ.
    if case == 'A50':
        with yolo_running(cfg, root, values={'start_wait_green': 0},
                          yolo_schedule=((1, 'empty'), (120, 'red_edge'))) as r:
            r.boot(); r.go()
            need(last_detection_frame(r.events()) < 120, 'Bỏ lỡ trước đỏ/throw', 'setup')
            end = red_stop(r, 120, len(r.events()))
            r.inject('yolo_throw'); mark = len(r.events())
            def recovered():
                held(r, end)
                return len([b for b in detection_batches(r.events()) if b[0] >= mark]) >= 3
            r.wait(recovered, 10, 'YOLO không tiếp tục sau throw')
            need(r.process.poll() is None, 'YOLO throw làm thoát tiến trình')
            r.finish()
        return
    # A51 — B5: CSV thật sau khi đóng writer, gồm cả hàng lô rỗng.
    if case == 'A51':
        with yolo_running(cfg, root, yolo_schedule=((1, 'empty'), (90, 'green'), (180, 'red_edge'))) as r:
            r.boot()
            r.wait(lambda: last_detection_frame(r.events()) >= 195, 25, 'Chưa có đủ ba pha CSV')
            r.finish(); inspect_detections(r)
        return
    # A45 — B4/P2: ngoại lệ Planner một lần không thoát, chốt0 tới G mới.
    if case == 'A45':
        with yolo_running(cfg, root, values={'start_wait_green': 0}) as r:
            r.boot(); r.go(); mark = len(r.events()); ev = r.inject('planner_throw')
            stopped(r, ev[0], 150, mark=mark)
            r.wait(lambda: any(e[2] == 'LOI_DIEU_KHIEN' for e in r.of('reason', mark)),
                   message='Planner throw thiếu LOI_DIEU_KHIEN')
            end = stop_tail(r, ev[0], mark); r.frames(8, lambda: held(r, end))
            need(r.process.poll() is None, 'Planner throw làm thoát tiến trình')
            r.go(); r.finish()
        return
    # A46 — hợp đồng P2/B4: biến lỗi nạp KHÔNG mang tiền tố FAKE.
    if case == 'A46':
        with yolo_running(cfg, root, model_load_fail='yolo') as r:
            reject(r)
        return
    # A47 — H6/P2: --no-yolo bỏ chờ xanh và không đọc YOLO, cả sau R.
    if case == 'A47':
        values = dict(YOLO_VALUES)
        with running(cfg, root, values=values, yolo_schedule=((1, 'red_edge'),),
                     model_load_fail='yolo', setup=lambda r: (r.root / 'yolo.csv').write_text('hong\n', encoding='utf-8')) as r:
            r.boot(); r.go(); speed_is(r, 8)
            need(not r.of('detections'), '--no-yolo vẫn suy luận YOLO')
            end = space(r); mark = len(r.events())
            patch_yaml(r, values); reload(r, 'accepted')
            r.frames(5, lambda: no_motion(r, end))
            r.go(); speed_is(r, 8)
            need(not any(e[2] in ('CHO_DEN_XUAT_PHAT', 'DUNG_DEN_DO') for e in r.of('reason', mark)),
                 '--no-yolo áp luật đèn sau R')
            r.finish()
        return
    # A48 — hợp đồng fixture P2: lỗi CSV/NPY không được che bằng lô rỗng.
    if case == 'A48':
        def wrong_tensor(run, old, new):
            path = run.root / 'empty.npy'
            path.write_bytes(path.read_bytes().replace(old, new, 1))
        bad = [lambda r: (r.root / 'yolo.csv').unlink(),
               lambda r: (r.root / 'yolo.csv').write_text('frame_id,tensor\nsai,empty.npy\n', encoding='utf-8'),
               lambda r: (r.root / 'empty.npy').unlink(),
               lambda r: (r.root / 'empty.npy').write_bytes(b'hong'),
               lambda r: wrong_tensor(r, b'<f4', b'<i4'),
               lambda r: wrong_tensor(r, b'(1, 13, 100)', b'(1, 12, 100)')]
        for i, setup in enumerate(bad):
            with yolo_running(cfg, root / str(i), setup=setup) as r:
                reject(r)
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
                if kind == 'road':
                    for _ in range(5):
                        r.wait(lambda: 0 <= td.now_ms() - float(str(r.of('mask')[-1][2]).split()[1])
                               <= r.values['mask_max_age_ms'],
                               3 + r.slack / 1000.0, 'Chưa có mask tươi trước G', tag='setup')
                        mark = len(r.events()); r.key('G')
                        result = r.wait(lambda: next(iter(r.of('go', mark)), None)
                                        or next((e for e in r.of('motor_speed', mark)
                                                 if e[2] > 0 and not e[3]), None),
                                        3 + r.slack / 1000.0, 'G tươi không tạo chuyển động')
                        if result[1] == 'go':
                            need(result[2] in ('ignored MAT_NAN_CU', 'ignored MAT_ANH'),
                                 'G bị bỏ vì lý do khác: ' + str(result[2]), 'go_fresh')
                        else:
                            break
                    else:
                        raise Failure('go_fresh', 'G vẫn bị bỏ sau 5 lần chờ mask tươi')
                else:
                    r.go()
                r.finish()
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
    raise Failure('setup', 'Chưa có ca ' + case)


def selftest():
    import ast
    from types import SimpleNamespace
    from unittest.mock import MagicMock, patch
    def fails(call, tag):
        try:
            call()
        except Failure as error:
            assert error.tag == tag
        else:
            assert False, 'Bộ kiểm bỏ lọt ' + tag
    assert len(P2_BOUNDS) == 12
    assert not {'conf_min', 'box_min_area_frac', 'box_min_area_px'} & set(YOLO_VALUES)
    for light in ('red', 'green', 'off'):
        assert YOLO_VALUES[light + '_conf_min'] == 0.5
        assert YOLO_VALUES[light + '_box_min_area_frac'] == 0.0001
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
    assert last_detection_frame([]) == 0
    assert last_detection_frame(straight) == 0
    for frame in (119, 120, 121):
        events = [(200, 'detections', '{} 190'.format(frame), False)]
        assert last_detection_frame(events) == frame
        assert (last_detection_frame(events) < 120) == (frame < 120)
    # Fixture tự chứa: đối chiếu header NPY, xác suất và đổi ngược letterbox.
    with tempfile.TemporaryDirectory(prefix='ucr-yolo-selftest-') as tmp:
        root = Path(tmp)
        yolo_fixtures(root, ((1, 'empty'), (120, 'green'), (210, 'red_edge')))
        info = json.loads((root / 'yolo_info.json').read_text(encoding='utf-8'))
        assert info['class_names'] == YOLO_CLASSES and info['output_shape'] == [1, 13, 100]
        assert (root / 'yolo.csv').read_text(encoding='utf-8').splitlines() == [
            'frame_id,tensor', '1,empty.npy', '120,green.npy', '210,red_edge.npy']
        for mode, boxes in YOLO_BOXES.items():
            data = (root / (mode + '.npy')).read_bytes()
            assert data[:8] == b'\x93NUMPY\x01\x00'
            length = struct.unpack('<H', data[8:10])[0]
            assert (10 + length) % 16 == 0
            assert ast.literal_eval(data[10:10 + length].decode('ascii')) == dict(
                descr='<f4', fortran_order=False, shape=(1, 13, 100))
            values = struct.unpack('<1300f', data[10 + length:])
            assert sum(v > 0.5 for v in values[400:]) == len(boxes)
            if not boxes:
                assert not any(values)
            for proposal, (cls, expected_x2) in enumerate(boxes):
                assert abs(values[(4 + YOLO_CLASSES.index(cls)) * 100 + proposal] - 0.9) < 1e-6
                x2 = (values[proposal] + values[200 + proposal] / 2) / 0.25
                assert abs(x2 - expected_x2) < 0.001
                assert (values[100 + proposal] - 70) / 0.25 == 300
                assert values[200 + proposal] * values[300 + proposal] / (0.25 ** 2 * 1280 * 720) > 0.0001
        csv_events = [(100, 'detections', '1 90', False), (100, 'detections', '90 95', False),
                      (100, 'detections', '180 99', False)]
        csv_run = SimpleNamespace(session=lambda: root, events=lambda: csv_events)
        csv_rows = [['1', '90', '', '', '', '', '', ''],
                    ['90', '95', 'green', '0.9', '648', '200', '800', '400'],
                    ['180', '99', 'red', '0.9', '1000', '200', '1152', '400']]
        def write_csv(rows, header='frame_id captured_ms class conf x1 y1 x2 y2'):
            with (root / 'detections.csv').open('w', encoding='utf-8', newline='') as stream:
                writer = csv.writer(stream)
                writer.writerow(header.split()); writer.writerows(rows)
        write_csv(csv_rows); inspect_detections(csv_run)
        for row, col, value in ((0, 3, '0'), (1, 1, '96'), (1, 6, '200'), (2, 2, 'green'), (2, 3, 'nan')):
            bad = [list(r) for r in csv_rows]; bad[row][col] = value
            write_csv(bad)
            fails(lambda: inspect_detections(csv_run), 'record_format')
        write_csv(csv_rows, header='frame_id captured_ms class conf x1 y1 x2')
        fails(lambda: inspect_detections(csv_run), 'record_format')
        hooks = root / 'hooks'
        hooks.write_bytes(b'UCR_AUTO_FAKE_TRACE')
        cfg = SimpleNamespace(hooks=str(hooks), slack=0)
        with patch.object(subprocess, 'Popen') as popen:
            run = Run(cfg, root / 'launch', no_yolo=False, yolo_schedule=((1, 'empty'),), model_load_fail='yolo')
            try:
                argv = popen.call_args[0][0]
                env = popen.call_args[1]['env']
                assert '--no-yolo' not in argv and '--yolo-engine' in argv and '--yolo-info' in argv
                assert env['UCR_AUTO_MODEL_LOAD_FAIL'] == 'yolo'
                assert env['UCR_AUTO_FAKE_YOLO_TENSORS'] == str(run.root / 'yolo.csv')
                assert 'UCR_AUTO_FAKE_MODEL_LOAD_FAIL' not in env
            finally:
                run.log.close()
    # Mốc cùng ms: lô mới đếm bằng frame_id; lệnh trước lô thứ3 không được lọt.
    events = [(100, 'detections', '3 90', False), (100, 'motor_speed', 0, False),
              (100, 'servo_angle', 0, False), (100, 'detections', '6 95', False),
              (100, 'detections', '9 99', False), (100, 'motor_speed', 8, False),
              (100, 'reason', 'DEN_XANH', False)]
    class SyntheticRun:
        def __init__(self, es):
            self.es = es
            self.slack = 0
            self.values = {'control_period_ms': 20}

        def events(self):
            return self.es

        def of(self, kind, mark=0):
            return [e for e in self.es[mark:] if e[1] == kind]

        def wait(self, predicate, *args, **kwargs):
            result = predicate()
            assert result, kwargs.get('message', 'Không đạt điều kiện tổng hợp')
            return result
    assert green_release(SyntheticRun(events), 3, 0)[1] == 9
    # Không nhận lô có ảnh chụp trước G dù công bố sau G.
    since_events = [(100, 'detections', '1 89', False)] + events
    assert green_release(SyntheticRun(since_events), 1, 0, since=89)[1] == 9
    assert len(phase_batches(SyntheticRun(events), 3, 9, 2)) == 2
    fails(lambda: phase_batches(SyntheticRun(events), 3, 9, 3), 'setup')
    moving(SyntheticRun([(100, 'motor_speed', 8, False)]), 0)
    fails(lambda: moving(SyntheticRun(events), 0), 'red_boundary')
    for bad, tag in ((events[:3] + [(100, 'detections', '3 90', False)], 'interface'),
                     (events[:3] + [(100, 'motor_speed', 8, False)] + events[3:], 'green_frames')):
        try:
            green_release(SyntheticRun(bad), 3, 0)
        except Failure as error:
            assert error.tag == tag
        else:
            assert False, 'Bỏ lọt lô lặp/chạy trước đủ xanh'
    try:
        held(SyntheticRun(events[:3] + [(101, 'motor_speed', 1, False)]), 3)
    except Failure as error:
        assert error.tag == 'light_latch'
    else:
        assert False, 'Bỏ lọt chuyển động khi chốt đỏ'
    red = [(99, 'motor_speed', 8, False), (100, 'detections', '3 90', False),
           (101, 'detections', '6 95', False), (102, 'detections', '9 99', False),
           (103, 'motor_speed', 0, False), (104, 'servo_angle', 0, False),
           (104, 'reason', 'DUNG_DEN_DO', False)]
    assert red_stop(SyntheticRun(red), 3, 0) == 6
    # Hồi quy review: không được bỏ đoạn motor8→0 đã đến cùng lần đọc trace.
    bad_red = red[:6] + [(105, 'motor_speed', 8, False), (106, 'motor_speed', 0, False),
                         (107, 'servo_angle', 0, False)] + red[6:]
    try:
        red_stop(SyntheticRun(bad_red), 3, 0)
    except Failure as error:
        assert error.tag == 'light_latch'
    else:
        assert False, 'Bỏ lọt tái chạy sau cặp dừng'
    # A32: kiểm vòng chờ/bấm lại thật bằng vết tổng hợp, không cần binary hooks.
    for rejections, expected in (([], None),
                                 (['ignored MAT_NAN_CU', 'ignored MAT_ANH'], None),
                                 (['ignored MAT_NAN_CU'] * 4, None),
                                 (['ignored MAT_NAN_CU'] * 5, 'go_fresh'),
                                 (['ignored LOI_DIEU_KHIEN'], 'go_fresh')):
        runs = []
        @contextmanager
        def fake_running(cfg, root, **kwargs):
            run = MagicMock(slack=0, values=kwargs['values'])
            es = [(0, 'frame', '1 0', False), (0, 'mask', '1 0', False)]
            attempts = []
            resumed = []
            run.events.side_effect = lambda: es
            run.of.side_effect = lambda kind, mark=0: [e for e in es[mark:] if e[1] == kind]
            def inject(command):
                if command.endswith('_resume'):
                    resumed.append(True)
            run.inject.side_effect = inject
            def wait(predicate, *args, **kwargs):
                result = predicate()
                if not result and resumed:
                    es.append((1000, 'mask', '2 1000', False))
                    result = predicate()
                assert result, kwargs.get('message', 'Vòng chờ tổng hợp không đạt')
                return result
            run.wait.side_effect = wait
            run.frames.side_effect = lambda count, check: check()
            def key(value):
                ev = (1000, 'key', value, False)
                es.append(ev)
                if not resumed:
                    es.append((1000, 'go', 'ignored MAT_NAN_CU', False))
                else:
                    assert 0 <= td.now_ms() - float(run.of('mask')[-1][2].split()[1]) <= 100
                    index = len(attempts)
                    attempts.append(value)
                    es.append((1000, 'go', rejections[index], False) if index < len(rejections)
                              else (1000, 'motor_speed', 8, False))
                    # Lần sau phải chờ mask tươi lại, không dùng lần chờ trước.
                    es.append((1000, 'mask', '3 0', False))
                return ev
            run.key.side_effect = key
            runs.append((root.name, run, attempts))
            yield run
        with patch('{}.running'.format(__name__), fake_running), patch.object(td, 'now_ms', return_value=1000):
            call = lambda: test_case('A32', None, Path('synthetic'))
            if expected:
                fails(call, expected)
            else:
                call()
        assert runs[0][0] == 'camera' and runs[0][1].go.call_count == 1
        assert runs[1][0] == 'road' and runs[1][1].go.call_count == 0
        assert len(runs[1][2]) == (len(rejections) if expected else len(rejections) + 1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--selftest', action='store_true')
    parser.add_argument('--case', choices=CASES + ['B5'])
    parser.add_argument('--hooks')
    parser.add_argument('--slack', type=int, choices=(0, 300), default=0)
    cfg = parser.parse_args()
    if cfg.selftest:
        selftest()
        print('PASS selftest: P1/M7, fixture YOLO, CLI/env, lô mới, chốt đèn và A32 bấm lại G (vết tổng hợp)')
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
