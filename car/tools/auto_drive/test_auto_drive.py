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
          'A40b', 'A42b', 'A43b', 'A49', 'A50', 'A51',
          'A70u', 'A70y', 'A71u', 'A71y', 'A72u', 'A72y', 'A73',
          'A74ua', 'A74ub', 'A74ya', 'A74yb', 'A75', 'A77', 'A78', 'A79', 'A80', 'A81', 'A60']
CASES += ['t57_disabled_equivalence', 't57_open_imu', 't57_open_imu_drift',
          't57_read_imu', 't57_read_encoder', 't57_reload', 't57_stale_imu',
          't57_exit_turning', 't57_lost_image_turning', 't57_space_during_turn',
          't57_g_pending_after_fault', 't57_reload_during_turn', 't57_init_ok_order',
          't57_transient_imu_armed', 't57_transient_encoder_armed',
          't57_transient_imu_turning', 't57_transient_encoder_turning',
          't57_transient_idle', 't57_csv_imu', 't57_csv_encoder', 't57_csv_both']
CASES += ['t62_hold_170', 't62_hold_200', 't62_green_approach',
          't62_reset_space', 't62_reset_go', 't62_reset_image',
          't62_stop_089', 't62_stop_090', 't62_stop_091',
          't62_stop_flicker', 't62_no_light', 't62_green_latch']
CASES += ['t71_defaults', 't71_steer_40', 't71_steer_40_1', 't71_cruise_15',
          't71_cruise_16', 't71_motor_cap', 't71_curve_30',
          't72_zero_valid', 't72_negative_rejected', 't72_zero_keeps_text', 't72_reload_0_to_5',
          't73_default_enable', 't73_init_imu_fail', 't73_init_drift_fail', 't73_reload_no_imu',
          't73_imu_turning', 't73_encoder_turning', 't73_stale_imu_turning',
          't73_stale_encoder_turning', 't73_imu_lane', 't73_encoder_lane',
          't73_g_reopens_dung', 't73_lost_image_turning', 't73_stale_mask_turning']
T73_ADDED_CASES = ['t73_turn_max_cm_exceeded', 't73_turn_max_ms_exceeded',
                   't73_stuck_turning', 't73_stale_armed_lane',
                   't73_motor_init_once_imu_fail', 't73_fault_stop_order',
                   't73_fault_then_lost_image']
CASES += T73_ADDED_CASES
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
                  red_091=[('red', 0.91 * 1280)],
                  red_050=[('red', 0.50 * 1280)], red_085=[('red', 0.85 * 1280)],
                  red_green=[('red', 1152), ('green', 800)])
BOUNDS = {
    'control_period_ms': (50, 20, 100, 'ms'), 'mask_max_age_ms': (300, 50, 1000, 'ms'),
    'cmd_max_age_ms': (200, 50, 500, 'ms'), 'det_max_age_ms': (500, 50, 2000, 'ms'),
    'max_speed_cms': (10, 0, 20, 'cm/s'), 'cruise_cms': (8, 0, 20, 'cm/s'),
    'min_cms': (4, 0, 10, 'cm/s'), 'lost_cms': (4, 0, 10, 'cm/s'),
    'slow_per_deg': (0.1, 0, 1, 'cm/s/'), 'steer_max_deg': (30, 0, 40, ''),
    'steer_rate_dps': (200, 10, 1000, '/s'), 'kp': (30, 0, 200, ''),
    'kd': (2, 0, 50, ''), 'lane_width_min_ratio': (0.6, 0.1, 1, ''),
    'lane_width_max_ratio': (1.4, 1, 3, ''), 'center_offset_frac': (0, -0.5, 0.5, ''),
    'min_valid_rows': (1, 1, 5, ''), 'lost_hold_ms': (300, 0, 2000, 'ms'),
    'lost_stop_ms': (800, 0, 3000, 'ms'), 'regain_frames': (3, 1, 20, ''),
    'record_every_n': (5, 0, 40, ''), 'yolo_every_n': (3, 1, 10, '')}
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
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', len(rows[0]), len(rows), 8, 0, 0, 0, 0)) +
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


def model_files(models, name, info):
    """T47l: file thật để kiểm nguồn; engine hooks vẫn lấy mask/tensor từ env."""
    models.mkdir(exist_ok=True)
    onnx = b'fake-' + name.encode('ascii') + b'-onnx\n'
    (models / (name + '.onnx')).write_bytes(onnx)
    (models / (name + '_info.json')).write_text(json.dumps(info), encoding='utf-8')
    (models / (name + '.engine')).write_bytes(b'fake-' + name.encode('ascii') + b'-engine\n')
    (models / (name + '.engine.src')).write_bytes((hashlib.sha256(onnx).hexdigest() + '\n').encode('ascii'))


def fixtures(root, schedule):
    for mode in ('straight', 'left', 'right', 'empty'):
        rows = mask_rows(mode)
        need(len(rows) == 96 and all(len(r) == 160 and set(r) <= {0, 1, 2} for r in rows),
             'Fixture sai cỡ/lớp', 'setup')
        (root / (mode + '.png')).write_bytes(png_bytes(rows))
    (root / 'list.csv').write_text('frame_id,mask\n' + ''.join(
        '{},{}.png\n'.format(frame, mode) for frame, mode in schedule), encoding='utf-8')
    model_files(root / 'models', 'unet', road_fixture.info(96, 160))


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
    model_files(root / 'models', 'yolo', info)
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
            limit = 20 if kind == 'motor_speed' else 40
            need(math.isfinite(value) and abs(value) <= limit, 'Vượt trần cứng: {} {}'.format(kind, value), 'invariant_cap')


class Run(td.Run):
    """Dùng lại wait/observe/text/send/ack/children/close của T33; chỉ đổi khởi chạy auto."""
    def __init__(self, cfg, root, values=None, env=None, schedule=((1, 'straight'),),
                 args=None, lock=None, setup=None, no_yolo=True, stderr=None,
                 yolo_schedule=None, model_load_fail=None, use_home_models=False):
        need(b'UCR_AUTO_FAKE_TRACE' in Path(cfg.hooks).read_bytes(),
             'Từ chối binary không có hooks auto', 'environment')
        self.root, self.slack, self.seq = root, cfg.slack, 0
        self.models = root / 'models'
        self.no_yolo = no_yolo
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
        if use_home_models:
            home = root / 'home'
            home.mkdir()
            shutil.move(str(self.models), str(home / 'ucr_models'))
            self.models = home / 'ucr_models'
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
        if use_home_models:
            child_env['HOME'] = str(home)
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
        if not use_home_models:
            argv.extend(['--model-dir', str(self.models)])
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
    case = getattr(cfg, 'case', '')
    legacy_defaults = None
    if (case.startswith('A') or case == 'B5') and not (case == 'A08' and not kwargs.get('values')):
        # T71: các ca cũ giữ cấu hình cũ tường minh, ca mặc định thật vẫn dùng YAML rỗng.
        legacy_defaults = dict(cruise_cms=8, max_speed_cms=10, steer_max_deg=30,
                               slow_per_deg=0.1, turn_enable=0, obstacle_action=0)
        values = dict(legacy_defaults)
        values.update(kwargs.get('values') or {})
        kwargs['values'] = values
    run = Run(cfg, root, **kwargs)
    if legacy_defaults is not None:
        run.legacy_defaults = legacy_defaults
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


def reject_model(run):
    """T47l mục 1–3: lỗi model phải trước cả helper và lệnh dừng phần cứng."""
    reject(run)
    need(not any(e[1] in ('motor_init', 'motor_speed', 'motor_emergency', 'servo_angle', 'test_ucr_stop')
                 for e in run.events()), 'Lỗi model vẫn tác động phần cứng', 'model_preflight')


def stale_onnx(run, name):
    path = run.models / (name + '.onnx')
    original = path.read_bytes()
    changed = bytes([original[0] ^ 1]) + original[1:]
    path.write_bytes(changed)
    need(sum(a != b for a, b in zip(original, changed)) == 1, 'Phải sửa đúng 1 byte', 'setup')
    need((run.models / (name + '.engine.src')).read_bytes() ==
         (hashlib.sha256(original).hexdigest() + '\n').encode('ascii'), 'src phải giữ băm cũ', 'setup')


def inspect_model_session(run):
    data = json.loads((run.session() / 'session.json').read_text(encoding='utf-8'))
    for name in ('unet', 'yolo'):
        enabled = name == 'unet' or not run.no_yolo
        info = json.loads((run.models / (name + '_info.json')).read_text(encoding='utf-8')) if enabled else {}
        for field in ('variant', 'seed', 'data_tag', 'onnx_sha256'):
            key = name + '_' + field
            need(key in data, 'session thiếu ' + key, 'model_session')
            expected = ''
            if enabled:
                expected = (hashlib.sha256((run.models / (name + '.onnx')).read_bytes()).hexdigest()
                            if field == 'onnx_sha256' else info[field])
            need(data[key] == expected, 'session sai ' + key, 'model_session')
        key = 'engine_sha256' if name == 'unet' else 'yolo_engine_sha256'
        need(key in data, 'session mất khóa cũ ' + key, 'model_session')
        if enabled:
            need(data[key] == hashlib.sha256((run.models / (name + '.engine')).read_bytes()).hexdigest(),
                 'session sai băm engine ' + key, 'model_session')


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


def red_stop(run, first_frame, mark, consumed=False):
    batches = run.wait(lambda: [b for b in detection_batches(run.events())
                               if b[0] >= mark and b[1] >= first_frame][2:],
                       25, 'Thiếu 3 lô đỏ mới')
    third = batches[0]
    first = next(b for b in detection_batches(run.events()) if b[0] >= mark and b[1] >= first_frame)
    stamp, stop_mark = third[3], third[0] + 1
    if consumed:
        samples = t62_consumed(run, [first] + [b for b in detection_batches(run.events())
                                if first[1] < b[1] <= third[1]])
        stamp, stop_mark = t62_control_mark(run, third, samples[-1])
        need(stamp - first[2] < 500, 'Phiếu đỏ đã hết tuổi trước vòng tiêu thụ', 'setup')
    need(not any(motor_zero(e) for e in run.events()[first[0]:stop_mark - 1]),
         'Dừng đỏ trước khi đủ 3 phiếu mới', 'red_votes')
    stopped(run, stamp, run.values['control_period_ms'] + 200, mark=stop_mark)
    run.wait(lambda: any(e[2] == 'DUNG_DEN_DO' for e in run.of('reason', mark)),
             message='Thiếu reason DUNG_DEN_DO')
    # Bắt luôn chuyển động lọt giữa cặp dừng và lần đọc trace này.
    end = stop_tail(run, stamp, stop_mark)
    held(run, end)
    return end


def phase_batches(run, lo, hi, count):
    batches = [b for b in detection_batches(run.events()) if lo <= b[1] < hi]
    need(len(batches) == count, 'Lịch cần {} lô trong [{},{}), thực tế {}'.format(
        count, lo, hi, [b[1] for b in batches]), 'setup')
    return batches


def moving(run, mark):
    speeds = [e[2] for e in run.of('motor_speed', mark) if not e[3]]
    max_speed = min(run.values.get('max_speed_cms', BOUNDS['max_speed_cms'][0]), 20)
    need(not any(motor_zero(e) for e in run.events()[mark:]) and all(0 < v <= max_speed for v in speeds),
         'Dừng đỏ khi chưa đủ phiếu/vị trí hoặc vượt trần max_speed_cms', 'red_boundary')


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
    if not isinstance(values, str):
        control = dict(getattr(run, 'legacy_defaults', {}))
        control.update(values)
        values = yaml_text(control)
    run.control.write_text(values, encoding='utf-8')


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


def cap20_curve(rows):
    curved = [row for row in rows if abs(float(row['requested_steer_deg'])) >= 20]
    need(len(curved) >= 3, 'Thiếu mẫu cua góc lớn', 'cap20_curve')
    for row in curved:
        expected = 20 - 0.1 * abs(float(row['requested_steer_deg']))
        need(all(abs(float(row[key]) - expected) < 0.01
                 for key in ('requested_speed_cms', 'sent_speed_cms')),
             'Tốc độ cua không giảm theo slow_per_deg mặc định', 'cap20_curve')


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
    need(data['info_json'] == json.loads((run.models / 'unet_info.json').read_text(encoding='utf-8')),
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
        need(abs(float(row['sent_speed_cms'])) <= 20 and abs(float(row['sent_steer_deg'])) <= 40, 'CSV vượt trần')
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
    if git is None:
        ignored = False
        for line in (HERE.parents[1] / '.gitignore').read_text(encoding='utf-8').splitlines():
            line = line.strip()
            if line.startswith('#'):
                continue
            if line in ('runs_auto/', '/runs_auto/', 'runs_auto'):
                ignored = True
            elif line.startswith('!runs_auto'):
                ignored = False
        need(ignored, 'runs_auto chưa được bỏ qua trong ucr_car/.gitignore', 'gitignore')
        return
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


# Header cũ đầy đủ từ log_writer.cpp tại cây nền; khóa cả vị trí cột.
T57_OLD_FIELDS = ('t_ms frame_id mask_file cmd_cms cmd_deg mask_ms image_ms go stop lane_ref planner_state '
                  'image_age_ms mask_age_ms det_age_ms scan_rows e p d requested_steer_deg requested_speed_cms '
                  'sent_steer_deg sent_speed_cms reason road_ms yolo_ms loop_ms dropped csv_dropped '
                  'camera_fps loop_p95_ms rss_mb free_mb command_id command_ms mask_id det_frame_id '
                  'det_captured_ms detections_dropped').split()


def t57_log(rows):
    """Chỉ nối hai cột cuối; khi tắt để trống cả dòng thoát."""
    need(rows, 'Thiếu CSV để kiểm T57', 't57_log')
    for row in rows:
        need(list(row) == T57_OLD_FIELDS + ['yaw_deg', 'odo_cm'],
             'T57 đổi/mất/thừa/đảo cột cũ hoặc không nối cuối', 't57_log')
        need(row['yaw_deg'] == row['odo_cm'] == '', 'Tắt nhưng ghi số cảm biến', 't57_log')


def t57_fault_csv(rows, failed, odo=0):
    """Kiểm đúng mẫu lỗi trong lệnh, trước khi cấp lại mẫu tốt."""
    faults = [row for row in rows if row['reason'] == 'HUY_RE_CAM_BIEN']
    need(faults, 'CSV chưa có mẫu HUY_RE_CAM_BIEN', 't57_fault_csv')
    for row in faults:
        need(row['planner_state'] == 'BAM_LAN' and
             0 < float(row['sent_speed_cms']) <= 15 and abs(float(row['sent_steer_deg'])) <= 40,
             'CSV hủy rẽ không bám làn trong giới hạn', 't57_fault_csv')
        for sensor, column in (('imu', 'yaw_deg'), ('encoder', 'odo_cm')):
            value = row[column]
            need(value == '' if sensor in failed else value != '' and float(value) == (0 if sensor == 'imu' else odo),
                 'CSV sai ô {} của mẫu lỗi {}'.format(column, failed), 't57_fault_csv')


def t57_sample(run):
    if run.t57_imu:
        run.inject('imu_sample {} 0'.format(getattr(run, 't57_yaw', 0)))
    if run.t57_encoder:
        run.inject('encoder_sample {}'.format(getattr(run, 't57_odo', 0)))


def t57_feed(run):
    # Cấp mẫu age_ms=0 mỗi lượt chờ, gồm boot/key/frames/stop; không ngủ cố định.
    original = run.wait
    run.t57_imu = run.t57_encoder = True
    feeding = [False]
    def wait(predicate, *args, **kwargs):
        def poll():
            if not feeding[0] and run.process.poll() is None:
                feeding[0] = True
                try:
                    t57_sample(run)
                except td.Failure:
                    # Ca đã cho chương trình thoát (phím E/tín hiệu) giữa hai lần bơm: bỏ mẫu, không phải lỗi.
                    if run.process.poll() is None:
                        raise
                finally:
                    feeding[0] = False
            return predicate()
        return original(poll, *args, **kwargs)
    run.wait = wait


def t57_wait(run, predicate, message):
    return run.wait(predicate, 10, message)


def t68a_armed(run, mark, since):
    """Giữ encoder tại mốc0 tới khi biển đủ phiếu; J1 không quyết định rẽ."""
    def enough():
        need(not any(e[2] in ('RE_TRAI', 'RE_PHAI') for e in run.of('state', mark)),
             'Rẽ khi encoder chưa tới turn_start_cm', 't68a_start')
        batches = [b for b in detection_batches(run.events()) if b[0] >= mark and
                   b[2] > since and td.now_ms() - b[2] <= run.values.get('det_max_age_ms', 500)]
        return len(batches) >= run.values.get('vote_min', 3)
    need(getattr(run, 't57_odo', 0) == 0, 'Encoder phải giữ mốc0 trước đủ phiếu', 'setup')
    run.wait(enough, 10, 'Chưa đủ phiếu biển tươi sau G')
    run.frames(3, enough)


def t68a_turn(run, mark, since, direction='left'):
    t68a_armed(run, mark, since)
    run.t57_odo = run.values.get('turn_start_cm', 30)
    return run.wait(lambda: next((e for e in run.of('state', mark) if
                    e[2] == ('RE_TRAI' if direction == 'left' else 'RE_PHAI')), None),
                    10, 'Chưa vào rẽ sau đủ phiếu và turn_start_cm')


def t57_transient_window(stalled, resumed, created, observed=None, events=()):
    """Không cho bộ giám sát tuổi lệnh che lỗi cảm biến trong cửa sổ Planner."""
    need(0 <= resumed - stalled < 500 and 0 <= resumed - created < 500,
         'Stall Planner chạm cmd_max_age_ms=500', 't57_transient')
    if observed is not None:
        need(resumed <= observed < created + 500 and
             any(e[1] == 'command' and float(str(e[2]).split()[1]) == observed for e in events),
             'Thiếu vòng Planner mới trước khi lệnh cũ hết tuổi', 't57_transient')
        need(not any(motor_zero(e) or (e[1] == 'motor_speed' and e[2] <= 0 and not e[3]) or
                     (e[1] == 'state' and e[2] == 'DUNG') for e in events),
             'Lỗi thoáng làm dừng/chốt', 't57_transient')


def t57_case(case, cfg, root):
    if case == 't57_disabled_equivalence':
        # T73–5: tắt tường minh, thêm tham số rẽ vẫn giữ nguyên lệnh làn.
        snapshots = []
        for explicit in (False, True):
            values = dict(turn_enable=0, cruise_cms=8, max_speed_cms=10, steer_max_deg=30,
                          obstacle_action=0, kp=0, kd=0, slow_per_deg=0, start_wait_green=0)
            if explicit:
                values.update(turn_yaw_deg=80, turn_steer_deg=25, turn_arm_cm=80,
                              turn_max_cm=70, turn_max_ms=8000)
            with running(cfg, root / str(explicit), values=values, env={'OPEN_FAIL': 'imu'}) as r:
                r.boot(); started = r.go(); mark = r.events().index(started)
                r.frames(6)
                commands = [e for e in r.events()[mark:] if e[1] in ('motor_speed', 'servo_angle')]
                need(commands and all(not e[3] and e[2] == (8 if e[1] == 'motor_speed' else 0)
                                      for e in commands), 'Tắt đổi lệnh làn thẳng cũ', 't57_disabled')
                space(r); r.t57_imu = r.t57_encoder = False; r.finish()
                need(not r.of('imu_init') and not r.of('imu_drift'), 'Tắt vẫn init IMU', 't57_disabled')
                rows = r.rows(); t57_log(rows)
                stable = [row for row in rows if row['planner_state'] == 'BAM_LAN' and row['reason'] == 'BAM_LAN']
                need(stable, 'Thiếu đoạn bám làn trong CSV', 't57_log')
                # Cột thời gian/đếm khung không xác định giữa hai tiến trình thật.
                fields = ['planner_state', 'scan_rows', 'e', 'p', 'd', 'requested_steer_deg',
                          'requested_speed_cms', 'sent_steer_deg', 'sent_speed_cms', 'reason']
                snapshots.append((list(rows[0])[:-2], {tuple(row[k] for k in fields) for row in stable}))
        need(snapshots[0] == snapshots[1], 'Tham số rẽ đổi cột cũ khi turn_enable=0', 't57_disabled')
        return
    if case.startswith('t57_open_'):
        # T73–1,6: dùng lại kịch bản init lỗi vẫn chạy, cảnh báo và bỏ rẽ.
        return t73_case('t73_init_imu_fail' if case == 't57_open_imu' else 't73_init_drift_fail', cfg, root)
    if case == 't57_reload':
        # T73–4: R áp cruise=9 nhưng không tự init IMU/bật rẽ.
        return t73_case('t73_reload_no_imu', cfg, root)
    def setup(run):
        run.ctrl.write_text('1 imu_sample 0 0\n2 encoder_sample 0\n', encoding='utf-8')
        run.seq = 2

    mode = 't57_left'
    previous = YOLO_BOXES.get(mode)
    YOLO_BOXES[mode] = [('left', 800)]
    modes = ('term', 'esc', 'kill_preview') if case == 't57_exit_turning' else ('single',)
    idle = case == 't57_transient_idle'
    if idle:
        modes = ('imu', 'encoder')
    values = {'turn_enable': 1, 'start_wait_green': 0, 'vote_min': 1,
              'yolo_every_n': 1, 'mask_max_age_ms': 1000}
    if case.startswith('t57_transient_'):
        values['cmd_max_age_ms'] = 500
    try:
        for action in modes:
            with running(cfg, root / action, values=values,
                         setup=setup, no_yolo=False,
                         yolo_schedule=((1, 'empty'), (120, mode)) if idle else ((1, mode),)) as r:
                t57_feed(r); r.boot()
                if case == 't57_init_ok_order':
                    kinds = [e[1] for e in r.events()]
                    need(all(k in kinds for k in ('imu_init', 'imu_drift', 'motor_init')) and
                         kinds.index('imu_init') < kinds.index('imu_drift') < kinds.index('motor_init'),
                         'Sai thứ tự init IMU/đo trôi/motor', 't57_init')
                    need(not r.of('tof_init') and not r.of('motor_config'),
                         'Auto init ToF/gửi cấu hình ngoài Motor::Init', 't57_init')
                    r.t57_imu = r.t57_encoder = False
                    r.finish(); continue
                mark = len(r.events()); started = r.key('G')
                if case.endswith('_armed') or idle:
                    t57_wait(r, lambda: any(e[2] == 'BAM_LAN' for e in r.of('state', mark)),
                             'Chưa bám làn trước turn_start_cm')
                    if not idle:
                        t68a_armed(r, mark, started[0])
                    r.frames(4)
                    need(not any(e[2] in ('RE_TRAI', 'RE_PHAI') for e in r.of('state', mark)),
                         'Chưa tới turn_start_cm vẫn vào rẽ', 'setup')
                else:
                    t68a_turn(r, mark, started[0])
                need(any(e[1] == 'motor_speed' and e[2] > 0 and not e[3] for e in r.events()[mark:]),
                     'Tiền đề chưa có lệnh chạy', 'setup')
                mark = len(r.events())
                if case == 't57_exit_turning':
                    r.t57_imu = r.t57_encoder = False
                    if action == 'esc':
                        r.finish(key='\x1b')
                    else:
                        stamp = external_stamp_ms()
                        if action == 'term':
                            os.kill(r.process.pid, signal.SIGTERM)
                        else:
                            windows = sorted(pid for pid in r.children() if td.nice(pid) == 10)
                            need(windows, 'Thiếu preview chính', 'setup')
                            os.kill(windows[0], signal.SIGKILL)
                        stopped(r, stamp, 1000, mark=mark)
                        r.exited(0 if action == 'term' else 1)
                    import fcntl
                    with (r.root / 'vehicle.lock').open('a') as lock:
                        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    continue
                if case == 't57_reload_during_turn':
                    patch_yaml(r, {'turn_enable': 0, 'min_cms': 2})
                    reload(r, 'ignored')
                    def unchanged():
                        need(not any(e[2] != 'RE_TRAI' for e in r.of('state', mark)),
                             'R đổi trạng thái đang rẽ', 't57_reload')
                        need(all(e[3] or e[2] == 4 for e in r.of('motor_speed', mark)),
                             'R áp dụng YAML khi đang rẽ', 't57_reload')
                    r.frames(4, unchanged); unchanged(); space(r); r.t57_imu = r.t57_encoder = False; r.finish(); continue
                if case == 't57_space_during_turn':
                    # Chặn biển mới và đợi phiếu cũ hết tuổi trước G.
                    r.inject('yolo_stall')
                    last = float(str(r.of('detections')[-1][2]).split()[1])
                    t57_wait(r, lambda: td.now_ms() > last + 500, 'Phiếu cũ chưa hết tuổi')
                    space(r); mark = len(r.events()); r.key('G')
                    t57_wait(r, lambda: any(e[2] == 'BAM_LAN' for e in r.of('state', mark)),
                             'Space/G không về bám làn')
                    r.t57_odo += r.values.get('turn_start_cm', 30)
                    def no_old_turn():
                        need(not any(e[2] in ('RE_TRAI', 'RE_PHAI') for e in r.of('state', mark)),
                             'Space/G rẽ tiếp phiên cũ', 't57_reset')
                    r.frames(4, no_old_turn); no_old_turn(); space(r); r.t57_imu = r.t57_encoder = False; r.finish(); continue
                # T73–2: lỗi cảm biến chỉ hủy khả năng rẽ, không dừng/chốt.
                stamp, limit, fresh_since = None, 200, None
                if case.startswith('t57_transient_'):
                    # Chặn Planner, hồi mẫu ngay sau ack lỗi, trước kỳ lệnh kế.
                    stalled = r.inject('controller_stall'); r.frames(2)
                    before = len(r.of('command'))
                    created = float(str(r.of('command')[-1][2]).split()[1])
                    r.t57_imu = r.t57_encoder = False
                    sensor = action if idle else ('encoder' if '_encoder_' in case else 'imu')
                    event = r.inject(sensor + '_read_fail')
                    fresh_since = r.inject('imu_sample 0 0' if sensor == 'imu' else
                                           'encoder_sample {}'.format(getattr(r, 't57_odo', 0)))[0]
                    need(len(r.of('command')) == before,
                         'Planner chưa dừng trong cửa sổ tiêm lỗi/hồi mẫu', 'setup')
                    r.t57_imu = r.t57_encoder = True
                    resumed = r.inject('controller_resume')
                    t57_transient_window(stalled[0], resumed[0], created)
                    command = t57_wait(r, lambda: next((e for e in r.of('command', mark)
                                       if float(str(e[2]).split()[1]) >= resumed[0]), None),
                                       'Planner chưa tạo lệnh mới sau hồi mẫu')
                    t57_transient_window(stalled[0], resumed[0], created,
                                         float(str(command[2]).split()[1]), r.events()[mark:])
                    stamp, limit = event[0], 500
                elif case.startswith('t57_csv_'):
                    failed = ('imu', 'encoder') if case == 't57_csv_both' else (case[8:],)
                    # Hai giá trị riêng, khác 0 và khác mẫu hồi để bắt giữ số cũ.
                    r.t57_yaw, r.t57_odo = 17, 53
                    r.wait(lambda: any(row['yaw_deg'] != '' and row['odo_cm'] != '' and
                                       float(row['yaw_deg']) == 17 and float(row['odo_cm']) == 53
                                       and (len(failed) == 1 or row['planner_state'] == 'RE_TRAI')
                                       for row in r.rows()), 10, 'Chưa có mẫu CSV tốt 17/53 trước lỗi',
                           tag='setup' if len(failed) == 2 else 'interface')
                    r.t57_yaw, r.t57_odo = 0, 54
                    r.inject('imu_sample 0 0'); zero = r.inject('encoder_sample 54')
                    if len(failed) == 2:
                        r.wait(lambda: any(float(row['command_ms']) >= zero[0]
                                           and row['planner_state'] == 'RE_TRAI'
                                           and row['yaw_deg'] != '' and row['odo_cm'] != ''
                                           and float(row['yaw_deg']) == 0 and float(row['odo_cm']) == 54
                                           for row in r.rows()), 3, 'Chưa tiêu thụ mẫu 0/54 trước hai lỗi', tag='setup')
                        # Đợi lệnh mới thay vì tốn tuổi lệnh cho số khung cố định.
                        latest = r.of('command')[-1]
                        r.wait(lambda: r.of('command')[-1] != latest, 3,
                               'Chưa có lệnh tươi trước hai lỗi', tag='setup')
                        need(not any(e[2] == 'HUY_RE_CAM_BIEN' or motor_zero(e)
                                     for e in r.events()[mark:]), 'Đã hủy/dừng trước hai lỗi', 'setup')
                        stalled = r.inject('controller_stall')
                        created = float(str(r.of('command')[-1][2]).split()[1])
                        before = len(r.of('command'))
                    r.t57_imu = 'imu' not in failed
                    r.t57_encoder = 'encoder' not in failed
                    for sensor in failed:
                        event = r.inject(sensor + '_read_fail')
                        if stamp is None:
                            stamp = event[0]
                    if len(failed) == 2:
                        need(len(r.of('command')) == before,
                             'Planner chưa dừng khi tiêm hai lỗi', 'setup')
                        resumed = r.inject('controller_resume')
                        age = r.values.get('cmd_max_age_ms', 200)
                        need(0 <= resumed[0] - stalled[0] < age and 0 <= resumed[0] - created < age,
                             'Thiết lập hai lỗi làm lệnh hết tuổi', 'setup')
                        command = r.wait(lambda: next((e for e in r.of('command', mark)
                                         if float(str(e[2]).split()[1]) >= resumed[0]), None),
                                         3, 'Thiếu lệnh sau hai lỗi', tag='setup')
                        need(float(str(command[2]).split()[1]) < created + age,
                             'Planner hồi sau khi lệnh cũ hết tuổi', 'setup')
                    # Hai lỗi tiêm bằng hai lệnh: vòng an toàn có thể đọc giữa hai lệnh nên mẫu lỗi Planner nhận
                    # chỉ chắc chắn có ô của cảm biến lỗi ĐẦU (imu) trống; ô kia trống ở các dòng sau.
                    first = failed[:1]
                    t57_wait(r, lambda: any(row['reason'] == 'HUY_RE_CAM_BIEN' and
                                            all(row[column] == '' for sensor, column in
                                                (('imu', 'yaw_deg'), ('encoder', 'odo_cm'))
                                                if sensor in first) for row in r.rows()),
                             'Thiếu dòng CSV mẫu lỗi với ô tương ứng trống')
                    if len(failed) == 2:
                        t57_wait(r, lambda: any(row['yaw_deg'] == '' and row['odo_cm'] == '' for row in r.rows()),
                                 'Thiếu dòng CSV có cả hai ô cảm biến trống')
                    rows = r.rows()
                    if len(failed) == 2:
                        for row in rows:
                            if row['reason'] == 'HUY_RE_CAM_BIEN' and row['odo_cm'] != '' and float(row['odo_cm']) == 54:
                                row['odo_cm'] = ''  # mẫu đọc trước lệnh lỗi encoder: chấp nhận số tốt 54
                    t57_fault_csv(rows, failed, 54)
                elif case == 't57_stale_imu':
                    r.t57_imu = False
                    stamp, limit = external_stamp_ms(), 400
                elif case == 't57_lost_image_turning':
                    event = r.inject('camera_pause_600ms'); reason = 'MAT_ANH'
                    end = stopped(r, event[0], 700, mark=mark)
                else:
                    if case == 't57_g_pending_after_fault':
                        r.key('G')
                    imu = case != 't57_read_encoder'
                    if imu:
                        r.t57_imu = False
                    else:
                        r.t57_encoder = False
                    event = r.inject('imu_read_fail' if imu else 'encoder_read_fail')
                    if case == 't57_g_pending_after_fault':
                        r.key('G')
                    stamp = event[0]
                if case != 't57_lost_image_turning':
                    lane_mark = mark
                    if not idle and not case.endswith('_armed'):
                        r.wait(lambda: any(e[2] == 'BAM_LAN' for e in r.of('state', mark)) and
                               any(e[2] == 'HUY_RE_CAM_BIEN' for e in r.of('reason', mark)),
                               (limit + r.slack) / 1000.0 + 1,
                               'Không hủy rẽ về BAM_LAN/HUY_RE_CAM_BIEN')
                        lane_mark = next(i for i, e in enumerate(r.events()) if i >= mark and
                                         e[1] == 'state' and e[2] == 'BAM_LAN')
                        need(0 <= r.events()[lane_mark][0] - stamp <= limit + r.slack,
                             'Hủy rẽ quá hạn {} ms'.format(limit + r.slack), 't57_cancel')
                    def lane_only():
                        t73_lane_check(r, mark, lane_mark)
                        need(all(e[2] > 0 for e in r.of('motor_speed', mark) if not e[3]),
                             'Lỗi cảm biến làm motor không tiến', 't57_lane')
                    r.frames(4, lane_only); lane_only()
                    if idle:
                        need(int(str(r.of('frame')[-1][2]).split()[0]) < 120,
                             'Biển pha sau tới trước khi kiểm xong lỗi', 'setup')
                    need(any(e[2] > 0 and not e[3] for e in r.of('motor_speed', mark)),
                         'Không có lệnh motor mới sau lỗi', 't57_lane')
                    r.key('G')
                    t73_recover(r, mark, lane_mark, fresh_since=fresh_since, early_votes=not idle)
                    space(r); r.t57_imu = r.t57_encoder = False; r.finish(); continue
                reasons = ('LOI_CAM_BIEN', 'CAM_BIEN_CU') if case.startswith('t57_transient_') else (reason,)
                r.wait(lambda: any(e[2] == 'DUNG' for e in r.of('state', mark)) and
                       any(e[2] in reasons for e in r.of('reason', mark)),
                       message='Thiếu DUNG/' + '/'.join(reasons))
                r.t57_imu = r.t57_encoder = True
                if case == 't57_lost_image_turning':
                    t57_wait(r, lambda: r.of('mask') and float(str(r.of('mask')[-1][2]).split()[1]) > event[0] + 500,
                             'Chưa có ảnh/mask hồi phục')
                r.frames(4, lambda: no_motion(r, end, 't57_latch'))
                if case != 't57_lost_image_turning':
                    r.key('G')
                    r.frames(4, lambda: no_motion(r, end, 't57_latch'))
                space(r)
                r.frames(2, lambda: no_motion(r, end, 't57_latch'))
                mark = len(r.events()); r.key('G')
                t57_wait(r, lambda: any(e[1] == 'motor_speed' and e[2] > 0 and not e[3]
                                        for e in r.events()[mark:]), 'Space rồi G không mở chốt')
                space(r); r.t57_imu = r.t57_encoder = False; r.finish()
    finally:
        if previous is None:
            del YOLO_BOXES[mode]
        else:
            YOLO_BOXES[mode] = previous


def t71_cap(events):
    # T71–5/T73–6: kiểm cả lệnh gửi lỗi, ngoài trần cứng chung 20/40.
    cap(events)
    need(all(abs(e[2]) <= 15 for e in events if e[1] == 'motor_speed'),
         'Motor vượt max_speed_cms=15', 't71_cap')


def default_value(text, key, value):
    pattern = r'\b' + re.escape(key) + r'\b\s*[:=|]?\s*([-+\d.eE]+)'
    need(any('(mặc định)' in line and re.search(pattern, line) and
             float(re.search(pattern, line).group(1)) == value for line in text.splitlines()),
         'Bảng mặc định thiếu/sai ' + key, 't71_default')


def repo_value(key, value):
    text = (HERE.parent.parent / 'control.yaml').read_text(encoding='utf-8')
    found = re.search(r'^' + re.escape(key) + r':\s*([-+\d.eE]+)', text, re.MULTILINE)
    need(found and float(found.group(1)) == value, 'control.yaml sai ' + key, 't71_default')


def t71_case(case, cfg, root):
    # T71–5: mặc định lấy từ giao diện công bố; không đọc ControlParams.
    if case == 't71_defaults':
        with running(cfg, root) as r:
            t57_feed(r); r.boot(); r.finish()
            for key, value in (('cruise_cms', 15), ('max_speed_cms', 15),
                               ('steer_max_deg', 40), ('obstacle_action', 2)):
                default_value(r.text(), key, value); repo_value(key, value)
        return
    values = dict(turn_enable=0, start_wait_green=0, cruise_cms=15, max_speed_cms=15,
                  steer_max_deg=40, slow_per_deg=0, kd=0)
    if case in ('t71_steer_40_1', 't71_cruise_16'):
        values.update({'steer_max_deg': 40.1} if case.endswith('40_1') else {'cruise_cms': 16})
        with running(cfg, root, values=values) as r:
            reject(r); t71_cap(r.events())
        return
    if case == 't71_curve_30':
        # T71–3: góc 30° phải gần 10 cm/s, không khóa số slow_per_deg.
        values.update(kp=200, steer_max_deg=30, center_offset_frac=0.3)
        del values['slow_per_deg']
        with running(cfg, root, values=values, schedule=((1, 'left'),)) as r:
            r.boot(); started = r.go()
            def curved():
                return [row for row in r.rows() if float(row['t_ms']) >= started[0] and
                        29.9 <= abs(float(row['requested_steer_deg'])) <= 30.01 and
                        29.9 <= abs(float(row['sent_steer_deg'])) <= 30.01 and
                        float(row['sent_speed_cms']) > 0]
            r.wait(lambda: len(curved()) >= 3, 10, 'Chưa có ba mẫu góc yêu cầu 30 độ')
            rows = curved()
            need(all(9 <= float(row[key]) <= 11 for row in rows
                     for key in ('requested_speed_cms', 'sent_speed_cms')),
                 'Góc 30 độ không cho tốc độ khoảng 10 cm/s', 't71_curve')
            need(all(any(e[1] == 'motor_speed' and not e[3] and
                         abs(e[2] - float(row['sent_speed_cms'])) < 0.01 and
                         abs(e[0] - float(row['t_ms'])) <= 50 for e in r.events()) for row in rows),
                 'Tốc độ cua CSV không có lệnh motor tương ứng', 't71_curve')
            r.finish(); t71_cap(r.events())
        return
    # T71–5: 40 nhận, 15/15 nhận; gây góc lớn hai chiều để kiểm trần lệnh.
    modes = ('straight', 'left', 'right') if case == 't71_motor_cap' else ('straight',)
    for mode in modes:
        with running(cfg, root / mode, values=dict(values, kp=200), schedule=((1, mode),)) as r:
            r.boot(); r.go(); r.frames(12, lambda: t71_cap(r.events()))
            if mode == 'straight':
                speed_is(r, 15)
            else:
                need(any(abs(e[2]) >= 20 for e in r.of('servo_angle') if not e[3]),
                     'Chưa tạo được góc lớn để thử trần', 'setup')
            r.finish(); t71_cap(r.events())


def media_files(run):
    return [p for p in run.session().rglob('*') if p.suffix.lower() in ('.jpg', '.jpeg', '.png')]


def t72_case(case, cfg, root):
    # T72–6: 0 hợp lệ; -1 bị từ chối trước khi chạm motor.
    values = dict(record_every_n=-1 if case == 't72_negative_rejected' else 0,
                  turn_enable=0, start_wait_green=0, slow_per_deg=0)
    with running(cfg, root, values=values, no_yolo=False, yolo_schedule=((1, 'empty'),)) as r:
        if case == 't72_negative_rejected':
            reject(r); return
        r.boot()
        if case == 't72_zero_valid':
            r.finish(); return
        # T72–5,6: đối chiếu tệp thực, log văn bản vẫn tăng khi không có ảnh.
        r.go(); r.frames(15)
        r.wait(lambda: len(r.rows()) >= 3, 10, 'Tắt ảnh làm mất CSV điều khiển')
        before_rows = len(r.rows())
        need(not media_files(r), 'record=0 vẫn sinh ảnh/mặt nạ', 't72_media')
        r.frames(6)
        r.wait(lambda: len(r.rows()) > before_rows, 10, 'record=0 ngừng log văn bản')
        space(r)
        if case == 't72_reload_0_to_5':
            # T72–6: R có sẵn chỉ áp trong DUNG; không thêm phím/cờ.
            patch_yaml(r, dict(values, record_every_n=5)); reload(r, 'accepted')
            r.go()
            r.wait(lambda: list(r.session().rglob('*.jpg')) and list(r.session().rglob('*.png')),
                   10, 'R từ 0 sang 5 không bắt đầu ghi ảnh')
            r.frames(10); space(r)
        r.finish()
        need(len(r.rows()) > before_rows, 'R/ghi ảnh làm mất CSV điều khiển', 't72_media')
        detection_log = r.session() / 'detections.csv'
        need(detection_log.is_file() and len(detection_log.read_text(encoding='utf-8').splitlines()) > 1,
             'Tắt ảnh làm mất detections.csv', 't72_media')
        if case == 't72_zero_keeps_text':
            need(not media_files(r), 'record=0 vẫn để lại ảnh/mặt nạ', 't72_media')
        else:
            inspect_images(r, every=5)


def t73_lane_check(run, mark, lane_mark=None):
    t71_cap(run.events())
    need(not any(e[2] == 'DUNG' for e in run.of('state', mark)) and
         not any(e[2] in ('RE_TRAI', 'RE_PHAI') for e in run.of('state', mark if lane_mark is None else lane_mark)),
         'Lỗi cảm biến vẫn rẽ lại hoặc làm DUNG', 't73_lane')
    need(not any(motor_zero(e) for e in run.events()[mark:]),
         'Lỗi cảm biến làm motor dừng', 't73_lane')


def t73_recover(run, mark, lane_mark=None, direction='left', fresh_since=None, early_votes=True):
    """Q9 TAM: đủ biển/quãng trước 1 s vẫn bám làn; sau đó phải nhận phiếu mới."""
    if hasattr(run, 't73_progress'):
        run.t73_progress = False
    if fresh_since is None:
        # Không để bộ bơm mẫu tự hồi trước mốc ACK dùng cho phép đo 1000 ms.
        run.t57_imu = run.t57_encoder = False
        imu = run.inject('imu_sample {} 0'.format(getattr(run, 't57_yaw', 0)))
        encoder = run.inject('encoder_sample {}'.format(getattr(run, 't57_odo', 0)))
        fresh_since = max(imu[0], encoder[0])
    run.t57_imu = run.t57_encoder = True
    step = run.values.get('turn_start_cm', 30)
    run.t57_odo = getattr(run, 't57_odo', 0) + step
    def lane_only():
        t73_add_lane(run, mark, lane_mark)
    def enough(since, first):
        lane_only()
        batches = [b for b in detection_batches(run.events()) if b[0] >= first and
                   b[2] > since and td.now_ms() - b[2] <= run.values.get('det_max_age_ms', 500)]
        return batches if len(batches) >= run.values.get('vote_min', 3) else None
    if early_votes:
        first = len(run.events())
        run.inject('yolo_resume')
        run.wait(lambda: enough(fresh_since, first), 3, 'Thiếu phiếu thử trước 1000 ms')
        run.frames(3, lane_only)
    run.inject('yolo_stall')
    age = run.values.get('det_max_age_ms', 500)
    last = max((b[2] for b in detection_batches(run.events())), default=fresh_since - age - 1)
    need(last + age < fresh_since + 1000,
         'Phiếu thử tới quá muộn để hết tuổi trước mốc hồi 1000 ms', 'setup')
    def cooled():
        lane_only()
        return td.now_ms() >= fresh_since + 1000 and td.now_ms() > last + age
    run.wait(cooled, 3, 'Chưa đủ 1000 ms mẫu tươi liên tục')
    # Giữ nguyên quãng đã đủ: phiếu trước khi hồi không được tự khởi động lại rẽ.
    run.frames(3, lane_only)
    first = len(run.events())
    resumed = run.inject('yolo_resume')
    if not early_votes:
        # Fixture idle chỉ có biển từ frame120; lô rỗng trước đó không phải phiếu biển.
        def idle_sign():
            lane_only()
            return last_detection_frame(run.events()) >= 126
        run.wait(idle_sign, 10, 'Chưa có biển mới frame126 sau lỗi khi idle')
    run.wait(lambda: enough(resumed[0], first), 10, 'Thiếu biển mới đủ phiếu sau hồi cảm biến')
    run.frames(3, lane_only)
    run.t57_odo += step
    turn = run.wait(lambda: next((e for e in run.of('state', first) if
                    e[2] == ('RE_TRAI' if direction == 'left' else 'RE_PHAI')), None),
                    3, 'Mẫu tươi >=1000 ms và biển/quãng mới không cho rẽ lại')
    need(turn[0] >= fresh_since + 1000, 'Rẽ lại trước đủ 1000 ms mẫu tươi', 't73_recover')
    need(not any(e[2] == 'DUNG' for e in run.of('state', mark)) and
         not any(motor_zero(e) or (e[1] == 'motor_speed' and e[2] <= 0 and not e[3])
                 for e in run.events()[mark:]),
         'Hồi cảm biến làm dừng/chốt hoặc motor không tiến', 't73_recover')
    t73_add_fresh(run, fresh_since, turn[0])
    t71_cap(run.events())
    return turn


def t73_case(case, cfg, root):
    values = dict(cruise_cms=15, max_speed_cms=15, steer_max_deg=40, slow_per_deg=0,
                  kp=0, kd=0, turn_enable=1, start_wait_green=0, yolo_every_n=1,
                  vote_min=1, mask_max_age_ms=1000, cmd_max_age_ms=500)
    if case == 't73_g_reopens_dung':
        # T73–3: bỏ chốt mọi DUNG, giữ dừng an toàn của lỗi điều khiển.
        with running(cfg, root, values=values) as r:
            t57_feed(r); r.boot(); r.go(); event = r.inject('controller_throw')
            stopped(r, event[0], 150)
            r.wait(lambda: any(e[0] >= event[0] and e[2] == 'LOI_DIEU_KHIEN' for e in r.of('reason')),
                   10, 'Thiếu lý do dừng điều khiển')
            r.frames(4); r.go(); r.frames(6); r.finish(); t71_cap(r.events())
        return
    init_fail = case in ('t73_init_imu_fail', 't73_init_drift_fail')
    lane_fault = case in ('t73_imu_lane', 't73_encoder_lane')
    both_directions = case in ('t73_imu_turning', 't73_encoder_turning')
    for direction in (('left', 'right') if both_directions else ('left',)):
        mode = 't73_' + direction
        YOLO_BOXES[mode] = [(direction, 800)]
        def setup(run):
            run.ctrl.write_text('1 imu_sample 0 0\n2 encoder_sample 0\n' +
                                ('3 yolo_stall\n' if lane_fault else ''), encoding='utf-8')
            run.seq = 3 if lane_fault else 2
        current = dict(values)
        if case == 't73_default_enable':
            del current['turn_enable']
        if case == 't73_reload_no_imu':
            current.update(turn_enable=0, cruise_cms=6)
        if case == 't73_stale_mask_turning':
            current.update(mask_max_age_ms=100, control_period_ms=20)
        env = {'OPEN_FAIL': 'imu' if case == 't73_init_imu_fail' else 'imu_drift'} if init_fail else {}
        try:
            with running(cfg, root / direction, values=current, env=env, setup=setup, no_yolo=False,
                         yolo_schedule=((1, mode),)) as r:
                if case == 't73_encoder_turning':
                    # Rẽ theo quãng: encoder giữ mốc0 tới khi bắt đầu rẽ rồi mới tiến đều.
                    t73_add_feed(r); r.t73_progress = False
                else:
                    t57_feed(r)
                r.boot()
                if lane_fault:
                    r.wait(lambda: any(e[2] == '3 yolo_stall' for e in r.of('inject')),
                           10, 'Thiếu ACK giữ biển trước G')
                    need(not r.of('detections'), 'Biển tới trước lỗi BAM_LAN', 'setup')
                if init_fail:
                    # T73–1,6: init lỗi vẫn bám làn khi có biển và đủ quãng.
                    need('IMU LOI: KHONG RE' in r.text(), 'Thiếu cảnh báo IMU LOI: KHONG RE')
                    need(len(r.of('motor_init')) == 1, 'Motor::Init không đúng một lần')
                if case == 't73_reload_no_imu':
                    # T73–4: khóa khác áp được dù chưa thể bật rẽ.
                    space(r); patch_yaml(r, dict(current, turn_enable=1, cruise_cms=9))
                    reload(r, 'accepted')
                    need(not r.of('imu_init'), 'R tự khởi tạo IMU')
                go_mark = len(r.events())
                started = r.go(); mark = r.events().index(started)
                if init_fail or case == 't73_reload_no_imu':
                    r.wait(lambda: r.of('detections', go_mark), 10, 'Thiếu biển thử bỏ rẽ')
                    r.frames(3)
                    r.t57_odo = r.values.get('turn_start_cm', 30)
                    r.frames(12, lambda: t73_lane_check(r, mark, go_mark))
                    need(any(e[2] == 'BAM_LAN' for e in r.of('state')), 'Không bám làn')
                    speed_is(r, 9 if case == 't73_reload_no_imu' else 15)
                    r.finish(); continue
                if lane_fault:
                    # Q9 TAM: BAM_LAN lỗi vẫn chạy, biển tới khi còn lỗi không được armed/rẽ.
                    need(any(e[2] == 'BAM_LAN' for e in r.of('state', go_mark)),
                         'Chưa bám làn trước lỗi cảm biến', 'setup')
                else:
                    t68a_turn(r, go_mark, started[0], direction)
                    if case == 't73_encoder_turning':
                        r.t73_base, r.t73_origin, r.t73_progress = r.t57_odo, td.now_ms(), True
                if case == 't73_default_enable':
                    # T73–5,6: chứng minh bật thật bằng hành vi rẽ khi YAML bỏ khóa.
                    default_value(r.text(), 'turn_enable', 1); repo_value('turn_enable', 1)
                    r.finish(); continue
                mark = len(r.events())
                if case == 't73_lost_image_turning':
                    # T73–6: đường MAT_ANH không được biến thành hủy rẽ rồi chạy.
                    event = r.inject('camera_pause_600ms'); end = stopped(r, event[0], 700, mark=mark)
                    r.wait(lambda: any(e[2] == 'MAT_ANH' for e in r.of('reason', mark)), 10, 'Thiếu MAT_ANH')
                    r.wait(lambda: any(float(str(e[2]).split()[1]) > event[0] + 500 for e in r.of('mask', mark)),
                           10, 'Chưa có mask hồi phục')
                    r.frames(4, lambda: no_motion(r, end)); r.finish(); continue
                if case == 't73_stale_mask_turning':
                    # T73–6: mask cũ vẫn motor0 trước servo0, chỉ hồi sau mask mới.
                    r.inject('road_stall'); captured = float(str(r.of('mask')[-1][2]).split()[1])
                    fresh_recovery(r, captured, 'mask', 320, 100, 'MAT_NAN_CU')
                    r.finish(); continue
                sensor = 'encoder' if 'encoder' in case else 'imu'
                setattr(r, 't57_' + sensor, False)
                if 'stale' not in case:
                    r.inject(sensor + '_read_fail')
                lane_mark = mark
                if lane_fault:
                    r.frames(4, lambda: t73_lane_check(r, mark))
                    r.inject('yolo_resume')
                    r.wait(lambda: r.of('detections', mark), 10, 'Chưa có biển mới sau lỗi BAM_LAN')
                    r.frames(3)
                    r.t57_odo = r.values.get('turn_start_cm', 30)
                    r.frames(6, lambda: t73_lane_check(r, mark))
                else:
                    # T73–2,6: lỗi/cũ hủy rẽ về làn, không chốt DUNG.
                    r.wait(lambda: any(e[2] == 'HUY_RE_CAM_BIEN' for e in r.of('reason', mark)) and
                           any(e[2] == 'BAM_LAN' for e in r.of('state', mark)),
                           3 + r.slack / 1000.0, 'Không hủy rẽ về BAM_LAN/HUY_RE_CAM_BIEN')
                    # Cho phép lệnh rẽ cũ trước vòng phát hiện lỗi, cấm rẽ lại từ lúc về làn.
                    lane_mark = next(i for i, e in enumerate(r.events()) if i >= mark and
                                     e[1] == 'state' and e[2] == 'BAM_LAN')
                t73_lane_check(r, mark, lane_mark)
                need(any(e[2] > 0 and not e[3] for e in r.of('motor_speed', mark)), 'Không tiếp tục motor')
                t73_recover(r, mark, lane_mark, direction)
                space(r)
                r.finish(); t71_cap(r.events())
        finally:
            del YOLO_BOXES[mode]


def t73_add_feed(run):
    """Mẫu luôn mới; encoder tiến 5 cm/s trừ khi ca chủ động gây kẹt/vượt ngưỡng."""
    original = run.wait
    run.t73_origin, run.t73_base, run.t73_progress = td.now_ms(), 0, True
    def wait(predicate, *args, **kwargs):
        def poll():
            if run.t73_progress:
                run.t57_odo = run.t73_base + (td.now_ms() - run.t73_origin) * 0.005
            return predicate()
        return original(poll, *args, **kwargs)
    run.wait = wait
    t57_feed(run)


def t73_add_lane(run, mark, lane_mark=None):
    t73_lane_check(run, mark, lane_mark)
    need(all(e[3] or e[2] > 0 for e in run.of('motor_speed', mark)),
         'Hủy rẽ gửi motor không tiến', 't73_add_lane')


def t73_add_cancel(run, mark, seconds=3):
    run.wait(lambda: any(e[2] == 'HUY_RE_CAM_BIEN' for e in run.of('reason', mark)) and
             any(e[2] == 'BAM_LAN' for e in run.of('state', mark)),
             seconds + run.slack / 1000.0, 'Không hủy rẽ về BAM_LAN/HUY_RE_CAM_BIEN')
    lane_mark = next(i for i, e in enumerate(run.events()) if i >= mark and
                     e[1] == 'state' and e[2] == 'BAM_LAN')
    run.wait(lambda: any(e[2] > 0 and not e[3] for e in run.of('motor_speed', lane_mark)),
             3, 'Không có lệnh motor tiến mới sau hủy rẽ')
    t73_add_lane(run, mark, lane_mark)
    return lane_mark


def t73_add_fresh(run, start, end):
    """Không cho stale cảm biến làm ca giới hạn/kẹt đạt nhầm."""
    for sensor in ('imu', 'encoder'):
        stamps = [e[0] for e in run.of('inject') if not e[3] and e[0] <= end and
                  len(str(e[2]).split()) >= 3 and str(e[2]).split()[1] == sensor + '_sample']
        before = [stamp for stamp in stamps if stamp <= start]
        need(before, 'Thiếu mẫu {} trước khi rẽ'.format(sensor), 'setup')
        window = [before[-1]] + [stamp for stamp in stamps if stamp > start] + [end]
        need(all(0 <= b - a <= 200 for a, b in zip(window, window[1:])),
             'Mẫu {} quá cũ trong ca giới hạn/kẹt'.format(sensor), 'setup')


def t73_add_case(case, cfg, root):
    values = dict(cruise_cms=15, max_speed_cms=15, steer_max_deg=40, slow_per_deg=0,
                  kp=0, kd=0, turn_enable=1, turn_yaw_deg=80, turn_steer_deg=25,
                  turn_arm_cm=80, turn_max_cm=70, turn_max_ms=8000,
                  start_wait_green=0, yolo_every_n=1, vote_window=5, vote_min=3,
                  mask_max_age_ms=1000, cmd_max_age_ms=500, control_period_ms=20)
    armed = case == 't73_stale_armed_lane'
    init = case == 't73_motor_init_once_imu_fail'
    modes = ('left', 'right')
    if armed:
        modes = ('control', 'imu', 'encoder')
    elif case == 't73_fault_stop_order':
        modes = ('space', 'term')
    elif init or case == 't73_fault_then_lost_image':
        modes = ('left',)
    for action in modes:
        direction = 'right' if action == 'right' else 'left'
        mode = 't73_add_' + direction
        YOLO_BOXES[mode] = [(direction, 800)]
        def setup(run):
            run.ctrl.write_text('1 imu_sample 0 0\n2 encoder_sample 0\n', encoding='utf-8')
            run.seq = 2
        current = dict(values)
        if case == 't73_turn_max_ms_exceeded':
            current['turn_max_ms'] = 1000
        try:
            with running(cfg, root / action, values=current, setup=setup, no_yolo=False,
                         env={'OPEN_FAIL': 'imu'} if init else {},
                         yolo_schedule=((1, mode),)) as r:
                t73_add_feed(r)
                r.t73_progress = False
                r.boot()
                if init:
                    # T73–1: init lỗi và R giữa phiên không được gọi Motor::Init lần hai.
                    need('IMU LOI: KHONG RE' in r.text(), 'Thiếu cảnh báo IMU lỗi')
                    need(len(r.of('motor_init')) == 1, 'Motor::Init sai số lần sau boot')
                    state_mark = len(r.events()); started = r.go(); mark = r.events().index(started)
                    t68a_armed(r, state_mark, started[0])
                    r.t57_odo = current.get('turn_start_cm', 30)
                    r.frames(8, lambda: t73_add_lane(r, mark, state_mark))
                    space(r); patch_yaml(r, dict(current, cruise_cms=9)); reload(r, 'accepted')
                    need(len(r.of('motor_init')) == 1, 'R gọi lại Motor::Init')
                    state_mark = len(r.events()); started = r.go(); mark = r.events().index(started)
                    r.frames(3, lambda: t73_add_lane(r, mark, state_mark))
                    r.t57_odo += current.get('turn_start_cm', 30)
                    r.frames(8, lambda: t73_add_lane(r, mark, state_mark)); speed_is(r, 9)
                    t73_add_lane(r, mark, state_mark)
                    r.t57_imu = r.t57_encoder = False
                    r.finish()
                    need(len(r.of('motor_init')) == 1, 'Motor::Init không đúng một lần cả phiên')
                    continue
                go_mark = len(r.events()); started = r.go()
                if armed:
                    # T73–2: cùng biển/ảnh, chỉ đối chứng mẫu tươi được rẽ khi đủ quãng.
                    t68a_armed(r, go_mark, started[0])
                    need(any(e[2] == 'BAM_LAN' for e in r.of('state', go_mark)),
                         'Chưa có đoạn armed/BAM_LAN trước turn_start_cm', 'setup')
                    mark = r.events().index(started)
                    if action != 'control':
                        # Encoder không có dấu thời gian (chỉ IMU có sensor_ms): "encoder cũ" không quan sát được
                        # khi chưa rẽ, nên lượt encoder dùng lỗi đọc; lượt IMU giữ ngừng cấp mẫu (cũ > 200 ms).
                        if action == 'encoder':
                            r.t57_encoder = False
                            r.inject('encoder_read_fail')
                        else:
                            setattr(r, 't57_' + action, False)
                        stamp = td.now_ms()
                        def stale_lane():
                            t73_add_lane(r, mark)
                            return td.now_ms() > stamp + 400 + r.slack
                        r.wait(stale_lane, 3, 'Cảm biến chưa quá cũ khi armed')
                    if action == 'control':
                        r.t57_odo = current.get('turn_start_cm', 30)
                        r.wait(lambda: any(e[2] == 'RE_TRAI' for e in r.of('state', go_mark)),
                               3, 'Đối chứng mẫu tươi không armed/rẽ', tag='setup')
                    else:
                        need(any(e[2] > 0 and not e[3] for e in r.of('motor_speed', mark)),
                             'Stale khi armed không tiếp tục motor')
                        t73_recover(r, mark)
                    r.t57_imu = r.t57_encoder = False
                    r.finish(); continue
                turn = t68a_turn(r, go_mark, started[0], direction)
                r.t73_base, r.t73_origin, r.t73_progress = r.t57_odo, td.now_ms(), True
                mark = len(r.events())
                if case == 't73_turn_max_cm_exceeded':
                    # T73–2: vượt quãng đường, không dùng lỗi đọc/cảm biến cũ hay timeout.
                    need(td.now_ms() - turn[0] < 1000, 'Quá chậm trước mẫu vượt quãng đường', 'setup')
                    r.t73_base = r.t57_odo + current['turn_max_cm'] + 10
                    r.t73_origin = td.now_ms()
                    r.inject('encoder_sample {}'.format(r.t73_base))
                    lane_mark = t73_add_cancel(r, mark)
                elif case == 't73_turn_max_ms_exceeded':
                    # T73–2: encoder tăng chậm, yaw chưa đạt đích; cấm hủy trước timer.
                    baseline = r.t57_odo
                    lane_mark = t73_add_cancel(r, mark)
                    cancelled = next(e for e in r.of('reason', mark) if e[2] == 'HUY_RE_CAM_BIEN')
                    need(cancelled[0] >= turn[0] + current['turn_max_ms'] - current['control_period_ms'],
                         'Hủy rẽ trước turn_max_ms (có thể bị kẹt/cảm biến cũ)', 't73_timer')
                    need(0 < r.t57_odo - baseline < current['turn_max_cm'],
                         'Encoder không tăng chậm dưới turn_max_cm', 'setup')
                elif case == 't73_stuck_turning':
                    # T73–2, leader chốt: số đo đứng nhưng timestamp luôn mới, hủy trước timer.
                    r.t73_progress = False
                    fixed = r.t57_odo
                    lane_mark = t73_add_cancel(r, mark, (current['turn_max_ms'] - 1000) / 1000.0)
                    cancelled = next(e for e in r.of('reason', mark) if e[2] == 'HUY_RE_CAM_BIEN')
                    need(r.t57_odo == fixed and cancelled[0] < turn[0] + current['turn_max_ms'],
                         'Ca kẹt bị che bởi quá quãng đường/thời gian', 't73_stuck')
                else:
                    # T73–6: chỉ thử đường dừng sau khi lỗi đã hủy rẽ và motor vẫn tiến.
                    r.t57_imu = False; r.inject('imu_read_fail')
                    lane_mark = t73_add_cancel(r, mark)
                    r.t57_imu = True
                if case in ('t73_turn_max_cm_exceeded', 't73_turn_max_ms_exceeded', 't73_stuck_turning'):
                    cancelled = next(e for e in r.of('reason', mark) if e[2] == 'HUY_RE_CAM_BIEN')
                    t73_add_fresh(r, turn[0], cancelled[0])
                if case == 't73_fault_stop_order':
                    r.frames(4, lambda: t73_add_lane(r, mark, lane_mark))
                    stop_mark = len(r.events())
                    if action == 'space':
                        stamp, limit = r.key(' ')[0], 150
                    else:
                        r.t57_imu = r.t57_encoder = False
                        stamp, limit = external_stamp_ms(), 1000
                        os.kill(r.process.pid, signal.SIGTERM)
                    stopped(r, stamp, limit, mark=stop_mark)
                    events = r.events()
                    # Cùng ms vẫn phân biệt lệnh trước dừng và tái chạy sau motor0 đầu tiên.
                    end = events.index(stopping_pair(events[stop_mark:], stamp)[0], stop_mark)
                    if action == 'space':
                        r.frames(4, lambda: held(r, end))
                        r.t57_imu = r.t57_encoder = False
                        r.finish()
                    else:
                        r.exited(); held(r, end)
                    continue
                if case == 't73_fault_then_lost_image':
                    r.frames(4, lambda: t73_add_lane(r, mark, lane_mark))
                    stop_mark = len(r.events()); event = r.inject('camera_pause_600ms')
                    end = stopped(r, event[0], 700, mark=stop_mark)
                    r.wait(lambda: any(e[2] == 'MAT_ANH' for e in r.of('reason', stop_mark)),
                           3, 'Thiếu MAT_ANH sau lỗi cảm biến')
                    r.wait(lambda: any(float(str(e[2]).split()[1]) > event[0] + 500
                                      for e in r.of('mask', stop_mark)), 10, 'Chưa có ảnh hồi phục')
                    r.frames(4, lambda: no_motion(r, end))
                    r.t57_imu = r.t57_encoder = False
                    r.finish(); continue
                t73_recover(r, mark, lane_mark, direction, fresh_since=cancelled[0])
                space(r)
                r.t57_imu = r.t57_encoder = False
                r.finish(); t71_cap(r.events())
        finally:
            del YOLO_BOXES[mode]


def t62_window(events, rows, since, until, speed, reason, setup=False):
    """T62 mục 4: kiểm cả lệnh motor cuối và CSV, không nhận đoạn rỗng là đạt."""
    motors = [e for e in events if since <= e[0] < until and e[1] == 'motor_speed' and not e[3]]
    samples = [row for row in rows if since <= float(row['t_ms']) < until]
    enough = len(motors) >= 2 and len(samples) >= 2
    if not setup:
        need(enough, 'Thiếu mẫu trong cửa sổ T62', 't62_window')
    need(all(abs(e[2] - speed) < 1e-6 for e in motors),
         'Motor không giữ {}: {}'.format(speed, motors), 't62_window')
    need(all(row['reason'] == reason and row['planner_state'] == 'BAM_LAN'
             and abs(float(row['sent_speed_cms']) - speed) < 1e-6 for row in samples),
         'CSV sai reason/trạng thái/tốc độ trong cửa sổ T62', 't62_window')
    if setup:
        need(enough, 'Thiếu mẫu trong cửa sổ T62', 'setup')


def t62_consumed(run, batches):
    """Lô đã công bố chưa chắc đã được vòng điều khiển dùng; CSV xác nhận đúng ID/mốc."""
    def samples():
        rows = run.rows()
        result = [next((row for row in rows if int(row['det_frame_id']) == batch[1]
                        and float(row['det_captured_ms']) == batch[2]), None) for batch in batches]
        return result if all(result) else None
    return run.wait(samples, 3, 'Vòng điều khiển bỏ lỡ lô trong kịch bản', tag='setup')


def t62_control_mark(run, batch, row):
    stamp = float(row['command_ms'])
    command = next((i for i, e in enumerate(run.events()) if i > batch[0] and e[1] == 'command'
                    and float(str(e[2]).split()[1]) == stamp), None)
    need(command is not None, 'Thiếu sự kiện điều khiển đã nhận lô đỏ', 'setup')
    return stamp, command + 1


def t62_next_red(run, delta, attempts=3):
    # Không có đồng hồ giả: dùng tuổi khung thật, hậu kiểm captured_ms thay vì ngủ cố định.
    for _ in range(attempts):
        previous = detection_batches(run.events())[-1]
        run.wait(lambda: float(str(run.of('frame')[-1][2]).split()[1]) >= previous[2] + delta,
                 3, 'Chưa tới khung cho lô đỏ kế tiếp')
        mark = len(run.events())
        run.inject('yolo_throw_off')
        run.wait(lambda: detection_batches(run.events()[mark:]), 3, 'Thiếu lô đỏ kế tiếp')
        run.inject('yolo_throw_on')
        batches = [b for b in detection_batches(run.events()) if b[0] >= mark]
        if len(batches) == 1 and delta <= batches[0][2] - previous[2] <= delta + 120:
            if attempts > 1:
                t62_consumed(run, batches)
            return batches[0]
    need(False, 'Lịch lô không đạt Δ={} ms sau {} lần: {}'.format(delta, attempts, batches), 'setup')


def t62_case(case, cfg, root):
    # T62v2 mục 4 + T62 gốc mục 4: không đổi ngưỡng, bỏ khóa để kiểm mặc định x2=0,90.
    values = dict(YOLO_VALUES, turn_enable=0, start_wait_green=0,
                  cruise_cms=15, max_speed_cms=15, kp=0)
    del values['red_stop_x_frac']
    if case == 't62_no_light':
        # Không đèn: cả YOLO rỗng và --no-yolo giữ cùng lệnh làn thẳng.
        outputs = []
        for disabled in (False, True):
            with running(cfg, root / str(disabled), values=values, no_yolo=disabled,
                         yolo_schedule=None if disabled else ((1, 'empty'),)) as r:
                r.boot(); gm = len(r.events()); r.go()
                # Trước lô YOLO tươi đầu tiên P2 kẹp approach_cms (MAT_YOLO, hành vi có sẵn): chỉ so sau đó.
                r.wait(lambda: any(e[2] == 15 and not e[3] for e in r.of('motor_speed', gm)), 3, 'Chưa chạy 15 sau G')
                speed_is(r, 15)
                mark = len(r.events()); r.frames(8)
                commands = [e for e in r.events()[mark:] if e[1] in ('motor_speed', 'servo_angle') and not e[3]]
                need(commands and all(e[2] == (15 if e[1] == 'motor_speed' else 0) for e in commands),
                     'Không đèn đổi lệnh làn thẳng', 't62_no_light')
                need(not any(e[2] in ('TIEP_CAN_DEN_DO', 'DUNG_DEN_DO', 'DEN_XANH')
                             for e in r.of('reason', mark)), 'Không đèn có reason đèn', 't62_no_light')
                outputs.append(set((e[1], e[2]) for e in commands)); r.finish()
        need(outputs[0] == outputs[1], 'YOLO rỗng khác --no-yolo', 't62_no_light')
        return
    if case.startswith('t62_stop_') and case != 't62_stop_flicker':
        # Biên mép phải x2/W: tâm cả ba hộp đều dưới 0,90; chỉ x2 quyết định dừng.
        mode = {'089': 'red_089', '090': 'red_edge', '091': 'red_091'}[case[-3:]]
        with running(cfg, root, values=values, no_yolo=False,
                     yolo_schedule=((1, 'empty'), (120, mode))) as r:
            r.boot(); r.go(); mark = len(r.events())
            need(last_detection_frame(r.events()) < 120, 'Bỏ lỡ trước hộp biên', 'setup')
            if case.endswith('089'):
                r.wait(lambda: any(e[2] == 'TIEP_CAN_DEN_DO' for e in r.of('reason', mark)),
                       25, '0,89 không vào tiếp cận')
                since = external_stamp_ms(); r.frames(10)
                t62_window(r.events(), r.rows(), since, external_stamp_ms(), 5, 'TIEP_CAN_DEN_DO')
                need(not any(e[2] == 'DUNG_DEN_DO' for e in r.of('reason', mark)), '0,89 vẫn chốt đỏ')
            else:
                end = red_stop(r, 120, mark, consumed=case.endswith('090'))
                r.frames(8, lambda: held(r, end))
            r.finish()
        return
    if case == 't62_stop_flicker':
        # Leader: chốt GIỮ NGUYÊN, chỉ khi lô mới đủ phiếu; approach không thay phiếu dừng.
        values['yolo_every_n'] = 1
        for attempt in range(3):
            try:
                with running(cfg, root / str(attempt), values=values, no_yolo=False,
                             yolo_schedule=((1, 'empty'), (120, 'red_mid'), (123, 'empty'), (126, 'red_edge'))) as r:
                    r.boot(); r.go(); mark = len(r.events())
                    need(last_detection_frame(r.events()) < 120, 'Bỏ lỡ trước phiếu nhấp nháy', 'setup')
                    r.wait(lambda: last_detection_frame(r.events()) >= 128, 25,
                           'Chưa đủ phiếu tại lô mới', tag='setup')
                    batches = phase_batches(r, 120, 129, 9)
                    need([b[1] for b in batches] == list(range(120, 129))
                         and batches[-1][2] - batches[2][2] < 500,
                         'Lịch phiếu nhấp nháy thiếu khung/đỏ đã hết tươi', 'setup')
                    samples = t62_consumed(r, batches)
                    stamp, stop_mark = t62_control_mark(r, batches[-1], samples[-1])
                    need(stamp - batches[2][2] < 500,
                         'Đỏ hết tươi trước vòng nhận phiếu cuối', 'setup')
                    need(not any(motor_zero(e) for e in r.events()[batches[0][0]:stop_mark - 1]),
                         'Chốt trước lô 128 đủ ba phiếu', 't62_stop_votes')
                    t62_window(r.events(), r.rows(), float(samples[5]['command_ms']) + 20,
                               stamp, 5, 'TIEP_CAN_DEN_DO', setup=True)
                    stopped(r, stamp, 220, mark=stop_mark)
                    r.wait(lambda: any(e[2] == 'DUNG_DEN_DO' for e in r.of('reason', mark)),
                           3, 'Không chốt tại lô đủ phiếu')
                    end = stop_tail(r, stamp, stop_mark)
                    r.frames(8, lambda: held(r, end)); r.finish()
                break
            except Failure as error:
                if error.tag != 'setup' or attempt == 2:
                    raise
        return
    if case in ('t62_green_approach', 't62_green_latch'):
        # T58 qua hợp đồng A40b/A43/A44: chốt đỏ chỉ mở bằng ba xanh mới liên tiếp.
        latched = case.endswith('latch')
        schedule = ((1, 'empty'), (120, 'red_edge' if latched else 'red_mid'))
        schedule += (((180, 'empty'), (240, 'off'), (300, 'green'), (306, 'empty'), (360, 'green'))
                     if latched else ((300, 'green'),))
        with running(cfg, root, values=values, no_yolo=False, yolo_schedule=schedule) as r:
            r.boot(); r.go(); mark = len(r.events())
            need(last_detection_frame(r.events()) < 120, 'Bỏ lỡ tiền đề đỏ/xanh', 'setup')
            if latched:
                end = red_stop(r, 120, mark)
                phase_reached(r, 330, end)
                need(last_detection_frame(r.events()) < 360, 'Bỏ lỡ xanh bị ngắt', 'setup')
                phase_batches(r, 300, 306, 2)
                green_release(r, 360, end)
            else:
                r.wait(lambda: any(e[2] == 'TIEP_CAN_DEN_DO' for e in r.of('reason', mark)),
                       25, 'Chưa tiếp cận trước xanh')
                since = external_stamp_ms()
                r.wait(lambda: last_detection_frame(r.events()) >= 306, 25, 'Thiếu ba xanh')
                greens = phase_batches(r, 300, 309, 3)
                t62_window(r.events(), r.rows(), since + 20, greens[2][3], 5, 'TIEP_CAN_DEN_DO')
                r.wait(lambda: any(e[2] == 15 and not e[3] for e in r.of('motor_speed', greens[2][0] + 1)),
                       3, 'Đủ xanh không thả approach')
            speed_is(r, 15); r.finish()
        return
    # Giữ tiếp cận qua phiếu hết tuổi; tiêm lỗi YOLO giữ camera/mask tươi như A30c.
    with running(cfg, root, values=values, no_yolo=False,
                 yolo_schedule=((1, 'empty'), (120, 'red_mid'), (600, 'empty'))) as r:
        r.boot(); r.go()
        need(last_detection_frame(r.events()) < 120, 'Bỏ lỡ trước tiếp cận', 'setup')
        r.wait(lambda: any(e[2] == 'TIEP_CAN_DEN_DO' for e in r.of('reason')),
               25, 'Chưa xác nhận tiếp cận')
        r.inject('yolo_throw_on'); since = external_stamp_ms() + 20
        delta = 170 if case.endswith('170') else 200
        for _ in range(3):
            t62_next_red(r, delta, attempts=3)
        last = detection_batches(r.events())[-1]
        if case == 't62_reset_space':
            t62_consumed(r, [last])
        # Lô tới trễ thêm150ms tạo cửa sổ thiếu phiếu dài hơn một vòng điều khiển.
        r.wait(lambda: float(str(r.of('frame')[-1][2]).split()[1]) >= last[2] + delta + 150,
               3, 'Chưa tới pha lô trễ')
        until = external_stamp_ms()
        recent = detection_batches(r.events())[-5:]
        need(until - last[2] < 500 and sum(0 <= until - b[2] <= 500 for b in recent) < 3,
             'Chưa tạo được thiếu phiếu khi đỏ còn tươi', 'setup')
        t62_window(r.events(), r.rows(), since, until, 5, 'TIEP_CAN_DEN_DO')
        if case.startswith('t62_reset_'):
            # Space/G: không có lô mới và chưa hết tuổi đỏ, tránh việc tự xác nhận lại.
            if case.endswith('space'):
                space(r)
            if case.endswith('image'):
                mark = len(r.events()); event = r.inject('camera_pause_600ms')
                stopped(r, event[0], 700, mark=mark)
                r.wait(lambda: any(e[2] == 'MAT_ANH' for e in r.of('reason', mark)), 3, 'Thiếu MAT_ANH')
                end = stop_tail(r, event[0], mark)
                r.wait(lambda: any(float(str(e[2]).split()[1]) > event[0] + 500 for e in r.of('mask', mark)),
                       3, 'Ảnh chưa hồi phục')
                r.frames(5, lambda: held(r, end))
                r.wait(lambda: int(str(r.of('frame')[-1][2]).split()[0]) >= 600, 25, 'Chưa tới lô rỗng')
                r.inject('yolo_throw_off')
                r.wait(lambda: last_detection_frame(r.events()) >= 600, 3, 'Thiếu lô rỗng tươi')
            mark = len(r.events())
            if case.endswith('go'):
                # G khi xe đang chạy không có tác dụng trong ucr_auto (G chỉ chạy từ dừng): kẹp tiếp cận giữ nguyên.
                r.go(); r.frames(2)
                sent = [e[2] for e in r.of('motor_speed', mark) if not e[3]]
                need(sent and all(v == 5 for v in sent), 'G khi đang chạy đổi kẹp tiếp cận: ' + repr(sent), 't62_window')
                r.finish(); return
            r.go()
            r.wait(lambda: len([e for e in r.of('motor_speed', mark) if e[2] == 15 and not e[3]]) >= 2,
                   3, 'G mới vẫn mang kẹp approach')
            if not case.endswith('image'):
                first = next(e for e in r.of('motor_speed', mark) if e[2] == 15 and not e[3])
                need(first[0] - last[2] < 500 and detection_batches(r.events())[-1] == last,
                     'Reset chỉ được quan sát sau khi đỏ hết tuổi/lô mới', 'setup')
            r.finish(); return
        # Lô rỗng tươi tách hết tuổi đỏ khỏi MAT_YOLO cũng kẹp5.
        r.inject('yolo_throw_off')
        r.wait(lambda: last_detection_frame(r.events()) >= 600, 25, 'Chưa tới pha rỗng')
        reds = [b for b in detection_batches(r.events()) if 120 <= b[1] < 600]
        last = reds[-1]
        r.wait(lambda: any(float(row['t_ms']) > last[2] + 500 and row['reason'] == 'BAM_LAN'
                           and float(row['sent_speed_cms']) == 15 for row in r.rows()),
               5, 'Đỏ cuối quá500ms không thả về15')
        t62_window(r.events(), r.rows(), since, last[2] + 500, 5, 'TIEP_CAN_DEN_DO')
        need(any(last[2] + 400 <= e[0] < last[2] + 500 and e[1] == 'motor_speed' and e[2] == 5
                 for e in r.events()), 'Thiếu mẫu sát hạn đỏ cuối', 't62_window')
        speed_is(r, 15); r.finish()


def a28_origin(rows, events):
    # Đồng hồ/góc phải cùng lệnh đã dùng mask hợp lệ, không lấy mask chỉ mới công bố.
    valid = [i for i, row in enumerate(rows) if 0 < int(row['mask_id']) < 120]
    need(len(valid) >= 2, 'Thiếu lệnh làn hợp lệ trước mất làn', 'setup')
    last = rows[valid[-1]]
    angle = float(last['sent_steer_deg'])
    need(last['planner_state'] == 'BAM_LAN' and float(last['sent_speed_cms']) == 8 and angle != 0
         and angle == float(rows[valid[-2]]['sent_steer_deg']),
         'Góc chưa ổn định trước mất làn', 'setup')
    need(0 <= float(last['image_age_ms']) <= 500
         and 0 <= float(last['mask_age_ms']) <= BOUNDS['mask_max_age_ms'][0]
         and 0 <= float(last['t_ms']) - float(last['command_ms']) <= BOUNDS['cmd_max_age_ms'][0],
         'Dữ liệu làn trước mất làn đã cũ', 'setup')
    # Rơi trước hai lệnh hợp lệ cuối không làm mất mốc của cửa sổ đang kiểm.
    window = rows[valid[-2]:]
    need(all(int(row['csv_dropped']) == int(window[0]['csv_dropped']) for row in window),
         'CSV rơi mất mốc tiêu thụ làn', 'setup')
    mask = next((e for e in events if e[1] == 'mask' and int(str(e[2]).split()[0]) == int(last['mask_id'])), None)
    need(mask is not None, 'Thiếu mask của lệnh làn hợp lệ cuối', 'setup')
    captured = float(str(mask[2]).split()[1])
    need(math.isfinite(captured) and captured >= 0 and math.isfinite(angle), 'Mốc làn không hữu hạn', 'setup')
    return captured, angle


def test_case(case, cfg, root):
    if case in T73_ADDED_CASES:
        return t73_add_case(case, cfg, root)
    if case.startswith('t62_'):
        return t62_case(case, cfg, root)
    if case.startswith('t71_'):
        return t71_case(case, cfg, root)
    if case.startswith('t72_'):
        return t72_case(case, cfg, root)
    if case.startswith('t73_'):
        return t73_case(case, cfg, root)
    if case.startswith('t57_'):
        return t57_case(case, cfg, root)
    # A70 — T47l mục 1–2: thiếu riêng từng file, trước mọi tác động motor/servo.
    if case in ('A70u', 'A70y'):
        name = 'unet' if case.endswith('u') else 'yolo'
        start = running if name == 'unet' else yolo_running
        with start(cfg, root / 'valid') as r:
            if name == 'unet':
                need(not any(r.models.glob('yolo*')), 'Tiền đề phải thiếu toàn bộ YOLO', 'setup')
            r.boot(); r.frames(3, lambda: no_motion(r, 0)); r.finish()
        for suffix in ('.onnx', '_info.json', '.engine', '.engine.src'):
            filename = name + suffix
            with start(cfg, root / ('missing_' + filename),
                       setup=lambda r, filename=filename: (r.models / filename).unlink()) as r:
                reject_model(r)
                if suffix == '.engine.src':
                    need('build_engine.sh ' + name in r.text(), 'Thiếu lệnh build lại ' + name, 'model_rebuild')
        return
    # A71 — T47l mục 2: chỉ lật một byte ONNX; thông báo chỉ đúng lệnh build lại.
    if case in ('A71u', 'A71y'):
        name = 'unet' if case.endswith('u') else 'yolo'
        start = running if name == 'unet' else yolo_running
        with start(cfg, root / 'valid') as r:
            r.boot(); r.finish()
        with start(cfg, root / 'stale', setup=lambda r: stale_onnx(r, name)) as r:
            reject_model(r)
            need('build_engine.sh ' + name in r.text(), 'Thiếu lệnh build lại ' + name, 'model_rebuild')
            if name == 'unet':
                need('build_engine.sh yolo' not in r.text(), 'no-yolo vẫn yêu cầu build yolo', 'model_rebuild')
        return
    # A72 — T47l mục 3: file fixture vẫn hợp lệ, chỉ kích thước khác info.
    if case in ('A72u', 'A72y'):
        name = 'unet' if case.endswith('u') else 'yolo'
        start = running if name == 'unet' else yolo_running
        with start(cfg, root / 'valid') as r:
            r.boot(); r.finish()
        def wrong_size(r):
            if name == 'unet':
                (r.root / 'straight.png').write_bytes(png_bytes([row[:-1] for row in mask_rows('straight')]))
            else:
                path = r.root / 'empty.npy'
                data = path.read_bytes()
                need(b'(1, 13, 100)' in data, 'Thiếu shape NPY gốc', 'setup')
                path.write_bytes(data.replace(b'(1, 13, 100)', b'(1, 13, 99) ', 1)[:-13 * 4])
        with start(cfg, root / 'mismatch', setup=wrong_size) as r:
            reject_model(r)
        return
    # A74 — leader lượt 2: chỉ nhận 64 hex thường, không LF hoặc đúng một LF.
    if case in ('A74ua', 'A74ub', 'A74ya', 'A74yb'):
        name = 'unet' if case[3] == 'u' else 'yolo'
        start = running if name == 'unet' else yolo_running
        for ending, label in ((b'', 'no_lf'), (b'\n', 'one_lf')):
            def valid_src(r):
                digest = hashlib.sha256((r.models / (name + '.onnx')).read_bytes()).hexdigest().encode('ascii')
                (r.models / (name + '.engine.src')).write_bytes(digest + ending)
            with start(cfg, root / label, setup=valid_src) as r:
                r.boot(); r.finish()
        labels = ('upper', 'short', 'long') if case.endswith('a') else ('nonhex', 'empty', 'two_lines', 'crlf')
        for label in labels:
            def bad_src(r):
                digest = hashlib.sha256((r.models / (name + '.onnx')).read_bytes()).hexdigest().encode('ascii')
                need(digest.upper() != digest, 'Băm phải có chữ để kiểm hex HOA', 'setup')
                data = dict(upper=digest.upper(), short=digest[:-1], long=digest + b'0',
                            nonhex=b'g' + digest[1:], empty=b'', two_lines=digest + b'\n' + digest + b'\n',
                            crlf=digest + b'\r\n')[label]
                (r.models / (name + '.engine.src')).write_bytes(data)
            with start(cfg, root / label, setup=bad_src) as r:
                reject_model(r)
        return
    # A75 — không tính được SHA: ONNX là thư mục, không phụ thuộc quyền chmod.
    if case == 'A75':
        with running(cfg, root / 'valid') as r:
            r.boot(); r.finish()
        def directory_onnx(r):
            path = r.models / 'unet.onnx'
            path.unlink(); path.mkdir()
            need(path.is_dir(), 'ONNX phải là thư mục', 'setup')
        with running(cfg, root / 'directory', setup=directory_onnx) as r:
            reject_model(r)
        return
    # A77 — no-yolo phải bỏ qua cả bộ YOLO có thật nhưng hỏng.
    if case == 'A77':
        for label in ('valid', 'stale_onnx', 'bad_src', 'bad_info'):
            def corrupt_yolo(r):
                if label == 'stale_onnx':
                    stale_onnx(r, 'yolo')
                elif label == 'bad_src':
                    (r.models / 'yolo.engine.src').write_bytes(b'rac\n')
                elif label == 'bad_info':
                    (r.models / 'yolo_info.json').write_bytes(b'not JSON')
            with running(cfg, root / label, yolo_schedule=((1, 'empty'),), setup=corrupt_yolo) as r:
                r.boot(); r.finish()
                data = json.loads((r.session() / 'session.json').read_text(encoding='utf-8'))
                for field in ('variant', 'seed', 'data_tag', 'onnx_sha256'):
                    need('yolo_' + field in data, 'session thiếu yolo_' + field, 'model_session')
                need(all(data['yolo_' + field] == '' for field in ('variant', 'seed', 'data_tag', 'onnx_sha256')),
                     'no-yolo phải để rỗng 4 khóa yolo_* mới', 'model_session')
                need('yolo_engine_sha256' in data, 'session thiếu yolo_engine_sha256', 'model_session')
        return
    # A78 — lỗi model ưu tiên hơn tranh khóa; tiến trình test giữ khóa như A22.
    if case == 'A78':
        import fcntl
        with running(cfg, root / 'valid') as r:
            r.boot(); r.finish()
        with (root / 'vehicle.lock').open('w') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with running(cfg, root / 'contender', lock=root / 'vehicle.lock',
                         setup=lambda r: stale_onnx(r, 'unet')) as r:
                reject_model(r)
                need('build_engine.sh unet' in r.text(), 'Lỗi khóa che lỗi model', 'model_rebuild')
                need(r.process.returncode != 6, 'Trả lỗi khóa thay vì lỗi model', 'model_preflight')
        return
    # A79 — từng cờ cũ vẫn phải bị từ chối khi có model-dir hợp lệ.
    if case == 'A79':
        with yolo_running(cfg, root / 'valid') as r:
            r.boot(); r.finish()
        for flag, filename in (('--engine', 'unet.engine'), ('--info', 'unet_info.json'),
                               ('--yolo-engine', 'yolo.engine'), ('--yolo-info', 'yolo_info.json')):
            folder = root / flag[2:]
            with yolo_running(cfg, folder, args=[flag, str(folder / 'models' / filename)]) as r:
                reject_model(r)
        return
    # A80 — HOME tạm đủ bộ, không truyền model-dir; đường dẫn session nếu có phải khớp.
    if case == 'A80':
        with yolo_running(cfg, root / 'default_home', use_home_models=True) as r:
            r.boot(); r.finish()
            data = json.loads((r.session() / 'session.json').read_text(encoding='utf-8'))
            for key, value in data.items():
                if isinstance(value, str) and value and (key == 'model_dir' or
                        (('/' in value or '\\' in value) and
                         any(part in key for part in ('model', 'engine', 'onnx', 'info')))):
                    path = Path(value)
                    need(path.is_absolute() and (path == r.models or path.parent == r.models),
                         'Đường dẫn session không thuộc HOME tạm: ' + key, 'model_session')
        return
    # A73 — T47l mục 4: truy nguồn theo từng info/ONNX, giữ khóa băm engine cũ.
    if case == 'A73':
        for no_yolo in (False, True):
            def provenance(r):
                for name, variant, seed in (('unet', 'S2', 17), ('yolo', 'D2', 29)):
                    path = r.models / (name + '_info.json')
                    if path.exists():
                        data = json.loads(path.read_text(encoding='utf-8'))
                        data.update(variant=variant, seed=seed if not no_yolo else 0,
                                    data_tag='t47l_{}_{}'.format(name, no_yolo))
                        path.write_text(json.dumps(data), encoding='utf-8')
            with running(cfg, root / str(no_yolo), no_yolo=no_yolo, setup=provenance,
                         yolo_schedule=None if no_yolo else ((1, 'empty'),)) as r:
                r.boot(); r.finish(); inspect_model_session(r)
        return
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
        for before, after in ((6, 9), (10, 10)):
            old = dict(max_speed_cms=10, cruise_cms=before, slow_per_deg=0)
            new = dict(max_speed_cms=10, cruise_cms=after, slow_per_deg=0)
            with running(cfg, root / str(before), values=old) as r:
                r.boot(); r.go(); speed_is(r, before)
                if case == 'A03':
                    patch_yaml(r, new); reload(r, 'ignored')
                    space(r); r.go(); speed_is(r, before)
                else:
                    space(r)
                    patch_yaml(r, new); stamp = reload(r, 'accepted')
                    r.frames(5, lambda: no_motion(r, stamp))
                    r.go(); speed_is(r, after); space(r)
                    if case == 'A05':
                        for bad in ('cruise_cms: [\n', {'unknown': 1}, {'kp': 201},
                                    {'min_cms': 9, 'cruise_cms': 8}, {'lost_hold_ms': 900, 'lost_stop_ms': 800}):
                            patch_yaml(r, bad); stamp = reload(r, 'error')
                            r.frames(4, lambda: no_motion(r, stamp))
                            r.go(); speed_is(r, after); space(r)
                r.finish()
        return
    # A06 — hợp đồng P2 thay L1/P1: đủ YOLO chạy; thiếu/hỏng từ chối trước motor_init.
    if case == 'A06':
        with yolo_running(cfg, root / 'valid') as r:
            r.boot(); mark = len(r.events()); r.key('G')
            r.wait(lambda: any(e[2] == 'CHO_DEN_XUAT_PHAT' for e in r.of('reason', mark)),
                   message='YOLO hợp lệ không vào chờ xanh')
            r.frames(6, lambda: held(r, mark)); r.finish()
        for missing in (('yolo.engine', 'yolo_info.json'), ('yolo.engine',), ('yolo_info.json',)):
            def remove_files(r, missing=missing):
                for name in missing:
                    (r.models / name).unlink()
            with yolo_running(cfg, root / ('missing_' + '_'.join(missing)), setup=remove_files) as r:
                reject(r)
        bad = [lambda r: (r.models / 'yolo.engine').unlink(),
               lambda r: (r.models / 'yolo.engine').write_bytes(b''),
               lambda r: (r.models / 'yolo_info.json').unlink(),
               lambda r: (r.models / 'yolo_info.json').write_text('{', encoding='utf-8'),
               lambda r: (r.models / 'yolo_info.json').write_text('{}', encoding='utf-8')]
        for i, setup in enumerate(bad):
            with yolo_running(cfg, root / ('bad_' + str(i)), setup=setup) as r:
                reject(r)
        for index in range(9):
            def wrong_class(r, index=index):
                path = r.models / 'yolo_info.json'
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
        # T60: cô lập miền cruise khỏi quan hệ cruise <= max_speed.
        for values in bad:
            if 'cruise_cms' in values:
                values['max_speed_cms'] = 20
        bad += [dict(max_speed_cms=20.1), dict(max_speed_cms=20, cruise_cms=20.1)]
        bad += [dict(max_speed_cms=20, cruise_cms=20, **{key: 10.1})
                for key in ('approach_cms', 'min_cms', 'lost_cms')]
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
                    values.update(max_speed_cms=20, cruise_cms=bound, min_cms=0, lost_cms=0)
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
        variants += [dict(max_speed_cms=20, cruise_cms=20, steer_max_deg=40),
                     dict(scan_rows=[0.5], row_weights=[1], lane_width_frac=[1]),
                     dict(scan_rows=[0.1, 0.3, 0.5, 0.7, 0.9], row_weights=[0, 0, 0, 0, 1], lane_width_frac=[1] * 5)]
        for i, values in enumerate(variants):
            with running(cfg, root / str(i), values=values) as r:
                r.boot(); r.frames(3); r.finish(); inspect_session(r)
                if not values:
                    for key, (default, _, _, unit) in BOUNDS.items():
                        default = dict(cruise_cms=15, max_speed_cms=15, steer_max_deg=40).get(key, default)
                        lines = [l for l in r.text().splitlines() if re.search(r'\b' + key + r'\b', l)]
                        need(any('(mặc định)' in l and (key == 'slow_per_deg' or
                                 re.search(r'\b' + key + r'\b\s*[:=|]?\s*' + re.escape(str(default)) + r'(?![\d.])', l))
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
                  lambda r: (r.models / 'unet_info.json').unlink(),
                  lambda r: (r.models / 'unet_info.json').write_text('{}', encoding='utf-8'),
                  lambda r: (r.models / 'unet_info.json').write_text('{', encoding='utf-8'),
                  lambda r: (r.root / 'straight.png').write_bytes(png_bytes([bytearray([3] * 160)] * 96))]
        for i, setup in enumerate(setups):
            with running(cfg, root / str(i), setup=setup) as r:
                reject(r)
        with running(cfg, root / 'no_info_file', setup=lambda r: (r.models / 'unet_info.json').unlink()) as r:
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
    # A16/A35 — L6/bản duyệt: đúng ±20/±40 cả khi yaml thấp hơn; CSV giữ yêu cầu trước kẹp.
    if case in ('A16', 'A35'):
        for name, values in (('default', {}),
                             ('low', dict(max_speed_cms=5, cruise_cms=5, min_cms=0, steer_max_deg=10)),
                             ('cap20', dict(max_speed_cms=20, cruise_cms=20))):
            with running(cfg, root / name, values=values) as r:
                r.boot(); r.go()
                for sign in (1, -1):
                    def sent():
                        es = [e for e in r.events() if e[0] >= ev[0] and not e[3]]
                        return (any(e[1] == 'motor_speed' and e[2] == sign * 20 for e in es) and
                                any(e[1] == 'servo_angle' and e[2] == sign * 40 for e in es))
                    # Lệnh tiêm một nhịp có thể bị nhịp thường ghi đè trước vòng an toàn.
                    for attempt in range(5):
                        ev = r.inject('controller_command {} {}'.format(sign * 50, sign * 90))
                        try:
                            r.wait(sent, 2, message='Không gửi đúng biên kẹp từ lệnh tiêm', tag='hard_cap')
                        except Failure as error:
                            if error.tag != 'hard_cap' or r.process.poll() is not None or attempt == 4:
                                raise
                        else:
                            break
                    r.wait(lambda: any(float(row['t_ms']) >= ev[0] and
                                       float(row['requested_speed_cms']) == sign * 50 and
                                       float(row['requested_steer_deg']) == sign * 90 and
                                       float(row['sent_speed_cms']) == sign * 20 and
                                       float(row['sent_steer_deg']) == sign * 40 for row in r.rows()),
                           message='CSV không giữ yêu cầu 50/90 và đã gửi 20/40')
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
            r.wait(lambda: any(0 < int(row['mask_id']) < 120 and row['planner_state'] == 'BAM_LAN'
                               and float(row['sent_speed_cms']) == 8
                               and abs(float(row['sent_steer_deg'])) > 0
                               for row in r.rows()), 15, 'Chưa tiêu thụ làn lệch tâm trước mất làn', tag='setup')
            r.wait(lambda: any(e[2] == 'MAT_LAN' for e in r.of('reason')), 15, 'Không vào MAT_LAN trước phục hồi')
            need(int(str(r.of('mask')[-1][2]).split()[0]) < 300, 'Fixture không đủ thời gian mất làn', 'setup')
            def recovery_rows():
                rows = r.rows()
                return rows if (any(int(str(e[2]).split()[0]) >= 305 for e in r.of('mask'))
                                and any(int(row['mask_id']) >= 305 for row in rows)) else None
            rows = r.wait(recovery_rows, 20, 'Chưa có dòng CSV tiêu thụ làn phục hồi', tag='setup')
            last_valid, angle = a28_origin(rows, r.events())
            lost = [row for row in rows if 120 <= int(row['mask_id']) < 300]
            need(lost and all(row['reason'] not in ('MAT_ANH', 'MAT_NAN_CU')
                              and 0 <= float(row['image_age_ms']) <= 500
                              and 0 <= float(row['mask_age_ms']) <= BOUNDS['mask_max_age_ms'][0]
                              and 0 <= float(row['t_ms']) - float(row['command_ms']) <= BOUNDS['cmd_max_age_ms'][0]
                              for row in lost),
                 'Dữ liệu cũ che kịch bản mất làn', 'setup')
            seen = set()
            for row in lost:
                age = float(row['t_ms']) - last_valid
                need(math.isfinite(age) and age >= 0, 'Tuổi mất làn không hợp lệ', 'setup')
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
            r.boot(); r.go()
            r.wait(lambda: any(row['planner_state'] == row['reason'] == 'BAM_LAN'
                               and abs(float(row['sent_speed_cms']) - 8) < 1e-6
                               and 0 <= float(row['det_age_ms']) <= r.values['det_max_age_ms']
                               for row in r.rows()), 15, 'Chưa bám làn 8 với lô YOLO tươi')
            speed_is(r, 8)
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
        for cruise, expected, speed in ((8, 'accepted', 8), (20, 'accepted', 20),
                                        (21, 'error', 8), (20.1, 'error', 8)):
            values = dict(YOLO_VALUES)
            values.update(max_speed_cms=10, cruise_cms=8)
            with running(cfg, root / str(cruise), values=values, yolo_schedule=((1, 'red_edge'),),
                         model_load_fail='yolo', setup=lambda r: (r.root / 'yolo.csv').write_text('hong\n', encoding='utf-8')) as r:
                r.boot(); r.go(); speed_is(r, 8)
                need(not r.of('detections'), '--no-yolo vẫn suy luận YOLO')
                end = space(r); mark = len(r.events())
                changed = dict(values)
                changed.update(max_speed_cms=10 if cruise == 8 else 20, cruise_cms=cruise)
                patch_yaml(r, changed); reload(r, expected)
                r.frames(5, lambda: no_motion(r, end))
                r.go(); speed_is(r, speed)
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
        for kind, value in (('motor_speed', 20.01), ('servo_angle', -40.01)):
            try:
                cap([(0, kind, value, False)])
            except Failure as error:
                need(error.tag == 'invariant_cap', 'Checker hỏng sai lý do')
            else:
                raise Failure('self_check', 'Checker bỏ lọt trần')
        cap([(0, 'motor_speed', -20, False), (1, 'servo_angle', 40, False)])
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
    # A81 — T60: chạy bình thường tới 20, trần motor/góc và thông báo khởi động.
    if case == 'A81':
        with running(cfg, root / 'straight', values=dict(max_speed_cms=20, cruise_cms=20, slow_per_deg=0)) as r:
            r.boot()
            need('Trần cứng: 20 cm/s, 40 độ' in r.text() and 'chạy <=20 cm/s' in r.text(),
                 'Thông báo khởi động phải dùng trần 20', 'cap20_banner')
            r.go(); speed_is(r, 20); r.finish()
            events = r.events()
            cap(events)
            speeds = [e[2] for e in events if e[1] == 'motor_speed' and not e[3]]
            need(speeds and max(speeds) == 20, 'Vết motor phải đạt đúng 20', 'cap20_speed')
            inspect_session(r)
        for mode in ('left', 'right'):
            with running(cfg, root / mode, values=dict(max_speed_cms=20, cruise_cms=20, kp=200, kd=0),
                         schedule=((1, mode),)) as r:
                r.boot(); event = r.go()
                r.wait(lambda: len([row for row in r.rows() if float(row['t_ms']) >= event[0]
                                    and abs(float(row['requested_steer_deg'])) >= 20
                                    and float(row['sent_speed_cms']) > 0]) >= 3,
                       message='Thiếu lệnh cua góc lớn tại cruise 20')
                cap20_curve([row for row in r.rows() if float(row['t_ms']) >= event[0]
                             and float(row['sent_speed_cms']) > 0])
                r.finish(); cap(r.events()); inspect_session(r)
        return
    # A60 — T47m LOW-4: lỗi sha256sum vẫn đủ log như A21 và cảnh báo trên stderr.
    if case == 'A60':
        from unittest.mock import patch
        real_sha256 = shutil.which('sha256sum')
        need(real_sha256, 'Không tìm thấy sha256sum thật', 'setup')
        fake_bin = root / 'fake_bin'
        fake_bin.mkdir(parents=True)
        sha256 = fake_bin / 'sha256sum'
        # Chỉ gây lỗi băm bản chép trong phiên; P2 phải băm model thành công trước khởi động.
        sha256.write_text('#!' + sys.executable + '\nimport os, sys\nfrom pathlib import Path\n'
                          'log_root = Path({!r}).resolve()\n'.format(str(root / 'run' / 'out')) +
                          'if any(log_root in Path(arg).resolve().parents for arg in sys.argv[1:] if arg != "--"):\n'
                          ' Path(__file__).with_name("called").touch()\n raise SystemExit(1)\n'
                          'os.execv({0!r}, [{0!r}] + sys.argv[1:])\n'.format(real_sha256), encoding='utf-8')
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
    import ast
    from types import SimpleNamespace
    from unittest.mock import MagicMock, patch
    with tempfile.TemporaryDirectory(prefix='ucr-t47-ignore-selftest-') as tmp:
        here = Path(tmp) / 'tools' / 'auto_drive'
        with patch.dict(globals(), HERE=here), patch.object(shutil, 'which', return_value=None):
            for text, expected in (('  runs_auto/  \n', True), ('/runs_auto/\n', True),
                                   ('runs_auto\n', True), ('# runs_auto/\n', False),
                                   ('runs_auto/\n !runs_auto/keep\n', False)):
                (Path(tmp) / '.gitignore').write_text(text, encoding='utf-8')
                try:
                    check_gitignore()
                except Failure as error:
                    assert not expected and error.tag == 'gitignore'
                else:
                    assert expected
    def fails(call, tag):
        try:
            call()
        except Failure as error:
            assert error.tag == tag
        else:
            assert False, 'Bộ kiểm bỏ lọt ' + tag
    # T71–T73: tự kiểm các bộ kiểm mới bằng cả mẫu hợp lệ và mẫu phải bị bắt.
    for failed in (False, True):
        t71_cap([(0, 'motor_speed', 15, failed), (1, 'servo_angle', -40, failed)])
        fails(lambda: t71_cap([(0, 'motor_speed', 15.1, failed)]), 't71_cap')
    default_value('cruise_cms: 15 (mặc định)', 'cruise_cms', 15)
    for text in ('cruise_cms: 8 (mặc định)', 'cruise_cms: 15', 'other_cruise_cms: 15 (mặc định)'):
        fails(lambda: default_value(text, 'cruise_cms', 15), 't71_default')
    with tempfile.TemporaryDirectory(prefix='ucr-t72-selftest-') as tmp:
        folder = Path(tmp)
        run = SimpleNamespace(session=lambda: folder)
        (folder / 'control_log.csv').write_text('log', encoding='utf-8')
        assert not media_files(run)
        (folder / 'frames').mkdir()
        for name in ('frames/5.jpg', 'frames/5.png', 'frames/10.JPEG'):
            (folder / name).write_bytes(b'image')
        assert len(media_files(run)) == 3
    for kind, limit in (('motor_speed', 20), ('servo_angle', 40)):
        for sign in (-1, 1):
            for failed in (False, True):
                cap([(0, kind, sign * limit, failed)])
                fails(lambda: cap([(0, kind, sign * (limit + 0.1), failed)]), 'invariant_cap')
    for sign in (-1, 1):
        row = dict(requested_steer_deg=sign * 30, requested_speed_cms=17, sent_speed_cms=17)
        cap20_curve([row] * 3)
        fails(lambda: cap20_curve([row] * 2), 'cap20_curve')
        for speed in (20, 20.1, 8, 0):
            bad = dict(row, sent_speed_cms=speed)
            fails(lambda: cap20_curve([bad] * 3), 'cap20_curve')
    assert BOUNDS['max_speed_cms'] == (10, 0, 20, 'cm/s')
    assert BOUNDS['cruise_cms'] == (8, 0, 20, 'cm/s')
    assert BOUNDS['min_cms'] == BOUNDS['lost_cms'] == (4, 0, 10, 'cm/s')
    assert P2_BOUNDS['approach_cms'] == (5, 0, 10)
    class SensorFeed:
        def __init__(self):
            self.process = SimpleNamespace(poll=lambda: None)
            self.commands = []
        def wait(self, predicate, *args, **kwargs):
            for _ in range(5):
                result = predicate()
                if result:
                    return result
            assert False, 'Bộ cấp mẫu không kết thúc hữu hạn'
        def inject(self, command):
            self.commands.append(command)
            return self.wait(lambda: True)
    feed = SensorFeed(); t57_feed(feed)
    feed.wait(lambda: True)
    assert feed.commands == ['imu_sample 0 0', 'encoder_sample 0']
    feed.t57_yaw, feed.t57_odo = 17, 23
    feed.commands[:] = []
    feed.wait(lambda: True)
    assert feed.commands == ['imu_sample 17 0', 'encoder_sample 23']
    feed.t57_yaw = feed.t57_odo = 0
    feed.t57_imu = False; feed.commands[:] = []
    feed.wait(lambda: True)
    assert feed.commands == ['encoder_sample 0']
    feed.t57_encoder = False; feed.commands[:] = []
    feed.wait(lambda: True)
    assert not feed.commands
    # T73 bổ sung: cấp encoder tăng đều, rồi giữ nguyên số đo nhưng vẫn cấp mẫu mới.
    with patch.object(td, 'now_ms', return_value=1000) as clock:
        feed = SensorFeed(); t73_add_feed(feed)
        clock.return_value = 1200
        feed.wait(lambda: True)
        assert feed.commands == ['imu_sample 0 0', 'encoder_sample 1.0']
        feed.t73_progress = False; feed.commands[:] = []; clock.return_value = 1400
        feed.wait(lambda: True)
        assert feed.commands == ['imu_sample 0 0', 'encoder_sample 1.0']
        feed.t73_base, feed.t73_origin, feed.t73_progress = 81, 1400, True
        feed.commands[:] = []; clock.return_value = 1600
        feed.wait(lambda: True)
        assert feed.commands == ['imu_sample 0 0', 'encoder_sample 82.0']
    trace = [(763, 'state', 'BAM_LAN', False), (763, 'reason', 'HUY_RE_CAM_BIEN', False),
             (763, 'command', '12 763', False), (763, 'motor_speed', 15, False)]
    t57_transient_window(740, 760, 730)
    t57_transient_window(740, 760, 730, 763, trace)
    for stalled, resumed, created, observed, events in (
            (100, 600, 100, None, ()),
            (100, 120, -380, None, ()),
            (740, 760, 263, 763, trace),
            (740, 764, 730, 763, trace),
            (740, 760, 730, 763, trace[:2]),
            (740, 760, 730, 763, trace + [(764, 'motor_speed', 0, False)]),
            (740, 760, 730, 763, trace + [(764, 'motor_speed', -1, False)]),
            (740, 760, 730, 763, trace + [(764, 'motor_emergency', 0, False)]),
            (740, 760, 730, 763, trace + [(764, 'state', 'DUNG', False)])):
        fails(lambda: t57_transient_window(stalled, resumed, created, observed, events),
              't57_transient')
    for failed in (('imu',), ('encoder',), ('imu', 'encoder')):
        fault_row = dict(reason='HUY_RE_CAM_BIEN', planner_state='BAM_LAN',
                         sent_speed_cms='15', sent_steer_deg='40',
                         yaw_deg='' if 'imu' in failed else '0',
                         odo_cm='' if 'encoder' in failed else '0')
        t57_fault_csv([fault_row], failed)
        progressed = dict(fault_row, odo_cm='' if 'encoder' in failed else '54')
        t57_fault_csv([progressed], failed, 54)
        if 'encoder' not in failed:
            fails(lambda: t57_fault_csv([fault_row], failed, 54), 't57_fault_csv')
        fails(lambda: t57_fault_csv([], failed), 't57_fault_csv')
        for column in ('yaw_deg', 'odo_cm'):
            bad = dict(fault_row)
            bad[column] = '0' if fault_row[column] == '' else ''
            fails(lambda: t57_fault_csv([bad], failed), 't57_fault_csv')
        for column, value in (('planner_state', 'DUNG'), ('reason', 'LOI_CAM_BIEN'),
                              ('sent_speed_cms', '0'), ('sent_speed_cms', '-1'),
                              ('sent_speed_cms', '15.1'), ('sent_steer_deg', '40.1'),
                              ('yaw_deg', '17'), ('odo_cm', '23')):
            bad = dict(fault_row); bad[column] = value
            fails(lambda: t57_fault_csv([bad], failed), 't57_fault_csv')
    # T57: tự kiểm bộ đọc log bằng CSV đúng và các bản cố ý làm sai.
    row = dict((key, '') for key in T57_OLD_FIELDS + ['yaw_deg', 'odo_cm'])
    t57_log([row])
    fails(lambda: t57_log([]), 't57_log')
    bad = dict(row); bad['yaw_deg'] = '0'
    fails(lambda: t57_log([bad]), 't57_log')
    bad = dict(row); del bad['sent_speed_cms']
    fails(lambda: t57_log([bad]), 't57_log')
    bad = dict(row); del bad['detections_dropped']
    fails(lambda: t57_log([bad]), 't57_log')
    keys = list(row); keys[0], keys[1] = keys[1], keys[0]
    fails(lambda: t57_log([dict((key, row[key]) for key in keys)]), 't57_log')
    bad = dict((key, row[key]) for key in ['yaw_deg', 'odo_cm'] + CSV_FIELDS)
    fails(lambda: t57_log([bad]), 't57_log')
    bad = dict(row); bad['extra_old'] = ''
    fails(lambda: t57_log([bad]), 't57_log')
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
        info = json.loads((root / 'models' / 'yolo_info.json').read_text(encoding='utf-8'))
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
                assert '--no-yolo' not in argv
                assert argv[argv.index('--model-dir') + 1] == str(run.models)
                assert not {'--engine', '--info', '--yolo-engine', '--yolo-info'} & set(argv)
                for name in ('unet', 'yolo'):
                    for suffix in ('.onnx', '_info.json', '.engine', '.engine.src'):
                        assert (run.models / (name + suffix)).is_file()
                    assert (run.models / (name + '.engine.src')).read_bytes() == (
                        hashlib.sha256((run.models / (name + '.onnx')).read_bytes()).hexdigest() + '\n').encode('ascii')
                assert env['UCR_AUTO_MODEL_LOAD_FAIL'] == 'yolo'
                assert env['UCR_AUTO_FAKE_YOLO_TENSORS'] == str(run.root / 'yolo.csv')
                assert 'UCR_AUTO_FAKE_MODEL_LOAD_FAIL' not in env
            finally:
                run.log.close()
        with patch.object(subprocess, 'Popen') as popen:
            run = Run(cfg, root / 'launch_no_yolo')
            try:
                argv = popen.call_args[0][0]
                assert '--no-yolo' in argv and '--model-dir' in argv
                assert not list(run.models.glob('yolo*'))
            finally:
                run.log.close()
        with patch.object(subprocess, 'Popen') as popen:
            run = Run(cfg, root / 'launch_home', no_yolo=False,
                      yolo_schedule=((1, 'empty'),), use_home_models=True)
            try:
                argv, options = popen.call_args[0][0], popen.call_args[1]
                assert '--model-dir' not in argv and '--no-yolo' not in argv
                assert options['env']['HOME'] == str(run.root / 'home')
                assert run.models == run.root / 'home' / 'ucr_models'
                assert not (run.root / 'models').exists()
                assert (run.models / 'unet.onnx').is_file() and (run.models / 'yolo.onnx').is_file()
                assert options['stdout'] is options['stderr'] is run.log
            finally:
                run.log.close()
        # Chạy luồng ca bằng Popen giả: kiểm fixture thật và số lượt, không xác nhận DEV.
        launches = []
        fault = [None]
        @contextmanager
        def model_running(cfg, folder, **kwargs):
            with patch.object(subprocess, 'Popen'):
                run = Run(cfg, folder, **kwargs)
            try:
                bad = []
                for name in ('unet', 'yolo'):
                    if name == 'yolo' and run.no_yolo:
                        continue
                    paths = [run.models / (name + suffix) for suffix in
                             ('.onnx', '_info.json', '.engine', '.engine.src')]
                    if not all(p.is_file() for p in paths):
                        bad.append(name)
                        continue
                    digest = hashlib.sha256(paths[0].read_bytes()).hexdigest().encode('ascii')
                    if paths[3].read_bytes() not in (digest, digest + b'\n'):
                        bad.append(name)
                if struct.unpack('>II', (folder / 'straight.png').read_bytes()[16:24]) != (160, 96):
                    bad.append('unet')
                if not run.no_yolo:
                    raw = (folder / 'empty.npy').read_bytes()
                    length = struct.unpack('<H', raw[8:10])[0]
                    shape = ast.literal_eval(raw[10:10 + length].decode('ascii'))['shape']
                    assert len(raw[10 + length:]) == shape[0] * shape[1] * shape[2] * 4
                    if shape != (1, 13, 100):
                        bad.append('yolo')
                legacy = any(flag in (kwargs.get('args') or []) for flag in
                             ('--engine', '--info', '--yolo-engine', '--yolo-info'))
                invalid = bool(bad or legacy)
                run.process.returncode = 1 if invalid else 0
                run.process.poll.return_value = run.process.returncode
                run.events = MagicMock(return_value=[])
                run.of = MagicMock(return_value=[])
                run.wait = lambda predicate, *args, **kw: predicate()
                run.text = MagicMock(return_value='Lỗi ' + ' '.join('build_engine.sh ' + n for n in bad))
                if fault[0] == 'yolo_hint':
                    run.text.return_value += ' build_engine.sh yolo'
                if fault[0] == 'lock_code' and kwargs.get('lock'):
                    run.process.returncode = 6
                run.boot = MagicMock(side_effect=lambda: need(not invalid, 'Đối chứng bị hỏng', 'setup'))
                run.finish = MagicMock()
                run.frames = lambda count, check: check()
                session = folder / 'session'
                session.mkdir()
                data = {}
                for name in ('unet', 'yolo'):
                    enabled = name == 'unet' or not run.no_yolo
                    info = json.loads((run.models / (name + '_info.json')).read_text(encoding='utf-8')) if enabled and not invalid else {}
                    for field in ('variant', 'seed', 'data_tag', 'onnx_sha256'):
                        data[name + '_' + field] = (hashlib.sha256((run.models / (name + '.onnx')).read_bytes()).hexdigest()
                                                   if field == 'onnx_sha256' else info.get(field, '')) if enabled and not invalid else ''
                    key = 'engine_sha256' if name == 'unet' else 'yolo_engine_sha256'
                    data[key] = hashlib.sha256((run.models / (name + '.engine')).read_bytes()).hexdigest() if enabled and not invalid else ''
                data['engine_name'] = str(run.models / 'unet.engine')
                if fault[0] == 'home_path':
                    data['engine_name'] = str(folder / 'wrong' / 'unet.engine')
                if fault[0] == 'yolo_session':
                    data['yolo_variant'] = 'khong rong'
                (session / 'session.json').write_text(json.dumps(data), encoding='utf-8')
                run.session = lambda: session
                launches.append((folder, invalid))
                yield run
                assert run.boot.call_count == run.finish.call_count == (0 if invalid else 1)
            finally:
                run.log.close()
        counts = dict(A70u=5, A70y=5, A71u=2, A71y=2, A72u=2, A72y=2, A73=2,
                      A74ua=5, A74ub=6, A74ya=5, A74yb=6, A75=2, A77=4, A78=2, A79=5, A80=1)
        flock = SimpleNamespace(flock=MagicMock(), LOCK_EX=2, LOCK_NB=4)
        with patch('{}.running'.format(__name__), model_running), patch.dict(sys.modules, fcntl=flock):
            for case, count in counts.items():
                launches[:] = []
                test_case(case, cfg, root / ('flow_' + case))
                assert len(launches) == count <= 6
                assert not launches[0][1], 'Thiếu đối chứng hợp lệ ' + case
                if case.startswith('A74'):
                    assert not launches[1][1] and all(bad for _, bad in launches[2:])
                elif case not in ('A73', 'A77', 'A80'):
                    assert all(bad for _, bad in launches[1:])
                else:
                    assert not any(bad for _, bad in launches)
            assert flock.flock.call_count == 1
            for broken, case, tag in (('yolo_hint', 'A71u', 'model_rebuild'),
                                      ('lock_code', 'A78', 'model_preflight'),
                                      ('home_path', 'A80', 'model_session'),
                                      ('yolo_session', 'A77', 'model_session')):
                fault[0] = broken
                fails(lambda: test_case(case, cfg, root / ('mutant_' + broken)), tag)
        # Bộ kiểm mới phải bắt cả lệnh 0 / lệnh fail, không chỉ chuyển động.
        probe = MagicMock()
        probe.process.returncode = 1
        probe.of.return_value = []
        probe.text.return_value = 'Lỗi model'
        probe.events.return_value = []
        reject_model(probe)
        for kind in ('motor_init', 'motor_speed', 'motor_emergency', 'servo_angle', 'test_ucr_stop'):
            for failed in (False, True):
                probe.events.return_value = [(0, kind, 0, failed)]
                fails(lambda: reject_model(probe), 'model_preflight')
        # Session tổng hợp: bắt thiếu khóa, ghi cứng, băm sai và no-yolo không rỗng.
        for no_yolo in (False, True):
            models = root / ('session_models_' + str(no_yolo))
            data = {}
            for name, seed in (('unet', 17), ('yolo', 29)):
                enabled = name == 'unet' or not no_yolo
                meta = dict(variant=name + '_variant', seed=seed, data_tag=name + '_tag')
                if enabled:
                    model_files(models, name, meta)
                for field in ('variant', 'seed', 'data_tag', 'onnx_sha256'):
                    data[name + '_' + field] = (hashlib.sha256((models / (name + '.onnx')).read_bytes()).hexdigest()
                                                if field == 'onnx_sha256' else meta[field]) if enabled else ''
                key = 'engine_sha256' if name == 'unet' else 'yolo_engine_sha256'
                data[key] = hashlib.sha256((models / (name + '.engine')).read_bytes()).hexdigest() if enabled else ''
            session_run = SimpleNamespace(models=models, no_yolo=no_yolo, session=lambda: root)
            def write_session(value):
                (root / 'session.json').write_text(json.dumps(value), encoding='utf-8')
            write_session(data); inspect_model_session(session_run)
            for key in data:
                bad = dict(data); del bad[key]
                write_session(bad)
                fails(lambda: inspect_model_session(session_run), 'model_session')
                if key != 'yolo_engine_sha256' or not no_yolo:
                    bad = dict(data); bad[key] = 'sai'
                    write_session(bad)
                    fails(lambda: inspect_model_session(session_run), 'model_session')
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
    # T68a: thiếu phiếu, phiếu trước G/cũ hoặc rẽ trước quãng đều phải bị bắt.
    signs = [(1000, 'detections', '{} {}'.format(frame, 900 + frame), False) for frame in (1, 2, 3)]
    def turn_probe(events, direction='left'):
        run = SyntheticRun(list(events))
        run.t57_odo = 0
        run.values['vote_min'] = 3
        run.frames = lambda count, check: [check() for _ in range(count)]
        def wait(predicate, *args, **kwargs):
            if run.t57_odo == 30:
                run.es.append((1001, 'state', 'RE_TRAI' if direction == 'left' else 'RE_PHAI', False))
            result = predicate()
            need(result, 'Không đủ điều kiện tổng hợp T68a', 't68a_wait')
            return result
        run.wait = wait
        return run
    with patch.object(td, 'now_ms', return_value=1000):
        for direction in ('left', 'right'):
            probe = turn_probe(signs, direction)
            turn = t68a_turn(probe, 0, 900, direction)
            assert probe.t57_odo == 30 and turn[2] == ('RE_TRAI' if direction == 'left' else 'RE_PHAI')
        for bad in (signs[:2], [(1000, 'detections', '1 900', False)] + signs[1:],
                    [(1000, 'detections', '{} {}'.format(frame, frame), False) for frame in (1, 2, 3)]):
            fails(lambda: t68a_turn(turn_probe(bad), 0, 900), 't68a_wait')
        fails(lambda: t68a_turn(turn_probe(signs + [(1000, 'state', 'RE_TRAI', False)]), 0, 900),
              't68a_start')
    # Lô rỗng bị bỏ qua phải là setup; servo0 vòng thường trước khi nhận lô không thuộc cặp dừng.
    control = [(90, 'motor_speed', 15, False), (100, 'detections', '120 90', False),
               (105, 'command', '120 105', False), (106, 'motor_speed', 15, False),
               (107, 'servo_angle', 0, False), (120, 'detections', '123 110', False),
               (125, 'command', '123 125', False), (126, 'motor_speed', 15, False),
               (127, 'servo_angle', 0, False), (140, 'detections', '126 130', False),
               (141, 'servo_angle', 0, False), (145, 'command', '126 145', False),
               (146, 'motor_speed', 0, False), (147, 'servo_angle', 0, False),
               (147, 'reason', 'DUNG_DEN_DO', False)]
    samples = [dict(det_frame_id=str(frame), det_captured_ms=str(captured), command_ms=str(command))
               for frame, captured, command in ((120, 90, 105), (123, 110, 125), (126, 130, 145))]
    def setup_wait(predicate, *args, **kwargs):
        result = predicate()
        need(result, 'Thiếu tiền đề tổng hợp', kwargs.get('tag', 'interface'))
        return result
    sync = SyntheticRun(control)
    sync.rows = lambda: samples
    sync.wait = setup_wait
    batches = detection_batches(control)
    assert t62_consumed(sync, batches) == samples
    assert t62_control_mark(sync, batches[-1], samples[-1]) == (145, 12)
    assert red_stop(sync, 120, 0, consumed=True) == 14
    fails(lambda: red_stop(SyntheticRun(control), 120, 0), 'stop_order')
    missing = samples.pop(1)
    fails(lambda: t62_consumed(sync, batches), 'setup')
    samples.insert(1, dict(missing, det_captured_ms='111'))
    fails(lambda: t62_consumed(sync, batches), 'setup')
    samples[1] = missing
    late = SyntheticRun(control[:12] + [(400, 'motor_speed', 0, False),
                                       (401, 'servo_angle', 0, False), control[-1]])
    late.rows = sync.rows
    fails(lambda: red_stop(late, 120, 0, consumed=True), 'stop_order')
    fails(lambda: t62_window([], [], 0, 100, 5, 'TIEP_CAN_DEN_DO', setup=True), 'setup')
    fails(lambda: t62_window([(1, 'motor_speed', 15, False)], [], 0, 100, 5,
                            'TIEP_CAN_DEN_DO', setup=True), 't62_window')
    class RedRetry(SyntheticRun):
        def __init__(self, gaps):
            super().__init__([(0, 'detections', '1 0', False)])
            self.gaps, self.attempts = list(gaps), 0
        def of(self, kind, mark=0):
            if kind == 'frame':
                return [(0, 'frame', '999 {}'.format(detection_batches(self.es)[-1][2] + 200), False)]
            return super().of(kind, mark)
        def inject(self, command):
            if command == 'yolo_throw_off':
                self.attempts += 1
                captured = detection_batches(self.es)[-1][2] + self.gaps.pop(0)
                self.es.append((captured, 'detections', '{} {}'.format(self.attempts + 1, captured), False))
        def rows(self):
            return [dict(det_frame_id=str(b[1]), det_captured_ms=str(b[2])) for b in detection_batches(self.es)]
    for delta in (170, 200):
        for gaps in ([400, delta], [400, 400, delta]):
            retry = RedRetry(gaps)
            assert t62_next_red(retry, delta)[2] == sum(gaps) and retry.attempts == len(gaps)
        retry = RedRetry([400, 400, 400])
        fails(lambda: t62_next_red(retry, delta, attempts=3), 'setup')
        assert retry.attempts == 3
    # Kiểm đường gọi thật: vòng ba lô không thay thế ba lần thử của từng lô.
    for case in ('t62_hold_170', 't62_hold_200', 't62_reset_space', 't62_reset_go', 't62_reset_image'):
        with patch('{}.running'.format(__name__)) as start, \
                patch('{}.t62_next_red'.format(__name__)) as next_red, \
                patch('{}.external_stamp_ms'.format(__name__), side_effect=[0, Failure('retry_probe', '')]):
            run = start.return_value.__enter__.return_value
            run.events.return_value = [(0, 'detections', '1 0', False)]
            fails(lambda: t62_case(case, None, Path('synthetic')), 'retry_probe')
            assert next_red.call_count == 3
            assert all(args == (run, 170 if case.endswith('170') else 200) and kwargs == dict(attempts=3)
                       for args, kwargs in next_red.call_args_list)
    # T73: chạy cả hai đường dừng; khác0 sau motor0 phải bị bắt dù cùng ms và đã trả0.
    class FaultStopRun(SyntheticRun):
        def __init__(self, bad):
            super().__init__([])
            self.bad = bad
            self.process = SimpleNamespace(pid=123)
        def boot(self):
            pass
        def go(self):
            event = (0, 'go', '', False)
            self.es.extend([event, (1, 'state', 'RE_TRAI', False)])
            return event
        def inject(self, command):
            assert command == 'imu_read_fail'
            self.es.extend([(2, 'reason', 'HUY_RE_CAM_BIEN', False), (2, 'state', 'BAM_LAN', False),
                            (2, 'motor_speed', 15, False)])
        def frames(self, count=5, check=lambda: None):
            check()
        def stop(self):
            commands = [(100, 'motor_speed', 15, False), (100, 'servo_angle', 0, False),
                        (100, 'motor_speed', 0, False), (100, 'servo_angle', 0, False)]
            if self.bad:
                commands.insert(self.bad[0], (100, self.bad[1], 1, False))
                commands.extend([(100, 'motor_speed', 0, False), (100, 'servo_angle', 0, False)])
            self.es.extend(commands)
        def key(self, key):
            assert key == ' '
            self.stop()
            return (100, 'key', key, False)
        def exited(self):
            pass
        def finish(self):
            pass
    for action in ('space', 'term'):
        for bad in (None, (3, 'motor_speed'), (3, 'servo_angle'), (4, 'motor_speed'), (4, 'servo_angle')):
            runs = []
            @contextmanager
            def fault_running(cfg, root, **kwargs):
                run = FaultStopRun(bad if root.name == action else None)
                runs.append(run)
                yield run
            with patch('{}.running'.format(__name__), fault_running), \
                    patch('{}.t73_add_feed'.format(__name__)), \
                    patch('{}.t68a_turn'.format(__name__), side_effect=lambda run, *a, **k: (setattr(run, 't57_odo', 30), run.of('state')[-1])[1]), \
                    patch.object(td, 'now_ms', return_value=100), \
                    patch.object(os, 'kill', create=True, side_effect=lambda *args: runs[-1].stop()):
                call = lambda: t73_add_case('t73_fault_stop_order', None, Path('synthetic'))
                if bad:
                    fails(call, 'light_latch')
                else:
                    call()
                    assert len(runs) == 2
    # A28: ảnh đã tới 121 nhưng lệnh cuối chỉ dùng mask118; mask119 chỉ mới công bố.
    masks = [(1000, 'mask', '118 1000', False), (1030, 'mask', '119 1030', False)]
    lane = dict(mask_id='118', frame_id='121', command_ms='1040', csv_dropped='0',
                t_ms='1045', image_age_ms='5', mask_age_ms='45',
                planner_state='BAM_LAN', sent_speed_cms='8', sent_steer_deg='12', requested_steer_deg='12')
    assert a28_origin([lane, lane], masks) == (1000, 12)
    # Góc gửi âm và khác góc yêu cầu vẫn là tiền đề hợp lệ khi hai lệnh gửi đã ổn định.
    negative = dict(lane, sent_steer_deg='-4', requested_steer_deg='-4.25')
    assert a28_origin([negative, negative], masks) == (1000, -4)
    counted = dict(negative, csv_dropped='1')
    lost = dict(counted, mask_id='120')
    recovered = dict(counted, mask_id='305')
    window = [counted, counted, lost, lost, recovered]
    assert a28_origin([lane] + window, masks) == (1000, -4)
    for i in range(1, len(window)):
        dropped = window[:i] + [dict(row, csv_dropped='2') for row in window[i:]]
        fails(lambda: a28_origin(dropped, masks), 'setup')
    fails(lambda: a28_origin([counted, dict(lost, mask_id='0', csv_dropped='2'), counted], masks), 'setup')
    with patch('{}.running'.format(__name__)) as start:
        run = start.return_value.__enter__.return_value
        run.rows.return_value = [negative, negative]
        def first_lane(predicate, *args, **kwargs):
            need(predicate(), 'Không nhận làn lệch tâm với góc gửi âm', 'setup')
            raise Failure('origin_probe', '')
        run.wait.side_effect = first_lane
        fails(lambda: test_case('A28', None, Path('synthetic')), 'origin_probe')
    # Mask đã công bố nhưng CSV chưa ghi: chờ trong đúng hạn sẵn có, không đo sớm.
    for arrives in (False, True):
        with patch('{}.running'.format(__name__)) as start, \
                patch('{}.a28_origin'.format(__name__), side_effect=Failure('origin_probe', '')) as origin:
            run = start.return_value.__enter__.return_value
            run.rows.return_value = window[:2]
            run.of.return_value = [(1100, 'mask', '299 1100', False)]
            def csv_wait(predicate, seconds, *args, **kwargs):
                if seconds != 20:
                    return True
                assert kwargs['tag'] == 'setup'
                run.of.return_value = [(1200, 'mask', '305 1200', False)]
                assert predicate() is None
                if arrives:
                    run.rows.return_value = window
                return setup_wait(predicate, **kwargs)
            run.wait.side_effect = csv_wait
            fails(lambda: test_case('A28', None, Path('synthetic')), 'origin_probe' if arrives else 'setup')
            assert run.wait.call_args[0][1] == 20
            if arrives:
                assert origin.call_args[0][0] is window
            else:
                origin.assert_not_called()
    fails(lambda: a28_origin([], masks), 'setup')
    fails(lambda: a28_origin([dict(lane, sent_steer_deg='11'), lane], masks), 'setup')
    fails(lambda: a28_origin([lane, dict(lane, csv_dropped='1')], masks), 'setup')
    fails(lambda: a28_origin([lane, lane], masks[1:]), 'setup')
    for column, value in (('sent_steer_deg', '0'), ('sent_steer_deg', 'nan'),
                          ('sent_steer_deg', 'inf'), ('image_age_ms', '501'),
                          ('mask_age_ms', '301'), ('command_ms', '844')):
        bad = dict(lane, **{column: value})
        fails(lambda: a28_origin([bad, bad], masks), 'setup')
    lane_events = [(0, 'state', 'RE_TRAI', False), (1, 'motor_speed', 4, False),
                   (2, 'state', 'BAM_LAN', False), (3, 'motor_speed', 15, False)]
    t73_lane_check(SyntheticRun(lane_events), 0, 2)
    fails(lambda: t73_lane_check(SyntheticRun(lane_events), 0), 't73_lane')
    for bad in ((4, 'state', 'RE_PHAI', False), (4, 'state', 'DUNG', False),
                (4, 'motor_speed', 0, False), (4, 'motor_emergency', 0, False)):
        fails(lambda: t73_lane_check(SyntheticRun(lane_events + [bad]), 0, 2), 't73_lane')
    t73_add_lane(SyntheticRun(lane_events), 0, 2)
    fails(lambda: t73_add_lane(SyntheticRun(lane_events + [(4, 'motor_speed', -1, False)]),
                               0, 2), 't73_add_lane')
    cancelled = lane_events[:3] + [(2, 'reason', 'HUY_RE_CAM_BIEN', False)] + lane_events[3:]
    assert t73_add_cancel(SyntheticRun(cancelled), 0) == 2
    for bad in ((4, 'state', 'RE_PHAI', False), (4, 'state', 'DUNG', False),
                (4, 'motor_speed', 0, False), (4, 'motor_emergency', 0, False)):
        fails(lambda: t73_add_cancel(SyntheticRun(cancelled + [bad]), 0), 't73_lane')
    samples = [(stamp, 'inject', '{} {}_sample 0{}'.format(stamp, sensor, ' 0' if sensor == 'imu' else ''), False)
               for stamp in (0, 200, 400) for sensor in ('imu', 'encoder')]
    t73_add_fresh(SyntheticRun(samples), 0, 400)
    fails(lambda: t73_add_fresh(SyntheticRun(samples), 0, 601), 'setup')
    for sensor in ('imu', 'encoder'):
        missing = [e for e in samples if sensor + '_sample' not in e[2]]
        fails(lambda: t73_add_fresh(SyntheticRun(missing), 0, 400), 'setup')
        gap = [e for e in samples if not (e[0] == 200 and sensor + '_sample' in e[2])]
        fails(lambda: t73_add_fresh(SyntheticRun(gap), 0, 400), 'setup')
    # Q9: bộ kiểm hồi phải bắt rẽ sớm, dùng lại phiếu cũ, khóa cả lượt và mẫu đứt quãng.
    class RecoveryRun(SyntheticRun):
        def __init__(self, direction, bad=None):
            super().__init__([(0, 'state', 'BAM_LAN', False), (0, 'motor_speed', 15, False),
                              (0, 'inject', '0 imu_sample 0 0', False),
                              (0, 'inject', '0 encoder_sample 0', False)])
            self.clock, self.t57_odo, self.yolo, self.resumes, self.late_votes = 0, 0, False, 0, 0
            self.direction, self.bad = direction, bad
            self.late_mode, self.armed_odo = False, None
            self.values.update(vote_min=3)
        def inject(self, command):
            event = (self.clock, 'inject', '0 ' + command, False)
            self.es.append(event)
            if command.startswith('yolo_'):
                self.yolo = command == 'yolo_resume'
                self.resumes += int(self.yolo)
            return event
        def tick(self):
            self.clock += 20
            for sensor in ('imu', 'encoder'):
                if not (self.bad == 'stale' and sensor == 'encoder' and 400 < self.clock < 800):
                    self.inject(sensor + '_sample 0' + (' 0' if sensor == 'imu' else ''))
            self.es.append((self.clock, 'motor_speed', 15, False))
            frame = self.clock // 20
            if self.yolo:
                self.late_votes += int(self.resumes > 1)
                if not (self.bad == 'missing_votes' and self.late_votes > 2):
                    self.es.append((self.clock, 'detections', '{} {}'.format(frame, self.clock), False))
                if self.armed_odo is None and (not self.late_mode or frame >= 120):
                    self.armed_odo = self.t57_odo
            early = self.bad == 'early' and self.clock >= 100 and self.t57_odo >= 30
            old = self.bad == 'old_votes' and self.clock >= 1000
            ready = self.armed_odo is not None and self.t57_odo >= self.armed_odo + 30 and self.clock >= 1000 and self.yolo
            if ready and self.bad == 'stop':
                self.es.append((self.clock, 'motor_speed', 0, False))
            if early or old or (ready and self.bad != 'permanent'):
                self.es.append((self.clock, 'state', 'RE_TRAI' if self.direction == 'left' else 'RE_PHAI', False))
        def wait(self, predicate, seconds=3, message='', **kwargs):
            for _ in range(int(seconds * 50)):
                self.tick()
                result = predicate()
                if result:
                    return result
            raise Failure('t73_recover', message)
        def frames(self, count=5, check=lambda: None):
            for _ in range(count):
                self.tick(); check()
    for direction in ('left', 'right'):
        for early_votes in (False, True):
            for fresh_since in (None, 0):
                probe = RecoveryRun(direction)
                probe.late_mode = not early_votes
                with patch.object(td, 'now_ms', side_effect=lambda: probe.clock):
                    assert t73_recover(probe, 0, direction=direction, fresh_since=fresh_since,
                                       early_votes=early_votes)[0] >= 1000
    for bad, tag in (('early', 't73_lane'), ('old_votes', 't73_lane'),
                     ('permanent', 't73_recover'), ('stale', 'setup'),
                     ('missing_votes', 't73_recover'), ('stop', 't73_recover')):
        probe = RecoveryRun('left', bad)
        with patch.object(td, 'now_ms', side_effect=lambda: probe.clock):
            fails(lambda: t73_recover(probe, 0), tag)
    assert len(CASES) == len(set(CASES))
    cmake = (HERE / 'tests.cmake').read_text(encoding='utf-8')
    for case in CASES:
        if case.startswith(('t62_', 't71_', 't72_', 't73_')):
            assert re.search(r'\b' + re.escape(case) + r'\b', cmake), 'Chưa đăng ký ' + case
    assert green_release(SyntheticRun(events), 3, 0)[1] == 9
    # Không nhận lô có ảnh chụp trước G dù công bố sau G.
    since_events = [(100, 'detections', '1 89', False)] + events
    assert green_release(SyntheticRun(since_events), 1, 0, since=89)[1] == 9
    assert len(phase_batches(SyntheticRun(events), 3, 9, 2)) == 2
    fails(lambda: phase_batches(SyntheticRun(events), 3, 9, 3), 'setup')
    moving(SyntheticRun([(100, 'motor_speed', 8, False)]), 0)
    for maximum, speed, valid in ((10, 10, True), (10, 11, False),
                                  (20, 20, True), (20, 21, False), (30, 21, False)):
        run = SyntheticRun([(100, 'motor_speed', speed, False)])
        run.values['max_speed_cms'] = maximum
        if valid:
            moving(run, 0)
        else:
            fails(lambda: moving(run, 0), 'red_boundary')
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
    parser.add_argument('--case', choices=CASES + ['B5', 'gitignore'])
    parser.add_argument('--hooks')
    parser.add_argument('--slack', type=int, choices=(0, 300), default=0)
    cfg = parser.parse_args()
    if cfg.selftest:
        selftest()
        print('PASS selftest: gitignore, P1/M7, YOLO, CLI/env, chốt đèn, A32, model T47l, log T57 và bộ kiểm T68a/T71–T73 (vết tổng hợp)')
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
