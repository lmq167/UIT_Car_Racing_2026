"""Hợp đồng T25 (ca D114-D121): chỉ dùng bản giả, PTY (terminal giả) và vết lệnh công khai."""
import csv
from pathlib import Path
import signal
import subprocess

import test_drive_collect as d


def journal(run):
    paths = list(run.out.glob('*/stops.csv'))
    if not paths:
        return []
    with paths[0].open(encoding='utf-8', newline='') as stream:
        return list(csv.DictReader(stream))


def reason(run, wanted):
    return run.wait(lambda: next((r for r in journal(run) if r['reason'] == wanted), None),
                    message='Thiếu lý do dừng ' + wanted)


def reverse(run):
    run.ready()
    mark = len(run.events())
    run.key('S')
    try:
        event = run.event('motor_speed', run.hardware_speed(-run.reverse_speed), mark)
    except d.SafetyFailure as error:
        raise d.SafetyFailure('reverse_motion', str(error))
    d.need(d.matched(run.events()[mark:], 'state', 'CHAY_LUI'), 'S không vào CHAY_LUI', 'reverse_motion')
    run.pump_key = 'S'
    run.pump = True
    run.next_w = d.time.monotonic() + 0.1
    return event


def quiet(run, mark):
    d.need(not any(e[1] == 'motor_speed' and e[2] != 0 for e in run.events()[mark:]),
           'Phím cũ tự chạy/lật hướng', 'direction_fresh')


def test_case(n, cfg, root):
    if n == 114:
        for label, args in (('default', []), ('3', ['--reverse-speed-cms', '3']),
                            ('5', ['--reverse-speed-cms', '5']),
                            ('30', ['--reverse-speed-cms', '30'])):
            with d.running(cfg, root / label, args=args + ['--interval-ms', '100']) as run:
                run.boot()
                reverse(run)
                run.observe(0.35)
                d.need(not any(r['reason'] == 'key_s' for r in journal(run)), 'S vẫn là dừng', 'reverse_motion')
                run.wait(lambda: any(r['state'] == 'CHAY_LUI' and float(r['cmd_speed']) == -run.reverse_speed
                                     for r in run.rows()), message='CSV không ghi lệnh lùi')
                d.need('S = LÙI (giữ). DỪNG = Space / X / Esc. S KHÔNG CÒN LÀ DỪNG.' in run.text(),
                       'Thiếu bảng phím khởi động')
                run.finish()
        help_text = subprocess.check_output([cfg.hooks, '--help'], timeout=5).decode('utf-8')
        d.need('S KHÔNG CÒN LÀ DỪNG.' in help_text and '--reverse-speed-cms' in help_text, 'Thiếu trợ giúp lùi')
    elif n == 115:
        for initial in ('W', 'S'):
            with d.running(cfg, root / initial) as run:
                run.boot()
                reverse(run) if initial == 'S' else run.keep_driving()
                run.pump = False
                run.event('motor_speed', 0, len(run.events()))  # Hết hạn giữ hướng cũ.
                run.ready()
                key, speed = ('S', -3) if initial == 'W' else ('W', 5)
                mark = len(run.events())
                pressed = run.key(key)
                zero = d.stop_by(run, mark, pressed[0])
                row = reason(run, 'dir_change')
                d.need(float(row['t_motor0_done_ms']) >= pressed[0] - 5, 'Sai mốc ghi xong 0')
                # Hàng cũ và phím giữ qua nhiều hạn không được biến thành lần nhấn mới.
                for _ in range(6):
                    run.send(key * 256)
                    run.observe(0.2, lambda: quiet(run, mark))
                run.observe((run.hold_timeout + 50) / 1000, lambda: quiet(run, mark))
                fresh = len(run.events())
                run.key(key)
                moved = run.event('motor_speed', speed, fresh)
                d.need(moved[0] - zero[0] >= 300, 'Đổi hướng trước 300 ms', 'direction_pause')
                run.finish()
    elif n == 116:
        for sequence in ('WS', 'SW'):
            with d.running(cfg, root / sequence) as run:
                run.boot()
                mark = len(run.events())
                # Hai phím (< ngưỡng lô dồn 4 phím) để chính bộ xét WS/SW cùng lô ra quyết định.
                run.send(sequence)
                reason(run, 'reverse_conflict')
                run.observe(0.35, lambda: quiet(run, mark))
                # Không được có lệnh thoáng qua trước khi nhận ra xung đột trong cùng lô.
                quiet(run, mark)
                run.observe((run.hold_timeout + 50) / 1000)
                reverse(run)
                run.finish()
        for initial in ('W', 'S'):
            with d.running(cfg, root / ('separate_' + initial)) as run:
                run.boot()
                reverse(run) if initial == 'S' else run.keep_driving()
                run.pump = False
                mark = len(run.events())
                pressed = run.key('S' if initial == 'W' else 'W')
                d.stop_by(run, mark, pressed[0])
                reason(run, 'reverse_conflict')
                run.observe(0.35, lambda: quiet(run, mark))
                run.finish()
    elif n == 117:
        with d.running(cfg, root / 'no_frame', env={'UCR_DRIVE_FAKE_CAMERA': 'none'}) as run:
            run.boot(images=False)
            mark = len(run.events())
            run.key('S')
            run.observe(0.3, lambda: quiet(run, mark))
            run.finish()
        with d.running(cfg, root / 'stale') as run:
            run.boot()
            reverse(run)
            mark = len(run.events())
            stamp = run.inject('camera_stall')
            d.stop_by(run, mark, stamp, 500)
            reason(run, 'frame_stale')
            # S vẫn được giữ nhưng ảnh cũ không cho chạy lại.
            stopped = len(run.events())
            run.observe(0.3, lambda: quiet(run, stopped))
            run.pump = False
            run.finish()
    elif n == 118:
        for i, value in enumerate(('31', '30.01', '0', '-1', 'nan', 'NaN', 'inf', '-inf', '3x')):
            d.rejected(cfg, root / str(i), ['--reverse-speed-cms', value])
    elif n == 119:
        for cap in (10, 20, 40):
            with d.running(cfg, root / str(cap), args=['--max-steer-deg', str(cap), '--steer-step-deg', '10']) as run:
                run.boot()
                for _ in range(cap // 10 + 1):
                    run.key('A')
                run.event('servo_angle', cap)
                for _ in range(2 * cap // 10 + 1):
                    run.key('D')
                run.event('servo_angle', -cap)
                run.finish()
        # Trần góc theo bản chung: tối đa 60; bước lái không có trần riêng.
        d.rejected(cfg, root / 'invalid_0', ['--max-steer-deg', '61'])
    elif n == 120:
        for action in (' ', 'X', 'ESC', 'CTRL_C', 'motor_fail', 'servo_fail'):
            with d.running(cfg, root / action.strip().replace(' ', '_')) as run:
                run.boot()
                reverse(run)
                d.steer(run, 'A', 5)
                run.pump = False
                mark, stamp = len(run.events()), d.monotonic_ms()
                if action in ('motor_fail', 'servo_fail'):
                    run.inject(action)
                    run.key('S' if action == 'motor_fail' else 'A')
                    d.stop_by(run, mark, stamp)
                    run.finish(action)
                elif action == 'CTRL_C':
                    run.process.send_signal(signal.SIGINT)
                    d.stop_by(run, mark, stamp)
                    run.finish('sigint')
                elif action == 'ESC':
                    run.finish()
                else:
                    pressed = run.key(action)
                    d.stop_by(run, mark, pressed[0])
                    if action == 'X':
                        d.ordered_stop(run.events()[mark:])
                    run.finish()
    elif n == 121:
        result = subprocess.run([cfg.contract, 'direction_pause'], stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, timeout=10)
        if result.returncode != 0:
            d.need(result.stdout.strip() == b'direction_pause', 'Lỗi thiết lập ca nghỉ: ' + repr(result.stdout),
                   'mutation_setup')
        d.need(result.returncode == 0, result.stdout.decode('utf-8', errors='replace'), 'direction_pause')
    elif n in (122, 123, 124):
        invert_case(n, cfg, root)
    else:
        raise AssertionError('Chưa cài đặt ca lùi ' + str(n))


def invert_case(n, cfg, root):
    """T31: dấu phần cứng tách khỏi tốc độ logic, giữ nguyên đường dừng/đổi hướng."""
    if n == 122:
        for invert in (False, True):
            for key, logical, state in (('W', 7, 'CHAY_TIEN'), ('S', -30, 'CHAY_LUI')):
                args = ['--speed-cms', '7', '--reverse-speed-cms', '30', '--interval-ms', '100']
                if invert:
                    args.append('--invert-motor-direction')
                with d.running(cfg, root / ('{}_{}'.format(invert, key)), args=args) as run:
                    run.boot()
                    mark = len(run.events())
                    run.key(key)
                    run.event('motor_speed', -logical if invert else logical, mark)
                    run.pump_key, run.pump = key, True
                    run.next_w = d.time.monotonic() + 0.1
                    run.wait(lambda: any(float(r['cmd_speed']) == logical and r['state'] == state
                                         for r in run.rows()), message='CSV phải giữ dấu logic')
                    run.pump = False
                    mark = len(run.events())
                    pressed = run.key(' ')
                    d.stop_by(run, mark, pressed[0])
                    quiet(run, mark)
                    run.finish()
                    zeros = [line.split()[2] for line in run.trace.read_text(encoding='utf-8').splitlines()
                             if len(line.split()) >= 3 and line.split()[1] == 'motor_speed'
                             and float(line.split()[2]) == 0]
                    d.need(zeros and all(not value.startswith('-') for value in zeros), 'Lệnh dừng thành -0')
        help_text = subprocess.check_output([cfg.hooks, '--help'], timeout=5).decode('utf-8')
        d.need('--invert-motor-direction' in help_text and '>0..30' in help_text, 'Thiếu trợ giúp T31')
    elif n == 123:
        for initial in ('W', 'S'):
            for action in (' ', 'X', 'deadman'):
                with d.running(cfg, root / ('{}_{}'.format(initial, action)),
                               args=['--invert-motor-direction', '--reverse-speed-cms', '30']) as run:
                    run.boot()
                    reverse(run) if initial == 'S' else run.keep_driving()
                    d.steer(run, 'A', 5)
                    run.pump = False
                    mark = len(run.events())
                    if action == 'deadman':
                        row = reason(run, 'deadman')
                        d.stop_by(run, mark, float(row['t_trigger_ms']))
                    else:
                        pressed = run.key(action)
                        d.stop_by(run, mark, pressed[0])
                        reason(run, 'key_x' if action == 'X' else 'key_space')
                    quiet(run, mark)
                    if action == 'X':
                        run.event('servo_angle', 0, mark)
                    else:
                        # Space/deadman giữ góc; chỉ nhấn C sau khi nhả đủ hạn mới trả lái.
                        d.need(not d.matched(run.events()[mark:], 'servo_angle'), 'Đường dừng tự đổi góc')
                        run.ready()
                        d.steer(run, 'C', 0)
                    d.ordered_stop(run.events()[mark:])
                    quiet(run, mark)
                    run.finish()
    else:
        for initial in ('W', 'S'):
            for mode in ('dir_change', 'reverse_conflict', 'batch_conflict'):
                with d.running(cfg, root / (initial + '_' + mode),
                               args=['--invert-motor-direction', '--reverse-speed-cms', '30']) as run:
                    run.boot()
                    reverse(run) if initial == 'S' else run.keep_driving()
                    d.steer(run, 'A', 5)
                    run.pump = False
                    if mode == 'dir_change':
                        run.event('motor_speed', 0, len(run.events()))
                        reason(run, 'deadman')
                        run.ready()
                    mark = len(run.events())
                    opposite = 'S' if initial == 'W' else 'W'
                    stamp = d.monotonic_ms()
                    if mode == 'batch_conflict':
                        run.send('WS' if initial == 'W' else 'SW')
                    else:
                        stamp = run.key(opposite)[0]
                    zero = d.stop_by(run, mark, stamp)
                    reason(run, 'dir_change' if mode == 'dir_change' else 'reverse_conflict')
                    quiet(run, mark)
                    d.need(not d.matched(run.events()[mark:], 'servo_angle'), 'Đổi hướng tự đổi góc')
                    run.ready()
                    d.steer(run, 'C', 0)
                    d.ordered_stop(run.events()[mark:])
                    quiet(run, mark)
                    fresh = len(run.events())
                    run.key(opposite)
                    logical = -30 if opposite == 'S' else 5
                    moved = run.event('motor_speed', -logical, fresh)
                    d.need(moved[0] - zero[0] >= 300, 'Đổi hướng trước 300 ms', 'direction_pause')
                    run.finish()
