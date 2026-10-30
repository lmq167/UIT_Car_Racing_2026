"""T21b_design §1–§10: kiểm chấp nhận preview bằng con giả, không mở X thật.

Python 3.6, thư viện chuẩn; dùng lại PTY/vết phần cứng của T21.
P16–P18 vá bản sao sau khi DEV nối xong; không bỏ qua và không coi lỗi
thiết lập/biên dịch là phát hiện mutation (kiểm ngược bằng lỗi cố ý).
"""
import argparse
from contextlib import contextmanager
import csv
import io
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time

# Không phụ thuộc locale của container; Python 3.6 chưa có reconfigure().
for _name in ('stdout', 'stderr'):
    _stream = getattr(sys, _name)
    if hasattr(_stream, 'buffer'):
        setattr(sys, _name, io.TextIOWrapper(_stream.detach(), encoding='utf-8',
                                           errors='replace', line_buffering=True))

import test_drive_collect as d


CASES = ['P{:02d}'.format(i) for i in range(1, 24) if i != 20]
HIGHGUI = r'cv::(?:namedWindow|imshow|waitKey(?:Ex)?|getWindowProperty)\('


def child_ids(pid):
    """Linux: đọc tất cả luồng, không bỏ sót fork từ luồng phụ."""
    children = set()
    for path in Path('/proc/{}/task'.format(pid)).glob('*/children'):
        try:
            children.update(int(value) for value in path.read_text(encoding='utf-8').split())
        except FileNotFoundError:
            pass
    return children


def identity(pid):
    try:
        fields = Path('/proc/{}/stat'.format(pid)).read_text(encoding='utf-8').rsplit(')', 1)[1].split()
        return fields[19]  # starttime, trường 22; không nhầm PID được dùng lại.
    except FileNotFoundError:
        return None


@contextmanager
def preview(cfg, root, mode='ok', args=(), env=None, stderr=None):
    environment = {} if mode is None else dict(UCR_DRIVE_FAKE_PREVIEW=mode)
    environment.update(env or {})
    with d.running(cfg, root, args=['--interval-ms', '100'] + list(args),
                   env=environment, stderr=stderr, rec_on_start=True) as run:
        seen = {}

        def sample():
            for pid in child_ids(run.process.pid):
                token = identity(pid)
                if token is not None:
                    seen[pid] = token
            return set(seen)

        run.sample_children = sample
        run.preview_children = seen
        try:
            yield run
        finally:
            # Kể cả con bị hạ ưu tiên/đổi nhóm, chỉ dọn PID có cùng starttime.
            sample()
            for pid, token in seen.items():
                if identity(pid) == token:
                    try:
                        os.kill(pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass


def live_child(run):
    run.wait(lambda: run.sample_children(), 5, 'Không thấy con preview', alive=True)
    pids = child_ids(run.process.pid)
    d.need(len(pids) == 1, 'Cần đúng một con preview: ' + repr(pids), 'setup')
    return next(iter(pids))


def no_orphans(run):
    run.wait(lambda: all(identity(pid) != token for pid, token in run.preview_children.items()),
             2, 'Con preview chưa được thu hồi sau khi cha thoát', alive=False)


def journal(run):
    paths = list(run.out.glob('*/stops.csv'))
    if not paths:
        return []
    lines = paths[0].read_text(encoding='utf-8').splitlines(True)
    return list(csv.DictReader([line for line in lines if line.endswith('\n')]))


def stop_record(run, reason, tag='preview_stop'):
    row = run.wait(lambda: next((r for r in journal(run) if r['reason'] == reason), None),
                   5, 'Thiếu nhật ký ' + reason)
    trigger, done, latency = (float(row[key]) for key in
                              ('t_trigger_ms', 't_motor0_done_ms', 'latency_ms'))
    d.need(all(math.isfinite(v) for v in (trigger, done, latency)) and
           0 <= latency <= 200 + run.cfg.slack and abs(done - trigger - latency) <= 5,
           'Nhật ký dừng sai/hơn hạn: ' + repr(row), tag)
    events = run.events()
    zeros = [i for i, e in d.matched(events, 'motor_speed', 0) if abs(e[0] - done) <= 5]
    d.need(zeros, 'Nhật ký không khớp motor 0 trong vết', tag)
    index = zeros[0]
    run.wait(lambda: d.matched(run.events()[index:], 'servo_angle', 0),
             message='Thiếu servo 0 sau motor 0')
    events = run.events()
    before = [i for i, e in enumerate(events) if e[0] >= trigger - 1]
    d.ordered_stop(events[before[0]:], tag)
    return row, index


def no_zero(events, tag='preview_blocks'):
    d.need(not d.matched(events, 'motor_speed', 0) and
           not d.matched(events, 'motor_emergency'),
           'Preview làm dừng oan khi W còn được cấp', tag)


def check_deadman(events, start, hold, slack):
    zeros = [e for e in events if e[0] >= start and
             (e[1] == 'motor_emergency' or (e[1] == 'motor_speed' and e[2] == 0))]
    d.need(zeros and start + hold - 5 <= zeros[0][0] <= start + hold + 200 + slack,
           'Deadman không theo W cuối của hai nguồn', 'shared_deadman')


def blocked_display(case, cfg, root):
    # §8: SIGSTOP mô phỏng hiển thị chậm vô hạn, không giả định hook slow tồn tại.
    with preview(cfg, root, 'hang' if case == 'P01' else 'ok') as run:
        pid = live_child(run)
        run.boot()
        run.ready()
        run.keep_driving()
        if case == 'P02':
            os.kill(pid, signal.SIGSTOP)
        mark = len(run.events())
        rows = len(run.rows())
        run.observe(2, lambda: no_zero(run.events()[mark:]))
        run.wait(lambda: len(run.rows()) >= rows + 2, 3, 'Preview chặn ghi ảnh')
        run.pump = False
        mark = len(run.events())
        key = run.key('S')
        d.stop_by(run, mark, key[0])
        run.drive()
        mark = len(run.events())
        key = run.key('W')
        d.stop_by(run, mark, key[0], 800, 'shared_deadman')
        check_deadman(run.events()[mark:], key[0], 800, cfg.slack)
        run.keep_driving()
        run.pump = False
        mark = len(run.events())
        key = run.key('X')
        d.stop_by(run, mark, key[0])
        run.event('servo_angle', 0, mark)
        d.ordered_stop(run.events()[mark:])
        run.ready()
        run.drive()
        run.finish()
        no_orphans(run)


def child_loss(case, cfg, root):
    mode = dict(P03='die_after_frame', P04='close_after_frame', P05='ok')[case]
    reason = 'preview_closed' if case == 'P04' else 'preview_died'
    with preview(cfg, root, mode) as run:
        # Hai hook tự thoát có thể chết trước lần đọc /proc đầu tiên.
        if case == 'P05':
            pid = live_child(run)
            d.moving(run)
            run.wait(lambda: len(run.rows()) >= 3, 5, 'Thiếu ảnh trước khi giết con đang hiển thị')
            mark, stamp = len(run.events()), d.monotonic_ms()
            os.kill(pid, signal.SIGKILL)
            d.stop_by(run, mark, stamp, tag='preview_stop')
            run.pump = False
        else:
            run.boot(settle=False)  # sự cố nằm ngay sau Init: giữ nguyên tiền đề "W cũ trong vòng H kể từ sự cố"
        row, zero = stop_record(run, reason)
        run.wait(lambda: d.matched(run.events()[zero:], 'state', 'DUNG') or
                 d.matched(run.events(), 'state')[-1][1][2] == 'DUNG',
                 message='Con mất nhưng không về DUNG')
        if case == 'P05':
            d.need(float(row['t_motor0_done_ms']) <= stamp + 200 + cfg.slack,
                   'Dừng trễ từ lúc SIGKILL con', 'preview_stop')
        d.held_then_fresh(run, len(run.events()))
        run.pump = False
        mark = len(run.events())
        run.key('S')
        run.event('motor_speed', 0, mark)
        rows = len(run.rows())
        run.wait(lambda: len(run.rows()) > rows, 3, 'Không ghi ảnh sau khi mất preview')
        run.observe(0.3, lambda: d.need(not child_ids(run.process.pid),
                                      'Đã mở lại con/cửa sổ sau khi mất preview'))
        d.need(sum(r['reason'] == reason for r in journal(run)) == 1,
               'Lặp nhật ký mất preview')
        run.ready()
        run.drive()
        run.finish()
        if case == 'P04':
            text = run.text()
            d.need('preview_monitor=AUTOSIZE' in text and
                   'preview: cua so dong (prop=AUTOSIZE gia tri=-1)' in text,
                   'Cha không ghi chẩn đoán đóng từ vùng chia sẻ')
        no_orphans(run)


def key_effects(events):
    expected = [('W', ('motor_speed', 5)),
                ('A', ('servo_angle', 5)), ('A', ('servo_angle', 10)),
                ('D', ('servo_angle', 5)), ('C', ('servo_angle', 0)),
                ('A', ('servo_angle', 5)), ('S', ('motor_speed', 0)),
                ('W', ('motor_speed', 5)), (' ', ('motor_speed', 0)),
                ('W', ('motor_speed', 5)), ('X', ('motor_speed', 0)),
                ('W', None),
                ('W', ('motor_speed', 5)), ('D', ('servo_angle', -5)),
                ('C', ('servo_angle', 0))]
    keys = [(i, e) for i, e in d.matched(events, 'key') if
            str(e[2]).upper() in ('R', 'Y', 'W', 'A', 'D', 'C', 'S', 'X', ' ', 'SPACE', 'ESC', 'E', '3')]
    d.need(len(keys) >= len(expected), 'Thiếu phím của chuỗi cửa sổ', 'window_keys')
    for n, (wanted, effect) in enumerate(expected):
        index, event = keys[n]
        actual = str(event[2]).upper()
        d.need(actual == wanted or (wanted == ' ' and actual == 'SPACE'),
               'Sai thứ tự phím: {} / {}'.format(actual, wanted), 'window_keys')
        end = keys[n + 1][0] if n + 1 < len(keys) else len(events)
        chunk = events[index:end]
        if effect:
            d.need(d.matched(chunk, *effect), 'Thiếu hiệu lực phím ' + wanted, 'window_keys')
        if wanted == 'S' or wanted == ' ':
            d.need(not d.matched(chunk, 'servo_angle', 0), 'S/Space làm mất góc', 'window_keys')
        if wanted == 'X':
            d.ordered_stop(chunk, 'window_keys')
            d.need(d.matched(chunk, 'state', 'DUNG'), 'X không về DUNG', 'window_keys')
        if n == 11:
            d.limits(chunk, moving=False)


def window_keys(cfg, root):
    # '?' vô hại, kéo giãn 100 ms giữa phím; trước S chưa hết deadman 800 ms.
    sequence = 'WAADCASW WXWWDC'
    for source in ('window', 'terminal'):
        for end, code, reason in (('E', 0, 'esc'), ('3', 130, 'sigint')):
            stream = '?' * 200 + ''.join(key + '?' * (45 if i in (6, 8, 11) else 4)
                                                  for i, key in enumerate(sequence + end))
            with preview(cfg, root / (source + end), 'keys:' + stream if source == 'window' else 'ok') as run:
                pid = live_child(run)
                if source == 'window':
                    os.kill(pid, signal.SIGSTOP)
                run.boot()
                if source == 'window':
                    os.kill(pid, signal.SIGCONT)
                if source == 'terminal':
                    for i, key in enumerate(sequence):
                        run.key(key)
                        run.observe(0.90 if i in (6, 8, 11) else 0.10)
                    run.send('\x1b' if end == 'E' else '\x03')
                run.wait(lambda: run.process.poll() is not None, 12,
                         'Chuỗi phím không thoát', alive=False)
                d.need(run.process.returncode == code, 'E/3 sai mã thoát', 'window_keys')
                key_effects(run.events())
                d.final_stop(run.events())
                run.check_output(reason, code)
                no_orphans(run)


def shared_deadman(cfg, root):
    sequence = '?' * 200 + 'w' + '?' * 29 + 'w' + '?' * 29 + 'w'
    with preview(cfg, root, 'keys:' + sequence) as run:
        pid = live_child(run)
        os.kill(pid, signal.SIGSTOP)
        run.boot()
        os.kill(pid, signal.SIGCONT)
        run.event('motor_speed', 5, seconds=10)
        first = d.matched(run.events(), 'key', 'W') or d.matched(run.events(), 'key', 'w')
        d.need(first, 'Không có W từ cửa sổ', 'setup')
        mark = len(run.events())
        # Chèn W terminal giữa W cửa sổ thứ nhất và thứ hai.
        run.observe(0.3, lambda: no_zero(run.events()[mark:]))
        run.key('W')
        run.wait(lambda: len([e for e in run.events() if e[1] == 'key' and
                             str(e[2]).upper() == 'W']) >= 4, 3)
        keys = [e for e in run.events() if e[1] == 'key' and str(e[2]).upper() == 'W']
        last = keys[-1][0]
        no_zero([e for e in run.events()[mark:] if e[0] <= last])
        d.stop_by(run, len(run.events()), last, 800, 'shared_deadman')
        check_deadman(run.events(), last, 800, cfg.slack)
        # Terminal vẫn gia hạn cùng đồng hồ sau khi cửa sổ đã hết W.
        run.drive()
        mark = len(run.events())
        key = run.key('W')
        d.stop_by(run, mark, key[0], 800, 'shared_deadman')
        check_deadman(run.events()[mark:], key[0], 800, cfg.slack)
        run.finish()
        no_orphans(run)


def queue_quiet(events):
    stops = [i for i, e in enumerate(events) if e[1] == 'motor_emergency' or
             (e[1] == 'motor_speed' and e[2] == 0)]
    d.need(stops, 'Không có motor 0 để kiểm xóa hàng đợi', 'queue_clear')
    d.need(not any(e[1] == 'motor_speed' and e[2] > 0 for e in events[stops[0] + 1:]),
           'W tồn đọng của hai nguồn làm chạy lại', 'queue_clear')
    # P08 đã ngừng con trước khi cha tiếp tục; sau dừng chỉ còn byte cũ trong hàng.
    d.need(not any(e[1] == 'key' and str(e[2]).upper() in ('W', 'A', 'D', 'C')
                   for e in events[stops[0] + 1:]),
           'Đuôi hàng phím cũ vẫn bị diễn giải sau lệnh dừng', 'queue_clear')


def queue_fresh(rows, stop_ms):
    d.need(not any(row['reason'] == 'frame_stale' and
                   float(row['t_trigger_ms']) <= stop_ms for row in rows),
           'Dựng P08 sai: frame_stale trước phím dừng đã xóa hàng đợi', 'setup')


def queue_stop_key(events, key):
    # Vết ghi byte đã giải mã: E thành ESC, 3 thành Ctrl+C (0x03).
    names = {'E': ('E', 'ESC'), '3': ('3', '\x03')}.get(key, (key,))
    return next((e for e in events if e[1] == 'key' and str(e[2]).upper() in names), None)


def queue_pause(run, pid):
    os.kill(pid, signal.SIGSTOP)
    run.wait(lambda: Path('/proc/{}/stat'.format(pid)).read_text(encoding='utf-8').rsplit(')', 1)[1].split()[0] == 'T',
             0.1, 'Không xác nhận được tiến trình ngừng để dồn phím')


def terminal_pending(run):
    return d.struct.unpack('i', d.fcntl.ioctl(run.slave, d.termios.FIONREAD, d.struct.pack('i', 0)))[0]


def queues(cfg, root):
    """§4: dồn byte vào CẢ HAI nguồn khi cha tạm ngừng; không có W mới sau resume."""
    for source in ('terminal', 'window'):
        for key in ('S', 'X', 'E'):
            stream = '?' * 200 + 'w' + ('w' * 3 + key + 'w' * 30
                                                   if source == 'window' else 'w' * 200)
            with preview(cfg, root / (source + key), 'keys:' + stream) as run:
                pid = live_child(run)
                queue_pause(run, pid)
                run.boot()
                os.kill(pid, signal.SIGCONT)
                running = run.event('state', 'CHAY', seconds=10)
                d.need(d.matched(run.events(), 'motor_speed', 5),
                       'Dựng P08 sai: CHAY chưa có lệnh tiến', 'setup')
                run.pump = False
                mark = len(run.events())
                paused = d.monotonic_ms()
                try:
                    queue_pause(run, run.process.pid)
                    if source == 'terminal' and key == 'E':
                        # Esc đơn cần một lượt đọc riêng trước khi dồn W; ESC+W cùng lượt bị nuốt.
                        run.send('\x1b')
                        run.wait(lambda: terminal_pending(run) > 0, 0.1, 'Esc chưa tới PTY')
                        os.kill(run.process.pid, signal.SIGCONT)
                        run.wait(lambda: terminal_pending(run) == 0, 0.04, 'Terminal chưa đọc Esc đơn')
                        queue_pause(run, run.process.pid)
                    run.send((key if source == 'terminal' and key != 'E' else '') + 'W' * 512)
                    run.wait(lambda: terminal_pending(run) > 0, 0.1, 'Chưa dồn được W vào hàng terminal')
                    # Nhịp con 20 ms: đủ 3 W + phím dừng + W sau đó, dưới hạn khung 500 ms.
                    run.observe(0.15)
                    queue_pause(run, pid)
                finally:
                    os.kill(run.process.pid, signal.SIGCONT)
                d.need(d.monotonic_ms() - paused < 500,
                       'Dựng P08 sai: khoảng ngừng cha đạt 500 ms', 'setup')
                try:
                    stop_key = run.wait(lambda: queue_stop_key(run.events()[mark:], key),
                                        3, 'Không nhận phím dừng với hàng đợi kép', alive=key != 'E')
                except d.SafetyFailure as error:
                    raise d.SafetyFailure('setup', '{} ({}/{}); vết cuối: {}'.format(
                        error, source, key, run.events()[-20:]))
                d.need(stop_key[0] >= running[0], 'Phím dừng đến trước CHAY', 'setup')
                print('P08 {}/{}: vết key={!r}, sau CHAY {:.1f} ms'.format(
                    source, key, stop_key[2], stop_key[0] - running[0]))
                deadline = stop_key[0] + 200 + cfg.slack
                run.wait(lambda: d.matched(run.events()[mark:], 'motor_speed', 0),
                         max(0.01, (deadline - d.monotonic_ms()) / 1000),
                         'Phím dừng không ghi motor 0 đúng hạn', alive=False)
                zero = d.matched(run.events()[mark:], 'motor_speed', 0)[0][1]
                d.need(zero[0] <= deadline, 'Phím dừng trễ', 'queue_clear')
                rows = run.wait(lambda: journal(run), message='Thiếu nhật ký để loại trừ frame_stale',
                                alive=False)
                queue_fresh(rows, stop_key[0])
                if key == 'E':
                    run.wait(lambda: run.process.poll() is not None, 6, alive=False)
                    d.need(run.process.returncode == 0, 'Esc sai mã')
                    run.check_output('esc', 0)
                else:
                    run.observe(0.35, lambda: queue_quiet(run.events()[mark:]))
                    if key == 'X':
                        d.need(d.matched(run.events()[mark:], 'state', 'DUNG'), 'X không về DUNG')
                    run.finish()
                queue_fresh(journal(run), stop_key[0])
                queue_quiet(run.events()[mark:])
                d.final_stop(run.events())
                no_orphans(run)


def diagnostic(text):
    d.need('--preserve-env=DISPLAY,XAUTHORITY' in text and '--no-preview' in text,
           'Cảnh báo thiếu hai cách chạy đúng', 'startup_warning')
    d.need(re.search(r'(?i)fail|unavailable|display|wayland|kh[oô]ng|l[oỗ]i', text),
           'Không giải thích nguyên nhân preview hỏng', 'startup_warning')


@contextmanager
def missing_display():
    saved = {key: os.environ.pop(key, None) for key in ('DISPLAY', 'WAYLAND_DISPLAY')}
    try:
        yield
    finally:
        for key, value in saved.items():
            if value is not None:
                os.environ[key] = value


def unavailable(cfg, root):
    for mode in ('fail_start', None):
        for required in (False, True):
            with missing_display():
                with preview(cfg, root / ((mode or 'headless') + str(required)), mode,
                             args=['--require-preview'] if required else [],
                             env={'DISPLAY': ':ucr_fake_only'} if mode else None) as run:
                    if required:
                        run.wait(lambda: run.process.poll() is not None, 10, alive=False)
                        d.need(run.process.returncode == 7, 'Require preview cần mã 7')
                        d.final_stop(run.events())
                        run.check_output('preview_unavailable', 7)
                    else:
                        d.moving(run)
                        run.finish()
                    diagnostic(run.text())
                    no_orphans(run)


def disabled(cfg, root):
    with preview(cfg, root, 'ok', args=['--no-preview']) as run:
        def absent():
            d.need(not run.sample_children(), '--no-preview đã tạo con', 'no_fork')
        run.wait(lambda: bool(run.rows()) or absent(), 10)
        run.boot()
        absent()
        run.ready()
        run.keep_driving()
        run.observe(1, absent)
        run.finish()
        absent()


def arguments(cfg, root):
    bad = [('--no-preview', '--require-preview'), ('--require-preview', '--no-preview')]
    bad += [('--hold-timeout-ms', value) for value in ('699', '2001')]
    for flag, values in (('--preview-scale', ('0.199', '1.001', '0', '-1', 'nan', 'inf', 'abc')),
                         ('--preview-fps', ('0', '31', '-1', 'nan', 'inf', 'abc', '10.5', '1e99'))):
        bad += [(flag, value) for value in values] + [(flag,)]
    for i, args in enumerate(bad):
        d.rejected(cfg, root / str(i), args, env={'UCR_DRIVE_FAKE_PREVIEW': 'ok'})
    for i, args in enumerate((('--preview-scale', '0.2', '--preview-fps', '1', '--hold-timeout-ms', '700'),
                              ('--preview-scale', '1.0', '--preview-fps', '30', '--hold-timeout-ms', '2000'))):
        with preview(cfg, root / ('boundary' + str(i)), args=args) as run:
            live_child(run)
            run.boot()
            run.finish()
            no_orphans(run)


def signals(cfg, root):
    for sig, reason in ((signal.SIGTERM, 'sigterm'), (signal.SIGINT, 'sigint')):
        with preview(cfg, root / reason) as run:
            live_child(run)
            d.moving(run)
            run.pump = False
            mark, stamp = len(run.events()), d.monotonic_ms()
            run.process.send_signal(sig)
            d.stop_by(run, mark, stamp)
            run.finish(reason)
            no_orphans(run)


def stderr_hang(cfg, root):
    environment = {}
    if cfg.tsan_slack:
        # Như D86/D88: stderr vẫn đầy, nhưng bộ dò lỗi không bị kẹt khi ghi chẩn đoán.
        environment['TSAN_OPTIONS'] = ':'.join(filter(None, (
            os.environ.get('TSAN_OPTIONS', ''), 'log_path="{}"'.format(root / 'tsan'))))
    with d.stderr_pipe() as (_, writer):
        with preview(cfg, root, 'hang', env=environment, stderr=writer) as run:
            live_child(run)
            d.moving(run)
            d.fill_pipe(writer)
            run.pump = False
            mark, stamp = len(run.events()), d.monotonic_ms()
            run.process.send_signal(signal.SIGTERM)
            d.stop_by(run, mark, stamp)
            zero = run.event('servo_angle', 0, mark)
            deadline = zero[0] + 3300 + cfg.slack + cfg.tsan_slack
            run.wait(lambda: run.process.poll() is not None,
                     max(0.01, (deadline - d.monotonic_ms()) / 1000),
                     'Con hang + stderr đầy phá hạn cleanup 3 s', alive=False)
            d.check_tsan_logs(root)
            d.need(d.monotonic_ms() <= deadline and run.process.returncode == 143,
                   'Cleanup trễ/sai mã khi stderr nghẽn', 'cleanup_deadline')
            d.final_stop(run.events())
            d.no_restart(run.events()[mark:])
            no_orphans(run)


def object_structure(build_root, nm):
    files = list(Path(build_root).rglob('ucr_drive_collect.dir/main.cpp.o'))
    d.need(len(files) == 1, 'Cần đúng một main.cpp.o của bản xe (không LTO/strip)', 'artifact_setup')
    folder = files[0].parent
    objects = list(folder.rglob('*.o'))
    d.need(any(p.name == 'preview.cpp.o' for p in objects), 'Thiếu preview.cpp.o', 'artifact_setup')
    for obj in objects:
        symbols = d.tool_output([nm, '-C', str(obj)])
        d.need(obj.name == 'preview.cpp.o' or not re.search(HIGHGUI, symbols),
               'HighGUI nằm ngoài đơn vị preview: ' + str(obj), 'highgui_location')


def artifacts(cfg):
    for binary in (cfg.real, cfg.cam, cfg.hooks):
        d.need(binary and Path(binary).is_file() and Path(binary).read_bytes()[:4] == b'\x7fELF',
               'Thiếu hiện vật ELF: ' + str(binary), 'artifact_setup')
    d.need(b'UCR_DRIVE_FAKE_PREVIEW' not in Path(cfg.real).read_bytes(), 'Hook preview lọt vào bản xe')
    d.need(b'UCR_DRIVE_FAKE_PREVIEW' in Path(cfg.hooks).read_bytes(), 'Thiếu hook đối chứng')
    cam = d.tool_output([cfg.nm, '-C', cfg.cam])
    real = d.tool_output([cfg.nm, '-C', cfg.real])
    d.need(not re.search(r'\b(?:Motor|Servo)::', cam), 'Cam chứa Motor/Servo')
    d.need(not re.search(r'\b(?:ToF|TOF|IMU|BNO055|SH1106|OLED|Backend|App)::', real),
           'Drive chứa lớp cảm biến/app bị cấm')
    object_structure(cfg.build_root, cfg.nm)


def images(cfg, root):
    with preview(cfg, root, args=['--preview-scale', '0.2', '--preview-fps', '30']) as run:
        live_child(run)
        run.boot()
        run.wait(lambda: len(run.rows()) >= 4, 8)
        run.finish()
        rows = run.rows()
        folder = run.out
        for row in rows:
            d.need((int(row['width']), int(row['height'])) == (1280, 720), 'CSV bị thu nhỏ theo preview')
            size, tables = d.read_jpeg(folder / row['filename'])
            d.need(size == (1280, 720), 'Ảnh lưu bị thu nhỏ theo preview')
            d.quality90(tables)
        gaps = [(int(b['monotonic_ns']) - int(a['monotonic_ns'])) / 1e6 for a, b in zip(rows, rows[1:])]
        print('Khoảng chụp ảnh lớn nhất {:.1f} ms; không phải tuổi khung camera'.format(max(gaps)))
        no_orphans(run)


def no_preview_loss(rows):
    d.need(not any(row['reason'] in ('preview_closed', 'preview_died') for row in rows),
           'Preview bị coi là đóng/chết khi không đóng cửa sổ', 'false_preview_loss')


def same_preview(run, pid, token, state):
    d.need(identity(pid) == token and child_ids(run.process.pid) == {pid},
           'Con preview mất/đổi PID hoặc thời điểm khởi tạo', 'preview_lifetime')
    fields = Path('/proc/{}/stat'.format(pid)).read_text(encoding='utf-8').rsplit(')', 1)[1].split()
    d.need(fields[0] != 'Z', 'Con preview đã chết, chưa được thu hồi', 'preview_lifetime')
    no_preview_loss(journal(run))
    states = d.matched(run.events(), 'state')
    d.need(states and states[-1][1][2] == state,
           'Preview làm đổi trạng thái ' + state, 'preview_lifetime')


def recording(rows, folder):
    """P19/P23: ảnh phẳng khớp frames.csv; xe đứng cách ít nhất 2 s."""
    names = [row['filename'] for row in rows]
    d.need(rows and len(set(names)) == len(names) and
           set(names) == {path.name for path in folder.glob('*.jpg')},
           'JPEG không khớp từng dòng frames.csv', 'preview_recording')
    previous = None
    for row in rows:
        d.need((int(row['width']), int(row['height'])) == (1280, 720) and
               row['state'] == 'DUNG' and float(row['cmd_speed']) == 0 and
               float(row['cmd_steer']) == 0,
               'Metadata sai cỡ/trạng thái/lệnh khi đang DUNG', 'preview_recording')
        size, _ = d.jpeg_info((folder / row['filename']).read_bytes())
        d.need(size == (1280, 720), 'JPEG không còn 1280×720', 'preview_recording')
        stamp = int(row['monotonic_ns'])
        d.need(previous is None or stamp - previous >= 2000000000,
               'Xe đứng ghi ảnh nhanh hơn 1 ảnh/2 s', 'preview_recording')
        previous = stamp
    return rows


def long_idle(cfg, root):
    """T21b_test3/P19: quan sát đủ 60 s, không gửi bất kỳ phím nào trước Esc."""
    with preview(cfg, root) as run:
        pid = live_child(run)
        token = identity(pid)
        run.boot()
        counts = (len(list(run.out.glob('*.jpg'))), len(run.rows()))
        started = time.monotonic()

        def unchanged():
            same_preview(run, pid, token, 'DUNG')
            events = run.events()
            d.need(not d.matched(events, 'key'), 'Có phím trong P19', 'setup')
            d.need(all(e[2] == 'DUNG' for _, e in d.matched(events, 'state')),
                   'P19 từng rời DUNG', 'preview_lifetime')
            d.limits(events, moving=False)

        for _ in range(20):
            run.observe(3, unchanged)
            current = (len(list(run.out.glob('*.jpg'))), len(run.rows()))
            d.need(all(new > old for old, new in zip(counts, current)),
                   'JPEG/frames không tăng trong ba giây khi xe đứng', 'preview_recording')
            counts = current
        d.need(time.monotonic() - started >= 60, 'P19 chưa đủ 60 s', 'setup')
        run.finish()
        recording(run.rows(), run.out)
        no_preview_loss(journal(run))
        no_orphans(run)


def no_close_driving(cfg, root):
    """T21b_test3/P21: đối chứng CHAY không đóng; P04 giữ ca close_after_frame."""
    with preview(cfg, root) as run:
        pid = live_child(run)
        token = identity(pid)
        d.moving(run)
        mark = len(run.events())

        def unchanged():
            same_preview(run, pid, token, 'CHAY')
            no_zero(run.events()[mark:])

        run.observe(2, unchanged)
        run.finish()
        no_preview_loss(journal(run))
        no_orphans(run)


def exit_paths(cfg, root):
    """T21b_test3/P22: cả bốn đường thoát khi motor đang chạy, không nới hạn dừng."""
    for source in ('window_3', 'sigint', 'terminal_esc', 'window_E'):
        window = source.startswith('window_')
        end = source[-1]
        reason, code = ('sigint', 130) if source in ('window_3', 'sigint') else ('esc', 0)
        mode = 'keys:' + '?' * 200 + end if window else 'ok'
        with preview(cfg, root / source, mode) as run:
            pid = live_child(run)
            if window:
                queue_pause(run, pid)
            d.moving(run)
            mark = len(run.events())
            if window:
                os.kill(pid, signal.SIGCONT)
                key = run.wait(lambda: queue_stop_key(run.events()[mark:], end),
                               12, 'Không nhận phím thoát từ cửa sổ giả', alive=False)
                stamp = key[0]
                run.pump = False
            else:
                run.pump = False
                stamp = d.monotonic_ms()
                if source == 'sigint':
                    run.process.send_signal(signal.SIGINT)
                else:
                    run.send('\x1b')
            zero = d.stop_by(run, mark, stamp)
            run.wait(lambda: run.process.poll() is not None, 8, 'Không thoát ' + source, alive=False)
            d.need(run.process.returncode == code, 'Sai mã thoát ' + source, 'preview_exit')
            d.ordered_stop(run.events()[mark:])
            d.final_stop(run.events())
            d.no_restart(run.events()[mark:])
            run.check_output(reason, code)
            no_preview_loss(journal(run))
            no_orphans(run)
            print('P22 {}: mã {}, dừng {:.1f} ms (trần {} ms)'.format(
                source, code, zero[0] - stamp, 200 + cfg.slack))


def compare_counts(counts):
    d.need(len(counts) == 2 and min(counts) >= 2 and abs(counts[0] - counts[1]) <= 3,
           'Số JPEG có/không preview lệch hơn ±3 (nhịp lưu dao động dưới tải): ' + repr(counts), 'preview_recording')


def compare_images(cfg, root):
    """T21b_test3/P23: hai cửa sổ thời gian 4 s từ ảnh đầu, loại ảnh lúc dọn dẹp."""
    counts = []
    for enabled in (False, True):
        with preview(cfg, root / str(enabled), 'ok' if enabled else None,
                     args=[] if enabled else ['--no-preview']) as run:
            if enabled:
                pid = live_child(run)
                token = identity(pid)
            run.boot()
            start = int(run.rows()[0]['monotonic_ns'])
            end = start + 4000000000

            def unchanged():
                if enabled:
                    same_preview(run, pid, token, 'DUNG')
                else:
                    d.need(not run.sample_children(), '--no-preview vẫn tạo con')
                no_preview_loss(journal(run))
                return d.monotonic_ms() * 1000000 >= end

            run.wait(unchanged, 5, 'Không hoàn thành cửa sổ thu 4 s')
            run.finish()
            rows = recording(run.rows(), run.out)
            counts.append(sum(start <= int(row['monotonic_ns']) < end for row in rows))
            print('P23 preview={} moc_anh_ms={}'.format(enabled, [
                round((int(row['monotonic_ns']) - start) / 1000000, 3)
                for row in rows if start <= int(row['monotonic_ns']) < end]), flush=True)
            no_preview_loss(journal(run))
            no_orphans(run)
    compare_counts(counts)
    print('P23 JPEG trong 4 s, xe đứng tối đa 1 ảnh/2 s: không preview {}, có preview {}'.format(*counts))


def patch_mutant(case, text):
    """Chỉ ba điểm neo main.cpp sau báo cáo DEV hoàn tất tích hợp 29/10."""
    if case == 'P16':
        anchor = ('control.stop(childStatus == PreviewChildStatus::Closed ? "preview_closed" : "preview_died",\n'
                  '                                 trigger, true);')
        replacement = '(void)trigger;'
    elif case == 'P17':
        anchor = 'while (!control.exiting) {'
        replacement = anchor + '\n            cv::waitKey(1);'
        include = '#include <opencv2/core.hpp>'
        d.need(text.count(include) == 1, 'Điểm khai báo HighGUI không duy nhất', 'mutation_setup')
        text = text.replace(include, include + '\nnamespace cv { int waitKey(int); }', 1)
    else:
        # Bỏ cả xả hai nguồn và chặn đuôi lô để W cũ tới phép kiểm queue_clear.
        anchor = ('if (control.discardPending || control.exiting) {\n'
                  '                    drainPendingKeys(); control.discardPending = false;\n'
                  '                    discardMotion = true;\n'
                  '                    stoppedInBatch = true;')
        replacement = ('if (control.discardPending || control.exiting) {\n'
                       '                    control.discardPending = false;\n'
                       '                    discardMotion = false;')
    d.need(text.count(anchor) == 1, 'Điểm vá không khớp duy nhất: ' + case, 'mutation_setup')
    return text.replace(anchor, replacement, 1)


def build_mutant(case, cfg, root):
    source, build = root / 'source', root / 'build'
    source.mkdir(parents=True)
    build.mkdir()
    # Sao chép nguyên byte tệp phụ trợ; không phân tích header/preview của DEV.
    for name in ('main.cpp', 'options.h', 'preview.h', 'preview.cpp', 'pass_store.h', 'pass_store.cpp'):
        shutil.copyfile(str(Path(cfg.source) / name), str(source / name))
    main = source / 'main.cpp'
    main.write_text(patch_mutant(case, main.read_text(encoding='utf-8')), encoding='utf-8')
    target = ('add_library(ucr_drive_collect OBJECT main.cpp preview.cpp pass_store.cpp)\n' if case == 'P17' else
              'add_executable(ucr_drive_collect main.cpp preview.cpp pass_store.cpp)\n')
    # P17 chỉ cần biên dịch object có lời gọi HighGUI sai chỗ, không cần X/highgui
    # của container; ký hiệu undefined cv::waitKey đã đủ để nm phát hiện đúng lỗi.
    (source / 'CMakeLists.txt').write_text(
        'cmake_minimum_required(VERSION 3.10)\nproject(PreviewMutation LANGUAGES CXX)\n'
        'set(CMAKE_CXX_STANDARD 17)\nset(CMAKE_CXX_STANDARD_REQUIRED ON)\n'
        'find_package(OpenCV 4.1 REQUIRED COMPONENTS core imgcodecs videoio imgproc)\n'
        'find_package(Threads REQUIRED)\n' + target +
        'target_compile_definitions(ucr_drive_collect PRIVATE UCR_DRIVE_TEST_HOOKS=1)\n'
        'target_include_directories(ucr_drive_collect PRIVATE ${OpenCV_INCLUDE_DIRS})\n' +
        ('' if case == 'P17' else
         'target_link_libraries(ucr_drive_collect PRIVATE ${OpenCV_LIBS} Threads::Threads)\n'
         'if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9)\n'
         ' target_link_libraries(ucr_drive_collect PRIVATE stdc++fs)\nendif()\n'), encoding='utf-8')
    started = time.monotonic()
    for argv in ([cfg.cmake, '-DCMAKE_BUILD_TYPE=Release', '-DCMAKE_CXX_FLAGS=-O0 -g0',
                  '-DCMAKE_CXX_FLAGS_RELEASE=-DNDEBUG', '-DCMAKE_CXX_COMPILER=' + cfg.cxx, str(source)],
                 [cfg.cmake, '--build', '.', '--', '-j1']):
        remaining = 65 - (time.monotonic() - started)
        d.need(remaining > 0, 'Hết hạn build bản lỗi', 'mutation_setup')
        process = subprocess.Popen(argv, cwd=str(build), stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        try:
            try:
                output, _ = process.communicate(timeout=remaining)
            except subprocess.TimeoutExpired:
                raise d.SafetyFailure('mutation_setup', 'Build bản lỗi vượt tổng hạn 65 s')
        finally:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=5)
        d.need(process.returncode == 0, 'Build bản lỗi hỏng: ' + output.decode(errors='replace')[-3000:],
               'mutation_setup')
    return build


def mutation(case, cfg, root):
    """Bản gốc phải đạt trước khi vá/build; không nhận lỗi thiết lập làm phát hiện."""
    probe = dict(P16='P05', P17='P14', P18='P08')[case]
    expected = dict(P16='preview_stop', P17='highgui_location', P18='queue_clear')[case]
    test_case(probe, cfg, root / 'baseline')
    mutant = argparse.Namespace(**vars(cfg))
    build = build_mutant(case, cfg, root / 'mutation')
    if case == 'P17':
        mutant.build_root = str(build)
    else:
        mutant.hooks = str(build / 'ucr_drive_collect')
        data = Path(mutant.hooks).read_bytes()
        d.need(data[:4] == b'\x7fELF' and b'UCR_DRIVE_FAKE_PREVIEW' in data and
               data != Path(cfg.hooks).read_bytes(), 'Bản lỗi chưa phải hooks riêng', 'mutation_setup')
        mutant.slack = 0
        mutant.tsan_slack = 0
    try:
        test_case(probe, mutant, root / 'mutant')
    except d.SafetyFailure as error:
        d.need(error.tag == expected, 'Bản lỗi hỏng sai phép kiểm: ' + str(error), 'mutation_setup')
        print(case + ' phát hiện đúng ' + expected)
    else:
        raise d.SafetyFailure('mutation_survived', 'Bản lỗi không bị phát hiện')


def test_case(case, cfg, root):
    if case in ('P01', 'P02'):
        return blocked_display(case, cfg, root)
    if case in ('P03', 'P04', 'P05'):
        return child_loss(case, cfg, root)
    functions = dict(P06=window_keys, P07=shared_deadman, P08=queues, P09=unavailable,
                     P10=disabled, P11=arguments, P12=signals, P13=stderr_hang, P15=images,
                     P19=long_idle, P21=no_close_driving, P22=exit_paths, P23=compare_images)
    if case == 'P14':
        return artifacts(cfg)
    if case in ('P16', 'P17', 'P18'):
        return mutation(case, cfg, root)
    return functions[case](cfg, root)


def self_check():
    """Tự kiểm các cổng có thể chạy trên Windows; không thay thế ca Linux."""
    check_deadman([(1800, 'motor_speed', 0, False)], 1000, 800, 0)
    diagnostic('preview fail: sudo --preserve-env=DISPLAY,XAUTHORITY; --no-preview')
    failures = 0
    for events, slack in (([(1790, 'motor_speed', 0, False)], 0),
                          ([(2001, 'motor_speed', 0, False)], 0), ([], 300)):
        try:
            check_deadman(events, 1000, 800, slack)
        except d.SafetyFailure as error:
            assert error.tag == 'shared_deadman'
            failures += 1
    assert failures == 3
    for text in ('U cv::imshow(std::string const&, cv::_InputArray const&)',
                 'U cv::waitKey(int)', 'U cv::getWindowProperty(std::string, int)'):
        assert re.search(HIGHGUI, text)
    assert not re.search(HIGHGUI, 'T drive::publishFrame(cv::Mat const&)')
    with tempfile.TemporaryDirectory(prefix='ucr_preview_structure_') as directory:
        objects = Path(directory) / 'CMakeFiles' / 'ucr_drive_collect.dir'
        objects.mkdir(parents=True)
        for name in ('main.cpp.o', 'preview.cpp.o'):
            (objects / name).touch()
        original_tool = d.tool_output
        try:
            d.tool_output = lambda argv: ('U cv::waitKey(int)' if
                                          Path(argv[-1]).name == 'preview.cpp.o' else 'T run()')
            object_structure(directory, 'nm')
            d.tool_output = lambda argv: 'U cv::imshow(std::string, cv::Mat)'
            try:
                object_structure(directory, 'nm')
            except d.SafetyFailure as error:
                assert error.tag == 'highgui_location'
            else:
                raise AssertionError('Không bắt HighGUI ở main.cpp.o')
        finally:
            d.tool_output = original_tool
    queue_quiet([(1, 'motor_speed', 5, False), (2, 'motor_speed', 0, False)])
    for tail in ((2, 'motor_speed', 5, False), (2, 'key', 'W', False)):
        try:
            queue_quiet([(1, 'motor_speed', 0, False), tail])
        except d.SafetyFailure as error:
            assert error.tag == 'queue_clear'
        else:
            raise AssertionError('Không bắt W tồn đọng dù guard chặn motor')
    queue_fresh([], 100)
    queue_fresh([dict(reason='key_s', t_trigger_ms='90'),
                 dict(reason='frame_stale', t_trigger_ms='101')], 100)
    for stamp in ('99', '100'):
        try:
            queue_fresh([dict(reason='frame_stale', t_trigger_ms=stamp)], 100)
        except d.SafetyFailure as error:
            assert error.tag == 'setup'
        else:
            raise AssertionError('Không bắt frame_stale trước phím dừng')
    for key, names in (('S', ('s', 'S')), ('X', ('x', 'X')), ('E', ('E', 'ESC')),
                       ('3', ('3', '\x03'))):
        for name in names:
            event = (101, 'key', name, False)
            assert queue_stop_key([(100, 'key', '?', False), event], key) == event
        assert queue_stop_key([(100, 'key', 'W', False)], key) is None
    with tempfile.TemporaryDirectory(prefix='ucr_preview_ctrl_c_') as directory:
        trace = Path(directory) / 'trace'
        trace.write_bytes(b'101 key \x03\n')
        assert queue_stop_key(d.read_trace(trace), '3') == (101, 'key', '\x03', False)
    # Vết đủ phím/lệnh; thêm phím lạ để xác nhận không lệch bộ ghép phím.
    fixture = []
    operations = [('W', ('motor_speed', 5)),
                  ('A', ('servo_angle', 5)), ('A', ('servo_angle', 10)),
                  ('D', ('servo_angle', 5)), ('C', ('servo_angle', 0)),
                  ('A', ('servo_angle', 5)), ('S', ('motor_speed', 0)),
                  ('W', ('motor_speed', 5)), (' ', ('motor_speed', 0)),
                  ('W', ('motor_speed', 5)), ('X', ('motor_speed', 0)),
                  ('W', None),
                  ('W', ('motor_speed', 5)), ('D', ('servo_angle', -5)),
                  ('C', ('servo_angle', 0)), ('ESC', ('motor_speed', 0))]
    for n, (key, effect) in enumerate(operations):
        fixture += [(n * 100, 'key', '?', False), (n * 100 + 1, 'key', key, False)]
        if effect:
            fixture.append((n * 100 + 2, effect[0], effect[1], False))
        if key == 'X':
            fixture += [(n * 100 + 3, 'servo_angle', 0, False),
                        (n * 100 + 4, 'state', 'DUNG', False)]
    key_effects(fixture)
    try:
        key_effects([e for e in fixture if e[1:] != ('servo_angle', 10, False)])
    except d.SafetyFailure as error:
        assert error.tag == 'window_keys'
    else:
        raise AssertionError('Không bắt mất tác dụng phím A')
    try:
        no_zero([(1, 'motor_speed', 0, False)])
    except d.SafetyFailure as error:
        assert error.tag == 'preview_blocks'
    else:
        raise AssertionError('Không bắt dừng oan')
    no_preview_loss([dict(reason='esc')])
    for reason in ('preview_closed', 'preview_died'):
        try:
            no_preview_loss([dict(reason=reason)])
        except d.SafetyFailure as error:
            assert error.tag == 'false_preview_loss'
        else:
            raise AssertionError('Không bắt preview đóng/chết oan')
    compare_counts([2, 2])
    compare_counts([40, 40])
    compare_counts([40, 39])
    compare_counts([39, 40])
    compare_counts([40, 37])
    for counts in ([40, 36], [36, 40], [0, 0], [1, 1]):
        try:
            compare_counts(counts)
        except d.SafetyFailure as error:
            assert error.tag == 'preview_recording'
        else:
            raise AssertionError('Không bắt lệch/mất JPEG')
    with tempfile.TemporaryDirectory(prefix='ucr_preview_recording_') as directory:
        folder = Path(directory)
        # Chỉ marker cỡ ảnh cho bộ đọc JPEG chuẩn; không giả vờ giải mã điểm ảnh.
        header = b'\xff\xd8\xff\xc0\x00\x0b\x08'
        tail = b'\x01\x01\x11\x00\xff\xd9'
        jpeg = header + d.struct.pack('>HH', 720, 1280) + tail
        rows = [dict(filename='p0001_{:04d}.jpg'.format(n + 1), width='1280', height='720',
                     state='DUNG', cmd_speed='0', cmd_steer='0',
                     monotonic_ns=str(1000000000 + n * 2000000000)) for n in range(2)]
        for row in rows:
            (folder / row['filename']).write_bytes(jpeg)
        recording(rows, folder)
        bad_rows = []
        for key, value in (('width', '640'), ('state', 'CHAY'), ('cmd_speed', '5'),
                           ('cmd_steer', '5'), ('monotonic_ns', '2999999999'),
                           ('monotonic_ns', '1100000000'), ('filename', rows[0]['filename'])):
            altered = [dict(row) for row in rows]
            altered[1][key] = value
            bad_rows.append(altered)
        for altered in bad_rows:
            try:
                recording(altered, folder)
            except d.SafetyFailure as error:
                assert error.tag == 'preview_recording'
            else:
                raise AssertionError('Không bắt metadata sai')
        for data in (header + d.struct.pack('>HH', 360, 640) + tail, None):
            path = folder / rows[1]['filename']
            if data is None:
                path.unlink()
            else:
                path.write_bytes(data)
            try:
                recording(rows, folder)
            except d.SafetyFailure as error:
                assert error.tag == 'preview_recording'
            else:
                raise AssertionError('Không bắt JPEG sai cỡ/mất tệp')
    assert len(CASES) == len(set(CASES)) == 22 and 'P19' in CASES and 'P20' not in CASES
    cmake = Path(__file__).with_name('tests_preview.cmake').read_text(encoding='utf-8')
    assert re.search(r'set\(_preview_cases\s+([^)]*)\)', cmake).group(1).split() == CASES
    print('ĐẠT tự kiểm deadman/phím/hàng đợi/cảnh báo/HighGUI/dừng oan/đối chứng JPEG/22 mã; chưa chạy Linux')
    print('Chưa biên dịch/chạy bản lỗi; tự kiểm chỉ kiểm bộ đọc và dữ liệu giả')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=CASES)
    parser.add_argument('--self-check', action='store_true')
    for name in ('hooks', 'real', 'cam', 'build-root'):
        parser.add_argument('--' + name)
    parser.add_argument('--source', default=str(Path(__file__).resolve().parent))
    parser.add_argument('--cmake', default='cmake')
    parser.add_argument('--cxx', default='g++')
    parser.add_argument('--nm', default='nm')
    parser.add_argument('--slack', type=int, choices=(0, 300), default=0)
    parser.add_argument('--tsan-slack', type=int, choices=(0, 1000), default=0)
    cfg = parser.parse_args()
    if cfg.self_check:
        return self_check()
    d.need(os.name == 'posix' and Path('/proc/self').exists(), 'Ca PTY/PID cần Linux; không bỏ qua trên Windows')
    d.need(cfg.case and cfg.hooks, 'Thiếu --case/--hooks')
    for name in ('hooks', 'real', 'cam', 'build_root', 'source'):
        if getattr(cfg, name):
            setattr(cfg, name, str(Path(getattr(cfg, name)).resolve()))
    data = Path(cfg.hooks).read_bytes()
    d.need(data[:4] == b'\x7fELF' and b'UCR_DRIVE_FAKE_PREVIEW' in data and
           b'UCR_DRIVE_FAKE_TRACE' in data, 'Từ chối chạy binary không phải hooks preview')
    print('{}: dung sai dừng +{} ms; cleanup TSan +{} ms'.format(cfg.case, cfg.slack, cfg.tsan_slack))
    with tempfile.TemporaryDirectory(prefix='ucr_preview_' + cfg.case + '_') as directory:
        test_case(cfg.case, cfg, Path(directory))
    print(cfg.case + ' ĐẠT')


if __name__ == '__main__':
    main()
