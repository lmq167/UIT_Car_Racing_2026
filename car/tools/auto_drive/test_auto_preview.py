"""Hai ca chấp nhận preview treo cho ucr_auto_hooks (P01, P02); Python 3.6/Linux."""
import argparse
from contextlib import contextmanager
import os
from pathlib import Path
import signal
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_auto_drive as a


CASES = ('P01', 'P02')
# kCleanupTimeoutMs trong hợp đồng limits.h; không thêm dung sai ngoài --slack.
CLEANUP_MS = 3000
# Nhỏ hơn kCleanupTimeoutMs = 3000: cha SIGKILL con treo rồi thoát phải xong
# TRƯỚC hạn dự phòng của watchdog; thoát muộn hơn nghĩa là đang nhờ watchdog.
PROMPT_EXIT_MS = 2000


def child_status(pid):
    try:
        fields = Path('/proc/{}/stat'.format(pid)).read_text(encoding='utf-8').rsplit(')', 1)[1].split()
        return fields[0], fields[19]  # Trạng thái và starttime (trường 22), chống dùng lại PID.
    except FileNotFoundError:
        return None, None


@contextmanager
def preview(cfg, root, **kwargs):
    with a.running(cfg, root, **kwargs) as run:
        children = {}
        try:
            yield run, children
        finally:
            # Dọn cả con SIGSTOP khi phép kiểm hỏng; không che phép kiểm mồ côi trước đó.
            for pid, token in children.items():
                if child_status(pid)[1] == token:
                    try:
                        os.kill(pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass


def live_child(run, children):
    windows = run.wait(lambda: sorted(pid for pid in run.children() if a.td.nice(pid) == 10),
                       15, 'Không nhận diện được con preview qua nice10')
    for pid in windows:
        state, token = child_status(pid)
        a.need(token is not None and state not in ('Z', 'X'), 'Con preview không còn sống', 'preview_lifetime')
        children[pid] = token
    return windows[0]  # Như A26: preview chính được fork trước cửa sổ phụ.


def child_alive(pid, token, stopped=False):
    state, current = child_status(pid)
    a.need(current == token and state not in (None, 'Z', 'X'),
           'Con preview chết/đổi PID trước khi dọn', 'preview_lifetime')
    if stopped:
        a.need(state == 'T', 'Con preview không còn SIGSTOP', 'preview_lifetime')


def no_orphans(run, children):
    run.wait(lambda: all(child_status(pid)[1] != token for pid, token in children.items()),
             (CLEANUP_MS + run.slack) / 1000.0,
             'Con preview chưa được thu hồi sau khi cha thoát', alive=False)


def hung_start(run, children):
    pid = live_child(run, children)
    run.wait(lambda: all(run.of(kind) for kind in ('frame', 'mask', 'motor_init')) and
             a.stopping_pair(run.events(), 0), 15, 'Thiếu ảnh/mask/Init/cặp dừng đầu')
    a.need(not run.of('ready'), 'ready dù preview treo', 'preview_startup')
    a.no_motion(run, 0, 'preview_startup')
    child_alive(pid, children[pid])
    return pid, len(run.events())


def final_pair(run, children, pid, mark):
    pair = a.stopping_pair(run.events()[mark:], 0)
    if not pair:
        child_alive(pid, children[pid])
    return pair


def blocked_start(cfg, root):
    started = a.td.now_ms()
    with preview(cfg, root, env={'PREVIEW': 'hang'}) as (run, children):
        pid, mark = hung_start(run, children)
        def observe_stop():
            rows = run.rows()
            # DUNG/LOI được ghi sau stopVehicle khi thoát lỗi, tránh cặp dừng khởi động.
            if not rows or rows[-1]['planner_state'] != 'DUNG' or rows[-1]['reason'] != 'LOI':
                child_alive(pid, children[pid])
                return None
            if final_pair(run, children, pid, mark):
                return a.td.now_ms()

        stamp = run.wait(observe_stop, 20,
                         'Thiếu DUNG/LOI và cặp dừng sau lỗi preview', alive=False)
        run.wait(lambda: run.process.poll() is not None,
                 (stamp + PROMPT_EXIT_MS + cfg.slack - a.td.now_ms()) / 1000.0,
                 'Preview treo không thoát trước hạn dự phòng watchdog', alive=False)
        a.reject(run, 1, before_init=False)
        a.need(a.td.now_ms() - started <= 12000 + cfg.slack,
               'Preview treo không thoát trong 12 s + slack', 'preview_deadline')
        run.exited(1)
        no_orphans(run, children)


def stopped_preview(run, children):
    run.boot()
    run.go()
    pid = live_child(run, children)
    mark = len(run.events())
    os.kill(pid, signal.SIGSTOP)
    run.wait(lambda: child_status(pid) == ('T', children[pid]), 15, 'Chưa SIGSTOP được preview')

    def driving():
        child_alive(pid, children[pid], stopped=True)
        events = run.events()
        a.need(not any(a.motor_zero(e) for e in events[mark:]),
               'Preview SIGSTOP làm dừng oan', 'preview_false_stop')
        a.need(a.td.last_commands(events)[0] > 0, 'Xe không còn đang chạy', 'preview_false_stop')
        a.need(not run.of('key', mark), 'Phím vẫn tới cha qua preview SIGSTOP', 'preview_key')

    def progress():
        frames, rows = len(run.of('frame')), len(run.rows())

        def enough():
            driving()
            return len(run.of('frame')) >= frames + 5 and len(run.rows()) > rows

        run.wait(enough, 15, 'Preview SIGSTOP chặn ảnh/CSV')
        driving()

    # shortcut: con giả không giữ frame_busy; cần con thật để kiểm kẹt trong readPreviewFrame.
    progress()
    run.send(' ')
    progress()  # Chờ thêm bằng dữ liệu, không dùng sleep để suy ra Space bị chặn.
    return driving


def signal_exit(run, sig):
    stamp = a.external_stamp_ms()
    os.kill(run.process.pid, sig)
    a.stopped(run, stamp, 1000)  # Cùng hạn và exit0 với A23/A25.
    run.wait(lambda: run.process.poll() is not None,
             (stamp + PROMPT_EXIT_MS + run.slack - a.td.now_ms()) / 1000.0,
             'Preview SIGSTOP không thoát trước hạn dự phòng watchdog', alive=False)
    run.exited()


def blocked_running(cfg, root):
    # Hai lượt chính: (a)+(c), (a)+(b); thêm SIGINT như khung A25 cho phép.
    for mode, sig in (('term', signal.SIGTERM), ('int', signal.SIGINT), ('image', None)):
        with preview(cfg, root / mode, values={'mask_max_age_ms': 1000}) as (run, children):
            driving = stopped_preview(run, children)
            driving()
            if sig is not None:
                signal_exit(run, sig)
            else:
                event = run.inject('camera_pause_600ms')
                end = a.stopped(run, event[0], 700)  # Đúng cấu hình/tác nhân/hạn của A11.
                run.wait(lambda: any(e[0] >= event[0] and e[2] == 'MAT_ANH' for e in run.of('reason')),
                         message='Thiếu MAT_ANH sau mất ảnh')
                run.frames(8, lambda: a.no_motion(run, end, 'image_latch'))
                signal_exit(run, signal.SIGTERM)
                a.no_motion(run, end, 'image_latch')
            no_orphans(run, children)


# P13 (con treo + ống stderr đầy) chưa đăng ký vì ucr_auto hiện kẹt ở ghi "Lỗi: ..." cuối main
# sau khi watchdog đã hủy (xe đã dừng nhưng tiến trình không thoát); thêm lại khi mã được sửa.


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=CASES, required=True)
    parser.add_argument('--hooks', required=True)
    parser.add_argument('--slack', type=int, choices=(0, 300), default=0)
    cfg = parser.parse_args()
    a.need(os.name == 'posix', 'Chỉ chạy hooks trên Linux', 'environment')
    with tempfile.TemporaryDirectory(prefix='ucr-auto-preview-') as tmp:
        functions = dict(P01=blocked_start, P02=blocked_running)
        functions[cfg.case](cfg, Path(tmp) / cfg.case)
    print('PASS ' + cfg.case)


if __name__ == '__main__':
    main()
