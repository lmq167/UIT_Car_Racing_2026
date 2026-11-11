#!/usr/bin/env python3
"""TUI của ucr_auto (P5b): chạy ucr_auto_hooks với stdout/stderr là pty giả thì phải vẽ khung (ESC[H, dòng phím)
và thoát sạch (trả màu ESC[0m, motor 0 trước servo 0); chạy với ống (pipe) thì không có mã ANSI nào.
Dùng lại harness của test_auto_drive.py. Chạy: python3 test_tui_pty.py --hooks <ucr_auto_hooks>"""
import argparse
import importlib.util
import os
import pty
import subprocess
import sys
import tempfile
import threading
from pathlib import Path

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('test_auto_drive', str(HERE / 'test_auto_drive.py'))
tad = importlib.util.module_from_spec(spec)
sys.modules['test_auto_drive'] = tad
spec.loader.exec_module(tad)

_real_popen = subprocess.Popen
_use_pty = {'on': False}


def _popen(argv, **kw):
    """Khi bật: thay stdout/stderr bằng pty; luồng đọc chép đầu ra vào file log của harness (os.write, không seek)."""
    if not _use_pty['on']:
        return _real_popen(argv, **kw)
    log = kw.get('stdout')
    master, slave = pty.openpty()
    kw['stdout'] = slave
    kw['stderr'] = slave
    process = _real_popen(argv, **kw)
    os.close(slave)

    def pump():
        try:
            while True:
                try:
                    data = os.read(master, 65536)
                except OSError:
                    break
                if not data:
                    break
                os.write(log.fileno(), data)
        finally:
            os.close(master)
    threading.Thread(target=pump, daemon=True).start()
    return process


tad.subprocess.Popen = _popen


def check(cond, message):
    if not cond:
        raise SystemExit('KHONG DAT: ' + message)


def run_case(cfg, root, use_pty):
    _use_pty['on'] = use_pty
    with tad.running(cfg, root) as r:
        r.boot()
        r.go()
        r.frames(10)
        r.finish()
        return r.text()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hooks', required=True)
    parser.add_argument('--slack', type=int, default=300)
    args = parser.parse_args()
    cfg = argparse.Namespace(hooks=args.hooks, slack=args.slack, case='TUI')
    with tempfile.TemporaryDirectory(prefix='ucr-tui-') as tmp:
        text = run_case(cfg, Path(tmp) / 'pty', True)
        check('\x1b[H' in text, 'pty: không có khung (ESC[H)')
        check('G chay | Space dung | R nap yaml | Esc thoat' in text, 'pty: thiếu dòng phím')
        check('FSM ' in text and 'IMU' in text, 'pty: thiếu dòng trạng thái/IMU')
        check(text.rstrip('\r\n').endswith('\x1b[0m'), 'pty: không trả màu khi thoát')
        for seq in __import__('re').findall(r'\x1b\[[^A-Za-z]*[A-Za-z]', text):
            check(__import__('re').fullmatch(r'\x1b\[(H|K|J|[0-9;]*m)', seq) is not None, 'pty: mã lạ ' + repr(seq))
        plain = run_case(cfg, Path(tmp) / 'pipe', False)
        check('\x1b' not in plain, 'pipe: không được có mã ANSI')
        check('KHÔNG CÓ PHANH CHỐNG VẬT CẢN' in plain, 'pipe: phải giữ in dòng như cũ')
    print('test_tui_pty OK')


if __name__ == '__main__':
    main()
