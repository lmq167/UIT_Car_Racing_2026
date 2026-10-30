"""Kiểm tra của phần triển khai: trình bao dùng lệnh giả; Init dùng bản hooks."""
import argparse
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import unittest

# In được cả thông báo lỗi tiếng Việt khi Python 3.6 dùng locale ASCII.
for _name in ('stdout', 'stderr'):
    _stream = getattr(sys, _name)
    if hasattr(_stream, 'buffer'):
        setattr(sys, _name, io.TextIOWrapper(_stream.detach(), encoding='utf-8',
                                           errors='replace', line_buffering=True))


HERE = Path(__file__).resolve().parent
ARGS = None


def shell_path(path):
    path = Path(path).resolve()
    if os.name == 'nt':
        return '/' + path.drive[0].lower() + path.as_posix()[2:]
    return str(path)


class TextOutputTests(unittest.TestCase):
    def test_vietnamese_with_ascii_streams(self):
        # Ép ASCII thật ở cả hai đầu ra, không dựa vào locale trên máy chạy.
        message = 'ĐẠT: dung sai dừng; lỗi tiếng Việt'
        for name in ('test_drive_collect.py', 'test_preview.py', 'test_startup.py'):
            with self.subTest(script=name):
                command = (
                    "import io, runpy, sys; sys.path.insert(0, {folder}); "
                    "sys.stdout = io.TextIOWrapper(sys.stdout.detach(), encoding='ascii'); "
                    "sys.stderr = io.TextIOWrapper(sys.stderr.detach(), encoding='ascii'); "
                    "runpy.run_path({path}); "
                    "print({message}); print({message}, file=sys.stderr); "
                    "print('\\udcff'); print('\\udcff', file=sys.stderr)"
                ).format(folder=ascii(str(HERE)), path=ascii(str((HERE.parent / 'collect_common' if name == 'test_preview.py' else HERE) / name)),
                         message=ascii(message))
                env = os.environ.copy()
                env.update(LC_ALL='C', LANG='C', PYTHONCOERCECLOCALE='0', PYTHONUTF8='0',
                           PYTHONIOENCODING='ascii:strict', PYTHONDONTWRITEBYTECODE='1')
                result = subprocess.run([sys.executable, '-c', command], env=env,
                                        stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10)
                self.assertEqual(result.returncode, 0, result.stderr.decode('utf-8', errors='replace'))
                self.assertEqual(result.stdout.decode('utf-8').splitlines(), [message, '?'])
                self.assertEqual(result.stderr.decode('utf-8').splitlines(), [message, '?'])


class WrapperTests(unittest.TestCase):
    def test_preflight_and_cleanup(self):
        # Sao chép trình bao để không thể rơi về ucr_stop thật trong cây nguồn.
        modes = [('free', 0, ['stop', 'drive', 'stop']),
                 ('pre_fail', 1, ['stop']), ('post_fail', 1, ['stop', 'drive', 'stop']),
                 ('child_error', 4, ['stop', 'drive', 'stop']),
                 ('child_lock', 6, ['stop', 'drive']), ('busy', 6, []),
                 ('missing_stop', 1, []), ('bad_args', 2, [])]
        for mode, code, expected in modes:
            with self.subTest(mode=mode), tempfile.TemporaryDirectory(prefix='drive_startup_') as directory:
                root = Path(directory)
                scripts = root / 'vehicle' / 'scripts'
                scripts.mkdir(parents=True)
                wrapper = scripts / 'run_drive.sh'
                text = Path(ARGS.wrapper).read_text(encoding='utf-8')
                self.assertEqual(text.count('lock_file=/tmp/ucr_vehicle.lock'), 1)
                wrapper.write_bytes(text.replace('lock_file=/tmp/ucr_vehicle.lock', 'lock_file="$UCR_DRIVE_FAKE_LOCK"').encode('utf-8'))
                build = root / 'build with spaces'
                build.mkdir()
                journal, lock, ready = root / 'journal', root / 'vehicle.lock', root / 'ready'
                config = root / 'config with spaces.yaml'
                config.write_text('MO PHONG', encoding='utf-8')
                for kind, body in (
                    ('ucr_stop', 'printf "stop|%s|%s\\n" "$1" "$2" >> "$JOURNAL"\n'
                     '[ "$MODE" != pre_fail ] || exit 9\n'
                     'if [ "$MODE" = post_fail ] && [ -f "$CHILD_RAN" ]; then exit 9; fi\nexit 0\n'),
                    ('ucr_drive_collect', 'printf "drive\\n" >> "$JOURNAL"\n: > "$CHILD_RAN"\n'
                     '[ "$MODE" != child_error ] || exit 4\n'
                     '[ "$MODE" != child_lock ] || exit 6\nexit 0\n')):
                    if mode == 'missing_stop' and kind == 'ucr_stop':
                        continue
                    executable = build / kind
                    executable.write_text('#!/usr/bin/env bash\n' + body, encoding='utf-8')
                    executable.chmod(0o700)
                env = os.environ.copy()
                env.update(UCR_BUILD_DIR=shell_path(build), UCR_DRIVE_FAKE_LOCK=shell_path(lock),
                           JOURNAL=shell_path(journal), MODE=mode, CHILD_RAN=shell_path(root / 'child'))
                if os.name == 'nt':
                    env['PATH'] = '/usr/bin:/bin'
                holder = None
                try:
                    if mode == 'busy':
                        holder = subprocess.Popen([ARGS.bash, '-c',
                            'exec 9>>"$UCR_DRIVE_FAKE_LOCK"; flock -x 9 || exit; '
                            'printf ready > "$1"; read -r release', 'holder', shell_path(ready)],
                            env=env, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                        deadline = time.monotonic() + 5
                        while not ready.exists() and time.monotonic() < deadline and holder.poll() is None:
                            time.sleep(0.01)
                        self.assertTrue(ready.exists(), 'Không dựng được tiến trình giữ khóa giả')
                    command = [ARGS.bash, shell_path(wrapper), '--config']
                    if mode != 'bad_args':
                        command.append(shell_path(config))
                    result = subprocess.run(command, env=env, stdin=subprocess.DEVNULL,
                                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=15)
                    self.assertEqual(result.returncode, code, result.stdout.decode(errors='replace'))
                    rows = journal.read_text(encoding='utf-8').splitlines() if journal.exists() else []
                    self.assertEqual([row.split('|')[0] for row in rows], expected)
                    for row in rows:
                        if row.startswith('stop|'):
                            self.assertEqual(row, 'stop|--config|' + shell_path(config))
                    if mode in ('pre_fail', 'post_fail'):
                        self.assertIn(b'KIEM TRA XE, DUNG CONG TAC NGUON', result.stdout)
                finally:
                    if holder is not None:
                        holder.communicate(b'release\n', timeout=5)


@unittest.skipUnless(os.name == 'posix', 'Init cần Linux/POSIX và bản hooks đã biên dịch')
class InitTests(unittest.TestCase):
    def test_init_failures_and_signals(self):
        if not ARGS.hooks:
            self.skipTest('Chưa cung cấp --hooks')
        import pty
        import signal
        variants = [('failure', {'UCR_DRIVE_FAKE_OPEN_FAIL': 'motor_init'}, 3),
                    ('ctrl_failure', {}, 3),
                    ('motor_zero_failure', {'UCR_DRIVE_FAKE_MOTOR_FAIL': '1'}, 3),
                    ('watchdog', {'UCR_DRIVE_FAKE_INIT_MS': '8000'}, 3)]
        variants += [('signal_' + str(int(sig)), {'UCR_DRIVE_FAKE_INIT_MS': '1000'}, 128 + sig)
                     for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP, signal.SIGTSTP)]
        for mode, extra, code in variants:
            with self.subTest(mode=mode), tempfile.TemporaryDirectory(prefix='drive_init_') as directory:
                root = Path(directory)
                trace = root / 'trace'
                ctrl = root / 'ctrl'
                ctrl.write_text('motor_init\n' if mode == 'ctrl_failure' else '', encoding='utf-8')
                env = os.environ.copy()
                env.update(UCR_DRIVE_FAKE_TRACE=str(trace), UCR_DRIVE_FAKE_CTRL=str(ctrl),
                           UCR_DRIVE_FAKE_LOCK=str(root / 'lock'), UCR_DRIVE_FAKE_CAMERA='ok')
                env.update(extra)
                master, slave = pty.openpty()
                try:
                    with (root / 'output').open('wb') as output:
                        process = subprocess.Popen([ARGS.hooks, '--accept-no-obstacle-brake',
                            '--out-root', str(root / 'runs')], stdin=slave, stdout=output, stderr=output, env=env)
                        try:
                            deadline = time.monotonic() + 5
                            while time.monotonic() < deadline:
                                if trace.exists() and ' motor_init\n' in trace.read_text(encoding='utf-8'):
                                    break
                                self.assertIsNone(process.poll())
                                time.sleep(0.01)
                            self.assertTrue(trace.exists() and ' motor_init\n' in trace.read_text(encoding='utf-8'))
                            if mode.startswith('signal_'):
                                process.send_signal(int(mode.split('_')[1]))
                                os.write(master, b'RYW\n')
                            process.wait(timeout=6)
                            self.assertEqual(process.returncode, code)
                        finally:
                            if process.poll() is None:
                                process.kill()
                            process.wait(timeout=3)
                    rows = trace.read_text(encoding='utf-8').splitlines()
                    self.assertEqual(sum(row.endswith(' motor_init') for row in rows), 1)
                    self.assertFalse(any(' key ' in row for row in rows))
                    self.assertFalse(any('motor_speed ' in row and float(row.split()[2]) > 0 for row in rows))
                    if mode == 'watchdog':
                        self.assertFalse(any('motor_speed' in row or 'motor_emergency' in row for row in rows))
                    else:
                        self.assertTrue(any('motor_speed 0' in row for row in rows))
                        summaries = list((root / 'runs').glob('*/summary.json'))
                        self.assertEqual(len(summaries), 1)
                        summary = json.loads(summaries[0].read_text(encoding='utf-8'))
                        self.assertEqual(summary['exit_code'], code)
                        if not mode.startswith('signal_'):
                            self.assertEqual(summary['exit_reason'], 'motor_init_fail')
                    if code == 3:
                        self.assertIn(b'KIEM TRA XE, DUNG CONG TAC NGUON', (root / 'output').read_bytes())
                finally:
                    os.close(master)
                    os.close(slave)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bash', default='bash')
    parser.add_argument('--wrapper', default=str(HERE / '../../scripts/run_drive.sh'))
    parser.add_argument('--hooks')
    ARGS, rest = parser.parse_known_args()
    unittest.main(argv=[__file__] + rest)
