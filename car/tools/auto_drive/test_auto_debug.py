"""Hai ca chap nhan --debug-windows (D01, D02); Python 3.6/Linux."""
import argparse
import os
from pathlib import Path
import re
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_auto_drive as a


CASES = ('D01', 'D02')


def debug_rows(events):
    rows = []
    for event in events:
        if event[1] != 'debug':
            continue
        value = str(event[2])
        a.need(re.fullmatch(r'[0-9]+ [0-9]+', value),
               'Sai dinh dang debug <frame_id> <nrows>: ' + value, 'debug_format')
        rows.append(int(value.split(' ')[1]))
    return rows


def unchanged_commands(cfg, root):
    steering = {}
    for mask in ('straight', 'left', 'right'):
        samples = []
        for enabled in (False, True):
            args = ['--debug-windows'] if enabled else []
            with a.running(cfg, root / mask / ('on' if enabled else 'off'),
                           args=args, schedule=((1, mask),)) as run:
                run.boot()
                if enabled:
                    run.wait(lambda: run.of('debug'), message='Thieu debug truoc G')
                    a.need(set(debug_rows(run.events())) == {0},
                           'Truoc G phai co debug nrows=0', 'debug_before_go')
                run.go()
                mark = len(run.events())
                run.frames(30)
                events = run.events()[mark:]
                commands = {}
                for kind in ('motor_speed', 'servo_angle'):
                    values = [e[2] for e in events if e[1] == kind and not e[3]][-5:]
                    a.need(len(values) == 5, 'Thieu 5 lenh cuoi ' + kind, 'debug_commands')
                    commands[kind] = set(values)
                a.need(all(v > 0 for v in commands['motor_speed']),
                       'Mau so sanh phai dang chay', 'debug_commands')
                a.need(len(commands['servo_angle']) == 1,
                       'Goc cuoi chua on dinh: ' + repr(commands), 'debug_commands')
                samples.append(commands)
                if enabled:
                    a.need(set(debug_rows(events)) == {3},
                           'Sau G phai co debug nrows=3', 'debug_after_go')
                run.finish()
                if not enabled:
                    a.need(not run.of('debug'), 'Tat co van co debug', 'debug_disabled')
        off, on = samples
        speeds_off, speeds_on = off['motor_speed'], on['motor_speed']
        a.need(all(any(abs(x - y) <= 1e-6 for y in speeds_on) for x in speeds_off)
               and all(any(abs(y - x) <= 1e-6 for x in speeds_off) for y in speeds_on),
               'Co debug doi toc do {}: {} != {}'.format(mask, speeds_off, speeds_on),
               'debug_speed')
        a.need(off['servo_angle'] == on['servo_angle'],
               'Co debug doi goc {}: {} != {}'.format(mask, off, on), 'debug_steering')
        steering[mask] = next(iter(off['servo_angle']))
    a.need(steering['left'] * steering['right'] < 0,
           'Trai/phai phai lai khac 0 va trai dau: ' + repr(steering), 'debug_steering')


def invalid_flags(cfg, root):
    for name, args in (('extra', ['--debug-windows', 'x']),
                       ('equals', ['--debug-windows=1'])):
        with a.running(cfg, root / name, args=args) as run:
            a.reject(run)
            a.need(not run.of('motor_speed') and not run.of('servo_angle'),
                   'Co sai cu phap van gui lenh thiet bi', 'debug_reject')
            a.need(not run.of('debug'), 'Co sai cu phap van co debug', 'debug_reject')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=CASES, required=True)
    parser.add_argument('--hooks', required=True)
    parser.add_argument('--slack', type=int, choices=(0, 300), default=0)
    cfg = parser.parse_args()
    a.need(os.name == 'posix', 'Chi chay hooks tren Linux', 'environment')
    with tempfile.TemporaryDirectory(prefix='ucr-auto-debug-') as tmp:
        functions = dict(D01=unchanged_commands, D02=invalid_flags)
        functions[cfg.case](cfg, Path(tmp) / cfg.case)
    print('PASS ' + cfg.case)


if __name__ == '__main__':
    main()
