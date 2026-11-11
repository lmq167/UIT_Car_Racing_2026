#!/usr/bin/env python3
"""Thử bền (soak) ucr_drive_collect_hooks với camera giả: ghi ảnh liên tục, đo RSS, số FD, số ảnh.

Chạy trên Ubuntu/container (cần PTY, Linux). Dùng lại bộ khung test của drive_collect.
  python3 soak_drive_collect.py --source <tools/drive_collect> --hooks <ucr_drive_collect_hooks> \
      --real <ucr_drive_collect> --minutes 30 --interval-ms 200 --out <thư mục log>
Giữ W (xe giả) suốt thời gian thử; kết thúc bằng Esc. Không có động cơ thật.
"""
import argparse
import json
import os
import sys
import tempfile
import time
from pathlib import Path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--source', required=True)
    ap.add_argument('--hooks', required=True)
    ap.add_argument('--real', required=True)
    ap.add_argument('--minutes', type=float, default=30)
    ap.add_argument('--interval-ms', type=int, default=200)
    ap.add_argument('--sample-s', type=float, default=15)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    sys.path.insert(0, a.source)
    import test_drive_collect as d
    cfg = argparse.Namespace(case='SOAK', hooks=str(Path(a.hooks).resolve()), real=str(Path(a.real).resolve()),
                             cam=None, source=a.source, wrapper=str(Path(a.source).parents[1] / 'scripts' / 'run_drive.sh'),
                             slack=300, tsan_slack=0, nm='nm', strings='strings', ldd='ldd', bash='bash',
                             cmake='cmake', cxx='g++')
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    samples = []
    with tempfile.TemporaryDirectory(prefix='ucr_soak_') as tmp:
        with d.running(cfg, Path(tmp), args=['--interval-ms', str(a.interval_ms), '--no-preview']) as run:
            run.boot(settle=True)
            run.drive()
            run.pump = True
            pid = run.process.pid
            t0 = time.monotonic()
            end = t0 + a.minutes * 60
            while time.monotonic() < end:
                run.observe(a.sample_s)
                try:
                    rss = int([l for l in Path('/proc/%d/status' % pid).read_text().splitlines()
                               if l.startswith('VmRSS')][0].split()[1])
                    fds = len(os.listdir('/proc/%d/fd' % pid))
                except (OSError, IndexError):
                    rss, fds = -1, -1
                rows = len(run.rows())
                samples.append({'t_s': round(time.monotonic() - t0), 'rss_kb': rss, 'fds': fds, 'jpeg_rows': rows})
                print(json.dumps(samples[-1]), flush=True)
                state = d.matched(run.events(), 'state')
                d.need(state and state[-1][1][2] == 'CHAY', 'Xe giả rời CHAY giữa thử bền')
            run.pump = False
            run.finish()
            files = list(run.session_dir().glob('*.jpg'))
    first, last = samples[len(samples) // 10], samples[-1]
    result = {'minutes': a.minutes, 'interval_ms': a.interval_ms, 'jpeg': last['jpeg_rows'],
              'rss_kb_start': first['rss_kb'], 'rss_kb_end': last['rss_kb'],
              'fd_start': first['fds'], 'fd_end': last['fds'], 'jpg_files': len(files)}
    (out / 'soak.json').write_text(json.dumps({'result': result, 'samples': samples}, indent=1))
    print('SOAK ' + json.dumps(result))
    # Tiêu chí: >= 3000 ảnh, RSS cuối <= 1,25 x RSS đầu (sau khởi động), FD không tăng quá 2.
    ok = (last['jpeg_rows'] >= 3000 and last['rss_kb'] <= first['rss_kb'] * 1.25 and last['fds'] <= first['fds'] + 2)
    print('SOAK ' + ('ĐẠT' if ok else 'HỎNG'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
