#!/usr/bin/env python3
# Bọc một lệnh biên dịch (CMAKE_CXX_COMPILER_LAUNCHER): đo RSS đỉnh và thời gian từng file.
# Vượt trần (mặc định 1,2 GB, 300 s) thì thoát 97 để build hỏng ngay.
import os
import resource
import signal
import subprocess
import sys
import time

limit_kb = int(os.environ.get('UCR_RSS_LIMIT_KB', '1258291'))
limit_s = int(os.environ.get('UCR_FILE_SECONDS', '300'))
cmd = sys.argv[1:]
if not cmd:
    sys.exit(2)
name = next((cmd[i + 1] for i, a in enumerate(cmd[:-1]) if a == '-c'), cmd[-1])
start = time.time()
proc = subprocess.Popen(cmd, start_new_session=True)
try:
    rc = proc.wait(timeout=limit_s)
except subprocess.TimeoutExpired:
    os.killpg(proc.pid, signal.SIGKILL)
    proc.wait()
    sys.stderr.write('UCR_RSS_FAIL quá %d s: %s\n' % (limit_s, name))
    sys.exit(97)
elapsed = time.time() - start
peak = resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss  # KB trên Linux
sys.stderr.write('UCR_RSS %d KB %.1f s %s\n' % (peak, elapsed, name))
if rc == 0 and peak > limit_kb:
    sys.stderr.write('UCR_RSS_FAIL vượt %d KB: %s\n' % (limit_kb, name))
    sys.exit(97)
sys.exit(rc)
