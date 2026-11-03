"""Phân tích một phiên chạy ucr_auto (runs_auto/<phiên>): báo cáo ngắn và ảnh tổng hợp.

Dùng: python analyze_auto_run.py runs_auto/<phiên> [--out THƯ_MỤC]
Ghi <phiên>/report/summary.md (<= 60 dòng); thêm timeseries.png, valid_rows.png nếu có matplotlib
và contact_sheet.png (ảnh + mask quanh các lần dừng / hàng quét không hợp lệ) nếu có opencv.
Cột chưa có trong control_log.csv thì ghi "thiếu", không đoán.
"""
import argparse
import csv
import json
import math
import sys
from collections import Counter
from pathlib import Path

MISSING = 'thiếu'
MAX_LINES = 60
FIELDS = ('t_ms', 'frame_id', 'go', 'planner_state', 'reason', 'e', 'sent_steer_deg', 'sent_speed_cms',
          'scan_rows', 'loop_ms', 'road_ms', 'rss_mb', 'free_mb')
SCANNED_STATES = ('BAM_LAN', 'GIU_GOC', 'MAT_LAN')  # LaneController thật sự quét mask
MAX_GAP_MS = 1000  # khoảng cách giữa hai dòng lớn hơn mức này không tính vào thời gian
FLIP_MIN_DEG = 1.0  # chỉ tính đổi dấu khi |góc| >= mức này (bỏ nhiễu quanh 0)


def num(value):
    try:
        x = float(value)
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) else None


def pct(values, q):
    """Hạng gần nhất (cùng cách percentile95 của ucr_auto)."""
    if not values:
        return None
    ordered = sorted(values)
    return ordered[max(0, math.ceil(len(ordered) * q) - 1)]


def fmt(x, digits=2):
    return MISSING if x is None else '{:.{}f}'.format(x, digits)


def load_session(session):
    path = session / 'control_log.csv'
    if not path.is_file():
        raise SystemExit('Không thấy control_log.csv trong ' + str(session))
    with path.open(encoding='utf-8', newline='') as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames or []
        rows = [row for row in reader if None not in row.values() and None not in row]
    meta = {}
    info = session / 'session.json'
    if info.is_file():
        try:
            meta = json.loads(info.read_text(encoding='utf-8'))
        except ValueError:
            meta = {}
    return fields, rows, meta


def column(rows, name):
    return [num(r.get(name)) for r in rows]


def durations(times):
    """Thời lượng mỗi dòng = khoảng tới dòng sau (<= MAX_GAP_MS); dòng cuối = 0."""
    out = []
    for i, t in enumerate(times):
        nxt = times[i + 1] if i + 1 < len(times) else None
        gap = (nxt - t) if (t is not None and nxt is not None) else 0
        out.append(gap if 0 <= gap <= MAX_GAP_MS else 0)
    return out


def scan_flags(row):
    """Danh sách valid theo từng hàng quét, hoặc None nếu thiếu/hỏng."""
    try:
        data = json.loads(row.get('scan_rows') or '')
        return [bool(item['valid']) for item in data]
    except (ValueError, TypeError, KeyError):
        return None


def stop_segments(rows, speeds, times):
    """Đoạn dừng = các dòng liên tiếp có tốc độ gửi 0 SAU lần chạy đầu tiên (speed > 0)."""
    segments, start, seen_motion = [], None, False
    for i, speed in enumerate(speeds):
        if speed is None:
            continue
        if speed > 0:
            seen_motion = True
            if start is not None:
                segments.append((start, i))
                start = None
        elif seen_motion and start is None:
            start = i
    if start is not None:
        segments.append((start, len(rows)))
    out = []
    for a, b in segments:
        t0, t1 = times[a], times[b - 1] if b - 1 < len(times) else None
        out.append({'row': a, 't_ms': t0, 'length_ms': (t1 - t0) if (t0 is not None and t1 is not None) else None,
                    'reason': rows[a].get('reason', '?')})
    return out


def analyze(fields, rows, meta):
    stats = {'missing': [name for name in FIELDS if name not in fields], 'rows': len(rows)}
    times = column(rows, 't_ms')
    speeds = column(rows, 'sent_speed_cms')
    angles = column(rows, 'sent_steer_deg')
    gaps = durations(times)
    total_ms = sum(gaps)
    state = [r.get('planner_state', '') for r in rows]
    lane_ms = sum(g for g, s in zip(gaps, state) if s == 'BAM_LAN')
    stats['total_s'] = total_ms / 1000.0
    stats['lane_s'] = lane_ms / 1000.0
    stats['lane_pct'] = (100.0 * lane_ms / total_ms) if total_ms else None
    stats['go_events'] = sum(1 for r in rows if (r.get('go') or '0') in ('1', 'true', 'True'))
    stats['reasons'] = Counter(r.get('reason', '?') for r in rows)
    # Tỉ lệ hợp lệ theo từng hàng quét, chỉ tính các dòng mà bộ điều khiển thật sự quét mask.
    per_row, scanned = [], 0
    for r in rows:
        if r.get('planner_state') in SCANNED_STATES:
            flags = scan_flags(r)
            if flags:
                scanned += 1
                while len(per_row) < len(flags):
                    per_row.append([0, 0])
                for i, ok in enumerate(flags):
                    per_row[i][0] += ok
                    per_row[i][1] += 1
    stats['valid_by_row'] = [(ok / n if n else None) for ok, n in per_row]
    stats['scanned_rows'] = scanned
    for name in ('e', 'sent_steer_deg', 'sent_speed_cms'):
        values = [v for v, s in zip(column(rows, name), state) if v is not None and s == 'BAM_LAN']
        stats[name] = values
    # Độ lắc: số lần góc lái đổi dấu mỗi giây trong lúc BAM_LAN.
    flips, previous = 0, None
    for a, s in zip(angles, state):
        if a is None or s != 'BAM_LAN' or abs(a) < FLIP_MIN_DEG:
            continue
        if previous is not None and previous * a < 0:
            flips += 1
        previous = a
    stats['flips'] = flips
    stats['flips_per_s'] = (flips / stats['lane_s']) if stats['lane_s'] > 0 else None
    for name in ('loop_ms', 'road_ms'):
        stats[name] = [v for v in column(rows, name) if v is not None and v > 0]
    for name in ('rss_mb', 'free_mb'):
        stats[name] = [v for v in column(rows, name) if v is not None and v > 0]
    stats['stops'] = stop_segments(rows, speeds, times)
    stats['t0'] = next((t for t in times if t is not None), None)
    stats['git'] = str(meta.get('git_hash', MISSING))[:10]
    stats['engine'] = Path(str(meta.get('engine_name') or MISSING)).name
    return stats


def summarize(session, stats):
    def have(*names):
        return all(n not in stats['missing'] for n in names)

    def spread(values, digits=2):
        if not values:
            return MISSING
        mean = sum(values) / len(values)
        std = math.sqrt(sum((v - mean) ** 2 for v in values) / len(values))
        return 'trung bình {} lệch chuẩn {} min {} max {}'.format(
            fmt(mean, digits), fmt(std, digits), fmt(min(values), digits), fmt(max(values), digits))

    lines = ['# Phiên {}'.format(session.name),
             '- {} dòng, {} s; git {}; engine {}'.format(stats['rows'], fmt(stats['total_s'], 1), stats['git'],
                                                          stats['engine'])]
    if stats['missing']:
        lines.append('- Cột {}: {}'.format(MISSING, ', '.join(stats['missing'])))
    lines += ['## Dòng theo reason']
    for reason, count in stats['reasons'].most_common(10):
        lines.append('- {}: {}'.format(reason, count))
    if len(stats['reasons']) > 10:
        lines.append('- ... {} reason nữa'.format(len(stats['reasons']) - 10))
    lines += ['## Lái xe',
              '- Sự kiện go (phím G): {}; thời gian BAM_LAN {} s = {}% thời gian'.format(
                  stats['go_events'] if have('go') else MISSING, fmt(stats['lane_s'], 1),
                  fmt(stats['lane_pct'], 1) if have('t_ms', 'planner_state') else MISSING),
              '- Góc lái gửi (độ): ' + (spread(stats['sent_steer_deg']) if have('sent_steer_deg') else MISSING),
              '- Sai số e (đích điều khiển, -1..1): ' + (spread(stats['e'], 3) if have('e') else MISSING),
              '- Tốc độ gửi (cm/s): ' + (spread(stats['sent_speed_cms']) if have('sent_speed_cms') else MISSING),
              '- Độ lắc: {} lần đổi dấu góc (|góc| >= {}°) = {} lần/giây'.format(
                  stats['flips'], FLIP_MIN_DEG, fmt(stats['flips_per_s']))
              if have('sent_steer_deg') else '- Độ lắc: ' + MISSING]
    if stats['sent_speed_cms'] and have('sent_speed_cms'):
        speeds = Counter(round(v, 1) for v in stats['sent_speed_cms'])
        lines.append('- Phân bố tốc độ: ' + ', '.join('{} cm/s: {}'.format(v, c) for v, c in sorted(speeds.items())[:8]))
    lines.append('## Hàng quét (chỉ dòng mà bộ điều khiển quét mask: {} dòng)'.format(stats['scanned_rows']))
    if have('scan_rows') and stats['valid_by_row']:
        lines.append('- Tỉ lệ hợp lệ theo hàng (từ hàng gần xe): ' +
                     ', '.join('hàng {} = {}'.format(i, fmt(None if v is None else v * 100, 1) + '%')
                               for i, v in enumerate(stats['valid_by_row'])))
    else:
        lines.append('- ' + MISSING)
    lines.append('## Thời gian xử lý (ms; p50 / p95)')
    for name in ('loop_ms', 'road_ms'):
        values = stats[name]
        lines.append('- {}: {} / {}'.format(name, fmt(pct(values, 0.5)), fmt(pct(values, 0.95)))
                     if values and have(name) else '- {}: {}'.format(name, MISSING))
    for name in ('rss_mb', 'free_mb'):
        values = stats[name]
        lines.append('- {}: min {} max {} cuối {}'.format(name, fmt(min(values), 0), fmt(max(values), 0), fmt(values[-1], 0))
                     if values and have(name) else '- {}: {}'.format(name, MISSING))
    lines.append('## Các đoạn dừng sau khi đã chạy ({})'.format(len(stats['stops'])))
    t0 = stats['t0']
    for seg in stats['stops'][:8]:
        start = (seg['t_ms'] - t0) / 1000.0 if (seg['t_ms'] is not None and t0 is not None) else None
        lines.append('- t={} s, dài {} ms, lý do {}'.format(
            fmt(start, 1), fmt(seg['length_ms'], 0) if seg['length_ms'] is not None else MISSING, seg['reason']))
    if len(stats['stops']) > 8:
        lines.append('- ... {} đoạn nữa'.format(len(stats['stops']) - 8))
    if len(lines) > MAX_LINES:
        lines = lines[:MAX_LINES - 1] + ['- ... (cắt bớt cho đủ {} dòng)'.format(MAX_LINES)]
    return lines


def plots(out, rows, stats):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return 'thiếu matplotlib'
    times = column(rows, 't_ms')
    t0 = stats['t0']
    if t0 is None:
        return 'thiếu t_ms'
    x = [(t - t0) / 1000.0 if t is not None else float('nan') for t in times]
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 7))
    for ax, name, label in zip(axes, ('sent_steer_deg', 'e', 'sent_speed_cms'), ('góc lái (độ)', 'e', 'tốc độ (cm/s)')):
        ys = column(rows, name)
        ax.plot(x, [y if y is not None else float('nan') for y in ys], lw=0.8)
        ax.set_ylabel(label)
        for seg in stats['stops']:
            if seg['t_ms'] is not None and seg['length_ms']:
                ax.axvspan((seg['t_ms'] - t0) / 1000.0, (seg['t_ms'] - t0 + seg['length_ms']) / 1000.0, color='red', alpha=0.15)
    axes[-1].set_xlabel('giây')
    fig.tight_layout()
    fig.savefig(str(out / 'timeseries.png'), dpi=90)
    plt.close(fig)
    if stats['valid_by_row']:
        fig, ax = plt.subplots(figsize=(5, 3))
        ax.bar([str(i) for i in range(len(stats['valid_by_row']))], [(v or 0) * 100 for v in stats['valid_by_row']])
        ax.set_ylabel('% hợp lệ')
        ax.set_xlabel('hàng quét (0 = gần xe)')
        fig.tight_layout()
        fig.savefig(str(out / 'valid_rows.png'), dpi=90)
        plt.close(fig)
    return 'đã ghi'


def contact_sheet(session, out, rows, stats, limit=12):
    try:
        import cv2
        import numpy as np
    except ImportError:
        return 'thiếu opencv/numpy'
    folder = session / 'images'
    saved = {int(p.stem): p for p in folder.glob('*.jpg') if p.stem.isdigit()} if folder.is_dir() else {}
    if not saved:
        return 'thiếu ảnh (images/ rỗng)'
    picks = []
    for seg in stats['stops'][:limit]:
        i = max(0, seg['row'] - 1)
        picks.append(('DUNG ' + str(seg['reason']), num(rows[i].get('frame_id'))))
    bad = [r for r in rows if r.get('planner_state') in SCANNED_STATES and (lambda f: f and not all(f))(scan_flags(r))]
    step = max(1, len(bad) // max(1, limit - len(picks)))
    for r in bad[::step][:max(0, limit - len(picks))]:
        picks.append(('HANG KHONG HOP LE', num(r.get('frame_id'))))
    picks = [(label, fid) for label, fid in picks if fid is not None][:limit]
    if not picks:
        return 'không có lần dừng hoặc hàng không hợp lệ để lấy mẫu'
    palette = np.array([[40, 40, 40], [200, 100, 30], [0, 255, 255]], dtype=np.uint8)
    tiles = []
    for label, fid in picks:
        near = min(saved, key=lambda k: abs(k - fid))
        image = cv2.imread(str(saved[near]))
        if image is None:
            continue
        image = cv2.resize(image, (320, 180))
        mask = cv2.imread(str(session / 'masks' / '{}.png'.format(near)), cv2.IMREAD_UNCHANGED)
        if mask is None or mask.ndim != 2:
            mask_view = np.full((180, 320, 3), 90, np.uint8)
        else:
            mask_view = cv2.resize(palette[np.minimum(mask, 2)], (320, 180), interpolation=cv2.INTER_NEAREST)
        tile = np.hstack([image, mask_view])
        cv2.putText(tile, '{} khung {} (mau {})'.format(label, int(fid), near), (4, 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        tiles.append(tile)
    if not tiles:
        return 'thiếu ảnh đọc được'
    if len(tiles) % 2:
        tiles.append(np.zeros_like(tiles[0]))
    sheet = np.vstack([np.hstack(tiles[i:i + 2]) for i in range(0, len(tiles), 2)])
    cv2.imwrite(str(out / 'contact_sheet.png'), sheet)
    return 'đã ghi {} ô'.format(len(picks))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('session', type=Path, help='thư mục phiên runs_auto/<phiên>')
    parser.add_argument('--out', type=Path, help='thư mục ra (mặc định <phiên>/report)')
    args = parser.parse_args(argv)
    if hasattr(sys.stdout, 'reconfigure'):  # Windows cp1258 không in được hết tiếng Việt
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    session = args.session.resolve()
    out = (args.out or session / 'report')
    out.mkdir(parents=True, exist_ok=True)
    fields, rows, meta = load_session(session)
    stats = analyze(fields, rows, meta)
    lines = summarize(session, stats)
    (out / 'summary.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print('\n'.join(lines))
    print('Ảnh biểu đồ: {}; contact sheet: {}'.format(plots(out, rows, stats), contact_sheet(session, out, rows, stats)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
