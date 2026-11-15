#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Phát lại ngoại tuyến ExtractCorridor (bản Python bám sát LaneController.cpp) trên mặt nạ thật.

Dùng để kiểm hiện tượng: cạnh phải chạm biên ảnh + cạnh trái thật
-> hàng bị dựng thành "một làn" -> target nằm ở làn TRÁI của đường hai làn.

Cách dùng:
  python round3_final/tools/corridor_replay.py <thư_mục_phiên> [--t0 280 --t1 294] [--fix]
In mỗi khung: thời điểm (s), trạng thái planner, e log, và mỗi hàng: cờ trái/phải, lanes, target_frac.
Luật hai làn khi biên phải bị cắt theo mã hiện hành (chỉ ROUNDABOUT, hàng dưới 2 làn); --fix giữ để tương thích.
Chỉ đọc dữ liệu, không ghi gì vào data/.
"""
import argparse
import csv
import pathlib

import cv2
import numpy as np

K_NOISE = 0.15
K_CONF_PEN = 0.3
K_NO_REAL = 0.1
K_MERGE = 1.5
K_MIN_W = 0.6
K_RIGHT, K_LEFT = 0.25, 0.75
THAT, EDGE, JUMP = "THAT", "MEP_ANH", "NHAY"

P = dict(corridor_rows=8, corridor_bottom_frac=0.95, corridor_top_frac=0.62,
         lane1_width_frac=(0.57, 0.17), two_lane_ratio=1.5, max_jump_frac=0.08,
         avoid_min_width_ratio=0.75, center_offset_frac=0.0)


def extract(mask, p=P, mode="LANE", fix=False):
    branch = mode in ("ROUNDABOUT", "TURN_LEFT", "TURN_RIGHT")
    H, W = mask.shape
    n = p["corridor_rows"]
    axis = W / 2.0 + p["center_offset_frac"] * W
    y_near = round(p["corridor_bottom_frac"] * H)
    y_far = round(p["corridor_top_frac"] * H)
    rows = []
    lower = -1
    lower_raw = (0, 0)
    stopped = False
    for i in range(n):
        frac = p["corridor_bottom_frac"] + (p["corridor_top_frac"] - p["corridor_bottom_frac"]) * i / max(1, n - 1)
        row = dict(y=int(min(max(round(frac * H), 0), H - 1)), valid=False)
        t = min(max((y_near - row["y"]) / (y_near - y_far), 0), 1) if y_near > y_far else 0
        w = W * (p["lane1_width_frac"][0] + (p["lane1_width_frac"][1] - p["lane1_width_frac"][0]) * t)
        row["w"] = w
        if stopped:
            rows.append(row)
            continue
        px = mask[row["y"]] > 0
        segs = []
        x = 0
        while x < W:
            if not px[x]:
                x += 1
                continue
            a = x
            while x < W and px[x]:
                x += 1
            if x - a >= K_NOISE * w:
                segs.append((a, x - 1))
        pick = []
        if lower < 0:
            best = w
            for s in segs:
                d = max(0.0, max(s[0] - axis, axis - s[1]))
                if d <= best:
                    best = d
                    pick = [s]
                    if d == 0:
                        break
            if not branch:
                for k in range(1, len(segs)):
                    if segs[k - 1][1] < axis and segs[k][0] > axis and \
                            segs[k][1] - segs[k - 1][0] <= p["two_lane_ratio"] * w * K_MERGE:
                        pick = [segs[k - 1], segs[k]]
                        break
        else:
            overlap = [s for s in segs if s[1] >= lower_raw[0] and s[0] <= lower_raw[1]]
            if not overlap:
                stopped = True
                rows.append(row)
                continue
            if len(overlap) >= 2 and not branch and \
                    overlap[-1][1] - overlap[0][0] <= p["two_lane_ratio"] * w * K_MERGE:
                pick = overlap
            elif branch:
                pick = [overlap[0] if mode == "TURN_LEFT" else overlap[-1]]
            else:
                best = -1
                for s in overlap:
                    amt = min(s[1], lower_raw[1]) - max(s[0], lower_raw[0])
                    if amt > best:
                        best = amt
                        pick = [s]
        if not pick:
            rows.append(row)
            continue
        raw = (pick[0][0], pick[-1][1])
        row["hole"] = len(pick) >= 2
        below = rows[lower] if lower >= 0 else None
        jump = p["max_jump_frac"] * W
        fl = EDGE if raw[0] <= 0 else (JUMP if below and below["flag_left"] == THAT and abs(raw[0] - lower_raw[0]) > jump else THAT)
        fr = EDGE if raw[1] >= W - 1 else (JUMP if below and below["flag_right"] == THAT and abs(raw[1] - lower_raw[1]) > jump else THAT)
        row["flag_left"], row["flag_right"] = fl, fr
        lok, rok = fl == THAT, fr == THAT
        ref = below["right"] - below["left"] if below and below["flag_left"] == THAT and below["flag_right"] == THAT else w
        row["raw_l"], row["raw_r"] = raw
        left, right = float(raw[0]), float(raw[1])
        if not lok and rok:
            left = right - ref
        elif lok and not rok:
            right = left + ref
            if mode == "ROUNDABOUT" and fr == EDGE and raw[1] - raw[0] >= p["two_lane_ratio"] * w and (below is None or below.get("lanes") == 2):
                right = left + max(ref, 2 * w)
        elif not lok and not rok:
            c = (below["left"] + below["right"]) / 2 if below else (raw[0] + raw[1]) / 2.0
            left, right = c - ref / 2, c + ref / 2
        if branch:
            left = max(left, raw[0])
            right = min(right, raw[1])
            if left > right:
                left, right = float(raw[0]), float(raw[1])
        width = right - left
        row["left"], row["right"] = left, right
        row["lanes"] = 2 if width >= p["two_lane_ratio"] * w else 1
        row["target"] = right - K_RIGHT * width if row["lanes"] == 2 else (left + right) / 2
        bad = int(not lok) + int(not rok)
        odd = width < K_MIN_W * w or width > p["two_lane_ratio"] * K_MERGE * w
        row["conf"] = min(max(1 - K_CONF_PEN * (bad + int(row["hole"]) + int(odd)) - (K_NO_REAL if bad == 2 else 0), 0), 1)
        row["valid"] = True
        rows.append(row)
        lower = len(rows) - 1
        lower_raw = raw
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session")
    ap.add_argument("--t0", type=float, default=0)
    ap.add_argument("--t1", type=float, default=1e9)
    ap.add_argument("--fix", action="store_true")
    ap.add_argument("--every", type=int, default=1)
    a = ap.parse_args()
    s = pathlib.Path(a.session)
    log = list(csv.DictReader(open(s / "control_log.csv", encoding="utf-8")))
    t0 = int(log[0]["t_ms"])
    seen = set()
    k = 0
    stats = dict(frames=0, edge_right_1lane=0, edge_right_wide=0)
    for r in log:
        ts = (int(r["t_ms"]) - t0) / 1000.0
        mf = r["mask_file"]
        if not mf or mf in seen or ts < a.t0 or ts > a.t1:
            continue
        seen.add(mf)
        mp = s / mf if (s / mf).exists() else s / "masks" / pathlib.Path(mf).name
        if not mp.exists():
            continue
        k += 1
        if k % a.every:
            continue
        m = cv2.imread(str(mp), cv2.IMREAD_UNCHANGED)
        if m is None:
            continue
        if m.ndim == 3:
            m = m[:, :, 0]
        rows = extract(m, mode="ROUNDABOUT" if r["reason"].startswith("VX_") else "LANE", fix=a.fix)
        W = m.shape[1]
        stats["frames"] += 1
        r0 = rows[0]
        if r0.get("valid") and r0["flag_left"] == THAT and r0["flag_right"] == EDGE:
            stats["edge_right_1lane"] += r0["lanes"] == 1
            stats["edge_right_wide"] += (r0["raw_r"] - r0["raw_l"]) >= P["two_lane_ratio"] * r0["w"]
        parts = []
        for rw in rows[:4]:
            if not rw.get("valid"):
                parts.append("--")
                continue
            parts.append("%s%s L%d T%.2f raw%.2f-%.2f" % (rw["flag_left"][0], rw["flag_right"][0], rw["lanes"],
                                                          rw["target"] / W, rw["raw_l"] / W, rw["raw_r"] / W))
        print("%6.1f %-12s e=%6s deg=%5s %s | %s" % (ts, r["planner_state"][:12], r["e"][:6], r["sent_steer_deg"][:5],
                                                   pathlib.Path(mf).name, " | ".join(parts)))
    print("THONG KE", stats)


if __name__ == "__main__":
    main()
