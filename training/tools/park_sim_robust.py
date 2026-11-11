# -*- coding: utf-8 -*-
"""Độ bền của kế hoạch đỗ nhiều lượt trước sai số (phiếu PHIEU_DO_XE_TOF_1111 bước 2). [MP]
Lập kế hoạch lý tưởng bằng park_sim_occupied.plan rồi chạy LẠI hở vòng (độ dài đoạn cố định) với nhiễu:
sai số bắt đầu (x,y,yaw), R thực lệch, tỉ lệ encoder lệch. Báo tỉ lệ thành công/va chạm."""
import math, random, sys
sys.path.insert(0, __file__.rsplit("park_sim_robust.py", 1)[0] if "park_sim_robust.py" in __file__ else ".")
import park_sim_occupied as P

def segments(path):
    segs = []
    for a, b in zip(path, path[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        d = math.hypot(dx, dy)
        dth = b[2] - a[2]
        fwd = (dx * math.cos(a[2]) + dy * math.sin(a[2])) >= 0
        k = 0 if abs(dth) < 1e-9 else (1 if (dth > 0) == fwd else -1)
        key = (1 if fwd else -1, k)
        if segs and segs[-1][0] == key:
            segs[-1][1] += d
        else:
            segs.append([key, d])
    return segs

def run(start, segs, r, obs, noise=None, rng=None):
    x, y, th = start
    rs = noise.get("r", 1.0) if noise else 1.0
    ls = noise.get("len", 1.0) if noise else 1.0
    rr = r * rs
    for (d, k), length in segs:
        n = max(1, int(round(length * ls / 1.0)))
        step = length * ls / n
        for _ in range(n):
            if k == 0:
                x += d * step * math.cos(th); y += d * step * math.sin(th)
            else:
                nth = th + d * k * step / rr
                x += k * rr * (math.sin(nth) - math.sin(th)); y -= k * rr * (math.cos(nth) - math.cos(th)); th = nth
            if P.collides((x, y, th), obs):
                return "VA", (x, y, th)
    ok = P.in_slot((x, y, th)) and abs(math.degrees(th)) <= 12
    return ("DAT" if ok else "LECH"), (x, y, th)

def main():
    rng = random.Random(1)
    for variant, open_entry, pm in (("vao_mo_m3", True, 3.0), ("vao_mo_m2", True, 2.0), ("vao_kin_m1", False, 1.0)):
        obs = P.make_world(open_entry)            # hình học THẬT (không phồng) để chấm
        obs_plan = P.make_world(open_entry, margin=pm)   # lập kế hoạch với độ hở pm
        best = None
        for x0 in ([-20, 0, 25, 50, 75]):
            res = P.plan((float(x0), 0.0, 0.0), obs_plan, 51.0, max_nodes=120000)
            if res and (best is None or res[2] < best[1][2]):
                best = (x0, res)
        if not best:
            print(variant, "khong kha thi"); continue
        x0, (path, sw, cost, n) = best
        segs = segments(path)
        print("== %s x0=%d doi_chieu=%d" % (variant, x0, sw))
        for (d, k), ln in segs:
            print("  %s lai=%+d dai=%.1f cm" % ("TIEN" if d > 0 else "LUI ", k, ln))
        for label, sx, sy, syaw, sr, sl in (("ly_tuong", 0, 0, 0, 0, 0), ("nhe", 2, 1.5, 2, .05, .03), ("vua", 4, 3, 4, .10, .05), ("manh", 6, 4, 6, .15, .08)):
            cnt = {"DAT": 0, "VA": 0, "LECH": 0}
            N = 1 if label == "ly_tuong" else 300
            for _ in range(N):
                st = (x0 + rng.uniform(-sx, sx), rng.uniform(-sy, sy), math.radians(rng.uniform(-syaw, syaw)))
                noise = {"r": 1 + rng.uniform(-sr, sr), "len": 1 + rng.uniform(-sl, sl)}
                if P.collides(st, obs):
                    cnt["VA"] += 1; continue
                cnt[run(st, segs, 51.0, obs, noise)[0]] += 1
            print("  nhieu=%-9s (x±%.0f y±%.1f yaw±%.0f° R±%d%% dai±%d%%): %s" % (label, sx, sy, syaw, sr * 100, sl * 100, cnt))

if __name__ == "__main__":
    main()
