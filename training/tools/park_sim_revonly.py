# -*- coding: utf-8 -*-
"""Đỗ LÙI VÀO (đúng quyết định Quân: đỗ lùi là phương án chính): chỉ tiến-lùi nhưng tổng đoạn TIẾN <= fwd_budget cm,
xe bắt đầu ở phía trước ô (trục sau x0 >= 40). [MP] Chạy: python park_sim_revonly.py"""
import heapq, math, sys, time
sys.path.insert(0, ".")
import park_sim_occupied as P

def plan_rev(start, obs, r, fwd_budget, max_nodes=150000):
    # trạng thái mở rộng thêm: ngân sách tiến đã dùng (làm tròn 3 cm)
    prims = [(d, k) for d in (-1, 1) for k in (-1, 0, 1)]
    cy = -30.0
    def key(p, f): return (round(p[0]), round(p[1]), round(math.degrees(p[2]) / 3.0), f)
    def goal(p):
        return abs(p[0] - 25) <= 6 and abs(p[1] - cy) <= 3 and abs(math.degrees(p[2])) <= 8 and P.in_slot(p)
    openq = [(0, 0.0, 0, start, None, 0, 0)]
    best = {key(start, 0): 0.0}
    parent = {}
    n = 0; cnt = 1
    while openq and n < max_nodes:
        f, g, _, p, last, sw, fw = heapq.heappop(openq)
        n += 1
        kk = key(p, fw)
        if goal(p):
            path = [p]
            while kk in parent:
                kk, p2 = parent[kk]; path.append(p2)
            path.reverse(); return path, sw, g
        if best.get(kk, 1e18) < g - 1e-9: continue
        for d, k in prims:
            x, y, th = p
            nf = fw + (1 if d > 0 else 0)
            if nf * P.STEP > fwd_budget: continue
            if k == 0: nx, ny, nth = x + d * P.STEP * math.cos(th), y + d * P.STEP * math.sin(th), th
            else:
                nth = th + d * k * P.STEP / r
                nx = x + k * r * (math.sin(nth) - math.sin(th)); ny = y - k * r * (math.cos(nth) - math.cos(th))
            if abs(math.degrees(nth)) > 100 or not (-120 <= nx <= 260) or not (-45 <= ny <= 15): continue
            q = (nx, ny, nth)
            if P.collides(q, obs): continue
            ng = g + P.STEP + (12.0 if last is not None and last[0] != d else 0.0) + (2.0 if d > 0 else 0)
            nk = key(q, nf)
            if ng < best.get(nk, 1e18) - 1e-9:
                best[nk] = ng; parent[nk] = (kk, p); cnt += 1
                h = math.hypot(nx - 25, ny - cy) * 0.8
                heapq.heappush(openq, (ng + h, ng, cnt, q, (d, k), sw + (1 if last is not None and last[0] != d else 0), nf))
    return None

t0 = time.time()
print("bien_the | R | tien_toi_da_cm | m | ket_qua | x0 | doi_chieu | chi_phi")
for variant, oe in (("vao_kin", False), ("vao_mo", True)):
    for r in (51.0,):
        for fb in (40, 60):
            for m in (0.0, 1.0, 2.0):
                obs = P.make_world(oe, margin=m)
                best = None
                for x0 in (40, 50, 60, 75, 90):
                    s = (float(x0), 0.0, 0.0)
                    if P.collides(s, obs): continue
                    res = plan_rev(s, obs, r, fb)
                    if res and (best is None or res[2] < best[1][2]): best = (x0, res)
                if best:
                    print(variant, r, fb, m, "KHA THI", best[0], best[1][1], round(best[1][2]), flush=True)
                else:
                    print(variant, r, fb, m, "KHONG", flush=True); break
print("giay", round(time.time() - t0))
