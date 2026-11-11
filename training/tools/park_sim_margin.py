# -*- coding: utf-8 -*-
"""Độ hở hình học tối đa cho đỗ lùi có ô kề bị chiếm: tìm m lớn nhất (cm) sao cho vẫn có kế hoạch né vật cản
phồng thêm m cm (m = sai số chịu được cho mọi nguồn sai số cộng lại). [MP] Chạy: python park_sim_margin.py"""
import sys, time
sys.path.insert(0, ".")
import park_sim_occupied as P

def best_plan(open_entry, r, margin, multi, starts):
    obs = P.make_world(open_entry, margin=margin)
    best = None
    for x0 in starts:
        s = (float(x0), 0.0, 0.0)
        if P.collides(s, obs):
            continue
        res = P.plan(s, obs, r, allow_multi=multi, max_nodes=120000)
        if res and (best is None or res[2] < best[1][2]):
            best = (x0, res)
    return best

t0 = time.time()
print("bien_the | R | kieu | m(cm) | ket_qua | x0 | doi_chieu | chi_phi")
for variant, oe in (("vao_kin", False), ("vao_mo", True)):
    for r in (51.0, 60.0):
        for multi in (False, True):
            for m in (0.0, 1.0, 2.0, 3.0, 4.0):
                b = best_plan(oe, r, m, multi, [-20, 0, 25, 50, 75])
                if b:
                    x0, (path, sw, cost, n) = b
                    print(variant, r, "nhieu" if multi else "mot", m, "KHA THI", x0, sw, round(cost), flush=True)
                else:
                    print(variant, r, "nhieu" if multi else "mot", m, "KHONG", flush=True)
                    break
print("giay:", round(time.time() - t0))
