# -*- coding: utf-8 -*-
"""Mô phỏng hình học đỗ lùi song song CÓ ô kế bên bị chiếm (phiếu PHIEU_DO_XE_TOF_1111 bước 2).

Mô hình [MP] (chưa kiểm trên xe): xe 27x22 cm, trục sau cách đuôi 4 cm (GIẢ ĐỊNH), cơ sở 19 cm, bán kính quay trục sau
R (đo 51 cm ở lái phần mềm 35 độ). Lối y in [-15,15], ô bên phải y in [-45,-15], mỗi ô 50 cm dọc lối (x in [0,50] là ô đích).
Vật cản: mọi ô khác bị chiếm (hộp đặc kín cả ô: giả định bất lợi nhất); phía đối diện y>15 bị chiếm hết.
Biến thể "vao_mo": phần x<0, y<-15 (phía đầu bãi) là chỗ trống; "vao_kin": bị chặn như ô chiếm.
Chạy: python training/tools/park_sim_occupied.py [--quick]
"""
import heapq
import math
import sys

L_REAR, L_FRONT, HALF_W = 4.0, 23.0, 11.0     # tính từ tâm trục sau (cm)
WHEELBASE = 19.0
STEP = 3.0                                    # cm mỗi bước cung
GRID_XY, GRID_TH = 1.0, 3.0                   # lưới trạng thái


def car_poly(x, y, th):
    c, s = math.cos(th), math.sin(th)
    pts = [(-L_REAR, -HALF_W), (L_FRONT, -HALF_W), (L_FRONT, HALF_W), (-L_REAR, HALF_W)]
    return [(x + px * c - py * s, y + px * s + py * c) for px, py in pts]


def rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def axes(poly):
    out = []
    for i in range(len(poly)):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % len(poly)]
        ex, ey = x2 - x1, y2 - y1
        n = math.hypot(ex, ey)
        out.append((-ey / n, ex / n))
    return out


def overlap(a, b):
    """Hai đa giác lồi: giao nhau có diện tích (chạm mép = không va)."""
    for ax in axes(a) + axes(b):
        pa = [p[0] * ax[0] + p[1] * ax[1] for p in a]
        pb = [p[0] * ax[0] + p[1] * ax[1] for p in b]
        if max(pa) <= min(pb) + 1e-6 or max(pb) <= min(pa) + 1e-6:
            return False
    return True


def make_world(open_entry, margin=0.0, slot_len=50.0, depth=30.0, lane=30.0, far=400.0):
    """Danh sách hộp cấm. Ô đích: x in [0,slot_len], y in [-lane/2-depth, -lane/2]."""
    h = lane / 2
    m = margin
    obs = [rect(-far, h - m, far, h + 200)]                              # hàng đối diện (cả phía y>15)
    obs.append(rect(-far, -h - depth - 200, far, -h - depth + m))         # sau đáy ô
    obs.append(rect(slot_len - m, -h - depth - 200, far, -h + m))         # ô kề phía sau (x>50)
    if not open_entry:
        obs.append(rect(-far, -h - depth - 200, m, -h + m))               # phía đầu bãi (x<0): chặn
    return obs


def collides(pose, obs):
    p = car_poly(*pose)
    return any(overlap(p, o) for o in obs)


def in_slot(pose, slot_len=50.0, depth=30.0, lane=30.0, tol=0.5):
    x, y, th = pose
    h = lane / 2
    return all(0 - tol <= px <= slot_len + tol and -h - depth - tol <= py <= -h + tol for px, py in car_poly(x, y, th))


def plan(start, obs, r_min, slot_len=50.0, max_nodes=400000, tol_th=8.0, y_tol=3.0, allow_multi=True, goal_cy=None):
    """A* trên (x,y,th). Tập nguyên thủy: tiến/lùi x lái trái/phải/thẳng. Trả về (đường, số lần đảo chiều, chi phí) hoặc None."""
    prims = []
    for d in (-1, 1):
        for k in (-1, 0, 1):
            prims.append((d, k))
    if not allow_multi:
        prims = [(-1, -1), (-1, 0), (-1, 1)]
    cy_goal = -15 - 15 if goal_cy is None else goal_cy

    def key(p):
        return (round(p[0] / GRID_XY), round(p[1] / GRID_XY), round(math.degrees(p[2]) / GRID_TH))

    def h(p):
        return math.hypot(p[0] - slot_len / 2, p[1] - cy_goal) * 0.8

    def goal(p):
        c = abs(p[0] - slot_len / 2) <= 6 and abs(p[1] - cy_goal) <= y_tol
        return c and abs(math.degrees(p[2])) <= tol_th and in_slot(p, slot_len)

    openq = [(h(start), 0.0, 0, start, None, 0)]
    best = {key(start): 0.0}
    parent = {}
    n = 0
    cnt = 1
    while openq and n < max_nodes:
        f, g, _, p, last, sw = heapq.heappop(openq)
        n += 1
        kk = key(p)
        if goal(p):
            path = [p]
            while kk in parent:
                kk2, p2 = parent[kk]
                path.append(p2)
                kk = kk2
            path.reverse()
            return path, sw, g, n
        if best.get(kk, 1e18) < g - 1e-9:
            continue
        for d, k in prims:
            x, y, th = p
            if k == 0:
                nx, ny, nth = x + d * STEP * math.cos(th), y + d * STEP * math.sin(th), th
            else:
                # bán kính r_min, k=+1: quay trái khi tiến
                dth = d * k * STEP / r_min
                nth = th + dth
                nx = x + k * r_min * (math.sin(nth) - math.sin(th))
                ny = y - k * r_min * (math.cos(nth) - math.cos(th))
            if abs(math.degrees(nth)) > 100 or not (-120 <= nx <= 260) or not (-45 <= ny <= 15):
                continue
            q = (nx, ny, nth)
            if collides(q, obs):
                continue
            ng = g + STEP + (12.0 if last is not None and last[0] != d else 0.0) + (0.2 if k == 0 else 0.0)
            nk = key(q)
            if ng < best.get(nk, 1e18) - 1e-9:
                best[nk] = ng
                parent[nk] = (kk, p)
                cnt += 1
                heapq.heappush(openq, (ng + h(q), ng, cnt, q, (d, k),
                                       sw + (1 if last is not None and last[0] != d else 0)))
    return None


def dense(path, r_min):
    return path


def sweep_stats(path):
    xs, ys = [], []
    for p in path:
        for px, py in car_poly(*p):
            xs.append(px)
            ys.append(py)
    return min(xs), max(xs), min(ys), max(ys)


def main():
    quick = "--quick" in sys.argv
    r_list = [51.0] if quick else [51.0, 57.0, 67.0]
    starts_x = [-20, 0, 25, 50, 75] if not quick else [0, 50]
    rows = []
    for variant, open_entry in (("vao_kin", False), ("vao_mo", True)):
        for r in r_list:
            for multi in (False, True):
                best = None
                for x0 in starts_x:
                    obs = make_world(open_entry)
                    # xe bắt đầu thẳng trong lối, căn giữa (y=0), hướng +x; kiểm không va ngay từ đầu
                    start = (float(x0), 0.0, 0.0)
                    if collides(start, obs):
                        continue
                    res = plan(start, obs, r, allow_multi=multi, max_nodes=60000 if quick else 250000)
                    if res and (best is None or res[2] < best[1][2]):
                        best = (x0, res)
                if best:
                    x0, (path, sw, cost, n) = best
                    xmin, xmax, ymin, ymax = sweep_stats(path)
                    rows.append((variant, r, "nhieu_luot" if multi else "mot_luot", "KHA THI", x0, sw, round(cost),
                                 round(xmin), round(xmax), round(ymin), round(ymax)))
                else:
                    rows.append((variant, r, "nhieu_luot" if multi else "mot_luot", "KHONG", None, None, None, None, None, None, None))
    print("bien_the | R_cm | kieu | ket_qua | x_bat_dau(truc_sau) | so_lan_doi_chieu | chi_phi_cm | xmin xmax ymin ymax (quet than xe)")
    for r in rows:
        print(" | ".join(str(c) for c in r))


if __name__ == "__main__":
    main()
