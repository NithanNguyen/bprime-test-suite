#!/usr/bin/env python3
"""Unit tests for common/t7_lib.py on a synthetic two-room house (no Isaac Sim needed).

Run on the host:  python3 common/test_t7_lib.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from t7_lib import find_spawn, floor_area_xy, grade_ratio, image_stats  # noqa: E402

# Axis-aligned boxes (xmin, ymin, zmin, xmax, ymax, zmax), units = metres.
HOUSE = [
    (0, 0, -0.1, 10, 8, 0.0),        # floor slab, house only
    (10, 0, -0.1, 14, 8, 0.0),       # yard ground: floor but no ceiling -> outdoor
    (0, 0, 2.7, 10, 8, 2.8),         # ceiling
    (-0.1, 0, 0, 0, 8, 2.7),         # outer walls
    (10, 0, 0, 10.1, 3, 2.7),        # wall with a door gap to the yard at y in [3, 4]
    (10, 4, 0, 10.1, 8, 2.7),
    (0, -0.1, 0, 10, 0, 2.7),
    (0, 8, 0, 10, 8.1, 2.7),
    (5, 0, 0, 5.1, 5, 2.7),          # interior wall from y=0 to y=5; the opening y in [5, 8] joins the rooms
    (2.0, 2.0, 0.72, 3.2, 3.0, 0.75),  # table top (legs omitted, rays at 0.10 m pass under it)
    (7.0, 6.0, 0.0, 9.0, 7.0, 0.8),  # sofa
]


def ray_box(o, d, b):
    tmin, tmax = -math.inf, math.inf
    for k in range(3):
        lo, hi = b[k], b[k + 3]
        if abs(d[k]) < 1e-12:
            if o[k] < lo or o[k] > hi:
                return None
            continue
        t1, t2 = (lo - o[k]) / d[k], (hi - o[k]) / d[k]
        tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
    if tmax < max(tmin, 0.0):
        return None
    return tmin if tmin >= 0.0 else tmax   # from inside a box: report the exit face (both-sides query)


def make_raycast(boxes):
    def raycast(o, d, max_dist):
        best = None
        for b in boxes:
            t = ray_box(o, d, b)
            if t is not None and t <= max_dist and (best is None or t < best):
                best = t
        return best
    return raycast


def check(name, cond, detail=""):
    print(f"{'ok  ' if cond else 'FAIL'} {name} {detail}")
    return cond


def main():
    ok = True
    rc = make_raycast(HOUSE)
    res, diag = find_spawn(rc, (-0.1, -0.1, -0.1), (14, 8.1, 2.8), log=print)
    ok &= check("spawn found", res is not None, str(diag))
    if res:
        x, y = res["x"], res["y"]
        ok &= check("floor height = 0", abs(res["floor_z"]) < 1e-9, f"floor_z={res['floor_z']}")
        ok &= check("spawn indoors", 0 < x < 10 and 0 < y < 8, f"({x:.2f}, {y:.2f})")
        ok &= check("not under table", not (2.0 <= x <= 3.2 and 2.0 <= y <= 3.0))
        ok &= check("clear of interior wall", abs(x - 5.05) > 0.35 + 0.05)
        ok &= check("clear of sofa", not (6.65 <= x <= 9.35 and 5.65 <= y <= 7.35))
    ok &= check("outdoor rejected", diag["rejected"].get("no_ceiling(outdoor)", 0) > 0)
    ok &= check("under-table rejected", diag["rejected"].get("low_headroom(under_furniture)", 0) > 0)

    res2, diag2 = find_spawn(make_raycast([]), (0, 0, 0), (5, 5, 3))
    ok &= check("no colliders -> None with reason", res2 is None and "no collider" in diag2.get("reason", ""))

    # bbox bottom far below the floor (Office case: zmin -86 m): the floor probe misses,
    # a given floor_z fixes it
    res3, diag3 = find_spawn(rc, (-0.1, -0.1, -86.0), (14, 8.1, 2.8))
    ok &= check("deep bbox -> probe misses", res3 is None and "no collider" in diag3.get("reason", ""))
    res4, diag4 = find_spawn(rc, (-0.1, -0.1, -86.0), (14, 8.1, 2.8), floor_z=0.0)
    ok &= check("deep bbox + floor_z -> spawn indoors",
                res4 is not None and 0 < res4["x"] < 10 and 0 < res4["y"] < 8, str(diag4))

    # floor area: 4 m x 5 m slab, top and bottom faces both listed -> counted once
    sq = [(0, 0, 4, 0, 4, 5), (0, 0, 4, 5, 0, 5)]
    area = floor_area_xy(sq + sq, cell=0.1)
    ok &= check("floor area 20 m^2", abs(area - 20.0) < 0.5, f"{area:.2f}")
    lshape = [(0, 0, 4, 0, 4, 2), (0, 0, 4, 2, 0, 2), (0, 2, 2, 2, 2, 4), (0, 2, 2, 4, 0, 4)]
    area = floor_area_xy(lshape, cell=0.05)
    ok &= check("L-shaped area 12 m^2", abs(area - 12.0) < 0.3, f"{area:.2f}")

    m, s = image_stats(bytes([0]) * 3000)
    ok &= check("black image", m == 0 and s == 0)
    m, s = image_stats(bytes(range(256)) * 50, stride=1)
    ok &= check("ramp image", abs(m - 127.5) < 0.01 and s > 70, f"mean={m:.1f} std={s:.1f}")

    ok &= check("grades", [grade_ratio(r) for r in (1.0, 0.6, 2.0, float("nan"))] == ["PASS", "WARN", "FAIL", "FAIL"])
    print("ALL PASS" if ok else "SOME TESTS FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
