"""
Pure-Python helpers shared by the T7 scripts (no Isaac Sim / ROS imports, so they can be
unit-tested on any machine with `python3 common/test_t7_lib.py`).

  find_spawn()      pick a free, indoor floor cell for the robot using only a raycast callback
  floor_area_xy()   union area of horizontal triangles projected on the ground plane (raster)
  image_stats()     mean / std of an 8-bit image buffer (detects black or flat renders)
  grade_ratio()     PASS 0.8-1.2, WARN 0.5-1.5, FAIL otherwise (same rule as T6d/T6e)

numpy is imported lazily: the Foxy container only needs image_stats() and grade_ratio().
"""
import math
from collections import Counter


def grade_ratio(ratio):
    if ratio is None or (isinstance(ratio, float) and math.isnan(ratio)):
        return "FAIL"
    if 0.8 <= ratio <= 1.2:
        return "PASS"
    if 0.5 <= ratio <= 1.5:
        return "WARN"
    return "FAIL"


# ---------------------------------------------------------------- spawn search
def _horizontal_dirs(n):
    return [(math.cos(2 * math.pi * k / n), math.sin(2 * math.pi * k / n), 0.0) for k in range(n)]


def find_spawn(raycast, bbox_min, bbox_max, m2u=1.0, step_m=0.5, margin_m=0.3,
               probe_h_m=1.0, clearance_m=0.35, headroom_m=1.5, ceiling_max_m=5.0,
               floor_tol_m=0.03, ray_heights_m=(0.10, 0.40), n_dirs=16, floor_z=None, log=None):
    """Return the best spawn cell or None.

    raycast(origin, direction, max_dist) -> hit distance (float) or None. Coordinates are in
    stage units, Z up. m2u converts metres to stage units (1 / metersPerUnit).

    A cell (x, y) is valid when:
      a) a ray cast down from floor+0.5 m hits the floor itself (no furniture top in between);
      b) a ray cast up from floor+0.05 m hits something between headroom_m and ceiling_max_m
         (indoors, and not under a table or bed);
      c) horizontal rays at each height in ray_heights_m, in n_dirs directions, find nothing
         closer than clearance_m.
    Among valid cells, the one with the most valid 8-neighbours wins (deepest in free space);
    ties go to the largest horizontal clearance, then to the cell closest to the bbox centre.
    """
    s = m2u
    xmin, ymin, zmin = bbox_min
    xmax, ymax, _zmax = bbox_max
    step, margin = step_m * s, margin_m * s
    xs, ys = [], []
    x = xmin + margin
    while x <= xmax - margin:
        xs.append(x)
        x += step
    y = ymin + margin
    while y <= ymax - margin:
        ys.append(y)
        y += step
    diag = {"cells": len(xs) * len(ys)}
    if not xs or not ys:
        diag["reason"] = "bounding box smaller than the search margin"
        return None, diag

    # pass 1: most common horizontal surface height = floor (skipped when the caller gives floor_z)
    down = (0.0, 0.0, -1.0)
    if floor_z is None:
        z0 = zmin + probe_h_m * s
        heights = []
        for x in xs:
            for y in ys:
                d = raycast((x, y, z0), down, 2.0 * probe_h_m * s)
                if d is not None:
                    heights.append(round((z0 - d) / (0.01 * s)))
        if not heights:
            diag["reason"] = ("no collider hit by downward rays near the bottom of the search bbox "
                              "(no colliders? try --add-colliders; floor not at the bbox bottom? give --floor-z)")
            return None, diag
        floor_z = Counter(heights).most_common(1)[0][0] * 0.01 * s
        diag["pass1_hits"] = len(heights)
    diag["floor_z"] = floor_z

    up = (0.0, 0.0, 1.0)
    dirs = _horizontal_dirs(n_dirs)
    valid = {}
    rejected = Counter()
    for i, x in enumerate(xs):
        for j, y in enumerate(ys):
            d = raycast((x, y, floor_z + 0.5 * s), down, 0.6 * s)
            if d is None or abs((floor_z + 0.5 * s - d) - floor_z) > floor_tol_m * s:
                rejected["not_floor"] += 1
                continue
            d = raycast((x, y, floor_z + 0.05 * s), up, ceiling_max_m * s)
            if d is None:
                rejected["no_ceiling(outdoor)"] += 1
                continue
            if d < headroom_m * s:
                rejected["low_headroom(under_furniture)"] += 1
                continue
            min_clear = float("inf")
            blocked = False
            for h in ray_heights_m:
                for dvec in dirs:
                    dd = raycast((x, y, floor_z + h * s), dvec, 3.0 * clearance_m * s)
                    if dd is not None:
                        min_clear = min(min_clear, dd)
                        if dd < clearance_m * s:
                            blocked = True
                            break
                if blocked:
                    break
            if blocked:
                rejected["obstacle_within_clearance"] += 1
                continue
            valid[(i, j)] = min_clear
    diag["valid"] = len(valid)
    diag["rejected"] = dict(rejected)
    if not valid:
        diag["reason"] = "no cell passed all checks"
        return None, diag

    cx, cy = (xmin + xmax) / 2.0, (ymin + ymax) / 2.0
    best, best_key = None, None
    for (i, j), clear in valid.items():
        nb = sum((i + di, j + dj) in valid for di in (-1, 0, 1) for dj in (-1, 0, 1) if (di, dj) != (0, 0))
        dist_c = math.hypot(xs[i] - cx, ys[j] - cy)
        clear_v = clear if math.isfinite(clear) else 3.0 * clearance_m * s
        key = (nb, clear_v, -dist_c)
        if best_key is None or key > best_key:
            best_key, best = key, (xs[i], ys[j])
    result = {"x": best[0], "y": best[1], "floor_z": floor_z, "neighbours": best_key[0],
              "clearance": best_key[1]}
    if log:
        log(f"spawn search: {diag['cells']} cells, {len(valid)} valid, rejected={dict(rejected)}")
    return result, diag


# ---------------------------------------------------------------- floor area
def floor_area_xy(triangles, cell=0.1):
    """Union area (m^2) of 2-D triangles [(ax, ay, bx, by, cx, cy), ...] given in metres.

    Raster on a `cell` grid, so a floor slab whose top and bottom faces overlap is counted once.
    """
    import numpy as np

    if not triangles:
        return 0.0
    t = np.asarray(triangles, dtype=float).reshape(-1, 3, 2)
    lo = t.min(axis=(0, 1))
    occupied = set()
    for tri in t:
        a, b, c = tri
        tmin = np.floor((tri.min(axis=0) - lo) / cell).astype(int)
        tmax = np.floor((tri.max(axis=0) - lo) / cell).astype(int)
        ii, jj = np.meshgrid(np.arange(tmin[0], tmax[0] + 1), np.arange(tmin[1], tmax[1] + 1), indexing="ij")
        px = lo[0] + (ii.ravel() + 0.5) * cell
        py = lo[1] + (jj.ravel() + 0.5) * cell
        v0, v1 = c - a, b - a
        den = v0[0] * v1[1] - v1[0] * v0[1]
        if abs(den) < 1e-12:
            continue
        wx, wy = px - a[0], py - a[1]
        u = (wx * v1[1] - v1[0] * wy) / den
        v = (v0[0] * wy - wx * v0[1]) / den
        inside = (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9)
        occupied.update(zip(ii.ravel()[inside].tolist(), jj.ravel()[inside].tolist()))
    return len(occupied) * cell * cell


# ---------------------------------------------------------------- image statistics
def image_stats(data, stride=97):
    """Mean and standard deviation of every `stride`-th byte of an 8-bit image buffer."""
    sample = bytes(data[::stride]) if not isinstance(data, (bytes, bytearray)) else data[::stride]
    n = len(sample)
    if n == 0:
        return float("nan"), float("nan")
    mean = sum(sample) / n
    var = sum((v - mean) ** 2 for v in sample) / n
    return mean, math.sqrt(var)
