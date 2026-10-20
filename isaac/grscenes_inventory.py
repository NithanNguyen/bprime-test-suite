"""
Inventory of GRScenes-100 home scenes, to choose which apartments to use for T7 and the thesis
experiments. Runs inside the Isaac Sim 4.1 container (only for its USD libraries; nothing is simulated):

  ./bprime.sh grscenes-inventory [--ids ID1 ID2 ...] [--variant raw|navigation]

Per scene it reports: metersPerUnit, upAxis, number of object models, number of categories,
an estimate of the walkable floor area (union of horizontal faces of the 'ground'/'floor' meshes,
rasterised on a 0.1 m grid) and the floor bounding box. Category names are read from the prim
paths /Root/Meshes/<scope>/<category>/<model>, the layout used by InternUtopia's
toolkits/grscenes_scripts/preprocess.py. The floor area needs the scene's 'ground' model folders
to be extracted (grscenes-select extract does that); otherwise it is reported as nan.

Outputs (host folder ~/bprime_logs):
  grscenes_inventory.csv     one row per scene
  grscenes_categories.csv    per category: in how many scenes it appears, total instances
"""
import argparse
import csv
import glob
import os
import sys
import time

parser = argparse.ArgumentParser()
parser.add_argument("--root", default="/grscenes/scenes", help="folder that holds the <id>_usd scene folders")
parser.add_argument("--variant", default="raw", choices=["raw", "navigation", "interaction"])
parser.add_argument("--ids", nargs="*", help="only these scene ids")
parser.add_argument("--floor-categories", default="ground,floor", help="comma list of floor category names")
parser.add_argument("--out", default="/bprime_logs")
args, _ = parser.parse_known_args()

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "common"))
from bprime_app import start_app  # noqa: E402
from t7_lib import floor_area_xy  # noqa: E402

app = start_app(livestream=False)

import numpy as np  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402


def log(msg):
    print(f"[inventory] {msg}", flush=True)


FLOOR_CATS = {c.strip() for c in args.floor_categories.split(",") if c.strip()}


def world_matrix(prim):
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return np.array([[m[i][j] for j in range(4)] for i in range(4)])


def horizontal_triangles(prim, mpu, up):
    """2-D triangles (metres) of the faces of `prim`'s meshes whose normal is within ~25 deg of `up`."""
    h = [a for a in range(3) if a != up]
    tris = []
    it = Usd.PrimRange(prim, Usd.TraverseInstanceProxies(Usd.PrimDefaultPredicate))
    for p in it:
        if not p.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(p)
        pts = mesh.GetPointsAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        idx = mesh.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts or not idx:
            continue
        M = world_matrix(p)
        P = (np.asarray(pts, dtype=float) @ M[:3, :3] + M[3, :3]) * mpu   # USD row-vector convention
        idx = np.asarray(idx)
        k = 0
        for c in counts:
            face = idx[k:k + c]
            k += c
            for t in range(1, c - 1):
                a, b, d = P[face[0]], P[face[t]], P[face[t + 1]]
                n = np.cross(b - a, d - a)
                nn = np.linalg.norm(n)
                if nn > 1e-12 and abs(n[up]) / nn > 0.9:
                    tris.append((a[h[0]], a[h[1]], b[h[0]], b[h[1]], d[h[0]], d[h[1]]))
    return tris


def scene_dirs():
    dirs = sorted(d for d in glob.glob(os.path.join(args.root, "*_usd")) if os.path.isdir(d))
    if args.ids:
        wanted = {i[:-4] if i.endswith("_usd") else i for i in args.ids}
        dirs = [d for d in dirs if os.path.basename(d)[:-4] in wanted]
        unknown = sorted(wanted - {os.path.basename(d)[:-4] for d in dirs})
        if unknown:
            log(f"WARNING: no folder <id>_usd under {args.root} for id(s) {unknown} "
                "(placeholder or typo? use the ids printed by ./bprime.sh grscenes-select plan)")
    return dirs


rows, cat_scenes, cat_total = [], {}, {}
dirs = scene_dirs()
log(f"{len(dirs)} scene folders under {args.root} (variant={args.variant})")
for d in dirs:
    sid = os.path.basename(d)[:-4]
    path = os.path.join(d, f"start_result_{args.variant}.usd")
    if not os.path.isfile(path):
        log(f"{sid}: {os.path.basename(path)} missing, skipped")
        continue
    t = time.time()
    try:
        stage = Usd.Stage.Open(path, Usd.Stage.LoadNone)
        mpu = UsdGeom.GetStageMetersPerUnit(stage) or 1.0
        up_axis = str(UsdGeom.GetStageUpAxis(stage))
        up = 2 if up_axis == "Z" else 1
        meshes = stage.GetPrimAtPath("/Root/Meshes")
        cats, floor_prims = {}, []
        if meshes and meshes.IsValid():
            for scope in meshes.GetChildren():
                for cat in scope.GetChildren():
                    models = [m for m in cat.GetChildren()]
                    cats[cat.GetName()] = cats.get(cat.GetName(), 0) + len(models)
                    if cat.GetName() in FLOOR_CATS:
                        floor_prims.append(cat)
        area, dims = float("nan"), ("", "")
        if floor_prims:
            tris = []
            for fp in floor_prims:
                stage.Load(fp.GetPath())
                tris += horizontal_triangles(fp, mpu, up)
            if tris:
                area = floor_area_xy(tris, cell=0.1)
                arr = np.asarray(tris).reshape(-1, 2)
                ext = arr.max(axis=0) - arr.min(axis=0)
                dims = (f"{ext[0]:.1f}", f"{ext[1]:.1f}")
        n_models = sum(cats.values())
        rows.append({"scene_id": sid, "meters_per_unit": mpu, "up_axis": up_axis, "n_models": n_models,
                     "n_categories": len(cats), "floor_area_m2": f"{area:.1f}", "floor_bbox_x_m": dims[0],
                     "floor_bbox_y_m": dims[1],
                     "categories": ";".join(f"{c}:{n}" for c, n in sorted(cats.items(), key=lambda kv: -kv[1]))})
        for c, nmod in cats.items():
            cat_scenes[c] = cat_scenes.get(c, 0) + 1
            cat_total[c] = cat_total.get(c, 0) + nmod
        log(f"{sid}: mpu={mpu} up={up_axis} models={n_models} categories={len(cats)} "
            f"floor~{area:.1f} m2 bbox={dims[0]}x{dims[1]} m ({time.time() - t:.1f} s)")
    except Exception as e:  # noqa: BLE001
        log(f"{sid}: ERROR {type(e).__name__}: {e}")

os.makedirs(args.out, exist_ok=True)
out1 = os.path.join(args.out, "grscenes_inventory.csv")
with open(out1, "w", newline="") as f:
    fields = ["scene_id", "meters_per_unit", "up_axis", "n_models", "n_categories", "floor_area_m2",
              "floor_bbox_x_m", "floor_bbox_y_m", "categories"]
    w = csv.DictWriter(f, fieldnames=fields)
    w.writeheader()
    w.writerows(rows)
out2 = os.path.join(args.out, "grscenes_categories.csv")
with open(out2, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["category", "scenes_with_category", "total_instances"])
    for c in sorted(cat_scenes, key=lambda c: (-cat_scenes[c], -cat_total[c])):
        w.writerow([c, cat_scenes[c], cat_total[c]])
log(f"wrote {out1} ({len(rows)} scenes) and {out2} ({len(cat_scenes)} categories)")
app.close()
