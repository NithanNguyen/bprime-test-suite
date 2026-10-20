#!/usr/bin/env python3
"""
Choose GRScenes-100 home scenes and extract only what they need (runs on the HOST; Python >= 3.8,
standard library only, plus the `unzip` command for the extraction).

  plan     Read every scene's metadata.json inside the merged archive (nothing is extracted).
           Per scene: size of the model folders it references, number of object models and
           categories, and which of the --targets categories it contains.
           -> table on screen + <log-dir>/grscenes_plan.csv
  extract  Extract Materials/, the chosen scenes/<id>_usd/ folders and ONLY the model folders they
           reference, after a free-disk-space check; then verify that every referenced
           instance.usd resolves through the scene's models -> ../../models symlink.

Why: the home archive unpacks to ~111 GB (models/ ~91 GB, Materials/ ~19 GB, scenes/ ~0.3 GB),
while one scene references only 0.6-2.5 GB of models (measured, README section 6).
`unzip` is used instead of Python's zipfile because it restores the symlinks of the dataset.

Examples (via bprime.sh, which adds --log-dir):
  ./bprime.sh grscenes-select plan    --zip home_scenes_full.zip
  ./bprime.sh grscenes-select extract --zip home_scenes_full.zip --dest /data/datasets/GRScenes/home \
                                      --ids MV7J6NIKTKJZ2AABAAAAADQ8 MVUHLWYKTKJ5EAABAAAAAAQ8
"""
import argparse
import csv
import json
import os
import shutil
import subprocess
import sys
import zipfile
from collections import Counter

# Small personal items an older adult may ask the robot to find ("Tìm giúp tôi..."), restricted to
# category names that exist in GRScenes-100 home. A proposal of this suite; change with --targets.
DEFAULT_TARGETS = ("cup,bottle,book,shoe,towel,clock,laptop,pen,keyboard,mouse,telephone,backpack,"
                   "box,basket,toy,plate,bowl,clothes,tray,monitor")
GB = 1e9


def norm_id(s):
    return s[:-4] if s.endswith("_usd") else s


class Archive:
    """Index of the merged GRScenes zip: prefix, model-folder sizes, per-scene metadata."""

    def __init__(self, path):
        self.path = path
        self.zf = zipfile.ZipFile(path)
        self.infos = self.zf.infolist()
        self.prefix = None
        self.meta = {}                       # scene id -> zip name of its metadata.json
        for i in self.infos:
            p = i.filename.split("/")
            if len(p) == 4 and p[1] == "scenes" and p[2].endswith("_usd") and p[3] == "metadata.json":
                self.prefix = self.prefix or p[0]
                self.meta[norm_id(p[2])] = i.filename
        if not self.meta:
            sys.exit(f"ERROR: no <prefix>/scenes/<id>_usd/metadata.json in {path} (not the GRScenes home zip?)")
        self.model_size = Counter()          # 'models/<type>/<group>/<category>/<hash>' -> bytes
        self.top_size = Counter()            # 'Materials' | 'models' | 'scenes' -> bytes
        for i in self.infos:
            p = i.filename.split("/")
            if p[0] != self.prefix or len(p) < 2:
                continue
            self.top_size[p[1]] += i.file_size
            if p[1] == "models" and len(p) > 6:
                self.model_size["/".join(p[1:6])] += i.file_size

    def scene(self, sid):
        """Model folders referenced by a scene and its object-category counts."""
        md = json.loads(self.zf.read(self.meta[sid]).decode("utf-8"))
        models = [m[2:] if m.startswith("./") else m for m in md.get("models", [])]
        dirs = {"/".join(m.split("/")[:5]) for m in models}
        cats = Counter(d.split("/")[3] for d in dirs if d.split("/")[1] == "object")
        missing = sorted(d for d in dirs if d not in self.model_size)
        return {"id": sid, "dirs": dirs, "models": models, "cats": cats, "missing": missing,
                "bytes": sum(self.model_size[d] for d in dirs)}


def cmd_plan(a, arc):
    targets = [t.strip() for t in a.targets.split(",") if t.strip()]
    rows, every = [], set()
    for sid in sorted(arc.meta):
        s = arc.scene(sid)
        every |= s["dirs"]
        hits = [(t, s["cats"][t]) for t in targets if s["cats"][t]]
        rows.append({"scene_id": sid, "models_gb": round(s["bytes"] / GB, 2), "n_models": len(s["dirs"]),
                     "n_object_models": sum(s["cats"].values()), "n_categories": len(s["cats"]),
                     "n_targets": len(hits), "targets": " ".join(f"{t}:{n}" for t, n in hits),
                     "missing_dirs": len(s["missing"])})
    if a.sort == "targets":
        rows.sort(key=lambda r: (-r["n_targets"], r["models_gb"]))
    else:
        rows.sort(key=lambda r: r["models_gb"])

    print(f"archive: {arc.path}  prefix={arc.prefix}  scenes={len(rows)}")
    for k in ("models", "Materials", "scenes"):
        print(f"  {k + '/':11s} {arc.top_size[k] / GB:7.2f} GB uncompressed")
    print(f"  model folders referenced by all scenes: {sum(arc.model_size[d] for d in every) / GB:.2f} GB")
    print(f"targets ({len(targets)}): {','.join(targets)}\n")
    print(f"{'scene_id':26s} {'GB':>5s} {'models':>6s} {'objs':>5s} {'cats':>4s} {'tgt':>5s} {'miss':>4s}  targets")
    for r in rows[:a.top]:
        print(f"{r['scene_id']:26s} {r['models_gb']:5.2f} {r['n_models']:6d} {r['n_object_models']:5d} "
              f"{r['n_categories']:4d} {r['n_targets']:2d}/{len(targets):<2d} {r['missing_dirs']:4d}  {r['targets']}")
    if len(rows) > a.top:
        print(f"... {len(rows) - a.top} more rows in the CSV")
    bad = [r["scene_id"] for r in rows if r["missing_dirs"]]
    if bad:
        print(f"WARNING: {len(bad)} scene(s) reference model folders that are not in the archive: {bad[:5]}")
    os.makedirs(a.log_dir, exist_ok=True)
    out = os.path.join(a.log_dir, "grscenes_plan.csv")
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f"\nwrote {out}")


def cmd_extract(a, arc):
    ids = [norm_id(i) for i in a.ids]
    unknown = [i for i in ids if i not in arc.meta]
    if unknown:
        sys.exit(f"ERROR: unknown scene id(s) {unknown}. Use real ids from the 'plan' table "
                 f"(e.g. {sorted(arc.meta)[0]}).")
    scenes = [arc.scene(i) for i in ids]
    dirs = set().union(*(s["dirs"] for s in scenes))
    prefixes = [f"{arc.prefix}/Materials/"] + [f"{arc.prefix}/scenes/{i}_usd/" for i in ids] + \
               [f"{arc.prefix}/{d}/" for d in sorted(dirs)]
    pset = tuple(prefixes)
    members = [i for i in arc.infos if i.filename.startswith(pset)]
    need = sum(i.file_size for i in members if not os.path.lexists(os.path.join(a.dest, i.filename)))
    os.makedirs(a.dest, exist_ok=True)
    free = shutil.disk_usage(a.dest).free
    print(f"{len(ids)} scene(s), {len(dirs)} model folders ({sum(arc.model_size[d] for d in dirs) / GB:.2f} GB), "
          f"Materials/ {arc.top_size['Materials'] / GB:.2f} GB")
    for s in scenes:
        print(f"  {s['id']}: " + ", ".join(f"{c}:{n}" for c, n in s["cats"].most_common(15)))
    print(f"to write: {need / GB:.2f} GB (files already present are skipped); free on {a.dest}: {free / GB:.1f} GB")
    if need + a.margin_gb * GB > free:
        sys.exit(f"ERROR: not enough free space (keep >= {a.margin_gb:.0f} GB margin). Free space or choose fewer scenes.")
    if a.dry_run:
        print("--dry-run: nothing extracted")
        return
    if shutil.which("unzip") is None:
        sys.exit("ERROR: 'unzip' not found (sudo apt install unzip)")
    patterns = [p + "*" for p in prefixes]          # unzip's '*' also matches '/' (no -W)
    for k in range(0, len(patterns), 400):
        cmd = ["unzip", "-q", "-n", os.path.abspath(a.zip)] + patterns[k:k + 400]
        r = subprocess.run(cmd, cwd=a.dest)
        if r.returncode not in (0, 1):               # 1 = warnings only (Info-ZIP exit codes)
            sys.exit(f"ERROR: unzip exited with code {r.returncode}")
    root = os.path.join(a.dest, arc.prefix)
    bad = []
    for s in scenes:
        sdir = os.path.join(root, "scenes", f"{s['id']}_usd")
        for m in s["models"]:
            if not os.path.isfile(os.path.join(sdir, m)):
                bad.append(f"{s['id']}: {m}")
    if not os.path.isdir(os.path.join(root, "Materials")):
        bad.append("Materials/ missing")
    if bad:
        print(f"VERIFY FAIL: {len(bad)} referenced file(s) do not resolve, e.g. {bad[:3]}")
        sys.exit(1)
    print(f"VERIFY PASS: every model referenced by the {len(ids)} scene(s) resolves through the scene symlinks")
    print(f"Set before running T7 or grscenes-inventory:  export GRSCENES_ROOT={root}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--log-dir", default=os.path.expanduser("~/bprime_logs"))
    sub = ap.add_subparsers(dest="cmd")
    p = sub.add_parser("plan", help="per-scene size and target categories (reads the zip only)")
    p.add_argument("--zip", required=True, help="merged archive home_scenes_full.zip")
    p.add_argument("--targets", default=DEFAULT_TARGETS, help="comma list of object categories")
    p.add_argument("--sort", choices=["targets", "size"], default="targets")
    p.add_argument("--top", type=int, default=25, help="rows shown on screen (CSV has all)")
    e = sub.add_parser("extract", help="extract Materials/ + chosen scenes + the models they reference")
    e.add_argument("--zip", required=True)
    e.add_argument("--dest", required=True, help="folder that will contain <prefix>/ (e.g. home_scenes/)")
    e.add_argument("--ids", nargs="+", required=True, help="scene ids from the plan table")
    e.add_argument("--margin-gb", type=float, default=10.0, help="free space to keep after extraction")
    e.add_argument("--dry-run", action="store_true", help="only report sizes and the space check")
    a = ap.parse_args()
    if a.cmd is None:
        ap.print_help()
        sys.exit(1)
    arc = Archive(a.zip)
    {"plan": cmd_plan, "extract": cmd_extract}[a.cmd](a, arc)


if __name__ == "__main__":
    main()
