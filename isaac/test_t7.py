"""
B' test T7 for Isaac Sim 4.1 (headless, ROS 2 Foxy internal libraries):
the T6 robot setup (Jetbot + RTX 2D lidar + /odom, /tf) plus an RGB camera, inside a
multi-room scene loaded from USD (GRScenes-100 home scene, or the Isaac Sim Office).

Started from the host with:
  ./bprime.sh isaac t7 --scene office                 # Isaac Sim 4.1 Office (smoke test)
  ./bprime.sh isaac t7 --scene <GRSCENES_SCENE_ID>     # GRScenes-100 home scene (needs GRSCENES_ROOT)
Then, in a second terminal:  ./bprime.sh foxy t7

Isaac-side sub-tests (each prints a [B'] line):
  BACKEND  Foxy backend loaded, Humble not loaded (same check as T6)
  T7a LOAD       stage opened, loading finished; load time, metersPerUnit, upAxis, prim/mesh counts
  T7f SEMANTICS  labelled objects found; ground-truth list written to /bprime_logs/t7_objects_<scene>.json
  SPAWN          free indoor floor cell found with PhysX raycasts (or --spawn X Y)
  ROBOT / LIDAR / CAMERA / ODOM_TF / DRIVE   as in T6
  T7b STABLE     robot rests on the scene floor (does not fall through / sink) 2-5 s after start
  T7e RTF        simulated seconds per wall-clock second over the run
T7c/T7g/T7h (scan, camera, motion in the scene) run on the Foxy side; T7d (VRAM) and the
error count of T7a are computed by bprime.sh on the host after the run.
"""

import argparse
import json
import os
import re
import sys
import time

parser = argparse.ArgumentParser()
parser.add_argument("--scene", default="office",
                    help="'office' | GRScenes scene id (folder <id>_usd under $GRSCENES_ROOT/scenes) | USD path/URL")
parser.add_argument("--variant", default="navigation", choices=["navigation", "raw", "interaction"],
                    help="GRScenes file start_result_<variant>.usd (default navigation)")
parser.add_argument("--duration", type=float, default=600.0, help="wall-clock seconds to run after play")
parser.add_argument("--lidar-config", default="RPLIDAR_S2E", help="RTX lidar config name (as in T6)")
parser.add_argument("--pointcloud", action="store_true", help="also publish /point_cloud (off by default in T7)")
parser.add_argument("--no-camera", action="store_true", help="do not create the RGB camera")
parser.add_argument("--cam-res", default="640x480", help="camera render resolution WxH (default 640x480)")
parser.add_argument("--cam-hfov", type=float, default=70.0, help="camera horizontal field of view [deg]")
parser.add_argument("--spawn", type=float, nargs=2, metavar=("X", "Y"), help="robot position in metres (skip search)")
parser.add_argument("--floor-z", type=float, help="floor height in metres (with --spawn or --ground-plane)")
parser.add_argument("--hide-hdr", action="store_true",
                    help="deactivate /Root/__default_setting/HDR_Sphere (GRScenes DayMaterial.mdl workaround)")
parser.add_argument("--add-dome-light", action="store_true", help="add a dome light (if the scene renders dark)")
parser.add_argument("--add-colliders", action="store_true",
                    help="add static triangle-mesh colliders to every scene mesh that has none")
parser.add_argument("--ground-plane", action="store_true", help="add a ground plane at --floor-z (default 0)")
parser.add_argument("--search-region", type=float, nargs=4, metavar=("XMIN", "YMIN", "XMAX", "YMAX"),
                    help="spawn search area in metres (default: bbox of the meshes no larger than --max-object-size)")
parser.add_argument("--max-object-size", type=float, default=50.0,
                    help="meshes wider than this [m] (backdrops, terrain, sky) are left out of the search bbox")
parser.add_argument("--max-cells", type=int, default=300000,
                    help="refuse a spawn search with more 0.5 m cells than this (use --search-region)")
parser.add_argument("--livestream", action="store_true", help="WebRTC livestream on port 8211 (viewing only)")
args, _unknown = parser.parse_known_args()

TAG = "[B']"
WHEEL_RADIUS, WHEEL_BASE = 0.03, 0.1125      # Jetbot values from the Isaac Sim core-API tutorial (as T6)
LIDAR_OFFSET_M = (0.0, 0.0, 0.15)            # lidar relative to the chassis prim [m] (as T6)
CAM_OFFSET_M = (0.06, 0.0, 0.12)             # camera relative to the chassis prim [m]
# USD cameras look along -Z with +Y up. This quaternion (w, x, y, z) turns them to look along the
# chassis +X with +Z up: camera -Z -> +X, camera +Y -> +Z (checked numerically, see README T7).
CAM_ORIENT = (0.5, 0.5, -0.5, -0.5)
GRSCENES_MOUNT = "/grscenes"
HDR_PRIM = "/Root/__default_setting/HDR_Sphere"   # from InternUtopia issue #30


def log(msg):
    print(f"{TAG} {msg}", flush=True)


log(f"python={sys.version.split()[0]} scene={args.scene} variant={args.variant}")
for k in ("ROS_DISTRO", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "FASTRTPS_DEFAULT_PROFILES_FILE", "MDL_SYSTEM_PATH"):
    log(f"env {k}={os.environ.get(k, '<unset>')}")

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "common"))
from bprime_app import start_app  # noqa: E402
from t7_lib import find_spawn  # noqa: E402

simulation_app = start_app(livestream=args.livestream, log=log)

try:  # GRScenes' own play_scene.py/warmup.py turn this off before opening a scene
    import omni.physx.bindings._physx as physx_bindings
    simulation_app.set_setting(physx_bindings.SETTING_UJITSO_COLLISION_COOKING, False)
    log("setting: UJITSO collision cooking disabled")
except Exception as e:  # noqa: BLE001
    log(f"setting: UJITSO collision cooking left unchanged ({type(e).__name__}: {e})")

from omni.isaac.core.utils.extensions import enable_extension  # noqa: E402

for ext in ("omni.isaac.ros2_bridge", "omni.isaac.sensor", "omni.isaac.wheeled_robots"):
    try:
        enable_extension(ext)
    except Exception as e:  # noqa: BLE001
        log(f"WARNING: enable_extension({ext}) failed: {type(e).__name__}: {e}")
for _ in range(10):
    simulation_app.update()


def backend_check(stage_name):
    libs = set()
    try:
        with open("/proc/self/maps", "r") as f:
            for line in f:
                parts = line.split()
                if len(parts) >= 6 and "ros2_bridge" in parts[-1]:
                    libs.add(parts[-1])
    except OSError as e:
        log(f"cannot read /proc/self/maps: {e}")
    foxy = any("foxy" in p for p in libs)
    humble = any("humble" in p for p in libs)
    if foxy and not humble:
        log(f"BACKEND PASS ({stage_name}): Foxy bridge backend loaded, Humble not loaded")
    elif humble:
        log(f"BACKEND FAIL ({stage_name}): Humble backend loaded -> results INVALID for B'")
    else:
        log(f"BACKEND UNKNOWN ({stage_name}): no foxy/humble bridge library in memory maps")


backend_check("after-enable")

import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.kit.commands  # noqa: E402
import omni.usd  # noqa: E402
import usdrt.Sdf  # noqa: E402
from omni.isaac.core.utils.nucleus import get_assets_root_path  # noqa: E402
from omni.isaac.core.utils.stage import is_stage_loading, open_stage  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics  # noqa: E402


def finish(code=0):
    backend_check("end-of-run")
    log("done")
    simulation_app.close()
    sys.exit(code)


# ---------------------------------------------------------------- resolve the scene
def resolve_scene(s):
    if s.lower() == "office":
        root = get_assets_root_path()
        if not root:
            raise RuntimeError("get_assets_root_path() returned None (asset server unreachable)")
        return root + "/Isaac/Environments/Office/office.usd", "office"
    if s.endswith((".usd", ".usda", ".usdc", ".usdz")) or "://" in s:
        return s, os.path.splitext(os.path.basename(s.rstrip("/")))[0]
    sid = s[:-4] if s.endswith("_usd") else s
    path = f"{GRSCENES_MOUNT}/scenes/{sid}_usd/start_result_{args.variant}.usd"
    if not os.path.isdir(GRSCENES_MOUNT):
        raise RuntimeError(f"{GRSCENES_MOUNT} is not mounted: set GRSCENES_ROOT in config/bprime.env")
    if not os.path.isfile(path):
        raise RuntimeError(f"{path} not found (scene id or --variant wrong?)")
    return path, f"{sid}_{args.variant}"


try:
    scene_path, scene_name = resolve_scene(args.scene)
except Exception as e:  # noqa: BLE001
    log(f"T7a LOAD FAIL: {e}")
    finish(1)
log(f"scene file: {scene_path}")

# ---------------------------------------------------------------- T7a LOAD
t_load = time.time()
try:
    ok = open_stage(scene_path)
except Exception as e:  # noqa: BLE001
    ok = False
    log(f"open_stage raised {type(e).__name__}: {e}")
if not ok:
    log("T7a LOAD FAIL: open_stage returned False")
    finish(1)
stage = omni.usd.get_context().get_stage()
if args.hide_hdr:
    hdr = stage.GetPrimAtPath(HDR_PRIM)
    if hdr and hdr.IsValid():
        hdr.SetActive(False)
        log(f"--hide-hdr: {HDR_PRIM} deactivated (session only, file unchanged)")
    else:
        log(f"--hide-hdr: {HDR_PRIM} not found, nothing to deactivate")
next_beat = time.time() + 30.0
while is_stage_loading():
    simulation_app.update()
    if time.time() >= next_beat:
        log(f"loading... {time.time() - t_load:.0f} s")
        next_beat += 30.0
for _ in range(60):          # first frames compile shaders / MDL materials
    simulation_app.update()
load_s = time.time() - t_load

mpu = UsdGeom.GetStageMetersPerUnit(stage) or 1.0
up_axis = str(UsdGeom.GetStageUpAxis(stage))
S = 1.0 / mpu                # metres -> stage units
n_prims = n_meshes = 0
for p in stage.Traverse():
    n_prims += 1
    if p.IsA(UsdGeom.Mesh):
        n_meshes += 1
verdict = "PASS" if (abs(mpu - 1.0) < 1e-6 and up_axis == "Z") else "WARN"
log(f"T7a LOAD {verdict}: {load_s:.1f} s, metersPerUnit={mpu}, upAxis={up_axis}, prims={n_prims}, meshes={n_meshes}")
if verdict == "WARN":
    log("     WARN: the robot asset is authored in metres with Z up; scaling/orientation in this scene is unverified")
if args.add_dome_light:
    light = UsdLux.DomeLight.Define(stage, "/World/T7DomeLight")
    light.CreateIntensityAttr(1000.0)
    log("--add-dome-light: /World/T7DomeLight (intensity 1000) added")

# ---------------------------------------------------------------- T7f SEMANTICS + ground truth
SEM_RE = re.compile(r"^semantic:(.+):params:semanticData$")


def semantic_labels(prim):
    out = []
    for a in prim.GetAttributes():
        m = SEM_RE.match(a.GetName())
        if m:
            v = a.Get()
            if v:
                out.append(str(v))
    return out


bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                               useExtentsHint=True)
objects = []
for p in stage.Traverse():
    labels = semantic_labels(p)
    if not labels:
        continue
    parent = p.GetParent()
    if parent and parent.IsValid() and set(labels) <= set(semantic_labels(parent)):
        continue             # keep the top-most prim that carries the label
    rng = bbox_cache.ComputeWorldBound(p).ComputeAlignedRange()
    if rng.IsEmpty():
        continue
    lo, hi = np.array(rng.GetMin()) * mpu, np.array(rng.GetMax()) * mpu
    objects.append({"label": labels[0], "category": labels[0].split("/")[0], "prim": str(p.GetPath()),
                    "center_m": [round(float(v), 3) for v in (lo + hi) / 2],
                    "size_m": [round(float(v), 3) for v in hi - lo]})
cats = {}
for o in objects:
    cats[o["category"]] = cats.get(o["category"], 0) + 1
gt_file = f"/bprime_logs/t7_objects_{re.sub(r'[^A-Za-z0-9_.-]', '_', scene_name)}.json"
try:
    with open(gt_file, "w") as f:
        json.dump({"scene": scene_path, "meters_per_unit": mpu, "up_axis": up_axis,
                   "categories": dict(sorted(cats.items(), key=lambda kv: -kv[1])), "objects": objects}, f, indent=1)
    written = gt_file
except OSError as e:
    written = f"(not written: {e})"
if objects:
    top = ", ".join(f"{c}:{n}" for c, n in sorted(cats.items(), key=lambda kv: -kv[1])[:40])
    log(f"T7f SEMANTICS PASS: {len(objects)} labelled objects in {len(cats)} categories; ground truth -> {written}")
    log(f"     categories (top 40): {top}")
else:
    log("T7f SEMANTICS N/A: no prim carries a semantic label in this scene")

# ---------------------------------------------------------------- physics world
from omni.isaac.core import World  # noqa: E402

phys_scenes = [str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
phys_path = phys_scenes[0] if phys_scenes else "/physicsScene"
log(f"physics scene(s) in file: {phys_scenes or 'none'} -> using {phys_path}")
world = World(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0, stage_units_in_meters=mpu, physics_prim_path=phys_path)

scene_rng = bbox_cache.ComputeWorldBound(stage.GetPseudoRoot()).ComputeAlignedRange()
bb_min, bb_max = tuple(scene_rng.GetMin()), tuple(scene_rng.GetMax())
log("scene bbox [m]: min=({:.2f}, {:.2f}, {:.2f}) max=({:.2f}, {:.2f}, {:.2f})".format(
    *[v * mpu for v in bb_min + bb_max]))

# Search bbox: union of the meshes no wider than --max-object-size. One backdrop, terrain or sky mesh
# can make the whole-scene bbox hundreds of metres wide and put its bottom far below the floor
# (seen with Office: x span ~1 km, zmin -86 m), which breaks the floor probe of the spawn search.
core_lo, core_hi, big = None, None, []
for p in stage.Traverse():
    if not p.IsA(UsdGeom.Mesh):
        continue
    r = bbox_cache.ComputeWorldBound(p).ComputeAlignedRange()
    if r.IsEmpty():
        continue
    lo, hi = np.array(r.GetMin(), dtype=float), np.array(r.GetMax(), dtype=float)
    span = float(max(hi[0] - lo[0], hi[1] - lo[1])) * mpu
    if span > args.max_object_size:
        big.append((span, str(p.GetPath()), lo * mpu, hi * mpu))
        continue
    core_lo = lo if core_lo is None else np.minimum(core_lo, lo)
    core_hi = hi if core_hi is None else np.maximum(core_hi, hi)
big.sort(reverse=True)
for span, path, lo, hi in big[:5]:
    log(f"     left out of search bbox (wider than {args.max_object_size:.0f} m): {path} span={span:.1f} m "
        f"z=[{lo[2]:.2f}, {hi[2]:.2f}]")
if len(big) > 5:
    log(f"     ... and {len(big) - 5} more")
if core_lo is not None:
    bb_min, bb_max = tuple(float(v) for v in core_lo), tuple(float(v) for v in core_hi)
if args.search_region:
    x0, y0, x1, y1 = (v * S for v in args.search_region)
    bb_min = (min(x0, x1), min(y0, y1), bb_min[2])
    bb_max = (max(x0, x1), max(y0, y1), bb_max[2])
log("search bbox [m]: min=({:.2f}, {:.2f}, {:.2f}) max=({:.2f}, {:.2f}, {:.2f}){}".format(
    *[v * mpu for v in bb_min + bb_max], " (--search-region)" if args.search_region else ""))

pre_coll = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
for p in pre_coll[:10]:
    approx = "-"
    if p.HasAPI(UsdPhysics.MeshCollisionAPI):
        a = UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr().Get()
        approx = str(a) if a else "-"
    r = bbox_cache.ComputeWorldBound(p).ComputeAlignedRange()
    zr = "" if r.IsEmpty() else f" z=[{r.GetMin()[2] * mpu:.2f}, {r.GetMax()[2] * mpu:.2f}] m"
    log(f"     collider in file: {p.GetPath()} type={p.GetTypeName()} approximation={approx}{zr}")
if args.add_colliders:
    added = 0
    for p in stage.Traverse():
        if p.IsA(UsdGeom.Mesh) and not p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(p)
            UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr().Set("none")   # static triangle mesh
            added += 1
    log(f"--add-colliders: static triangle-mesh colliders added to {added} meshes")
if args.ground_plane:
    gz = (args.floor_z or 0.0) * S
    world.scene.add_default_ground_plane(z_position=gz)
    log(f"--ground-plane: ground plane at z={gz * mpu:.3f} m")

n_coll = sum(1 for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI))
log(f"prims with a collider: {n_coll}")

world.reset()
for _ in range(10):
    world.step(render=False)

# ---------------------------------------------------------------- SPAWN
from omni.physx import get_physx_scene_query_interface  # noqa: E402

sq = get_physx_scene_query_interface()
_both_sides = [True]


def raycast(o, d, dist):
    origin, direction = carb.Float3(*[float(v) for v in o]), carb.Float3(*[float(v) for v in d])
    if _both_sides[0]:
        try:
            hit = sq.raycast_closest(origin, direction, float(dist), True)   # bothSides: rays from inside a mesh
        except TypeError:
            _both_sides[0] = False
            log("raycast_closest has no bothSides argument in this build; using front faces only")
            hit = sq.raycast_closest(origin, direction, float(dist))
    else:
        hit = sq.raycast_closest(origin, direction, float(dist))
    if hit and hit.get("hit"):
        return float(hit["distance"])
    return None


spawn = None
try:
    if args.spawn:
        x, y = args.spawn[0] * S, args.spawn[1] * S
        if args.floor_z is not None:
            fz = args.floor_z * S
        else:
            d = raycast((x, y, bb_min[2] + 1.0 * S), (0.0, 0.0, -1.0), 2.0 * S)
            fz = (bb_min[2] + 1.0 * S - d) if d is not None else 0.0
        spawn = {"x": x, "y": y, "floor_z": fz, "neighbours": "n/a", "clearance": float("nan")}
        log("SPAWN (given): x={:.2f} y={:.2f} floor_z={:.3f} m".format(x * mpu, y * mpu, fz * mpu))
    else:
        n_cells = int((bb_max[0] - bb_min[0]) * mpu / 0.5 + 1) * int((bb_max[1] - bb_min[1]) * mpu / 0.5 + 1)
        if n_cells > args.max_cells:
            raise RuntimeError(f"search area has ~{n_cells} cells (> --max-cells {args.max_cells}); "
                               "narrow it with --search-region XMIN YMIN XMAX YMAX")
        fz_hint = args.floor_z * S if args.floor_z is not None else None
        spawn, diag = find_spawn(raycast, bb_min, bb_max, m2u=S, floor_z=fz_hint, log=log)
        if spawn is None:
            log(f"SPAWN FAIL: {diag.get('reason')} {diag}")
            log("     give a position with --spawn X Y [--floor-z Z], or add --add-colliders / --ground-plane")
        else:
            log("SPAWN PASS: x={:.2f} y={:.2f} floor_z={:.3f} m, free neighbours={}/8, clearance={:.2f} m".format(
                spawn["x"] * mpu, spawn["y"] * mpu, spawn["floor_z"] * mpu, spawn["neighbours"],
                spawn["clearance"] * mpu))
except Exception as e:  # noqa: BLE001
    log(f"SPAWN FAIL: {type(e).__name__}: {e}")
    spawn = None

keys = og.Controller.Keys
GRAPH = "/BPrimeT7"


def dump_attrs(node_path):
    try:
        node = og.get_node_by_path(node_path)
        if node is None or not node.is_valid():
            log(f"ATTRS {node_path}: node does not exist")
            return
        names = sorted(a.get_name() for a in node.get_attributes())
        log(f"ATTRS {node_path} [{node.get_type_name()}]: {', '.join(names)}")
    except Exception as e:  # noqa: BLE001
        log(f"ATTRS {node_path}: cannot inspect ({type(e).__name__}: {e})")


def graph_edit(label, spec, nodes_to_dump):
    try:
        og.Controller.edit(GRAPH, spec)
        return True
    except Exception as e:  # noqa: BLE001
        log(f"{label} FAIL(graph): {type(e).__name__}: {e}")
        for n in nodes_to_dump:
            dump_attrs(f"{GRAPH}/{n}")
        return False


# ---------------------------------------------------------------- ROBOT
robot = controller = chassis_path = None
ROBOT_PATH = "/World/Jetbot"
if spawn is not None:
    try:
        from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
        from omni.isaac.wheeled_robots.robots import WheeledRobot

        assets_root = get_assets_root_path()
        robot = world.scene.add(WheeledRobot(
            prim_path=ROBOT_PATH, name="jetbot",
            wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
            create_robot=True, usd_path=assets_root + "/Isaac/Robots/Jetbot/jetbot.usd",
            position=np.array([spawn["x"], spawn["y"], spawn["floor_z"] + 0.05 * S]),
        ))
        controller = DifferentialController(name="bprime_diff", wheel_radius=WHEEL_RADIUS, wheel_base=WHEEL_BASE)
        for _ in range(5):
            simulation_app.update()
        root_prim = stage.GetPrimAtPath(ROBOT_PATH)
        cands = [p for p in Usd.PrimRange(root_prim) if p.GetName().lower() in ("chassis", "base_link", "chassis_link")]
        chassis_path = str(cands[0].GetPath()) if cands else ROBOT_PATH
        log(f"ROBOT PASS: Jetbot at {ROBOT_PATH}; chassis prim = {chassis_path}")
    except Exception as e:  # noqa: BLE001
        log(f"ROBOT FAIL: {type(e).__name__}: {e}")
        robot = None
else:
    log("ROBOT SKIPPED (no spawn). The scene keeps running so you can inspect it with --livestream.")
    log("     without a robot there is no /scan, /odom, /tf or camera: ./bprime.sh foxy t7 will only wait."
        " Stop both sides (Ctrl+C) and fix SPAWN first.")

# ---------------------------------------------------------------- BASE graph
base_ok = False
try:
    og.Controller.edit(
        {"graph_path": GRAPH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                ("SubTwist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("OnPlaybackTick.outputs:tick", "SubTwist.inputs:execIn"),
                ("Context.outputs:context", "SubTwist.inputs:context"),
            ],
            keys.SET_VALUES: [
                ("Context.inputs:useDomainIDEnvVar", True),
                ("PublishClock.inputs:topicName", "clock"),
                ("SubTwist.inputs:topicName", "cmd_vel"),
            ],
        },
    )
    base_ok = True
    log("BASE PASS: /clock publisher and /cmd_vel subscriber created")
except Exception as e:  # noqa: BLE001
    log(f"BASE FAIL: {type(e).__name__}: {e}")

ready = base_ok and chassis_path is not None

# ---------------------------------------------------------------- LIDAR (same as T6)
if ready:
    try:
        sensor, used = None, None
        for cfg in (args.lidar_config, "Example_Rotary"):
            try:
                ok, sensor = omni.kit.commands.execute(
                    "IsaacSensorCreateRtxLidar", path="/lidar", parent=chassis_path, config=cfg,
                    translation=tuple(v * S for v in LIDAR_OFFSET_M), orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0))
                if ok and sensor is not None and sensor.IsValid():
                    used = cfg
                    break
                log(f"LIDAR config '{cfg}' not accepted (ok={ok})")
            except Exception as e:  # noqa: BLE001
                log(f"LIDAR config '{cfg}' failed: {type(e).__name__}: {e}")
        if used is None:
            raise RuntimeError("no lidar config could be created")
        lidar_path = str(sensor.GetPath())
        log(f"LIDAR created: prim={lidar_path} config='{used}'")
        create = [("LidarRP", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                  ("ScanHelper", "omni.isaac.ros2_bridge.ROS2RtxLidarHelper")]
        connect = [(f"{GRAPH}/OnPlaybackTick.outputs:tick", "LidarRP.inputs:execIn"),
                   ("LidarRP.outputs:execOut", "ScanHelper.inputs:execIn"),
                   ("LidarRP.outputs:renderProductPath", "ScanHelper.inputs:renderProductPath"),
                   (f"{GRAPH}/Context.outputs:context", "ScanHelper.inputs:context")]
        values = [("LidarRP.inputs:cameraPrim", [usdrt.Sdf.Path(lidar_path)]),
                  ("ScanHelper.inputs:type", "laser_scan"),
                  ("ScanHelper.inputs:topicName", "scan"),
                  ("ScanHelper.inputs:frameId", "lidar_link")]
        if args.pointcloud:
            create.append(("PcHelper", "omni.isaac.ros2_bridge.ROS2RtxLidarHelper"))
            connect += [("LidarRP.outputs:execOut", "PcHelper.inputs:execIn"),
                        ("LidarRP.outputs:renderProductPath", "PcHelper.inputs:renderProductPath"),
                        (f"{GRAPH}/Context.outputs:context", "PcHelper.inputs:context")]
            values += [("PcHelper.inputs:type", "point_cloud"), ("PcHelper.inputs:topicName", "point_cloud"),
                       ("PcHelper.inputs:frameId", "lidar_link"), ("PcHelper.inputs:fullScan", True)]
        if graph_edit("LIDAR", {keys.CREATE_NODES: create, keys.CONNECT: connect, keys.SET_VALUES: values},
                      ["LidarRP", "ScanHelper", "PcHelper"]):
            log("LIDAR PASS(created): /scan" + (" and /point_cloud" if args.pointcloud else "") + " helpers created")
    except Exception as e:  # noqa: BLE001
        log(f"LIDAR FAIL: {type(e).__name__}: {e}")

# ---------------------------------------------------------------- CAMERA (RGB + camera_info, as T5)
if ready and not args.no_camera:
    try:
        w_px, h_px = (int(v) for v in args.cam_res.lower().split("x"))
        cam_path = f"{chassis_path}/t7_camera"
        cam = UsdGeom.Camera.Define(stage, cam_path)
        aperture = cam.GetHorizontalApertureAttr().Get() or 20.955
        focal = aperture / (2.0 * np.tan(np.radians(args.cam_hfov) / 2.0))
        cam.GetFocalLengthAttr().Set(float(focal))
        cam.GetVerticalApertureAttr().Set(float(aperture * h_px / w_px))
        cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.05 * S, 100.0 * S))
        xf = UsdGeom.Xformable(cam.GetPrim())
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*[v * S for v in CAM_OFFSET_M]))
        xf.AddOrientOp().Set(Gf.Quatf(*CAM_ORIENT))
        log(f"CAMERA prim {cam_path}: {w_px}x{h_px}, hfov={args.cam_hfov:.0f} deg (focal {focal:.2f} mm)")

        def cam_spec(rp, rgb, info, with_size):
            vals = [(f"{rp}.inputs:cameraPrim", [usdrt.Sdf.Path(cam_path)]),
                    (f"{rgb}.inputs:type", "rgb"), (f"{rgb}.inputs:topicName", "bprime/rgb"),
                    (f"{rgb}.inputs:frameId", "camera_link"),
                    (f"{info}.inputs:type", "camera_info"), (f"{info}.inputs:topicName", "bprime/camera_info"),
                    (f"{info}.inputs:frameId", "camera_link")]
            if with_size:
                vals += [(f"{rp}.inputs:width", w_px), (f"{rp}.inputs:height", h_px)]
            return {
                keys.CREATE_NODES: [(rp, "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                                    (rgb, "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                                    (info, "omni.isaac.ros2_bridge.ROS2CameraHelper")],
                keys.CONNECT: [(f"{GRAPH}/OnPlaybackTick.outputs:tick", f"{rp}.inputs:execIn"),
                               (f"{rp}.outputs:execOut", f"{rgb}.inputs:execIn"),
                               (f"{rp}.outputs:execOut", f"{info}.inputs:execIn"),
                               (f"{rp}.outputs:renderProductPath", f"{rgb}.inputs:renderProductPath"),
                               (f"{rp}.outputs:renderProductPath", f"{info}.inputs:renderProductPath"),
                               (f"{GRAPH}/Context.outputs:context", f"{rgb}.inputs:context"),
                               (f"{GRAPH}/Context.outputs:context", f"{info}.inputs:context")],
                keys.SET_VALUES: vals,
            }

        if graph_edit("CAMERA", cam_spec("CamRP", "CamRgb", "CamInfo", True), ["CamRP"]):
            log("CAMERA PASS(created): /bprime/rgb and /bprime/camera_info")
        elif graph_edit("CAMERA(retry without width/height)", cam_spec("CamRP2", "CamRgb2", "CamInfo2", False),
                        ["CamRP2"]):
            log("CAMERA PASS(created, default render size): /bprime/rgb and /bprime/camera_info")
    except Exception as e:  # noqa: BLE001
        log(f"CAMERA FAIL: {type(e).__name__}: {e}")

# ---------------------------------------------------------------- ODOM + TF (same as T6)
if ready:
    spec = {
        keys.CREATE_NODES: [
            ("ComputeOdom", "omni.isaac.core_nodes.IsaacComputeOdometry"),
            ("PubOdom", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),
            ("TfOdomBase", "omni.isaac.ros2_bridge.ROS2PublishRawTransformTree"),
            ("TfBaseLidar", "omni.isaac.ros2_bridge.ROS2PublishRawTransformTree"),
        ],
        keys.CONNECT: [
            (f"{GRAPH}/OnPlaybackTick.outputs:tick", "ComputeOdom.inputs:execIn"),
            ("ComputeOdom.outputs:execOut", "PubOdom.inputs:execIn"),
            ("ComputeOdom.outputs:execOut", "TfOdomBase.inputs:execIn"),
            (f"{GRAPH}/OnPlaybackTick.outputs:tick", "TfBaseLidar.inputs:execIn"),
            (f"{GRAPH}/Context.outputs:context", "PubOdom.inputs:context"),
            (f"{GRAPH}/Context.outputs:context", "TfOdomBase.inputs:context"),
            (f"{GRAPH}/Context.outputs:context", "TfBaseLidar.inputs:context"),
            (f"{GRAPH}/ReadSimTime.outputs:simulationTime", "PubOdom.inputs:timeStamp"),
            (f"{GRAPH}/ReadSimTime.outputs:simulationTime", "TfOdomBase.inputs:timeStamp"),
            (f"{GRAPH}/ReadSimTime.outputs:simulationTime", "TfBaseLidar.inputs:timeStamp"),
            ("ComputeOdom.outputs:position", "PubOdom.inputs:position"),
            ("ComputeOdom.outputs:orientation", "PubOdom.inputs:orientation"),
            ("ComputeOdom.outputs:linearVelocity", "PubOdom.inputs:linearVelocity"),
            ("ComputeOdom.outputs:angularVelocity", "PubOdom.inputs:angularVelocity"),
            ("ComputeOdom.outputs:position", "TfOdomBase.inputs:translation"),
            ("ComputeOdom.outputs:orientation", "TfOdomBase.inputs:rotation"),
        ],
        keys.SET_VALUES: [
            ("ComputeOdom.inputs:chassisPrim", [usdrt.Sdf.Path(chassis_path)]),
            ("PubOdom.inputs:odomFrameId", "odom"),
            ("PubOdom.inputs:chassisFrameId", "base_link"),
            ("PubOdom.inputs:topicName", "odom"),
            ("TfOdomBase.inputs:parentFrameId", "odom"),
            ("TfOdomBase.inputs:childFrameId", "base_link"),
            ("TfBaseLidar.inputs:parentFrameId", "base_link"),
            ("TfBaseLidar.inputs:childFrameId", "lidar_link"),
            ("TfBaseLidar.inputs:translation", list(LIDAR_OFFSET_M)),
        ],
    }
    if graph_edit("ODOM_TF", spec, ["ComputeOdom", "PubOdom", "TfOdomBase", "TfBaseLidar"]):
        log("ODOM_TF PASS(created): /odom and /tf (odom->base_link, base_link->lidar_link)")

# ---------------------------------------------------------------- run
world.reset()
if robot is not None:
    try:
        robot.initialize()
    except Exception:  # noqa: BLE001
        pass
world.play()
log(f"simulation playing for {args.duration:.0f} s wall time. Now run on the host: ./bprime.sh foxy t7")

lin_attr = og.Controller.attribute(f"{GRAPH}/SubTwist.outputs:linearVelocity") if base_ok else None
ang_attr = og.Controller.attribute(f"{GRAPH}/SubTwist.outputs:angularVelocity") if base_ok else None
drive_logged = drive_error_logged = False
z_ref = None
z_dev = 0.0
xy_drift = 0.0
stable_reported = fell_reported = False
last_cmd = (0.0, 0.0)
sim_start = world.current_time
t0 = time.time()
next_beat = t0
steps = 0
while simulation_app.is_running() and time.time() - t0 < args.duration:
    if robot is not None and lin_attr is not None:
        v = float(og.Controller.get(lin_attr)[0])
        w = float(og.Controller.get(ang_attr)[2])
        last_cmd = (round(v, 3), round(w, 3))
        try:
            action = controller.forward(command=[v, w])
            if hasattr(robot, "apply_wheel_actions"):
                robot.apply_wheel_actions(action)
            else:
                robot.apply_action(action)
            if not drive_logged and (abs(v) > 1e-6 or abs(w) > 1e-6):
                log(f"DRIVE PASS: cmd v={v:.3f} m/s w={w:.3f} rad/s -> wheel velocities {action.joint_velocities}")
                drive_logged = True
        except Exception as e:  # noqa: BLE001
            if not drive_error_logged:
                log(f"DRIVE FAIL: {type(e).__name__}: {e}")
                drive_error_logged = True
    world.step(render=True)
    steps += 1
    sim_t = world.current_time - sim_start

    pose = "n/a"
    if robot is not None:
        try:
            p, q = robot.get_world_pose()
            z = float(p[2]) * mpu
            yaw = np.degrees(np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2)))
            pose = f"x={p[0] * mpu:+.3f} y={p[1] * mpu:+.3f} z={z:+.3f} yaw={yaw:+.1f}deg"
            floor_m = spawn["floor_z"] * mpu
            if z_ref is None and sim_t >= 2.0:
                z_ref = z
            elif z_ref is not None:
                z_dev = max(z_dev, abs(z - z_ref))
            if not stable_reported:
                # no command is sent before T7b is judged (check_t7.py holds the robot still), so any
                # horizontal drift from the spawn point means the robot was pushed out of a collider
                drift = float(np.hypot(p[0] - spawn["x"], p[1] - spawn["y"])) * mpu
                xy_drift = max(xy_drift, drift)
            if not stable_reported and sim_t >= 5.0 and z_ref is not None:
                height = z_ref - floor_m
                ok_b = (-0.05 <= height <= 0.5) and z_dev <= 0.05 and xy_drift <= 0.2
                why = []
                if not -0.05 <= height <= 0.5:
                    why.append(f"chassis {height:+.2f} m from the spawn floor (expected 0..0.5 m)")
                if xy_drift > 0.2:
                    why.append(f"moved {xy_drift:.2f} m from the spawn point without a command "
                               "(spawned inside or under geometry and pushed out; check floor_z, try --floor-z)")
                if z_dev > 0.05:
                    why.append("chassis z still changing")
                log(f"T7b STABLE {'PASS' if ok_b else 'FAIL'}: chassis z settled at {z_ref:.3f} m "
                    f"(spawn floor {floor_m:.3f} m), max z deviation {z_dev:.3f} m over 2-5 s sim time, "
                    f"max xy drift {xy_drift:.3f} m" + ("" if ok_b else " -> " + "; ".join(why)))
                stable_reported = True
            if not fell_reported and z < floor_m - 0.2:
                log(f"T7b STABLE FAIL: robot fell through the floor (z={z:.3f} m, floor {floor_m:.3f} m)"
                    " -> the floor has no collider; try --add-colliders or --ground-plane")
                fell_reported = True
        except Exception:  # noqa: BLE001
            pass
    if time.time() >= next_beat:
        elapsed = time.time() - t0
        log(f"heartbeat t={elapsed:6.1f}s sim_time={sim_t:8.2f}s steps={steps} "
            f"(~{steps / max(elapsed, 1e-6):.1f} steps/s, RTF {sim_t / max(elapsed, 1e-6):.2f}) "
            f"cmd={last_cmd} ground_truth[{pose}]")
        next_beat += 10.0

elapsed = time.time() - t0
sim_t = world.current_time - sim_start
rtf = sim_t / max(elapsed, 1e-6)
log(f"T7e RTF (informative): {sim_t:.1f} s simulated in {elapsed:.1f} s wall = {rtf:.2f}x real time, "
    f"{steps / max(elapsed, 1e-6):.1f} steps/s (T6 reference on the laptop: 0.47-0.59)")
if robot is not None and not stable_reported:
    log("T7b STABLE FAIL: run ended before 5 s of simulated time")
if robot is not None and not drive_logged:
    log("DRIVE FAIL: no non-zero /cmd_vel received during the run")
world.stop()
finish(0)
