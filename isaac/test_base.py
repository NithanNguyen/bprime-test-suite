"""
B' bridge test for Isaac Sim 4.1 (headless) with the ROS 2 Foxy internal libraries.

Started from the host with:   ./bprime.sh isaac base [--duration S] [--no-camera] [--no-assets]
(bprime.sh sources /bprime/isaac/env.sh in the container and runs this file with /isaac-sim/python.sh).

Sub-tests (each isolated, each prints a PASS/FAIL line tagged [B']):
  BACKEND  : the process really loaded the *Foxy* bridge backend, not the Humble fallback
             (checked from /proc/self/maps, i.e. from what the OS actually mapped).
  GRAPH    : OmniGraph with /clock publisher + /cmd_vel subscriber was created.
  CAMERA   : (optional) RGB + camera_info publishers on /bprime/rgb, /bprime/camera_info.
  ASSETS   : (optional) the 4.1 cloud asset root is reachable.
  TWIST    : messages sent from the Foxy container on /cmd_vel arrived in Isaac Sim.
The /clock direction (Isaac -> Foxy) is verified on the Foxy side with `ros2 topic echo /clock`.

Node type and attribute names used here are taken from the omni.isaac.ros2_bridge /
omni.isaac.core_nodes OmniGraph references (4.x naming):
  ROS2Context (inputs:domain_id, inputs:useDomainIDEnvVar, outputs:context)
  ROS2PublishClock (inputs:execIn, inputs:context, inputs:timeStamp, inputs:topicName)
  ROS2SubscribeTwist (inputs:execIn, inputs:context, inputs:topicName,
                      outputs:execOut, outputs:linearVelocity, outputs:angularVelocity)
  ROS2CameraHelper (inputs:execIn, inputs:context, inputs:renderProductPath,
                    inputs:topicName, inputs:type, inputs:frameId)
  IsaacCreateRenderProduct (inputs:execIn, inputs:cameraPrim, outputs:execOut,
                            outputs:renderProductPath)
  IsaacReadSimulationTime (outputs:simulationTime)
"""

import argparse
import os
import sys
import time

parser = argparse.ArgumentParser()
parser.add_argument("--duration", type=float, default=300.0, help="wall-clock seconds to run")
parser.add_argument("--no-camera", action="store_true", help="skip the camera publisher sub-test")
parser.add_argument("--no-assets", action="store_true", help="skip the cloud asset reachability check")
args, _unknown = parser.parse_known_args()

TAG = "[B']"


def log(msg):
    print(f"{TAG} {msg}", flush=True)


# ---------------------------------------------------------------- environment
log(f"python={sys.version.split()[0]}")
for k in ("ROS_DISTRO", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "FASTRTPS_DEFAULT_PROFILES_FILE"):
    log(f"env {k}={os.environ.get(k, '<unset>')}")
ldp = os.environ.get("LD_LIBRARY_PATH", "")
if "omni.isaac.ros2_bridge/foxy/lib" not in ldp:
    log("WARNING: Foxy internal lib dir is not in LD_LIBRARY_PATH. Start this test with ./bprime.sh isaac base")

# ---------------------------------------------------------------- start Kit headless
try:
    from isaacsim import SimulationApp  # recommended import in 4.1 (avoids the deprecation warning)
except ImportError:  # pragma: no cover
    from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": True})

from omni.isaac.core.utils.extensions import enable_extension  # noqa: E402

enable_extension("omni.isaac.ros2_bridge")
for _ in range(10):
    simulation_app.update()


# ---------------------------------------------------------------- BACKEND check
def mapped_bridge_libs():
    libs = set()
    try:
        with open("/proc/self/maps", "r") as f:
            for line in f:
                parts = line.split()
                if len(parts) >= 6 and "ros2_bridge" in parts[-1]:
                    libs.add(parts[-1])
    except OSError as e:
        log(f"cannot read /proc/self/maps: {e}")
    return sorted(libs)


def backend_check(stage_name):
    libs = mapped_bridge_libs()
    for p in libs:
        log(f"mapped({stage_name}): {p}")
    foxy_loaded = any("/foxy/" in p or "foxy" in os.path.basename(p) for p in libs)
    humble_loaded = any("/humble/" in p or "humble" in os.path.basename(p) for p in libs)
    if foxy_loaded and not humble_loaded:
        log(f"BACKEND PASS ({stage_name}): Foxy bridge backend loaded, Humble not loaded")
    elif humble_loaded:
        log(f"BACKEND FAIL ({stage_name}): Humble backend loaded -> cross-distro, results INVALID for B'")
    else:
        log(f"BACKEND UNKNOWN ({stage_name}): no foxy/humble bridge library in memory maps; check Kit log")


backend_check("after-enable")

# ---------------------------------------------------------------- scene + graph
import omni.graph.core as og  # noqa: E402
from omni.isaac.core import World  # noqa: E402

world = World(stage_units_in_meters=1.0)
stage = world.stage
keys = og.Controller.Keys
GRAPH = "/BPrimeGraph"

graph_ok = False
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
    graph_ok = True
    log("GRAPH PASS: /clock publisher and /cmd_vel subscriber created")
except Exception as e:  # noqa: BLE001
    log(f"GRAPH FAIL: {type(e).__name__}: {e}")

# ---------------------------------------------------------------- optional camera
if graph_ok and not args.no_camera:
    try:
        import usdrt.Sdf
        from pxr import Gf, UsdGeom, UsdLux

        cam = UsdGeom.Camera.Define(stage, "/World/BPrimeCamera")
        UsdGeom.XformCommonAPI(cam).SetTranslate(Gf.Vec3d(-3.0, 0.0, 1.0))
        UsdLux.DistantLight.Define(stage, "/World/BPrimeLight")
        cube = UsdGeom.Cube.Define(stage, "/World/BPrimeCube")
        UsdGeom.XformCommonAPI(cube).SetTranslate(Gf.Vec3d(0.0, 0.0, 0.5))

        og.Controller.edit(
            GRAPH,
            {
                keys.CREATE_NODES: [
                    ("RenderProduct", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                    ("RgbHelper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ("InfoHelper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ],
                keys.CONNECT: [
                    (f"{GRAPH}/OnPlaybackTick.outputs:tick", "RenderProduct.inputs:execIn"),
                    ("RenderProduct.outputs:execOut", "RgbHelper.inputs:execIn"),
                    ("RenderProduct.outputs:execOut", "InfoHelper.inputs:execIn"),
                    ("RenderProduct.outputs:renderProductPath", "RgbHelper.inputs:renderProductPath"),
                    ("RenderProduct.outputs:renderProductPath", "InfoHelper.inputs:renderProductPath"),
                    (f"{GRAPH}/Context.outputs:context", "RgbHelper.inputs:context"),
                    (f"{GRAPH}/Context.outputs:context", "InfoHelper.inputs:context"),
                ],
                keys.SET_VALUES: [
                    ("RenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path("/World/BPrimeCamera")]),
                    ("RgbHelper.inputs:type", "rgb"),
                    ("RgbHelper.inputs:topicName", "bprime/rgb"),
                    ("RgbHelper.inputs:frameId", "bprime_camera"),
                    ("InfoHelper.inputs:type", "camera_info"),
                    ("InfoHelper.inputs:topicName", "bprime/camera_info"),
                    ("InfoHelper.inputs:frameId", "bprime_camera"),
                ],
            },
        )
        log("CAMERA PASS(created): publishers on /bprime/rgb and /bprime/camera_info (verify rate with ros2 topic hz)")
    except Exception as e:  # noqa: BLE001
        log(f"CAMERA FAIL(create): {type(e).__name__}: {e}  (clock/twist tests still valid)")

# ---------------------------------------------------------------- optional assets
if not args.no_assets:
    try:
        import omni.client
        from omni.isaac.core.utils.nucleus import get_assets_root_path

        root = get_assets_root_path()
        log(f"assets root = {root}")
        if root:
            probe = root + "/Isaac/Environments/Grid/default_environment.usd"
            result, _entry = omni.client.stat(probe)
            if result == omni.client.Result.OK:
                log(f"ASSETS PASS: reachable {probe}")
            else:
                log(f"ASSETS FAIL: {probe} -> {result}")
        else:
            log("ASSETS FAIL: get_assets_root_path() returned None")
    except Exception as e:  # noqa: BLE001
        log(f"ASSETS FAIL: {type(e).__name__}: {e}")

# ---------------------------------------------------------------- run
world.reset()
world.play()
log(f"simulation playing for {args.duration:.0f} s wall time. Now run on the host: ./bprime.sh foxy base")

twist_attr_lin = og.Controller.attribute(f"{GRAPH}/SubTwist.outputs:linearVelocity") if graph_ok else None
twist_attr_ang = og.Controller.attribute(f"{GRAPH}/SubTwist.outputs:angularVelocity") if graph_ok else None
first_twist_t = None
last_seen = None
t0 = time.time()
next_beat = t0
steps = 0
while simulation_app.is_running() and time.time() - t0 < args.duration:
    world.step(render=True)
    steps += 1
    if twist_attr_lin is not None:
        lin = tuple(round(float(v), 3) for v in og.Controller.get(twist_attr_lin))
        ang = tuple(round(float(v), 3) for v in og.Controller.get(twist_attr_ang))
        if any(abs(v) > 1e-6 for v in lin + ang):
            if first_twist_t is None:
                first_twist_t = time.time() - t0
                log(f"TWIST PASS: first /cmd_vel received at t={first_twist_t:.1f}s lin={lin} ang={ang}")
            last_seen = (lin, ang)
    if time.time() >= next_beat:
        elapsed = time.time() - t0
        rate = steps / elapsed if elapsed > 0 else 0.0
        log(f"heartbeat t={elapsed:6.1f}s sim_time={world.current_time:8.2f}s steps={steps} (~{rate:.1f} steps/s) last_twist={last_seen}")
        next_beat += 10.0

if graph_ok and first_twist_t is None:
    log("TWIST FAIL: no non-zero /cmd_vel received during the run")
backend_check("end-of-run")
log("done")
world.stop()
simulation_app.close()
