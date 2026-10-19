"""
B' test T6 for Isaac Sim 4.1 (headless, ROS 2 Foxy internal libraries):
2D RTX lidar -> /scan, differential-drive robot driven by /cmd_vel, /odom and /tf.

Started from the host with:   ./bprime.sh isaac t6 [--duration S] [--lidar-config NAME] [--no-pointcloud]
Then, in a second terminal:    ./bprime.sh foxy t6     (and optionally ./bprime.sh foxy scan)

Scene: ground plane, a closed 4 m x 4 m box of walls (inner faces at +-2.0 m), one pillar,
and a Jetbot (differential drive, asset Isaac/Robots/Jetbot/jetbot.usd) at the origin facing +x.

Isaac-side sub-tests (each isolated, each prints a [B'] line):
  BACKEND   Foxy backend loaded, Humble not loaded (/proc/self/maps)
  BASE      /clock publisher + /cmd_vel subscriber graph
  ROBOT     Jetbot loaded as WheeledRobot, chassis prim found
  LIDAR     RTX lidar created (config printed), helper graph for /scan (+ /point_cloud) created
  ODOM_TF   graph for /odom, /tf (odom->base_link, base_link->lidar_link) created
  DRIVE     non-zero /cmd_vel converted into wheel commands (DifferentialController)
The data checks (scan content, odom motion vs command, TF frames) run on the Foxy side
(foxy/check_t6.py), because that is the side B' must serve.

Names follow the Isaac Sim 4.x docs (RTX lidar tutorial, TF/odometry tutorial, Jetbot
core-API tutorial). When a node/attribute name does not match the 4.1 image, the script
prints the attributes the node really has (ATTRS lines) so the script can be corrected.
"""

import argparse
import glob
import os
import re
import sys
import time

parser = argparse.ArgumentParser()
parser.add_argument("--duration", type=float, default=300.0, help="wall-clock seconds to run")
parser.add_argument("--lidar-config", default="RPLIDAR_S2E",
                    help="RTX lidar config name (file stem). Default RPLIDAR_S2E (the robot's real lidar). "
                         "'auto' = first 2D-looking config, else Example_Rotary")
parser.add_argument("--no-pointcloud", action="store_true", help="do not publish /point_cloud")
args, _unknown = parser.parse_known_args()

TAG = "[B']"
WHEEL_RADIUS = 0.03     # Jetbot values used in the Isaac Sim core-API tutorial
WHEEL_BASE = 0.1125
LIDAR_OFFSET = (0.0, 0.0, 0.15)   # lidar position relative to the chassis prim [m]


def log(msg):
    print(f"{TAG} {msg}", flush=True)


log(f"python={sys.version.split()[0]}")
for k in ("ROS_DISTRO", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "FASTRTPS_DEFAULT_PROFILES_FILE"):
    log(f"env {k}={os.environ.get(k, '<unset>')}")
if "omni.isaac.ros2_bridge/foxy/lib" not in os.environ.get("LD_LIBRARY_PATH", ""):
    log("WARNING: Foxy internal lib dir is not in LD_LIBRARY_PATH. Start this test with ./bprime.sh isaac t6")

# ---------------------------------------------------------------- start Kit headless
try:
    from isaacsim import SimulationApp  # recommended import in 4.1 (avoids the deprecation warning)
except ImportError:  # pragma: no cover
    from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": True})

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

import numpy as np  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.kit.commands  # noqa: E402
import usdrt.Sdf  # noqa: E402
from omni.isaac.core import World  # noqa: E402
from omni.isaac.core.objects import FixedCuboid  # noqa: E402
from omni.isaac.core.utils.nucleus import get_assets_root_path  # noqa: E402
from pxr import Gf, Usd  # noqa: E402

world = World(stage_units_in_meters=1.0)
stage = world.stage
keys = og.Controller.Keys
GRAPH = "/BPrimeT6"


def dump_attrs(node_path):
    """Print the attribute names a node really has (to fix name mismatches against 4.1)."""
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


# ---------------------------------------------------------------- scene
try:
    world.scene.add_default_ground_plane()
    log("SCENE: default ground plane added")
except Exception as e:  # noqa: BLE001
    from omni.isaac.core.objects import GroundPlane
    GroundPlane(prim_path="/World/GroundPlane", z_position=0.0)
    log(f"SCENE: default ground plane failed ({type(e).__name__}), plain GroundPlane added instead")

WALL_H, WALL_T, HALF = 0.5, 0.1, 2.0
walls = {
    "WallPX": ((HALF + WALL_T / 2, 0.0, WALL_H / 2), (WALL_T, 2 * HALF + 2 * WALL_T, WALL_H)),
    "WallNX": ((-HALF - WALL_T / 2, 0.0, WALL_H / 2), (WALL_T, 2 * HALF + 2 * WALL_T, WALL_H)),
    "WallPY": ((0.0, HALF + WALL_T / 2, WALL_H / 2), (2 * HALF, WALL_T, WALL_H)),
    "WallNY": ((0.0, -HALF - WALL_T / 2, WALL_H / 2), (2 * HALF, WALL_T, WALL_H)),
    "Pillar": ((1.0, 1.0, WALL_H / 2), (0.2, 0.2, WALL_H)),
}
for name, (pos, scl) in walls.items():
    FixedCuboid(prim_path=f"/World/{name}", name=name.lower(), position=np.array(pos),
                scale=np.array(scl), size=1.0, color=np.array([0.6, 0.6, 0.6]))
log("SCENE: 4 m x 4 m box (inner faces at +-2.0 m) and a pillar at (1.0, 1.0) created")

# ---------------------------------------------------------------- ROBOT
robot = None
controller = None
chassis_path = None
ROBOT_PATH = "/World/Jetbot"
try:
    from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
    from omni.isaac.wheeled_robots.robots import WheeledRobot

    assets_root = get_assets_root_path()
    if not assets_root:
        raise RuntimeError("get_assets_root_path() returned None")
    robot = world.scene.add(WheeledRobot(
        prim_path=ROBOT_PATH, name="jetbot",
        wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
        create_robot=True, usd_path=assets_root + "/Isaac/Robots/Jetbot/jetbot.usd",
        position=np.array([0.0, 0.0, 0.05]),
    ))
    controller = DifferentialController(name="bprime_diff", wheel_radius=WHEEL_RADIUS, wheel_base=WHEEL_BASE)
    for _ in range(5):
        simulation_app.update()
    root_prim = stage.GetPrimAtPath(ROBOT_PATH)
    candidates = [p for p in Usd.PrimRange(root_prim) if p.GetName().lower() in ("chassis", "base_link", "chassis_link")]
    chassis_path = str(candidates[0].GetPath()) if candidates else ROBOT_PATH
    children = [c.GetName() for c in root_prim.GetChildren()]
    log(f"ROBOT PASS: Jetbot loaded at {ROBOT_PATH}; children={children}; chassis prim = {chassis_path}")
except Exception as e:  # noqa: BLE001
    log(f"ROBOT FAIL: {type(e).__name__}: {e}")

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

# ---------------------------------------------------------------- LIDAR
TWO_D = re.compile(r"(rplidar|s2e|tim5|tim7|lms1|picoscan|nanoscan|microscan|2d)", re.IGNORECASE)


def list_lidar_configs():
    found = []
    for path in glob.glob("/isaac-sim/**/*.json", recursive=True):
        if "/lidar" in path.lower():
            found.append(path)
    return sorted(set(found))


lidar_path = None
if base_ok and chassis_path is not None:
    try:
        configs = list_lidar_configs()
        log(f"LIDAR configs found: {len(configs)}")
        for p in configs:
            log(f"  lidar-config: {p}")
        if args.lidar_config != "auto":
            order = [args.lidar_config, "Example_Rotary"]
        else:
            two_d = [os.path.splitext(os.path.basename(p))[0] for p in configs if TWO_D.search(os.path.basename(p))]
            order = two_d[:3] + ["Example_Rotary"]
        sensor = None
        used = None
        for cfg in order:
            try:
                ok, sensor = omni.kit.commands.execute(
                    "IsaacSensorCreateRtxLidar",
                    path="/lidar",
                    parent=chassis_path,
                    config=cfg,
                    translation=LIDAR_OFFSET,
                    orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
                )
                if ok and sensor is not None and sensor.IsValid():
                    used = cfg
                    break
                log(f"LIDAR config '{cfg}' not accepted (ok={ok})")
            except Exception as e:  # noqa: BLE001
                log(f"LIDAR config '{cfg}' failed: {type(e).__name__}: {e}")
        if used is None:
            raise RuntimeError("no lidar config could be created")
        lidar_path = str(sensor.GetPath())
        log(f"LIDAR created: prim={lidar_path} config='{used}' (2D profile: {'yes' if TWO_D.search(used) else 'no/unknown'})")

        create = [
            ("LidarRP", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
            ("ScanHelper", "omni.isaac.ros2_bridge.ROS2RtxLidarHelper"),
        ]
        connect = [
            (f"{GRAPH}/OnPlaybackTick.outputs:tick", "LidarRP.inputs:execIn"),
            ("LidarRP.outputs:execOut", "ScanHelper.inputs:execIn"),
            ("LidarRP.outputs:renderProductPath", "ScanHelper.inputs:renderProductPath"),
            (f"{GRAPH}/Context.outputs:context", "ScanHelper.inputs:context"),
        ]
        values = [
            ("LidarRP.inputs:cameraPrim", [usdrt.Sdf.Path(lidar_path)]),
            ("ScanHelper.inputs:type", "laser_scan"),
            ("ScanHelper.inputs:topicName", "scan"),
            ("ScanHelper.inputs:frameId", "lidar_link"),
        ]
        if not args.no_pointcloud:
            create.append(("PcHelper", "omni.isaac.ros2_bridge.ROS2RtxLidarHelper"))
            connect += [
                ("LidarRP.outputs:execOut", "PcHelper.inputs:execIn"),
                ("LidarRP.outputs:renderProductPath", "PcHelper.inputs:renderProductPath"),
                (f"{GRAPH}/Context.outputs:context", "PcHelper.inputs:context"),
            ]
            values += [
                ("PcHelper.inputs:type", "point_cloud"),
                ("PcHelper.inputs:topicName", "point_cloud"),
                ("PcHelper.inputs:frameId", "lidar_link"),
                ("PcHelper.inputs:fullScan", True),
            ]
        if graph_edit("LIDAR", {keys.CREATE_NODES: create, keys.CONNECT: connect, keys.SET_VALUES: values},
                      ["LidarRP", "ScanHelper", "PcHelper"]):
            log("LIDAR PASS(created): /scan" + ("" if args.no_pointcloud else " and /point_cloud") + " helpers created")
    except Exception as e:  # noqa: BLE001
        log(f"LIDAR FAIL: {type(e).__name__}: {e}")

# ---------------------------------------------------------------- ODOM + TF
if base_ok and chassis_path is not None:
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
            ("TfBaseLidar.inputs:translation", list(LIDAR_OFFSET)),
        ],
    }
    if graph_edit("ODOM_TF", spec, ["ComputeOdom", "PubOdom", "TfOdomBase", "TfBaseLidar"]):
        log("ODOM_TF PASS(created): /odom (odom->base_link) and /tf (odom->base_link, base_link->lidar_link)")

# ---------------------------------------------------------------- run
world.reset()
if robot is not None:
    try:
        robot.initialize()
    except Exception:  # noqa: BLE001 - already initialised by world.reset() in most versions
        pass
world.play()
log(f"simulation playing for {args.duration:.0f} s wall time. Now run on the host: ./bprime.sh foxy t6")

lin_attr = og.Controller.attribute(f"{GRAPH}/SubTwist.outputs:linearVelocity") if base_ok else None
ang_attr = og.Controller.attribute(f"{GRAPH}/SubTwist.outputs:angularVelocity") if base_ok else None
drive_logged = False
drive_error_logged = False
last_cmd = (0.0, 0.0)
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
    if time.time() >= next_beat:
        elapsed = time.time() - t0
        pose = "n/a"
        if robot is not None:
            try:
                p, q = robot.get_world_pose()
                yaw = np.degrees(np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2)))
                pose = f"x={p[0]:+.3f} y={p[1]:+.3f} yaw={yaw:+.1f}deg"
            except Exception:  # noqa: BLE001
                pass
        log(f"heartbeat t={elapsed:6.1f}s sim_time={world.current_time:8.2f}s steps={steps} "
            f"(~{steps / max(elapsed, 1e-6):.1f} steps/s) cmd={last_cmd} ground_truth[{pose}]")
        next_beat += 10.0

if robot is not None and not drive_logged:
    log("DRIVE FAIL: no non-zero /cmd_vel received during the run")
backend_check("end-of-run")
log("done")
world.stop()
simulation_app.close()
