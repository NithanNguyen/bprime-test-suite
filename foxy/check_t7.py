#!/usr/bin/env python3
"""
T7 checks on the Foxy side (started by ./bprime.sh foxy t7 while ./bprime.sh isaac t7 runs).

  T7c SCAN     /scan arrives inside the apartment; share of valid ranges
  T7c TF       /tf has odom->base_link and base_link->lidar_link
  T7g CAMERA   /bprime/rgb rate, size, brightness (catches black or flat renders) + /bprime/camera_info
  T7h ROTATE   w=0.5 rad/s in place for 6 s; yaw change vs w * sim time (graded like T6e)
  T7h FORWARD  v=0.15 m/s for 4 s, only if the lidar sees >= 0.8 m free ahead (graded like T6d)

Waits up to 30 min for the Isaac side, because large scenes can take 15-20 min to load the first
time (shader / MDL compilation). Sim time comes from /clock, as in T6.
"""
import argparse
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_msgs.msg import TFMessage

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "common"))
from t7_lib import grade_ratio, image_stats  # noqa: E402

V_CMD, W_CMD, ROT_SECONDS, FWD_SECONDS, SETTLE = 0.15, 0.5, 6.0, 4.0, 2.0
FRONT_FREE_M = 0.8
SETTLE_SIM_S = 6.0
WAIT_S = 1800.0
BE = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class T7(Node):
    def __init__(self):
        super().__init__("bprime_t7_checker")
        self.scan = self.odom = self.clock = self.img = self.info = None
        self.img_count = 0
        self.tf_pairs = set()
        self.yaw_unwrapped, self._last_yaw = 0.0, None
        self.create_subscription(LaserScan, "scan", lambda m: setattr(self, "scan", m), BE)
        self.create_subscription(Odometry, "odom", self.on_odom, BE)
        self.create_subscription(Clock, "clock", self.on_clock, BE)
        self.create_subscription(TFMessage, "tf", self.on_tf, BE)
        self.create_subscription(Image, "bprime/rgb", self.on_img, BE)
        self.create_subscription(CameraInfo, "bprime/camera_info", lambda m: setattr(self, "info", m), BE)
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)

    def on_odom(self, m):
        self.odom = m
        y = yaw_of(m.pose.pose.orientation)
        if self._last_yaw is not None:
            self.yaw_unwrapped += (y - self._last_yaw + math.pi) % (2 * math.pi) - math.pi
        self._last_yaw = y

    def on_clock(self, m):
        self.clock = m.clock.sec + m.clock.nanosec * 1e-9

    def on_tf(self, m):
        for t in m.transforms:
            self.tf_pairs.add((t.header.frame_id, t.child_frame_id))

    def on_img(self, m):
        self.img, self.img_count = m, self.img_count + 1

    def spin_for(self, seconds, publish=None):
        end, next_pub = time.time() + seconds, 0.0
        while time.time() < end:
            if publish is not None and time.time() >= next_pub:
                self.pub.publish(publish)
                next_pub = time.time() + 0.1
            rclpy.spin_once(self, timeout_sec=0.02)

    def front_range(self):
        s = self.scan
        if s is None or len(s.ranges) < 2:
            return None
        step = (s.angle_max - s.angle_min) / (len(s.ranges) - 1)   # sign of angle_increment seen inconsistent (T6)
        if step == 0.0:
            return None
        i = int(round((0.0 - s.angle_min) / step))
        half = max(2, int(round(math.radians(10.0) / abs(step))))   # +-10 deg cone: the robot body is ~0.2 m wide
        window = [r for r in s.ranges[max(0, i - half):i + half + 1] if s.range_min <= r <= s.range_max]
        return min(window) if window else None


def twist(v=0.0, w=0.0):
    t = Twist()
    t.linear.x, t.angular.z = float(v), float(w)
    return t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-camera", action="store_true", help="the Isaac side runs without camera")
    args = ap.parse_args()
    rclpy.init()
    n = T7()
    print(f"=== T7: waiting up to {WAIT_S / 60:.0f} min for /scan, /odom, /tf, /clock"
          f"{'' if args.no_camera else ', /bprime/rgb'} (large scenes load slowly the first time) ===", flush=True)
    end, beat = time.time() + WAIT_S, time.time() + 60.0
    while time.time() < end:
        have = bool(n.scan and n.odom and n.clock is not None and n.tf_pairs and (args.no_camera or n.img))
        if have:
            break
        rclpy.spin_once(n, timeout_sec=0.1)
        if time.time() >= beat:
            print(f"  ...still waiting: scan={n.scan is not None} odom={n.odom is not None} "
                  f"clock={n.clock is not None} tf={bool(n.tf_pairs)} rgb={n.img is not None}", flush=True)
            beat += 60.0

    # The Isaac side judges T7b (robot at rest) between 2 and 5 s of sim time after play. Hold the
    # robot still until at least SETTLE_SIM_S of sim time has passed since the first /clock seen here,
    # so that the motion tests below cannot disturb T7b, whenever this script was started.
    if n.clock is not None:
        c_first = n.clock
        print(f"=== T7: holding the robot still for {SETTLE_SIM_S:.0f} s of sim time (T7b runs on the Isaac side) ===",
              flush=True)
        deadline = time.time() + 120.0
        while n.clock - c_first < SETTLE_SIM_S and time.time() < deadline:
            n.spin_for(0.5, publish=twist())

    # rest for 5 s and measure the camera rate meanwhile
    c_start, cnt_start = n.clock, n.img_count
    n.spin_for(5.0, publish=twist())
    c_end, cnt_end = n.clock, n.img_count

    s = n.scan
    if s is None:
        print("T7c SCAN FAIL: no /scan received (check LIDAR / SPAWN lines in the Isaac log)")
    else:
        valid = [r for r in s.ranges if s.range_min <= r <= s.range_max and math.isfinite(r)]
        ratio = len(valid) / len(s.ranges) if s.ranges else 0.0
        verdict = "PASS" if ratio >= 0.8 else ("WARN" if ratio >= 0.5 else "FAIL")
        print(f"T7c SCAN {verdict}: beams={len(s.ranges)} valid={len(valid)} ({100 * ratio:.0f}%) "
              f"min={min(valid) if valid else float('nan'):.2f} m max={max(valid) if valid else float('nan'):.2f} m")
        print("     note: T6 already showed ~11% of beams at -1.0 on an empty box (README section 8)")
    need = {("odom", "base_link"), ("base_link", "lidar_link")}
    missing = need - n.tf_pairs
    print(f"T7c TF {'PASS' if not missing else 'FAIL'}: missing={sorted(missing)}")

    if not args.no_camera:
        if n.img is None:
            print("T7g CAMERA FAIL: no /bprime/rgb received (check CAMERA lines in the Isaac log)")
        else:
            im = n.img
            wall_rate = (cnt_end - cnt_start) / 5.0
            sim_dt = (c_end - c_start) if (c_end is not None and c_start is not None) else float("nan")
            sim_rate = (cnt_end - cnt_start) / sim_dt if sim_dt and sim_dt > 0 else float("nan")
            mean, std = image_stats(im.data)
            bright_ok = 15.0 <= mean <= 240.0 and std >= 5.0
            verdict = "PASS" if (wall_rate > 0 and bright_ok) else ("WARN" if wall_rate > 0 else "FAIL")
            print(f"T7g CAMERA {verdict}: {im.width}x{im.height} {im.encoding} rate={wall_rate:.1f} Hz wall "
                  f"({sim_rate:.1f} per sim s) brightness mean={mean:.0f} std={std:.0f}")
            if not bright_ok:
                print("     dark or flat image: try --add-dome-light (or check the HDR/MDL errors in the Isaac log)")
            if n.info is None:
                print("     /bprime/camera_info: not received")
            else:
                print(f"     /bprime/camera_info: {n.info.width}x{n.info.height} fx={n.info.k[0]:.1f} "
                      f"cx={n.info.k[2]:.1f} cy={n.info.k[5]:.1f}")

    if n.odom is None or n.clock is None:
        print("T7h skipped: need /odom and /clock")
        n.destroy_node()
        rclpy.shutdown()
        return

    yaw_start, c0 = n.yaw_unwrapped, n.clock
    print(f"=== T7h: rotating in place w={W_CMD} rad/s for {ROT_SECONDS} s ===", flush=True)
    n.spin_for(ROT_SECONDS, publish=twist(w=W_CMD))
    c1 = n.clock
    n.spin_for(SETTLE, publish=twist())
    expected = W_CMD * (c1 - c0)
    dyaw = n.yaw_unwrapped - yaw_start
    ratio = dyaw / expected if expected > 0 else float("nan")
    print(f"T7h ROTATE {grade_ratio(ratio)}: sim_time_active={c1 - c0:.2f}s expected={math.degrees(expected):.1f} deg "
          f"measured={math.degrees(dyaw):.1f} deg ratio={ratio:.2f}")

    front = n.front_range()
    if front is None or front < FRONT_FREE_M:
        print(f"T7h FORWARD SKIPPED: free space ahead {front if front is not None else 'unknown'} m "
              f"< {FRONT_FREE_M} m (not a failure)")
    else:
        p0, yaw0, c0 = n.odom.pose.pose.position, yaw_of(n.odom.pose.pose.orientation), n.clock
        print(f"=== T7h: forward v={V_CMD} m/s for {FWD_SECONDS} s (free ahead {front:.2f} m) ===", flush=True)
        n.spin_for(FWD_SECONDS, publish=twist(v=V_CMD))
        c1 = n.clock
        n.spin_for(SETTLE, publish=twist())
        p1 = n.odom.pose.pose.position
        dx, dy = p1.x - p0.x, p1.y - p0.y
        dist = math.hypot(dx, dy)
        fwd = dx * math.cos(yaw0) + dy * math.sin(yaw0)
        expected = V_CMD * (c1 - c0)
        ratio = dist / expected if expected > 0 else float("nan")
        g = grade_ratio(ratio) if fwd > 0 else "FAIL"
        front1 = n.front_range()
        print(f"T7h FORWARD {g}: sim_time_active={c1 - c0:.2f}s expected={expected:.3f} m measured={dist:.3f} m "
              f"ratio={ratio:.2f}; lidar front {front:.2f} -> "
              f"{front1 if front1 is not None else float('nan'):.2f} m (informative)")

    n.spin_for(1.0, publish=twist())
    print("=== T7 done (robot stopped) ===")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
