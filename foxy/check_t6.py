#!/usr/bin/env python3
"""
T6 checks on the Foxy side (started by ./bprime.sh foxy t6 while ./bprime.sh isaac t6 runs).

  T6a SCAN      /scan arrives; ranges are mostly finite (robot is inside a closed 4 m x 4 m box)
  T6b ODOM      /odom arrives with frame ids odom / base_link
  T6c TF        /tf contains odom->base_link and base_link->lidar_link
  T6d LINEAR    publish v=0.15 m/s; measured displacement / (v * sim-time the command was active)
  T6e ANGULAR   publish w=0.5 rad/s; measured yaw change / (w * sim-time the command was active)
  T6f LIDAR-ODOM  forward range to the +x wall shrinks by about the odometry displacement (informative)

Sim time comes from /clock, so the result does not depend on how fast the laptop/PC simulates.
All subscriptions are best-effort, which matches both reliable and best-effort publishers.
Grades: PASS ratio 0.8-1.2, WARN 0.5-1.5, FAIL otherwise.
Note: /odom here comes from IsaacComputeOdometry (the chassis pose in sim), not from wheel encoders,
so T6d/T6e test the command path + controller + physics, not wheel-odometry accuracy.
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_msgs.msg import TFMessage

V_CMD, W_CMD, CMD_SECONDS, SETTLE_SECONDS = 0.15, 0.5, 6.0, 2.0
BE = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)


def grade(ratio):
    if ratio is None or math.isnan(ratio):
        return "FAIL"
    if 0.8 <= ratio <= 1.2:
        return "PASS"
    if 0.5 <= ratio <= 1.5:
        return "WARN"
    return "FAIL"


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class T6(Node):
    def __init__(self):
        super().__init__("bprime_t6_checker")
        self.scan = None
        self.scan_count = 0
        self.pc_count = 0
        self.odom = None
        self.odom_count = 0
        self.clock = None
        self.tf_pairs = set()
        self.yaw_unwrapped = 0.0
        self._last_yaw = None
        self.create_subscription(LaserScan, "scan", self.on_scan, BE)
        self.create_subscription(PointCloud2, "point_cloud", self.on_pc, BE)
        self.create_subscription(Odometry, "odom", self.on_odom, BE)
        self.create_subscription(Clock, "clock", self.on_clock, BE)
        self.create_subscription(TFMessage, "tf", self.on_tf, BE)
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)

    def on_scan(self, m):
        self.scan, self.scan_count = m, self.scan_count + 1

    def on_pc(self, _m):
        self.pc_count += 1

    def on_odom(self, m):
        self.odom, self.odom_count = m, self.odom_count + 1
        y = yaw_of(m.pose.pose.orientation)
        if self._last_yaw is not None:
            d = y - self._last_yaw
            d = (d + math.pi) % (2 * math.pi) - math.pi
            self.yaw_unwrapped += d
        self._last_yaw = y

    def on_clock(self, m):
        self.clock = m.clock.sec + m.clock.nanosec * 1e-9

    def on_tf(self, m):
        for t in m.transforms:
            self.tf_pairs.add((t.header.frame_id, t.child_frame_id))

    def spin_for(self, seconds, publish=None):
        end = time.time() + seconds
        next_pub = 0.0
        while time.time() < end:
            if publish is not None and time.time() >= next_pub:
                self.pub.publish(publish)
                next_pub = time.time() + 0.1  # 10 Hz
            rclpy.spin_once(self, timeout_sec=0.02)

    def front_range(self):
        s = self.scan
        if s is None or len(s.ranges) < 2:
            return None
        # use the step implied by angle_min/angle_max (the published angle_increment sign was seen inconsistent)
        step = (s.angle_max - s.angle_min) / (len(s.ranges) - 1)
        if step == 0.0:
            return None
        i = int(round((0.0 - s.angle_min) / step))
        if not 0 <= i < len(s.ranges):
            return None
        window = [r for r in s.ranges[max(0, i - 2):i + 3] if s.range_min <= r <= s.range_max]
        return sum(window) / len(window) if window else None


def twist(v=0.0, w=0.0):
    t = Twist()
    t.linear.x, t.angular.z = float(v), float(w)
    return t


def main():
    rclpy.init()
    n = T6()
    print("=== T6: waiting up to 300 s for /scan, /odom, /tf, /clock (start ./bprime.sh isaac t6 if not running) ===", flush=True)
    end = time.time() + 300.0
    while time.time() < end and not (n.scan and n.odom and n.clock is not None and n.tf_pairs):
        rclpy.spin_once(n, timeout_sec=0.1)
    n.spin_for(2.0, publish=twist())  # make sure the robot is at rest and collect a few messages

    # T6a
    s = n.scan
    if s is None:
        print("T6a SCAN FAIL: no /scan received (check LIDAR lines in the Isaac log)")
    else:
        valid = [r for r in s.ranges if s.range_min <= r <= s.range_max and math.isfinite(r)]
        ratio = len(valid) / len(s.ranges) if s.ranges else 0.0
        verdict = "PASS" if ratio >= 0.8 else ("WARN" if ratio > 0.0 else "FAIL")
        print(f"T6a SCAN {verdict}: frame_id={s.header.frame_id} beams={len(s.ranges)} "
              f"angle=[{math.degrees(s.angle_min):.1f}, {math.degrees(s.angle_max):.1f}] deg "
              f"step={math.degrees(s.angle_increment):.3f} deg range=[{s.range_min:.2f}, {s.range_max:.2f}] m "
              f"valid={len(valid)} ({100 * ratio:.0f}%) min={min(valid) if valid else float('nan'):.3f} m "
              f"max={max(valid) if valid else float('nan'):.3f} m msgs={n.scan_count}")
        print("     expected inside the box: valid close to 100%, walls between 2.0 m and ~2.8 m, pillar ~1.3 m")
    print(f"     /point_cloud messages received so far: {n.pc_count} (informative)")

    # T6b
    if n.odom is None:
        print("T6b ODOM FAIL: no /odom received")
    else:
        o = n.odom
        ok = o.header.frame_id == "odom" and o.child_frame_id == "base_link"
        print(f"T6b ODOM {'PASS' if ok else 'WARN'}: frame_id={o.header.frame_id} child={o.child_frame_id} msgs={n.odom_count}")

    # T6c
    need = {("odom", "base_link"), ("base_link", "lidar_link")}
    missing = need - n.tf_pairs
    print(f"T6c TF {'PASS' if not missing else 'FAIL'}: seen={sorted(n.tf_pairs)} missing={sorted(missing)}")

    if n.odom is None or n.clock is None:
        print("T6d/T6e skipped: need /odom and /clock")
        n.destroy_node()
        rclpy.shutdown()
        return

    # T6d linear
    p0 = n.odom.pose.pose.position
    yaw0 = yaw_of(n.odom.pose.pose.orientation)
    front0 = n.front_range()
    c0 = n.clock
    print(f"=== T6d: publishing v={V_CMD} m/s for {CMD_SECONDS} s ===", flush=True)
    n.spin_for(CMD_SECONDS, publish=twist(v=V_CMD))
    c1 = n.clock
    n.spin_for(SETTLE_SECONDS, publish=twist())
    p1 = n.odom.pose.pose.position
    dx, dy = p1.x - p0.x, p1.y - p0.y
    dist = math.hypot(dx, dy)
    forward = dx * math.cos(yaw0) + dy * math.sin(yaw0)
    expected = V_CMD * (c1 - c0)
    ratio = dist / expected if expected > 0 else float("nan")
    g = grade(ratio) if forward > 0 else "FAIL"
    print(f"T6d LINEAR {g}: sim_time_active={c1 - c0:.2f}s expected={expected:.3f} m measured={dist:.3f} m "
          f"(forward component {forward:+.3f} m) ratio={ratio:.2f}")

    # T6f
    front1 = n.front_range()
    if front0 is not None and front1 is not None:
        print(f"T6f LIDAR-ODOM (informative): front range {front0:.3f} -> {front1:.3f} m, "
              f"decrease={front0 - front1:.3f} m vs odom forward {forward:.3f} m")
    else:
        print("T6f LIDAR-ODOM: no valid beam at 0 deg (lidar frame may not face +x); skipped")

    # T6e angular
    yaw_start = n.yaw_unwrapped
    c0 = n.clock
    print(f"=== T6e: publishing w={W_CMD} rad/s for {CMD_SECONDS} s ===", flush=True)
    n.spin_for(CMD_SECONDS, publish=twist(w=W_CMD))
    c1 = n.clock
    n.spin_for(SETTLE_SECONDS, publish=twist())
    dyaw = n.yaw_unwrapped - yaw_start
    expected = W_CMD * (c1 - c0)
    ratio = dyaw / expected if expected > 0 else float("nan")
    print(f"T6e ANGULAR {grade(ratio)}: sim_time_active={c1 - c0:.2f}s expected={math.degrees(expected):.1f} deg "
          f"measured={math.degrees(dyaw):.1f} deg ratio={ratio:.2f}")

    n.spin_for(1.0, publish=twist())
    print("=== T6 done (robot stopped) ===")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
