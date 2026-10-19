#!/usr/bin/env python3
"""Diagnose /scan vs /point_cloud from the RTX lidar (started by ./bprime.sh foxy scan while ./bprime.sh isaac t6 runs).

Question answered: does the lidar see the walls (point cloud) while the LaserScan conversion is empty,
or does the lidar itself return nothing?  Robot sits inside a 4 m x 4 m box, so every horizontal beam
should hit a wall between 2.0 m and ~2.9 m (or the pillar at ~1.3 m).
"""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, PointCloud2

BE = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)


class Diag(Node):
    def __init__(self):
        super().__init__("bprime_scan_diag")
        self.scans, self.clouds = [], []
        self.create_subscription(LaserScan, "scan", lambda m: self.scans.append(m), BE)
        self.create_subscription(PointCloud2, "point_cloud", lambda m: self.clouds.append(m), BE)


def scan_report(m, k):
    r = np.asarray(m.ranges, dtype=np.float64)
    n = len(r)
    nan, inf = int(np.isnan(r).sum()), int(np.isinf(r).sum())
    fin = r[np.isfinite(r)]
    zero = int((fin == 0.0).sum())
    below = int(((fin > 0.0) & (fin < m.range_min)).sum())
    above = int((fin > m.range_max).sum())
    valid = fin[(fin >= m.range_min) & (fin <= m.range_max)]
    print(f"SCAN[{k}] stamp={m.header.stamp.sec}.{m.header.stamp.nanosec:09d} n={n} "
          f"angle_min={math.degrees(m.angle_min):.2f} angle_max={math.degrees(m.angle_max):.2f} "
          f"inc={math.degrees(m.angle_increment):.4f} deg (implied {math.degrees((m.angle_max - m.angle_min) / max(n - 1, 1)):.4f}) "
          f"time_inc={m.time_increment:.2e} scan_time={m.scan_time:.3f} range=[{m.range_min}, {m.range_max}]")
    print(f"   nan={nan} inf={inf} zero={zero} (0<r<min)={below} (r>max)={above} valid={len(valid)} "
          f"intensities={len(m.intensities)}")
    if len(valid):
        print(f"   valid ranges: min={valid.min():.3f} median={np.median(valid):.3f} max={valid.max():.3f}")
    idx = np.linspace(0, n - 1, 12).astype(int) if n else []
    print("   sample [index:value] " + " ".join(f"{i}:{r[i]:.3f}" for i in idx))


def cloud_report(m, k):
    fields = {f.name: f for f in m.fields}
    print(f"CLOUD[{k}] stamp={m.header.stamp.sec}.{m.header.stamp.nanosec:09d} frame={m.header.frame_id} "
          f"width={m.width} height={m.height} point_step={m.point_step} "
          f"fields={[(f.name, f.offset, f.datatype) for f in m.fields]}")
    if not all(a in fields for a in ("x", "y", "z")) or m.width * m.height == 0:
        print("   no x/y/z points")
        return
    npts = m.width * m.height
    buf = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(npts, m.point_step)

    def col(name):
        f = fields[name]
        return buf[:, f.offset:f.offset + 4].copy().view(np.float32).reshape(-1).astype(np.float64)

    x, y, z = col("x"), col("y"), col("z")
    d = np.hypot(x, y)
    ok = np.isfinite(d)
    d, z = d[ok], z[ok]
    walls = int(((d >= 1.9) & (d <= 3.0)).sum())
    pillar = int(((d >= 1.1) & (d < 1.9)).sum())
    print(f"   points={npts} finite={int(ok.sum())} horiz_dist min={d.min() if d.size else float('nan'):.3f} "
          f"median={np.median(d) if d.size else float('nan'):.3f} max={d.max() if d.size else float('nan'):.3f} "
          f"z range=[{z.min() if z.size else float('nan'):.3f}, {z.max() if z.size else float('nan'):.3f}]")
    print(f"   points at wall distance (1.9-3.0 m)={walls}  at pillar distance (1.1-1.9 m)={pillar}")


def main():
    rclpy.init()
    n = Diag()
    end = time.time() + 300.0
    while time.time() < end and (len(n.scans) < 3 or len(n.clouds) < 3):
        rclpy.spin_once(n, timeout_sec=0.05)
    print(f"received scans={len(n.scans)} clouds={len(n.clouds)} (waited up to 300 s)")
    for k, m in enumerate(n.scans[-3:]):
        scan_report(m, k)
    for k, m in enumerate(n.clouds[-3:]):
        cloud_report(m, k)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
