#!/usr/bin/env bash
# T2 - read-only inspection of the Isaac Sim image (run by: ./bprime.sh inspect).
# shellcheck disable=SC2010,SC2012  # read-only listing of known library names
set -u
B=/isaac-sim/exts/omni.isaac.ros2_bridge
echo "=== Isaac Sim version ==="; cat /isaac-sim/VERSION 2>/dev/null || echo "(no VERSION file)"
echo "=== Container OS ==="; grep PRETTY_NAME /etc/os-release
echo "=== ros2_bridge backends present ==="; ls -d ${B}/*/lib 2>/dev/null || echo "(none found)"
echo "=== Foxy internal libraries ==="; ls ${B}/foxy/lib 2>&1 | head -40
echo "=== Fast DDS libraries in the Foxy backend ==="; ls ${B}/foxy/lib 2>/dev/null | grep -iE "fastrtps|fastcdr" || echo "(none)"
echo "=== RTX lidar configs (SLAMTEC / SICK) ==="; ls /isaac-sim/exts/omni.isaac.sensor/data/lidar_configs/SLAMTEC /isaac-sim/exts/omni.isaac.sensor/data/lidar_configs/SICK 2>&1
echo "=== GPU inside container ==="; nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv
if [ -d "${B}/foxy/lib" ]; then echo "T2 PASS: Foxy internal libraries present"; else echo "T2 FAIL: ${B}/foxy/lib missing"; fi
