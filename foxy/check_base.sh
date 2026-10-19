#!/usr/bin/env bash
# T3/T4/T5 on the Foxy side (run by: ./bprime.sh foxy base) while `./bprime.sh isaac base` runs.
# Order: T3 (Isaac->Foxy), T4 (Foxy->Isaac), then T5 (camera, informative).
# shellcheck disable=SC1091  # exists inside the Foxy container only
set +u; source /opt/ros/foxy/setup.bash; set -u
export PYTHONUNBUFFERED=1   # stdout is a pipe (docker exec | tee): keep output that timeout would otherwise lose
T="timeout -s INT"          # SIGINT lets the ros2 CLI exit cleanly and flush
echo "=== env ==="
echo "ROS_DISTRO=${ROS_DISTRO} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>} RMW=${RMW_IMPLEMENTATION:-<unset>} FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-<unset>}"
ros2 daemon stop >/dev/null 2>&1 || true
echo "=== waiting up to 300 s for /clock (start ./bprime.sh isaac base if it is not running) ==="
found=0
for _ in $(seq 1 60); do
  if $T 10 ros2 topic list 2>/dev/null | grep -qx "/clock"; then found=1; break; fi
  sleep 3
done
[ "${found}" = 1 ] || { echo "T3 FAIL: /clock never appeared"; exit 1; }
echo "=== T3a: topics visible from Foxy ==="
$T 20 ros2 topic list -t || echo "topic list timed out"
echo "=== T3b: /clock echo, 2 s (Isaac -> Foxy) ==="
$T 2 ros2 topic echo /clock rosgraph_msgs/msg/Clock || true
echo "=== T3c: /clock rate, 10 s ==="
$T 10 ros2 topic hz /clock || true
echo "=== T4: publish /cmd_vel for 10 s (Foxy -> Isaac). Isaac log must print 'TWIST PASS' ==="
$T 10 ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" || true
echo "=== T5a: /bprime/rgb rate, 15 s (informative) ==="
$T 15 ros2 topic hz /bprime/rgb || true
echo "=== T5b: /bprime/camera_info, 1 message ==="
$T 5 ros2 topic echo /bprime/camera_info sensor_msgs/msg/CameraInfo 2>/dev/null | sed -n '1,/^---$/p' || true
echo "=== done ==="
