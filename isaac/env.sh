# Sourced INSIDE the Isaac Sim container (bprime.sh does this for you).
# Appends the Foxy internal ROS 2 libraries to LD_LIBRARY_PATH exactly once, as described in
# "Running ROS without a System Level Install" (Isaac Sim 4.1 docs, install_ros.html).
FOXY_LIB="/isaac-sim/exts/omni.isaac.ros2_bridge/foxy/lib"

if [ ! -d "${FOXY_LIB}" ]; then
  echo "[B'] ERROR: ${FOXY_LIB} not found -> T2 FAIL (no Foxy internal libraries in this image)."
  # shellcheck disable=SC2317
  return 1 2>/dev/null || exit 1
fi

case ":${LD_LIBRARY_PATH:-}:" in
  *":${FOXY_LIB}:"*) ;;
  *) export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+${LD_LIBRARY_PATH}:}${FOXY_LIB}" ;;
esac

export ROS_DISTRO=foxy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/bprime/config/fastdds_udp.xml
echo "[B'] ROS_DISTRO=${ROS_DISTRO} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"
echo "[B'] LD_LIBRARY_PATH=${LD_LIBRARY_PATH}"
