#!/usr/bin/env bash
# B' test suite - single entry point.
# Validates: Isaac Sim 4.1 container (ROS 2 Foxy internal libraries) <-> ROS 2 Foxy container.
# Run `./bprime.sh help` for the command list; see README.md for the full procedure.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# bprime.sh drives Docker, so it must run on the HOST, never inside a test container.
if [ -f /.dockerenv ] || [ -d /isaac-sim ]; then
  printf '\033[31mERROR:\033[0m bprime.sh must run on the host, not inside a container.\n' >&2
  printf 'You are in a container shell (prompt root@...:/isaac-sim#). Type "exit", then rerun the command.\n' >&2
  exit 1
fi
# shellcheck source=config/bprime.env
source "${HERE}/config/bprime.env"
mkdir -p "${BPRIME_LOG_DIR}"
DDS_PROFILE_IN_CONTAINER="/bprime/config/fastdds_udp.xml"
STREAM_URL="http://127.0.0.1:8211/streaming/webrtc-client?server=127.0.0.1"

say()  { printf '\n\033[1m== %s\033[0m\n' "$*"; }
die()  { printf '\033[31mERROR:\033[0m %s\n' "$*" >&2; exit 1; }
need_docker() { docker info >/dev/null 2>&1 || die "docker is not usable by this user (install it, or log out/in after install-docker)."; }
exists()  { docker ps -a --format '{{.Names}}' | grep -qx "$1"; }
running() { docker ps --format '{{.Names}}' | grep -qx "$1"; }

usage() {
  cat <<'EOF'
Usage: ./bprime.sh <command> [options]

Setup (once per machine)
  preflight              Read-only host report (OS, GPU, driver, Docker, RAM, disk)
  install-docker         Install Docker Engine (Ubuntu 20.04 only; skip if Docker exists)
  install-toolkit        Install/configure NVIDIA Container Toolkit (skip if already working)
  gpu-check              T0  GPU visible inside a container
  pull                   T1a Pull both images and record their digests
  inspect                T2  Foxy internal libraries present in the Isaac Sim image

Tests (terminal A = Isaac side, terminal B = Foxy side)
  isaac base [opts]      A: BACKEND/GRAPH/CAMERA/ASSETS + /clock, /cmd_vel   (opts: --duration S --no-camera --no-assets)
  foxy  base             B: T3 /clock, T4 /cmd_vel, T5 camera
  isaac t6   [opts]      A: Jetbot + RTX lidar + /odom, /tf                  (opts: --duration S --lidar-config NAME --no-pointcloud)
                         Both isaac base/t6 accept --livestream: view the scene in Chrome (WebRTC, port 8211)
  foxy  t6               B: T6a-T6f scan / odom / tf / motion checks
  foxy  scan             B: /scan vs /point_cloud diagnosis (optional)
  isaac t7   [opts]      A: T6 robot + RGB camera inside an apartment/office scene (README section 11)
                         opts: --scene office|<GRScenes id>|<usd> --variant navigation|raw --duration S
                               --spawn X Y --floor-z Z --hide-hdr --add-dome-light --add-colliders
                               --ground-plane --no-camera --cam-res WxH --cam-hfov DEG --pointcloud --livestream
  foxy  t7   [--no-camera]  B: T7c scan/TF, T7g camera, T7h rotate/forward inside the scene

GRScenes-100 (needs GRSCENES_ROOT in config/bprime.env)
  grscenes-inventory [--ids ID ...] [--variant raw|navigation]
                         Per-scene floor area, object categories -> ~/bprime_logs/grscenes_*.csv
  grscenes-prep ID ...   Re-create start_result_navigation.usd with Isaac Sim 4.1 (InternUtopia
                         preprocess.py -n); the shipped file is kept as start_result_navigation.shipped.usd

Viewing (WebRTC livestream, open http://127.0.0.1:8211/streaming/webrtc-client?server=127.0.0.1)
  isaac stream           A: full Isaac Sim app (empty stage) streamed via runheadless.webrtc.sh

Utilities
  shell isaac|foxy       Interactive shell in a container (environment already set)
  collect                Pack all logs into ~/bprime_report_<host>_<time>.tar.gz
  stop                   Stop and remove the test containers
EOF
}

# ------------------------------------------------------------------ setup
cmd_preflight() {
  local out="${BPRIME_LOG_DIR}/00_preflight.log"
  {
    echo "=== date / host ==="; date; hostname
    echo "=== OS ==="; lsb_release -ds 2>/dev/null; uname -r
    echo "=== GPU / driver ==="; nvidia-smi --query-gpu=name,driver_version,memory.total,memory.used --format=csv 2>&1
    echo "=== PRIME mode (hybrid-graphics laptops) ==="
    if command -v prime-select >/dev/null 2>&1; then prime-select query; else echo "prime-select not installed"; fi
    echo "=== RAM / swap ==="; free -h
    echo "=== Disk ==="; df -h / /var/lib 2>/dev/null
    echo "=== Docker ==="
    if command -v docker >/dev/null 2>&1; then docker --version; systemctl is-active docker || true; else echo "docker: NOT installed"; fi
    echo "=== NVIDIA Container Toolkit ==="
    if command -v nvidia-ctk >/dev/null 2>&1; then nvidia-ctk --version | head -1; else echo "nvidia-ctk: NOT installed"; fi
    echo "=== Other Isaac Sim / ROS containers running (stop them before testing) ==="
    if command -v docker >/dev/null 2>&1; then docker ps --format '{{.Names}}\t{{.Image}}' 2>/dev/null || true; fi
    echo "=== Host ROS processes (should be none) ==="; pgrep -af "ros2|_ros2_daemon" || echo "none"
    echo "=== Network interfaces (DDS needs one UP interface besides lo) ==="; ip -br addr
  } 2>&1 | tee "${out}"
  say "Saved ${out}"
}

cmd_install_docker() {
  [[ "$(lsb_release -cs)" == "focal" ]] || die "install-docker is written for Ubuntu 20.04 (focal); detected $(lsb_release -cs). Use the Docker docs for this OS, or skip if Docker already works."
  if command -v docker >/dev/null 2>&1; then
    echo "Docker already installed: $(docker --version)"
  else
    # Docker's apt-repository method; the focal repository is frozen but still served.
    sudo apt-get update
    sudo apt-get install -y ca-certificates curl
    sudo install -m 0755 -d /etc/apt/keyrings
    sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
    sudo chmod a+r /etc/apt/keyrings/docker.asc
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu focal stable" \
      | sudo tee /etc/apt/sources.list.d/docker.list >/dev/null
    sudo apt-get update
    sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
  fi
  getent group docker >/dev/null || sudo groupadd docker
  sudo usermod -aG docker "$USER"
  say "Done. Log out and back in, then check: docker run --rm hello-world"
}

cmd_install_toolkit() {
  need_docker
  if ! command -v nvidia-ctk >/dev/null 2>&1; then
    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
      | sudo gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
    curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
      | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
      | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list >/dev/null
    sudo apt-get update
    sudo apt-get install -y nvidia-container-toolkit
  fi
  sudo nvidia-ctk runtime configure --runtime=docker
  sudo systemctl restart docker
  cmd_gpu_check
}

cmd_gpu_check() {
  need_docker
  say "T0: GPU inside a container"
  if docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi 2>&1 | tee "${BPRIME_LOG_DIR}/01_gpu_check.log"; then
    echo "T0 PASS if the table above lists your NVIDIA GPU."
  else
    echo "T0 FAIL: fix Docker / NVIDIA Container Toolkit first (./bprime.sh install-toolkit)."; return 1
  fi
}

cmd_pull() {
  need_docker
  say "T1a: pull ${FOXY_IMAGE}"
  docker pull "${FOXY_IMAGE}"
  say "T1a: pull ${ISAAC_IMAGE}"
  if ! docker pull "${ISAAC_IMAGE}"; then
    echo "If the error is 'unauthorized': docker login nvcr.io  (Username: \$oauthtoken, Password: your NGC API key), then rerun."
    echo "Any other error (e.g. 'manifest unknown'): T1a FAIL - the 4.1 image is not available."
    return 1
  fi
  docker images --digests | grep -E "isaac-sim|osrf/ros" | tee "${BPRIME_LOG_DIR}/02_image_digests.txt"
  echo "T1a PASS. Digests saved to ${BPRIME_LOG_DIR}/02_image_digests.txt"
}

# ------------------------------------------------------------------ containers
isaac_run_args() {
  local c="${ISAAC_CACHE_ROOT}"
  mkdir -p "${c}/cache/kit" "${c}/cache/ov" "${c}/cache/pip" "${c}/cache/glcache" \
           "${c}/cache/computecache" "${c}/logs" "${c}/data" "${c}/documents"
  # Base command: Isaac Sim container installation guide. Added: Foxy internal libraries,
  # UDP-only Fast DDS, isolated domain, read-only suite mount, writable log folder.
  ISAAC_ARGS=(--name "${ISAAC_CONTAINER}" --rm --entrypoint bash
    --runtime=nvidia --gpus all --network=host
    -e ACCEPT_EULA=Y -e ROS_DISTRO=foxy -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    -e "ROS_DOMAIN_ID=${BPRIME_DOMAIN_ID}" -e "FASTRTPS_DEFAULT_PROFILES_FILE=${DDS_PROFILE_IN_CONTAINER}"
    -v "${c}/cache/kit:/isaac-sim/kit/cache:rw"
    -v "${c}/cache/ov:/root/.cache/ov:rw"
    -v "${c}/cache/pip:/root/.cache/pip:rw"
    -v "${c}/cache/glcache:/root/.cache/nvidia/GLCache:rw"
    -v "${c}/cache/computecache:/root/.nv/ComputeCache:rw"
    -v "${c}/logs:/root/.nvidia-omniverse/logs:rw"
    -v "${c}/data:/root/.local/share/ov/data:rw"
    -v "${c}/documents:/root/Documents:rw"
    -v "${HERE}:/bprime:ro"
    -v "${BPRIME_LOG_DIR}:/bprime_logs:rw")
}

# GRScenes-100 dataset mounted at /grscenes (relative symlinks Materials/models inside each scene
# folder keep working). MDL_SYSTEM_PATH points at its Materials folder, as the dataset card asks.
have_grscenes() { [ -n "${GRSCENES_ROOT}" ] && [ -d "${GRSCENES_ROOT}/scenes" ]; }
need_grscenes() {
  have_grscenes || die "GRSCENES_ROOT='${GRSCENES_ROOT}' is not a folder with scenes/ inside. Set it in config/bprime.env (README section 11)."
  [ -d "${GRSCENES_ROOT}/Materials" ] || echo "WARNING: ${GRSCENES_ROOT}/Materials not found - MDL materials will not resolve."
}
grscenes_mount() {   # $1 = ro | rw
  ISAAC_ARGS+=(-v "${GRSCENES_ROOT}:/grscenes:${1}" -e MDL_SYSTEM_PATH=/grscenes/Materials)
}

# T7d: host-side VRAM sampling (1 Hz) while the Isaac container runs, plus a T7a error summary.
VRAM_PID=""
T7_REPORTED=""
vram_start() {
  local csv="$1"
  : > "${csv}"
  if command -v nvidia-smi >/dev/null 2>&1; then
    nvidia-smi --query-gpu=memory.used,memory.total --format=csv,noheader,nounits -lms 1000 >> "${csv}" 2>/dev/null &
    VRAM_PID=$!
  fi
}
t7_report() {
  [ -n "${T7_REPORTED}" ] && return 0
  T7_REPORTED=1
  local csv="${BPRIME_LOG_DIR}/53_vram_t7.csv" log="${BPRIME_LOG_DIR}/50_isaac_t7.log"
  if [ -n "${VRAM_PID}" ]; then kill "${VRAM_PID}" 2>/dev/null || true; wait "${VRAM_PID}" 2>/dev/null || true; fi
  {
    say "T7 host-side summary"
    if [ -s "${csv}" ]; then
      awk -F', *' 'NR==1{base=$1} {if($1+0>max)max=$1+0; tot=$2+0}
        END{v=(max<=0.9*tot)?"PASS":((max<=tot-200)?"WARN":"FAIL");
            printf "T7d VRAM %s: peak %d MiB of %d MiB (%.0f%%), before start %d MiB, Isaac adds ~%d MiB (samples: %d)\n",
                   v, max, tot, 100*max/tot, base, max-base, NR}' "${csv}"
    else
      echo "T7d VRAM: no samples (nvidia-smi not found on the host?)"
    fi
    if [ -f "${log}" ]; then
      local n_err n_mdl
      n_err=$(grep -c '\[Error\]' "${log}" || true)
      n_mdl=$(grep '\[Error\]' "${log}" | grep -ci 'mdl' || true)
      echo "T7a log: ${n_err} [Error] lines, ${n_mdl} of them mention MDL (details: ${log})"
      if [ "${n_err}" != "0" ]; then
        echo "     most frequent errors:"
        grep '\[Error\]' "${log}" | sed -E 's/^.*\[Error\]/[Error]/' | cut -c1-160 | sort | uniq -c | sort -rn | head -5
      fi
    fi
  } | tee "${BPRIME_LOG_DIR}/55_t7_host_summary.log"
}

ensure_no_isaac() {
  if exists "${ISAAC_CONTAINER}"; then
    die "container ${ISAAC_CONTAINER} already exists (another test still running?). Stop it with: ./bprime.sh stop"
  fi
}

foxy_mount_src() {
  docker inspect -f '{{range .Mounts}}{{if eq .Destination "/bprime"}}{{.Source}}{{end}}{{end}}' "${FOXY_CONTAINER}" 2>/dev/null
}

ensure_foxy() {
  need_docker
  if running "${FOXY_CONTAINER}"; then
    # Reuse the long-lived Foxy container only if it mounts THIS suite folder. A container started
    # from an older copy of the suite (other path, or a folder deleted and re-cloned) sees stale files.
    if [ "$(foxy_mount_src)" = "${HERE}" ] && docker exec "${FOXY_CONTAINER}" test -f /bprime/foxy/check_t6.py; then
      return 0
    fi
    echo "Foxy container mounts '$(foxy_mount_src)' (stale or not '${HERE}') -> recreating it."
  fi
  exists "${FOXY_CONTAINER}" && docker rm -f "${FOXY_CONTAINER}" >/dev/null
  docker run -d --name "${FOXY_CONTAINER}" --rm --network=host \
    -e "ROS_DOMAIN_ID=${BPRIME_DOMAIN_ID}" -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e "FASTRTPS_DEFAULT_PROFILES_FILE=${DDS_PROFILE_IN_CONTAINER}" \
    -v "${HERE}:/bprime:ro" "${FOXY_IMAGE}" sleep infinity >/dev/null
  echo "Started Foxy container ${FOXY_CONTAINER}."
}

cmd_inspect() {
  need_docker; ensure_no_isaac; isaac_run_args
  say "T2: inspect ${ISAAC_IMAGE}"
  docker run "${ISAAC_ARGS[@]}" "${ISAAC_IMAGE}" /bprime/isaac/inspect.sh 2>&1 | tee "${BPRIME_LOG_DIR}/10_inspect.log"
}

cmd_isaac() {
  local which="${1:-}"; shift || true
  local script log
  case "${which}" in
    base) script=/bprime/isaac/test_base.py; log=20_isaac_base.log ;;
    t6)   script=/bprime/isaac/test_t6.py;   log=30_isaac_t6.log ;;
    t7)   script=/bprime/isaac/test_t7.py;   log=50_isaac_t7.log ;;
    stream) cmd_isaac_stream; return ;;
    *) die "usage: ./bprime.sh isaac base|t6|t7|stream [options]" ;;
  esac
  need_docker; ensure_no_isaac; isaac_run_args
  if [ "${which}" = "t7" ]; then
    if have_grscenes; then grscenes_mount ro; echo "GRScenes mounted read-only: ${GRSCENES_ROOT} -> /grscenes"; fi
    vram_start "${BPRIME_LOG_DIR}/53_vram_t7.csv"
    trap t7_report EXIT
  fi
  if [[ " $* " == *" --livestream "* ]]; then
    echo "Livestream on: when the log shows 'simulation playing', open in Chrome/Chromium:"
    echo "  ${STREAM_URL}   (press the red Play button; one viewer at a time)"
  fi
  say "Isaac side '${which}' (log: ${BPRIME_LOG_DIR}/${log}). First start compiles shaders and can take minutes. Ctrl+C to stop."
  docker run -it "${ISAAC_ARGS[@]}" "${ISAAC_IMAGE}" -c \
    'source /bprime/isaac/env.sh && cd /isaac-sim && ./python.sh "$0" "$@" 2>&1 | tee "/bprime_logs/'"${log}"'"' \
    "${script}" "$@"
}

cmd_isaac_stream() {
  need_docker; ensure_no_isaac; isaac_run_args
  say "Isaac Sim app streamed via WebRTC (log: ${BPRIME_LOG_DIR}/40_isaac_stream.log). Ctrl+C or File > Exit to stop."
  echo "When loading has finished, open in Chrome/Chromium: ${STREAM_URL}"
  docker run -it "${ISAAC_ARGS[@]}" "${ISAAC_IMAGE}" -c \
    'source /bprime/isaac/env.sh && cd /isaac-sim && ./runheadless.webrtc.sh 2>&1 | tee /bprime_logs/40_isaac_stream.log'
}

cmd_foxy() {
  local which="${1:-}"
  local inner log
  case "${which}" in
    base) inner='bash /bprime/foxy/check_base.sh'; log=21_foxy_base.log ;;
    t6)   inner='ros2 daemon stop >/dev/null 2>&1 || true; python3 /bprime/foxy/check_t6.py'; log=31_foxy_t6.log ;;
    t7)   local extra=""
          [ "${2:-}" = "--no-camera" ] && extra=" --no-camera"
          inner="ros2 daemon stop >/dev/null 2>&1 || true; python3 /bprime/foxy/check_t7.py${extra}"; log=51_foxy_t7.log ;;
    scan) inner='python3 /bprime/foxy/diag_scan.py'; log=32_foxy_scan.log ;;
    *) die "usage: ./bprime.sh foxy base|t6|t7|scan" ;;
  esac
  ensure_foxy
  say "Foxy side '${which}' (log: ${BPRIME_LOG_DIR}/${log}). Waits for the Isaac side automatically."
  docker exec "${FOXY_CONTAINER}" bash -c "set +u; source /opt/ros/foxy/setup.bash; set -u; export PYTHONUNBUFFERED=1; ${inner}" \
    2>&1 | tee "${BPRIME_LOG_DIR}/${log}"
}

cmd_grscenes_inventory() {
  need_grscenes; need_docker; ensure_no_isaac; isaac_run_args; grscenes_mount ro
  say "GRScenes inventory (log: ${BPRIME_LOG_DIR}/52_grscenes_inventory.log). Results: ${BPRIME_LOG_DIR}/grscenes_*.csv"
  docker run -it "${ISAAC_ARGS[@]}" "${ISAAC_IMAGE}" -c \
    'source /bprime/isaac/env.sh && cd /isaac-sim && ./python.sh "$0" "$@" 2>&1 | tee /bprime_logs/52_grscenes_inventory.log' \
    /bprime/isaac/grscenes_inventory.py "$@"
}

cmd_grscenes_prep() {
  [ $# -ge 1 ] || die "usage: ./bprime.sh grscenes-prep <scene_id> [<scene_id> ...]"
  need_grscenes
  local tool="${INTERNUTOPIA_DIR}/toolkits/grscenes_scripts/preprocess.py"
  [ -f "${tool}" ] || die "${tool} not found. Clone it once: git clone --depth 1 https://github.com/InternRobotics/InternUtopia ${INTERNUTOPIA_DIR}"
  local files=() id d
  for id in "$@"; do
    id="${id%_usd}"; d="${GRSCENES_ROOT}/scenes/${id}_usd"
    [ -f "${d}/start_result_raw.usd" ] || die "${d}/start_result_raw.usd not found"
    if [ -f "${d}/start_result_navigation.usd" ] && [ ! -f "${d}/start_result_navigation.shipped.usd" ]; then
      cp -p "${d}/start_result_navigation.usd" "${d}/start_result_navigation.shipped.usd"
      echo "backup: ${d}/start_result_navigation.shipped.usd"
    fi
    files+=("/grscenes/scenes/${id}_usd/start_result_raw.usd")
  done
  need_docker; ensure_no_isaac; isaac_run_args; grscenes_mount rw
  ISAAC_ARGS+=(-v "$(dirname "${tool}"):/grscenes_scripts:ro")
  say "Preprocess for navigation with Isaac Sim 4.1 (log: ${BPRIME_LOG_DIR}/54_grscenes_prep.log)"
  docker run -it "${ISAAC_ARGS[@]}" "${ISAAC_IMAGE}" -c \
    'source /bprime/isaac/env.sh && cd /isaac-sim && { ./python.sh -c "import tqdm" 2>/dev/null || ./python.sh -m pip install tqdm; } && ./python.sh /grscenes_scripts/preprocess.py -n -f "$@" 2>&1 | tee /bprime_logs/54_grscenes_prep.log' \
    _ "${files[@]}"
}

cmd_shell() {
  case "${1:-}" in
    isaac)
      need_docker
      if running "${ISAAC_CONTAINER}"; then
        docker exec -it "${ISAAC_CONTAINER}" bash -c 'source /bprime/isaac/env.sh; cd /isaac-sim; exec bash'
      else
        isaac_run_args
        docker run -it "${ISAAC_ARGS[@]}" "${ISAAC_IMAGE}" -c 'source /bprime/isaac/env.sh; cd /isaac-sim; exec bash'
      fi ;;
    foxy)
      ensure_foxy
      docker exec -it "${FOXY_CONTAINER}" bash -c 'source /opt/ros/foxy/setup.bash && exec bash' ;;
    *) die "usage: ./bprime.sh shell isaac|foxy" ;;
  esac
}

cmd_collect() {
  local out tmp
  out="${HOME}/bprime_report_$(hostname)_$(date +%Y%m%d_%H%M%S).tar.gz"
  tmp="$(mktemp -d)"
  cp -r "${BPRIME_LOG_DIR}" "${tmp}/bprime_logs" 2>/dev/null || true
  if [ -d "${ISAAC_CACHE_ROOT}/logs" ]; then
    mkdir -p "${tmp}/kit_logs"
    find "${ISAAC_CACHE_ROOT}/logs" -type f -name "*.log" -printf "%T@ %p\n" 2>/dev/null \
      | sort -nr | head -3 | cut -d' ' -f2- | while read -r f; do cp "$f" "${tmp}/kit_logs/"; done
  fi
  tar -czf "${out}" -C "${tmp}" .
  rm -rf "${tmp}"
  say "Report: ${out}"
}

cmd_stop() {
  for c in "${ISAAC_CONTAINER}" "${FOXY_CONTAINER}"; do
    if exists "$c"; then docker rm -f "$c" >/dev/null && echo "removed $c"; fi
  done
}

case "${1:-help}" in
  preflight)       cmd_preflight ;;
  install-docker)  cmd_install_docker ;;
  install-toolkit) cmd_install_toolkit ;;
  gpu-check)       cmd_gpu_check ;;
  pull)            cmd_pull ;;
  inspect)         cmd_inspect ;;
  isaac)           shift; cmd_isaac "$@" ;;
  foxy)            shift; cmd_foxy "$@" ;;
  shell)           shift; cmd_shell "$@" ;;
  grscenes-inventory) shift; cmd_grscenes_inventory "$@" ;;
  grscenes-prep)   shift; cmd_grscenes_prep "$@" ;;
  collect)         cmd_collect ;;
  stop)            cmd_stop ;;
  help|-h|--help)  usage ;;
  *)               usage; exit 1 ;;
esac
