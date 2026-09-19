#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="${RAICOM_ROOT:-/home/shigure/archive/raicom}"
WS="${RAICOM_WS:-$ROOT/ros2_ws}"
IMAGE="${RAICOM_DOCKER_IMAGE:-raicom-humble-x11:compiled}"
NAME="${RAICOM_CONTAINER:-raicom-sim}"
LABEL_KEY="raicom.control"
LABEL_VALUE="pyqt"
LOG_DIR="/tmp/raicom_gui_logs"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PACKAGE_DIR="$(cd "$PROJECT_DIR/.." && pwd)"
PROC_PATTERN='[r]os2 launch|[r]viz2|[i]gn gazebo|[i]gnition gazebo|[g]z sim|[p]arameter_bridge|[r]obot_state_publisher|[j]oint_state_publisher|[j]oint_state_publisher_gui|[c]hassis_motion_test|[r]oute_sequence_runner|[r]ecognition_.*commander|[o]rthogonal_centerline_controller|[c]enterline_planner|[y]olo_camera_detector|[o]dom_tf_broadcaster|[s]can_frame_republisher|[c]lock_republisher|[j]oint_state_republisher|[c]ollision_safety_node'

usage() {
  cat <<'EOF'
Usage: scripts/raicom_sim_control.sh <command>

Commands:
  start-sim       Start Gazebo simulation in the RAICOM arena.
  stop-sim        Stop Gazebo/navigation launch processes, keep the container.
  stop-nodes      Stop all ROS/Gazebo/RViz processes inside the control container.
  restart-nodes   Stop all nodes, then start the default simulation again.
  open-rviz       Open RViz in the control container.
  close-rviz      Close RViz.
  urdf-rviz       Show the robot URDF model in RViz only.
  gazebo-map      Show the Gazebo arena map only.
  chassis-test    Start open-world simulation and run the mecanum chassis test.
  centerline-plan Open the Qt centerline planner/editor.
  centerline-lap  Start the centerline one-lap controller test.
  competition-lap Start competition one-lap navigation with YOLO detection.
  centerline-tune Headless tune centerline speed/smoothing parameters.
  nav-lap         Start pure Nav2 fallback and run the 101,102,103 mission sequence.
  status          Print container and process status.
EOF
}

container_exists() {
  docker inspect "$NAME" >/dev/null 2>&1
}

container_running() {
  [[ "$(docker inspect -f '{{.State.Running}}' "$NAME" 2>/dev/null || true)" == "true" ]]
}

container_label() {
  docker inspect -f "{{ index .Config.Labels \"$LABEL_KEY\" }}" "$NAME" 2>/dev/null || true
}

is_control_container() {
  container_running && [[ "$(container_label)" == "$LABEL_VALUE" ]]
}

require_workspace() {
  if [[ ! -d "$WS/src" ]]; then
    echo "ERROR: workspace not found: $WS" >&2
    exit 2
  fi
}

require_image() {
  if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "ERROR: docker image not found: $IMAGE" >&2
    docker images --format '{{.Repository}}:{{.Tag}}\t{{.Size}}' | grep -Ei 'humble|raicom|ros' >&2 || true
    exit 2
  fi
}

remove_container() {
  if container_exists; then
    docker rm -f "$NAME" >/dev/null 2>&1 || true
  fi
}

ensure_control_container() {
  require_workspace
  require_image

  if is_control_container; then
    return
  fi

  if container_running; then
    echo "Replacing existing non-control container: $NAME"
    remove_container
  fi

  xhost +local:docker >/dev/null 2>&1 || true

  docker run -d --rm --name "$NAME" \
    --label "$LABEL_KEY=$LABEL_VALUE" \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e ROS_DOMAIN_ID=42 \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e RMW_FASTRTPS_USE_SHM=0 \
    -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -e MESA_GL_VERSION_OVERRIDE=3.3 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v "$WS:/ws" \
    -w /ws \
    --network host \
    "$IMAGE" \
    bash -lc 'mkdir -p /tmp/raicom_gui_logs; while true; do sleep 3600; done' >/dev/null

  echo "Started control container: $NAME"
}

run_in_container() {
  ensure_control_container
  docker exec "$NAME" bash -lc "$*"
}

find_host_pyqt_python() {
  local candidates=(
    "${RAICOM_HOST_PYTHON:-}"
    "$(command -v python3 2>/dev/null || true)"
    "/home/shigure/miniconda3/bin/python3"
    "/home/shigure/miniconda3/bin/python"
    "/usr/bin/python3"
  )

  local py
  for py in "${candidates[@]}"; do
    [[ -n "$py" && -x "$py" ]] || continue
    if "$py" - <<'PY' >/dev/null 2>&1
import PyQt5
PY
    then
      printf '%s\n' "$py"
      return 0
    fi
  done
  return 1
}

find_yolo_package_root() {
  local candidate
  for candidate in "$ROOT" "$PROJECT_DIR" "$PACKAGE_DIR"; do
    if [[ -d "$candidate/yolo_host" && -d "$candidate/models" ]]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done
  return 1
}

start_yolo_service() {
  local package_root
  if ! package_root="$(find_yolo_package_root)"; then
    echo "ERROR: cannot find yolo_host and models near ROOT=$ROOT" >&2
    exit 2
  fi

  local model="${YOLO_MODEL:-$package_root/models/recon_yolo26n_clean/best.pt}"
  if [[ ! -f "$model" ]]; then
    model="$package_root/models/old_yolov8n_raicom_detector/best.pt"
  fi
  if [[ ! -f "$model" ]]; then
    echo "ERROR: YOLO model not found. Set YOLO_MODEL or check $package_root/models." >&2
    exit 2
  fi

  local port="${YOLO_PORT:-8765}"
  local pid_file="/tmp/raicom_yolo_server.pid"
  local log_file="$LOG_DIR/yolo_server.log"
  mkdir -p "$LOG_DIR"

  if [[ -f "$pid_file" ]]; then
    local pid
    pid="$(cat "$pid_file" 2>/dev/null || true)"
    if [[ -n "$pid" ]] && kill -0 "$pid" >/dev/null 2>&1; then
      echo "YOLO server already running: pid=$pid port=$port log=$log_file"
      return
    fi
  fi

  : > "$log_file"
  RAICOM_YOLO_PACKAGE_ROOT="$package_root" \
    YOLO_MODEL="$model" \
    YOLO_PORT="$port" \
    YOLO_LOAD_ON_START=1 \
    setsid bash -lc 'cd "$RAICOM_YOLO_PACKAGE_ROOT" && exec bash yolo_host/start_packaged_yolo_server.sh' \
      >> "$log_file" 2>&1 < /dev/null &
  echo "$!" > "$pid_file"
  echo "started YOLO server: pid=$(cat "$pid_file") model=$model url=http://localhost:$port/detect log=$log_file"
}

start_process() {
  local proc="$1"
  local command="$2"

  ensure_control_container
  docker exec \
    -e RAICOM_PROC="$proc" \
    -e RAICOM_CMD="$command" \
    -e RAICOM_LOG_DIR="$LOG_DIR" \
    "$NAME" bash -lc '
set -Eeuo pipefail
mkdir -p "$RAICOM_LOG_DIR"
pid_file="/tmp/raicom_${RAICOM_PROC}.pid"
log_file="${RAICOM_LOG_DIR}/${RAICOM_PROC}.log"
if [[ -f "$pid_file" ]]; then
  pid="$(cat "$pid_file" 2>/dev/null || true)"
  if [[ -n "$pid" ]] && kill -0 "$pid" >/dev/null 2>&1; then
    echo "$RAICOM_PROC already running: pid=$pid log=$log_file"
    exit 0
  fi
fi
: > "$log_file"
setsid bash -lc "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && export LIBGL_ALWAYS_SOFTWARE=1 MESA_GL_VERSION_OVERRIDE=3.3 && $RAICOM_CMD" >> "$log_file" 2>&1 < /dev/null &
echo "$!" > "$pid_file"
echo "started $RAICOM_PROC: pid=$(cat "$pid_file") log=$log_file"
'
}

stop_process() {
  local proc="$1"
  if ! container_running; then
    echo "container is not running: $NAME"
    return
  fi

  if ! is_control_container; then
    echo "Removing non-control container while stopping $proc: $NAME"
    remove_container
    return
  fi

  docker exec -e RAICOM_PROC="$proc" "$NAME" bash -lc '
set +e
pid_file="/tmp/raicom_${RAICOM_PROC}.pid"
if [[ ! -f "$pid_file" ]]; then
  echo "$RAICOM_PROC is not tracked."
  exit 0
fi
pid="$(cat "$pid_file" 2>/dev/null || true)"
if [[ -n "$pid" ]] && kill -0 "$pid" >/dev/null 2>&1; then
  echo "stopping $RAICOM_PROC pid=$pid"
  kill -INT "-$pid" >/dev/null 2>&1 || kill -INT "$pid" >/dev/null 2>&1 || true
  sleep 1.5
  kill -TERM "-$pid" >/dev/null 2>&1 || true
  sleep 0.5
  kill -KILL "-$pid" >/dev/null 2>&1 || true
else
  echo "$RAICOM_PROC pid is not running."
fi
rm -f "$pid_file"
'
}

stop_all_nodes() {
  if ! container_running; then
    echo "container is not running: $NAME"
    return
  fi

  if ! is_control_container; then
    echo "Removing non-control container to avoid unsafe process cleanup: $NAME"
    remove_container
    return
  fi

  docker exec -e RAICOM_PROC_PATTERN="$PROC_PATTERN" "$NAME" bash -lc '
set +e
for pid_file in /tmp/raicom_*.pid; do
  [[ -e "$pid_file" ]] || continue
  pid="$(cat "$pid_file" 2>/dev/null || true)"
  if [[ -n "$pid" ]] && kill -0 "$pid" >/dev/null 2>&1; then
    kill -INT "-$pid" >/dev/null 2>&1 || kill -INT "$pid" >/dev/null 2>&1 || true
  fi
done
sleep 1.5
pkill -INT -f "$RAICOM_PROC_PATTERN" >/dev/null 2>&1 || true
sleep 1.0
pkill -TERM -f "$RAICOM_PROC_PATTERN" >/dev/null 2>&1 || true
sleep 0.5
pkill -KILL -f "$RAICOM_PROC_PATTERN" >/dev/null 2>&1 || true
rm -f /tmp/raicom_*.pid
echo "all tracked ROS/Gazebo/RViz processes stopped"
'
}

start_default_sim() {
  start_process sim 'ros2 launch mecanum_robot_sim keyboard_gazebo.launch.py gui:=true teleop:=false'
}

start_open_world_sim() {
  start_process sim 'ros2 launch mecanum_robot_sim keyboard_gazebo.launch.py gui:=true teleop:=false world_file:=open_test.world generate_arena:=false spawn_x:=0 spawn_y:=0 spawn_z:=0.02 spawn_yaw:=0'
}

start_navigation_lap() {
  start_process nav 'ros2 launch navigation navigation.launch.py map:=/ws/map01/compitation.yaml gui:=true rviz:=true teleop:=false enable_yolo_detector:=false enable_amcl:=false controller:=mppi nav_linear_speed:=0.45 nav_angular_speed:=1.80'
  start_process mission 'sleep 18; ros2 launch navigation auto_recon_mission.launch.py route_ids:=101,102,103'
}

start_urdf_rviz() {
  ensure_control_container
  stop_all_nodes
  start_process urdf_rviz 'ros2 launch mecanum_robot_sim display.launch.py'
}

start_gazebo_map() {
  ensure_control_container
  stop_all_nodes
  start_process gazebo_map 'ros2 launch mecanum_robot gazebo_map.launch.py gui:=true'
}

require_confirmed_centerline_track() {
  local track_file="$WS/src/navigation/config/centerline_track.yaml"
  if [[ ! -f "$track_file" ]]; then
    echo "ERROR: centerline track not found: $track_file" >&2
    echo "Open the centerline planner first and export a confirmed test route." >&2
    exit 2
  fi
  python3 - "$track_file" <<'PY'
import sys
import yaml

path = sys.argv[1]
with open(path, 'r', encoding='utf-8') as stream:
    data = yaml.safe_load(stream) or {}

segments = data.get('segments') or []
if data.get('confirmed_by_planner') is not True:
    print(
        'ERROR: centerline_track.yaml has not been confirmed by the planner.\n'
        'Open “中心线规划器”, inspect the rail route, then click “导出测试路线”.',
        file=sys.stderr,
    )
    sys.exit(2)
if len(segments) < 1:
    print('ERROR: confirmed centerline route has no segments.', file=sys.stderr)
    sys.exit(2)
print(f'confirmed centerline route: {data.get("route_id", "unknown")} ({len(segments)} segments)')
PY
}

start_centerline_lap() {
  require_confirmed_centerline_track
  start_process centerline 'ros2 launch navigation centerline_navigation.launch.py gui:=false rviz:=true teleop:=false track_file:=/ws/src/navigation/config/centerline_track.yaml auto_start:=true'
}

start_competition_lap() {
  require_confirmed_centerline_track
  start_yolo_service
  local yolo_url="http://localhost:${YOLO_PORT:-8765}/detect"
  start_process centerline "ros2 launch navigation centerline_navigation.launch.py gui:=true rviz:=true teleop:=false track_file:=/ws/src/navigation/config/centerline_track.yaml auto_start:=false controller_start_delay:=0.0 delayed_start:=true motion_start_delay:=30.0 rviz_start_delay:=2.0 enable_yolo_detector:=true yolo_url:=$yolo_url yolo_conf:=0.25 yolo_timeout:=5.0 rviz_config:=/ws/src/navigation/rviz/centerline_camera_raw.rviz"
}

run_centerline_tuning() {
  require_confirmed_centerline_track
  ensure_control_container
  stop_all_nodes
  docker exec \
    -e RAICOM_LOG_DIR="$LOG_DIR" \
    -e CENTERLINE_TUNE_TRIALS="${CENTERLINE_TUNE_TRIALS:-12}" \
    -e CENTERLINE_TUNE_TIMEOUT="${CENTERLINE_TUNE_TIMEOUT:-90}" \
    "$NAME" bash -lc '
set -Ee -o pipefail
mkdir -p "$RAICOM_LOG_DIR"
log_file="$RAICOM_LOG_DIR/centerline_tune.log"
: > "$log_file"
set +u
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
set -u
export LIBGL_ALWAYS_SOFTWARE=1 MESA_GL_VERSION_OVERRIDE=3.3
ros2 run navigation centerline_param_tuner.py \
  --base-track /ws/src/navigation/config/centerline_track.yaml \
  --output-track /ws/src/navigation/config/centerline_track_tuned.yaml \
  --results-file /ws/src/navigation/config/centerline_tuning_results.yaml \
  --trials "${CENTERLINE_TUNE_TRIALS:-12}" \
  --timeout "${CENTERLINE_TUNE_TIMEOUT:-90}" \
  "$@" 2>&1 | tee "$log_file"
' bash "$@"
  docker exec "$NAME" chown "$(id -u):$(id -g)" \
    /ws/src/navigation/config/centerline_track_tuned.yaml \
    /ws/src/navigation/config/centerline_tuning_results.yaml \
    >/dev/null 2>&1 || true
}

open_centerline_planner() {
  mkdir -p "$LOG_DIR"
  local pid_file="/tmp/raicom_planner.pid"
  local log_file="$LOG_DIR/planner.log"
  local python_bin
  if ! python_bin="$(find_host_pyqt_python)"; then
    echo "ERROR: cannot find a host Python with PyQt5 for centerline planner." >&2
    echo "Tried RAICOM_HOST_PYTHON, python3, /home/shigure/miniconda3/bin/python3, /usr/bin/python3." >&2
    return 2
  fi
  if [[ -f "$pid_file" ]]; then
    local pid
    pid="$(cat "$pid_file" 2>/dev/null || true)"
    if [[ -n "$pid" ]] && kill -0 "$pid" >/dev/null 2>&1; then
      echo "planner already running: pid=$pid log=$log_file"
      return
    fi
  fi
  : > "$log_file"
  echo "python=$python_bin" >> "$log_file"
  setsid "$python_bin" "$ROOT/scripts/centerline_planner.py" >> "$log_file" 2>&1 < /dev/null &
  echo "$!" > "$pid_file"
  echo "started planner: pid=$(cat "$pid_file") log=$log_file"
}

open_rviz() {
  ensure_control_container
  if docker exec "$NAME" bash -lc 'pgrep -f "[r]viz2" >/dev/null 2>&1'; then
    echo "RViz already running."
    return
  fi
  start_process rviz 'rviz2 -d /ws/src/navigation/rviz/centerline_light.rviz --ros-args -p use_sim_time:=true'
}

close_rviz() {
  stop_process rviz
  if is_control_container; then
    docker exec "$NAME" bash -lc 'pkill -INT -f "[r]viz2" >/dev/null 2>&1 || true; sleep 0.5; pkill -TERM -f "[r]viz2" >/dev/null 2>&1 || true'
  fi
}

run_chassis_test() {
  ensure_control_container
  stop_all_nodes
  start_open_world_sim
  echo "Waiting for open-world simulation, then running chassis test..."
  docker exec "$NAME" bash -lc 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 run mecanum_robot_sim chassis_motion_test.py'
}

print_status() {
  if ! container_running; then
    echo "container: stopped ($NAME)"
    return
  fi

  local label
  label="$(container_label)"
  echo "container: running ($NAME, image=$IMAGE, control=${label:-no})"

  if [[ "$label" == "$LABEL_VALUE" ]]; then
    docker exec -e RAICOM_PROC_PATTERN="$PROC_PATTERN" "$NAME" bash -lc '
set +e
	for proc in sim nav mission centerline planner rviz urdf_rviz gazebo_map; do
  pid_file="/tmp/raicom_${proc}.pid"
  if [[ -f "$pid_file" ]]; then
    pid="$(cat "$pid_file" 2>/dev/null || true)"
    if [[ -n "$pid" ]] && kill -0 "$pid" >/dev/null 2>&1; then
      echo "$proc: running pid=$pid"
    else
      echo "$proc: stale"
    fi
  else
    echo "$proc: stopped"
  fi
done
pgrep -af "$RAICOM_PROC_PATTERN" | sed "s/^/process: /" || true
'
  fi
}

main() {
  local command="${1:-}"
  case "$command" in
    start-sim)
      ensure_control_container
      stop_all_nodes
      start_default_sim
      ;;
    stop-sim)
      stop_process sim
      stop_process nav
      if is_control_container; then
        docker exec "$NAME" bash -lc 'pkill -INT -f "[i]gn gazebo|[i]gnition gazebo|[g]z sim|[p]arameter_bridge|[r]obot_state_publisher" >/dev/null 2>&1 || true'
      fi
      ;;
    stop-nodes)
      stop_all_nodes
      ;;
    restart-nodes)
      ensure_control_container
      stop_all_nodes
      start_default_sim
      ;;
    open-rviz)
      open_rviz
      ;;
    close-rviz)
      close_rviz
      ;;
    urdf-rviz)
      start_urdf_rviz
      ;;
    gazebo-map)
      start_gazebo_map
      ;;
    chassis-test)
      run_chassis_test
      ;;
    centerline-plan)
      open_centerline_planner
      ;;
    centerline-lap)
      ensure_control_container
      stop_all_nodes
      start_centerline_lap
      ;;
    competition-lap)
      ensure_control_container
      stop_all_nodes
      start_competition_lap
      ;;
    centerline-tune)
      shift || true
      run_centerline_tuning "$@"
      ;;
    nav-lap)
      ensure_control_container
      stop_all_nodes
      start_navigation_lap
      ;;
    status)
      print_status
      ;;
    -h|--help|help)
      usage
      ;;
    *)
      usage >&2
      exit 2
      ;;
  esac
}

main "$@"
