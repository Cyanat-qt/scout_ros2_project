#!/usr/bin/env bash
set -Eeuo pipefail

IMAGE="${RAICOM_DOCKER_IMAGE:-raicom-humble-x11:compiled}"
WS="${RAICOM_WS:-/home/shigure/archive/raicom/ros2_ws}"
NAME="${RAICOM_CONTAINER:-raicom-sim}"
MODE="${1:-sim}"
shift || true

if [[ ! -d "$WS/src" ]]; then
  echo "ERROR: workspace not found: $WS" >&2
  exit 2
fi

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "ERROR: docker image not found: $IMAGE" >&2
  echo "Available Humble/Raicom images:" >&2
  docker images --format '{{.Repository}}:{{.Tag}}\t{{.Size}}' | grep -Ei 'humble|raicom|ros' >&2 || true
  exit 2
fi

case "$MODE" in
  build)
    docker run --rm \
      -v "$WS:/ws" \
      -w /ws \
      "$IMAGE" \
      bash -c 'source /opt/ros/humble/setup.bash && colcon build --symlink-install --continue-on-error "$@"' bash "$@"
    exit $?
    ;;
  shell)
    docker rm -f "$NAME" >/dev/null 2>&1 || true
    exec docker run --rm -it --name "$NAME" \
      -e DISPLAY="${DISPLAY:-:0}" \
      -e ROS_DOMAIN_ID=42 \
      -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
      -e LIBGL_ALWAYS_SOFTWARE=1 \
      -e MESA_GL_VERSION_OVERRIDE=3.3 \
      -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
      -v "$WS:/ws" \
      -w /ws \
      --network host --pid host \
      "$IMAGE" \
      bash -c 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && exec bash'
    ;;
  sim)
    LAUNCH_CMD="ros2 launch mecanum_robot_sim keyboard_gazebo.launch.py gui:=true teleop:=false $*"
    ;;
  slam)
    LAUNCH_CMD="ros2 launch slam slam_gazebo.launch.py gui:=true teleop:=false rviz:=true $*"
    ;;
  nav)
    LAUNCH_CMD="ros2 launch navigation navigation.launch.py map:=/ws/map01/compitation.yaml gui:=true rviz:=true teleop:=false enable_yolo_detector:=true $*"
    ;;
  *)
    echo "Usage: $0 {build|sim|slam|nav|shell} [extra ros2 launch args]" >&2
    echo "  build: rebuild workspace inside Docker Humble image" >&2
    echo "  sim:   Gazebo + robot" >&2
    echo "  slam:  Gazebo + slam_toolbox + RViz" >&2
    echo "  nav:   Gazebo + Nav2 using /ws/map01/compitation.yaml" >&2
    echo "  shell: interactive container shell" >&2
    exit 2
    ;;
esac

xhost +local:docker >/dev/null 2>&1 || true
docker rm -f "$NAME" >/dev/null 2>&1 || true

exec docker run --rm --name "$NAME" \
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
  --network host --pid host \
  "$IMAGE" \
  bash -c "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && export LIBGL_ALWAYS_SOFTWARE=1 MESA_GL_VERSION_OVERRIDE=3.3 && $LAUNCH_CMD"
