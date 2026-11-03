#!/usr/bin/env bash
# Helper to build + run the ZRCar ROS 2 demo in Docker (Linux host).
#
#   ./run.sh build                 # build the image once
#   BOT_IP=192.168.1.46 ./run.sh bridge     # run the ROS<->bot bridge
#   ./run.sh teleop                # keyboard teleop -> /cmd_vel
#   ./run.sh obstacle [speed]      # autonomous brake/cruise demo
#   ./run.sh shell                 # a shell in the image (ros2 topic echo, etc.)
#
# Uses --network host so ROS 2 DDS discovery works across these containers AND
# the container can reach the bot on your LAN. Linux only (host networking).
set -euo pipefail

IMAGE="zros-ros2-demo"
BOT_IP="${BOT_IP:-192.168.1.46}"
HERE="$(cd "$(dirname "$0")" && pwd)"      # .../ZROS/PythonAPI/ros2_demo
API_DIR="$(dirname "$HERE")"               # .../ZROS/PythonAPI  (has zrsense.py)

# Mount the whole PythonAPI dir at /app so `import zrsense` resolves, and run
# scripts out of /app/ros2_demo. PYTHONPATH=/app makes the import work.
COMMON=(--rm -it --network host --ipc=host
        -e PYTHONPATH=/app
        -v "/mnt/e/rosdemo/zrcar_ros2_demo/app":/app:rw
        -w /app
        "$IMAGE")

case "${1:-bridge}" in
  build)
    sudo docker build -t "$IMAGE" -f "$HERE/Dockerfile" "$HERE"
    ;;
  bridge)
    sudo docker run "${COMMON[@]}" \
      python3 zros_driver.py --ros-args -p robot_ip:="$BOT_IP"
    ;;
  teleop)
    sudo docker run "${COMMON[@]}" \
      ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ;;
  obstacle)
    sudo docker run "${COMMON[@]}" \
      python3 obstacle_stop.py --ros-args -p cruise_speed:="${2:-0.0}"
    ;;
  shell)
    sudo docker run "${COMMON[@]}" bash
    ;;
  *)
    echo "usage: $0 {build|bridge|teleop|obstacle [speed]|shell}" >&2
    exit 2
    ;;
esac
