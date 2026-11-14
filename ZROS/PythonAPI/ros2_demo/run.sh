#!/usr/bin/env bash
# Helper to build + run the ZRCar ROS 2 demo in Docker (Linux host OR Windows WSL2).
#
#   ./run.sh build                       # build the image once
#   BOT_IP=192.168.1.46 ./run.sh bridge  # run the ROS<->bot bridge (leave running)
#   ./run.sh teleop                      # keyboard teleop -> /cmd_vel
#   ./run.sh obstacle [speed]            # autonomous brake/cruise demo
#   ./run.sh scan                        # one room scan -> /scan, /map_cloud, /map; needs bridge running
#   ./run.sh scan loop | scan stop       # keep scanning / stop after the current sweep
#   ./run.sh clear                       # empty the map (+ zero the heading) for a fresh start
#   ./run.sh shell                       # a shell in the image (ros2 topic echo, ...)
#   ./run.sh video                       # web_video_server  -> browser http://localhost:8080
#   ./run.sh foxglove                    # foxglove_bridge   -> Foxglove Studio ws://localhost:8765
#   ./run.sh rviz                        # rviz2 w/ zrcar.rviz   (needs a display — see below)
#   ./run.sh image                       # rqt_image_view        (needs a display)
#
# --network host : ROS 2 DDS discovery + reaching the bot on your LAN.
# --ipc=host     : REQUIRED so separate containers can exchange ROS data (shared
#                  memory). Without it they discover topics but get NO messages.
# Display (rviz/image): forwarded automatically on WSLg (Windows 11) or native X.
#                  No display? use `video` (browser) or `foxglove` instead.
set -euo pipefail

IMAGE="zros-ros2-demo"
BOT_IP="${BOT_IP:-192.168.1.46}"
HERE="$(cd "$(dirname "$0")" && pwd)"      # folder holding run.sh (…/ros2_demo)

# Locate zrsense.py and mount the dir that holds it at /app so `import zrsense`
# resolves. Repo layout has it in the PARENT (ZROS/PythonAPI); a standalone copy
# (e.g. an extracted zip) may sit right next to run.sh. Works from either.
PARENT="$(dirname "$HERE")"
if   [ -f "$PARENT/zrsense.py" ]; then MOUNT="$PARENT"; WORK="/app/$(basename "$HERE")"
elif [ -f "$HERE/zrsense.py" ];   then MOUNT="$HERE";   WORK="/app"
else
  echo "ERROR: zrsense.py not found next to run.sh or in its parent dir." >&2
  echo "Run from a repo clone at ZROS/PythonAPI/ros2_demo (recommended)," >&2
  echo "or copy zrsense.py into this folder next to run.sh." >&2
  exit 1
fi

# Docker must be runnable WITHOUT sudo (sudo drops BOT_IP and the host display env).
if ! docker info >/dev/null 2>&1; then
  echo "ERROR: can't talk to Docker as '$USER'." >&2
  echo "  start it:  sudo service docker start" >&2
  echo "  no sudo:   sudo usermod -aG docker \$USER   then CLOSE & reopen this shell" >&2
  echo "  (do NOT run this script with sudo — it drops BOT_IP and the display.)" >&2
  exit 1
fi

# --network host: DDS + reach the bot. --ipc=host: REQUIRED so separate
# containers share ROS data (shared memory). IMAGE is added per-command.
COMMON=(--rm -it --network host --ipc=host
        -e PYTHONPATH=/app
        -v "$MOUNT":/app:ro
        -w "$WORK")

# GUI passthrough for rviz/rqt. WSLg (Windows 11) exposes the display under
# /mnt/wslg; a native Linux host uses /tmp/.X11-unix. Harmless when absent —
# the GUI just can't open a window (use `video`/`foxglove` then).
GUI=()
if [ -d /mnt/wslg ]; then
  GUI=(-e DISPLAY -e WAYLAND_DISPLAY -e XDG_RUNTIME_DIR -e PULSE_SERVER
       -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg)
elif [ -d /tmp/.X11-unix ]; then
  GUI=(-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix)
fi
# GPU for rviz's OpenGL when the host exposes it (else Mesa software rendering:
# works, just slower). Tip from a tester running rviz/gazebo from WSL + Docker.
if [ ${#GUI[@]} -gt 0 ] && [ -e /dev/dri ]; then
  GUI+=(--device /dev/dri)
fi

case "${1:-bridge}" in
  build)
    docker build -t "$IMAGE" -f "$HERE/Dockerfile" "$HERE"
    ;;
  bridge)
    # extra args after `bridge` pass through to the node, e.g.:
    #   ./run.sh bridge -p min_throttle:=60 -p steer_trim:=10
    docker run "${COMMON[@]}" "$IMAGE" \
      python3 zros_driver.py --ros-args -p robot_ip:="$BOT_IP" "${@:2}"
    ;;
  teleop)
    docker run "${COMMON[@]}" "$IMAGE" \
      ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ;;
  obstacle)
    docker run "${COMMON[@]}" "$IMAGE" \
      python3 obstacle_stop.py --ros-args -p cruise_speed:="${2:-0.0}"
    ;;
  scan)
    # Room scan on the already-running bridge (./run.sh bridge). Sweeps the
    # steering servo (rangefinder geared 3:1) and publishes /scan, /map_cloud and
    # /map; open rviz (./run.sh rviz) to watch the map build.
    case "${2:-once}" in
      loop)
        echo ">> continuous scanning ON — ./run.sh scan stop to end. Bridge must be running."
        docker run "${COMMON[@]}" "$IMAGE" \
          ros2 service call /scan_loop std_srvs/srv/SetBool "{data: true}"
        ;;
      stop)
        docker run "${COMMON[@]}" "$IMAGE" \
          ros2 service call /scan_loop std_srvs/srv/SetBool "{data: false}"
        ;;
      *)
        echo ">> triggering a room scan — watch it in rviz (./run.sh rviz). Bridge must be running."
        docker run "${COMMON[@]}" "$IMAGE" \
          ros2 service call /scan_room std_srvs/srv/Trigger "{}"
        ;;
    esac
    ;;
  clear)
    # Forget every sweep (map + clouds) and zero the heading: a fresh map that
    # starts with the car facing "forward".
    docker run "${COMMON[@]}" "$IMAGE" \
      ros2 service call /clear_map std_srvs/srv/Trigger "{}"
    ;;
  shell)
    docker run "${COMMON[@]}" "$IMAGE" bash
    ;;
  video)
    echo ">> open a browser to  http://localhost:8080/stream_viewer?topic=/camera/image_raw"
    docker run "${COMMON[@]}" "$IMAGE" \
      ros2 run web_video_server web_video_server
    ;;
  foxglove)
    echo ">> open Foxglove Studio -> Open connection -> ws://localhost:8765"
    docker run "${COMMON[@]}" "$IMAGE" \
      ros2 run foxglove_bridge foxglove_bridge
    ;;
  rviz)
    docker run "${COMMON[@]}" "${GUI[@]}" "$IMAGE" \
      rviz2 -d zrcar.rviz
    ;;
  image)
    docker run "${COMMON[@]}" "${GUI[@]}" "$IMAGE" \
      ros2 run rqt_image_view rqt_image_view /camera/image_raw
    ;;
  *)
    echo "usage: $0 {build|bridge|teleop|obstacle [speed]|scan [loop|stop]|clear|shell|video|foxglove|rviz|image}" >&2
    exit 2
    ;;
esac
