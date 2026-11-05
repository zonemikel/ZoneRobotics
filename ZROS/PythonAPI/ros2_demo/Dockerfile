# ZRCar ROS 2 demo image.
# ROS 2 Humble base + OpenCV (JPEG decode) + the zrsense SDK's websocket dep,
# + teleop (drive), + the visualization stack: rviz2, rqt_image_view,
# web_video_server (browser video) and foxglove_bridge (browser 3D/plots).
# It's a chunky image (~2-3 GB) because rviz2 pulls a lot — the build takes a
# few minutes, but then everything runs from this one image.
FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-opencv \
        ros-humble-teleop-twist-keyboard \
        ros-humble-tf2-ros-py \
        ros-humble-rviz2 \
        ros-humble-rqt-image-view \
        ros-humble-web-video-server \
        ros-humble-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

# websocket-client PINNED to 1.6.4 — the ZR firmware's LAN WebSocket sets RSV
# bits that websocket-client >=1.9 strict-rejects ("rsv is not implemented").
RUN pip3 install --no-cache-dir "websocket-client==1.6.4" "numpy"

WORKDIR /app

# The scripts + zrsense.py are MOUNTED at runtime by run.sh (so you can edit
# them without rebuilding). To bake a self-contained image instead, copy the
# PythonAPI dir into the build context and uncomment:
#   COPY zrsense.py /app/zrsense.py
#   COPY ros2_demo/zros_driver.py ros2_demo/obstacle_stop.py /app/

# Default: run the bridge. Override the IP with -p robot_ip:=<your-bot-ip>.
CMD ["python3", "zros_driver.py", "--ros-args", "-p", "robot_ip:=192.168.1.46"]
