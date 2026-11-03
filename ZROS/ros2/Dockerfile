# ZRCar ROS 2 demo image.
# ROS 2 Humble base + OpenCV (JPEG decode) + the zrsense SDK's websocket dep,
# plus teleop_twist_keyboard so you can drive from the same image.
FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-opencv \
        ros-humble-teleop-twist-keyboard \
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
