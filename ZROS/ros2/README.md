# ZRCar × ROS 2 demo

Drive a ZoneRobotics **ZRCar** from ROS 2 and see its camera, IMU, and
time-of-flight sensor as standard ROS topics — then let a tiny ROS node drive
it on its own. It's a hands-on way to learn the core ROS 2 ideas (nodes,
topics, messages, teleop, visualization, bags) on real hardware, with no
firmware changes: everything rides the same LAN WebSocket the `/car` page uses,
via the [`zrsense`](../zrsense.py) SDK.

```
          ROS 2 topics                         zrsense SDK (LAN WebSocket)
 teleop ──/cmd_vel──►┐                       ┌─────────────────────────────┐
 obstacle_stop ──────┤                       │                             │
                     ▼                        │   throttle + steering  ───► │ ZRCar
 rviz/rqt ◄─/camera/image_raw─┐  zros_driver  │                             │ (ESP32)
          ◄─/imu/data─────────┤◄──────────────┤   ◄─── JPEG video           │
          ◄─/range────────────┘               │   ◄─── IMU + ToF telemetry  │
                                               └─────────────────────────────┘
```

## Topics

| Direction | Topic | Type | Maps to |
|---|---|---|---|
| ROS → car | `/cmd_vel` | `geometry_msgs/Twist` | `linear.x` → throttle, `angular.z` → steering |
| ROS → car | `/servo_angle` | `std_msgs/Int32` | an aux hobby servo (`aux_servo_pin`, default 4) |
| car → ROS | `/camera/image_raw` | `sensor_msgs/Image` (bgr8) | live JPEG, decoded |
| car → ROS | `/imu/data` | `sensor_msgs/Imu` | MPU6050 accel + heading/pitch/roll |
| car → ROS | `/range` | `sensor_msgs/Range` | VL53L5CX nearest distance |

It's a **car**, not a tank: `angular.z` steers the front wheels while
`linear.x` drives — it won't spin in place.

## Prerequisites

- A Linux host with **Docker** (host networking is Linux-only; that's what ROS 2 + DDS wants anyway).
- Your ZRCar on the **same LAN**, and its IP (shown on the bot's `/console`). Examples use `192.168.1.46`.
- The bot's **camera**, **rangefinder**, and **IMU** enabled (bot `/console` → 🔧 Components) for video/`/range`/`/imu`.
- **Close the browser `/car` tab** — the bot accepts one WebSocket client at a time.

Everything below runs from this folder (`ZROS/PythonAPI/ros2_demo`).

## 0. Build the image (once)

```bash
./run.sh build
```

## 1. Start the bridge — terminal 1

```bash
BOT_IP=192.168.1.46 ./run.sh bridge
```

You should see `ZRos bridge up: ...`. Leave it running.

## 2. Look around — terminal 2

A shell inside the ROS environment:

```bash
./run.sh shell
```

Then the first commands every ROS person learns:

```bash
ros2 topic list                 # see the 5 topics above
ros2 topic echo /range          # watch distance change as you wave your hand
ros2 topic echo /imu/data       # tilt the bot, watch accel/orientation
ros2 topic hz /camera/image_raw # frame rate of the video
```

> **What you just learned:** a ROS *node* (the bridge) publishes *topics*;
> any number of other processes can *subscribe*. Nothing is wired point-to-point.

## 3. Drive it — terminal 2 (or a third)

```bash
./run.sh teleop
```

Use the `i/j/k/l/,` keys (teleop prints the map). It publishes `geometry_msgs/Twist`
on `/cmd_vel`; the bridge turns that into throttle + steering.

> **Tip:** put the car on blocks (wheels up) the first time. The bridge has a
> keepalive + 0.6 s timeout, so if teleop stops or crashes, the car coasts to a
> stop instead of running away.

## 4. See the camera

```bash
# in a ./run.sh shell
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

(Need `apt-get install ros-humble-rqt-image-view` in the shell, or add it to the
Dockerfile.) Or load `/camera/image_raw` as an Image display in `rviz2`.

## 5. Let it drive itself — autonomy demo

`obstacle_stop.py` is a pure ROS node: it subscribes to `/range` and publishes
`/cmd_vel`. Stop teleop first (only one thing should command `/cmd_vel`).

```bash
# watch-only (safe): prints "STOP"/"go", publishes stop when a wall is near
./run.sh obstacle

# full auto-cruise: creeps forward, brakes before the wall (wheels up / open floor!)
./run.sh obstacle 0.3
```

> **What you just learned:** a "behaviour" in ROS is just a node between a
> sensor topic and a command topic. Swap `obstacle_stop` for a real planner and
> nothing else changes.

## 6. Record & inspect

```bash
# in a shell: record everything, drive around, Ctrl-C, then replay
ros2 bag record -a -o zrcar_run
ros2 bag play zrcar_run

# see the live node/topic graph
ros2 run rqt_graph rqt_graph
```

## How the bridge maps things (read `zros_driver.py`)

- `/cmd_vel`: `linear.x × linear_scale` → `bot.car.raw(throttle)` (−100..100);
  `angular.z` → `bot.car.set_steering(90 − angular.z × steer_scale)`.
  A 10 Hz keepalive re-sends the throttle and zeroes it if `/cmd_vel` goes quiet.
- Video: the SDK's `connect()` makes the firmware start streaming JPEG; each
  frame is decoded (OpenCV) to a bgr8 `Image`.
- Telemetry (~1 Hz): short keys `iax/iay/iaz` → `Imu` accel, `ih/ipt/irl` →
  orientation; `d` (mm) → `Range` (m).

Tunable params (`--ros-args -p name:=value`): `robot_ip`, `linear_scale`,
`steer_scale`, `aux_servo_pin`, `framesize`, `publish_image`, `cmd_timeout`.

## Troubleshooting

| Symptom | Fix |
|---|---|
| `connect failed` / no topics move | wrong IP, or the browser `/car` tab is holding the single WS slot — close it |
| Protocol error `rsv is not implemented` | `websocket-client` is too new — the image pins **1.6.4**; rebuild |
| No `/camera/image_raw` frames | camera disabled in `/console` → Components, or no camera on this bot |
| No `/range` or `/imu` | rangefinder / IMU disabled in Components (they're opt-in) |
| Car won't move | another client connected; or motor pins/power not set for your wiring (`bot.car.get_status()`), or it needs microSD disabled on bots where the motor shares SD pins |
| `imu not present` on first command | the bridge already waits 0.5 s after connect; if you see it, the IMU is simply disabled |

## Next steps (ideas)

- Publish a `tf` for `camera_link`/`imu_link` and view the car frame in rviz.
- Turn the 8×8 ToF grid (telemetry key `dg`) into a `LaserScan` for real nav.
- Map `/cmd_vel` through `nav2` for waypoint driving.

## Extra 

This is the raw docker command I used in wsl for troubleshooting if needed, I ran into path issues, you might too.
```
sudo docker run --rm -it --network host --ipc=host -e PYTHONPATH=/app -v "/mnt/e/rosdemo/zrcar_ros2_demo/app":/app:rw -w /app "zros-ros2-demo"               
```
