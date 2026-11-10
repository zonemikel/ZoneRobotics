# ZRCar × ROS 2 demo

Drive a ZoneRobotics **ZRCar** from ROS 2 and see its camera, IMU, and
time-of-flight sensor as standard ROS topics — in **rviz**, **Foxglove**, or a
plain browser — then let a tiny ROS node drive it on its own. A hands-on way to
learn core ROS 2 (nodes, topics, messages, teleop, TF, visualization, bags) on
real hardware, with no firmware changes: everything rides the same LAN
WebSocket the `/car` page uses, via the [`zrsense`](../zrsense.py) SDK.

```
          ROS 2 topics                         zrsense SDK (LAN WebSocket)
 teleop ──/cmd_vel──►┐                       ┌─────────────────────────────┐
 obstacle_stop ──────┤                       │                             │
                     ▼                        │   throttle + steering  ───► │ ZRCar
 rviz/foxglove ◄─/camera/image_raw─┐ zros_   │                             │ (ESP32)
              ◄─/imu/data──────────┤ driver ─┤   ◄─── JPEG video           │
              ◄─/range─────────────┘         │   ◄─── IMU + ToF telemetry  │
                                              └─────────────────────────────┘
```

## Quickstart — from the repo (do this)

The demo lives in the repo at `ZROS/PythonAPI/ros2_demo`. Clone it, go there, build once, run:

```bash
git clone https://github.com/zonemikel/ZoneRobotics.git
cd ZoneRobotics/ZROS/PythonAPI/ros2_demo
chmod +x run.sh
./run.sh build                        # once (~a few min). SKIP if already built.
BOT_IP=<your-bot-ip> ./run.sh bridge  # terminal 1 — leave it running
```
Then, in **separate terminals** (each one: `cd …/ros2_demo` first):
```bash
./run.sh teleop      # drive with i/j/k/l
./run.sh foxglove    # browser viz → Foxglove Studio → ws://localhost:8765
./run.sh video       # browser camera → http://localhost:8080
./run.sh shell       # then e.g. ros2 topic echo /range
```

**Don't use `sudo`.** Add yourself to the docker group once so `docker` runs as you:
```bash
sudo usermod -aG docker $USER    # then CLOSE and reopen the terminal
sudo service docker start        # once per session (WSL doesn't auto-start Docker)
```
`sudo ./run.sh` drops the `BOT_IP` you set and the display, and the read-only mount fails — that's the `mkdir /app/…: read-only file system` error. Run it as your normal user, from inside the repo (not from a loose zip — `run.sh` finds `zrsense.py` in the repo layout).

> Already built the image? **Don't run `build` again** — it rebuilds. Just use `bridge` / `teleop` / `foxglove` / etc.

## Topics

| Direction | Topic | Type | Maps to |
|---|---|---|---|
| ROS → car | `/cmd_vel` | `geometry_msgs/Twist` | `linear.x` → throttle, `angular.z` → steering |
| ROS → car | `/servo_angle` | `std_msgs/Int32` | an aux hobby servo (`aux_servo_pin`, default 4) |
| car → ROS | `/camera/image_raw` | `sensor_msgs/Image` (bgr8) | live JPEG, decoded |
| car → ROS | `/imu/data` | `sensor_msgs/Imu` | MPU6050 accel + heading/pitch/roll |
| car → ROS | `/range` | `sensor_msgs/Range` | VL53L5CX straight-ahead distance |
| car → ROS | `/scan` | `sensor_msgs/LaserScan` | swept-rangefinder 2D room scan (see §5) |
| ROS → car | `/scan_room` *(service)* | `std_srvs/Trigger` | run one room scan |

The bridge also publishes static TF: `base_link → camera_link / imu_link / tof_link / laser`.
It's a **car**, not a tank: `angular.z` steers the front wheels while `linear.x`
drives — it won't spin in place.

## Prerequisites

- **Docker**, on a Linux host **or Windows via WSL2** (see the WSL2 section below).
- Your ZRCar on the **same LAN**; note its IP from the bot's `/console` (examples use `192.168.1.46`).
- On the bot's `/console → 🔧 Components`, enable **Camera**, **IMU**, and **Rangefinder**
  (IMU + rangefinder are **OFF by default** — if they're off, `/imu`/`/range` stay silent). Restart the bot after enabling.
- **Close the browser `/car` tab** — the bot serves one WebSocket client at a time, and an open `/car` page steals the stream from the bridge.

Everything below runs from this folder (`ZROS/PythonAPI/ros2_demo`).

## Running on Windows (WSL2) — one-time setup

```powershell
wsl --install -d Ubuntu          # (reboot if it asks; set a username/password)
```
Then **inside Ubuntu** (`wsl`), install Docker and start it:
```bash
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER      # then close & reopen the Ubuntu terminal
sudo service docker start          # run once each session (no auto-start on WSL)
```
Copy this project in (or `git clone` the repo), then `cd` to it:
```bash
cp -r /mnt/c/Users/<you>/ZoneRobotics/ZROS/PythonAPI ~/zr-api
cd ~/zr-api/ros2_demo
```
> **Prompts:** `user@host:~$` = Ubuntu (run `./run.sh ...` here); `root@host:/app#` = inside a container (run `ros2 ...` here, no `run.sh`).

## 0. Build the image (once — a few minutes)

```bash
./run.sh build
```

## 1. Start the bridge — Terminal 1 (leave it running)

```bash
BOT_IP=192.168.1.46 ./run.sh bridge
```
Wait for `ZRos bridge up: ...`, then **don't touch this terminal.** Run **one** bridge only.

## 2. Look around — Terminal 2

```bash
./run.sh shell
```
then, at the `root@...#` prompt:
```bash
ros2 topic list                 # the 5 topics above
ros2 topic echo /range          # distance — wave your hand at the ToF
ros2 topic echo /imu/data       # tilt the bot — accel/orientation
ros2 topic hz /camera/image_raw # video frame rate (~10 Hz)
```
> A ROS *node* (the bridge) publishes *topics*; any number of other processes *subscribe*.

## 3. Drive it — Terminal 3

```bash
./run.sh teleop
```
`i/j/k/l/,` keys (teleop prints the map) → `geometry_msgs/Twist` on `/cmd_vel`.
- **Only buzzes, won't move at low speed?** The motor needs a minimum PWM. Press `q`/`w` a few times to raise the speed, or run the bridge with a floor: `./run.sh bridge` after editing — or pass `-p min_throttle:=60` (any nonzero command is bumped to ≥60%).
- **Wheels off-centre going straight?** pass `-p steer_trim:=10` (degrees) to re-centre.
- **Tip:** put the car on blocks the first time — the bridge coasts to a stop if teleop dies (keepalive + 0.6 s timeout).

## 4. See it — three ways

**(a) Browser video — works everywhere, no display needed (easiest on WSL):**
```bash
./run.sh video
```
then open `http://localhost:8080/stream_viewer?topic=/camera/image_raw` in a browser (plain **MJPEG** — don't add `&type=h264`, it won't render).

**(b) Foxglove — rviz-like 3D + image + plots in a browser (great on WSL, no X display):**
```bash
./run.sh foxglove
```
Open **Foxglove Studio** (desktop app or https://app.foxglove.dev) → *Open connection* → `ws://localhost:8765`. Add Image / 3D / Raw Message panels.

**(c) rviz — the classic (needs a display):**
```bash
./run.sh rviz
```
Loads `zrcar.rviz` (camera Image + Range cone + **RoomScan** `/scan` + TF frames). On **Windows/WSLg** (Win 11) the window pops up automatically via the display forwarding in `run.sh`. On a headless box you'll get `could not connect to display` — use **(a)** or **(b)** instead. (`./run.sh image` is the lighter `rqt_image_view`, same display requirement.)

## 5. Scan the room — a live 2D map (`/scan`)

The VL53 rangefinder is geared **3:1 to the steering servo** (36T→12T), so sweeping the
steering servo ±60° swings the laser a full ±180° — a cheap single-beam lidar. Call the
`scan_room` service and the bridge sweeps the servo, reads the straight-ahead distance at
each step, and publishes one `sensor_msgs/LaserScan` on `/scan`:
```bash
# bridge running in Terminal 1, rviz open in Terminal 2 (RoomScan display = /scan)
./run.sh scan          # == ros2 service call /scan_room std_srvs/srv/Trigger
```
Watch the green `/scan` points trace the room outline in rviz. The sweep takes ~25–30 s
(121 beams × a short settle); the car stays put and ignores teleop steering while it scans.

- **Mirrored?** Flip the gear sign: `./run.sh bridge -p scan_gear:=-3.0`.
- **Tune:** `scan_servo_min`/`scan_servo_max` (sweep width), `scan_center` (90 = forward),
  `scan_step` (bigger = faster/coarser), `scan_settle` (raise if readings smear).
- **No ROS?** `../examples/room_scan.py` does the same sweep and polar-plots it with matplotlib.

## 6. Let it drive itself — autonomy demo

`obstacle_stop.py` is a pure ROS node (subscribes `/range`, publishes `/cmd_vel`). Stop teleop first.
```bash
./run.sh obstacle        # watch-only: prints STOP/go, publishes stop near a wall
./run.sh obstacle 0.3    # auto-cruise: creeps forward, brakes at the wall (blocks/open floor!)
```
> A "behaviour" in ROS is just a node between a sensor topic and a command topic.

## 7. Record & inspect

```bash
ros2 bag record -a -o zrcar_run   # drive around, Ctrl-C, then: ros2 bag play zrcar_run
ros2 run rqt_graph rqt_graph      # live node/topic graph (needs a display)
```

## Params (`--ros-args -p name:=value` on the bridge)

`robot_ip`, `linear_scale` (x=1.0→%throttle), `steer_scale` (z=1.0→steer°),
`steer_trim` (centre offset°), `min_throttle` (stiction floor %), `aux_servo_pin`,
`framesize` (QQVGA/QVGA/VGA…), `publish_image`, `cmd_timeout`.

## Troubleshooting

| Symptom | Fix |
|---|---|
| **Topics listed but `echo` shows nothing / teleop won't drive** | containers aren't sharing memory — `run.sh` must use **`--ipc=host`** (it does; rebuild/re-copy if you edited it). Without it, nodes discover but exchange no data. |
| `connection reset` loop in the bridge | two clients fighting for the bot's one WS slot → a stale bridge container (`docker kill $(docker ps -q)`, run ONE) or an open browser `/car` tab (close it). |
| No `/camera/image_raw` frames | camera off on the bot, or a browser `/car` tab is holding the stream. |
| No `/imu` or `/range` | IMU/rangefinder are **OFF by default** — enable in `/console → Components` + restart. |
| `could not connect to display` (rviz/rqt) | no X display in this environment → use `./run.sh video` or `./run.sh foxglove`. On WSLg it should forward automatically. |
| `rsv is not implemented` | `websocket-client` too new — the image pins **1.6.4**; rebuild. |
| Motor only buzzes at low speed | raise teleop speed, or `-p min_throttle:=60`. |

## Next steps
- Turn the 8×8 ToF grid (telemetry key `dg`) into a `LaserScan` for real nav.
- Add a `/battery` + `/temperature` topic from the telemetry (keys `b`/`t`).
- Map `/cmd_vel` through `nav2` for waypoint driving.
