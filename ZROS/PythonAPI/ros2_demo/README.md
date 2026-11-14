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
| car → ROS | `/scan` | `sensor_msgs/LaserScan` | last room scan, flat 2D (see §5) |
| car → ROS | `/scan_cloud` | `sensor_msgs/PointCloud2` | last room scan, every zone in 3D |
| car → ROS | `/map_cloud` | `sensor_msgs/PointCloud2` | all scans so far, 3D (latched) |
| car → ROS | `/map` | `nav_msgs/OccupancyGrid` | floor map from all scans (latched) |
| car → ROS | `/robot_marker` | `visualization_msgs/MarkerArray` | a simple car outline on `base_link` |
| ROS → car | `/scan_room` *(service)* | `std_srvs/Trigger` | run one room scan |
| ROS → car | `/scan_loop` *(service)* | `std_srvs/SetBool` | keep scanning (`true`) / stop (`false`) |
| ROS → car | `/clear_map` *(service)* | `std_srvs/Trigger` | empty the map + zero the heading |

TF: **`odom → base_link`** comes from the IMU — the car model in rviz **turns with the
heading and tilts with pitch/roll** (rotation only: there are no wheel encoders, so
the bridge can't know *where* the car drove). Plus static `base_link → camera_link /
imu_link / tof_link / laser`.
It's a **car**, not a tank: `angular.z` steers the front wheels while `linear.x`
drives — it won't spin in place.

## Prerequisites

- **Docker**, on a Linux host **or Windows via WSL2** (see the WSL2 section below).
- Your ZRCar on the **same LAN**; note its IP from the bot's `/console` (examples use `192.168.1.46`).
- On the bot's main page (`http://<bot-ip>/` → **Info → 🔧 Components**), enable **Camera**, **IMU**, and **Rangefinder**
  (IMU + rangefinder are **OFF by default** — if they're off, `/imu`/`/range` stay silent). Restart the bot after enabling.
  The IMU is only detected at boot, so restart after re-wiring it too.
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
Open **Foxglove** (desktop app or https://app.foxglove.dev) → *Open connection* → `ws://localhost:8765`.
Then **Layouts → Import from file → [`zrcar.foxglove.json`](zrcar.foxglove.json)** (this folder) for a ready-made ZRCar layout:

| Panel | Shows |
|---|---|
| **3D** | TF frames + `/range` + the **`/scan` room map** — run `./run.sh scan` to populate it. This is the mapping view, and it needs **no X display**. |
| **Camera** | `/camera/image_raw` |
| **Range & servo** | `/range.range` (ToF distance) + `/servo_angle.data` |
| **Motion** | `/cmd_vel` throttle/steer + `/imu/data` yaw rate |
| **Logs** | `/rosout` — the `car.raw failed: not connected` lines are **harmless** keepalive during (re)connect |

> If a panel imports as "unknown" on your Foxglove version, delete it and add that panel manually (then set the topic) — the rest of the layout still loads. (The in-app AI assistant can also build panels, but the file above gives everyone the same starting point.)

**(c) rviz — the classic (needs a display):**
```bash
./run.sh rviz
```
Loads `zrcar.rviz`: the **Map** (`/map` floor map), **RoomCloud** (`/map_cloud`, coloured by height), the **Car** outline (turns/tilts with the IMU), the Range cone, TF and the camera image. Fixed frame is `odom`. (The flat `/scan` **RoomScan** display is there too, unticked.) On **Windows/WSLg** (Win 11) the window pops up automatically via the display forwarding in `run.sh`. On a headless box you'll get `could not connect to display` — use **(a)** or **(b)** instead. (`./run.sh image` is the lighter `rqt_image_view`, same display requirement.)

> **Getting a GUI (rviz/gazebo) up from WSL:** `run.sh` already forwards the display on **WSLg** (Windows 11 — run `wsl --update` once if `echo $DISPLAY` is empty). On **Windows 10** there's no WSLg: run an X server (e.g. VcXsrv) and `export DISPLAY=<windows-ip>:0` before `./run.sh rviz`. **Gazebo** has the same display requirement but is a much bigger lift (needs a robot URDF + a world) — for room mapping today, the `/scan` + Foxglove **3D** panel above gets you there without any of that.

## 5. Scan the room — a map in rviz (`/map`, `/map_cloud`, `/scan`)

The VL53L5CX rangefinder is geared **3:1 to the steering servo** (36T→12T), so sweeping the
steering servo ±60° swings it a full ±180°. It isn't a single beam: it's a tiny **4×4 / 8×8
depth camera** (~45° field of view). At every servo step the bridge reads the **whole zone
grid** and turns every zone into its own ray (beam direction + that zone's offset in the
field of view), so a flat wall comes out as a straight line instead of an arc.
```bash
# bridge running in Terminal 1, rviz open in Terminal 2
./run.sh scan          # one sweep (~35 s): /scan + /scan_cloud, added to /map + /map_cloud
./run.sh scan loop     # keep sweeping (car stays still);  ./run.sh scan stop  to end
./run.sh clear         # empty the map and zero the heading — a fresh start
```
The car stays put and ignores teleop while it scans — **the steering servo *is* the scanner**,
so it can't steer during a sweep. Scans **add up**: turn the car in place (by hand or a
short drive) and scan again — the IMU heading keeps the new sweep lined up with the map.
Driving somewhere new will **not** line up: with no wheel encoders the bridge can't tell how
far the car moved (that needs odometry — see *Next steps*). Run `./run.sh clear` first.

**Use 8×8:** on the `/car` page → ToF settings → resolution **8×8** (64 zones, 5.6° each, the
bot restarts once). 4×4 works too, just coarser (11°).

**Calibrate once** (depends on how the gears were meshed on *your* car). Put a flat wall
~40 cm straight in front, `./run.sh clear`, `./run.sh scan`, and look where the wall lands:

| The wall in front shows up… | Restart the bridge with |
|---|---|
| in front ✅ | nothing to do |
| on the **left** | `-p scan_offset_deg:=-90` |
| on the **right** | `-p scan_offset_deg:=90` |
| behind | `-p scan_offset_deg:=180` |
| in front, but the room is **mirrored** (left box drawn on the right) | `-p scan_gear:=-3.0` |
| walls look doubled / smeared even after that | `-p scan_zone_flip:=true` |

e.g. `BOT_IP=192.168.0.72 ./run.sh bridge -p scan_offset_deg:=-90`. In between, fine-tune
with small values (`-p scan_offset_deg:=-84`).

- **Tune:** `scan_servo_min`/`scan_servo_max` (sweep width), `scan_center` (90 = straight),
  `scan_step` (bigger = faster/coarser), `scan_settle` (raise if walls smear),
  `scan_min_range` (0.12 m — drops the car's own chassis when the beam swings backwards),
  `map_resolution` / `map_size` (5 cm cells, 8 m square).
- **Clear the view:** `./run.sh clear` (or rviz's **Reset** button bottom-left for the displays).
- **No ROS?** `../examples/room_scan.py` does a simple single-beam sweep and plots it with
  matplotlib (and the `/car` page has a 🗺️ Room Scan button).

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
`framesize` (QQVGA/QVGA/VGA…), `publish_image`, `cmd_timeout`,
`camera_vflip` / `camera_hmirror` (1 = flip on the sensor, -1 = leave as is — camera
upside down? set both to 1), and the scan params in §5 (`scan_offset_deg`, `scan_gear`,
`scan_zone_flip`, `scan_settle`, `scan_min_range`, `scan_max_range`, `scan_bin_deg`,
`scan_loop_pause`, `map_resolution`, `map_size`, `clear_resets_heading`).

## Troubleshooting

| Symptom | Fix |
|---|---|
| **Topics listed but `echo` shows nothing / teleop won't drive** | containers aren't sharing memory — `run.sh` must use **`--ipc=host`** (it does; rebuild/re-copy if you edited it). Without it, nodes discover but exchange no data. |
| `connection reset` loop in the bridge | two clients fighting for the bot's one WS slot → a stale bridge container (`docker kill $(docker ps -q)`, run ONE) or an open browser `/car` tab (close it). |
| No `/camera/image_raw` frames | camera off on the bot, or a browser `/car` tab is holding the stream. |
| Room scan: walls on the wrong side / mirrored | calibrate once — §5 (`scan_offset_deg`, `scan_gear`). |
| Room scan: walls curved, objects smeared | update to this bridge (it uses every zone); set the rangefinder to **8×8**; raise `scan_settle`. |
| Camera upside down in rviz | `./run.sh bridge -p camera_vflip:=1 -p camera_hmirror:=1` |
| Car model doesn't turn/tilt | IMU off or not detected (enable + restart the bot; it's detected at boot only). |
| No `/imu` or `/range` | IMU/rangefinder are **OFF by default** — enable on the bot's main page → Info → Components + restart. |
| `could not connect to display` (rviz/rqt) | no X display in this environment → use `./run.sh video` or `./run.sh foxglove`. On WSLg it should forward automatically. |
| `rsv is not implemented` | `websocket-client` too new — the image pins **1.6.4**; rebuild. |
| Motor only buzzes at low speed | raise teleop speed, or `-p min_throttle:=60`. |

## Next steps
- **A real car model:** export the Onshape design to URDF (e.g. `onshape-to-robot`) and run
  `robot_state_publisher` — it replaces the simple `/robot_marker` outline.
- **Map while driving:** needs odometry (wheel encoders, or a forward-facing optical-flow
  sensor) → publish `odom → base_link` *position* → then `slam_toolbox` can build a proper map.
- Add a `/battery` + `/temperature` topic from the telemetry (keys `b`/`t`).
- Map `/cmd_vel` through `nav2` for waypoint driving.
