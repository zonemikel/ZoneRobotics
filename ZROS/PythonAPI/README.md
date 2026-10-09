# ZRSense Python SDK

A tiny synchronous Python library for talking to a ZoneRobotics ESP32 bot
over LAN. Wraps the same WebSocket + JSON command protocol the firmware's
`/console` page uses, so **anything you can click in the browser you can
script here** — and any new device command added to the firmware works
through `.send()` immediately without an SDK update.

Designed for two audiences:

- **Users writing automated behaviours** (drive a square, patrol, follow
  a wall, react to ToF readings, …).
- **The dev team writing regression tests** — the SDK is one import away
  and every command surface is exposed.

## Install

```bash
python -m pip install "websocket-client==1.6.4"
```

**Pin 1.6.4.** websocket-client 1.9+ strict-validates frames and rejects the
bot's WebSocket frames with `rsv is not implemented, yet` or `Invalid opcode`,
which kills the receive loop. If you hit those errors you're on a newer
version — reinstall with the pin above.

The SDK itself is a single file (`zrsense.py`) — copy it into your project
or drop the folder on `PYTHONPATH`.

## 30-second tour

```python
from zrsense import Robot

with Robot("192.168.1.46") as bot:
    bot.rgb.color(0, 255, 0)                  # green
    bot.wait(1)
    bot.rgb.off()

    bot.drive.forward(power=80, ms=400)       # H-bridge / tank drive
    bot.drive.stop()

    bot.servo.set_angle(pin=4, angle=45)      # RMT servo on GPIO 4

    temp_c = bot.wait_for_telemetry("temp")   # blocks until next telem frame
    print(f"CPU temp: {temp_c} °C")
```

## Connection lifecycle

```python
bot = Robot("192.168.1.46")   # nothing on the wire yet
bot.connect()                 # WS handshake: {"type":"connect"} → SESSION:A → TELEM:all
# ... do things ...
bot.close()                   # tears down the socket + recv thread

# or use it as a context manager — connect/close automatic:
with Robot("192.168.1.46") as bot:
    ...
```

Optional constructor kwargs:

| kwarg          | default | notes                                                  |
| :------------- | :------ | :----------------------------------------------------- |
| `port`         | `81`    | firmware's WS port                                     |
| `path`         | `/ws`   | firmware's WS route                                    |
| `session`      | `"A"`   | LAN session slot; `"B"` also exists for a 2nd viewer   |
| `telemetry`    | `"all"` | pass `None` to skip the `TELEM:` subscription          |
| `timeout`      | `5.0`   | initial connect timeout in seconds                     |
| `auto_reconnect` | `True` | reconnect in the background if the link drops        |

### Reconnection

Telemetry is subscribed automatically on `connect()` (the `telemetry="all"`
default), so you don't need to turn anything on. If the link drops — bot
reboot, idle timeout, WiFi blip — the SDK no longer crashes with a traceback.
With `auto_reconnect=True` (default) it reconnects in the background and keeps
working on the **same** `Robot` object. You can also reconnect manually:

```python
if not bot.is_connected():
    bot.connect()                 # reusable — no need to recreate Robot(...)

bot.on_disconnect(lambda why: print("lost bot:", why))   # optional notice
```

### Telemetry fields

The telemetry stream uses **short keys**. `wait_for_telemetry()` accepts both
the raw key and a friendly alias, so `wait_for_telemetry("temp")` and
`wait_for_telemetry("t")` both work:

| ask for | key | | ask for | key |
| :------ | :-- | :-- | :------ | :-- |
| `temp` / `temperature` | `t` | | `heading` / `yaw` | `ih` |
| `battery` | `b` | | `pitch` | `ipt` |
| `rssi` | `r` | | `roll` | `irl` |
| `fps` | `f` | | `accel_x/y/z` | `iax/iay/iaz` |
| `cpu` | `c` | | `distance` | `d` |
| `mem` / `memory` | `m` | | `flipped` | `ifl` |

IMU fields only arrive when the MPU6050 is enabled; distance only when a
rangefinder is enabled. `bot.telemetry()` returns the latest full snapshot so
you can see exactly which keys are coming in.

## Device namespaces

Every namespace is a thin wrapper over `bot.send(device, action, **params)`.
Each method matches a button in `/console` one-for-one.

```python
bot.gpio.write(pin=7, value=True)               # ESP32 GPIO high
bot.gpio.read(pin=7)                            # request; reply via on_message

bot.servo.set_angle(pin=4, angle=90)            # RMT-driven servo
bot.servo.detach(pin=4)

bot.rgb.configure(pin=2, num_leds=12)
bot.rgb.color(255, 128, 0)                      # solid RGB
bot.rgb.brightness(80)
bot.rgb.effect("rainbow", speed=200)
bot.rgb.off()

bot.pca9685.set_pwm(pin=0, value=2048)          # 0..4095 raw
bot.pca9685.set_angle(pin=4, angle=45)          # 0..180 servo

bot.stepper.step(steps=200, direction=1)
bot.stepper.forward(speed=180); bot.wait(1); bot.stepper.stop()

bot.mcp23008.set_mode(pin=0, mode="input_pullup")
bot.mcp23008.write(pin=1, value=True)

bot.camera.framesize("QVGA")                    # QQVGA .. UXGA
bot.camera.quality(10)                          # 4 (best) .. 63 (worst)
bot.camera.flip(h=True, v=False)
bot.camera.brightness(2)

bot.tof.get_distance()                          # center + nearest
bot.tof.get_grid()                              # all 16 or 64 zones
bot.tof.get_status()
bot.tof.set_orientation(rotation=180, flip_x=True)
bot.tof.calibrate_xtalk()                       # one-time, persists NVS
bot.tof.get_xtalk_status()                      # 'idle'|'running'|'ok'|'failed'

bot.imu.get_all()                               # heading / pitch / roll / accel / gyro / flipped
bot.imu.get_heading()                           # just yaw [0,360)°
bot.imu.reset_heading()                         # 0° = current direction
bot.imu.recalibrate()                           # keep bot still ~1 s

bot.car.set_steer_mode("dualgpio", left_pin=43, right_pin=44)
bot.car.set_steering(angle=45)                  # steer left
bot.car.forward(); bot.wait(0.5); bot.car.stop()

bot.system.get_battery()
bot.system.get_all()                            # heap/temp/cpu/uptime/…
bot.system.set_name("MightyRider3")
bot.system.set_battery_divider(5.67)            # 120k-R7 batch
bot.system.set_component_enabled("imu", True)   # enable IMU; restart to apply
bot.system.restart()                            # hard reboot

bot.wifi.get_all()                              # IP/RSSI/MAC/…

bot.drive.forward(power=100, ms=500)            # LAN car/tank drive
bot.drive.left(power=60, ms=200)
bot.drive.backward(power=40); bot.drive.stop()
```

## Streaming telemetry

The bot broadcasts a telemetry frame every ~1 s (rate depends on which
`TELEM:` filter you subscribed to). Register a callback:

The raw dict uses **short keys** (see the Telemetry fields table above):
`t`=°C, `r`=rssi, `h`=heap KB, `b`=battery %, `u`=uptime s, `dg`=ToF grid.

```python
def on_telem(t):
    print(f"temp {t.get('t')} °C   rssi {t.get('r')} dBm   "
          f"heap {t.get('h')} KB   bat {t.get('b')} %")

bot.on_telemetry(on_telem)
bot.wait(10)                     # keep the main thread alive
```

Or read the latest snapshot on demand:

```python
snap = bot.telemetry()
print(snap.get("u"), snap.get("dg"))    # u = uptime (s), dg = ToF grid array (16 or 64 mm)
```

## Ask-and-wait pattern

Commands like `bot.tof.get_grid()` are fire-and-forget; the reply arrives
as a separate JSON message. Use `wait_for` when you need the reply:

```python
reply = bot.wait_for(
    lambda m: m.get("device") == "rangefinder",
    ask=bot.tof.get_grid,
    timeout=2.0,
)
if reply:
    print("got", reply)
else:
    print("timeout")
```

`wait_for` drains stale queued messages before firing `ask` so you always
get the response to *your* request, not a message that was already in the
queue when you called it.

For simple telemetry-field reads there's a shortcut:

```python
battery = bot.wait_for_telemetry("b",    timeout=3)   # % or -1 if warming up
temp_c  = bot.wait_for_telemetry("temp", timeout=3)
```

## Video frames

When the camera is up, every JPEG frame lands in a `video` callback:

```python
def save(jpeg, latency_ms):
    with open("frame.jpg", "wb") as f:
        f.write(jpeg)
    bot.close()                       # one-shot

bot.on_video(save)
bot.wait(5)                           # wait up to 5 s for a frame
```

`jpeg` is raw bytes — decode with Pillow, OpenCV, or write straight to disk.

## Escape hatch: raw `send()`

If the firmware exposes a device the SDK doesn't wrap yet, or you want to
send a one-off command, use `.send(device, action, **params)` — it produces
the exact same `CMD:{…}` frame the browser sends:

```python
bot.send("gpio", "write", pin=21, value=True)
bot.send("system", "setBatteryDivider", ratio=6.6)
```

## Testing

The SDK doubles as a regression test harness. Example structure:

```python
import time
from zrsense import Robot

def test_rgb_smoketest():
    with Robot("192.168.1.46", timeout=3) as bot:
        for r, g, b in [(255,0,0), (0,255,0), (0,0,255)]:
            bot.rgb.color(r, g, b)
            time.sleep(0.5)
        bot.rgb.off()

def test_telemetry_alive():
    with Robot("192.168.1.46") as bot:
        temp = bot.wait_for_telemetry("temp", timeout=3)
        assert temp is not None and 20 <= temp <= 90, f"CPU temp implausible: {temp}"
```

Wire that into `pytest` and it's a real hardware-loop test suite.

## Threading model

- One background daemon thread pumps `ws.recv()` and dispatches every frame
  to registered callbacks. Callbacks run on that thread — don't do long
  blocking work in them; hand off to a queue or another thread.
- `bot.send(...)` is called from your main thread and is guarded by an
  internal lock so it's safe to call concurrently from other threads too.

## Examples

See [`examples/`](examples/) for runnable scripts:

- [`examples/blink_rgb.py`](examples/blink_rgb.py) — smallest possible SDK use
- [`examples/drive_wiggle.py`](examples/drive_wiggle.py) — drive a short pattern
- [`examples/telemetry_monitor.py`](examples/telemetry_monitor.py) — stream telem to stdout

## Protocol reference

Everything the SDK does is a wrapper over these three frame types on
`ws://<ip>:81/ws`:

| Direction | Frame                                                     | Meaning                                             |
| :-------- | :-------------------------------------------------------- | :-------------------------------------------------- |
| → bot     | `{"type":"connect"}`                                      | first handshake step                                |
| → bot     | `SESSION:A`                                               | claim LAN slot A (or B for a second viewer)         |
| → bot     | `TELEM:all`                                               | subscribe to every telemetry field                  |
| → bot     | `CMD:{"device":"…", "action":"…", "params":{…}}`          | any command                                         |
| ← bot     | `{"type":"connected", "mode":"A"}`                        | handshake OK                                        |
| ← bot     | `{"type":"telem", "d":{…}}`                               | telemetry payload                                   |
| ← bot     | `{"type":"camera_lost"}` / `{"type":"camera_ready"}`      | camera hot-swap events                              |
| ← bot     | `{...device-specific...}`                                 | command responses                                   |
| ← bot     | binary: JPEG (optionally prefixed with `'T'` + 4-byte ts) | one video frame                                     |

The same protocol is served on the cloud path (`wss://…`) but that flow
uses WebRTC data channels rather than a direct WebSocket; the SDK targets
LAN mode only.
