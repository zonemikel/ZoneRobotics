# ZRSense Python SDK — Commands Cheat Sheet

Quick scan of every call surface. All examples assume:

```python
from zrsense import Robot
bot = Robot("192.168.1.46")
bot.connect()
import time; time.sleep(0.5)
```

Replies arrive asynchronously over the WebSocket. Register `bot.on_message(lambda m: print("RAW:", m))` to see them, or use `bot.wait_for(filter, ask=…, timeout=…)` to get a specific reply back as a dict.

---

## Steering a car (`bot.car`)

Uses the CarDevice — single rear motor + steering (servo or dual-GPIO).

| Call | What it does |
|---|---|
| `bot.car.forward()` | Full throttle forward |
| `bot.car.backward()` | Full throttle backward |
| `bot.car.stop()` | Motor off |
| `bot.car.raw(throttle)` | `-100..+100` — negative = reverse |
| `bot.car.left(offset=90)` | Steer left. `offset` 0 = straight, 90 = full lock |
| `bot.car.right(offset=90)` | Steer right. Same scale |
| `bot.car.center()` | Steering back to straight |
| `bot.car.set_steering(angle)` | `0..180°` — direct servo angle |
| `bot.car.set_motor_pins(pin_a, pin_b, ctrl_a='esp32', ctrl_b='esp32')` | Reassign H-bridge pins |
| `bot.car.set_steer_pin(pin, ctrl='esp32')` | Reassign servo pin |
| `bot.car.set_steer_config(center=90, range_deg=45)` | Trim centre + max offset |
| `bot.car.set_steer_mode(mode, left_pin=43, right_pin=44)` | `'servo'` or `'dualgpio'` |
| `bot.car.set_power(power)` | Global throttle scale `0..100` (PCA9685 only) |
| `bot.car.set_duration(ms)` | Auto-stop timeout; 0 = disabled |
| `bot.car.get_status()` | Snapshot all pins / mode / power |

## Driving a tank or generic 2-wheel (`bot.drive`)

Uses DriverDevice — unified `car`/`tank` action. Works on either driver kind.

| Call | What it does |
|---|---|
| `bot.drive.forward(power=100)` | Both motors forward |
| `bot.drive.backward(power=100)` | Both reverse |
| `bot.drive.left(power=100)` | Pivot left |
| `bot.drive.right(power=100)` | Pivot right |
| `bot.drive.stop()` | All motors off |
| `bot.drive.tank(left, right)` | Independent `-255..+255` per side |
| `bot.drive.set_speed(left, right)` | Alias — same shape |

## IMU — orientation (`bot.imu`)

Returns `{heading, pitch, roll, ax, ay, az, gx, gy, gz, flipped}`.

```python
reply = bot.wait_for(lambda m: 'heading' in m,
                     ask=bot.imu.get_heading, timeout=2.0)
print(reply['heading'])
```

| Call | What it does |
|---|---|
| `bot.imu.get_all()` | Full snapshot |
| `bot.imu.get_heading()` | Just yaw `[0,360)°` |
| `bot.imu.reset_heading()` | Current direction = 0° |
| `bot.imu.recalibrate()` | 1 s gyro-bias cal (hold bot still) |

## Rangefinder — VL53L5CX ToF (`bot.tof`)

```python
reply = bot.wait_for(lambda m: 'zoneCount' in m,
                     ask=bot.tof.get_grid, timeout=2.0)
# reply['zones'] = [{'i', 'distanceMm', 'status', 'targets'}, ...]
```

| Call | What it does |
|---|---|
| `bot.tof.get_distance()` | Center-of-FoV + nearest-anywhere, in mm |
| `bot.tof.get_grid()` | All 16 or 64 zones with status |
| `bot.tof.get_all()` | Same as get_distance |
| `bot.tof.get_status()` | ready / sensorStatus / silicon temp |
| `bot.tof.get_config()` | rotation / flipX/Y / resolution |
| `bot.tof.set_orientation(rotation=0, flip_x=False, flip_y=False)` | Live rotate / flip, persists NVS |
| `bot.tof.set_resolution(mode)` | `'4x4'` or `'8x8'`. Reboots bot |
| `bot.tof.calibrate_xtalk(reflect_pct=3, distance_mm=600, samples=4)` | One-time crosstalk cal |
| `bot.tof.get_xtalk_status()` | `'idle' / 'running' / 'ok' / 'failed'` |

## Camera (`bot.camera`)

```python
jpg = bot.camera.grab_frame(timeout=2.0)
if jpg:
    open("snap.jpg","wb").write(jpg)
```

| Call | What it does |
|---|---|
| `bot.camera.framesize(size)` | `QQVGA / QVGA / HVGA / VGA / SVGA / XGA / HD / SXGA / UXGA` |
| `bot.camera.quality(value)` | JPEG quality `4..63` (4 = best) |
| `bot.camera.flip(h=None, v=None)` | Mirror / vertical flip |
| `bot.camera.brightness(value)` | `-2..+2` |
| `bot.camera.contrast(value)` | `-2..+2` |
| `bot.camera.saturation(value)` | `-2..+2` |
| `bot.camera.take_photo()` | Save to SD card (if mounted) |
| **`bot.camera.grab_frame(timeout=2.0)`** | **Return next JPEG as bytes — pass to cv2 / PIL** |

Grab + decode with OpenCV:
```python
import cv2, numpy as np
jpg = bot.camera.grab_frame()
img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
```

## GPIO — raw pins (`bot.gpio`)

| Call | What it does |
|---|---|
| `bot.gpio.mode(pin, mode)` | `'input'` / `'output'` / `'input_pullup'` / `'input_pulldown'` |
| `bot.gpio.write(pin, value)` | Set HIGH/LOW |
| `bot.gpio.read(pin)` | Query current level (via reply) |

## MCP23008 — I²C GPIO expander (`bot.mcp23008`)

| Call | What it does |
|---|---|
| `bot.mcp23008.set_mode(pin, mode)` | `'input' / 'output' / 'input_pullup'` |
| `bot.mcp23008.write(pin, value)` | HIGH / LOW |
| `bot.mcp23008.read(pin)` | Read level |

## Servo — RMT direct PWM (`bot.servo`)

| Call | What it does |
|---|---|
| `bot.servo.set_angle(pin, angle)` | 0..180° |
| `bot.servo.detach(pin)` | Stop PWM on pin |

## PCA9685 — I²C PWM driver (`bot.pca9685`)

| Call | What it does |
|---|---|
| `bot.pca9685.set_pwm(pin, value)` | Raw 0..4095 duty |
| `bot.pca9685.set_angle(pin, angle)` | 0..180° (servo-style) |

## RGB LED (`bot.rgb`)

| Call | What it does |
|---|---|
| `bot.rgb.configure(pin, num_leds)` | Set up strip (one-time) |
| `bot.rgb.color(r, g, b)` | Set solid colour |
| `bot.rgb.brightness(value)` | 0..255 |
| `bot.rgb.effect(name, speed=128)` | `'rainbow' / 'chase' / 'pulse' / 'strobe' / 'fade' / 'twinkle' / 'fire' / 'comet'` |
| `bot.rgb.off()` | Clear |

## Stepper (`bot.stepper`)

| Call | What it does |
|---|---|
| `bot.stepper.step(steps, direction=1)` | Discrete steps (`direction` = 1 or -1) |
| `bot.stepper.forward(speed=128)` | Continuous forward |
| `bot.stepper.backward(speed=128)` | Continuous backward |
| `bot.stepper.stop()` | Halt |

## System (`bot.system`)

| Call | What it does |
|---|---|
| `bot.system.get_all()` | heap / temp / cpu / uptime / battery |
| `bot.system.get_battery()` | Battery pct + mV + raw ADC |
| `bot.system.set_name(name)` | Rename bot (persists NVS) |
| `bot.system.get_name()` | Read current name |
| `bot.system.set_battery_divider(ratio)` | Trim ADC divider; `6.6` default, `5.67` for 120k-R7 batch |
| `bot.system.get_component_states()` | Which components (camera/imu/…) are enabled |
| `bot.system.set_component_enabled(component, enabled)` | Toggle. **Restart to apply.** |
| `bot.system.restart()` | Reboot the bot |

## WiFi (`bot.wifi`)

| Call | What it does |
|---|---|
| `bot.wifi.get_all()` | IP / RSSI / SSID / MAC |

---

## Patterns

### Fire-and-forget (no reply needed)
```python
bot.rgb.color(0, 255, 0)
bot.car.forward()
```

### Capture a reply
```python
reply = bot.wait_for(lambda m: 'heading' in m,
                     ask=bot.imu.get_heading, timeout=2.0)
print(reply['heading'])
```

### Stream telemetry push
```python
bot.on_telemetry(lambda t: print(t.get('ih'), '°'))
bot.wait(10)
```

### Grab video frame
```python
jpg = bot.camera.grab_frame(timeout=2.0)
```

### Lifecycle
```python
with Robot("192.168.1.46") as bot:
    bot.car.forward(); bot.wait(1); bot.car.stop()
# auto-closes on exit
```

### Important rules
- **One WebSocket client at a time** — close the browser `/car` tab while scripting.
- **`time.sleep(0.5)` after `connect()`** before the first command, or expect `imu not present` from a race with device registration.
- Firmware response payloads **do not include a `"device"` field** — filter `wait_for` lambdas by content keys (`'heading' in m`, `'zoneCount' in m`, …).
- `websocket-client` must be `1.6.4` or older — 1.9+ strict-rejects frames with RSV bits set, which the ESP32 LAN server emits.

```
python -m pip install "websocket-client==1.6.4"
```
