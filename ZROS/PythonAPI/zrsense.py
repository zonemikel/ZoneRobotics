"""
ZRSense Python SDK
==================

Talk to a ZoneRobotics ESP32 bot over LAN from Python. This wraps the exact
same WebSocket + JSON command protocol the /console page in the firmware
uses, so anything you can click in the browser you can script here.

Install
-------
    python -m pip install "websocket-client==1.6.4"

    IMPORTANT: pin 1.6.4. websocket-client 1.9+ strict-validates frames and
    rejects the bot's WebSocket frames with "rsv is not implemented, yet" or
    "Invalid opcode", which kills the receive loop. If you see those errors,
    you are on a newer version — reinstall with the pin above.

Quick start
-----------
    from zrsense import Robot

    with Robot("192.168.1.46") as bot:
        bot.rgb.color(0, 255, 0)   # green
        bot.wait(1)
        bot.rgb.off()

        # Ask for a ToF frame and wait for the reply
        grid = bot.wait_for(lambda m: m.get("device") == "rangefinder",
                            ask=bot.tof.get_grid)
        print(grid)

Protocol reference
------------------
The bot listens on ws://<ip>:81/ws and speaks three kinds of frame:

  * Handshake:  {"type":"connect"}  then  "SESSION:A"  then  "TELEM:all"
  * Command:    "CMD:" + json({"device":"...", "action":"...", "params":{...}})
  * Response:   json({"type":"telem", "d":{...}})  |  json({"type":"connected", ...})
                |  json({...device-specific...})  |  binary JPEG frame

This library gives you a device-namespaced sync API for the common cases
plus `.send(device, action, **params)` for anything the SDK doesn't yet
wrap. New device commands added to the firmware work through `.send()`
immediately — no SDK update needed.
"""
from __future__ import annotations

import json
import logging
import queue
import threading
import time
from typing import Any, Callable, Dict, List, Optional

try:
    import websocket  # pip install websocket-client
except ImportError as _e:  # pragma: no cover
    raise ImportError(
        "zrsense requires the 'websocket-client' package.\n"
        "  Install with:  pip install websocket-client"
    ) from _e

__all__ = ["Robot", "ZRSenseError"]
__version__ = "0.1.0"

log = logging.getLogger("zrsense")

# Telemetry frames use terse single/short keys. Map friendly names people
# naturally reach for onto the real keys so wait_for_telemetry("temp") works.
# (The TELEM subscription filter uses the friendly names, but the emitted
# telemetry KEYS are short — that mismatch tripped up early testers.)
_TELEM_ALIASES = {
    "temp": "t", "temperature": "t",
    "battery": "b", "batt": "b",
    "rssi": "r", "signal": "r",
    "fps": "f",
    "heap": "h",
    "cpu": "c", "cpuload": "c",
    "mem": "m", "memory": "m",
    "uptime": "u",
    "distance": "d", "dist": "d", "range": "d",
    "heading": "ih", "yaw": "ih",
    "pitch": "ipt",
    "roll": "irl",
    "accel_x": "iax", "ax": "iax",
    "accel_y": "iay", "ay": "iay",
    "accel_z": "iaz", "az": "iaz",
    "flipped": "ifl", "flip": "ifl",
}


class ZRSenseError(RuntimeError):
    """Base class for SDK errors (connection lifecycle, timeouts, etc.)."""


# ══════════════════════════════════════════════════════════════════════
#  Device namespaces — thin wrappers over Robot.send(device, action, ...)
# ══════════════════════════════════════════════════════════════════════

class _Device:
    device_name = ""
    def __init__(self, bot: "Robot"):
        self._bot = bot
    def _s(self, action: str, **params) -> None:
        self._bot.send(self.device_name, action, **params)


class _GPIO(_Device):
    """Direct ESP32 GPIO — write/read a pin, or query the pin-mode map."""
    device_name = "gpio"
    def write(self, pin: int, value: bool) -> None:
        self._s("write", pin=int(pin), value=bool(value))
    def read(self, pin: int) -> None:
        """Request the pin state. Response arrives via `on_message`."""
        self._s("read", pin=int(pin))
    def get_config(self) -> None:
        self._s("getConfig")


class _Servo(_Device):
    """RMT-driven servos on any ESP32 pin (not via PCA9685)."""
    device_name = "servo"
    def set_angle(self, pin: int, angle: int) -> None:
        self._s("setAngle", pin=int(pin), angle=max(0, min(180, int(angle))))
    def detach(self, pin: int) -> None:
        self._s("detach", pin=int(pin))


class _RGB(_Device):
    """WS2812 / SK6812 strip driver."""
    device_name = "rgb"
    def configure(self, pin: int, num_leds: int) -> None:
        self._s("configure", pin=int(pin), numLeds=int(num_leds))
    def color(self, r: int, g: int, b: int) -> None:
        self._s("setColor", r=int(r), g=int(g), b=int(b))
    def brightness(self, value: int) -> None:
        self._s("setBrightness", brightness=max(0, min(255, int(value))))
    def effect(self, name: str, speed: int = 128) -> None:
        """Common effects: 'rainbow' | 'chase' | 'breathe' | 'sparkle' | 'off'."""
        self._s("effect", effect=str(name), speed=int(speed))
    def off(self) -> None:
        self._s("off")


class _PCA9685(_Device):
    """PCA9685 16-channel I²C PWM driver (motors + servos)."""
    device_name = "pca9685"
    def set_pwm(self, pin: int, value: int) -> None:
        """Raw PWM value 0..4095 on a channel."""
        self._s("setPWM", pin=int(pin), value=int(value))
    def set_angle(self, pin: int, angle: int) -> None:
        """Convenience: pulse-width mapped to 0..180° for standard hobby servos."""
        self._s("setAngle", pin=int(pin), angle=int(angle))


class _Stepper(_Device):
    """Stepper motor via the firmware's stepper task."""
    device_name = "stepper"
    def step(self, steps: int, direction: int = 1) -> None:
        self._s("step", steps=int(steps), direction=1 if direction >= 0 else -1)
    def forward(self, speed: int = 128) -> None:
        self._s("forward", speed=int(speed))
    def backward(self, speed: int = 128) -> None:
        self._s("backward", speed=int(speed))
    def stop(self) -> None:
        self._s("stop")


class _MCP23008(_Device):
    """MCP23008 I²C 8-channel GPIO expander."""
    device_name = "mcp23008"
    def set_mode(self, pin: int, mode: str) -> None:
        """mode: 'input' | 'output' | 'input_pullup'"""
        self._s("setMode", pin=int(pin), mode=str(mode))
    def write(self, pin: int, value: bool) -> None:
        self._s("write", pin=int(pin), value=bool(value))
    def read(self, pin: int) -> None:
        self._s("read", pin=int(pin))


class _Camera(_Device):
    """Camera (OV3660 / OV2640) — resolution, JPEG quality, flip, colour."""
    device_name = "camera"
    def framesize(self, size: str) -> None:
        """QQVGA | QVGA | HVGA | VGA | SVGA | XGA | HD | SXGA | UXGA"""
        self._s("set_framesize", value=str(size))
    def quality(self, value: int) -> None:
        """JPEG quality 4..63  (4 = best, 63 = worst)."""
        self._s("set_quality", value=int(value))
    def flip(self, h: Optional[bool] = None, v: Optional[bool] = None) -> None:
        if h is not None: self._s("set_hmirror", value=bool(h))
        if v is not None: self._s("set_vflip",   value=bool(v))
    def brightness(self, value: int) -> None:  self._s("set_brightness", value=int(value))
    def contrast(self, value: int) -> None:    self._s("set_contrast",   value=int(value))
    def saturation(self, value: int) -> None:  self._s("set_saturation", value=int(value))
    def take_photo(self) -> None:
        """Save a photo to the bot's SD card (if mounted)."""
        self._s("takePhoto")
    def grab_frame(self, timeout: float = 2.0) -> Optional[bytes]:
        """Grab the next JPEG frame from the live LAN video stream and
        return it as raw bytes (what you'd write straight into a .jpg
        file, or feed to cv2.imdecode / PIL.Image.open).

        Blocks for up to `timeout` seconds; returns None on timeout.

        Only works while a LAN video session is active — the firmware
        pushes JPEG frames down the same WebSocket as binary payloads.
        If you're not seeing frames, the viewer / camera may be off;
        check `bot.last_telemetry` or hit bot.camera.framesize('QVGA')
        first to force a stream.

        Example:
            jpg = bot.camera.grab_frame()
            if jpg:
                open("snap.jpg","wb").write(jpg)
                # or: import cv2, numpy as np
                # img = cv2.imdecode(np.frombuffer(jpg,np.uint8), cv2.IMREAD_COLOR)
        """
        import threading
        evt = threading.Event()
        result: List[Optional[bytes]] = [None]
        def _cb(data: bytes, _latency_ms: int) -> None:
            if result[0] is None:
                result[0] = bytes(data)
                evt.set()
        self._bot._video_cbs.append(_cb)
        try:
            evt.wait(float(timeout))
        finally:
            try: self._bot._video_cbs.remove(_cb)
            except ValueError: pass
        return result[0]


class _Rangefinder(_Device):
    """VL53L5CX 8×8 (or 4×4) time-of-flight sensor."""
    device_name = "rangefinder"
    def get_distance(self) -> None: self._s("getDistance")
    def get_grid(self) -> None:     self._s("getGrid")
    def get_all(self) -> None:      self._s("getAll")
    def get_status(self) -> None:   self._s("getStatus")
    def get_config(self) -> None:   self._s("getConfig")
    def set_orientation(self, rotation: int = 0, flip_x: bool = False, flip_y: bool = False) -> None:
        """Live rotate / flip the grid. rotation in {0,90,180,270}. Persists to NVS."""
        self._s("setOrientation", rotation=int(rotation), flipX=bool(flip_x), flipY=bool(flip_y))
    def set_resolution(self, mode: str) -> None:
        """mode: '4x4' or '8x8'. Persists to NVS and auto-restarts the bot."""
        self._s("setResolution", mode=str(mode))
    def calibrate_xtalk(self, reflect_pct: int = 3, distance_mm: int = 600, samples: int = 4) -> None:
        """Run cover-glass crosstalk calibration. Point sensor at a Lambertian
        target at ~distance_mm in open space first. Fire-and-forget — poll
        get_xtalk_status until it returns state='ok'. Blob persists to NVS and
        auto-loads on every boot."""
        self._s("calibrateXtalk", reflectPct=int(reflect_pct),
                distanceMm=int(distance_mm), samples=int(samples))
    def get_xtalk_status(self) -> None:
        """Returns state: 'idle'|'running'|'ok'|'failed' plus lastStatus:int."""
        self._s("getXtalkStatus")


class _IMU(_Device):
    """MPU6050 6-DOF IMU. Heading, pitch, roll, accel, gyro + flipped-over bool.
    Polled at 20 Hz on the bot (after a 15 s boot grace). Gated by SYS_COMP_IMU
    — if your bot has no MPU6050, this is a no-op and `get_all` returns
    'imu not present'. Enable/disable at runtime via
    `bot.system.set_component_enabled('imu', True/False)`."""
    device_name = "imu"
    def get_all(self) -> None:
        """Full snapshot: heading, pitch, roll, ax/ay/az, gx/gy/gz, flipped."""
        self._s("getAll")
    def get_heading(self) -> None:
        """Fast path: just the integrated yaw [0, 360)°."""
        self._s("getHeading")
    def reset_heading(self) -> None:
        """Zero the integrated heading — wherever bot points now becomes 0°."""
        self._s("resetHeading")
    def recalibrate(self) -> None:
        """Re-run the 1 s gyro-bias static calibration. Keep the bot still."""
        self._s("recalibrate")


class _Car(_Device):
    """Car driver — single rear motor + steering. Steering mode is either
    'servo' (PWM-angle a hobby servo on steerPin) or 'dualgpio' (two
    digital GPIOs HIGH/LOW on leftPin/rightPin). All config persists NVS."""
    device_name = "car"
    def forward(self) -> None:  self._s("forward")
    def backward(self) -> None: self._s("backward")
    def stop(self) -> None:     self._s("stop")
    def raw(self, throttle: int) -> None:
        """throttle in -100..+100 (negative = reverse)."""
        self._s("raw", throttle=int(throttle))
    def set_steering(self, angle: int) -> None:
        """angle 0..180° — 90 is centred by default (see setSteerConfig)."""
        self._s("setSteering", angle=int(angle))
    def center_steering(self) -> None: self._s("centerSteering")
    # Convenience steering helpers for cars (independent of forward/stop).
    # The firmware's doSetSteering clamps to the configured steerCenter ±
    # steerRange, so sending 0° / 180° gives you "full lock" regardless of
    # what the user configured. Pass offset=0..90 if you want a shallower
    # turn — 90 = full lock, 20 = gentle, etc.
    def left (self, offset: int = 90) -> None:
        """Steer left. offset = how far from center (0=straight, 90=full lock)."""
        self._s("setSteering", angle=max(0, 90 - int(offset)))
    def right(self, offset: int = 90) -> None:
        """Steer right. offset = how far from center (0=straight, 90=full lock)."""
        self._s("setSteering", angle=min(180, 90 + int(offset)))
    def center(self) -> None:
        """Alias for center_steering() — straight ahead."""
        self._s("centerSteering")
    def set_motor_pins(self, pin_a: int, pin_b: int,
                       ctrl_a: str = "esp32", ctrl_b: str = "esp32") -> None:
        """ctrl_a / ctrl_b: 'esp32' or 'pca9685'."""
        self._s("setMotorPins", pinA=int(pin_a), ctrlA=ctrl_a,
                pinB=int(pin_b), ctrlB=ctrl_b)
    def set_steer_pin(self, pin: int, ctrl: str = "esp32") -> None:
        self._s("setSteerPin", pin=int(pin), ctrl=ctrl)
    def set_steer_config(self, center: int = 90, range_deg: int = 45) -> None:
        """center = angle at straight-ahead (90 for most servos),
        range_deg = max ± offset from center (lower if wheels bind at full lock)."""
        self._s("setSteerConfig", center=int(center), range=int(range_deg))
    def set_steer_mode(self, mode: str = "servo",
                       left_pin: int = 43, right_pin: int = 44) -> None:
        """mode: 'servo' (one PWM pin, uses setSteerPin/setSteerConfig) or
        'dualgpio' (two digital pins HIGH when steering that side, LOW otherwise).
        Previous servo settings are preserved when switching to dualgpio."""
        self._s("setSteerMode", mode=mode,
                leftPin=int(left_pin), rightPin=int(right_pin))
    def set_power(self, power: int) -> None:
        """0..100 — throttle scale on PCA9685 motor controller (no-op on direct GPIO)."""
        self._s("setPower", power=max(0, min(100, int(power))))
    def set_duration(self, ms: int) -> None:
        """Auto-stop timeout. 0 = disabled (hold-to-drive); otherwise ms until stop."""
        self._s("setDuration", duration=int(ms))
    def get_status(self) -> None: self._s("getStatus")


class _System(_Device):
    """Bot-level info: battery, temperature, heap, uptime, restart, name,
    component gating."""
    device_name = "system"
    def get_battery(self) -> None: self._s("getBattery")
    def get_all(self) -> None:     self._s("getAll")
    def restart(self) -> None:     self._s("restart")
    def set_name(self, name: str) -> None:
        """Rename the bot (1–31 alphanumeric / _-). Persists to NVS, takes
        effect on next signaling reconnect."""
        self._s("setName", name=str(name))
    def get_name(self) -> None: self._s("getName")
    def set_battery_divider(self, ratio: float) -> None:
        """Set the ADC → battery mV divider ratio. Default 6.6 (R15 560k + R7 100k).
        Use 5.67 for the 120k-R7 batch. Persists to NVS."""
        self._s("setBatteryDivider", value=float(ratio))
    def get_component_states(self) -> None:
        """Returns per-component {name, pref, active} for camera / mcp23008 /
        pca9685 / rangefinder / imu."""
        self._s("getComponentStates")
    def set_component_enabled(self, component: str, enabled: bool) -> None:
        """component: 'camera' | 'mcp23008' | 'pca9685' | 'rangefinder' | 'imu'.
        Persists to NVS, takes effect on next restart."""
        self._s("setComponentEnabled", component=str(component), enabled=bool(enabled))


class _WiFi(_Device):
    device_name = "wifi"
    def get_all(self) -> None: self._s("getAll")


class _Drive(_Device):
    """Generic 2-wheel driver device — unified car/tank interface for the
    DriverDevice (firmware id='driver'). car() takes a direction string;
    tank() takes independent left/right speeds. Prefer bot.car.* / bot.tank
    for anything beyond basic nudging."""
    device_name = "driver"
    def forward (self, power: int = 100) -> None: self._s("car", direction="forward",  speed=int(power))
    def backward(self, power: int = 100) -> None: self._s("car", direction="backward", speed=int(power))
    def left    (self, power: int = 100) -> None: self._s("car", direction="left",     speed=int(power))
    def right   (self, power: int = 100) -> None: self._s("car", direction="right",    speed=int(power))
    def stop    (self) -> None:                   self._s("stop")
    def tank    (self, left: int, right: int) -> None:
        """Independent per-side speeds, each -255..+255 (signed)."""
        self._s("tank", leftSpeed=int(left), rightSpeed=int(right))
    def set_speed(self, left: int, right: int) -> None:
        self._s("setSpeed", left=int(left), right=int(right))


# ══════════════════════════════════════════════════════════════════════
#  Robot — the connection + device namespaces
# ══════════════════════════════════════════════════════════════════════

class Robot:
    """A single ZRSense LAN bot.

    Basic usage:

        with Robot("192.168.1.46") as bot:
            bot.gpio.write(pin=7, value=True)   # headlight on
            bot.rgb.color(0, 255, 0)
            bot.wait(1)

    Streaming telemetry (fires ~1/s from the firmware). The dict uses SHORT
    keys: t=°C, r=rssi, h=heap KB, b=battery %, u=uptime s (see
    wait_for_telemetry's docstring for the full map):

        bot.on_telemetry(lambda t: print(t.get("t"), "°C"))

    Video (only if the bot's camera is up):

        def save(frame_bytes, latency_ms):
            open("frame.jpg", "wb").write(frame_bytes)
        bot.on_video(save)

    Ask-and-wait (for commands that return a value):

        reply = bot.wait_for(lambda m: m.get("device") == "system",
                             ask=bot.system.get_battery, timeout=2.0)
        print(reply)

    Device namespaces:
        bot.gpio      bot.servo     bot.rgb      bot.pca9685
        bot.stepper   bot.mcp23008  bot.camera   bot.tof       bot.imu
        bot.system    bot.wifi      bot.drive    bot.car
    """

    def __init__(
        self,
        host: str,
        port: int = 81,
        path: str = "/ws",
        session: str = "A",
        telemetry: Optional[str] = "all",
        timeout: float = 5.0,
        auto_reconnect: bool = True,
    ):
        self.host = host
        self.port = port
        self.path = path
        self.session = session
        self.telemetry_filter = telemetry
        self.timeout = float(timeout)
        # When True, an unexpected drop (bot reboot, idle timeout, WiFi blip)
        # is handled in the background: the recv loop won't crash, state is
        # reset, and a reconnect thread retries with backoff until it's back.
        # The same Robot object keeps working — no need to recreate it.
        self.auto_reconnect = bool(auto_reconnect)

        # WebSocket + receive thread state
        self._ws: Optional[websocket.WebSocket] = None
        self._recv_thread: Optional[threading.Thread] = None
        self._reconnect_thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._send_lock = threading.Lock()
        self._conn_lock = threading.Lock()   # serialises connect/reconnect

        # Callback lists (thread-safe under GIL for append/iterate-copy)
        self._telem_cbs: List[Callable[[Dict[str, Any]], None]] = []
        self._video_cbs: List[Callable[[bytes, int], None]] = []
        self._msg_cbs:   List[Callable[[Dict[str, Any]], None]] = []
        self._disconnect_cbs: List[Callable[[str], None]] = []

        # Latest telemetry snapshot
        self._last_telem: Dict[str, Any] = {}
        # Queue of every non-telem JSON message, used by wait_for()
        self._response_q: "queue.Queue[Dict[str, Any]]" = queue.Queue(maxsize=256)

        # Device namespaces
        self.gpio      = _GPIO(self)
        self.servo     = _Servo(self)
        self.rgb       = _RGB(self)
        self.pca9685   = _PCA9685(self)
        self.stepper   = _Stepper(self)
        self.mcp23008  = _MCP23008(self)
        self.camera    = _Camera(self)
        self.tof       = _Rangefinder(self)
        self.imu       = _IMU(self)
        self.car       = _Car(self)
        self.system    = _System(self)
        self.wifi      = _WiFi(self)
        self.drive     = _Drive(self)

    # ── context manager ─────────────────────────────────────────────
    def __enter__(self) -> "Robot":
        self.connect()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ── lifecycle ───────────────────────────────────────────────────
    def connect(self) -> None:
        """Open the WebSocket and run the ZR handshake.

        Reusable: if an earlier connection dropped (bot reboot, idle timeout,
        protocol error), just call ``bot.connect()`` again on the SAME object
        — no need to recreate ``Robot(...)``. With ``auto_reconnect=True``
        (the default) the SDK does this for you in the background anyway.
        """
        with self._conn_lock:
            if self._ws is not None:
                return  # already connected
            # Reap a dead recv thread from a previous (dropped) session so we
            # don't leak threads across reconnects.
            if self._recv_thread and self._recv_thread.is_alive() \
               and self._recv_thread is not threading.current_thread():
                self._recv_thread.join(timeout=1.0)
            url = f"ws://{self.host}:{self.port}{self.path}"
            log.info("connecting %s", url)
            try:
                ws = websocket.create_connection(
                    url, timeout=self.timeout, enable_multithread=True,
                )
            except Exception as e:
                raise ZRSenseError(f"connect failed: {url}: {e}") from e
            # Non-blocking recv (short timeout) so the recv loop can honour ._stop.
            ws.settimeout(0.5)
            self._ws = ws
            self._stop.clear()
            self._recv_thread = threading.Thread(
                target=self._recv_loop, name="zrsense-recv", daemon=True,
            )
            self._recv_thread.start()
            # Handshake: same three frames the browser /console page sends.
            self._raw_send(json.dumps({"type": "connect"}))
            self._raw_send(f"SESSION:{self.session}")
            if self.telemetry_filter:
                self._raw_send(f"TELEM:{self.telemetry_filter}")

    def close(self) -> None:
        """Tear down the WebSocket + recv thread (and stop auto-reconnect)."""
        self._stop.set()   # tells recv loop + reconnect loop this is intentional
        ws, self._ws = self._ws, None
        if ws is not None:
            try: ws.close()
            except Exception: pass
        if self._recv_thread and self._recv_thread.is_alive() \
           and self._recv_thread is not threading.current_thread():
            self._recv_thread.join(timeout=1.0)

    def is_connected(self) -> bool:
        return self._ws is not None and not self._stop.is_set()

    # ── raw send / high-level send ──────────────────────────────────
    def _raw_send(self, payload: str) -> None:
        if self._ws is None:
            raise ZRSenseError("not connected — call .connect() first")
        with self._send_lock:
            self._ws.send(payload)

    def send(self, device: str, action: str, **params) -> None:
        """Send an arbitrary CMD frame.

        Same shape /console uses:  CMD:{"device":..., "action":..., "params":{...}}
        Use this for any device command the SDK doesn't yet wrap.
        """
        cmd: Dict[str, Any] = {"device": device, "action": action}
        if params:
            cmd["params"] = params
        self._raw_send("CMD:" + json.dumps(cmd, separators=(",", ":")))

    # ── receive loop ────────────────────────────────────────────────
    def _recv_loop(self) -> None:
        reason: Optional[str] = None
        ws = self._ws
        while not self._stop.is_set() and ws is not None and ws is self._ws:
            try:
                data = ws.recv()
            except websocket.WebSocketTimeoutException:
                continue
            except websocket.WebSocketConnectionClosedException:
                reason = "connection closed by bot (reboot / idle timeout / slot taken)"
                break
            except websocket.WebSocketException as e:
                # Protocol error — e.g. "rsv is not implemented, yet" or
                # "Invalid opcode" from a too-new websocket-client (pin
                # 1.6.4), or a desynced frame on a flaky link. Treat as a
                # drop and handle gracefully instead of dumping a traceback.
                reason = f"protocol error: {e}"
                break
            except OSError as e:
                reason = f"socket error: {e}"
                break
            except Exception as e:  # pragma: no cover - defensive
                reason = f"receive error: {e}"
                break
            if not data:
                continue
            try:
                if isinstance(data, (bytes, bytearray)):
                    self._handle_binary(bytes(data))
                else:
                    self._handle_text(data)
            except Exception:
                log.exception("handling message")
        # Loop ended. If the user didn't call close(), this was an unexpected
        # drop — clean up and (optionally) reconnect in the background.
        if not self._stop.is_set():
            self._handle_unexpected_disconnect(reason or "receive loop ended")

    def _handle_unexpected_disconnect(self, reason: str) -> None:
        """Called from the recv thread when the link drops on its own."""
        # Clear the socket so is_connected() is honest and connect() will work.
        ws, self._ws = self._ws, None
        if ws is not None:
            try: ws.close()
            except Exception: pass
        log.warning("disconnected: %s", reason)
        for cb in list(self._disconnect_cbs):
            try: cb(reason)
            except Exception: log.exception("disconnect cb")
        if self.auto_reconnect and not self._stop.is_set():
            self._start_reconnect()

    def _start_reconnect(self) -> None:
        # One reconnect thread at a time.
        if self._reconnect_thread and self._reconnect_thread.is_alive():
            return
        self._reconnect_thread = threading.Thread(
            target=self._reconnect_loop, name="zrsense-reconnect", daemon=True,
        )
        self._reconnect_thread.start()

    def _reconnect_loop(self) -> None:
        """Retry connect() with capped exponential backoff until it's back."""
        delay = 0.5
        fast_fails = 0
        last_attempt = 0.0
        while not self._stop.is_set() and self._ws is None:
            # Wait out the backoff in short slices so close() stays responsive.
            waited = 0.0
            while waited < delay and not self._stop.is_set():
                time.sleep(0.1)
                waited += 0.1
            if self._stop.is_set():
                return
            now = time.time()
            quick = (now - last_attempt) < 3.0 if last_attempt else False
            last_attempt = now
            try:
                self.connect()
                log.info("reconnected")
                return
            except Exception as e:
                # If connect keeps failing within seconds, it's likely the
                # websocket-client version (frames rejected on handshake) or
                # the bot being down — surface a clear hint, don't spin fast.
                fast_fails = fast_fails + 1 if quick else 0
                if fast_fails == 3:
                    log.warning(
                        "reconnect keeps failing fast (%s). If this is a "
                        "'rsv'/'opcode' error, pin websocket-client==1.6.4.", e,
                    )
                else:
                    log.info("reconnect attempt failed: %s", e)
                delay = min(delay * 2, 10.0)

    def _handle_binary(self, data: bytes) -> None:
        # Optional 8-byte "T-prefix" carries the esp32-tx timestamp so the
        # browser can compute per-frame latency. We just skip it; callers
        # rarely need latency and the SDK doesn't clock-sync anyway.
        jpeg = data[8:] if len(data) > 8 and data[0] == 0x54 else data
        for cb in list(self._video_cbs):
            try: cb(jpeg, 0)
            except Exception: log.exception("video cb")

    def _handle_text(self, data: str) -> None:
        try:
            msg = json.loads(data)
        except json.JSONDecodeError:
            log.debug("non-json text: %r", data[:80])
            return
        if isinstance(msg, dict) and msg.get("type") == "telem":
            payload = msg.get("d") or {}
            self._last_telem = payload
            for cb in list(self._telem_cbs):
                try: cb(payload)
                except Exception: log.exception("telem cb")
            return
        # Non-telem: fan out to callbacks + enqueue for wait_for()
        for cb in list(self._msg_cbs):
            try: cb(msg)
            except Exception: log.exception("msg cb")
        try: self._response_q.put_nowait(msg)
        except queue.Full:
            # Drop oldest to keep the newest — a slow consumer of wait_for()
            # shouldn't cause unbounded growth.
            try: self._response_q.get_nowait()
            except queue.Empty: pass
            try: self._response_q.put_nowait(msg)
            except queue.Full: pass

    # ── callbacks + polling ─────────────────────────────────────────
    def on_telemetry(self, cb: Callable[[Dict[str, Any]], None]) -> None:
        """Register a callback for every {"type":"telem","d":{...}} frame."""
        self._telem_cbs.append(cb)

    def on_video(self, cb: Callable[[bytes, int], None]) -> None:
        """Register a callback for every JPEG frame — cb(jpeg_bytes, latency_ms)."""
        self._video_cbs.append(cb)

    def on_message(self, cb: Callable[[Dict[str, Any]], None]) -> None:
        """Register a callback for every non-telemetry JSON message."""
        self._msg_cbs.append(cb)

    def on_disconnect(self, cb: Callable[[str], None]) -> None:
        """Register a callback fired when the link drops unexpectedly.

        cb receives a short reason string. With auto_reconnect on, a reconnect
        is already in progress by the time this fires — this is just a notice.

            bot.on_disconnect(lambda why: print("lost bot:", why))
        """
        self._disconnect_cbs.append(cb)

    def telemetry(self) -> Dict[str, Any]:
        """Return the most recent telemetry snapshot (empty dict before the first frame)."""
        return dict(self._last_telem)

    def wait(self, seconds: float) -> None:
        """Sleep — convenience so scripts read as `bot.rgb.color(...); bot.wait(1); bot.rgb.off()`."""
        time.sleep(seconds)

    def wait_for(
        self,
        predicate: Callable[[Dict[str, Any]], bool],
        timeout: float = 3.0,
        ask: Optional[Callable[[], None]] = None,
    ) -> Optional[Dict[str, Any]]:
        """Optionally send `ask()`, then block until a message matches `predicate` or timeout.

        Drains any queued messages before calling `ask()` so you always get the
        response to *your* request. Returns the matched message, or None on timeout.

            reply = bot.wait_for(
                lambda m: m.get("device") == "rangefinder",
                ask=bot.tof.get_grid,
                timeout=2.0,
            )
        """
        # Drain stale queued messages so we synchronise on OUR ask.
        while True:
            try: self._response_q.get_nowait()
            except queue.Empty: break
        if ask is not None:
            ask()
        deadline = time.time() + float(timeout)
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                return None
            try:
                msg = self._response_q.get(timeout=min(0.5, remaining))
            except queue.Empty:
                continue
            try:
                if predicate(msg):
                    return msg
            except Exception:
                log.exception("predicate")

    def wait_for_telemetry(
        self,
        field: str,
        timeout: float = 3.0,
    ) -> Optional[Any]:
        """Block until the next telemetry frame carries `field`, return its value.

        The telemetry stream uses SHORT keys (temperature is "t", not "temp").
        This method accepts BOTH the raw key and a friendly alias, so all of
        these work:  bot.wait_for_telemetry("temp")  ==  wait_for_telemetry("t").

        Common fields (alias -> raw key):
            temp/temperature -> t      battery -> b        rssi -> r
            fps -> f                   heap -> h           cpu -> c
            mem/memory -> m            uptime -> u         distance/dist -> d
            heading/yaw -> ih          pitch -> ipt        roll -> irl
            accel_x -> iax  accel_y -> iay  accel_z -> iaz  flipped -> ifl

        Note: IMU fields (ih/ipt/irl/...) only arrive when the MPU6050 is
        enabled, and distance (d) only when a rangefinder is enabled. Call
        bot.telemetry() to see the latest full snapshot of what's arriving.
        Returns None on timeout.
        """
        key = _TELEM_ALIASES.get(field.lower(), field)
        got: List[Any] = []
        evt = threading.Event()
        def _cb(t: Dict[str, Any]) -> None:
            if key in t:
                got.append(t[key])
                evt.set()
            elif field in t:   # also honour the exact name the caller passed
                got.append(t[field])
                evt.set()
        self._telem_cbs.append(_cb)
        try:
            if evt.wait(timeout):
                return got[0]
            # Timed out — help the caller see what keys ARE arriving.
            snap = self._last_telem
            if snap:
                log.info("wait_for_telemetry(%r): no '%s' in telemetry; "
                         "available keys: %s", field, key, sorted(snap.keys()))
            else:
                log.info("wait_for_telemetry(%r): no telemetry received — is "
                         "the bot connected and telemetry subscribed?", field)
            return None
        finally:
            try: self._telem_cbs.remove(_cb)
            except ValueError: pass
