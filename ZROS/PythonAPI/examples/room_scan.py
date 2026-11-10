"""room_scan.py — 2D room scan with the swept laser rangefinder.

The ZRCar's VL53 laser rangefinder is geared to the STEERING servo (a 36T→12T
gear train), so for every 1° the servo turns, the rangefinder swings 3°. By
sweeping the steering servo across its travel and reading the straight-ahead
distance at each step, we trace a 2D cross-section of the room — a cheap lidar.

    servo 30°..150°  (±60° about centre 90°)
      × 3:1 gear  =  rangefinder ±180°  =  a full 360° room slice.

What it does:
  1. Sweeps the steering servo SERVO_MIN..SERVO_MAX in STEP° steps.
  2. At each step, asks the rangefinder for the centre-of-FoV distance
     (telemetry/key `d`, i.e. the beam pointing straight out of the sensor).
  3. Maps each sample to a polar angle via the gear ratio and plots a polar
     "room map", and writes the raw points to a CSV.

Notes:
  * The MPU6050 is now body-mounted (orientation of the robot, not the beam),
    so we also print the robot's heading — the scan is taken relative to it.
  * Distances are millimetres on the wire; we plot centimetres.
  * Invalid zones read back as -1 (no target / out of 40..4000 mm) and are
    dropped from the plot.
  * The bot must be sitting still for a clean scan (the steering wheels turn
    while it scans — that's expected, the rangefinder is geared to them).

    python room_scan.py <bot-ip>

Requires matplotlib + numpy for the plot (pip install matplotlib numpy). If
matplotlib is missing it still scans and writes the CSV.
"""
import math
import sys
import time

from zrsense import Robot

# ─── Scan geometry (tune to your gear train / servo) ────────────────────────
SERVO_MIN    = 30      # steering servo sweep start (deg)
SERVO_MAX    = 150     # steering servo sweep end   (deg)
SERVO_CENTER = 90      # servo angle at which the rangefinder points forward
GEAR_RATIO   = 3.0     # rangefinder degrees per servo degree (36T→12T = 3:1).
                       # Flip the SIGN to -3.0 if your map comes out mirrored.
STEP         = 1       # servo step in degrees (1° = finest; bigger = faster)
SETTLE       = 0.20    # s to wait after each move so a FRESH ranging frame
                       # lands (sensor ranges at 5 Hz → ~0.2 s per frame)


def run_room_scan(bot):
    """Sweep the servo and collect (beam_angle_rad, distance_cm) samples.

    Returns three parallel lists: angles (radians), distances (cm), and the
    servo angle each came from. Invalid reads are kept as None in distances
    so callers can line angles/servos up; the plotter drops them.
    """
    angles, distances, servos = [], [], []

    # Make sure we start centred, settle, then sweep.
    bot.car.set_steering(SERVO_CENTER)
    bot.wait(0.4)

    lo, hi = min(SERVO_MIN, SERVO_MAX), max(SERVO_MIN, SERVO_MAX)
    sweep = list(range(lo, hi + 1, STEP))
    print(f"Scanning: servo {lo}..{hi}° step {STEP}°  "
          f"({len(sweep)} samples, ~{len(sweep) * (SETTLE + 0.05):.0f}s)…")

    for servo_angle in sweep:
        bot.car.set_steering(servo_angle)
        time.sleep(SETTLE)  # let the servo move AND a fresh frame arrive

        # Request/response: fire getDistance and wait for the reply that
        # carries distanceMm (the centre-2×2 min = straight-ahead beam).
        reply = bot.wait_for(lambda m: "distanceMm" in m,
                             ask=bot.tof.get_distance, timeout=0.5)
        d_mm = reply.get("distanceMm", -1) if reply else -1

        # Gear maps servo angle → beam angle. 0 rad = straight ahead.
        beam_deg = (servo_angle - SERVO_CENTER) * GEAR_RATIO
        angles.append(math.radians(beam_deg))
        servos.append(servo_angle)
        distances.append(None if d_mm is None or d_mm < 0 else d_mm / 10.0)  # → cm

    bot.car.center()  # steering back to straight
    valid = sum(1 for d in distances if d is not None)
    print(f"Scan complete: {valid}/{len(distances)} valid points.")
    return angles, distances, servos


def save_csv(path, angles, distances, servos):
    with open(path, "w") as f:
        f.write("servo_deg,beam_deg,distance_cm\n")
        for a, d, s in zip(angles, distances, servos):
            f.write(f"{s},{math.degrees(a):.1f},{'' if d is None else f'{d:.1f}'}\n")
    print(f"Wrote {path}")


def plot_room(angles, distances):
    try:
        import matplotlib
        import numpy as np
    except Exception:
        print("matplotlib/numpy not installed — skipping plot "
              "(pip install matplotlib numpy). CSV still written.")
        return
    # Use a file backend if there's no display (e.g. headless / WSL).
    import os
    if not os.environ.get("DISPLAY") and sys.platform != "win32":
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    a = [ang for ang, d in zip(angles, distances) if d is not None]
    r = [d for d in distances if d is not None]
    if not r:
        print("No valid points to plot.")
        return

    fig, ax = plt.subplots(subplot_kw={"projection": "polar"})
    ax.set_theta_zero_location("N")   # forward (beam 0°) points up
    ax.set_theta_direction(1)         # +angle = counter-clockwise (= left), ROS-style
    sc = ax.scatter(a, r, s=8, c=r, cmap="viridis")
    ax.set_rmax(max(r) * 1.15)
    ax.set_rticks([50, 100, 150, 200, 300])
    ax.set_rlabel_position(-22.5)
    ax.grid(True)
    fig.colorbar(sc, ax=ax, label="distance (cm)", pad=0.1)
    plt.title("ZRCar room scan", va="bottom")

    out = "room_scan.png"
    plt.savefig(out, dpi=120, bbox_inches="tight")
    print(f"Saved {out}")
    try:
        plt.show()
    except Exception:
        pass


def main(host):
    with Robot(host) as bot:
        bot.wait(0.5)  # SDK rule: settle before the first command
        try:
            heading = bot.wait_for_telemetry("heading", timeout=2.0)
            if heading is not None:
                print(f"Robot heading (MPU6050): {heading:.1f}°  "
                      f"(scan is relative to this)")
        except Exception:
            pass

        angles, distances, servos = run_room_scan(bot)
        save_csv("room_scan.csv", angles, distances, servos)
        plot_room(angles, distances)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("usage: python room_scan.py <bot-ip>")
    main(sys.argv[1])
