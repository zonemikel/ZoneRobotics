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

Requires matplotlib for the plot (pip install matplotlib). If it's missing the
script still scans and writes the CSV.
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


def _nice_rings(maxr):
    """A few evenly-spaced ring radii (m) up to maxr, for scale."""
    step = 0.5 if maxr <= 3 else (1.0 if maxr <= 6 else 2.0)
    rings, r = [], step
    while r <= maxr * 1.05:
        rings.append(round(r, 2))
        r += step
    return rings or [round(maxr, 2)]


def plot_room(angles, distances):
    """Draw a top-down room map (lidar style): robot at the centre, faint scan
    rays, and the wall hits traced into connected segments — broken at corners /
    openings (big range jumps) and gaps (missing beams) so it reads like a floor
    plan instead of scattered dots. Forward (beam 0) points up."""
    try:
        import matplotlib
    except Exception:
        print("matplotlib not installed — skipping plot "
              "(pip install matplotlib). CSV still written.")
        return
    import os
    if not os.environ.get("DISPLAY") and sys.platform != "win32":
        matplotlib.use("Agg")  # headless / WSL: write the PNG, no window
    import matplotlib.pyplot as plt

    # Valid points as (angle_rad, range_m), sorted by angle for wall tracing.
    pts = sorted(((a, d / 100.0) for a, d in zip(angles, distances) if d is not None),
                 key=lambda p: p[0])
    if not pts:
        print("No valid points to plot.")
        return
    valid, total = len(pts), len(distances)

    # Top-down cartesian: forward (beam 0) = +y (up); +angle = left (-x).
    xs = [-r * math.sin(a) for a, r in pts]
    ys = [r * math.cos(a) for a, r in pts]
    maxr = max(r for _, r in pts)

    plt.style.use("dark_background")
    fig, ax = plt.subplots(figsize=(7, 7))
    bg = "#0a0d14"
    fig.patch.set_facecolor(bg)
    ax.set_facecolor(bg)

    # Range rings for scale (metres).
    for rr in _nice_rings(maxr):
        ax.add_patch(plt.Circle((0, 0), rr, fill=False, ec="#243049", lw=0.8, zorder=1))
        ax.text(0.02, rr, f"{rr:g} m", color="#55637d", fontsize=8, ha="left", va="bottom", zorder=2)

    # Scan rays robot -> each hit, faint (the "lidar" look).
    for x, y in zip(xs, ys):
        ax.plot([0, x], [0, y], color="#1b8fff", alpha=0.10, lw=0.6, zorder=2)

    # Trace walls: connect consecutive hits, but BREAK at a big range jump
    # (corner / doorway) or a wide angular gap (missing beams) so we never draw
    # a fake wall straight across open space.
    jump = max(0.35, maxr * 0.18)
    seg_x, seg_y = [xs[0]], [ys[0]]
    for i in range(1, len(pts)):
        dr = abs(pts[i][1] - pts[i - 1][1])
        dth = pts[i][0] - pts[i - 1][0]
        if dr > jump or dth > math.radians(10):
            if len(seg_x) > 1:
                ax.plot(seg_x, seg_y, color="#39d98a", lw=2.2, solid_capstyle="round", zorder=4)
            seg_x, seg_y = [], []
        seg_x.append(xs[i])
        seg_y.append(ys[i])
    if len(seg_x) > 1:
        ax.plot(seg_x, seg_y, color="#39d98a", lw=2.2, solid_capstyle="round", zorder=4)

    # Wall hit points, robot marker, forward arrow.
    ax.scatter(xs, ys, s=10, c="#eaf2ff", zorder=5, linewidths=0)
    ax.scatter([0], [0], s=70, c="#00d9ff", zorder=6)
    ax.annotate("", xy=(0, maxr * 0.16), xytext=(0, 0),
                arrowprops=dict(color="#00d9ff", width=1.4, headwidth=8, headlength=8), zorder=6)

    lim = maxr * 1.15
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_aspect("equal")
    ax.grid(False)
    ax.set_title(f"ZRCar room scan   ·   {valid}/{total} valid beams   ·   forward ↑",
                 color="#eaf2ff", fontsize=12, pad=12)
    ax.set_xlabel("←  left     metres     right  →", color="#8a98b5", fontsize=9)
    for s in ax.spines.values():
        s.set_color("#243049")
    ax.tick_params(colors="#55637d")

    out = "room_scan.png"
    plt.savefig(out, dpi=130, bbox_inches="tight", facecolor=fig.get_facecolor())
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
