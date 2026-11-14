"""scan_geometry.py — the maths behind the ZRCar swept room scan.

Pure Python (no ROS imports) so it can be unit-tested anywhere:

    python3 scan_geometry.py        # runs the self-test at the bottom

The ZRCar's VL53L5CX is a tiny 4x4 / 8x8 multizone ToF "camera" (~45 deg
square field of view) geared 3:1 to the steering servo. Each servo step we read
the WHOLE zone grid, and every zone becomes its own ray: its bearing is the
beam direction (from the servo angle) plus that zone's left/right offset inside
the field of view. Using one "closest anywhere in the middle" distance per step
instead bends flat walls into arcs around the robot — the nearest point of a
wall stays in view for ~20 deg of sweep (tester issue #55).

Frames follow ROS REP-103: x forward, y LEFT, z up; angles are counter-clockwise
(+ = left / up). Grids arrive in the bot's DISPLAY frame (row-major, row 0 = top,
column 0 = left as seen looking out of the sensor — the same orientation as the
/car ToF heatmap).
"""
import math

__all__ = [
    "zone_angles", "grid_side", "rotate_rpy", "horizontal_rows",
    "bin_scan", "OccupancyGrid2D",
]


def grid_side(zone_count):
    """4 for a 16-zone grid, 8 for 64; None for anything else."""
    side = int(round(math.sqrt(max(0, int(zone_count)))))
    return side if side in (4, 8) and side * side == int(zone_count) else None


def zone_angles(side, fov_h_deg=45.0, fov_v_deg=45.0, flip_lr=False):
    """(az, el) in radians for the centre of every zone of a side x side grid,
    indexed like the grid (row-major). +az = left, +el = up. flip_lr mirrors the
    columns, for a sensor whose heatmap comes out left/right swapped."""
    ph = math.radians(fov_h_deg) / side
    pv = math.radians(fov_v_deg) / side
    mid = (side - 1) / 2.0
    out = []
    for i in range(side * side):
        row, col = divmod(i, side)
        az = (mid - col) * ph
        out.append((-az if flip_lr else az, (mid - row) * pv))
    return out


def horizontal_rows(side):
    """Row indices nearest the horizon (the middle two) — the zones that belong
    in a flat 2D scan / floor map."""
    return {side // 2 - 1, side // 2}


def rotate_rpy(x, y, z, roll, pitch, yaw):
    """Rotate a vector by ROS fixed-axis roll/pitch/yaw (radians):
    R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    y1, z1 = cr * y - sr * z, sr * y + cr * z            # roll about x
    x2, z2 = cp * x + sp * z1, -sp * x + cp * z1          # pitch about y
    return cy * x2 - sy * y1, sy * x2 + cy * y1, z2       # yaw about z


def bin_scan(rays, n_bins, range_min, range_max):
    """Turn (bearing_rad, range_m) rays into LaserScan ranges over [-pi, pi).

    Many overlapping sweep steps see the same direction, so each bin keeps the
    MEDIAN of its readings (robust to the odd stray zone). Empty bins are inf.
    Returns (angle_min, angle_increment, ranges)."""
    inc = 2.0 * math.pi / n_bins
    buckets = [[] for _ in range(n_bins)]
    for bearing, r in rays:
        if not (range_min <= r <= range_max):
            continue
        k = int(((bearing + math.pi) % (2.0 * math.pi)) / inc) % n_bins
        buckets[k].append(r)
    ranges = []
    for b in buckets:
        if not b:
            ranges.append(float("inf"))
        else:
            b.sort()
            m = len(b) // 2
            ranges.append(b[m] if len(b) % 2 else 0.5 * (b[m - 1] + b[m]))
    return -math.pi, inc, ranges


class OccupancyGrid2D:
    """A small log-odds floor map centred on the odom origin.

    add_ray() marks the cells a ray passes through as free and its end cell as
    occupied (when it hit something). values() gives ROS OccupancyGrid data:
    -1 unknown, 0 free, 100 occupied, row-major with row 0 at the lowest y."""

    # One pass of a ray marks a cell free; a wall cell is hit by many
    # overlapping sweep steps, so a stray hit fades while real walls stay solid.
    L_HIT, L_MISS, L_CLAMP, L_SHOW = 0.85, -0.60, 5.0, 0.5

    def __init__(self, resolution=0.05, size_m=8.0):
        self.res = float(resolution)
        self.n = max(10, int(round(float(size_m) / self.res)))
        self.origin = -self.n * self.res / 2.0       # metres, same for x and y
        self.clear()

    def clear(self):
        self.lo = [0.0] * (self.n * self.n)

    def _cell(self, x, y):
        return (int(math.floor((x - self.origin) / self.res)),
                int(math.floor((y - self.origin) / self.res)))

    def _bump(self, ix, iy, dl):
        if 0 <= ix < self.n and 0 <= iy < self.n:
            k = iy * self.n + ix
            self.lo[k] = max(-self.L_CLAMP, min(self.L_CLAMP, self.lo[k] + dl))

    def add_ray(self, x0, y0, x1, y1, hit=True):
        (cx, cy), (ex, ey) = self._cell(x0, y0), self._cell(x1, y1)
        dx, dy = abs(ex - cx), -abs(ey - cy)
        sx, sy = (1 if ex > cx else -1), (1 if ey > cy else -1)
        err = dx + dy
        for _ in range(4 * self.n):                  # Bresenham; bounded loop
            if cx == ex and cy == ey:
                break
            self._bump(cx, cy, self.L_MISS)
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                cx += sx
            if e2 <= dx:
                err += dx
                cy += sy
        if hit:
            self._bump(ex, ey, self.L_HIT)

    def values(self):
        out = []
        for v in self.lo:
            out.append(100 if v > self.L_SHOW else (0 if v < -self.L_SHOW else -1))
        return out


# ── self-test: python3 scan_geometry.py ─────────────────────────────────────
if __name__ == "__main__":
    ok = True

    def check(name, cond):
        global ok
        ok = ok and cond
        print(("PASS " if cond else "FAIL ") + name)

    a8 = zone_angles(8)
    check("8x8: column 0 looks left, column 7 right",
          a8[0][0] > 0 > a8[7][0] and abs(a8[0][0] + a8[7][0]) < 1e-9)
    check("8x8: zone pitch 5.625 deg", abs(math.degrees(a8[1][0] - a8[0][0]) + 5.625) < 1e-9)
    check("8x8: row 0 looks up", a8[0][1] > 0 > a8[63][1])
    check("flip_lr mirrors columns", abs(zone_angles(8, flip_lr=True)[0][0] + a8[0][0]) < 1e-12)
    check("grid_side", grid_side(16) == 4 and grid_side(64) == 8 and grid_side(10) is None)
    check("horizontal rows", horizontal_rows(8) == {3, 4} and horizontal_rows(4) == {1, 2})

    x, y, z = rotate_rpy(1.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2)
    check("yaw +90: forward becomes left", abs(x) < 1e-9 and abs(y - 1) < 1e-9 and abs(z) < 1e-9)
    x, y, z = rotate_rpy(1.0, 0.0, 0.0, 0.0, -math.radians(30), 0.0)
    check("ROS pitch -30: nose UP", z > 0.49)

    # A flat wall 0.40 m ahead (x = 0.40) seen by a sweeping 8x8 sensor at the
    # origin: every zone ray should land ON the wall — a straight line, not an
    # arc. Simulate the true radial range of each zone and project it back.
    rays, pts = [], []
    for beam_deg in range(-60, 61, 3):
        beam = math.radians(beam_deg)
        for i, (az, el) in enumerate(a8):
            if i // 8 not in horizontal_rows(8):
                continue
            b = beam + az
            if math.cos(b) <= 0.2:
                continue
            r = 0.40 / (math.cos(b) * math.cos(el))      # radial distance to x = 0.40
            rays.append((b, r * math.cos(el)))
            pts.append((r * math.cos(el) * math.cos(b), r * math.cos(el) * math.sin(b)))
    check("flat wall projects to x = 0.40 everywhere",
          max(abs(px - 0.40) for px, _ in pts) < 1e-9)
    a0, inc, rng = bin_scan(rays, 360, 0.05, 4.0)
    ahead = rng[int((0.0 - a0) / inc)]
    check("scan bin straight ahead ~0.40 m", abs(ahead - 0.40) < 0.01)
    side_bin = rng[int((math.radians(40) - a0) / inc)]
    check("scan bin at 40 deg ~0.40/cos40 (a line, not an arc)",
          abs(side_bin - 0.40 / math.cos(math.radians(40))) < 0.02)

    g = OccupancyGrid2D(resolution=0.05, size_m=4.0)
    g.add_ray(0.0, 0.0, 1.0, 0.0, hit=True)
    v = g.values()
    hx, hy = g._cell(1.0, 0.0)
    mx, my = g._cell(0.5, 0.0)
    check("grid: end cell occupied", v[hy * g.n + hx] == 100)
    check("grid: cells along the ray free", v[my * g.n + mx] == 0)
    check("grid: untouched cell unknown", v[0] == -1)
    print("ALL OK" if ok else "SOME CHECKS FAILED")
    raise SystemExit(0 if ok else 1)
