"""telemetry_monitor.py — stream one line per telemetry frame to stdout.

Runs until Ctrl-C. Also prints the min value from the ToF grid when the
sensor is enabled — useful as a quick "how close is the wall" check.

    python telemetry_monitor.py 192.168.1.46
"""
import sys
import time
from zrsense import Robot


def main(host: str) -> None:
    def fmt(v, unit="", width=6):
        if v is None: return "  --".rjust(width) + unit
        return f"{v:>{width}}{unit}"

    def on_telem(t):
        dg = t.get("dg") or []
        nearest = min((v for v in dg if v > 0), default=None)
        print(
            f"temp {fmt(t.get('temp'), '°C')}  "
            f"rssi {fmt(t.get('rssi'), 'dBm')}  "
            f"heap {fmt(t.get('heap'), 'K', 5)}  "
            f"cpu {fmt(t.get('cpu'), '%',  4)}  "
            f"bat {fmt(t.get('b'),    '%',  4)}  "
            f"up {fmt(t.get('uptime'), 's', 6)}  "
            f"tof-nearest {fmt(nearest, 'mm', 5)}",
            flush=True,
        )

    with Robot(host) as bot:
        bot.on_telemetry(on_telem)
        print(f"streaming telemetry from {host} — Ctrl-C to stop")
        try:
            while bot.is_connected():
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\nbye")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("usage: python telemetry_monitor.py <bot-ip>")
    main(sys.argv[1])
