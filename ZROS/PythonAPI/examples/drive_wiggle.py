"""drive_wiggle.py — short scripted drive pattern.

Drives forward, wiggles left/right a couple of times, then reverses out.
Times are conservative — start with the bot on a wide table and adjust
the `ms=` values to your gear ratio.

    python drive_wiggle.py 192.168.1.46
"""
import sys
from zrsense import Robot


def main(host: str) -> None:
    with Robot(host) as bot:
        # ~0.5 s forward at 80% power. `ms` = auto-stop after this many ms
        # (the same 1-second safety watchdog on the firmware side kicks in
        # if we lose network, so this is the graceful version).
        bot.drive.forward(power=80, ms=500)
        bot.wait(0.6)

        for _ in range(3):
            bot.drive.left (power=70, ms=250);  bot.wait(0.3)
            bot.drive.right(power=70, ms=250);  bot.wait(0.3)

        bot.drive.backward(power=60, ms=500);   bot.wait(0.6)
        bot.drive.stop()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("usage: python drive_wiggle.py <bot-ip>")
    main(sys.argv[1])
