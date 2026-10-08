"""blink_rgb.py — the smallest possible ZRSense SDK example.

Cycles the on-board WS2812 strip through red/green/blue three times,
then turns it off. Pass the bot's IP as the only argument.

    python blink_rgb.py 192.168.1.46
"""
import sys
from zrsense import Robot


def main(host: str) -> None:
    with Robot(host) as bot:
        for _ in range(3):
            for r, g, b in [(255, 0, 0), (0, 255, 0), (0, 0, 255)]:
                bot.rgb.color(r, g, b)
                bot.wait(0.4)
        bot.rgb.off()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("usage: python blink_rgb.py <bot-ip>")
    main(sys.argv[1])
