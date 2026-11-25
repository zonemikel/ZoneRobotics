# ZoneRobotics firmware — what's new

Changes in each firmware release, newest first. Your bot's own page shows the
ones that are newer than the firmware it runs. To update a bot: open its page
on your Wi-Fi → **🔄 Update** → **Update from Server**.

<!-- The bot page reads this file. Keep the format: one heading per release,
     "## <version> — <YYYY-MM-DD>", where <version> is the release file's
     version without the date (ZoneRobotics_v1.0.7.868c49c-20261125.bin →
     1.0.7.868c49c) and the date is the one in the file name; then "- " bullet
     points in plain language. Newest first. -->

## 1.0.7.d890e96 — 2026-11-25
- After an update, the bot's page tells you when the bot is back and which version it runs, instead of "Device Restarting..." forever.
- The new-firmware notice shows what's new in the update.
- If an update fails, the page says so instead of showing "Downloading..." forever.

## 1.0.7.868c49c — 2026-11-25
- Your bot's page now tells you when new firmware is out: a banner at the top and a dot on the 🔄 Update tab.

## 1.0.7.94a0993 — 2026-11-24
- Cloud video connects much more reliably, usually on the first try — also for the claw machine.
- Fixed cloud video losing data about every 15 minutes.
- Fixed a memory leak that could make the bot restart after about 10 cloud sessions.

## 1.0.7.a3a7605 — 2026-11-22
- /car: the camera tilt servo has its own pin (Settings → Tilt servo, GPIO 4 by default); the Servo tab no longer moves it.

## 1.0.7.bbae405 — 2026-11-21
- /cam: a Home button walks pan and tilt smoothly back to the middle.
- /cam: holding the focus buttons on a phone no longer opens the "Download image" menu.

## 1.0.7.cd3a8ea — 2026-11-21
- On a slow connection, cloud video keeps the size you picked and compresses harder instead of shrinking the picture.
- New default video size: CIF (400×296).
- Heat protection: from 85 °C the video frame rate is lowered until the chip cools down.
- Wi-Fi transmit power follows the signal strength (less heat when the router is close).

## 1.0.7.b440414 — 2026-11-21
- Long-distance cloud sessions no longer drop after 1–2 minutes.
- Safety: buttons that hold a motor or output (like the claw) release it by themselves if the connection drops.
