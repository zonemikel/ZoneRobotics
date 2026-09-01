# Zone Robotics

Official Zone Robotics repository — the public home for firmware, media, datasheets and product files.

Zone Robotics builds a modular ESP32-S3 robotics platform: hardware, camera streaming, sensing and browser-based control. This repo is where those product files live.

- Website: <https://home.zonerobotics.com>
- Community forum: <https://discourse.zonerobotics.com>
- Discord: <https://discord.gg/cRujv5P52m>
- Contact: <admin@zonerobotics.com>

## What's in here

| Folder | Contents |
| --- | --- |
| [`ZROS/`](ZROS/) | **Free** ESP32-S3 firmware — the local Zone Robotics OS. Download and flash onto your own ESP32-S3 Sense. |
| [`ZRSense/`](ZRSense/) | ZRSense board — ESP32-S3 sensing board. Datasheets, pinouts, mechanical drawings, media. |
| [`ZRCore/`](ZRCore/) | ZRCore board — motion + power core. Datasheets, pinouts, mechanical drawings, media. |
| [`ZROneMicro/`](ZROneMicro/) | ZROne Micro — compact ZRSense-only robot. Assembly notes and media. |
| [`ZROne/`](ZROne/) | ZROne — full modular tracked platform. Media, docs, in-development notes. |
| [`media/`](media/) | Shared brand and product visuals — logos, renders, infographics. |

## ZROS — free firmware

The core reason this repo is public: **ZROS is free**. Anyone with an ESP32-S3 Sense board can flash it and have a working Zone Robotics robot with camera streaming, motor + servo control, RGB status lighting and a browser-based programmer.

- Firmware files: [`ZROS/`](ZROS/)
- Flashing guide: <https://discourse.zonerobotics.com/t/how-to-flash-esp32s3-sense-with-xiao-flash-download-tool/26>
- Product overview: <https://home.zonerobotics.com/products/zros>

## Products

The full product lineup — hardware to buy or flash yourself.

| # | Product | Status |
| --- | --- | --- |
| 00 | [ZROS](ZROS/) — free firmware | Available now |
| 01 | [ZRSense](ZRSense/) — sensing board | Selling on Tindie soon |
| 02 | [ZRCore](ZRCore/) — motion + power board | Available for larger builds |
| 03 | [ZROne Micro](ZROneMicro/) — compact robot | Selling first |
| 04 | [ZROne](ZROne/) — full modular tank | In development |
| 05 | ZR Cloud — browser control | Live at <https://home.zonerobotics.com/dashboard> |

## Issues and questions

- Bugs, firmware issues, feature requests: open an [issue](https://github.com/zonemikel/ZoneRobotics/issues) here.
- Community discussion, project builds, general questions: the [Discourse forum](https://discourse.zonerobotics.com) is the best place.
- Real-time chat: [Discord](https://discord.gg/cRujv5P52m).

## License

All brand and product assets remain the property of Zone Robotics. Firmware licensing is documented per-directory in the individual `ZROS/` files.
