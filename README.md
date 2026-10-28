# Drawbridge camera sensing node

My part of a three-node Raspberry Pi drawbridge, built with two teammates for the Embedded System Software course at Konkuk University (May to July 2026). Pi 1 drove the motor, LEDs and buzzer, Pi 2 ran the MQTT broker and the main control logic, and I built Pi 3: the camera and manual-control node in this repository.

Pi 3 runs an OV5647 camera through a custom V4L2 sensor driver without the Raspberry Pi camera firmware, watches three zones of the bridge for activity, and reports to Pi 2 over MQTT.

Demo video: https://www.youtube.com/watch?v=gtMp4sZuRjk
Korean setup guide: [README.ko.md](README.ko.md)

```
OV5647 ──CSI-2──▶ ov5647_skel (V4L2 subdev) ──▶ /dev/video0
                                                   │ raw Bayer 640×480, MMAP
                                                   ▼
                                    bridge_view: per-zone change detection
                                                   │ MQTT, QoS 1, every second
                                                   ▼
                                     Pi 2 broker ──▶ main control ──▶ Pi 1 actuators
```

Hardware and OS: Raspberry Pi 3B, Raspbian Buster, Linux 4.19.97-v7+, OV5647 on 2 CSI-2 data lanes, I²C address 0x36.

## What's here

- `driver/`: the OV5647 sensor driver (`ov5647_skel.c`, `ov5647_hw.h`, `ov5647_ctrls.h`) and the manual switch driver (`manual_switch.c`)
- `overlay/`: Device Tree overlay for the camera clock, power-down GPIO and CSI-2 link
- `detector/`: `bridge_view` (capture, detection, HTTP preview, MQTT), a headless `bridge_detect`, and `sw_bridge`, which forwards switch events to MQTT
- `systemd/`: units that load the drivers and start the programs in order at boot

## Camera driver

I started from the OV5647 driver in the Raspberry Pi `rpi-4.19.y` kernel (`drivers/media/i2c/ov5647.c`, GPL-2.0). Its register init table and stream on/off sequences are used as they are. My changes are what it took to run the sensor on a Pi 3B with `start_x=0`, where the firmware no longer sets up the camera:

- The stock overlay only declares a `fixed-clock` node and leaves the real clock to the firmware. My overlay wires the sensor to the BCM2835 CAM0 clock (`<&clocks 45>`) at 25 MHz, and the driver requests 25 MHz again at power-on.
- I removed `clock-noncontinuous` from the overlay. Non-continuous clocking relies on calibration done by the firmware.
- Power-down is mapped to the Pi 3B's GPIO expander pin (`<&expgpio 5 1>`).
- Without firmware the sensor has no clock until the driver turns one on, and it will not answer on I²C before that. The probe therefore powers the sensor up completely before reading the chip ID (0x5647).
- I added exposure, analogue gain, flip and test-pattern controls. The 4.19 stock driver has none.
- TRY and ACTIVE formats are stored separately, and every path reports the 640×480 SBGGR8 mode the sensor really outputs.
- I²C errors are printed with the register address and value. That made wiring faults easy to tell apart from driver bugs.

The driver binds to `compatible = "myorg,ov5647-skel"` so it can be loaded next to the in-tree driver.

## The rolling-frame bug

With my driver, frames came out repeated two or three times across the image and drifting sideways. The stock driver with firmware gave a clean picture.

Tuning PLL (0x3037), VTS and the MIPI line-sync registers didn't help, and a few of those changes gave a black frame, so I reverted them. I then ran the stock driver as a reference and read back the sensor registers from both. They matched, which ruled out the init tables and left I²C, the input clock and the CSI-2 link to check one by one.

There were two causes. With the sensor clock taken from CAM1, the CSI-2 receiver raised CAM1 to about 100 MHz when streaming started, so the sensor ran roughly four times too fast; moving it to CAM0 at 25 MHz fixed that. Non-continuous CSI-2 clocking also needs the firmware's D-PHY calibration, and without it the receiver lost frame sync, so I switched to continuous clocking.

Single-frame capture works after both changes. Some drift still shows up now and then during continuous streaming. I traced it to the Buster-era unicam receiver driver and left it as a known limitation.

## Detection

`bridge_view` captures raw Bayer frames over V4L2 MMAP with 4 buffers and averages each 2×2 Bayer block into a half-size grayscale image. The image is split into three vertical zones, one per lane of the bridge. Each zone keeps a moving-average background, and a zone counts as active when enough pixels differ from that background or from the previous frame. After a short warm-up, the node publishes a three-character status every second (`110` means activity in the left and middle zones).

Pixel-change detection was enough here: the controller only needs to know whether each lane is clear, and a Pi 3 has little CPU to spare for object detection. Light reflections caused false detections at first, and moving the camera and changing the lighting fixed that.

`bridge_view` also serves `/snapshot` and `/stream` on port 8080 for checking the camera live. The JPEG encoder is a small one bundled in `tiny_jpeg.h`, so there is no extra library to install.

## Manual switch driver

Manual control wasn't my job on the team. I wrote `manual_switch` on my own while preparing, to learn the interrupt path from GPIO to user space. Both switch lines (BCM 12 for up, BCM 5 for down) raise interrupts, with a 30 ms debounce and a `ready` flag that ignores edges while the module loads or unloads. Events go into a `kfifo`, `read()` blocks on a wait queue, and `poll()` works too. `sw_bridge` watches `/dev/manual_sw` and publishes `MANUAL_UP` or `MANUAL_DOWN`.

When the switches did nothing, I wrote a GPIO scan script to see whether the problem was code or wiring. It was wiring: the default pull resistors, a 4-pin tact switch connected the wrong way round, and BCM pin numbers mixed up with physical ones.

## MQTT messages

| Topic | Payload | Meaning |
|---|---|---|
| `drawbridge/pi3/event` | `000` to `111` | activity in the left, middle and right zones, sent every second |
| `drawbridge/pi3/event` | `MANUAL_UP`, `MANUAL_DOWN` | manual switch pressed |

Messages use QoS 1. If the broker can't be reached, the detector prints the status locally instead.

## Build and run

On the Pi, with the 4.19 kernel headers installed:

```bash
dtc -@ -I dts -O dtb -o /tmp/ov5647-skel-clk.dtbo overlay/ov5647-skel-clk-overlay.dts
sudo cp /tmp/ov5647-skel-clk.dtbo /boot/overlays/

# add to /boot/config.txt:
#   start_x=0
#   dtoverlay=ov5647-skel-clk
#   gpio=5,12=ip,pd

make -C driver
make -C detector MQTT=1          # needs libmosquitto-dev

sudo cp systemd/*.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now ov5647-skel bridge-view manual-switch sw-bridge
```

The broker address, topics, zone width and thresholds are in `detector/config.h`.

## Known limitations

- Occasional drift during continuous streaming with the Buster unicam driver.
- Control values set while the sensor is powered off are not written to it at the next power-on.
- Only the 640×480 SBGGR8 mode is supported.

## License

The camera driver files are derived from the Linux OV5647 driver (Copyright (C) 2016, Synopsys, Inc.; author Ramiro Oliveira) and are GPL-2.0 like the original. Each of them has an SPDX header.
