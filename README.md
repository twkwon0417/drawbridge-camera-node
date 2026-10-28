# Drawbridge Camera Sensing Node

Camera sensing node for a three-node Raspberry Pi drawbridge control system, built for the Embedded System Software course at Konkuk University (May – Jul 2026, team of 3).

This repository contains **my part** of the system: the sensing node (Pi 3). It runs an OV5647 camera through a custom V4L2 sensor driver without the Raspberry Pi camera firmware, detects activity in three zones of the bridge, and publishes the result over MQTT to the main controller.

Demo video: https://www.youtube.com/watch?v=gtMp4sZuRjk

[한국어 설치 가이드 (Korean setup guide)](README.ko.md)

## System overview

| Node | Role | Owner |
|---|---|---|
| Pi 1 | Drives the bridge motor, LEDs, and buzzer | Teammate |
| Pi 2 | MQTT broker and main control logic | Teammate |
| **Pi 3** | **Camera sensing and manual control input (this repository)** | **Me** |

```
OV5647 ──CSI-2──▶ ov5647_skel (V4L2 subdev) ──▶ /dev/video0
                                                   │ V4L2 MMAP, raw Bayer 640×480
                                                   ▼
                                    bridge_view: 3-zone change/occupancy detection
                                                   │ MQTT, QoS 1, every 1 s
                                                   ▼
                                     Pi 2 broker ──▶ main control ──▶ Pi 1 actuators
```

Environment: Raspberry Pi 3B, Raspbian Buster, Linux 4.19.97-v7+, OV5647 (MIPI CSI-2, 2 data lanes, I²C address 0x36).

## Repository layout

```
driver/     ov5647_skel.c      OV5647 V4L2 sub-device driver
            ov5647_hw.h        register tables, stream on/off, sensor init
            ov5647_ctrls.h     exposure, gain, flip, test-pattern controls
            manual_switch.c    GPIO interrupt driver for two manual switches
overlay/    ov5647-skel-clk-overlay.dts   Device Tree overlay (CAM0 clock, PWDN, CSI-2)
detector/   bridge_view.c      capture + detection + HTTP preview + MQTT (main service)
            bridge_detect.c    headless version of the detector
            sw_bridge.c        forwards switch events from /dev/manual_sw to MQTT
            tiny_jpeg.h        minimal baseline JPEG encoder for the HTTP preview
            mqtt_pub.h         thin libmosquitto wrapper with log fallback
systemd/    four units that load the drivers and start the services in order
```

## Camera driver

`ov5647_skel` is a port of the Raspberry Pi `rpi-4.19.y` OV5647 driver (`drivers/media/i2c/ov5647.c`, GPL-2.0). The register initialization table and the stream on/off sequences come from that driver unchanged. What I changed is what it takes to run the sensor on this board **without the camera firmware** (`start_x=0`):

- **Real sensor clock.** The stock overlay declares a `fixed-clock` node and relies on the firmware to drive the sensor clock. My overlay connects the BCM2835 CAM0 clock (`<&clocks 45>`) and assigns it 25 MHz, and the driver requests 25 MHz again at power-on.
- **Continuous CSI-2 clock.** The stock overlay sets `clock-noncontinuous`, which depends on calibration done by the firmware. My overlay removes it, so the driver keeps the clock lane running.
- **Power-down GPIO for the Pi 3B.** The PWDN line is mapped to the GPIO expander pin (`<&expgpio 5 1>`, CAM_GPIO0).
- **Probe order.** The stock driver reads the chip ID with only the PWDN line released, since the firmware clock is already running. Without firmware the sensor does not answer on I²C until its clock is on, so this driver powers the sensor fully (clock, output enable, init) before checking the chip ID (0x5647).
- **V4L2 controls.** Added exposure, analogue gain, horizontal/vertical flip, and test-pattern controls, which the 4.19 stock driver does not have.
- **Format handling.** TRY and ACTIVE formats are stored separately, and every path reports the 640×480 SBGGR8 mode the sensor actually outputs.
- **Error reporting.** I²C failures are logged with the register address and value instead of at debug level, which helped separate wiring faults from driver bugs.

The driver binds to `compatible = "myorg,ov5647-skel"`, so it loads alongside the in-tree `ovti,ov5647` driver without a conflict.

## Debugging the rolling frames

With the custom driver, captured frames came back repeated two or three times across the image and drifting sideways. The stock driver with the firmware produced a clean image.

1. Tuning PLL (0x3037), VTS, and MIPI line-sync registers did not fix it, and some changes produced a black frame, so I reverted them.
2. I used the stock driver as a known-good baseline. Register readback showed both drivers wrote the same sensor registers, which ruled out the register tables.
3. I then checked the remaining layers one at a time: I²C, the input clock, and the CSI-2 link.

Two causes came out of this:

- **Clock.** With the sensor clock taken from CAM1, the CSI-2 receiver raised CAM1 to about 100 MHz when streaming started, so the sensor ran about four times too fast. Moving the sensor to the CAM0 clock at 25 MHz fixed the timing.
- **CSI-2 clock mode.** Non-continuous clocking needs the D-PHY calibration that the firmware performs. Without the firmware the receiver lost frame sync. Switching to continuous clocking fixed it.

Result: single-frame capture works. Occasional residual drift remains, which I traced to the Buster-era unicam receiver driver and left documented as a known limitation.

## Detection

`bridge_view` captures raw Bayer frames through V4L2 MMAP (4 buffers) and works on a half-resolution luminance image built from 2×2 Bayer blocks.

- The image is split into three vertical zones that match the lanes of the bridge.
- Each zone keeps an exponential moving-average background. A zone is marked active when the share of pixels that differ from the background (occupancy) or from the previous frame (motion) passes a threshold.
- A short warm-up period lets the background settle after start-up.
- Every second the node publishes a three-character status, e.g. `110` means activity in the left and middle zones.

I chose pixel-change detection over an object-detection model because the Pi 3 CPU budget is small and the controller only needs to know whether each zone is clear. When light reflections caused false detections, I fixed the camera placement and lighting rather than making the algorithm more complex.

`bridge_view` also serves `/snapshot` and `/stream` over HTTP on port 8080 for live checking, using the bundled JPEG encoder so there is no extra library dependency.

## Manual switch driver

Manual control was not my assigned role on the team. While preparing the project I built `manual_switch` on my own to learn the interrupt path end to end:

- GPIO interrupts on both switch lines (BCM 12 = up, BCM 5 = down)
- 30 ms debounce in the interrupt handler, plus a `ready` flag that drops spurious edges while the module is loading or unloading
- events queued in a `kfifo`; `read()` blocks on a wait queue and `poll()` is supported
- `sw_bridge` polls `/dev/manual_sw` and publishes `MANUAL_UP` / `MANUAL_DOWN`

When the switches did not respond, I wrote a GPIO scan script to tell code bugs from wiring faults. The causes were the default pull resistors, a miswired 4-pin tact switch, and BCM versus physical pin numbering.

## MQTT messages

| Topic | Payload | Meaning |
|---|---|---|
| `drawbridge/pi3/event` | `000` – `111` | Activity in the left / middle / right zone, sent every second |
| `drawbridge/pi3/event` | `MANUAL_UP`, `MANUAL_DOWN` | Manual switch pressed |

Messages are sent with QoS 1. If the broker is unreachable, the detector logs the status locally instead.

## Build and run

On the Raspberry Pi (kernel headers for 4.19 installed):

```bash
# Device Tree overlay
dtc -@ -I dts -O dtb -o /tmp/ov5647-skel-clk.dtbo overlay/ov5647-skel-clk-overlay.dts
sudo cp /tmp/ov5647-skel-clk.dtbo /boot/overlays/

# /boot/config.txt
#   start_x=0
#   dtoverlay=ov5647-skel-clk
#   gpio=5,12=ip,pd

# Kernel modules and user-space programs (libmosquitto-dev needed for MQTT)
make -C driver
make -C detector MQTT=1

# Start everything at boot
sudo cp systemd/*.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now ov5647-skel bridge-view manual-switch sw-bridge
```

The broker address, topics, zone width, and thresholds are set in `detector/config.h`. The systemd units load the camera driver first, wait for `/dev/video0`, and restart the services if they exit.

## Known limitations

- Occasional residual frame drift remains with the Buster unicam receiver driver.
- Control values set while the sensor is powered off are stored but not written to the sensor at the next power-on.
- Only the 640×480 SBGGR8 mode is supported.

## License

The camera driver (`driver/ov5647_skel.c`, `driver/ov5647_hw.h`, `driver/ov5647_ctrls.h`) is derived from the Linux OV5647 driver (Copyright (C) 2016, Synopsys, Inc.; author Ramiro Oliveira) and is licensed under GPL-2.0, like the original. Each driver file carries an SPDX license header.
