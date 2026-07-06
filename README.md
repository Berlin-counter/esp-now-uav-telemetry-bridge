# ESP-NOW UAV Telemetry Bridge

ESP-NOW UAV Telemetry Bridge is a lightweight communication bridge for small UAV ground-station experiments. It links one or more onboard computers to a ground station through ESP32-S3 modules, carrying telemetry, command acknowledgements, and compact point-cloud previews over ESP-NOW.

The project is designed around a simple framed binary protocol and a minimal safety model: the ground station can request a mission-mode transition, but arming remains outside this bridge and should stay under the pilot or flight-controller safety workflow.

## What It Includes

- ESP32-S3 drone-side firmware that forwards USB serial frames over ESP-NOW.
- ESP32-S3 ground-station firmware that receives ESP-NOW telemetry, drives a TJC serial display, and forwards command frames.
- A ROS1/MAVROS Python bridge for publishing odometry, state, battery, EKF, command acknowledgement, and downsampled point-cloud data.
- A small PC-side serial display tool for debugging without the TJC screen.
- A documented binary protocol with CRC8 protection and multi-drone IDs.

## Repository Layout

```text
.
├── bridge/
│   ├── cloud_sender.py      # Trigger point-cloud transmission from the onboard computer
│   └── esp32_bridge.py      # ROS1/MAVROS to ESP32 serial bridge
├── gcs/
│   ├── gcs_display.py       # PC serial monitor and command tool
│   └── ui_mockup.html       # 1024x600 TJC-style UI mockup
├── src/
│   ├── drone_main.cpp       # Drone-side ESP32-S3 firmware
│   └── gcs_main.cpp         # Ground-station ESP32-S3 firmware
├── platformio.ini           # PlatformIO environments for drone and GCS firmware
└── PROTOCOL.md              # Binary frame protocol
```

## Hardware Assumptions

- ESP32-S3 development boards on the drone side and ground-station side.
- Drone-side ESP32 connected to an onboard computer over USB CDC serial.
- Ground-station ESP32 connected to a TJC serial display over UART, with optional USB connection to a PC for debugging.
- Optional ROS1/MAVROS stack on the onboard computer for flight-controller state and odometry.

The firmware uses ESP-NOW broadcast by default. This avoids pairing-specific configuration and makes lab bring-up easier, but it is not encrypted. Add pairing, encryption, or application-level authentication before using this pattern in an untrusted environment.

## Frame Protocol

All payloads use this frame structure:

```text
[0xAA] [DRONE_ID] [TYPE] [LEN_H] [LEN_L] [PAYLOAD...] [CRC8]
```

- `DRONE_ID`: `0x01` and `0x02` are drone IDs, `0x00` is reserved for local ESP32 statistics.
- `TYPE`: telemetry, command, acknowledgement, point-cloud metadata, or point-cloud batch.
- `LEN`: big-endian payload length.
- `CRC8`: polynomial `0x07`, calculated from the header through the payload.

See [PROTOCOL.md](PROTOCOL.md) for the full message table.

## Build Firmware

Install PlatformIO, then build either firmware target:

```bash
pio run -e drone
pio run -e gcs
```

Upload with the serial port for your board:

```bash
pio run -e drone -t upload --upload-port /dev/ttyACM0
pio run -e gcs -t upload --upload-port /dev/ttyACM1
```

The PlatformIO environments enable USB CDC on boot for ESP32-S3 boards.

## Run The ROS Bridge

Install the Python dependencies in your ROS environment:

```bash
python3 -m pip install pyserial numpy
```

Start ROS, MAVROS, and your odometry source, then run:

```bash
python3 bridge/esp32_bridge.py --id 1 --port /dev/ttyACM0 --baud 115200
```

The bridge subscribes to common ROS1/MAVROS topics such as `/Odometry`, `/mavros/state`, `/mavros/battery`, and `/mavlink/from`.

## PC Debug Display

The ground-station ESP32 mirrors received frames to USB. You can inspect them from a PC:

```bash
python3 gcs/gcs_display.py /dev/ttyACM1
```

Keyboard shortcuts:

- `1` / `2`: select active drone.
- `m`: send the start-mission command to the selected drone.
- `q`: quit.

## Point-Cloud Preview

The ROS bridge can project a PCD file into compact 2D batches for display on the ground station:

```bash
python3 bridge/cloud_sender.py
python3 bridge/cloud_sender.py --pcd /path/to/scans.pcd
```

Large point clouds are downsampled and colorized before transmission so they fit within ESP-NOW frame limits.

## Safety Notes

- This code is experimental and intended for controlled lab or simulation use.
- Do not use it as the only safety boundary for a UAV.
- Keep arming, disarming, failsafe, and emergency mode switching under the pilot or flight-controller workflow.
- Validate all commands in simulation before connecting real aircraft hardware.
- ESP-NOW broadcast traffic is not private or authenticated by default.

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE).
