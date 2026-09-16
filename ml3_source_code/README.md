# ELIBot Source Code

This folder contains everything that runs on ELIBot: the STM32 firmware and the ROS2 software stack. See the [project README](../README.md) for the overall hardware/software architecture — this document is about building and running the code.

## Layout

```
ml3_source_code/
├── ML3_Nucleo_Projects/     # STM32 firmware (PlatformIO/mbed) — see its own README
│   ├── combined_node/       # final firmware: 4 motors + 3 servos
│   ├── motors_only/         # debug isolation build: motors only
│   └── servos_only/         # debug isolation build: servos only
├── ml3_ros2_ws_robot/       # ROS2 workspace deployed ON the robot's Raspberry Pi 4
│   └── src/
│       ├── ml3_camera/      # USB camera -> /camera/image_raw
│       ├── ml3_vision/      # QR + traffic-light detection, LED indicator
│       └── ml3_robot_ctrl/  # evdev keyboard teleop -> STM32 serial
└── ml3_ros2_ws_server/      # optional: remote-compute deployment of the vision node (see below)
    └── src/ml3_vision/
```

## Prerequisites

- **Firmware**: [PlatformIO](https://platformio.org/) (CLI or the VS Code extension). No other toolchain needed — `platform = ststm32`/`framework = mbed` pulls in the rest.
- **ROS2**: built and tested against **ROS2 Humble** on the Raspberry Pi (Ubuntu). `colcon` for building.
- **Python packages** (install via `pip3` on top of the ROS2 Python env):
  ```
  pip3 install opencv-python pyserial evdev pyzbar ultralytics gpiozero
  sudo apt-get install libzbar0   # native dependency of pyzbar
  ```
  (`cv_bridge`, `sensor_msgs`, `rclpy` etc. come from the ROS2 Humble install itself.)

## 1. Firmware — build & flash

```
cd ML3_Nucleo_Projects/combined_node
pio run              # build
pio run -t upload    # flash via ST-Link
```

See [`ML3_Nucleo_Projects/README.md`](./ML3_Nucleo_Projects/README.md) for the three firmware variants, and [`ML3_Nucleo_Projects/combined_node/README.md`](./ML3_Nucleo_Projects/combined_node/README.md) for the full serial protocol, pinout, and wiring notes.

## 2. Robot-side ROS2 workspace — build & run

On the Raspberry Pi:

```
cd ml3_ros2_ws_robot
colcon build
source install/setup.bash
```

Four nodes make up the running system (see the [project README](../README.md#software-architecture) for the data flow diagram):

```
# Camera + vision + QR LED indicator, together:
ros2 launch ml3_vision qr_led.launch.py

# Teleop (separate terminal — needs the STM32 connected, e.g. /dev/ttyACM0):
ros2 run ml3_robot_ctrl motorservo_teleop_controller --port /dev/ttyACM0
```

Teleop controls (forwarded raw to the STM32, which owns all interpretation):

```
Motors: w=forward  s=backward  a=strafe/turn-left  d=strafe/turn-right  q=stop
Speed:  0-9 = 10%-100% (default 7 = 80%)
Servos: u/j=servo1±  i/k=servo2±  o/l=servo3±  p/;=servo4±  c=center all
```

## 3. Vision node: onboard vs. remote deployment

`ml3_vision`'s vision-status node (QR + traffic-light detection) can run two ways:

- **Onboard (default, competition configuration)** — `ml3_ros2_ws_robot/src/ml3_vision/ml3_vision/vision_status_node.py` runs directly on the Raspberry Pi, launched via `qr_led.launch.py` above. It's optimized for the Pi's weaker CPU: detection runs on its own thread (decoupled from the ROS executor) and the YOLOv8n model is exported to NCNN for faster inference.
- **Remote/offboard (optional, used during development)** — `ml3_ros2_ws_server/src/ml3_vision/ml3_vision/vision_status.py` is the same node, meant to run on a separate, more powerful machine (e.g. a laptop) instead of the Pi, useful when experimenting with heavier detection models than the Pi can run in real time. It publishes to the exact same topic/schema, so it's a drop-in replacement: run this on the remote machine, and the Pi only needs `ml3_camera` (to publish `/camera/image_raw`) and `ml3_vision`'s `qr_led_indicator.py` (to react to `/vision_status`) running locally. Both machines must be on the **same ROS2 domain** and reachable over the network — set `ROS_DOMAIN_ID` to match on both, and use the dedicated wifi router set up for the competition field (regular venue wifi may block the multicast traffic ROS2 discovery needs).

To build/run the remote variant:
```
cd ml3_ros2_ws_server
colcon build
source install/setup.bash
ros2 run ml3_vision vision_status
```

## Notes

- `yolov8n.pt` (the pretrained YOLOv8n weights) is intentionally not tracked in this repo — `ultralytics` auto-downloads it on first run if missing. It's identical in both ROS2 workspaces.
- Each firmware project's `.pio/` directory and the ROS2 `build`/`install`/`log` directories are build artifacts and are not tracked in git (see [`.gitignore`](../.gitignore)); regenerate them locally with `pio run` / `colcon build`.
