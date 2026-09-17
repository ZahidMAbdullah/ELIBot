# ELIBot — Mechatronics Laboratory (Robot: Advanced)

**ELIBot** is a four-wheeled, all-terrain robot with spiked/clawed wheels, developed by a team of five undergraduate students for the *Mechatronics Laboratory (Robot: Advanced)* course at Kyoto University of Advanced Science (KUAS). Designed to navigate a challenging obstacle course—including a door, uneven terrain, slope, ditch, QR-code checkpoint, ledge, roller obstacle, speed bump, stairs, a traffic-light-controlled barrier, and an object-retrieval task—the robot combines a raised, high-clearance chassis, custom rigid/flexible claw wheels, and a lightweight single-DOF manipulator arm. It's primarily teleoperated from a remote operator station, watching fixed cameras placed along the course (plus an optional onboard camera feed), with support for minimal autonomous behaviors (e.g. reacting to the traffic light on its own) layered on top.

> This repository documents the full design process — from the original tri-wheel concept to the final ELIBot — along with its firmware, ROS2 software stack, CAD/print files, and build media.

## Table of Contents

- [Team](#team)
- [What It Was Built For](#what-it-was-built-for)
- [Design Journey](#design-journey)
- [Final Design](#final-design)
  - [Mechanical](#mechanical)
  - [Manipulator Arm](#manipulator-arm)
  - [Electronics](#electronics)
  - [Software Architecture](#software-architecture)
- [Operating Instructions](#operating-instructions)
- [Repository Structure](#repository-structure)
- [Documents](#documents)
- [Media](#media)
- [Acknowledgments & References](#acknowledgments--references)

## Team

**Team 12**, Mechatronics Laboratory (Robot: Advanced), Spring 2026:

- Abdullah Al Ayaat
- G. C. Mohnish
- Catherine Ella Burtenshaw
- Daphne Jillian Gan Tan
- Muhammad Abdullah Zahid

## What It Was Built For

Mechatronics Laboratory (Robot: Advanced) is a hands-on robotics course, run over one semester, where small teams design, fabricate, and program a robot from scratch. It culminates in a robot challenge held during the course's 15th week: a field of physical obstacles that every team's robot has to get through under its own power. ELIBot is this team's entry for that challenge.

The competition field laid out a mock "street" of obstacles a mobile robot has to get through under its own locomotion and, in places, its own manipulation — pushing/opening a door, climbing a slope, crossing uneven terrain and a ditch, reading a QR checkpoint, climbing down a ledge, crossing a speed bump and a roller, climbing up and down the stairs, responding correctly to a traffic light to get a barrier opened, and picking up and carrying an object to a goal. ELIBot's design — the raised chassis and clawed wheels in particular — was driven directly by that obstacle set.

The robot had to be untethered and was operated from a fixed station with no direct line of sight to the field — the operator watched a multi-window monitor fed by fixed cameras placed along the course, optionally supplemented by the robot's own onboard camera feed on a separate laptop — with a lightweight autonomous layer handling a few specific perception-driven behaviors (QR/traffic-light detection) on top of manual driving.

## Design Journey

ELIBot went through three distinct mechanical designs:

1. **Tri-wheel concept** (original pitch): a spring-suspended front "wheel-leg" (inspired by configurable wheel-leg mechanisms in the literature) for climbing stairs/ditches, a rear pair of wheels on a 4-bar spring suspension for uneven terrain, and a top-mounted 3-DOF manipulator arm with a solenoid-driven pincher gripper.
2. **Four-wheel, front-spiked only**: the tri-wheel base proved **unstable**, so the team pivoted to a four-wheeled layout — but only the front pair used spiked/clawed wheels, with plain wheels still at the rear.
3. **Four-wheel, all-spiked** (final): the plain rear wheels in design 2 didn't grip well enough to climb the stairs, so the team switched to spiked/clawed wheels on all four corners. The manipulator arm was also simplified from 3-DOF + pincher down to a lighter 1-DOF hook mechanism, for better weight distribution and stair-climbing stability.

Designs 1 and 2 are documented here only through their presentations — [`Robot Concept Presentation/`](./Robot%20Concept%20Presentation) for design 1 (the tri-wheel concept), and [`Intermediate Presentation/`](./Intermediate%20Presentation) for the transition from design 1 to design 2 — rather than through their raw CAD/design files. Design 3, the final version that was actually built, is described below, in the [final report](./Teams12_final_report.pdf), and via the source/print files elsewhere in this repository.

## Final Design

### Mechanical

ELIBot's body has three main parts:

- **Chassis** — a simple removable-lid box housing the microcontroller, motor drivers, buck converter, and perfboards; sized to be sturdy against fall/testing impacts while giving easy access to the electronics.
- **Brackets** — front (shorter) and rear (taller) leg-brackets holding the drive motors, sized asymmetrically to keep the robot level, shrink its turning footprint, and shift the center of mass rearward for stair climbing without tipping.
- **Cage** — a lightweight top frame protecting the 12V LiPo battery, a power bank, and the Raspberry Pi 4, and keeping wiring organized.

The signature feature is the **clawed/spiked wheel**: a rigid **PLA** inner hub (bolted to the motor shaft via a threaded brass connector) paired with a flexible **TPU** outer ring of 8 claws. The rigid hub keeps the wheel stable while the TPU claws flex on impact and grip stair edges/ledges — giving some of the benefit of a reconfigurable wheel-leg design without its mechanical complexity.

### Manipulator Arm

The arm went through the same simplification arc as the chassis: an initial 3-DOF, top-mounted arm with a solenoid-driven pincher was replaced by a **1-DOF, front-mounted "elephant trunk"** arm ending in a curved hook — lighter, mechanically simpler, and strong enough to support the robot's own weight if it tips, unlike the original pincher which broke under load during testing. The final assembly uses three servos: one for base rotation, one for the second link, one for the hook.

### Electronics

| Component | Role |
|---|---|
| STM32 Nucleo-F091RC | Real-time motor/servo control (low-level) |
| Raspberry Pi 4 Model B | ROS2 high-level control, perception, decision-making |
| 2× L298N | Motor drivers for 4 brushed DC drive motors (skid-steering) |
| PCA9685 (16-ch) | PWM driver for 3 manipulator servos, over I²C |
| Buck converter | Steps down battery voltage for logic circuitry |
| 12V LiPo + power bank | Main drive power / Raspberry Pi power |
| USB camera | Onboard vision (QR codes, traffic light) |

The Pi and STM32 communicate over USB serial at 115200 baud: the Pi sends motion/servo setpoints, the STM32 executes deterministic PWM control and echoes actuator feedback. A watchdog on the STM32 stops all motors if commands stop arriving, preventing runaways from a dropped link.

### Software Architecture

High-level software runs as a modular **ROS2** stack on the Raspberry Pi:

```
Operator (keyboard) ─┐
                      ▼
              Teleoperation Node ── USB Serial (115200) ──▶ STM32 (motors + servos)
                      │
USB Camera ──▶ Camera Publisher Node ──/camera/image_raw──▶ Vision Status Node
                                                                  │  QR (Pyzbar) + traffic light (YOLOv8 + HSV)
                                                                  ▼
                                                         /vision_status ──▶ LED Indicator Node (GPIO)
                                                                       └──▶ Traffic-light ROS2 notification (wireless, for the barrier)
```

- **Camera publisher** — captures RGB frames via OpenCV, keeps only the latest frame to avoid perception lag.
- **Vision status node** — runs QR detection (grayscale + Pyzbar) and traffic-light detection (YOLOv8n localizes the light, HSV classifies red/yellow/green) in parallel; debounces both before publishing state changes.
- **LED indicator node** — drives GPIO LEDs based on QR detection state.
- **Teleoperation node** — converts operator keyboard input into the STM32's serial command protocol.

Firmware protocol (STM32 side): 8 slash-separated values — 4 motor duty cycles `[-1.0, 1.0]` and 4 servo angles (degrees) — terminated by `d`, e.g. `0.5/0.5/0/0/90/0/0/-45d`. See [`source-code/Nucleo_Firmware/combined_node/README.md`](./source-code/Nucleo_Firmware/combined_node/README.md) for the full protocol and pin mapping.

## Operating Instructions

1. Connect the STM32 to the Raspberry Pi 4; power the Pi from the power bank.
2. (Setup/debug) Connect via HDMI to a monitor, or view the Pi's desktop remotely, to launch ROS2 nodes from a terminal.
3. Launch, in order:
   1. `ros2 launch elibot_vision qr_led.launch.py` → starts the camera publisher, vision status node, and QR LED indicator together
   2. `ros2 run elibot_robot_ctrl motorservo_teleop_controller --port /dev/ttyACM0` → keyboard teleop, sends serial commands to the STM32
4. Disconnect the setup peripherals, connect the drive battery, and verify wheel + arm motion before running the course.

Full build/run instructions and dependencies: [`source-code/README.md`](./source-code/README.md). Full narrative details: [final report, §3](./Teams12_final_report.pdf).

The vision node can also run off-robot on a separate, more powerful machine over the network instead of on the Pi — see [`source-code/README.md`](./source-code/README.md#3-vision-node-onboard-vs-remote-deployment) for that configuration (`elibot_ros2_ws_server/`), which was used during development to experiment with heavier detection models than the Pi could run in real time.

## Repository Structure

```
.
├── README.md
├── Teams12_final_report.pdf
├── Robot Challenge and Field Details 2025.pdf
├── Robot Concept Presentation/       # superseded tri-wheel concept pitch — not the final design
├── Intermediate Presentation/        # mid-project checkpoint — shows the tri-wheel -> four-wheel transition
├── STL Files/                        # final ELIBot print files (STL/gcode)
├── source-code/                      # firmware + ROS2 software — see its own README
│   ├── Nucleo_Firmware/              # STM32 firmware (PlatformIO/mbed): combined_node (final), motors_only/servos_only (debug isolation builds)
│   ├── elibot_ros2_ws_robot/         # ROS2 workspace deployed ON the Raspberry Pi (camera, vision, teleop)
│   └── elibot_ros2_ws_server/        # optional: vision node deployable on a separate, more powerful machine instead of the Pi
└── Media/                            # photos and video (large raw footage hosted externally)
```

## Documents

- [Final report](./Teams12_final_report.pdf) — full design writeup (this README summarizes it)
- [Challenge & field rules](./Robot%20Challenge%20and%20Field%20Details%202025.pdf)
- [Concept presentation](./Robot%20Concept%20Presentation/Concept%20Presentation%20%28Superseded%20Tri-Wheel%20Design%29.pdf) — the original tri-wheel pitch; **superseded**, describes a different robot than the one that was actually built
- [Intermediate presentation](./Intermediate%20Presentation/Intermediate%20Presentation%20%28Design%20Transition%20Checkpoint%29.pdf) — a mid-project checkpoint documenting the pivot from the tri-wheel design to the second, four-wheel design (front wheels spiked, rear wheels still plain); that rear-wheel choice and the manipulator's pincher end effector shown here were both changed again before the final design
- [Final presentation video (Week 15)](https://drive.google.com/file/d/1nr6MhxmOqBarv2mEdx8B2zVPzM3RgYNS/view?usp=drive_link) — the team's video presentation of the finished robot, made right before the competition run (hosted on Google Drive; not tracked in this repo due to file size)

## Media

### Videos

- [Stage II official run (Week 15)](https://drive.google.com/file/d/196SE-jIKrzIXMWrg3RmbG3_Q3yIPCxZ9/view?usp=sharing) — full video of ELIBot's official run on the challenge field for Stage II of the competition (hosted on Google Drive; not tracked in this repo due to file size)
- [Full event livestream (Week 15)](https://www.youtube.com/live/wm4ukXukuMc) — the university's official broadcast of the entire robot challenge event, covering all teams, not just ELIBot's run. This team's segments:
  - [1:34:22](https://www.youtube.com/live/wm4ukXukuMc?t=5662s) – 1:37:34 — team introduction and the idea behind the robot's design
  - 1:37:34 – 1:43:20 — Stage I run
  - [3:35:19](https://www.youtube.com/live/wm4ukXukuMc?t=12919s) – 3:43:02 — Stage II run
  - 3:43:02 – 3:45:35 — post-run team interview

### Photos

Final prototype, as built (see [`Media/Pictures/Robot Pictures/`](<./Media/Pictures/Robot Pictures>) for the originals):

<p align="center">
<img src="./Media/Pictures/Robot%20Pictures/Final%20Prototype%20%28Photo%201%29.jpg" width="240" />
<img src="./Media/Pictures/Robot%20Pictures/Final%20Prototype%20%28Photo%202%29.jpg" width="240" />
<img src="./Media/Pictures/Robot%20Pictures/Final%20Prototype%20%28Photo%203%29.jpg" width="240" />
<img src="./Media/Pictures/Robot%20Pictures/Final%20Prototype%20%28Photo%204%29.jpg" width="240" />
<img src="./Media/Pictures/Robot%20Pictures/Final%20Prototype%20%28Photo%205%29.jpg" width="240" />
<img src="./Media/Pictures/Robot%20Pictures/Final%20Prototype%20%28Photo%206%29.jpg" width="240" />
<img src="./Media/Pictures/Robot%20Pictures/Final%20Prototype%20%28Photo%207%29.jpg" width="240" />
<img src="./Media/Pictures/Robot%20Pictures/Final%20Prototype%20%28Photo%208%29.jpg" width="240" />
<img src="./Media/Pictures/Robot%20Pictures/Final%20Prototype%20%28Photo%209%29.jpg" width="240" />
</p>

## Acknowledgments & References

Built for the Mechatronics Laboratory (Robot: Advanced) course, Kyoto University of Advanced Science, Spring 2026.

The clawed-wheel concept was inspired by prior work on configurable wheel-legs:

1. R. Sell, G. Aryassov, A. Petritshenko, and M. Kaeeli, "Kinematics and dynamics of configurable wheel-leg," in *Proc. 8th Int. DAAAM Baltic Conf. Industrial Engineering*.
2. C. Zheng and K. Lee, "Wheeler: Wheel-leg reconfigurable mechanism with passive gears for mobile robot applications," in *Proc. IEEE Int. Conf. Robotics and Automation (ICRA)*.
