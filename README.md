# ELIBot — Mechatronics Laboratory (Robot: Advanced), Team 12

**ELIBot** is a four-wheeled, spiked/clawed-wheel all-terrain robot built for the Mechatronics Laboratory (Robot: Advanced) course at **Kyoto University of Advanced Science (KUAS)**, Spring 2026. It combines a raised, high-clearance chassis, a custom rigid/flexible claw wheel, and a lightweight single-DOF manipulator arm to autonomously/teleoperatively clear a 10-obstacle challenge course — doors, stairs, ditches, uneven terrain, a traffic-light-gated barrier, QR-code checkpoints, and an object-retrieval task.

> This repository documents the full design process — from the original tri-wheel concept to the final ELIBot — along with its firmware, ROS2 software stack, CAD/print files, and build media.

## Table of Contents

- [Team](#team)
- [The Challenge](#the-challenge)
- [Design Journey](#design-journey)
- [Final Design](#final-design)
  - [Mechanical](#mechanical)
  - [Manipulator Arm](#manipulator-arm)
  - [Electronics](#electronics)
  - [Software Architecture](#software-architecture)
- [Operating Instructions](#operating-instructions)
- [Repository Structure](#repository-structure)
- [Documents](#documents)
- [Acknowledgments & References](#acknowledgments--references)

## Team

**Team 12**, Mechatronics Laboratory (Robot: Advanced), Spring 2025:

- Abdullah Al Ayaat
- G. C. Mohnish
- Catherine Ella Burtenshaw
- Daphne Jillian Gan Tan
- Muhammad Abdullah Zahid

## The Challenge

The course culminates in a 15th-week competition on a **364 cm × 273 cm** field with a 20 mm-wide line to follow and 10 distinct obstacles, run across two stages of increasing difficulty:

| # | Obstacle | Stage-I | Stage-II |
|---|---|---|---|
| 1 | Door | Open by any means | Must be opened using the door handle |
| 2 | Slope | Climb up | Climb up |
| 3 | Uneven terrain | Traverse random-height steps (5–25 mm) | Traverse random-height steps |
| 4 | Ditch crossing | Cross a 10×44×4 cm ditch | Cross the ditch |
| 5 | QR code | Read it, change an onboard LED's color | Rotate the cardboard, then read + change LED color |
| 6 | Speed bump | Cross it | Cross it |
| 7 | Roller (pipe) | Cross a ¾" pipe roller | Cross the roller |
| 8 | Stairs | Climb up/down from any direction | Climb up/down |
| 9 | Traffic light & barrier | Detect the light color, signal it wirelessly to open the barrier | Same |
| 10 | Object transport | Move a 5×5×11 cm object to the goal | **Lift** the object and transport it |

Robots may be **autonomous, teleoperated, or hybrid**, must be **untethered**, and score out of 140 points (100 from field performance + 40 from instructor evaluation on originality, presentation, robot completeness, and team effort). Autonomous robots earn a 15-point bonus for reaching a marked checkpoint unassisted. Teleoperated robots must be controlled from a fixed operator station using only the on-robot camera feed (no direct line of sight), over a dedicated wireless link.

Full rules: [`Robot Challenge and Field Details 2025.pdf`](./Robot%20Challenge%20and%20Field%20Details%202025.pdf)

## Design Journey

The team's first concept was a **tri-wheel robot**: a spring-suspended front "wheel-leg" (inspired by configurable wheel-leg mechanisms in the literature) for climbing stairs/ditches, a rear pair of wheels on a 4-bar spring suspension for uneven terrain, and a top-mounted 3-DOF manipulator arm with a solenoid-driven pincher gripper.

During prototyping, the tri-wheel base proved **unstable**, so the team pivoted to a **four-wheeled** layout partway through the build — trading the reconfigurable front wheel-leg for a fixed spiked/clawed wheel on all four corners, and simplifying the arm from 3-DOF + pincher down to a lighter 1-DOF hook mechanism for better weight distribution and stair-climbing stability.

Both design phases are preserved in this repository — see [`Old Design/`](./Old%20Design) for the original tri-wheel concept (CAD, images, videos) and [`Robot Concept Presentation/`](./Robot%20Concept%20Presentation) for the original pitch, versus the final design described below and detailed in the [final report](./Teams12_final_report.pdf).

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

Firmware protocol (STM32 side): 8 slash-separated values — 4 motor duty cycles `[-1.0, 1.0]` and 4 servo angles (degrees) — terminated by `d`, e.g. `0.5/0.5/0/0/90/0/0/-45d`. See [`ml3_source_code/ML3_Nucleo_Projects/combined_node/README.md`](./ml3_source_code/ML3_Nucleo_Projects/combined_node/README.md) for the full protocol and pin mapping.

## Operating Instructions

1. Connect the STM32 to the Raspberry Pi 4; power the Pi from the power bank.
2. (Setup/debug) Connect via HDMI to a monitor, or view the Pi's desktop remotely, to launch ROS2 nodes from a terminal.
3. Launch, in order:
   1. Camera publisher node → publishes to `/camera/image_raw`
   2. Vision status node → subscribes to the camera topic, publishes `/vision_status` on state changes
   3. LED indicator node → subscribes to `/vision_status`, drives the QR indicator LEDs
   4. Teleoperation node → reads keyboard input, sends serial commands to the STM32
4. Disconnect the setup peripherals, connect the drive battery, and verify wheel + arm motion before running the course.

Full details: [final report, §3](./Teams12_final_report.pdf).

## Repository Structure

> This repo is being populated incrementally; the layout below is the target structure and will be filled in over the next few commits.

```
.
├── README.md
├── Teams12_final_report.pdf
├── Robot Challenge and Field Details 2025.pdf
├── Robot Concept Presentation/       # original tri-wheel pitch deck
├── Intermediate Presentation/        # mid-project checkpoint deck
├── Old Design/                       # scrapped tri-wheel concept: CAD, images, videos
├── STL Files/                        # final ELIBot print files (STL/gcode)
├── ml3_source_code/
│   ├── ML3_Nucleo_Projects/          # STM32 firmware (PlatformIO/mbed)
│   ├── ml3_ros2_ws_robot/            # ROS2 workspace deployed on the Raspberry Pi
│   └── ml3_ros2_ws_server/           # ROS2 workspace for the off-robot vision server
└── Media/                            # photos and video (large raw footage hosted externally)
```

## Documents

- [Final report](./Teams12_final_report.pdf) — full design writeup (this README summarizes it)
- [Challenge & field rules](./Robot%20Challenge%20and%20Field%20Details%202025.pdf)
- [Concept presentation](./Robot%20Concept%20Presentation/ML3_Presentation_Team12.pdf) — original tri-wheel pitch
- [Intermediate presentation](./Intermediate%20Presentation/Team12_IntermediatePresentation.pdf) — mid-project design pivot and status

## Acknowledgments & References

Built for the Mechatronics Laboratory (Robot: Advanced) course, Kyoto University of Advanced Science, Spring 2025.

The clawed-wheel concept was inspired by prior work on configurable wheel-legs:

1. R. Sell, G. Aryassov, A. Petritshenko, and M. Kaeeli, "Kinematics and dynamics of configurable wheel-leg," in *Proc. 8th Int. DAAAM Baltic Conf. Industrial Engineering*.
2. C. Zheng and K. Lee, "Wheeler: Wheel-leg reconfigurable mechanism with passive gears for mobile robot applications," in *Proc. IEEE Int. Conf. Robotics and Automation (ICRA)*.
