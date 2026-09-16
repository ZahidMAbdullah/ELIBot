# ML3_Nucleo_Projects

PlatformIO/mbed firmware for the STM32 Nucleo-F091RC that drives ELIBot's 4 drive motors and 3 manipulator servos. Three build targets live here:

| Project | Purpose |
|---|---|
| **`combined_node/`** | **The final firmware.** Controls all 4 DC motors (open-loop PWM via 2× L298N) and all 3 servos (via a PCA9685 I2C board) together, over one USB-serial link. This is what ships on the robot. |
| `motors_only/` | Debug isolation build — motors and the serial protocol only, no servo/PCA9685 subsystem. Extracted verbatim from `combined_node` to check whether a motor problem is specific to running motors and servos together (shared power rail / brownout) or exists on its own. |
| `servos_only/` | Debug isolation build — servos and the serial protocol only, no motor subsystem. Same idea as `motors_only`, for isolating servo/I2C problems. |

All three share the same serial command protocol (see `combined_node/README.md`) and the same hold-to-move/watchdog behavior; `motors_only` and `servos_only` are kept in sync with `combined_node`'s key mappings so a bug reproduced in isolation reflects the same behavior in the combined firmware.

## Building

Each project is a standalone PlatformIO project:

```
cd combined_node        # or motors_only / servos_only
pio run                 # build
pio run -t upload       # flash via ST-Link
```

## Shared libraries (`lib/`)

- **`PWMMotor`** — open-loop PWM H-bridge motor driver (no encoder/PID), used by `combined_node` and `motors_only`.
- **`PCA9685`** — driver for the 16-channel I2C PWM board used to generate all 3 servo signals, used by `combined_node` and `servos_only`.
- **`ros2mbed` (`SerialROS2`)** — the serial command parser shared by all three projects.

Two earlier servo-driving approaches — a hardware-`PwmOut`-based `Servo` class, then a bit-banged-GPIO `SoftServo` — were tried before settling on the PCA9685 board (freeing up GPIO pins and sidestepping the Nucleo-F091RC's limited free PWM timers) and have since been removed from `combined_node/lib`, since neither is `#include`d by the final firmware.
