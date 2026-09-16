# combined_node

Firmware for the Nucleo-F091RC that controls **4 DC motors** (open-loop PWM,
taken unchanged from `mbed_node`) and **4 RC servos**, all commanded over one
serial link (ST-Link virtual COM port, 115200 baud).

## Pinout

### Motors (unchanged from mbed_node)
| Motor | Forward | Backward |
|-------|---------|----------|
| 1 | PA_8 | PA_9 |
| 2 | PC_6 | PC_7 |
| 3 | PB_14 (ALT0) | PB_15 (ALT1) |
| 4 | PA_4 | PA_6 (ALT0) |

### Servos (new)
| Servo | Pin |
|-------|-----|
| 1 | PB_9 |
| 2 | PB_8 |
| 3 | PB_11 |
| 4 | PA_7 |

Servos are driven by **`SoftServo`** (`lib/SoftServo`), a bit-banged PWM
implementation using `Ticker` + `Timeout`, not mbed's hardware `PwmOut`.
This is a deliberate design choice, not a workaround of convenience:

- Every STM32 timer with more than one channel is already claimed by the
  4 motors (TIM1, TIM3, TIM14, TIM15, TIM16), and a timer's period is
  shared across all of its channels — a servo sharing a motor's timer
  would corrupt one or the other's PWM frequency.
- `TIM2`, which pins like `PB_10`/`PB_11`/`PA_0`/`PB_3` map to, is reserved
  internally by mbed OS as the microsecond ticker and has **no PWM pin
  mapping at all** on this target (`PeripheralPins.c` has every TIM2 row
  commented out). Constructing a hardware `Servo` on one of those pins
  hits an invalid pinmap lookup and asserts at boot — before the serial
  link even comes up, so the whole board appears dead, not just the servo.
- After the motors, only one hardware PWM channel remains free on this
  chip (TIM17, one channel) — not enough for 4 independent servos.

`SoftServo` sidesteps all of this: `Ticker`/`Timeout` are OS-level
scheduled callbacks, not exclusive timer ownership, so they work on any
plain GPIO without colliding with the motors or mbed's internal timing.
That means the servo pins above can be **any** free GPIO — they don't
need to be PWM-capable, so wiring isn't constrained by the chip's timer
map.

Power the servos and motors from an external supply, not the Nucleo's 5V
pin; connect the supply ground to the Nucleo GND.

## Serial protocol

**Command** (host → board): 8 values separated by `/`, terminated by `d`:

```
<duty1>/<duty2>/<duty3>/<duty4>/<ang1>/<ang2>/<ang3>/<ang4>d
```

- Duties: motor duty cycle in `[-1.0, 1.0]` (negative = reverse, 0 = stop)
- Angles: servo position in degrees, `[-servo_range, +servo_range]`
  (default ±135° for the 8125MG; edit `servo_range[]` in `src/main.cpp`)

Example — motors 1+2 forward at 50%, servo 1 to +90°, servo 4 to −45°:

```
0.5/0.5/0/0/90/0/0/-45d
```

Always send all 8 values. The onboard LED toggles each time a command is
parsed.

**Feedback** (board → host): current setpoints echoed continuously as one
comma-separated line, duties first, then angles:

```
0.50,0.50,0.00,0.00,90.0,0.0,0.0,-45.0
```

## Build & flash

```
pio run              # build
pio run -t upload    # flash via ST-Link
```
