// =============================================================
//  Combined node: 4x open-loop PWM DC motors + RC servos on a
//  PCA9685 16-channel I2C PWM driver, all commanded over one
//  serial link.
//
//  Motor control is taken verbatim from mbed_node — unchanged.
//
//  Servos are driven by a PCA9685 board over I2C instead of
//  bit-banged GPIO (SoftServo). WIRING:
//    Nucleo D14 (SDA, PB_9) -> PCA9685 SDA
//    Nucleo D15 (SCL, PB_8) -> PCA9685 SCL
//    Nucleo 3V3             -> PCA9685 VCC  (logic power only)
//    Nucleo GND             -> PCA9685 GND  <-- MUST be common with servo supply
//    5-6V supply +/-        -> PCA9685 V+/GND (separate servo power rail)
//  Do NOT power servos from the Nucleo 5V pin — a stalled servo can pull
//  several hundred mA and brown out the board. V+ and VCC are separate
//  rails; never bridge them (12V on VCC kills the chip).
//  This frees PB_9/PB_8/PB_11/PA_7, previously used as individual
//  SoftServo GPIO pins — PB_9/PB_8 now carry the I2C bus, PB_11/PA_7
//  are unused.
//
//  Serial command: the Pi forwards raw, single-byte keypresses — no
//  framing or terminator, one byte is one command. This firmware (not
//  the Pi) owns all interpretation now:
//    w a s d q         -> motor direction, scaled by the current speed level
//    0-9               -> select motor speed, 10%-100%
//    u j i k o l p ;   -> nudge servo 1-4 by +-SERVO_STEP degrees
//    c                 -> center all servos
//  Motors are hold-to-move: if no w/a/s/d/q byte arrives within
//  HOLD_TIMEOUT, they're stopped automatically (deadman watchdog —
//  this used to live in the Pi script, now lives here since the Pi
//  only forwards raw keys and no longer tracks timing itself). Servos
//  hold their last commanded angle instead — no watchdog.
//
//  Feedback: comma-separated echo of all setpoints on one line,
//  duties first, then servo angles, e.g.
//      0.50,-0.50,0.00,1.00,90.0,-45.0,0.0,135.0
//  (unchanged — still consumed by the Pi for ROS2 topic telemetry)
// =============================================================
#define NUM_MOTORS 4
#define NUM_SERVOS 3

#include "mbed.h"
#include "SerialROS2.hpp"
#include "PWMMotor.hpp"
#include "PCA9685.hpp"
#include <cstdio>

// ---------------- motors (identical to mbed_node) ----------------

// Pin pairs {forward_pin, backward_pin} for each motor slot.
static const PinName MOTOR_PINS[4][2] = {
    {PA_8, PA_9},   // Motor 1
    {PC_6, PC_7},   // Motor 2 //PB_15
    {PB_14_ALT0, PB_15_ALT1},  // Motor 3  <-- set your pins here
    {PA_4, PA_6_ALT0},   // Motor 4  <-- set your pins here
};

PWMMotor* motors[NUM_MOTORS];
float     duties[NUM_MOTORS] = {};

// ---------------- servos (PCA9685 over I2C) ----------------

// Sized to 4 regardless of NUM_SERVOS (same pattern as MOTOR_PINS above):
// only the first NUM_SERVOS entries are used, so NUM_SERVOS can be set to
// 1-4 without touching these tables.
static_assert(NUM_SERVOS >= 1 && NUM_SERVOS <= 4, "NUM_SERVOS must be 1-4, or add more entries to SERVO_CH/PULSE_*_US/servo_range below");

// NOTE: named PCA_SDA/PCA_SCL, not I2C_SDA/I2C_SCL — mbed's HAL headers
// already #define I2C_SDA/I2C_SCL (Arduino-shield pin aliases), and reusing
// those names here gets silently macro-expanded into the wrong declaration.
static const PinName PCA_SDA  = D14;   // PB_9
static const PinName PCA_SCL  = D15;   // PB_8
static const uint8_t PCA_ADDR = 0x40;  // bump if you bridged the A0..A5 jumpers
static const float   SERVO_FREQ = 50.0f;   // Hz — standard hobby servo frame

// Which PCA9685 output channel (0-15) each servo is plugged into.
static const uint8_t SERVO_CH[4] = { 0, 1, 2, 3 };

// Per-servo pulse-width calibration, in microseconds, at full deflection.
// 500-2500us is the usual full range, but every servo differs — widen
// gradually and STOP before the servo grinds against its physical end
// stop, or you will strip the gears.
static const float PULSE_MIN_US[4] = { 500.0f, 500.0f, 500.0f, 500.0f };
static const float PULSE_MAX_US[4] = { 2500.0f, 2500.0f, 2500.0f, 2500.0f };

// notes: range from center to min/max, in degrees.
// int servo_range[4] = {90, 90, 90, 90};     // DS3225
int servo_range[4] = {90, 90, 90, 90};    // 8125MG

PCA9685 pca(PCA_SDA, PCA_SCL, PCA_ADDR);
float servo_angles[NUM_SERVOS]  = {};   // target angle, nudged by servo keys (centered at 0)
float servo_current[NUM_SERVOS] = {};   // ramped/smoothed angle actually sent to the PCA9685

static const float SERVO_MAX_SPEED = 300.0f;    // deg/s slew limit (same default as before)
static constexpr auto SERVO_FRAME  = 20ms;       // one PWM frame — matches SERVO_FREQ
static constexpr float SERVO_FRAME_S = 0.020f;

// Map a centered angle (-servo_range..+servo_range) to this servo's
// calibrated pulse width and send it over I2C.
void set_servo_angle(int servo, float deg_from_center) {
    if (servo < 0 || servo >= NUM_SERVOS) return;

    float range = (float)servo_range[servo];
    if (deg_from_center < -range) deg_from_center = -range;
    if (deg_from_center >  range) deg_from_center =  range;

    float center_us    = (PULSE_MIN_US[servo] + PULSE_MAX_US[servo]) / 2.0f;
    float half_span_us = (PULSE_MAX_US[servo] - PULSE_MIN_US[servo]) / 2.0f;
    float pulse_us      = center_us + (deg_from_center / range) * half_span_us;

    pca.set_pulse_us(SERVO_CH[servo], pulse_us);
}

// ---------------- serial link ----------------

SerialROS2 pc(USBTX, USBRX, 115200);

DigitalOut led(LED1);   // onboard green user LED — toggles on every recognized keypress

// ---------------- keypress interpretation (moved here from the Pi) ----------------

static const float SPEED_LEVELS[10] = {
    0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f
};
static const int DEFAULT_SPEED_INDEX = 7;   // '7' -> 0.8

float current_speed = SPEED_LEVELS[DEFAULT_SPEED_INDEX];      // duty magnitude picked by 0-9
float last_motor_signs[NUM_MOTORS] = {0, 0, 0, 0};            // direction template of the last motor key

// Sign template per motor for each recognized movement key.
// Layout: [front-left, front-right, rear-left, rear-right].
struct MotorKeyEntry { char key; float signs[NUM_MOTORS]; };
static const MotorKeyEntry MOTOR_KEYS[] = {
    { 'w', { 1,  1,  1,  1} },
    { 's', {-1, -1, -1, -1} },
    { 'a', {1,  -1, 1,  -1} },
    { 'd', { -1, 1,  -1, 1} },
    { 'q', { 0,  0,  0,  0} },
};
static const int NUM_MOTOR_KEYS = sizeof(MOTOR_KEYS) / sizeof(MOTOR_KEYS[0]);

// Stop the motors if no movement key arrives within this long — the same
// hold-to-move deadman watchdog the Pi used to run, now living here since
// the Pi only forwards raw keys and no longer tracks timing itself.
static constexpr auto HOLD_TIMEOUT = 300ms;
bool moving = false;
Timer cmd_timer;                                 // free-running, started once in main()
std::chrono::microseconds last_cmd_time{0};

void applyMotorSigns(const float* signs) {
    bool any_nonzero = false;
    for (int i = 0; i < NUM_MOTORS; i++) {
        last_motor_signs[i] = signs[i];
        duties[i] = signs[i] * current_speed;
        if (duties[i] != 0.0f) any_nonzero = true;
    }
    last_cmd_time = cmd_timer.elapsed_time();
    moving = any_nonzero;   // 'q' -> all zero -> not moving
}

static const float SERVO_STEP = 2.5f;   // degrees added per keypress / auto-repeat

// Keyboard -> (servo index, direction) for incremental servo moves.
struct ServoKeyEntry { char key; int index; float dir; };
static const ServoKeyEntry SERVO_KEYS[] = {
    { 'u', 0, -1 }, { 'j', 0, +1 },   // servo 1
    { 'i', 1, -1 }, { 'k', 1, +1 },   // servo 2
    { 'o', 2, -1 }, { 'l', 2, +1 },   // servo 3
    { 'p', 3, -1 }, { ';', 3, +1 },   // servo 4
};
static const int NUM_SERVO_KEYS = sizeof(SERVO_KEYS) / sizeof(SERVO_KEYS[0]);
static const char CENTER_KEY = 'c';   // snap all servos back to 0 degrees

// Dispatch one raw byte from the Pi. LED toggles once per recognized key,
// same heartbeat role the old setCommand() callback used to serve.
void handleKey(char key) {
    bool matched = false;

    for (int i = 0; i < NUM_MOTOR_KEYS && !matched; i++) {
        if (MOTOR_KEYS[i].key == key) {
            applyMotorSigns(MOTOR_KEYS[i].signs);
            matched = true;
        }
    }

    if (!matched && key >= '0' && key <= '9') {
        current_speed = SPEED_LEVELS[key - '0'];
        if (moving) {
            // Rescale the motors already in motion immediately, instead of
            // waiting for the next movement keypress.
            for (int i = 0; i < NUM_MOTORS; i++) {
                duties[i] = last_motor_signs[i] * current_speed;
            }
        }
        matched = true;
    }

    if (!matched) {
        for (int i = 0; i < NUM_SERVO_KEYS; i++) {
            if (SERVO_KEYS[i].key == key) {
                int idx = SERVO_KEYS[i].index;
                if (idx < NUM_SERVOS) {
                    float a     = servo_angles[idx] + SERVO_KEYS[i].dir * SERVO_STEP;
                    float range = (float)servo_range[idx];
                    if (a < -range) a = -range;
                    if (a >  range) a =  range;
                    servo_angles[idx] = a;
                }
                matched = true;
                break;
            }
        }
    }

    if (!matched && key == CENTER_KEY) {
        for (int i = 0; i < NUM_SERVOS; i++) servo_angles[i] = 0;
        matched = true;
    }

    if (matched) led = !led;
}

int main() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i] = new PWMMotor(MOTOR_PINS[i][0], MOTOR_PINS[i][1]);
        motors[i]->init();
    }

    if (!pca.init(SERVO_FREQ)) {
        // No ACK on the I2C bus. Check: SDA/SCL swapped? VCC missing?
        // Wrong address? Common ground absent?
        while (1) {                       // fast blink = I2C failure
            led = !led;
            ThisThread::sleep_for(100ms);
        }
    }
    for (int i = 0; i < NUM_SERVOS; i++) {
        servo_current[i] = 0;
        set_servo_angle(i, 0);   // start centered
        // Stagger each servo's very first pulse. The slew-rate ramp below
        // only smooths CHANGES to an already-moving servo — it can't soften
        // this first command, since every channel is hard "off" (no pulse
        // at all) until now. If a servo isn't already sitting at 0 degrees,
        // it will race there at its own full speed/torque the instant it
        // gets this pulse. Spacing these out stops N servos' worst-case
        // inrush current from lining up on the same instant at boot.
        ThisThread::sleep_for(150ms);
    }

    pc.init();

    char fb[128];

    Timer servo_timer;
    servo_timer.start();
    cmd_timer.start();

    while (1) {
        // Drain all pending bytes so a burst of auto-repeated keys doesn't
        // lag behind — same reasoning the old Pi-side loop used.
        char c;
        while (pc.pc.readable()) {
            if (pc.pc.read(&c, 1) == 1) {
                handleKey(c);
            }
        }

        // Watchdog: if we were moving but no movement key has refreshed
        // last_cmd_time within HOLD_TIMEOUT, the key was released — stop
        // the motors. Servos are NOT touched: they keep holding their angle.
        if (moving && (cmd_timer.elapsed_time() - last_cmd_time) > HOLD_TIMEOUT) {
            for (int i = 0; i < NUM_MOTORS; i++) duties[i] = 0.0f;
            moving = false;
        }

        for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i]->set_duty(duties[i]);
        }

        // Servo update: rate-limited to one PWM frame (20ms, matches
        // SERVO_FREQ) with a slew-rate ramp toward the target — same
        // smoothing behavior SoftServo used to provide internally.
        if (servo_timer.elapsed_time() >= SERVO_FRAME) {
            servo_timer.reset();

            float max_step = SERVO_MAX_SPEED * SERVO_FRAME_S;
            for (int i = 0; i < NUM_SERVOS; i++) {
                float diff = servo_angles[i] - servo_current[i];
                if (diff > max_step)       diff = max_step;
                else if (diff < -max_step) diff = -max_step;
                servo_current[i] += diff;
                set_servo_angle(i, servo_current[i]);
            }
        }

        // Feedback: duties then servo angles, comma-separated on one line
        // e.g. "0.60,-0.60,0.00,0.00,90.0,-45.0,0.0,135.0\n"
        // Sent through pc (the same BufferedSerial), NOT printf, to avoid
        // two drivers contending for the console UART. Reports the target
        // setpoint (servo_angles), same as before — not the ramped value.
        int len = 0;
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (i > 0) len += snprintf(fb + len, sizeof(fb) - len, ",");
            len += snprintf(fb + len, sizeof(fb) - len, "%.2f", duties[i]);
        }
        for (int i = 0; i < NUM_SERVOS; i++) {
            len += snprintf(fb + len, sizeof(fb) - len, ",%.1f", servo_angles[i]);
        }
        len += snprintf(fb + len, sizeof(fb) - len, "\n");
        pc.send(fb, len);

        wait_us(1000);
    }
}
