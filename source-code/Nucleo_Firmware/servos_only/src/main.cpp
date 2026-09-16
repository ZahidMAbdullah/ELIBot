// =============================================================
//  DEBUG ISOLATION BUILD — servos only.
//
//  Extracted verbatim from combined_node/src/main.cpp: every line here
//  is unchanged from the combined project except that the DC motor
//  subsystem has been removed entirely. If servos misbehave here exactly
//  as they do in combined_node, the bug is in the servo/I2C path or the
//  serial protocol. If servos work fine here, the bug is specific to
//  running motors and servos together (power domain / brownout).
//
//  NOTE: the servo key directions and SERVO_STEP below were previously
//  out of sync with combined_node (inverted directions, different step
//  size) — fixed to match exactly, so this build is actually usable as
//  a same-behavior isolation copy.
//
//  WIRING:
//    Nucleo D14 (SDA, PB_9) -> PCA9685 SDA
//    Nucleo D15 (SCL, PB_8) -> PCA9685 SCL
//    Nucleo 3V3             -> PCA9685 VCC  (logic power only)
//    Nucleo GND             -> PCA9685 GND  <-- MUST be common with servo supply
//    5-6V supply +/-        -> PCA9685 V+/GND (separate servo power rail)
//
//  Serial command: the Pi forwards raw, single-byte keypresses — no
//  framing or terminator, one byte is one command.
//    u j i k o l p ;   -> nudge servo 1-4 by +-SERVO_STEP degrees
//    c                 -> center all servos
//
//  Feedback: comma-separated echo of servo angles on one line, e.g.
//      90.0,-45.0,0.0,135.0
// =============================================================
#define NUM_SERVOS 3

#include "mbed.h"
#include "SerialROS2.hpp"
#include "PCA9685.hpp"
#include <cstdio>

// ---------------- servos (PCA9685 over I2C, identical to combined_node) ----------------

static_assert(NUM_SERVOS >= 1 && NUM_SERVOS <= 4, "NUM_SERVOS must be 1-4, or add more entries to SERVO_CH/PULSE_*_US/servo_range below");

// NOTE: named PCA_SDA/PCA_SCL, not I2C_SDA/I2C_SCL — mbed's HAL headers
// already #define I2C_SDA/I2C_SCL (Arduino-shield pin aliases), and reusing
// those names here gets silently macro-expanded into the wrong declaration.
static const PinName PCA_SDA  = D14;   // PB_9
static const PinName PCA_SCL  = D15;   // PB_8
static const uint8_t PCA_ADDR = 0x40;  // bump if you bridged the A0..A5 jumpers
static const float   SERVO_FREQ = 50.0f;   // Hz — standard hobby servo frame

static const uint8_t SERVO_CH[4] = { 0, 1, 2, 3 };
static const float PULSE_MIN_US[4] = { 500.0f, 500.0f, 500.0f, 500.0f };
static const float PULSE_MAX_US[4] = { 2500.0f, 2500.0f, 2500.0f, 2500.0f };

int servo_range[4] = {90, 90, 90, 90};    // 8125MG

PCA9685 pca(PCA_SDA, PCA_SCL, PCA_ADDR);
float servo_angles[NUM_SERVOS]  = {};   // target angle, nudged by servo keys (centered at 0)
float servo_current[NUM_SERVOS] = {};   // ramped/smoothed angle actually sent to the PCA9685

static const float SERVO_MAX_SPEED = 300.0f;    // deg/s slew limit
static constexpr auto SERVO_FRAME  = 20ms;       // one PWM frame — matches SERVO_FREQ
static constexpr float SERVO_FRAME_S = 0.020f;

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

// ---------------- keypress interpretation (identical to combined_node) ----------------

static const float SERVO_STEP = 2.5f;   // degrees added per keypress / auto-repeat

struct ServoKeyEntry { char key; int index; float dir; };
static const ServoKeyEntry SERVO_KEYS[] = {
    { 'u', 0, -1 }, { 'j', 0, +1 },   // servo 1
    { 'i', 1, -1 }, { 'k', 1, +1 },   // servo 2
    { 'o', 2, -1 }, { 'l', 2, +1 },   // servo 3
    { 'p', 3, -1 }, { ';', 3, +1 },   // servo 4
};
static const int NUM_SERVO_KEYS = sizeof(SERVO_KEYS) / sizeof(SERVO_KEYS[0]);
static const char CENTER_KEY = 'c';   // snap all servos back to 0 degrees

void handleKey(char key) {
    bool matched = false;

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

    if (!matched && key == CENTER_KEY) {
        for (int i = 0; i < NUM_SERVOS; i++) servo_angles[i] = 0;
        matched = true;
    }

    if (matched) led = !led;
}

int main() {
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
        // Stagger each servo's very first pulse — see combined_node's
        // main.cpp header comment for why.
        ThisThread::sleep_for(150ms);
    }

    pc.init();

    char fb[64];

    Timer servo_timer;
    servo_timer.start();

    while (1) {
        char c;
        while (pc.pc.readable()) {
            if (pc.pc.read(&c, 1) == 1) {
                handleKey(c);
            }
        }

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

        // Feedback: servo angles, comma-separated on one line, e.g.
        // "90.0,-45.0,0.0,135.0\n" — reports the target setpoint
        // (servo_angles), same as combined_node, not the ramped value.
        int len = 0;
        for (int i = 0; i < NUM_SERVOS; i++) {
            if (i > 0) len += snprintf(fb + len, sizeof(fb) - len, ",");
            len += snprintf(fb + len, sizeof(fb) - len, "%.1f", servo_angles[i]);
        }
        len += snprintf(fb + len, sizeof(fb) - len, "\n");
        pc.send(fb, len);

        wait_us(1000);
    }
}
