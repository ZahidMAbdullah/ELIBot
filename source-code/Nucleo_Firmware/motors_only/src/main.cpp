// =============================================================
//  DEBUG ISOLATION BUILD — motors only.
//
//  Extracted verbatim from combined_node/src/main.cpp: every line here
//  is unchanged from the combined project except that the PCA9685/servo
//  subsystem has been removed entirely. If motors misbehave here exactly
//  as they do in combined_node, the bug is in the motor path or the
//  serial protocol. If motors work fine here, the bug is specific to
//  running motors and servos together (power domain / brownout).
//
//  NOTE: the a/d (strafe/turn) sign tables below were previously out of
//  sync with combined_node (mirrored signs) — fixed to match exactly, so
//  this build is actually usable as a same-behavior isolation copy.
//
//  Serial command: the Pi forwards raw, single-byte keypresses — no
//  framing or terminator, one byte is one command.
//    w a s d q  -> motor direction, scaled by the current speed level
//    0-9        -> select motor speed, 10%-100%
//  Motors are hold-to-move: if no w/a/s/d/q byte arrives within
//  HOLD_TIMEOUT, they're stopped automatically (deadman watchdog).
//
//  Feedback: comma-separated echo of motor duties on one line, e.g.
//      0.50,-0.50,0.00,1.00
// =============================================================
#define NUM_MOTORS 4

#include "mbed.h"
#include "SerialROS2.hpp"
#include "PWMMotor.hpp"
#include <cstdio>

// ---------------- motors (identical to combined_node) ----------------

static const PinName MOTOR_PINS[4][2] = {
    {PA_8, PA_9},   // Motor 1
    {PC_6, PC_7},   // Motor 2
    {PB_14_ALT0, PB_15_ALT1},  // Motor 3
    {PA_4, PA_6_ALT0},   // Motor 4
};

PWMMotor* motors[NUM_MOTORS];
float     duties[NUM_MOTORS] = {};

// ---------------- serial link ----------------

SerialROS2 pc(USBTX, USBRX, 115200);

DigitalOut led(LED1);   // onboard green user LED — toggles on every recognized keypress

// ---------------- keypress interpretation (identical to combined_node) ----------------

static const float SPEED_LEVELS[10] = {
    0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f
};
static const int DEFAULT_SPEED_INDEX = 7;   // '7' -> 0.8

float current_speed = SPEED_LEVELS[DEFAULT_SPEED_INDEX];
float last_motor_signs[NUM_MOTORS] = {0, 0, 0, 0};

struct MotorKeyEntry { char key; float signs[NUM_MOTORS]; };
static const MotorKeyEntry MOTOR_KEYS[] = {
    { 'w', { 1,  1,  1,  1} },
    { 's', {-1, -1, -1, -1} },
    { 'a', {1,  -1, 1,  -1} },
    { 'd', { -1, 1,  -1, 1} },
    { 'q', { 0,  0,  0,  0} },
};
static const int NUM_MOTOR_KEYS = sizeof(MOTOR_KEYS) / sizeof(MOTOR_KEYS[0]);

static constexpr auto HOLD_TIMEOUT = 300ms;
bool moving = false;
Timer cmd_timer;
std::chrono::microseconds last_cmd_time{0};

void applyMotorSigns(const float* signs) {
    bool any_nonzero = false;
    for (int i = 0; i < NUM_MOTORS; i++) {
        last_motor_signs[i] = signs[i];
        duties[i] = signs[i] * current_speed;
        if (duties[i] != 0.0f) any_nonzero = true;
    }
    last_cmd_time = cmd_timer.elapsed_time();
    moving = any_nonzero;
}

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
            for (int i = 0; i < NUM_MOTORS; i++) {
                duties[i] = last_motor_signs[i] * current_speed;
            }
        }
        matched = true;
    }

    if (matched) led = !led;
}

int main() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i] = new PWMMotor(MOTOR_PINS[i][0], MOTOR_PINS[i][1]);
        motors[i]->init();
    }

    pc.init();

    char fb[64];

    cmd_timer.start();

    while (1) {
        // Drain all pending bytes so a burst of auto-repeated keys doesn't
        // lag behind — same reasoning the combined project used.
        char c;
        while (pc.pc.readable()) {
            if (pc.pc.read(&c, 1) == 1) {
                handleKey(c);
            }
        }

        if (moving && (cmd_timer.elapsed_time() - last_cmd_time) > HOLD_TIMEOUT) {
            for (int i = 0; i < NUM_MOTORS; i++) duties[i] = 0.0f;
            moving = false;
        }

        for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i]->set_duty(duties[i]);
        }

        int len = 0;
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (i > 0) len += snprintf(fb + len, sizeof(fb) - len, ",");
            len += snprintf(fb + len, sizeof(fb) - len, "%.2f", duties[i]);
        }
        len += snprintf(fb + len, sizeof(fb) - len, "\n");
        pc.send(fb, len);

        wait_us(1000);
    }
}
