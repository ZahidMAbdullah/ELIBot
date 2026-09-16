#ifndef _PWMMOTOR_HPP
#define _PWMMOTOR_HPP

#include "mbed.h"

// Open-loop PWM motor driver — no encoder, no PID.
// Accepts duty cycle in range [-1.0, 1.0]:
//   +1.0 = full forward, -1.0 = full reverse, 0.0 = stop

class PWMMotor {
public:
    PWMMotor(PinName pin_a, PinName pin_b)
        : pwm_a(pin_a), pwm_b(pin_b) {}

    void init();
    void set_duty(float duty);   // -1.0 to 1.0

private:
    PwmOut pwm_a;
    PwmOut pwm_b;

    static const int PWM_PERIOD_US = 4000;   // 250 Hz, same as original

    void forward(int pw);
    void backward(int pw);
};

#endif
