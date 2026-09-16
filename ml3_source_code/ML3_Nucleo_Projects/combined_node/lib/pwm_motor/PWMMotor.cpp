#include "PWMMotor.hpp"

void PWMMotor::init() {
    pwm_a.period_us(PWM_PERIOD_US);
    pwm_b.period_us(PWM_PERIOD_US);
    pwm_a.pulsewidth_us(0);
    pwm_b.pulsewidth_us(0);
}

void PWMMotor::set_duty(float duty) {
    if (duty >  1.0f) duty =  1.0f;
    if (duty < -1.0f) duty = -1.0f;

    int pw = (int)(duty * PWM_PERIOD_US);

    if (pw > 0)       forward(pw);
    else if (pw < 0)  backward(-pw);
    else {
        pwm_a.pulsewidth_us(0);
        pwm_b.pulsewidth_us(0);
    }
}

void PWMMotor::forward(int pw) {
    pwm_a.pulsewidth_us(pw);
    pwm_b.pulsewidth_us(0);
}

void PWMMotor::backward(int pw) {
    pwm_a.pulsewidth_us(0);
    pwm_b.pulsewidth_us(pw);
}
