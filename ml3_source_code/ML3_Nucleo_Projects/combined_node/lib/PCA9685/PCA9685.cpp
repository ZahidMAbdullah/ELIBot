#include "PCA9685.hpp"

// --- Low-level register access ------------------------------------------

bool PCA9685::write_reg(uint8_t reg, uint8_t value) {
    char buf[2] = { (char)reg, (char)value };
    // I2C::write returns 0 on success (all bytes ACKed).
    return i2c.write(addr8, buf, 2) == 0;
}

bool PCA9685::read_reg(uint8_t reg, uint8_t& value) {
    char r = (char)reg;
    // repeated=true: hold the bus between write and read (no STOP).
    if (i2c.write(addr8, &r, 1, true) != 0) return false;
    char v = 0;
    if (i2c.read(addr8, &v, 1) != 0) return false;
    value = (uint8_t)v;
    return true;
}

// --- Setup ---------------------------------------------------------------

bool PCA9685::init(float freq_hz) {
    i2c.frequency(400000);              // 400 kHz fast mode

    // MODE1: wake from sleep, enable auto-increment (needed to write the
    // 4 consecutive ON_L/ON_H/OFF_L/OFF_H bytes of a channel in one go).
    if (!write_reg(REG_MODE1, MODE1_AI | MODE1_ALLCALL)) {
        return false;                   // no ACK -> wrong address or no wiring
    }
    ThisThread::sleep_for(5ms);         // oscillator needs ~500us to stabilise

    set_pwm_freq(freq_hz);

    // Park all 16 channels off so nothing twitches at boot.
    for (uint8_t ch = 0; ch < 16; ch++) {
        set_off(ch);
    }
    return true;
}

void PCA9685::set_pwm_freq(float freq_hz) {
    if (freq_hz < 40.0f)   freq_hz = 40.0f;      // datasheet limits
    if (freq_hz > 1600.0f) freq_hz = 1600.0f;
    freq = freq_hz;

    // prescale = round(osc / (4096 * freq)) - 1
    float prescale_f = (OSC_CLOCK / (4096.0f * freq_hz)) - 1.0f;
    uint8_t prescale = (uint8_t)(prescale_f + 0.5f);

    // PRESCALE is only writable while the chip is asleep.
    uint8_t old_mode = 0;
    read_reg(REG_MODE1, old_mode);

    uint8_t sleep_mode = (old_mode & ~MODE1_RESTART) | MODE1_SLEEP;
    write_reg(REG_MODE1, sleep_mode);        // sleep
    write_reg(REG_PRESCALE, prescale);       // set frequency
    write_reg(REG_MODE1, old_mode);          // wake
    ThisThread::sleep_for(5ms);

    // RESTART restores the PWM outputs after the sleep cycle.
    write_reg(REG_MODE1, old_mode | MODE1_RESTART | MODE1_AI);
}

// --- Output --------------------------------------------------------------

void PCA9685::set_pwm(uint8_t channel, uint16_t on, uint16_t off) {
    if (channel > 15) return;

    // Auto-increment lets us push all 4 bytes in a single transaction.
    char buf[5];
    buf[0] = (char)(REG_LED0_ON_L + 4 * channel);
    buf[1] = (char)(on  & 0xFF);
    buf[2] = (char)(on  >> 8);
    buf[3] = (char)(off & 0xFF);
    buf[4] = (char)(off >> 8);
    i2c.write(addr8, buf, 5);
}

void PCA9685::set_pulse_us(uint8_t channel, float pulse_us) {
    // One frame is 4096 ticks long and lasts (1e6 / freq) microseconds.
    float period_us = 1000000.0f / freq;
    float ticks     = (pulse_us / period_us) * 4096.0f;

    if (ticks < 0.0f)      ticks = 0.0f;
    if (ticks > 4095.0f)   ticks = 4095.0f;

    set_pwm(channel, 0, (uint16_t)ticks);
}

void PCA9685::set_off(uint8_t channel) {
    // Bit 12 of OFF_H is the "full off" flag — cleaner than writing 0 ticks.
    set_pwm(channel, 0, 4096);
}