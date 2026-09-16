#ifndef _PCA9685_HPP
#define _PCA9685_HPP

#include "mbed.h"

// =============================================================
//  PCA9685 16-channel, 12-bit PWM / servo driver (I2C)
//
//  The chip generates all 16 PWM signals itself. The MCU only
//  writes tick counts (0..4095) into registers over I2C — no
//  PwmOut pins are used at all.
//
//  Default I2C address is 0x40 (7-bit). mbed's I2C API wants an
//  8-bit address, so we shift left by 1 internally.
// =============================================================

class PCA9685 {
public:
    // addr7 = 7-bit I2C address (0x40 default, +offset if you bridged
    // the A0..A5 solder jumpers to chain boards).
    PCA9685(PinName sda, PinName scl, uint8_t addr7 = 0x40)
        : i2c(sda, scl), addr8(addr7 << 1) {}

    // Wake the chip, enable register auto-increment, set PWM frequency.
    // Returns false if the chip does not ACK on the bus.
    bool init(float freq_hz = 50.0f);

    // Set the PWM frequency. Datasheet allows 40..1600 Hz.
    // Servos want ~50 Hz. ALL 16 channels share one frequency.
    void set_pwm_freq(float freq_hz);

    // Raw control: when in the 4096-tick frame the output goes high (on)
    // and when it goes low (off). Both 0..4095.
    void set_pwm(uint8_t channel, uint16_t on, uint16_t off);

    // Drive a channel with a servo pulse of `pulse_us` microseconds.
    void set_pulse_us(uint8_t channel, float pulse_us);

    // Stop pulsing a channel entirely (servo goes limp / stops holding).
    void set_off(uint8_t channel);

private:
    I2C     i2c;
    uint8_t addr8;
    float   freq = 50.0f;      // cached, needed to convert us -> ticks

    // Register map (PCA9685 datasheet)
    static const uint8_t REG_MODE1    = 0x00;
    static const uint8_t REG_MODE2    = 0x01;
    static const uint8_t REG_LED0_ON_L = 0x06;   // 4 bytes per channel
    static const uint8_t REG_PRESCALE = 0xFE;

    // MODE1 bits
    static const uint8_t MODE1_RESTART = 0x80;
    static const uint8_t MODE1_AI      = 0x20;   // register auto-increment
    static const uint8_t MODE1_SLEEP   = 0x10;
    static const uint8_t MODE1_ALLCALL = 0x01;

    // Internal oscillator. Nominally 25 MHz, but real parts vary by a few
    // percent. If your servo pulses measure long/short on a scope, tweak
    // this value rather than fudging the pulse widths.
    static constexpr float OSC_CLOCK = 25000000.0f;

    bool write_reg(uint8_t reg, uint8_t value);
    bool read_reg(uint8_t reg, uint8_t& value);
};

#endif