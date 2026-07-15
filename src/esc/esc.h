#pragma once

#include <cstdint>
#include "pico/types.h"

class EscDriver
{
public:
    EscDriver(uint esc1_pin, uint esc2_pin, uint esc3_pin, uint esc4_pin);

    void initialise();

    void write_all_us(uint16_t esc1_us,
                      uint16_t esc2_us,
                      uint16_t esc3_us,
                      uint16_t esc4_us);

    void write_all_min();

    void write_motor_us(uint motor_index, uint16_t pulse_us);

private:
    void configure_pin(uint pin);
    void write_pin_us(uint pin, uint16_t pulse_us);

    uint pins_[4];
};