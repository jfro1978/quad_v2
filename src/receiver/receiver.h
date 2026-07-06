#pragma once

#include <cstdint>
#include "pico/types.h"

class Receiver
{
public:
    explicit Receiver(uint32_t gpio_pin);

    void initialise();

    uint16_t pulse_width_us() const;
    bool signal_valid() const;

private:
    static void gpio_callback(uint gpio, uint32_t events);

    uint32_t gpio_pin_;
};