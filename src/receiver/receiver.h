#pragma once

#include <cstdint>
#include "pico/types.h"

class Receiver
{
public:
    Receiver(uint ch1_pin,
             uint ch2_pin,
             uint ch3_pin,
             uint ch4_pin);

    void initialise();

    uint16_t roll() const; 
    uint16_t pitch() const; 
    uint16_t throttle() const; 
    uint16_t yaw() const; 

    bool ch1_valid() const;
    bool ch2_valid() const;
    bool ch3_valid() const;
    bool ch4_valid() const;

    bool all_channels_valid() const;

private:
    static void gpio_callback(uint gpio, uint32_t events);

    int channel_from_gpio(uint gpio) const;

    static Receiver* instance_;

    uint pins_[4];

    volatile uint32_t rise_time_us_[4];
    volatile uint16_t pulse_width_us_[4];
    volatile bool valid_[4];

    
};