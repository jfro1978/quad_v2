#include "receiver/receiver.h"

#include "pico/stdlib.h"
#include "hardware/gpio.h"

namespace
{
constexpr uint16_t MIN_VALID_PULSE_US = 900;
constexpr uint16_t MAX_VALID_PULSE_US = 2100;

volatile uint32_t rising_edge_time_us = 0;
volatile uint16_t latest_pulse_width_us = 0;
volatile bool latest_pulse_valid = false;
}

Receiver::Receiver(uint32_t gpio_pin)
    : gpio_pin_(gpio_pin)
{
}

void Receiver::initialise()
{
    gpio_init(gpio_pin_);
    gpio_set_dir(gpio_pin_, GPIO_IN);
    gpio_pull_down(gpio_pin_);

    gpio_set_irq_enabled_with_callback(
        gpio_pin_,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        &Receiver::gpio_callback
    );
}

uint16_t Receiver::pulse_width_us() const
{
    return latest_pulse_width_us;
}

bool Receiver::signal_valid() const
{
    return latest_pulse_valid;
}

void Receiver::gpio_callback(uint gpio, uint32_t events)
{
    (void)gpio;

    const uint32_t now_us = time_us_32();

    if (events & GPIO_IRQ_EDGE_RISE)
    {
        rising_edge_time_us = now_us;
    }

    if (events & GPIO_IRQ_EDGE_FALL)
    {
        const uint32_t pulse_width = now_us - rising_edge_time_us;

        latest_pulse_width_us = static_cast<uint16_t>(pulse_width);

        latest_pulse_valid =
            pulse_width >= MIN_VALID_PULSE_US &&
            pulse_width <= MAX_VALID_PULSE_US;
    }
}