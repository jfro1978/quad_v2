#include "receiver/receiver.h"

#include "hardware/gpio.h"
#include "pico/stdlib.h"

namespace
{
constexpr uint16_t MIN_VALID_PULSE_US = 900;
constexpr uint16_t MAX_VALID_PULSE_US = 2100;
}

Receiver* Receiver::instance_ = nullptr;

Receiver::Receiver(uint ch1_pin,
                   uint ch2_pin,
                   uint ch3_pin,
                   uint ch4_pin)
    : pins_{ch1_pin, ch2_pin, ch3_pin, ch4_pin},
      rise_time_us_{0, 0, 0, 0},
      pulse_width_us_{1500, 1500, 1000, 1500},
      valid_{false, false, false, false}
{
}

void Receiver::initialise()
{
    instance_ = this;

    for (uint i = 0; i < 4; ++i)
    {
        gpio_init(pins_[i]);
        gpio_set_dir(pins_[i], GPIO_IN);
        gpio_pull_down(pins_[i]);

        gpio_set_irq_enabled(
            pins_[i],
            GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
            true
        );
    }

    gpio_set_irq_callback(&Receiver::gpio_callback);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

uint16_t Receiver::roll() const { return pulse_width_us_[0]; }
uint16_t Receiver::pitch() const { return pulse_width_us_[1]; }
uint16_t Receiver::throttle() const { return pulse_width_us_[2]; }
uint16_t Receiver::yaw() const { return pulse_width_us_[3]; }

bool Receiver::ch1_valid() const { return valid_[0]; }
bool Receiver::ch2_valid() const { return valid_[1]; }
bool Receiver::ch3_valid() const { return valid_[2]; }
bool Receiver::ch4_valid() const { return valid_[3]; }

bool Receiver::all_channels_valid() const
{
    return valid_[0] && valid_[1] && valid_[2] && valid_[3];
}

int Receiver::channel_from_gpio(uint gpio) const
{
    for (int i = 0; i < 4; ++i)
    {
        if (pins_[i] == gpio)
        {
            return i;
        }
    }

    return -1;
}

void Receiver::gpio_callback(uint gpio, uint32_t events)
{
    if (instance_ == nullptr)
    {
        return;
    }

    const int channel = instance_->channel_from_gpio(gpio);

    if (channel < 0)
    {
        return;
    }

    const uint32_t now_us = time_us_32();

    if (events & GPIO_IRQ_EDGE_RISE)
    {
        instance_->rise_time_us_[channel] = now_us;
    }

    if (events & GPIO_IRQ_EDGE_FALL)
    {
        const uint32_t pulse_width =
            now_us - instance_->rise_time_us_[channel];

        instance_->pulse_width_us_[channel] =
            static_cast<uint16_t>(pulse_width);

        instance_->valid_[channel] =
            pulse_width >= MIN_VALID_PULSE_US &&
            pulse_width <= MAX_VALID_PULSE_US;
    }
}