#include "esc/esc.h"

#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

namespace
{
constexpr uint16_t ESC_MIN_US = 1000;
constexpr uint16_t ESC_MAX_US = 2000;

constexpr uint32_t PWM_COUNTER_FREQUENCY_HZ = 1'000'000;
constexpr uint16_t PWM_PERIOD_US = 20'000;
constexpr uint16_t PWM_WRAP = PWM_PERIOD_US - 1;
}

EscDriver::EscDriver(uint esc1_pin, uint esc2_pin, uint esc3_pin, uint esc4_pin)
    : pins_{esc1_pin, esc2_pin, esc3_pin, esc4_pin}
{
}

void EscDriver::initialise()
{
    for (uint pin : pins_)
    {
        configure_pin(pin);
    }

    write_all_min();
}

void EscDriver::configure_pin(uint pin)
{
     gpio_set_function(pin, GPIO_FUNC_PWM);

    const uint slice = pwm_gpio_to_slice_num(pin);

    const uint32_t system_clock_hz = clock_get_hz(clk_sys);
    const float clock_divider =
        static_cast<float>(system_clock_hz) /
        static_cast<float>(PWM_COUNTER_FREQUENCY_HZ);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, clock_divider);
    pwm_config_set_wrap(&config, PWM_WRAP);

    pwm_init(slice, &config, true);
}

void EscDriver::write_all_us(uint16_t esc1_us,
                             uint16_t esc2_us,
                             uint16_t esc3_us,
                             uint16_t esc4_us)
{
    write_pin_us(pins_[0], esc1_us);
    write_pin_us(pins_[1], esc2_us);
    write_pin_us(pins_[2], esc3_us);
    write_pin_us(pins_[3], esc4_us);
}

void EscDriver::write_all_min()
{
    write_all_us(ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US);
}

void EscDriver::write_pin_us(uint pin, uint16_t pulse_us)
{
    if (pulse_us < ESC_MIN_US)
    {
        pulse_us = ESC_MIN_US;
    }

    if (pulse_us > ESC_MAX_US)
    {
        pulse_us = ESC_MAX_US;
    }

    pwm_set_gpio_level(pin, pulse_us);
}