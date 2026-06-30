#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

namespace 
{
    constexpr i2c_inst_t* I2C_PORT = i2c0;
    constexpr uint I2C_SDA_PIN = 4;
    constexpr uint I2C_SCL_PIN = 5;
    constexpr uint I2C_BAUDRATE_HZ = 100000;

    bool is_i2c_device_present(uint8_t address)
    {
        uint8_t dummy = 0;
        const int result = i2c_read_blocking(I2C_PORT, address, &dummy, 1, false);
        return result >= 0;
    }

    void initialise_i2c()
    {
        i2c_init(I2C_PORT, I2C_BAUDRATE_HZ);

        gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
        gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);

        gpio_pull_up(I2C_SDA_PIN);
        gpio_pull_up(I2C_SCL_PIN);
    }

    void scan_i2c_bus()
    {
        printf("\nScanning I2C bus...\n");

        bool found_device = false;

        for (uint8_t address = 0x08; address <= 0x77; ++address)
        {
            if (is_i2c_device_present(address))
            {
                printf("Found I2C device at 0x%02X\n", address);
                found_device = true;
            }
        }

        if (!found_device)
        {
            printf("No I2C devices found.\n");
        }
    }
}

int main()
{
    stdio_init_all();

    sleep_ms(10000);

    printf("\nPiQuad I2C scanner\n");
    printf("I2C0 SDA = GP%u, SCL = GP%u, baud = %u Hz\n",
           I2C_SDA_PIN,
           I2C_SCL_PIN,
           I2C_BAUDRATE_HZ);

    initialise_i2c();

    while (true)
    {
        scan_i2c_bus();
        sleep_ms(2000);
    }
}
