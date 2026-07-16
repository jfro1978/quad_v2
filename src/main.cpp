#include <cstdio>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "imu/mpu6xxx.h"
#include "attitude/complementary_filter.h"
#include "receiver/receiver.h"
#include "esc/esc.h"

namespace
{
    constexpr i2c_inst_t* I2C_PORT = i2c0;
    constexpr uint I2C_SDA_PIN = 4;
    constexpr uint I2C_SCL_PIN = 5;
    constexpr uint I2C_BAUDRATE_HZ = 100000;

    constexpr uint8_t IMU_ADDRESS = 0x68;

    constexpr uint RECEIVER_CH1_PIN = 6;
    constexpr uint RECEIVER_CH2_PIN = 7;
    constexpr uint RECEIVER_CH3_PIN = 8;
    constexpr uint RECEIVER_CH4_PIN = 9;

    constexpr uint ESC1_PIN = 10;
    constexpr uint ESC2_PIN = 11;
    constexpr uint ESC3_PIN = 12;
    constexpr uint ESC4_PIN = 13;

    constexpr uint16_t RECEIVER_MIN_US = 1000;
    constexpr uint16_t RECEIVER_MAX_US = 2000;
    constexpr uint16_t MOTOR_TEST_MAX_US = 1200;


    void initialise_i2c()
    {
        i2c_init(I2C_PORT, I2C_BAUDRATE_HZ);

        gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
        gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);

        gpio_pull_up(I2C_SDA_PIN);
        gpio_pull_up(I2C_SCL_PIN);
    }
}

int main()
{
    stdio_init_all();
    sleep_ms(5000);

    printf("\nPiQuad MPU-6xxx gyro test\n");

    initialise_i2c();

    Mpu6xxx imu(I2C_PORT, IMU_ADDRESS);

    if (!imu.initialise())
    {
        while (true)
        {
            printf("IMU init failed.\n");
            sleep_ms(1000);
        }
    }

    imu.calibrate_gyro();

    Receiver receiver(
        RECEIVER_CH1_PIN,
        RECEIVER_CH2_PIN,
        RECEIVER_CH3_PIN,
        RECEIVER_CH4_PIN
    );
    receiver.initialise();

    EscDriver escs(ESC1_PIN, ESC2_PIN, ESC3_PIN, ESC4_PIN);
    escs.initialise();

    escs.write_all_min();
    sleep_ms(3000);

    ComplementaryFilter attitude_filter(0.98f);

    absolute_time_t previous_time = get_absolute_time();

    while (true)
    {
        const uint16_t throttle_us = receiver.throttle();

        printf("Throttle: %u us\n",
        static_cast<unsigned>(throttle_us));

        if (receiver.ch3_valid())
        {
            uint16_t motor_command_us = throttle_us;

            if (motor_command_us < RECEIVER_MIN_US)
            {
                motor_command_us = RECEIVER_MIN_US;
            }

            if (motor_command_us > MOTOR_TEST_MAX_US)
            {
                motor_command_us = MOTOR_TEST_MAX_US;
            }

            escs.write_motor_us(0, motor_command_us);

            escs.write_motor_us(1, motor_command_us);
            escs.write_motor_us(2, motor_command_us);
            escs.write_motor_us(3, motor_command_us);

            printf("Motor 1 command: %u us\n",
            static_cast<unsigned>(motor_command_us));
        }
        else
        {
            escs.write_all_min();
            printf("Throttle signal invalid; motors held at minimum.\n");
        }

        sleep_ms(20);
    }
}