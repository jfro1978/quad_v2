#include <cstdio>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "imu/mpu6xxx.h"
#include "attitude/complementary_filter.h"
#include "receiver/receiver.h"

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
    sleep_ms(10000);

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

    printf("Receiver CH1 on GP%u\n", RECEIVER_CH1_PIN);

    ComplementaryFilter attitude_filter(0.98f);

    absolute_time_t previous_time = get_absolute_time();

    while (true)
    {
        GyroSample gyro{};
        AccelSample accel{};

        const bool gyro_ok = imu.read_gyro(gyro);
        const bool accel_ok = imu.read_accel(accel);

        absolute_time_t now = get_absolute_time();
        const float dt_seconds =
        absolute_time_diff_us(previous_time, now) / 1000000.0f;
        previous_time = now;

        if (gyro_ok && accel_ok)
        {
            const AttitudeEstimate attitude =
                attitude_filter.update(gyro, accel, dt_seconds);

            printf("CH1:%4u CH2:%4u CH3:%4u CH4:%4u valid=%d\n",
                receiver.roll(),
                receiver.pitch(),
                receiver.throttle(),
                receiver.yaw(),
                receiver.all_channels_valid()
            );
            
        }
        else
        {
            printf("ERROR: sensor read failed. gyro_ok=%d accel_ok=%d\n",
                gyro_ok,
                accel_ok);
        }

        sleep_ms(20);
    }
}