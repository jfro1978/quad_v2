#include <cstdio>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "imu/mpu6xxx.h"

namespace
{
constexpr i2c_inst_t* I2C_PORT = i2c0;
constexpr uint I2C_SDA_PIN = 4;
constexpr uint I2C_SCL_PIN = 5;
constexpr uint I2C_BAUDRATE_HZ = 100000;
constexpr uint8_t IMU_ADDRESS = 0x68;

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

    while (true)
    {
        GyroSample gyro{};

        if (imu.read_gyro(gyro))
        {
            printf("Gyro X: %8.3f dps | Y: %8.3f dps | Z: %8.3f dps\n",
                   gyro.x_dps,
                   gyro.y_dps,
                   gyro.z_dps);
        }
        else
        {
            printf("ERROR: Failed to read gyro.\n");
        }

        sleep_ms(250);
    }
}