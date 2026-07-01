#include "imu/mpu6xxx.h"

#include <cstdio>

#include "pico/stdlib.h"

namespace
{
constexpr uint8_t WHO_AM_I     = 0x75;
constexpr uint8_t PWR_MGMT_1   = 0x6B;
constexpr uint8_t GYRO_CONFIG  = 0x1B;
constexpr uint8_t GYRO_XOUT_H  = 0x43;

constexpr float GYRO_LSB_PER_DPS_250 = 131.0f;
}

Mpu6xxx::Mpu6xxx(i2c_inst_t* i2c, uint8_t address)
    : i2c_(i2c),
      address_(address)
{
}

void Mpu6xxx::calibrate_gyro(uint16_t sample_count)
{
    printf("Calibrating gyro. Keep IMU still...\n");

    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;

    for (uint16_t i = 0; i < sample_count; ++i)
    {
        GyroSample sample{};

        if (read_gyro(sample))
        {
            sum_x += sample.x_dps;
            sum_y += sample.y_dps;
            sum_z += sample.z_dps;
        }

        sleep_ms(2);
    }

    gyro_x_offset_dps_ = sum_x / sample_count;
    gyro_y_offset_dps_ = sum_y / sample_count;
    gyro_z_offset_dps_ = sum_z / sample_count;

    printf("Gyro offsets: X=%.3f, Y=%.3f, Z=%.3f dps\n",
           gyro_x_offset_dps_,
           gyro_y_offset_dps_,
           gyro_z_offset_dps_);
}

bool Mpu6xxx::initialise()
{
    uint8_t who = 0;

    if (!read_register(WHO_AM_I, who))
    {
        printf("ERROR: Failed to read WHO_AM_I.\n");
        return false;
    }

    printf("IMU WHO_AM_I = 0x%02X\n", who);

    if (who != 0x68 && who != 0x70)
    {
        printf("ERROR: Unsupported MPU device.\n");
        return false;
    }

    // Wake device. 0x00 selects internal oscillator and clears sleep bit.
    if (!write_register(PWR_MGMT_1, 0x00))
    {
        printf("ERROR: Failed to wake IMU.\n");
        return false;
    }

    sleep_ms(100);

    // Gyro range +/-250 deg/s.
    if (!write_register(GYRO_CONFIG, 0x00))
    {
        printf("ERROR: Failed to configure gyro.\n");
        return false;
    }

    printf("MPU-6xxx initialised.\n");
    return true;
}

bool Mpu6xxx::read_gyro(GyroSample& sample)
{
    uint8_t data[6] = {};

    if (!read_registers(GYRO_XOUT_H, data, sizeof(data)))
    {
        return false;
    }

    const int16_t raw_x = read_i16_be(&data[0]);
    const int16_t raw_y = read_i16_be(&data[2]);
    const int16_t raw_z = read_i16_be(&data[4]);

    sample.x_dps = (static_cast<float>(raw_x) / GYRO_LSB_PER_DPS_250) - gyro_x_offset_dps_;
    sample.y_dps = (static_cast<float>(raw_y) / GYRO_LSB_PER_DPS_250) - gyro_y_offset_dps_;
    sample.z_dps = (static_cast<float>(raw_z) / GYRO_LSB_PER_DPS_250) - gyro_z_offset_dps_;
    
    return true;
}

bool Mpu6xxx::write_register(uint8_t reg, uint8_t value)
{
    const uint8_t data[2] = {reg, value};
    return i2c_write_blocking(i2c_, address_, data, 2, false) == 2;
}

bool Mpu6xxx::read_register(uint8_t reg, uint8_t& value)
{
    if (i2c_write_blocking(i2c_, address_, &reg, 1, true) != 1)
    {
        return false;
    }

    return i2c_read_blocking(i2c_, address_, &value, 1, false) == 1;
}

bool Mpu6xxx::read_registers(uint8_t start_reg, uint8_t* buffer, uint8_t length)
{
    if (i2c_write_blocking(i2c_, address_, &start_reg, 1, true) != 1)
    {
        return false;
    }

    return i2c_read_blocking(i2c_, address_, buffer, length, false) == length;
}

int16_t Mpu6xxx::read_i16_be(const uint8_t* data)
{
    return static_cast<int16_t>(
        (static_cast<uint16_t>(data[0]) << 8) | data[1]
    );
}