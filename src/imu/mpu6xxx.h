#pragma once

#include <cstdint>

#include "hardware/i2c.h"

struct GyroSample
{
    float x_dps;
    float y_dps;
    float z_dps;
};

struct AccelSample
{
    float x_g;
    float y_g;
    float z_g;
};

struct ImuSample
{
    GyroSample gyro;
    AccelSample accel;
};

class Mpu6xxx
{
public:
    Mpu6xxx(i2c_inst_t* i2c, uint8_t address);

    bool initialise();
    void calibrate_gyro(uint16_t sample_count = 1000);
    bool read_gyro(GyroSample& sample);    
    bool read_accel(AccelSample& sample);
    bool read(ImuSample& sample);

private:
    bool write_register(uint8_t reg, uint8_t value);
    bool read_register(uint8_t reg, uint8_t& value);
    bool read_registers(uint8_t start_reg, uint8_t* buffer, uint8_t length);

    float gyro_x_offset_dps_ = 0.0f;
    float gyro_y_offset_dps_ = 0.0f;
    float gyro_z_offset_dps_ = 0.0f;

    static int16_t read_i16_be(const uint8_t* data);

    i2c_inst_t* i2c_;
    uint8_t address_;
};