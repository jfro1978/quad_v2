#pragma once

#include <cstdint>

#include "hardware/i2c.h"

struct GyroSample
{
    float x_dps;
    float y_dps;
    float z_dps;
};

class Mpu6xxx
{
public:
    Mpu6xxx(i2c_inst_t* i2c, uint8_t address);

    bool initialise();
    bool read_gyro(GyroSample& sample);

private:
    bool write_register(uint8_t reg, uint8_t value);
    bool read_register(uint8_t reg, uint8_t& value);
    bool read_registers(uint8_t start_reg, uint8_t* buffer, uint8_t length);

    static int16_t read_i16_be(const uint8_t* data);

    i2c_inst_t* i2c_;
    uint8_t address_;
};