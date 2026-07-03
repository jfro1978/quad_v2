#pragma once

#include "imu/mpu6xxx.h"

struct AttitudeEstimate
{
    float pitch_deg;
    float roll_deg;
};

class ComplementaryFilter
{
public:
    explicit ComplementaryFilter(float alpha = 0.98f);

    void reset(float pitch_deg = 0.0f, float roll_deg = 0.0f);

    AttitudeEstimate update(const GyroSample& gyro,
                            const AccelSample& accel,
                            float dt_seconds);

private:
    float alpha_;
    float pitch_deg_;
    float roll_deg_;
};