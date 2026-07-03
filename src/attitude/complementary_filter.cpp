#include "attitude/complementary_filter.h"

#include <cmath>

ComplementaryFilter::ComplementaryFilter(float alpha)
    : alpha_(alpha),
      pitch_deg_(0.0f),
      roll_deg_(0.0f)
{
}

void ComplementaryFilter::reset(float pitch_deg, float roll_deg)
{
    pitch_deg_ = pitch_deg;
    roll_deg_ = roll_deg;
}

AttitudeEstimate ComplementaryFilter::update(const GyroSample& gyro,
                                             const AccelSample& accel,
                                             float dt_seconds)
{
    constexpr float RAD_TO_DEG = 57.2957795f;

    const float accel_pitch_deg =
        std::atan2(accel.y_g, accel.z_g) * RAD_TO_DEG;

    const float accel_roll_deg =
        std::atan2(-accel.x_g,
                   std::sqrt((accel.y_g * accel.y_g) +
                             (accel.z_g * accel.z_g))) * RAD_TO_DEG;

    const float gyro_roll_deg = roll_deg_ + (gyro.y_dps * dt_seconds);
    const float gyro_pitch_deg = pitch_deg_ + (gyro.x_dps * dt_seconds);

    roll_deg_ =
        (alpha_ * gyro_roll_deg) +
        ((1.0f - alpha_) * accel_roll_deg);

    pitch_deg_ =
        (alpha_ * gyro_pitch_deg) +
        ((1.0f - alpha_) * accel_pitch_deg);

    return AttitudeEstimate{
        .pitch_deg = pitch_deg_,
        .roll_deg = roll_deg_,
    };
}