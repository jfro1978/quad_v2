#include "mixer/motor_mixer.h"

MotorMixer::MotorMixer(uint16_t minimum_us,
                       uint16_t maximum_us)
    : minimum_us_(minimum_us),
      maximum_us_(maximum_us)
{
}

MotorCommands MotorMixer::mix(
    uint16_t throttle_us,
    float pitch_correction_us,
    float roll_correction_us,
    float yaw_correction_us) const
{
    const float throttle = static_cast<float>(throttle_us);

    /*
     * Assumed control conventions:
     *
     * Positive pitch correction:
     *     raise the nose
     *
     * Positive roll correction:
     *     raise the right side / roll left
     *
     * Positive yaw correction:
     *     increase CCW motors and decrease CW motors
     *
     * Motor order:
     *     0 = front-left,  CW
     *     1 = front-right, CCW
     *     2 = rear-right,  CW
     *     3 = rear-left,   CCW
     */

    const float front_left =
        throttle
        - pitch_correction_us
        + roll_correction_us
        - yaw_correction_us;

    const float front_right =
        throttle
        - pitch_correction_us
        - roll_correction_us
        + yaw_correction_us;

    const float rear_right =
        throttle
        + pitch_correction_us
        - roll_correction_us
        - yaw_correction_us;

    const float rear_left =
        throttle
        + pitch_correction_us
        + roll_correction_us
        + yaw_correction_us;

    return MotorCommands{
        .front_left_us  = clamp_motor_command(front_left),
        .front_right_us = clamp_motor_command(front_right),
        .rear_right_us  = clamp_motor_command(rear_right),
        .rear_left_us   = clamp_motor_command(rear_left)
    };
}

uint16_t MotorMixer::clamp_motor_command(float command_us) const
{
    if (command_us < static_cast<float>(minimum_us_))
    {
        return minimum_us_;
    }

    if (command_us > static_cast<float>(maximum_us_))
    {
        return maximum_us_;
    }

    return static_cast<uint16_t>(command_us);
}