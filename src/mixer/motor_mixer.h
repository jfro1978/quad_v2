#pragma once

#include <cstdint>

struct MotorCommands {
    uint16_t front_left_us;
    uint16_t front_right_us;
    uint16_t rear_right_us;
    uint16_t rear_left_us;
};

class MotorMixer
{
    public:
        MotorMixer(uint16_t minimum_us,
                   uint16_t maximum_us);

                   MotorCommands mix(uint16_t throttle_us,
                      float pitch_correction_us,
                      float roll_correction_us,
                      float yaw_correction_us) const;

private:
    uint16_t clamp_motor_command(float command_us) const;

    uint16_t minimum_us_;
    uint16_t maximum_us_;
};
