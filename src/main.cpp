#include <cstdio>
#include <algorithm>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/time.h"

#include "imu/mpu6xxx.h"
#include "attitude/complementary_filter.h"
#include "receiver/receiver.h"
#include "esc/esc.h"
#include "mixer/motor_mixer.h"
#include "control/pid_controller.h"

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

    constexpr uint ESC_FRONT_LEFT_PIN  = 10;
    constexpr uint ESC_FRONT_RIGHT_PIN = 11;
    constexpr uint ESC_REAR_RIGHT_PIN  = 12;
    constexpr uint ESC_REAR_LEFT_PIN   = 13;

    constexpr uint16_t RECEIVER_MIN_US = 1000;
    constexpr uint16_t RECEIVER_MAX_US = 2000;
    constexpr uint16_t MOTOR_TEST_MAX_US = 1200;

    constexpr uint16_t MOTOR_MIN_US = 1000;
    constexpr uint16_t MOTOR_MAX_US = 2000;

    constexpr float TEST_PITCH_CORRECTION_US = 0.0f;
    constexpr float TEST_ROLL_CORRECTION_US  = 100.0f;
    constexpr float TEST_YAW_CORRECTION_US   = 0.0f;

    constexpr uint16_t TEST_THROTTLE_US = 1200;

    // Arming constants
    constexpr uint16_t ARM_THROTTLE_MAX_US = 1050;
    constexpr uint16_t ARM_YAW_RIGHT_MIN_US = 1900;
    constexpr uint16_t DISARM_YAW_LEFT_MAX_US = 1100;
    constexpr int64_t ARM_HOLD_TIME_US = 1'000'000;

    // Receiver centre deadband.
    constexpr uint16_t YAW_DEADBAND_US = 10;

    constexpr PidController::Gains ROLL_PID_GAINS{
        .proportional = 2.0f,
        .integral = 0.0f,
        .derivative = 0.0f
    };

    constexpr PidController::Gains PITCH_PID_GAINS{
        .proportional = 2.0f,
        .integral = 0.0f,
        .derivative = 0.0f
    };

    constexpr PidController::Gains YAW_PID_GAINS{
        .proportional = 1.0f,
        .integral = 0.0f,
        .derivative = 0.0f
    };

    constexpr float PID_OUTPUT_MIN_US = -200.0f;
    constexpr float PID_OUTPUT_MAX_US = 200.0f;

    constexpr float PID_INTEGRAL_MIN = -100.0f;
    constexpr float PID_INTEGRAL_MAX = 100.0f;

    constexpr float MAX_ROLL_ANGLE_DEG = 20.0f;
    constexpr float MAX_PITCH_ANGLE_DEG = 20.0f;
    
    // Maximum commanded yaw rotation rate.
    constexpr float MAX_YAW_RATE_DPS = 90.0f;

    constexpr float RECEIVER_MID_US = 1500.0f;
    constexpr float RECEIVER_HALF_RANGE_US = 500.0f;

    constexpr int64_t CONTROL_LOOP_PERIOD_US = 4000;

    void initialise_i2c()
    {
        i2c_init(I2C_PORT, I2C_BAUDRATE_HZ);

        gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
        gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);

        gpio_pull_up(I2C_SDA_PIN);
        gpio_pull_up(I2C_SCL_PIN);
    }

    float receiver_to_angle(uint16_t pulse_us, float max_angle_deg)
    {
        const float normalised =
            (static_cast<float>(pulse_us) - RECEIVER_MID_US) /
            (RECEIVER_MAX_US - RECEIVER_MID_US);

        return std::clamp(normalised, -1.0f, 1.0f) * max_angle_deg;
    }

    float receiver_to_yaw_rate(uint16_t pulse_us)
    {
        const float offset_us =
            static_cast<float>(pulse_us) - RECEIVER_MID_US;

        // Ignore small deviations around the receiver centre.
        if (offset_us >= -static_cast<float>(YAW_DEADBAND_US) &&
            offset_us <= static_cast<float>(YAW_DEADBAND_US))
        {
            return 0.0f;
        }

        const float normalised =
            offset_us / RECEIVER_HALF_RANGE_US;

        return std::clamp(normalised, -1.0f, 1.0f) *
            MAX_YAW_RATE_DPS;
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

    EscDriver escs(
        ESC_FRONT_LEFT_PIN,
        ESC_FRONT_RIGHT_PIN,
        ESC_REAR_RIGHT_PIN,
        ESC_REAR_LEFT_PIN
    );

    escs.initialise();

    escs.write_all_min();
    printf("Apply LiPo battery power now.\n");
    sleep_ms(10000);

    MotorMixer mixer(MOTOR_MIN_US, MOTOR_MAX_US);

    PidController roll_pid(
        ROLL_PID_GAINS,
        PID_OUTPUT_MIN_US,
        PID_OUTPUT_MAX_US,
        PID_INTEGRAL_MIN,
        PID_INTEGRAL_MAX);

    PidController pitch_pid(
        PITCH_PID_GAINS,
        PID_OUTPUT_MIN_US,
        PID_OUTPUT_MAX_US,
        PID_INTEGRAL_MIN,
        PID_INTEGRAL_MAX);

    PidController yaw_pid(
        YAW_PID_GAINS,
        PID_OUTPUT_MIN_US,
        PID_OUTPUT_MAX_US,
        PID_INTEGRAL_MIN,
        PID_INTEGRAL_MAX);

    ComplementaryFilter attitude_filter(0.98f);

    absolute_time_t previous_loop_time = get_absolute_time();
    absolute_time_t next_loop_time =
    delayed_by_us(previous_loop_time, CONTROL_LOOP_PERIOD_US);

    bool armed = false;

    absolute_time_t arm_start_time = nil_time;
    absolute_time_t disarm_start_time = nil_time;

    while (true)
    {
        // Calculate the elapsed time since the previous control-loop iteration.
        const absolute_time_t current_time = get_absolute_time();

        const int64_t elapsed_us =
            absolute_time_diff_us(previous_loop_time, current_time);

        previous_loop_time = current_time;

        const float delta_time_seconds =
            static_cast<float>(elapsed_us) / 1'000'000.0f;

        // Read the transmitter inputs.
        const uint16_t throttle_us = receiver.throttle();
        const uint16_t yaw_us = receiver.yaw();
        const uint16_t roll_us = receiver.roll();
        const uint16_t pitch_us = receiver.pitch();

        // Check that all receiver channels are valid.
        if (!receiver.all_channels_valid())
        {
            armed = false;

            arm_start_time = nil_time;
            disarm_start_time = nil_time;

            escs.write_all_min();
        }
        else
        {
            // Determine whether the throttle is fully down.
            const bool throttle_low =
                throttle_us <= ARM_THROTTLE_MAX_US;

            // Arming: throttle down, yaw fully right.
            const bool arm_requested =
                throttle_low &&
                yaw_us >= ARM_YAW_RIGHT_MIN_US;

            // Disarming: throttle down, yaw fully left.
            const bool disarm_requested =
                throttle_low &&
                yaw_us <= DISARM_YAW_LEFT_MAX_US;

            if (!armed)
            {
                // Keep the motors stopped while disarmed.
                escs.write_all_min();

                if (arm_requested)
                {
                    if (is_nil_time(arm_start_time))
                    {
                        arm_start_time = current_time;
                    }
                    else if (
                        absolute_time_diff_us(
                            arm_start_time,
                            current_time) >= ARM_HOLD_TIME_US)
                    {
                        armed = true;
                        arm_start_time = nil_time;

                        printf("ARMED\n");
                    }
                }
                else
                {
                    arm_start_time = nil_time;
                }
            }
            else
            {
                // Check whether the operator is requesting disarming.
                if (disarm_requested)
                {
                    if (is_nil_time(disarm_start_time))
                    {
                        disarm_start_time = current_time;
                    }
                    else if (
                        absolute_time_diff_us(
                            disarm_start_time,
                            current_time) >= ARM_HOLD_TIME_US)
                    {
                        armed = false;
                        disarm_start_time = nil_time;

                        escs.write_all_min();

                        printf("DISARMED\n");
                    }
                }
                else
                {
                    disarm_start_time = nil_time;
                }

                // Only calculate and apply motor commands while armed.
                if (armed)
                {
                    ImuSample imu_sample{};

                    if (imu.read(imu_sample))
                    {
                        // Estimate the current roll and pitch angles.
                        const AttitudeEstimate attitude =
                            attitude_filter.update(
                                imu_sample.gyro,
                                imu_sample.accel,
                                delta_time_seconds);

                        // Convert transmitter roll/pitch commands
                        // into desired attitude angles.
                        //
                        // 1000 us -> -20 degrees
                        // 1500 us ->   0 degrees
                        // 2000 us -> +20 degrees

                        // ----------------------------------------------------
                        // Convert receiver inputs into target angles/rates.
                        // ----------------------------------------------------

                        // Desired roll angle.
                        const float target_roll_deg =
                            receiver_to_angle(
                                roll_us,
                                MAX_ROLL_ANGLE_DEG);

                        // Desired pitch angle.
                        const float target_pitch_deg =
                            receiver_to_angle(
                                pitch_us,
                                MAX_PITCH_ANGLE_DEG);

                        // Desired yaw rotation rate.
                        const float target_yaw_rate_dps =
                            receiver_to_yaw_rate(yaw_us);

                        // ----------------------------------------------------
                        // Calculate PID corrections.
                        // ----------------------------------------------------

                        // Roll correction.
                        const float roll_correction_us =
                            roll_pid.update(
                                target_roll_deg,
                                attitude.roll_deg,
                                delta_time_seconds);

                        // Pitch correction.
                        const float pitch_correction_us =
                            pitch_pid.update(
                                target_pitch_deg,
                                attitude.pitch_deg,
                                delta_time_seconds);

                        // Yaw correction.
                        //
                        // Unlike roll and pitch, yaw uses the measured
                        // angular velocity directly from the gyroscope.
                        const float yaw_correction_us =
                            yaw_pid.update(
                                target_yaw_rate_dps,
                                imu_sample.gyro.z_dps,
                                delta_time_seconds);

                        // ----------------------------------------------------
                        // Calculate individual motor commands.
                        // ----------------------------------------------------

                        const MotorCommands motor_commands =
                            mixer.mix(
                                throttle_us,
                                pitch_correction_us,
                                roll_correction_us,
                                yaw_correction_us);

                        // ----------------------------------------------------
                        // Apply motor commands.
                        // ----------------------------------------------------

                        if (throttle_us <= ARM_THROTTLE_MAX_US)
                        {
                            escs.write_all_min();
                        }
                        else
                        {
                            escs.write_all_us(
                                motor_commands.front_left_us,
                                motor_commands.front_right_us,
                                motor_commands.rear_right_us,
                                motor_commands.rear_left_us);
                        }
                    }
                    else
                    {
                        // An IMU communication failure causes
                        // the quadcopter to disarm.
                        armed = false;

                        arm_start_time = nil_time;
                        disarm_start_time = nil_time;

                        roll_pid.reset();
                        pitch_pid.reset();
                        yaw_pid.reset();

                        escs.write_all_min();

                        printf("IMU failure - DISARMED\n");
                    }
                }
            }
        }

        // Maintain the 250 Hz control-loop frequency.
        sleep_until(next_loop_time);

        next_loop_time =
            delayed_by_us(
                next_loop_time,
                CONTROL_LOOP_PERIOD_US);
    }
}
