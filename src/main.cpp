#include <cstdio>

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

    constexpr uint ESC1_PIN = 10;
    constexpr uint ESC2_PIN = 11;
    constexpr uint ESC3_PIN = 12;
    constexpr uint ESC4_PIN = 13;

    constexpr uint16_t RECEIVER_MIN_US = 1000;
    constexpr uint16_t RECEIVER_MAX_US = 2000;
    constexpr uint16_t MOTOR_TEST_MAX_US = 1200;

    constexpr uint16_t MOTOR_MIN_US = 1000;
    constexpr uint16_t MOTOR_MAX_US = 2000;

    constexpr float TEST_PITCH_CORRECTION_US = 0.0f;
    constexpr float TEST_ROLL_CORRECTION_US  = 100.0f;
    constexpr float TEST_YAW_CORRECTION_US   = 0.0f;

    constexpr uint16_t TEST_THROTTLE_US = 1400;

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

    constexpr float PID_OUTPUT_MIN_US = -200.0f;
    constexpr float PID_OUTPUT_MAX_US = 200.0f;

    constexpr float PID_INTEGRAL_MIN = -100.0f;
    constexpr float PID_INTEGRAL_MAX = 100.0f;

    constexpr float TEST_TARGET_ROLL_DEGREES = 0.0f;
    constexpr float TEST_TARGET_PITCH_DEGREES = 0.0f;

    constexpr int64_t CONTROL_LOOP_PERIOD_US = 4000;

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
    sleep_ms(5000);

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

    EscDriver escs(ESC1_PIN, ESC2_PIN, ESC3_PIN, ESC4_PIN);
    escs.initialise();

    escs.write_all_min();
    sleep_ms(3000);

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

    ComplementaryFilter attitude_filter(0.98f);

    absolute_time_t previous_loop_time = get_absolute_time();
    absolute_time_t next_loop_time =
    delayed_by_us(previous_loop_time, CONTROL_LOOP_PERIOD_US);

    uint32_t print_counter = 0;

    while (true)
    {
        const absolute_time_t current_time = get_absolute_time();

        const int64_t elapsed_us =
            absolute_time_diff_us(
                previous_loop_time,
                current_time);

        previous_loop_time = current_time;

        const float delta_time_seconds =
            static_cast<float>(elapsed_us) /
            1'000'000.0f;

        ImuSample imu_sample{};

        if (imu.read(imu_sample))
        {
            const AttitudeEstimate attitude =
                attitude_filter.update(
                    imu_sample.gyro,
                    imu_sample.accel,
                    delta_time_seconds);

            const float measured_roll_degrees =
                attitude.roll_deg;

            const float measured_pitch_degrees =
                attitude.pitch_deg;

            const float roll_correction_us =
                roll_pid.update(
                    TEST_TARGET_ROLL_DEGREES,
                    measured_roll_degrees,
                    delta_time_seconds);

            const float pitch_correction_us =
                pitch_pid.update(
                    TEST_TARGET_PITCH_DEGREES,
                    measured_pitch_degrees,
                    delta_time_seconds);

            const MotorCommands motor_commands =
                mixer.mix(
                    TEST_THROTTLE_US,
                    pitch_correction_us,
                    roll_correction_us,
                    TEST_YAW_CORRECTION_US);

            /*
            * Stage 1 safety:
            * Calculate the motor commands, but do not send them to the motors.
            */
            escs.write_all_min();

            ++print_counter;

            if (print_counter >= 25)
            {
                print_counter = 0;

                printf(
                    "dt: %.2f ms | "
                    "roll: %.2f deg, correction: %.2f us | "
                    "pitch: %.2f deg, correction: %.2f us | "
                    "motors FL:%u FR:%u RR:%u RL:%u\n",
                    static_cast<double>(
                        delta_time_seconds * 1000.0f),
                    static_cast<double>(
                        measured_roll_degrees),
                    static_cast<double>(
                        roll_correction_us),
                    static_cast<double>(
                        measured_pitch_degrees),
                    static_cast<double>(
                        pitch_correction_us),
                    motor_commands.front_left_us,
                    motor_commands.front_right_us,
                    motor_commands.rear_right_us,
                    motor_commands.rear_left_us);
            }
        }

        sleep_until(next_loop_time);

        next_loop_time =
            delayed_by_us(
                next_loop_time,
                CONTROL_LOOP_PERIOD_US);
    }

}