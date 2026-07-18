#include <cstdio>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

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

    constexpr PidController::Gains TEST_PID_GAINS{
        .proportional = 2.0f,
        .integral = 0.0f,
        .derivative = 0.0f
    };

    constexpr float PID_OUTPUT_MIN = -200.0f;
    constexpr float PID_OUTPUT_MAX = 200.0f;

    constexpr float PID_INTEGRAL_MIN = -100.0f;
    constexpr float PID_INTEGRAL_MAX = 100.0f;

    constexpr float TEST_SETPOINT = 0.0f;
    constexpr float TEST_MEASUREMENT = -10.0f;
    constexpr float TEST_DELTA_TIME_SECONDS = 0.004f;

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

    PidController test_pid(
        TEST_PID_GAINS,
        PID_OUTPUT_MIN,
        PID_OUTPUT_MAX,
        PID_INTEGRAL_MIN,
        PID_INTEGRAL_MAX
    );

    ComplementaryFilter attitude_filter(0.98f);

    absolute_time_t previous_time = get_absolute_time();

    while (true)
    {
        const float correction_us =
            test_pid.update(
                TEST_SETPOINT,
                TEST_MEASUREMENT,
                TEST_DELTA_TIME_SECONDS);

        printf(
            "Setpoint: %.2f, measurement: %.2f, correction: %.2f us\n",
            static_cast<double>(TEST_SETPOINT),
            static_cast<double>(TEST_MEASUREMENT),
            static_cast<double>(correction_us));

        escs.write_all_min();

        sleep_ms(250);
    }
}