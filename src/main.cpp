#include <cstdio>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "imu/mpu6xxx.h"
#include "attitude/complementary_filter.h"
#include "receiver/receiver.h"
#include "esc/esc.h"
#include "mixer/motor_mixer.h"

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

    ComplementaryFilter attitude_filter(0.98f);

    absolute_time_t previous_time = get_absolute_time();

    while (true)
    {
        const MotorCommands motors =
            mixer.mix(
                TEST_THROTTLE_US,
                TEST_PITCH_CORRECTION_US,
                TEST_ROLL_CORRECTION_US,
                TEST_YAW_CORRECTION_US
            );

        printf(
            "FL:%u FR:%u RR:%u RL:%u\n",
            static_cast<unsigned>(motors.front_left_us),
            static_cast<unsigned>(motors.front_right_us),
            static_cast<unsigned>(motors.rear_right_us),
            static_cast<unsigned>(motors.rear_left_us)
        );

        sleep_ms(250);
    }
}