#pragma once

class PidController
{
public:
    struct Gains
    {
        float proportional;
        float integral;
        float derivative;
    };

    PidController(
        Gains gains,
        float output_min,
        float output_max,
        float integral_min,
        float integral_max);

    float update(
        float setpoint,
        float measurement,
        float delta_time_seconds);

    void reset();

private:
    static float clamp(
        float value,
        float minimum,
        float maximum);

    Gains gains_;

    float output_min_;
    float output_max_;

    float integral_min_;
    float integral_max_;

    float integral_;
    float previous_error_;

    bool has_previous_error_;
};