#include "control/pid_controller.h"

PidController::PidController(
    Gains gains,
    float output_min,
    float output_max,
    float integral_min,
    float integral_max)
    : gains_(gains),
      output_min_(output_min),
      output_max_(output_max),
      integral_min_(integral_min),
      integral_max_(integral_max),
      integral_(0.0f),
      previous_error_(0.0f),
      has_previous_error_(false)
{
}

float PidController::update(
    float setpoint,
    float measurement,
    float delta_time_seconds)
{
    if (delta_time_seconds <= 0.0f)
    {
        return 0.0f;
    }

    const float error = setpoint - measurement;

    const float proportional_term =
        gains_.proportional * error;

    integral_ += error * delta_time_seconds;

    integral_ = clamp(
        integral_,
        integral_min_,
        integral_max_);

    const float integral_term =
        gains_.integral * integral_;

    float derivative_term = 0.0f;

    if (has_previous_error_)
    {
        const float error_rate =
            (error - previous_error_) / delta_time_seconds;

        derivative_term =
            gains_.derivative * error_rate;
    }

    previous_error_ = error;
    has_previous_error_ = true;

    const float output =
        proportional_term +
        integral_term +
        derivative_term;

    return clamp(
        output,
        output_min_,
        output_max_);
}

void PidController::reset()
{
    integral_ = 0.0f;
    previous_error_ = 0.0f;
    has_previous_error_ = false;
}

float PidController::clamp(
    float value,
    float minimum,
    float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}