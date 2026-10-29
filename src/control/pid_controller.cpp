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
      previous_measurement_(0.0f),
      has_previous_measurement_(false)
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

    // -------------------------------------------------
    // Proportional term
    // -------------------------------------------------

    const float proportional_term =
        gains_.proportional * error;

    // -------------------------------------------------
    // Derivative term
    // -------------------------------------------------

    float derivative_term = 0.0f;

    if (has_previous_measurement_)
    {
        const float measurement_rate =
            (measurement - previous_measurement_) /
            delta_time_seconds;

        derivative_term =
            -gains_.derivative * measurement_rate;
    }

    previous_measurement_ = measurement;
    has_previous_measurement_ = true;

    // -------------------------------------------------
    // Integral term with anti-windup
    // -------------------------------------------------

    // Calculate what the integral accumulator would
    // become if we accepted the current error.
    const float candidate_integral = clamp(
        integral_ + error * delta_time_seconds,
        integral_min_,
        integral_max_);



    // Calculate the PID output using the proposed
    // integral accumulator.
    const float candidate_output =
        proportional_term +
        gains_.integral * candidate_integral +
        derivative_term;

    // Determine whether the output would be saturated.
    const bool saturated_high =
        candidate_output > output_max_;

    const bool saturated_low =
        candidate_output < output_min_;

    // Only accept the new integral value if it would
    // not drive the output further into saturation.
    //
    // Assumes a non-negative integral gain.
    const bool allow_integration =
        !(saturated_high && error > 0.0f) &&
        !(saturated_low && error < 0.0f);

    if (allow_integration)
    {
        integral_ = candidate_integral;
    }

    // -------------------------------------------------
    // Calculate final PID output
    // -------------------------------------------------

    const float output =
        proportional_term +
        gains_.integral * integral_ +
        derivative_term;

    return clamp(
        output,
        output_min_,
        output_max_);
}

void PidController::reset()
{
    integral_ = 0.0f;
    previous_measurement_ = 0.0f;
    has_previous_measurement_ = false;
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