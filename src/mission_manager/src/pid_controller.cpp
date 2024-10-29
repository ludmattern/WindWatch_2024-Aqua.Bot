// src/pid_controller.cpp

#include "mission_manager/pid_controller.hpp"

namespace mission_manager {

PIDController::PIDController(double kp, double ki, double kd)
: kp_(kp), ki_(ki), kd_(kd), previous_error_(0.0), integral_(0.0)
{}

void PIDController::set_parameters(double kp, double ki, double kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

double PIDController::compute(double setpoint, double measured, double dt)
{
    double error = setpoint - measured;
    integral_ += error * dt;
    double derivative = (error - previous_error_) / dt;
    previous_error_ = error;
    return kp_ * error + ki_ * integral_ + kd_ * derivative;
}

void PIDController::reset()
{
    previous_error_ = 0.0;
    integral_ = 0.0;
}

} // namespace mission_manager
