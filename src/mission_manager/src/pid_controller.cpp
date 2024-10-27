#include "mission_manager/pid_controller.hpp"

PIDController::PIDController()
	: kp_(0.0), ki_(0.0), kd_(0.0), integral_(0.0), prev_error_(0.0)
{}

PIDController::PIDController(double kp, double ki, double kd)
	: kp_(kp), ki_(ki), kd_(kd), integral_(0.0), prev_error_(0.0)
{}

double PIDController::compute(double setpoint, double measured_value, double dt)
{
	double error = setpoint - measured_value;
	integral_ += error * dt;
	double derivative = (error - prev_error_) / dt;
	prev_error_ = error;

	double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
	return output;
}

void PIDController::reset()
{
	integral_ = 0.0;
	prev_error_ = 0.0;
}

void PIDController::set_parameters(double kp, double ki, double kd)
{
	kp_ = kp;
	ki_ = ki;
	kd_ = kd;
}
