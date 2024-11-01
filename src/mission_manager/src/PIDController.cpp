// src/PIDController.cpp

#include "mission_manager/PIDController.hpp"
#include <cmath>

PIDController::PIDController()
	: kp_(0.0),
	ki_(0.0),
	kd_(0.0),
	max_output_(0.0),
	min_output_(0.0),
	kp_dynamic_factor_(0.0),
	integral_threshold_(0.0),
	integral_error_(0.0),
	last_error_(0.0),
	max_integral_(10.0)
{}

PIDController::PIDController(double kp, double ki, double kd, double max_output, double min_output, double kp_dynamic_factor, double integral_threshold)
	: kp_(kp),
	ki_(ki),
	kd_(kd),
	max_output_(max_output),
	min_output_(min_output),
	kp_dynamic_factor_(kp_dynamic_factor),
	integral_threshold_(integral_threshold),
	integral_error_(0.0),
	last_error_(0.0),
	max_integral_(10.0)
{}

PIDController &PIDController::operator=(const PIDController &pid_controller)
{
	kp_ = pid_controller.kp_;
	ki_ = pid_controller.ki_;
	kd_ = pid_controller.kd_;
	max_output_ = pid_controller.max_output_;
	min_output_ = pid_controller.min_output_;
	kp_dynamic_factor_ = pid_controller.kp_dynamic_factor_;
	integral_threshold_ = pid_controller.integral_threshold_;
	integral_error_ = pid_controller.integral_error_;
	last_error_ = pid_controller.last_error_;
	max_integral_ = pid_controller.max_integral_;

	return *this;
}

double PIDController::calculate(double error)
{
	if (std::abs(error) > integral_threshold_)
	{
		integral_error_ += error;
		if (integral_error_ > max_integral_) integral_error_ = max_integral_;
		else if (integral_error_ < -max_integral_) integral_error_ = -max_integral_;
	}

	double proportional = kp_ * error;

	double integral = ki_ * integral_error_;

	double derivative = kd_ * (error - last_error_);
	last_error_ = error;

	double output = proportional + integral + derivative;

	if (output > max_output_)
		output = max_output_;
	else if (output < min_output_)
		output = min_output_;

	return output;
}

double PIDController::calculate(double error, double dynamic_factor)
{
	if (std::abs(error) > integral_threshold_)
	{
		integral_error_ += error;
		if (integral_error_ > max_integral_) integral_error_ = max_integral_;
		else if (integral_error_ < -max_integral_) integral_error_ = -max_integral_;
	}

	double kp_dynamic = (dynamic_factor < 20) ? kp_ * kp_dynamic_factor_ : kp_;
	double proportional = kp_dynamic * error;

	double integral = ki_ * integral_error_;

	double derivative = kd_ * (error - last_error_);
	last_error_ = error;

	double output = proportional + integral + derivative;

	if (output > max_output_)
		output = max_output_;
	else if (output < min_output_)
		output = min_output_;

	return output;
}

void PIDController::setMultipliers(double kp, double ki, double kd)
{
	kp_ = kp;
	ki_ = ki;
	kd_ = kd;
}

void PIDController::reset()
{
	integral_error_ = 0.0;
	last_error_ = 0.0;
}
