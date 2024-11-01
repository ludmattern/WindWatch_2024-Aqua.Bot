#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include <limits>

class PIDController
{
public:
    PIDController();
	PIDController(double kp, double ki, double kd, double max_output, double min_output, double kp_dynamic_factor, double integral_threshold);
	PIDController &operator=(const PIDController &pid_controller);

    double calculate(double error);
	double calculate(double error, double dynamic_factor);

    void setMultipliers(double kp, double ki, double kd);
	void setMaxOutput(double max_output);

    void reset();

private:
    double kp_, ki_, kd_;
    double max_output_, min_output_;
    double integral_error_, last_error_;
	double kp_dynamic_factor_;
	double integral_threshold_;
    double max_integral_ = 10.0;
};

#endif  // PID_CONTROLLER_HPP_