#ifndef MISSION_MANAGER__PID_CONTROLLER_HPP_
#define MISSION_MANAGER__PID_CONTROLLER_HPP_

class PIDController
{
public:
    PIDController(); // Default constructor
    PIDController(double kp, double ki, double kd);
    double compute(double setpoint, double measured_value, double dt);
    void reset();
    void set_parameters(double kp, double ki, double kd); // Added setter for parameters

private:
    double kp_;
    double ki_;
    double kd_;
    double integral_;
    double prev_error_;
};

#endif  // MISSION_MANAGER__PID_CONTROLLER_HPP_
