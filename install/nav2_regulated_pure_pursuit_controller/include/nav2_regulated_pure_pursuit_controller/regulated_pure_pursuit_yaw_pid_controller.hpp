#ifndef REGULATED_PURE_PURSUIT_YAW_PID_CONTROLLER_HPP_
#define REGULATED_PURE_PURSUIT__YAW_PID_CONTROLLER_HPP_
#include <chrono>

class PIDController {
public:
  PIDController(double kp, double ki, double kd);
  double compute(double setpoint, double actual);

private:
  double kp_, ki_, kd_;
  double prev_error_;
  double integral_;
  std::chrono::steady_clock::time_point last_time_;
};

#endif  // PID_CONTROLLER_HPP_
