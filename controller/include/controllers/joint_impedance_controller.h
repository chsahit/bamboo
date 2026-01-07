// Joint Impedance Controller

#ifndef BAMBOO_JOINT_IMPEDANCE_CONTROLLER_H
#define BAMBOO_JOINT_IMPEDANCE_CONTROLLER_H

#include <Eigen/Dense>
#include <array>
#include <franka/model.h>
#include <franka/robot_state.h>

namespace bamboo {
namespace controllers {

struct ControllerResult {
  std::array<double, 7> torques;
  bool torque_limit_violated;
};

class JointImpedanceController {
public:
  JointImpedanceController(franka::Model *model);

  // Returns torques and a boolean indicating if any joint limit was violated
  ControllerResult Step(const franka::RobotState &robot_state,
                        const Eigen::Matrix<double, 7, 1> &q_desired,
                        const Eigen::Matrix<double, 7, 1> &dq_desired =
                            Eigen::Matrix<double, 7, 1>::Zero(),
                        const Eigen::Matrix<double, 7, 1> &ddq_desired =
                            Eigen::Matrix<double, 7, 1>::Zero());

  // Set controller gains (optional - uses defaults if not called)
  void SetGains(const std::array<double, 7> &kp,
                const std::array<double, 7> &kd);

private:
  franka::Model *model_;

  // Controller gains
  Eigen::Matrix<double, 7, 1> Kp_;
  Eigen::Matrix<double, 7, 1> Kd_;

  // Torque limits per joint
  Eigen::Matrix<double, 7, 1> joint_tau_limits_;

  // Simple exponential smoothing for velocity estimation
  bool first_state_;
  Eigen::Matrix<double, 7, 1> smoothed_q_;
  Eigen::Matrix<double, 7, 1> smoothed_dq_;
  double alpha_q_;
  double alpha_dq_;
};

} // namespace controllers
} // namespace bamboo

#endif // BAMBOO_JOINT_IMPEDANCE_CONTROLLER_H
