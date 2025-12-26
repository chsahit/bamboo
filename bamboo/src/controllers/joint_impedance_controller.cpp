// Joint Impedance Controller Implementation

#include "controllers/joint_impedance_controller.h"
#include <iostream>

namespace bamboo {
namespace controllers {

JointImpedanceController::JointImpedanceController(franka::Model *model)
    : model_(model), first_state_(true), alpha_q_(1.0), alpha_dq_(0.95) {

  Kp_ << 875.0, 1050.0, 1050.0, 875.0, 175.0, 350.0, 87.5;
  Kd_ << 37.5, 50.0, 37.5, 25.0, 5.0, 3.75, 2.5;

  joint_max_ << 2.8978, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
  joint_min_ << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;

  joint_tau_limits_ << 60.0, 60.0, 60.0, 60.0, 30.0, 15.0, 15.0;

  smoothed_q_.setZero();
  smoothed_dq_.setZero();
}

void JointImpedanceController::SetGains(const std::array<double, 7> &kp,
                                        const std::array<double, 7> &kd) {
  Kp_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(kp.data());
  Kd_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(kd.data());
}

ControllerResult
JointImpedanceController::Step(const franka::RobotState &robot_state,
                               const Eigen::Matrix<double, 7, 1> &desired_q,
                               const Eigen::Matrix<double, 7, 1> &desired_dq,
                               const Eigen::Matrix<double, 7, 1> &desired_ddq) {

  // Check for joint position limit violations
  bool joint_position_limit_violated = false;
  for (int i = 0; i < 7; i++) {
    if (desired_q[i] > joint_max_[i]) {
      std::cout << "[JOINT_POSITION_LIMIT] Joint " << i
                << " desired position exceeds upper limit: " << desired_q[i]
                << " > " << joint_max_[i] << " rad" << std::endl;
      joint_position_limit_violated = true;
    } else if (desired_q[i] < joint_min_[i]) {
      std::cout << "[JOINT_POSITION_LIMIT] Joint " << i
                << " desired position exceeds lower limit: " << desired_q[i]
                << " < " << joint_min_[i] << " rad" << std::endl;
      joint_position_limit_violated = true;
    }
  }

  // Get dynamics from Franka model and copy to Eigen objects
  const std::array<double, 49> mass_array = model_->mass(robot_state);
  Eigen::Matrix<double, 7, 7> M = Eigen::MatrixXd::Map(mass_array.data(), 7, 7);

  const std::array<double, 7> coriolis_array = model_->coriolis(robot_state);
  Eigen::Matrix<double, 7, 1> coriolis =
      Eigen::VectorXd::Map(coriolis_array.data(), 7);

  // Get current state and copy to Eigen objects
  Eigen::Matrix<double, 7, 1> q = Eigen::VectorXd::Map(robot_state.q.data(), 7);
  Eigen::Matrix<double, 7, 1> dq =
      Eigen::VectorXd::Map(robot_state.dq.data(), 7);

  Eigen::Matrix<double, 7, 1> current_q, current_dq;

  if (first_state_) {
    smoothed_q_ = q;
    smoothed_dq_ = dq;
    first_state_ = false;
  } else {
    smoothed_q_ = alpha_q_ * q + (1.0 - alpha_q_) * smoothed_q_;
    smoothed_dq_ = alpha_dq_ * dq + (1.0 - alpha_dq_) * smoothed_dq_;
  }

  current_q = smoothed_q_;
  current_dq = smoothed_dq_;

  Eigen::Matrix<double, 7, 1> joint_pos_error = desired_q - current_q;
  Eigen::Matrix<double, 7, 1> joint_vel_error = desired_dq - current_dq;

  // tau = -Kp * (q-q_cmd) - Kd * (dq-dq_cmd) + M*a_cmd + coriolis
  // NOTE: Franka already does automatic gravity compensation,
  // so we don't add it here
  Eigen::Matrix<double, 7, 1> tau_d = Kp_.cwiseProduct(joint_pos_error) +
                                      Kd_.cwiseProduct(joint_vel_error) +
                                      M * desired_ddq + coriolis;

  // Apply torque limits and track violations
  bool torque_limit_violated = false;
  for (int i = 0; i < 7; i++) {
    if (tau_d[i] > joint_tau_limits_[i]) {
      std::cout << "[TORQUE_LIMIT] Joint " << i
                << " hit upper limit: " << tau_d[i] << " -> "
                << joint_tau_limits_[i] << " Nm" << std::endl;
      tau_d[i] = joint_tau_limits_[i];
      torque_limit_violated = true;
    } else if (tau_d[i] < -joint_tau_limits_[i]) {
      std::cout << "[TORQUE_LIMIT] Joint " << i
                << " hit lower limit: " << tau_d[i] << " -> "
                << -joint_tau_limits_[i] << " Nm" << std::endl;
      tau_d[i] = -joint_tau_limits_[i];
      torque_limit_violated = true;
    }
  }

  std::array<double, 7> tau_d_array;
  Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

  return ControllerResult{tau_d_array, joint_position_limit_violated, torque_limit_violated};
}

} // namespace controllers
} // namespace bamboo
