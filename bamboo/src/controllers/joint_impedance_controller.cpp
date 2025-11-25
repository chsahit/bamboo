// Joint Impedance Controller Implementation

#include "controllers/joint_impedance_controller.h"
#include <iostream>

namespace bamboo {
namespace controllers {

JointImpedanceController::JointImpedanceController(franka::Model *model)
    : model_(model), first_state_(true), alpha_q_(1.0), alpha_dq_(0.95) {

  // Kp_ << 100.0, 100.0, 100.0, 100.0, 75.0, 150.0, 50.0;
  // Kd_ << 20.0, 20.0, 20.0, 20.0, 7.5, 15.0, 5.0;

  // This seems to be better
  Kp_ << 600.0, 600.0, 600.0, 300.0, 250.0, 100.0, 50.0;
  Kd_ << 20.0, 20.0, 20.0, 20.0, 7.5, 15.0, 5.0;
  inertia_comp_ = false;
    

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

std::array<double, 7>
JointImpedanceController::Step(const franka::RobotState &robot_state,
                               const Eigen::Matrix<double, 7, 1> &desired_q,
                               const Eigen::Matrix<double, 7, 1> &desired_dq) {

  std::array<double, 49> mass_array = model_->mass(robot_state);
  Eigen::Map<const Eigen::Matrix<double, 7, 7, Eigen::ColMajor>> M(mass_array.data());

  std::array<double, 7> coriolis_array = model_->coriolis(robot_state);
  std::array<double, 7> gravity_array = model_->gravity(robot_state);
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());

  Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

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

  Eigen::Matrix<double, 7, 1> tau_d;

  // NOTE: Franka already does automatic gravity compensation, so we don't add it here
  if (inertia_comp_) {
    // Inertial compensation: tau = M(q) * (Kd * dq_err + Kp * q_err) + coriolis
    Eigen::Matrix<double, 7, 1> ddq_star = Kd_.cwiseProduct(joint_vel_error) +
                                            Kp_.cwiseProduct(joint_pos_error);
    tau_d = M * ddq_star + coriolis;
  } else {
    // Original stiffness control law: tau = Kp * (q_desired - q) + Kd * (dq_desired - dq) + coriolis
    tau_d = Kp_.cwiseProduct(joint_pos_error) +
            Kd_.cwiseProduct(joint_vel_error) +
            coriolis;
  }

  // Debug output
  static int debug_counter = 0;
  if (debug_counter % 100 == 0) {  // Print every 100 cycles
    std::cout << "[DEBUG] Mass array first 10 elements: ";
    for (int i = 0; i < 10; i++) {
      std::cout << mass_array[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "[DEBUG] M(0,0): " << M(0,0) << " M(1,1): " << M(1,1) << " M(6,6): " << M(6,6) << std::endl;
    std::cout << "[DEBUG] M diagonal: " << M.diagonal().transpose() << std::endl;
    std::cout << "[DEBUG] Full M matrix:" << std::endl << M << std::endl;
    if (inertia_comp_) {
      Eigen::Matrix<double, 7, 1> ddq_star = Kd_.cwiseProduct(joint_vel_error) +
                                              Kp_.cwiseProduct(joint_pos_error);
      std::cout << "[DEBUG] ddq_star: " << ddq_star.transpose() << std::endl;
      std::cout << "[DEBUG] M*ddq_star: " << (M * ddq_star).transpose() << std::endl;
    }
    std::cout << "[DEBUG] coriolis: " << coriolis.transpose() << std::endl;
  }
  debug_counter++;

  // Apply torque limits
  for (int i = 0; i < 7; i++) {
    if (tau_d[i] > joint_tau_limits_[i]) {
      std::cout << "[TORQUE_LIMIT] Joint " << i << " hit upper limit: "
                << tau_d[i] << " -> " << joint_tau_limits_[i] << " Nm" << std::endl;
      tau_d[i] = joint_tau_limits_[i];
    } else if (tau_d[i] < -joint_tau_limits_[i]) {
      std::cout << "[TORQUE_LIMIT] Joint " << i << " hit lower limit: "
                << tau_d[i] << " -> " << -joint_tau_limits_[i] << " Nm" << std::endl;
      tau_d[i] = -joint_tau_limits_[i];
    }
  }

  std::array<double, 7> tau_d_array;
  Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

  return tau_d_array;
}

} // namespace controllers
} // namespace bamboo
