// Minimal Joint Impedance Controller Implementation
// Based on deoxys_control implementation

#include "joint_impedance_controller.h"
#include <iostream>

JointImpedanceController::JointImpedanceController(franka::Model* model)
    : model_(model), first_state_(true), alpha_q_(0.9), alpha_dq_(0.9) {

    // Default gains from deoxys_control/deoxys/config/joint-impedance-controller.yml
    Kp_ << 100.0, 100.0, 100.0, 100.0, 75.0, 150.0, 50.0;
    Kd_ << 20.0, 20.0, 20.0, 20.0, 7.5, 15.0, 5.0;

    // Joint limits from deoxys_control/deoxys/franka-interface/src/controllers/joint_impedance.cpp:43-44
    joint_max_ << 2.8978, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
    joint_min_ << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;

    // Torque limits from deoxys_control defaults
    joint_tau_limits_ << 10.0, 10.0, 10.0, 10.0, 10.0, 5.0, 5.0;

    smoothed_q_.setZero();
    smoothed_dq_.setZero();
}

void JointImpedanceController::SetGains(
    const std::array<double, 7>& kp,
    const std::array<double, 7>& kd) {
    Kp_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(kp.data());
    Kd_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(kd.data());
}

std::array<double, 7> JointImpedanceController::Step(
    const franka::RobotState& robot_state,
    const Eigen::Matrix<double, 7, 1>& desired_q) {

    // This function is adapted from:
    // deoxys_control/deoxys/franka-interface/src/controllers/joint_impedance.cpp:86-170

    // Get dynamics from robot model (lines 95-100 in deoxys)
    std::array<double, 49> mass_array = model_->mass(robot_state);
    Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass_array.data());

    std::array<double, 7> coriolis_array = model_->coriolis(robot_state);
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

    // Current joint state (lines 103-106 in deoxys)
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

    // State estimation with exponential smoothing (lines 112-121 in deoxys)
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

    // Compute position error (line 122 in deoxys)
    Eigen::Matrix<double, 7, 1> joint_pos_error = desired_q - current_q;

    // Joint impedance control law (line 124 in deoxys)
    // tau = Kp * (q_desired - q) - Kd * dq
    Eigen::Matrix<double, 7, 1> tau_d =
        Kp_.cwiseProduct(joint_pos_error) - Kd_.cwiseProduct(current_dq);

    // Joint limit safety (lines 156-161 in deoxys)
    // Zero torque when approaching joint limits (within 0.1 rad)
    Eigen::Matrix<double, 7, 1> dist2joint_max = joint_max_ - current_q;
    Eigen::Matrix<double, 7, 1> dist2joint_min = current_q - joint_min_;

    for (int i = 0; i < 7; i++) {
        if (dist2joint_max[i] < 0.1 && tau_d[i] > 0.0) {
            tau_d[i] = 0.0;
        }
        if (dist2joint_min[i] < 0.1 && tau_d[i] < 0.0) {
            tau_d[i] = 0.0;
        }
    }

    // Apply torque limits (lines 163-165 in deoxys)
    for (int i = 0; i < 7; i++) {
        if (tau_d[i] > joint_tau_limits_[i]) {
            tau_d[i] = joint_tau_limits_[i];
        } else if (tau_d[i] < -joint_tau_limits_[i]) {
            tau_d[i] = -joint_tau_limits_[i];
        }
    }

    // Convert to std::array (lines 166-167 in deoxys)
    std::array<double, 7> tau_d_array;
    Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

    return tau_d_array;
}
