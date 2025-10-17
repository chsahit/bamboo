// Minimal Joint Impedance Controller
// Based on deoxys_control implementation

#ifndef MINIMAL_JOINT_IMPEDANCE_CONTROLLER_H
#define MINIMAL_JOINT_IMPEDANCE_CONTROLLER_H

#include <array>
#include <Eigen/Dense>
#include <franka/robot_state.h>
#include <franka/model.h>

class JointImpedanceController {
public:
    JointImpedanceController(franka::Model* model);

    // Compute torque command given current state and desired joint position
    // Adapted from deoxys_control/deoxys/franka-interface/src/controllers/joint_impedance.cpp
    std::array<double, 7> Step(
        const franka::RobotState& robot_state,
        const Eigen::Matrix<double, 7, 1>& desired_q
    );

    // Set controller gains (optional - uses defaults if not called)
    void SetGains(
        const std::array<double, 7>& kp,
        const std::array<double, 7>& kd
    );

private:
    franka::Model* model_;

    // Controller gains - using deoxys_control defaults
    // From deoxys_control/deoxys/config/joint-impedance-controller.yml
    Eigen::Matrix<double, 7, 1> Kp_;
    Eigen::Matrix<double, 7, 1> Kd_;

    // Joint limits - from deoxys_control/deoxys/franka-interface/src/controllers/joint_impedance.cpp:43-44
    Eigen::Matrix<double, 7, 1> joint_max_;
    Eigen::Matrix<double, 7, 1> joint_min_;

    // Torque limits per joint
    Eigen::Matrix<double, 7, 1> joint_tau_limits_;

    // Simple exponential smoothing for velocity estimation
    bool first_state_;
    Eigen::Matrix<double, 7, 1> smoothed_q_;
    Eigen::Matrix<double, 7, 1> smoothed_dq_;
    double alpha_q_;
    double alpha_dq_;
};

#endif // MINIMAL_JOINT_IMPEDANCE_CONTROLLER_H
