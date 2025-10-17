// Joint Impedance Controller
// Adapted from deoxys_control/deoxys/franka-interface/include/controllers/joint_impedance.h

#ifndef BAMBOO_JOINT_IMPEDANCE_CONTROLLER_H
#define BAMBOO_JOINT_IMPEDANCE_CONTROLLER_H

#include <array>
#include <Eigen/Dense>
#include <franka/robot_state.h>
#include <franka/model.h>

namespace bamboo {
namespace controllers {

class JointImpedanceController {
public:
    JointImpedanceController(franka::Model* model);

    // Compute torque command given current state and desired joint position
    // From deoxys_control/deoxys/franka-interface/src/controllers/joint_impedance.cpp
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
    Eigen::Matrix<double, 7, 1> Kp_;
    Eigen::Matrix<double, 7, 1> Kd_;

    // Joint limits - from deoxys_control
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

}  // namespace controllers
}  // namespace bamboo

#endif  // BAMBOO_JOINT_IMPEDANCE_CONTROLLER_H
