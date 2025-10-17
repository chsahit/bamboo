// Min-Jerk Trajectory Interpolator Implementation
// Copied from deoxys_control/deoxys/franka-interface/include/utils/traj_interpolators/min_jerk_joint_position_traj_interpolator.h

#include "min_jerk_interpolator.h"

MinJerkInterpolator::MinJerkInterpolator()
    : dt_(0.0),
      last_time_(0.0),
      max_time_(1.0),
      start_time_(0.0),
      start_(false),
      first_goal_(true) {
    q_start_.setZero();
    q_goal_.setZero();
    last_q_t_.setZero();
    prev_q_goal_.setZero();
}

// Implementation copied from deoxys min_jerk_joint_position_traj_interpolator.h:37-60
void MinJerkInterpolator::Reset(
    const double& time_sec,
    const Vector7d& q_start,
    const Vector7d& q_goal,
    const int& policy_rate,
    const int& traj_rate,
    const double& traj_interpolator_time_fraction) {

    dt_ = 1.0 / static_cast<double>(traj_rate);
    last_time_ = time_sec;

    max_time_ = (1.0 / static_cast<double>(policy_rate)) * traj_interpolator_time_fraction;
    start_time_ = time_sec;

    start_ = false;

    if (first_goal_) {
        q_start_ = q_start;
        prev_q_goal_ = q_start;
        first_goal_ = false;
    } else {
        prev_q_goal_ = q_goal_;
        q_start_ = prev_q_goal_;
    }

    q_goal_ = q_goal;
}

// Implementation copied from deoxys min_jerk_joint_position_traj_interpolator.h:62-79
void MinJerkInterpolator::GetNextStep(const double& time_sec, Vector7d& q_t) {
    if (!start_) {
        start_time_ = time_sec;
        last_q_t_ = q_start_;
        start_ = true;
    }

    if (last_time_ + dt_ <= time_sec) {
        // Compute normalized time: t ∈ [0, 1]
        double t = std::min(std::max((time_sec - start_time_) / max_time_, 0.0), 1.0);

        // Apply 5th-order polynomial transformation for minimum jerk
        // This is the key min-jerk formula: 10t³ - 15t⁴ + 6t⁵
        double transformed_t = 10.0 * std::pow(t, 3) - 15.0 * std::pow(t, 4) + 6.0 * std::pow(t, 5);

        // Interpolate using transformed time
        last_q_t_ = q_start_ + transformed_t * (q_goal_ - q_start_);
        last_time_ = time_sec;
    }

    q_t = last_q_t_;
}
