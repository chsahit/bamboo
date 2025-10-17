// Min-Jerk Trajectory Interpolator
// Copied from deoxys_control/deoxys/franka-interface/include/utils/traj_interpolators/min_jerk_joint_position_traj_interpolator.h

#ifndef MIN_JERK_INTERPOLATOR_H
#define MIN_JERK_INTERPOLATOR_H

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

class MinJerkInterpolator {
public:
    using Vector7d = Eigen::Matrix<double, 7, 1>;

    MinJerkInterpolator();

    // Reset interpolator with new start and goal positions
    // Copied from deoxys min_jerk_joint_position_traj_interpolator.h:37-60
    void Reset(
        const double& time_sec,
        const Vector7d& q_start,
        const Vector7d& q_goal,
        const int& policy_rate,
        const int& traj_rate,
        const double& traj_interpolator_time_fraction
    );

    // Get next interpolated joint position
    // Copied from deoxys min_jerk_joint_position_traj_interpolator.h:62-79
    void GetNextStep(const double& time_sec, Vector7d& q_t);

private:
    Vector7d q_start_;
    Vector7d q_goal_;
    Vector7d last_q_t_;
    Vector7d prev_q_goal_;

    double dt_;
    double last_time_;
    double max_time_;
    double start_time_;
    bool start_;
    bool first_goal_;
};

#endif // MIN_JERK_INTERPOLATOR_H
