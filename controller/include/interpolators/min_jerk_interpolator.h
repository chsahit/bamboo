#ifndef BAMBOO_MIN_JERK_INTERPOLATOR_H_
#define BAMBOO_MIN_JERK_INTERPOLATOR_H_

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace bamboo {
namespace interpolators {

class MinJerkInterpolator {
private:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Vector7i = Eigen::Matrix<int, 7, 1>;

  Vector7d q_start_;
  Vector7d q_goal_;
  Vector7d v_start_;
  Vector7d v_goal_;

  Vector7d last_q_t_;
  Vector7d last_v_t_;
  Vector7d prev_q_goal_;
  Vector7d prev_v_goal_;

  double dt_;
  double last_time_;
  double max_time_;
  double start_time_;
  bool start_;
  bool first_goal_;
  bool do_min_jerk_;

public:
  inline MinJerkInterpolator()
      : dt_(0.), last_time_(0.), max_time_(1.), start_time_(0.), start_(false),
        first_goal_(true), do_min_jerk_(false) {};

  inline ~MinJerkInterpolator() {};

  inline void Reset(const double &time_sec,
                    const Eigen::Matrix<double, 7, 1> &q_start,
                    const Eigen::Matrix<double, 7, 1> &q_goal, const int &rate,
                    const double &max_time) {
    Vector7d v_zero = Vector7d::Zero();
    Reset(time_sec, q_start, q_goal, v_zero, v_zero, rate, max_time);
  };

  inline void Reset(const double &time_sec,
                    const Eigen::Matrix<double, 7, 1> &q_start,
                    const Eigen::Matrix<double, 7, 1> &q_goal,
                    const Eigen::Matrix<double, 7, 1> &v_start,
                    const Eigen::Matrix<double, 7, 1> &v_goal, const int &rate,
                    const double &max_time) {
    dt_ = 1. / static_cast<double>(rate);
    last_time_ = time_sec;
    start_time_ = time_sec;
    max_time_ = max_time;

    start_ = false;

    if (first_goal_) {
      q_start_ = q_start;
      v_start_ = v_start;
      prev_q_goal_ = q_start;
      prev_v_goal_ = v_start;
      first_goal_ = false;
    } else {
      prev_q_goal_ = q_goal_;
      prev_v_goal_ = v_goal_;
      q_start_ = prev_q_goal_;
      v_start_ = prev_v_goal_;
    }
    q_goal_ = q_goal;
    v_goal_ = v_goal;
  };

  inline void GetNextStep(const double &time_sec, Vector7d &q_t) {
    Vector7d v_t;
    GetNextStep(time_sec, q_t, v_t);
  };

  inline void GetNextStep(const double &time_sec, Vector7d &q_t,
                          Vector7d &v_t) {
    if (!start_) {
      start_time_ = time_sec;
      last_q_t_ = q_start_;
      last_v_t_ = v_start_;
      start_ = true;
    }

    if (last_time_ + dt_ <= time_sec) {
      double t =
          std::min(std::max((time_sec - start_time_) / max_time_, 0.), 1.);
      // Min-jerk 5th-order polynomial transformation
      double transformed_t = t;
      if (do_min_jerk_) {
        transformed_t =
            10 * std::pow(t, 3) - 15 * std::pow(t, 4) + 6 * std::pow(t, 5);
      }

      last_q_t_ = q_start_ + transformed_t * (q_goal_ - q_start_);
      last_v_t_ = v_start_ + transformed_t * (v_goal_ - v_start_);
      last_time_ = time_sec;
    }
    q_t = last_q_t_;
    v_t = last_v_t_;
  };
};

} // namespace interpolators
} // namespace bamboo

#endif // BAMBOO_MIN_JERK_INTERPOLATOR_H_