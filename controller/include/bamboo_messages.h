#pragma once

#include <string>
#include <vector>
#include <msgpack.hpp>

namespace bamboo_msgs {
  // Robot state message
  struct RobotState {
    std::vector<double> q;           // Joint positions (7)
    std::vector<double> dq;          // Joint velocities (7)
    std::vector<double> tau_J;       // Joint torques (7)
    std::vector<double> O_T_EE;      // End-effector pose (16 - 4x4 matrix)
    double time_sec;

    MSGPACK_DEFINE_MAP(q, dq, tau_J, O_T_EE, time_sec)
  };

  // Waypoint with timing information
  struct TimedWaypoint {
    std::vector<double> q_goal;      // Goal joint positions (7)
    std::vector<double> velocity;    // Desired velocity at waypoint (7)
    double duration;                 // Duration for this waypoint
    std::vector<double> kp;          // Stiffness (7)
    std::vector<double> kd;          // Damping (7)

    MSGPACK_DEFINE_MAP(q_goal, velocity, duration, kp, kd)
  };

  // Trajectory request
  struct TrajectoryRequest {
    std::vector<TimedWaypoint> waypoints;
    double default_duration;
    std::vector<double> default_velocity;

    MSGPACK_DEFINE_MAP(waypoints, default_duration, default_velocity)
  };
}
