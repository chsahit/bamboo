// Joint Impedance control with Min-Jerk interpolation

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <exception>
#include <getopt.h>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <signal.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>
#include <map>

#include <zmq.hpp>
#include <msgpack.hpp>

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include <Eigen/Dense>

#include "controllers/joint_impedance_controller.h"
#include "interpolators/min_jerk_interpolator.h"
#include "bamboo_messages.h"

// Global flag for signal handling
std::atomic<bool> global_shutdown{false};

// Global exception handling
std::mutex exception_mutex;
std::exception_ptr thread_exception_ptr = nullptr;

void setThreadException(std::exception_ptr ex) {
  std::lock_guard<std::mutex> lock(exception_mutex);
  if (!thread_exception_ptr) {
    thread_exception_ptr = ex;
    global_shutdown = true;
  }
}

std::exception_ptr getThreadException() {
  std::lock_guard<std::mutex> lock(exception_mutex);
  return thread_exception_ptr;
}

// Signal handler for graceful shutdown
void signalHandler(int signal) {
  if (signal == SIGINT) {
    std::cout << "\nReceived Ctrl+C, shutting down gracefully..." << std::endl;
    global_shutdown = true;
  }
}

// Server implementation
class BambooControlServer {
private:
  franka::Robot *robot_;
  franka::Model *model_;
  bamboo::controllers::JointImpedanceController *controller_;
  bamboo::interpolators::MinJerkInterpolator *interpolator_;

  std::atomic<bool> control_running_{false};
  std::atomic<bool> joint_limit_hit_{false};

  // Control parameters
  const int traj_rate_ = 500; // Hz
  const double max_time = 1.0;
  const bool log_err_ = true;

  Eigen::Matrix<double, 7, 1> q_current_;
  Eigen::Matrix<double, 7, 1> q_goal_;

  // For acceleration computation via finite differencing
  Eigen::Matrix<double, 7, 1> velocity_cmd_prev_;
  Eigen::Matrix<double, 7, 1> a_cmd_latest_;

  // Low-pass filter frequency for acceleration
  const double diff_low_pass_freq_ = 30.0; // Hz

public:
  BambooControlServer(
      franka::Robot *robot, franka::Model *model,
      bamboo::controllers::JointImpedanceController *controller,
      bamboo::interpolators::MinJerkInterpolator *interpolator)
      : robot_(robot), model_(model), controller_(controller),
        interpolator_(interpolator) {

    // Get initial robot state
    franka::RobotState init_state = robot_->readOnce();
    q_current_ = Eigen::VectorXd::Map(init_state.q.data(), 7);
    q_goal_ = q_current_;

    // Initialize velocity and acceleration tracking
    velocity_cmd_prev_.setZero();
    a_cmd_latest_.setZero();

    std::cout << "Initial joint positions: " << q_current_.transpose()
              << std::endl;
  }

  bamboo_msgs::RobotState GetRobotState() {
    try {
      // Get current robot state
      franka::RobotState current_state = robot_->readOnce();

      bamboo_msgs::RobotState state_msg;

      // Add joint positions, velocities, and torques
      state_msg.q.resize(7);
      state_msg.dq.resize(7);
      state_msg.tau_J.resize(7);
      for (size_t i = 0; i < 7; ++i) {
        state_msg.q[i] = current_state.q[i];
        state_msg.dq[i] = current_state.dq[i];
        state_msg.tau_J[i] = current_state.tau_J[i];
      }

      // Add end-effector pose (4x4 transformation matrix: O_T_EE)
      state_msg.O_T_EE.resize(16);
      for (size_t i = 0; i < 16; ++i) {
        state_msg.O_T_EE[i] = current_state.O_T_EE[i];
      }

      // Add timing information
      state_msg.time_sec = current_state.time.toSec();

      return state_msg;
    } catch (const franka::Exception &e) {
      throw std::runtime_error(std::string("Franka exception: ") + e.what());
    } catch (const std::exception &e) {
      throw std::runtime_error(std::string("Exception: ") + e.what());
    }
  }

  bool ExecuteJointImpedanceTrajectory(const bamboo_msgs::TrajectoryRequest &request) {
    std::cout << "[SERVER] Received trajectory with " << request.waypoints.size()
              << " waypoints" << std::endl;

    try {
      // Check if already running
      if (control_running_.load()) {
        throw std::runtime_error("Control loop already running");
      }

      if (request.waypoints.empty()) {
        throw std::runtime_error("Empty trajectory");
      }

      // Parse and prepare all waypoints
      std::vector<Eigen::Matrix<double, 7, 1>> trajectory_goals;
      std::vector<Eigen::Matrix<double, 7, 1>> trajectory_velocities;
      std::vector<double> trajectory_durations;

      for (size_t i = 0; i < request.waypoints.size(); ++i) {
        const bamboo_msgs::TimedWaypoint &waypoint = request.waypoints[i];

        // Validate goal has 7 values
        if (waypoint.q_goal.size() != 7) {
          throw std::runtime_error("Joint configuration must have 7 values");
        }

        // Get waypoint duration
        double waypoint_duration;
        if (waypoint.duration > 0) {
          waypoint_duration = waypoint.duration;
        } else if (request.default_duration > 0) {
          waypoint_duration = request.default_duration;
        } else {
          waypoint_duration = 1.0; // Fallback default
        }

        // Get waypoint velocity
        Eigen::Matrix<double, 7, 1> waypoint_velocity = Eigen::Matrix<double, 7, 1>::Zero();
        if (waypoint.velocity.size() == 7) {
          // Use waypoint-specific velocity if provided
          for (int j = 0; j < 7; ++j) {
            waypoint_velocity(j) = waypoint.velocity[j];
          }
        } else if (request.default_velocity.size() == 7) {
          // Use default velocity if provided
          for (int j = 0; j < 7; ++j) {
            waypoint_velocity(j) = request.default_velocity[j];
          }
        }
        // If neither is provided, waypoint_velocity remains zero

        // Check for termination
        if (global_shutdown) {
          std::cout << "[SERVER] Termination requested" << std::endl;
          break;
        }

        // Store goal position
        Eigen::Matrix<double, 7, 1> goal;
        for (int j = 0; j < 7; ++j) {
          goal(j) = waypoint.q_goal[j];
        }

        trajectory_goals.push_back(goal);
        trajectory_velocities.push_back(waypoint_velocity);
        trajectory_durations.push_back(waypoint_duration);
      }

      // Execute entire trajectory in single control call
      bool success = executeTrajectory(trajectory_goals, trajectory_velocities,
                                       trajectory_durations);
      if (!success) {
        throw std::runtime_error("Trajectory execution failed");
      }

      std::cout << "[SERVER] Trajectory completed successfully" << std::endl;
      return true;

    } catch (const franka::ControlException &e) {
      std::cerr << "[SERVER] Control exception: " << e.what() << std::endl;
      throw;
    } catch (const std::exception &e) {
      std::cerr << "[SERVER] Exception: " << e.what() << std::endl;
      throw;
    }
  }

private:
  bool executeTrajectory(const std::vector<Eigen::Matrix<double, 7, 1>> &goals,
                        const std::vector<Eigen::Matrix<double, 7, 1>> &velocities,
                        const std::vector<double> &durations) {
    if (goals.empty())
      return false;

    control_running_ = true;
    joint_limit_hit_ = false;
    double control_time = 0.0;

    // Reset velocity and acceleration tracking for new trajectory
    velocity_cmd_prev_.setZero();
    a_cmd_latest_.setZero();

    // Current waypoint tracking
    std::size_t current_waypoint = 0;
    double waypoint_start_time = 0.0;

    // Max joint error tracking (L1 norm across waypoint final errors)
    double max_joint_error_rad = 0.0;

    // Max end-effector error tracking
    double max_ee_position_error_m = 0.0;
    double max_ee_orientation_error_rad = 0.0;

    // Final waypoint end-effector error tracking
    double final_ee_position_error_m = 0.0;
    double final_ee_orientation_error_rad = 0.0;

    // Initialize first waypoint
    q_goal_ = goals[0];
    Eigen::Matrix<double, 7, 1> velocity_start =
        Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> velocity_goal =
        velocities.empty() ? Eigen::Matrix<double, 7, 1>::Zero()
                           : velocities[0];
    interpolator_->Reset(control_time, q_current_, q_goal_, velocity_start,
                         velocity_goal, traj_rate_, durations[0]);

    std::cout << "[CONTROL] Starting trajectory with " << goals.size()
              << " waypoints" << std::endl;

    // Single control callback for entire trajectory
    auto control_callback = [&](const franka::RobotState &robot_state,
                                franka::Duration period) -> franka::Torques {
      try {
        // Get time step
        const double dt = period.toSec();

        // Update time
        control_time += dt;

        // Update current position
        q_current_ = Eigen::VectorXd::Map(robot_state.q.data(), 7);

        // Check if current waypoint is complete
        double waypoint_elapsed = control_time - waypoint_start_time;
        if (waypoint_elapsed >= durations[current_waypoint]) {
          if (log_err_) {
            // Log joint error for waypoint that timed out
            Eigen::Matrix<double, 7, 1> waypoint_error = q_goal_ - q_current_;
            double waypoint_final_error_rad = waypoint_error.cwiseAbs().sum();
            // Update max error across all waypoints
            if (waypoint_final_error_rad > max_joint_error_rad) {
              max_joint_error_rad = waypoint_final_error_rad;
            }

            // Calculate end-effector position and orientation errors for
            // completed waypoint Get desired EE pose from goal joint angles
            std::array<double, 7> q_goal_array;
            Eigen::VectorXd::Map(&q_goal_array[0], 7) = q_goal_;

            // Create a temporary robot state with goal joint positions
            franka::RobotState temp_state = robot_state;
            temp_state.q = q_goal_array;

            std::array<double, 16> desired_ee_pose_array =
                model_->pose(franka::Frame::kEndEffector, temp_state);
            Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>
                desired_ee_pose(desired_ee_pose_array.data());

            // Get current EE pose from robot state
            Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>
                current_ee_pose(robot_state.O_T_EE.data());

            // Calculate position error (translation part)
            Eigen::Vector3d desired_position =
                desired_ee_pose.block<3, 1>(0, 3);
            Eigen::Vector3d current_position =
                current_ee_pose.block<3, 1>(0, 3);
            double current_ee_position_error =
                (desired_position - current_position).norm();
            if (current_ee_position_error > max_ee_position_error_m) {
              max_ee_position_error_m = current_ee_position_error;
            }

            // Calculate orientation error (rotation part)
            Eigen::Matrix3d desired_rotation =
                desired_ee_pose.block<3, 3>(0, 0);
            Eigen::Matrix3d current_rotation =
                current_ee_pose.block<3, 3>(0, 0);
            Eigen::Matrix3d rotation_error =
                desired_rotation * current_rotation.transpose();

            // Convert rotation matrix to angle-axis to get scalar error
            Eigen::AngleAxisd angle_axis(rotation_error);
            double current_ee_orientation_error = std::abs(angle_axis.angle());
            if (current_ee_orientation_error > max_ee_orientation_error_rad) {
              max_ee_orientation_error_rad = current_ee_orientation_error;
            }
          }

          // Move to next waypoint
          current_waypoint++;

          if (current_waypoint >= goals.size()) {
            // If still moving, continue with current control to let robot
            // settle
            current_waypoint = goals.size() - 1; // Stay on last waypoint
          } else {
            // Setup next waypoint
            waypoint_start_time = control_time;
            q_goal_ = goals[current_waypoint];
            Eigen::Matrix<double, 7, 1> velocity_prev =
                (current_waypoint > 0 &&
                 current_waypoint - 1 < velocities.size())
                    ? velocities[current_waypoint - 1]
                    : Eigen::Matrix<double, 7, 1>::Zero();
            Eigen::Matrix<double, 7, 1> velocity_curr =
                (current_waypoint < velocities.size())
                    ? velocities[current_waypoint]
                    : Eigen::Matrix<double, 7, 1>::Zero();
            interpolator_->Reset(control_time, q_current_, q_goal_,
                                 velocity_prev, velocity_curr, traj_rate_,
                                 durations[current_waypoint]);
          }
        }

        // Get interpolated desired position and velocity for current waypoint
        Eigen::Matrix<double, 7, 1> q_desired;
        Eigen::Matrix<double, 7, 1> dq_desired;
        interpolator_->GetNextStep(control_time, q_desired, dq_desired);

        // Compute desired acceleration via finite differencing
        Eigen::Matrix<double, 7, 1> ddq_desired =
            Eigen::Matrix<double, 7, 1>::Zero();
        if (dt > 0.0) {
          // Compute raw acceleration from velocity difference
          Eigen::Matrix<double, 7, 1> a_cmd_raw =
              (dq_desired - velocity_cmd_prev_) / dt;

          // Apply low-pass filter
          for (int i = 0; i < 7; ++i) {
            a_cmd_latest_[i] = franka::lowpassFilter(
                dt, a_cmd_raw[i], a_cmd_latest_[i], diff_low_pass_freq_);
          }
          ddq_desired = a_cmd_latest_;

          // Update previous velocity for next iteration
          velocity_cmd_prev_ = dq_desired;
        }

        // Compute control torques
        bamboo::controllers::ControllerResult result =
            controller_->Step(robot_state, q_desired , dq_desired, ddq_desired);

        // Check for torque limit violation
        if (result.torque_limit_violated) {
          joint_limit_hit_ = true;
          std::cout << "[CONTROL] Torque limit violated - ending trajectory early" << std::endl;
          std::array<double, 7> zero_torques = {0.0, 0.0, 0.0, 0.0,
                                                0.0, 0.0, 0.0};
          return franka::MotionFinished(franka::Torques(zero_torques));
        }

        // Apply rate limiting
        std::array<double, 7> tau_d_rate_limited = franka::limitRate(
            franka::kMaxTorqueRate, result.torques, robot_state.tau_J_d);

        // Check if all waypoints completed and robot has stopped
        if (current_waypoint >= goals.size() - 1 &&
            waypoint_elapsed >= durations[current_waypoint]) {
          Eigen::VectorXd dq_current =
              Eigen::VectorXd::Map(robot_state.dq.data(), 7);
          double velocity_norm = dq_current.norm();

          if (velocity_norm <
              0.01) { // Robot has stopped (threshold: 0.01 rad/s)
            // Calculate final waypoint errors (robot vs last goal) - robot is
            // now still
            Eigen::Matrix<double, 7, 1> final_goal = goals.back();
            std::array<double, 7> final_goal_array;
            Eigen::VectorXd::Map(&final_goal_array[0], 7) = final_goal;

            // Get desired EE pose for final waypoint
            franka::RobotState final_temp_state = robot_state;
            final_temp_state.q = final_goal_array;
            std::array<double, 16> final_desired_ee_pose_array =
                model_->pose(franka::Frame::kEndEffector, final_temp_state);
            Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>
                final_desired_ee_pose(final_desired_ee_pose_array.data());

            // Get current EE pose from robot state
            Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>
                current_ee_pose(robot_state.O_T_EE.data());

            // Calculate final position error
            Eigen::Vector3d final_desired_position =
                final_desired_ee_pose.block<3, 1>(0, 3);
            Eigen::Vector3d current_position =
                current_ee_pose.block<3, 1>(0, 3);
            final_ee_position_error_m =
                (final_desired_position - current_position).norm();

            // Calculate final orientation error
            Eigen::Matrix3d final_desired_rotation =
                final_desired_ee_pose.block<3, 3>(0, 0);
            Eigen::Matrix3d current_rotation =
                current_ee_pose.block<3, 3>(0, 0);
            Eigen::Matrix3d final_rotation_error =
                final_desired_rotation * current_rotation.transpose();
            Eigen::AngleAxisd final_angle_axis(final_rotation_error);
            final_ee_orientation_error_rad = std::abs(final_angle_axis.angle());

            std::cout << "[CONTROL] All waypoints completed, robot stopped"
                      << std::endl;
            return franka::MotionFinished(franka::Torques(tau_d_rate_limited));
          }
        }

        // Check if we should stop
        if (!control_running_ || global_shutdown) {
          return franka::MotionFinished(franka::Torques(tau_d_rate_limited));
        }

        return franka::Torques(tau_d_rate_limited);
      } catch (const std::exception &e) {
        std::cerr << "[CONTROL_CALLBACK] Exception: " << e.what() << std::endl;
        std::array<double, 7> zero_torques = {0.0, 0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0};
        return franka::MotionFinished(franka::Torques(zero_torques));
      }
    };

    try {
      // Execute control
      robot_->control(control_callback);
      control_running_ = false;

      // Check if trajectory failed due to joint limit violation
      if (joint_limit_hit_) {
        std::cerr << "[TRAJECTORY] Trajectory failed due to joint limit violation"
                  << std::endl;
        return false;
      }

      if (log_err_) {
        // Print max joint error across all waypoint final errors
        double max_joint_error_deg = max_joint_error_rad * 180.0 / M_PI;
        std::cout << "[CONTROL] Max sum of joint errors during trajectory: "
                  << std::fixed << std::setprecision(2) << max_joint_error_deg
                  << " degrees" << std::endl;

        // Print end-effector error metrics
        double max_ee_orientation_error_deg =
            max_ee_orientation_error_rad * 180.0 / M_PI;
        std::cout << "[CONTROL] Max EE position error during trajectory: "
                  << std::fixed << std::setprecision(4)
                  << max_ee_position_error_m * 1000.0 << " mm" << std::endl;
        std::cout << "[CONTROL] Max EE orientation error during trajectory: "
                  << std::fixed << std::setprecision(2)
                  << max_ee_orientation_error_deg << " degrees" << std::endl;
      }

      // Print final waypoint errors
      double final_ee_orientation_error_deg =
          final_ee_orientation_error_rad * 180.0 / M_PI;
      std::cout << "[CONTROL] Final EE position error (vs last waypoint): "
                << std::fixed << std::setprecision(4)
                << final_ee_position_error_m * 1000.0 << " mm" << std::endl;
      std::cout << "[CONTROL] Final EE orientation error (vs last waypoint): "
                << std::fixed << std::setprecision(2)
                << final_ee_orientation_error_deg << " degrees" << std::endl;

      return true;
    } catch (const franka::ControlException &e) {
      std::cerr << "[TRAJECTORY] Control exception: " << e.what() << std::endl;
      control_running_ = false;
      return false;
    }
  }
};

// Message parsing helpers
std::string parseCommand(const std::map<std::string, msgpack::object> &request_map) {
  std::string command;
  auto it = request_map.find("command");
  if (it != request_map.end()) {
    it->second.convert(command);
  }
  return command;
}

msgpack::sbuffer handleGetRobotState(BambooControlServer &server) {
  bamboo_msgs::RobotState state = server.GetRobotState();

  msgpack::sbuffer response_buf;
  msgpack::packer<msgpack::sbuffer> packer(response_buf);

  packer.pack_map(2);
  packer.pack("success");
  packer.pack(true);
  packer.pack("data");
  packer.pack(state);

  return response_buf;
}

msgpack::sbuffer handleExecuteTrajectory(BambooControlServer &server,
                                         const std::map<std::string, msgpack::object> &request_map) {
  bamboo_msgs::TrajectoryRequest traj_req;
  auto it = request_map.find("data");
  if (it != request_map.end()) {
    it->second.convert(traj_req);
  }

  bool success = server.ExecuteJointImpedanceTrajectory(traj_req);

  msgpack::sbuffer response_buf;
  msgpack::packer<msgpack::sbuffer> packer(response_buf);

  packer.pack_map(2);
  packer.pack("success");
  packer.pack(success);
  packer.pack("error");
  if (!success) {
    packer.pack(std::string("Joint limit violated during trajectory execution"));
  } else {
    packer.pack(std::string(""));
  }

  return response_buf;
}

msgpack::sbuffer handleTerminate() {
  std::cout << "[SERVER] Terminate request received" << std::endl;
  global_shutdown = true;

  msgpack::sbuffer response_buf;
  msgpack::packer<msgpack::sbuffer> packer(response_buf);

  packer.pack_map(2);
  packer.pack("success");
  packer.pack(true);
  packer.pack("error");
  packer.pack(std::string(""));

  return response_buf;
}

msgpack::sbuffer handleUnknownCommand(const std::string &command) {
  msgpack::sbuffer response_buf;
  msgpack::packer<msgpack::sbuffer> packer(response_buf);

  packer.pack_map(2);
  packer.pack("success");
  packer.pack(false);
  packer.pack("error");
  packer.pack(std::string("Unknown command: ") + command);

  return response_buf;
}

msgpack::sbuffer handleError(const std::string &error_msg) {
  msgpack::sbuffer response_buf;
  msgpack::packer<msgpack::sbuffer> packer(response_buf);

  packer.pack_map(2);
  packer.pack("success");
  packer.pack(false);
  packer.pack("error");
  packer.pack(error_msg);

  return response_buf;
}

void RunServer(const std::string &server_address, franka::Robot *robot,
               franka::Model *model,
               bamboo::controllers::JointImpedanceController *controller,
               bamboo::interpolators::MinJerkInterpolator *interpolator) {

  BambooControlServer server(robot, model, controller, interpolator);

  // Create context and socket
  zmq::context_t context(1);
  zmq::socket_t socket(context, zmq::socket_type::rep);
  socket.bind(server_address);

  std::cout << "Server listening on " << server_address << std::endl;

  // Message handling loop
  while (!global_shutdown) {
    try {
      // Receive message
      zmq::message_t request_msg;
#if CPPZMQ_VERSION >= ZMQ_MAKE_VERSION(4, 3, 0)
      auto result = socket.recv(request_msg, zmq::recv_flags::none);
#else
      auto result = socket.recv(&request_msg, 0);
#endif

      if (!result) {
        continue;
      }

      // Unpack the request
      msgpack::object_handle oh = msgpack::unpack(
          static_cast<const char*>(request_msg.data()),
          request_msg.size());
      msgpack::object obj = oh.get();

      // Parse as a map to get the command
      std::map<std::string, msgpack::object> request_map;
      obj.convert(request_map);

      std::string command = parseCommand(request_map);
      std::cout << "[SERVER] Received command: " << command << std::endl;

      // Handle the command
      msgpack::sbuffer response_buf;

      try {
        if (command == "get_robot_state") {
          response_buf = handleGetRobotState(server);
        } else if (command == "execute_trajectory") {
          response_buf = handleExecuteTrajectory(server, request_map);
        } else if (command == "terminate") {
          response_buf = handleTerminate();
        } else {
          response_buf = handleUnknownCommand(command);
        }
      } catch (const std::exception &e) {
        std::cerr << "[SERVER] Error handling command: " << e.what() << std::endl;
        response_buf = handleError(e.what());
      }

      // Send response
      zmq::message_t response_msg(response_buf.data(), response_buf.size());
#if CPPZMQ_VERSION >= ZMQ_MAKE_VERSION(4, 3, 0)
      socket.send(response_msg, zmq::send_flags::none);
#else
      socket.send(response_msg, 0);  // Old API uses reference, not pointer (https://github.com/zeromq/cppzmq/issues/69)
#endif

    } catch (const zmq::error_t &e) {
      if (e.num() == EINTR || global_shutdown) {
        break;
      }
      std::cerr << "[SERVER] Error: " << e.what() << std::endl;
    } catch (const std::exception &e) {
      std::cerr << "[SERVER] Exception in message loop: " << e.what() << std::endl;
    }
  }

  std::cout << "Shutting down server..." << std::endl;
  socket.close();
}

int main(int argc, char **argv) {
  // Register signal handler for graceful shutdown
  std::signal(SIGINT, signalHandler);

  std::string robot_ip;
  std::string port;
  std::string listen_address = "*";  // default

  int opt;
  while ((opt = getopt(argc, argv, "r:p:l:h")) != -1) {
    switch (opt) {
      case 'r':
        robot_ip = optarg;
        break;
      case 'p':
        port = optarg;
        break;
      case 'l':
        listen_address = optarg;
        break;
      case 'h':
      case '?':
      default:
        std::cerr << "Usage: " << argv[0] << " -r <robot-ip> -p <port> [-l <listen-address>]" << std::endl;
        std::cerr << "  -r: Robot IP address (required)" << std::endl;
        std::cerr << "  -p: Port number (required)" << std::endl;
        std::cerr << "  -l: Listen address (default: * for all interfaces)" << std::endl;
        std::cerr << "  -h: Show this help" << std::endl;
        return -1;
    }
  }

  // Validate required arguments
  if (robot_ip.empty() || port.empty()) {
    std::cerr << "Error: Robot IP and port are required" << std::endl;
    std::cerr << "Usage: " << argv[0] << " -r <robot-ip> -p <port> [-l <listen-address>]" << std::endl;
    return -1;
  }

  const std::string server_address = "tcp://" + listen_address + ":" + port;

  std::cout << "Bamboo Control Node Starting..." << std::endl;
  std::cout << "Robot IP: " << robot_ip << std::endl;
  std::cout << "Port: " << port << std::endl;
  std::cout << "Listen address: " << listen_address << std::endl;

  try {
    // Connect to robot
    std::cout << "Connecting to robot..." << std::endl;
    franka::Robot robot(robot_ip);
    robot.automaticErrorRecovery();

    // Set collision behavior
    robot.setCollisionBehavior(
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

    // Load model
    franka::Model model = robot.loadModel();

    // Create controller and interpolator
    bamboo::controllers::JointImpedanceController controller(&model);
    bamboo::interpolators::MinJerkInterpolator interpolator;

    // Start server
    RunServer(server_address, &robot, &model, &controller, &interpolator);

    std::cout << "Control node terminated successfully" << std::endl;

  } catch (const franka::Exception &e) {
    std::cerr << "Franka exception: " << e.what() << std::endl;
    return -1;
  } catch (const std::exception &e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return -1;
  }

  return 0;
}
