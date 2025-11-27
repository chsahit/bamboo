// Joint Impedance control with Min-Jerk interpolation

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <signal.h>
#include <stdexcept>
#include <string>
#include <thread>

#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include <Eigen/Dense>

#include "controllers/joint_impedance_controller.h"
#include "interpolators/min_jerk_interpolator.h"

// Protobuf messages
#include "bamboo_service.grpc.pb.h"
#include "franka_controller.pb.h"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;

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

// Helper function to populate robot state message
void populateRobotState(FrankaRobotStateMessage &robot_state_msg,
                        const franka::RobotState &robot_state) {
  // Clear previous data
  robot_state_msg.clear_q();
  robot_state_msg.clear_dq();
  robot_state_msg.clear_tau_j();
  robot_state_msg.clear_o_t_ee();

  // Add joint positions, velocities, and torques
  for (size_t i = 0; i < 7; ++i) {
    robot_state_msg.add_q(robot_state.q[i]);
    robot_state_msg.add_dq(robot_state.dq[i]);
    robot_state_msg.add_tau_j(robot_state.tau_J[i]);
  }

  // Add end-effector pose (4x4 transformation matrix: O_T_EE)
  for (size_t i = 0; i < 16; ++i) {
    robot_state_msg.add_o_t_ee(robot_state.O_T_EE[i]);
  }

  // Add basic timing information
  FrankaRobotStateMessage::Duration *time = robot_state_msg.mutable_time();
  time->set_tosec(robot_state.time.toSec());
  time->set_tomsec(robot_state.time.toMSec());
}

// Signal handler for graceful shutdown
void signalHandler(int signal) {
  if (signal == SIGINT) {
    std::cout << "\nReceived Ctrl+C, shutting down gracefully..." << std::endl;
    global_shutdown = true;
  }
}

// gRPC service implementation
class BambooControlServiceImpl final
    : public bamboo::BambooControlService::Service {
private:
  franka::Robot *robot_;
  franka::Model *model_;
  bamboo::controllers::JointImpedanceController *controller_;
  bamboo::interpolators::MinJerkInterpolator *interpolator_;

  std::atomic<bool> control_running_{false};

  // Control parameters
  const int traj_rate_ = 500; // Hz
  const double max_time = 1.0;
  const bool log_err_ = true; // Set to true to enable error logging (may add latency)

  Eigen::Matrix<double, 7, 1> current_q_;
  Eigen::Matrix<double, 7, 1> goal_q_;

  // For acceleration computation via finite differencing
  Eigen::Matrix<double, 7, 1> v_cmd_prev_;
  Eigen::Matrix<double, 7, 1> a_cmd_latest_;

  // Low-pass filter frequency for acceleration (matching reference driver)
  const double diff_low_pass_freq_ = 30.0; // Hz

public:
  BambooControlServiceImpl(
      franka::Robot *robot, franka::Model *model,
      bamboo::controllers::JointImpedanceController *controller,
      bamboo::interpolators::MinJerkInterpolator *interpolator)
      : robot_(robot), model_(model), controller_(controller),
        interpolator_(interpolator) {

    // Get initial robot state
    franka::RobotState init_state = robot_->readOnce();
    current_q_ = Eigen::VectorXd::Map(init_state.q.data(), 7);
    goal_q_ = current_q_;

    // Initialize velocity and acceleration tracking
    v_cmd_prev_.setZero();
    a_cmd_latest_.setZero();

    std::cout << "Initial joint positions: " << current_q_.transpose()
              << std::endl;
  }

  Status GetRobotState(ServerContext * /* context */,
                       const bamboo::RobotStateRequest * /* request */,
                       FrankaRobotStateMessage *response) override {
    try {
      // Get current robot state
      franka::RobotState current_state = robot_->readOnce();

      // Populate response message
      populateRobotState(*response, current_state);

      return Status::OK;
    } catch (const franka::Exception &e) {
      return Status(grpc::StatusCode::INTERNAL,
                    std::string("Franka exception: ") + e.what());
    } catch (const std::exception &e) {
      return Status(grpc::StatusCode::INTERNAL,
                    std::string("Exception: ") + e.what());
    }
  }

  Status ExecuteJointImpedanceTrajectory(
      ServerContext * /* context */,
      const bamboo::JointImpedanceTrajectoryRequest *request,
      FrankaCommandResponse *response) override {

    std::cout << "[GRPC] Received trajectory with " << request->waypoints_size()
              << " waypoints" << std::endl;

    try {
      // Check if already running
      if (control_running_.load()) {
        response->set_success(false);
        return Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                      "Control loop already running");
      }

      if (request->waypoints_size() == 0) {
        response->set_success(false);
        return Status(grpc::StatusCode::INVALID_ARGUMENT, "Empty trajectory");
      }

      // Parse and prepare all waypoints
      std::vector<Eigen::Matrix<double, 7, 1>> trajectory_goals;
      std::vector<Eigen::Matrix<double, 7, 1>> trajectory_velocities;
      std::vector<double> trajectory_durations;

      for (int i = 0; i < request->waypoints_size(); ++i) {
        const bamboo::TimedWaypoint &timed_waypoint = request->waypoints(i);
        const FrankaControlMessage &waypoint = timed_waypoint.waypoint();

        // Get waypoint duration
        double waypoint_duration;
        if (timed_waypoint.duration() > 0) {
          waypoint_duration = timed_waypoint.duration();
        } else if (request->default_duration() > 0) {
          waypoint_duration = request->default_duration();
        } else {
          waypoint_duration = 0.5; // Fallback default
        }

        // Get waypoint velocity
        Eigen::Matrix<double, 7, 1> waypoint_velocity =
            Eigen::Matrix<double, 7, 1>::Zero();
        if (timed_waypoint.velocity_size() == 7) {
          // Use waypoint-specific velocity if provided
          for (int j = 0; j < 7; ++j) {
            waypoint_velocity(j) = timed_waypoint.velocity(j);
          }
        } else if (request->default_velocity_size() == 7) {
          // Use default velocity if provided
          for (int j = 0; j < 7; ++j) {
            waypoint_velocity(j) = request->default_velocity(j);
          }
        }
        // If neither is provided, waypoint_velocity remains zero

        // Check for termination
        if (waypoint.termination() || global_shutdown) {
          std::cout << "[GRPC] Termination requested" << std::endl;
          break;
        }

        // Extract joint impedance goal
        FrankaJointImpedanceControllerMessage ji_msg;
        if (!waypoint.control_msg().UnpackTo(&ji_msg)) {
          response->set_success(false);
          return Status(grpc::StatusCode::INVALID_ARGUMENT,
                        "Failed to unpack joint impedance message");
        }

        // Store goal position
        Eigen::Matrix<double, 7, 1> goal;
        goal << ji_msg.goal().q1(), ji_msg.goal().q2(), ji_msg.goal().q3(),
            ji_msg.goal().q4(), ji_msg.goal().q5(), ji_msg.goal().q6(),
            ji_msg.goal().q7();

        trajectory_goals.push_back(goal);
        trajectory_velocities.push_back(waypoint_velocity);
        trajectory_durations.push_back(waypoint_duration);

        // std::cout << "[GRPC] Waypoint " << i+1 << "/" <<
        // request->waypoints_size()
        //           << " (duration: " << waypoint_duration << "s): " <<
        //           goal.transpose() << std::endl;
      }

      // Execute entire trajectory in single control call
      bool success = executeTrajectory(trajectory_goals, trajectory_velocities,
                                       trajectory_durations);
      if (!success) {
        response->set_success(false);
        return Status(grpc::StatusCode::INTERNAL,
                      "Trajectory execution failed");
      }

      std::cout << "[GRPC] Trajectory completed successfully" << std::endl;
      response->set_success(true);
      return Status::OK;

    } catch (const franka::ControlException &e) {
      std::cerr << "[GRPC] Control exception: " << e.what() << std::endl;
      response->set_success(false);
      return Status(grpc::StatusCode::INTERNAL,
                    std::string("Control exception: ") + e.what());
    } catch (const std::exception &e) {
      std::cerr << "[GRPC] Exception: " << e.what() << std::endl;
      response->set_success(false);
      return Status(grpc::StatusCode::INTERNAL,
                    std::string("Exception: ") + e.what());
    }
  }

  Status Terminate(ServerContext * /* context */,
                   const bamboo::TerminateRequest * /* request */,
                   FrankaCommandResponse *response) override {
    std::cout << "[GRPC] Terminate request received" << std::endl;
    global_shutdown = true;
    response->set_success(true);
    return Status::OK;
  }

private:
  bool
  executeTrajectory(const std::vector<Eigen::Matrix<double, 7, 1>> &goals,
                    const std::vector<Eigen::Matrix<double, 7, 1>> &velocities,
                    const std::vector<double> &durations) {
    if (goals.empty())
      return false;

    control_running_ = true;
    double control_time = 0.0;

    // Reset velocity and acceleration tracking for new trajectory
    v_cmd_prev_.setZero();
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
    goal_q_ = goals[0];
    interpolator_->Reset(control_time, current_q_, goal_q_, traj_rate_,
                         durations[0]);

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
        current_q_ = Eigen::VectorXd::Map(robot_state.q.data(), 7);

        // Check if current waypoint is complete
        double waypoint_elapsed = control_time - waypoint_start_time;
        if (waypoint_elapsed >= durations[current_waypoint]) {
          if (log_err_) {
            // Log joint error for waypoint that timed out
            Eigen::Matrix<double, 7, 1> waypoint_error = goal_q_ - current_q_;
            double waypoint_final_error_rad = waypoint_error.cwiseAbs().sum();
            // Update max error across all waypoints
            if (waypoint_final_error_rad > max_joint_error_rad) {
              max_joint_error_rad = waypoint_final_error_rad;
            }

            // Calculate end-effector position and orientation errors for completed waypoint
            // Get desired EE pose from goal joint angles
            std::array<double, 7> goal_q_array;
            Eigen::VectorXd::Map(&goal_q_array[0], 7) = goal_q_;

            // Create a temporary robot state with goal joint positions
            franka::RobotState temp_state = robot_state;
            temp_state.q = goal_q_array;

            std::array<double, 16> desired_ee_pose_array = model_->pose(franka::Frame::kEndEffector, temp_state);
            Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>> desired_ee_pose(desired_ee_pose_array.data());

            // Get current EE pose from robot state
            Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>> current_ee_pose(robot_state.O_T_EE.data());

            // Calculate position error (translation part)
            Eigen::Vector3d desired_position = desired_ee_pose.block<3, 1>(0, 3);
            Eigen::Vector3d current_position = current_ee_pose.block<3, 1>(0, 3);
            double current_ee_position_error = (desired_position - current_position).norm();
            if (current_ee_position_error > max_ee_position_error_m) {
              max_ee_position_error_m = current_ee_position_error;
            }

            // Calculate orientation error (rotation part)
            Eigen::Matrix3d desired_rotation = desired_ee_pose.block<3, 3>(0, 0);
            Eigen::Matrix3d current_rotation = current_ee_pose.block<3, 3>(0, 0);
            Eigen::Matrix3d rotation_error = desired_rotation * current_rotation.transpose();

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
            goal_q_ = goals[current_waypoint];
            interpolator_->Reset(control_time, current_q_, goal_q_, traj_rate_,
                                 durations[current_waypoint]);

            // std::cout << "[CONTROL] Moving to waypoint " << current_waypoint
            // + 1
            //           << "/" << goals.size() << std::endl;
          }
        }

        // Get interpolated desired position for current waypoint
        Eigen::Matrix<double, 7, 1> desired_q;
        interpolator_->GetNextStep(control_time, desired_q);

        // Get desired velocity for current waypoint (with bounds checking)
        Eigen::Matrix<double, 7, 1> desired_dq =
            Eigen::Matrix<double, 7, 1>::Zero();
        if (current_waypoint < velocities.size()) {
          desired_dq = velocities[current_waypoint];
        }

        // Compute desired acceleration via finite differencing (matching reference driver)
        Eigen::Matrix<double, 7, 1> desired_ddq =
            Eigen::Matrix<double, 7, 1>::Zero();
        if (dt > 0.0) {
          // Compute raw acceleration from velocity difference
          Eigen::Matrix<double, 7, 1> a_cmd_raw = (desired_dq - v_cmd_prev_) / dt;

          // Apply low-pass filter (matching reference driver line 585-587)
          for (int i = 0; i < 7; ++i) {
            a_cmd_latest_[i] = franka::lowpassFilter(
                dt, a_cmd_raw[i], a_cmd_latest_[i], diff_low_pass_freq_);
          }
          desired_ddq = a_cmd_latest_;

          // Update previous velocity for next iteration
          v_cmd_prev_ = desired_dq;
        }

        // Compute control torques
        std::array<double, 7> tau_d =
            controller_->Step(robot_state, desired_q, desired_dq, desired_ddq);

        // Apply rate limiting
        std::array<double, 7> tau_d_rate_limited = franka::limitRate(
            franka::kMaxTorqueRate, tau_d, robot_state.tau_J_d);

        // Check if all waypoints completed and robot has stopped
        if (current_waypoint >= goals.size() - 1 &&
            waypoint_elapsed >= durations[current_waypoint]) {
          Eigen::VectorXd current_dq =
              Eigen::VectorXd::Map(robot_state.dq.data(), 7);
          double velocity_norm = current_dq.norm();

          if (velocity_norm <
              0.01) { // Robot has stopped (threshold: 0.01 rad/s)
            // Calculate final waypoint errors (robot vs last goal) - robot is now still
            Eigen::Matrix<double, 7, 1> final_goal = goals.back();
            std::array<double, 7> final_goal_array;
            Eigen::VectorXd::Map(&final_goal_array[0], 7) = final_goal;

            // Get desired EE pose for final waypoint
            franka::RobotState final_temp_state = robot_state;
            final_temp_state.q = final_goal_array;
            std::array<double, 16> final_desired_ee_pose_array = model_->pose(franka::Frame::kEndEffector, final_temp_state);
            Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>> final_desired_ee_pose(final_desired_ee_pose_array.data());

            // Get current EE pose from robot state
            Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>> current_ee_pose(robot_state.O_T_EE.data());

            // Calculate final position error
            Eigen::Vector3d final_desired_position = final_desired_ee_pose.block<3, 1>(0, 3);
            Eigen::Vector3d current_position = current_ee_pose.block<3, 1>(0, 3);
            final_ee_position_error_m = (final_desired_position - current_position).norm();

            // Calculate final orientation error
            Eigen::Matrix3d final_desired_rotation = final_desired_ee_pose.block<3, 3>(0, 0);
            Eigen::Matrix3d current_rotation = current_ee_pose.block<3, 3>(0, 0);
            Eigen::Matrix3d final_rotation_error = final_desired_rotation * current_rotation.transpose();
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

      if (log_err_) {
        // Print max joint error across all waypoint final errors
        double max_joint_error_deg = max_joint_error_rad * 180.0 / M_PI;
        std::cout << "[CONTROL] Max sum of joint errors during trajectory: " << std::fixed
                  << std::setprecision(2) << max_joint_error_deg << " degrees"
                  << std::endl;

        // Print end-effector error metrics
        double max_ee_orientation_error_deg = max_ee_orientation_error_rad * 180.0 / M_PI;
        std::cout << "[CONTROL] Max EE position error during trajectory: "
                  << std::fixed << std::setprecision(4) << max_ee_position_error_m * 1000.0
                  << " mm" << std::endl;
        std::cout << "[CONTROL] Max EE orientation error during trajectory: "
                  << std::fixed << std::setprecision(2) << max_ee_orientation_error_deg
                  << " degrees" << std::endl;
      }

      // Print final waypoint errors
      double final_ee_orientation_error_deg = final_ee_orientation_error_rad * 180.0 / M_PI;
      std::cout << "[CONTROL] Final EE position error (vs last waypoint): "
                << std::fixed << std::setprecision(4) << final_ee_position_error_m * 1000.0
                << " mm" << std::endl;
      std::cout << "[CONTROL] Final EE orientation error (vs last waypoint): "
                << std::fixed << std::setprecision(2) << final_ee_orientation_error_deg
                << " degrees" << std::endl;

      return true;
    } catch (const franka::ControlException &e) {
      std::cerr << "[TRAJECTORY] Control exception: " << e.what() << std::endl;
      control_running_ = false;
      return false;
    }
  }
};

void RunServer(const std::string &server_address, franka::Robot *robot,
               franka::Model *model,
               bamboo::controllers::JointImpedanceController *controller,
               bamboo::interpolators::MinJerkInterpolator *interpolator) {

  BambooControlServiceImpl service(robot, model, controller, interpolator);

  grpc::EnableDefaultHealthCheckService(true);
  grpc::reflection::InitProtoReflectionServerBuilderPlugin();

  ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);

  std::unique_ptr<Server> server(builder.BuildAndStart());
  std::cout << "gRPC server listening on " << server_address << std::endl;

  // Wait for shutdown
  while (!global_shutdown) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::cout << "Shutting down gRPC server..." << std::endl;
  server->Shutdown();
}

int main(int argc, char **argv) {
  // Register signal handler for graceful shutdown
  std::signal(SIGINT, signalHandler);

  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <robot-ip> <grpc-port>" << std::endl;
    return -1;
  }

  const std::string robot_ip = argv[1];
  const std::string grpc_port = argv[2];
  const std::string server_address = "0.0.0.0:" + grpc_port;

  std::cout << "Bamboo Control Node (gRPC) Starting..." << std::endl;
  std::cout << "Robot IP: " << robot_ip << std::endl;
  std::cout << "gRPC Port: " << grpc_port << std::endl;

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

    // Start gRPC server
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
