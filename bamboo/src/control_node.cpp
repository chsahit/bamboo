// Joint Impedance control with Min-Jerk interpolation

#include <atomic>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <signal.h>
#include <csignal>
#include <exception>
#include <stdexcept>
#include <mutex>
#include <queue>

#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>
#include <grpcpp/ext/proto_server_reflection_plugin.h>

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include <Eigen/Dense>

#include "controllers/joint_impedance_controller.h"
#include "interpolators/min_jerk_interpolator.h"

// Protobuf messages
#include "franka_controller.pb.h"
#include "bamboo_service.grpc.pb.h"

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
void populateRobotState(FrankaRobotStateMessage& robot_state_msg, const franka::RobotState& robot_state) {
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
    FrankaRobotStateMessage::Duration* time = robot_state_msg.mutable_time();
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
class BambooControlServiceImpl final : public bamboo::BambooControlService::Service {
private:
    franka::Robot* robot_;
    franka::Model* model_;
    bamboo::controllers::JointImpedanceController* controller_;
    bamboo::interpolators::MinJerkInterpolator* interpolator_;

    std::atomic<bool> control_running_{false};

    // Control parameters
    const int traj_rate_ = 500;   // Hz
    const double max_time = 1.0;


    Eigen::Matrix<double, 7, 1> current_q_;
    Eigen::Matrix<double, 7, 1> goal_q_;

public:
    BambooControlServiceImpl(franka::Robot* robot,
                            franka::Model* model,
                            bamboo::controllers::JointImpedanceController* controller,
                            bamboo::interpolators::MinJerkInterpolator* interpolator)
        : robot_(robot), model_(model), controller_(controller), interpolator_(interpolator) {

        // Get initial robot state
        franka::RobotState init_state = robot_->readOnce();
        current_q_ = Eigen::VectorXd::Map(init_state.q.data(), 7);
        goal_q_ = current_q_;

        std::cout << "Initial joint positions: " << current_q_.transpose() << std::endl;
    }

    Status GetRobotState(ServerContext* /* context */,
                        const bamboo::RobotStateRequest* /* request */,
                        FrankaRobotStateMessage* response) override {
        try {
            // Get current robot state
            franka::RobotState current_state = robot_->readOnce();

            // Populate response message
            populateRobotState(*response, current_state);

            return Status::OK;
        } catch (const franka::Exception& e) {
            return Status(grpc::StatusCode::INTERNAL,
                         std::string("Franka exception: ") + e.what());
        } catch (const std::exception& e) {
            return Status(grpc::StatusCode::INTERNAL,
                         std::string("Exception: ") + e.what());
        }
    }

    Status ExecuteJointImpedanceTrajectory(
            ServerContext* /* context */,
            const bamboo::JointImpedanceTrajectoryRequest* request,
            FrankaCommandResponse* response) override {

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
                return Status(grpc::StatusCode::INVALID_ARGUMENT,
                            "Empty trajectory");
            }

            // Parse and prepare all waypoints
            std::vector<Eigen::Matrix<double, 7, 1>> trajectory_goals;
            std::vector<double> trajectory_durations;

            for (int i = 0; i < request->waypoints_size(); ++i) {
                const bamboo::TimedWaypoint& timed_waypoint = request->waypoints(i);
                const FrankaControlMessage& waypoint = timed_waypoint.waypoint();

                // Get waypoint duration
                double waypoint_duration;
                if (timed_waypoint.duration() > 0) {
                    waypoint_duration = timed_waypoint.duration();
                } else if (request->default_duration() > 0) {
                    waypoint_duration = request->default_duration();
                } else {
                    waypoint_duration = 1.0; // Fallback default
                }

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
                trajectory_durations.push_back(waypoint_duration);

                std::cout << "[GRPC] Waypoint " << i+1 << "/" << request->waypoints_size()
                          << " (duration: " << waypoint_duration << "s): " << goal.transpose() << std::endl;
            }

            // Execute entire trajectory in single control call
            bool success = executeTrajectory(trajectory_goals, trajectory_durations);
            if (!success) {
                response->set_success(false);
                return Status(grpc::StatusCode::INTERNAL,
                            "Trajectory execution failed");
            }

            std::cout << "[GRPC] Trajectory completed successfully" << std::endl;
            response->set_success(true);
            return Status::OK;

        } catch (const franka::ControlException& e) {
            std::cerr << "[GRPC] Control exception: " << e.what() << std::endl;
            response->set_success(false);
            return Status(grpc::StatusCode::INTERNAL,
                         std::string("Control exception: ") + e.what());
        } catch (const std::exception& e) {
            std::cerr << "[GRPC] Exception: " << e.what() << std::endl;
            response->set_success(false);
            return Status(grpc::StatusCode::INTERNAL,
                         std::string("Exception: ") + e.what());
        }
    }

    Status Terminate(ServerContext* /* context */,
                    const bamboo::TerminateRequest* /* request */,
                    FrankaCommandResponse* response) override {
        std::cout << "[GRPC] Terminate request received" << std::endl;
        global_shutdown = true;
        response->set_success(true);
        return Status::OK;
    }

private:
    bool executeTrajectory(const std::vector<Eigen::Matrix<double, 7, 1>>& goals,
                          const std::vector<double>& durations) {
        if (goals.empty()) return false;

        control_running_ = true;
        double control_time = 0.0;

        // Current waypoint tracking
        std::size_t current_waypoint = 0;
        double waypoint_start_time = 0.0;

        // Initialize first waypoint
        goal_q_ = goals[0];
        interpolator_->Reset(control_time, current_q_, goal_q_, traj_rate_, durations[0]);

        std::cout << "[CONTROL] Starting trajectory with " << goals.size() << " waypoints" << std::endl;

        // Single control callback for entire trajectory
        auto control_callback = [&](const franka::RobotState& robot_state,
                                    franka::Duration period) -> franka::Torques {
            try {
                // Update time
                control_time += period.toSec();

                // Update current position
                current_q_ = Eigen::VectorXd::Map(robot_state.q.data(), 7);

                // Check if current waypoint is complete
                double waypoint_elapsed = control_time - waypoint_start_time;
                if (waypoint_elapsed >= durations[current_waypoint]) {
                    // Move to next waypoint
                    current_waypoint++;

                    if (current_waypoint >= goals.size()) {
                        // All waypoints completed
                        std::cout << "[CONTROL] All waypoints completed" << std::endl;
                        return franka::MotionFinished(franka::Torques({0,0,0,0,0,0,0}));
                    }

                    // Setup next waypoint
                    waypoint_start_time = control_time;
                    goal_q_ = goals[current_waypoint];
                    interpolator_->Reset(control_time, current_q_, goal_q_, traj_rate_, durations[current_waypoint]);

                    std::cout << "[CONTROL] Moving to waypoint " << current_waypoint + 1
                              << "/" << goals.size() << std::endl;
                }

                // Get interpolated desired position for current waypoint
                Eigen::Matrix<double, 7, 1> desired_q;
                interpolator_->GetNextStep(control_time, desired_q);

                // Compute control torques
                std::array<double, 7> tau_d = controller_->Step(robot_state, desired_q);

                // Apply rate limiting
                std::array<double, 7> tau_d_rate_limited =
                    franka::limitRate(franka::kMaxTorqueRate, tau_d, robot_state.tau_J_d);

                // Check if we should stop
                if (!control_running_ || global_shutdown) {
                    return franka::MotionFinished(franka::Torques(tau_d_rate_limited));
                }

                return franka::Torques(tau_d_rate_limited);
            } catch (const std::exception& e) {
                std::cerr << "[CONTROL_CALLBACK] Exception: " << e.what() << std::endl;
                std::array<double, 7> zero_torques = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                return franka::MotionFinished(franka::Torques(zero_torques));
            }
        };

        try {
            // Execute control
            robot_->control(control_callback);
            control_running_ = false;
            return true;
        } catch (const franka::ControlException& e) {
            std::cerr << "[TRAJECTORY] Control exception: " << e.what() << std::endl;
            control_running_ = false;
            return false;
        }
    }

};

void RunServer(const std::string& server_address,
               franka::Robot* robot,
               franka::Model* model,
               bamboo::controllers::JointImpedanceController* controller,
               bamboo::interpolators::MinJerkInterpolator* interpolator) {

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

int main(int argc, char** argv) {
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
            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}
        );

        // Load model
        franka::Model model = robot.loadModel();

        // Create controller and interpolator
        bamboo::controllers::JointImpedanceController controller(&model);
        bamboo::interpolators::MinJerkInterpolator interpolator;

        // Start gRPC server
        RunServer(server_address, &robot, &model, &controller, &interpolator);

        std::cout << "Control node terminated successfully" << std::endl;

    } catch (const franka::Exception& e) {
        std::cerr << "Franka exception: " << e.what() << std::endl;
        return -1;
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
