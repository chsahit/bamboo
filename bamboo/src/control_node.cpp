// Bamboo Control Node - Simplified version of deoxys control node
// Supports only Joint Impedance control with Min-Jerk interpolation
// Based on deoxys_control/deoxys/franka-interface/src/franka_control_node.cpp

#include <atomic>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <signal.h>
#include <csignal>
#include <exception>
#include <stdexcept>
#include <mutex>

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include <zmq.hpp>
#include <Eigen/Dense>

#include "controllers/joint_impedance_controller.h"
#include "interpolators/min_jerk_interpolator.h"

// Protobuf messages - copied from deoxys
#include "franka_controller.pb.h"

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

int main(int argc, char** argv) {
    // Register signal handler for graceful shutdown
    std::signal(SIGINT, signalHandler);

    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <robot-ip> <zmq-port>" << std::endl;
        return -1;
    }

    const std::string robot_ip = argv[1];
    const std::string zmq_port = argv[2];

    // Control parameters - matching deoxys defaults
    const int policy_rate = 20;  // Hz
    const int traj_rate = 500;   // Hz
    const double traj_interpolator_time_fraction = 0.1;

    std::cout << "Bamboo Control Node Starting..." << std::endl;
    std::cout << "Robot IP: " << robot_ip << std::endl;
    std::cout << "ZMQ Port: " << zmq_port << std::endl;
    std::cout << "Policy rate: " << policy_rate << " Hz" << std::endl;
    std::cout << "Traj rate: " << traj_rate << " Hz" << std::endl;

    try {
        // Initialize ZMQ subscriber for control commands
        zmq::context_t zmq_context(1);
        zmq::socket_t zmq_sub(zmq_context, ZMQ_SUB);
        zmq_sub.bind("tcp://*:" + zmq_port);
        zmq_sub.set(zmq::sockopt::subscribe, "");
        zmq_sub.set(zmq::sockopt::rcvtimeo, 10);  // 10ms timeout for non-blocking

        // State publisher (like deoxys) - publishes on next port
        zmq::socket_t zmq_state_pub(zmq_context, ZMQ_PUB);
        std::string state_pub_port = std::to_string(std::stoi(zmq_port) + 1);
        zmq_state_pub.bind("tcp://*:" + state_pub_port);
        std::cout << "Robot state publisher on port: " << state_pub_port << std::endl;

        // Command response publisher - publishes on port+2
        zmq::socket_t zmq_response_pub(zmq_context, ZMQ_PUB);
        std::string response_pub_port = std::to_string(std::stoi(zmq_port) + 2);
        zmq_response_pub.bind("tcp://*:" + response_pub_port);
        std::cout << "Command response publisher on port: " << response_pub_port << std::endl;

        // Connect to robot
        std::cout << "Connecting to robot..." << std::endl;
        franka::Robot robot(robot_ip);
        robot.automaticErrorRecovery();

        // Set collision behavior (from deoxys line 241-245)
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

        // Get initial state
        franka::RobotState init_state = robot.readOnce();
        Eigen::Matrix<double, 7, 1> current_q =
            Eigen::VectorXd::Map(init_state.q.data(), 7);
        Eigen::Matrix<double, 7, 1> goal_q = current_q;

        std::cout << "Initial joint positions: " << current_q.transpose() << std::endl;
        std::cout << "Ready to receive commands on port " << zmq_port << std::endl;
        std::cout << "Waiting for first command..." << std::endl;

        // Control state
        std::atomic_bool running{false};
        std::atomic_bool termination{false};
        double control_time = 0.0;
        int no_msg_counter = 0;
        bool trajectory_completed = false;

        // State publisher rate (like deoxys)
        const int state_pub_rate = 100;  // Hz

        // State publisher thread - continuously publishes robot state like deoxys
        std::thread state_pub_thread([&]() {
            try {
                while (!termination && !global_shutdown) {
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(int(1.0 / state_pub_rate * 1000.0))
                    );
                    
                    try {
                        // Get current robot state
                        franka::RobotState current_state = robot.readOnce();

                        // Create and populate state message
                        FrankaRobotStateMessage state_msg;
                        populateRobotState(state_msg, current_state);

                        // Serialize and publish
                        std::string state_str;
                        if (!state_msg.SerializeToString(&state_str)) {
                            std::cerr << "[STATE_PUB] Failed to serialize state message" << std::endl;
                            continue;
                        }
                        zmq::message_t state_zmq_msg(state_str.size());
                        memcpy(state_zmq_msg.data(), state_str.c_str(), state_str.size());

                        auto result = zmq_state_pub.send(state_zmq_msg, zmq::send_flags::dontwait);
                        if (!result) {
                            std::cerr << "[STATE_PUB] Failed to send state message" << std::endl;
                        }

                    } catch (const franka::Exception& e) {
#ifdef DEBUG
                        std::cerr << "[STATE_PUB] Franka exception (198): " << e.what() << std::endl;
#endif
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    } catch (const std::exception& e) {
                        std::cerr << "[STATE_PUB] Exception: " << e.what() << std::endl;
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                }
            } catch (...) {
                std::cerr << "[STATE_PUB] Unhandled exception in state publisher thread" << std::endl;
                setThreadException(std::current_exception());
            }
            std::cout << "[STATE_PUB] State publisher thread exiting" << std::endl;
        });

        // Message receiving thread - based on deoxys line 339-573
        std::thread msg_thread([&]() {
            try {
                while (!termination && !global_shutdown) {
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(int(1.0 / policy_rate * 1000.0))
                    );

                    try {
                        // Try to receive message (non-blocking)
                        zmq::message_t zmq_msg;
                        auto result = zmq_sub.recv(zmq_msg, zmq::recv_flags::dontwait);

                        if (!result || zmq_msg.size() == 0) {
                            if (running) {
                                no_msg_counter++;
                                if (no_msg_counter >= 20) {
                                    std::cout << "[MSG_THREAD] No messages received for 20 steps, trajectory completed" << std::endl;
                                    running = false;

                                    // Send completion message - trajectory is done
                                    FrankaCommandResponse response;
                                    response.set_success(true);
                                    std::string response_str;
                                    if (!response.SerializeToString(&response_str)) {
                                        std::cerr << "[MSG_THREAD] Failed to serialize completion response" << std::endl;
                                        continue;
                                    }
                                    zmq::message_t response_zmq_msg(response_str.size());
                                    memcpy(response_zmq_msg.data(), response_str.c_str(), response_str.size());
                                    auto send_result = zmq_response_pub.send(response_zmq_msg, zmq::send_flags::dontwait);
                                    if (!send_result) {
                                        std::cerr << "[MSG_THREAD] Failed to send completion message" << std::endl;
                                    } else {
                                        std::cout << "[MSG_THREAD] Sent trajectory completion message" << std::endl;
                                    }
                                }
                            }
                            continue;
                        }

                        // Parse protobuf message
                        FrankaControlMessage control_msg;
                        std::string msg_str(static_cast<char*>(zmq_msg.data()), zmq_msg.size());

                        
                        if (!control_msg.ParseFromString(msg_str)) {
#ifdef DEBUG
                            std::cerr << "[MSG_THREAD] Failed to parse control message" << std::endl;
#endif
                            continue;
                        }

                        no_msg_counter = 0;

                        // Check for termination
                        if (control_msg.termination()) {
                            std::cout << "[MSG_THREAD] Termination message received" << std::endl;
                            running = false;
                            termination = true;
                            continue;
                        }

                        // Extract joint impedance goal (based on deoxys line 525-528)
                        FrankaJointImpedanceControllerMessage ji_msg;
                        if (!control_msg.control_msg().UnpackTo(&ji_msg)) {
                            std::cerr << "[MSG_THREAD] Failed to unpack joint impedance message" << std::endl;
                            continue;
                        }

                        // Update goal position
                        goal_q << ji_msg.goal().q1(), ji_msg.goal().q2(), ji_msg.goal().q3(),
                                 ji_msg.goal().q4(), ji_msg.goal().q5(), ji_msg.goal().q6(),
                                 ji_msg.goal().q7();

                        std::cout << "[MSG_THREAD] New goal received: " << goal_q.transpose() << std::endl;

                        // Reset completion flag
                        trajectory_completed = false;

                        // Reset interpolator (based on deoxys line 543-546)
                        interpolator.Reset(
                            control_time,
                            current_q,
                            goal_q,
                            policy_rate,
                            traj_rate,
                            traj_interpolator_time_fraction
                        );

                        running = true;

                    } catch (const std::exception& e) {
                        std::cerr << "[MSG_THREAD] Exception in message processing: " << e.what() << std::endl;
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                }
            } catch (...) {
                std::cerr << "[MSG_THREAD] Unhandled exception in message thread" << std::endl;
                setThreadException(std::current_exception());
            }
            std::cout << "[MSG_THREAD] Message thread exiting" << std::endl;
        });

        // Main control loop - based on deoxys line 575-634
        std::cout << "Starting control loop..." << std::endl;

        while (!termination && !global_shutdown) {
            // Check for thread exceptions
            auto thread_ex = getThreadException();
            if (thread_ex) {
                std::cerr << "[MAIN] Thread exception detected, shutting down" << std::endl;
                break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));

            if (!running) {
                continue;
            }

            // Control callback - based on deoxys torque_callback.h
            control_time = 0.0;

            auto control_callback = [&](const franka::RobotState& robot_state,
                                        franka::Duration period) -> franka::Torques {
                try {
                    // Update time
                    control_time += period.toSec();

                    // Update current position
                    current_q = Eigen::VectorXd::Map(robot_state.q.data(), 7);

                    // Get interpolated desired position
                    Eigen::Matrix<double, 7, 1> desired_q;
                    interpolator.GetNextStep(control_time, desired_q);

                    // Compute control torques
                    std::array<double, 7> tau_d = controller.Step(robot_state, desired_q);

                    // Apply rate limiting
                    std::array<double, 7> tau_d_rate_limited =
                        franka::limitRate(franka::kMaxTorqueRate, tau_d, robot_state.tau_J_d);

                    // Check if we should stop (no running flag, close to goal, or shutdown signal)
                    if (!running || global_shutdown) {
                        return franka::MotionFinished(franka::Torques(tau_d_rate_limited));
                    }

                    return franka::Torques(tau_d_rate_limited);
                } catch (const std::exception& e) {
                    std::cerr << "[CONTROL_CALLBACK] Exception: " << e.what() << std::endl;
                    std::array<double, 7> zero_torques = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                    return franka::MotionFinished(franka::Torques(zero_torques));
                } catch (...) {
                    std::cerr << "[CONTROL_CALLBACK] Unknown exception" << std::endl;
                    std::array<double, 7> zero_torques = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                    return franka::MotionFinished(franka::Torques(zero_torques));
                }
            };

            // Execute control
            try {
                robot.control(control_callback);
                std::cout << "[MAIN] Control motion completed. Ready for new commands..." << std::endl;
            } catch (const franka::ControlException& e) {
                std::cerr << "[MAIN] Control exception: " << e.what() << std::endl;

                // Send error response
                try {
                    FrankaCommandResponse response;
                    response.set_success(false);

                    std::string response_str;
                    if (!response.SerializeToString(&response_str)) {
                        std::cerr << "[MAIN] Failed to serialize error response" << std::endl;
                    } else {
                        zmq::message_t response_zmq_msg(response_str.size());
                        memcpy(response_zmq_msg.data(), response_str.c_str(), response_str.size());
                        auto send_result = zmq_response_pub.send(response_zmq_msg, zmq::send_flags::dontwait);
                        if (!send_result) {
                            std::cerr << "[MAIN] Failed to send error response" << std::endl;
                        } else {
                            std::cout << "[MAIN] Sent error completion message" << std::endl;
                        }
                    }
                } catch (const std::exception& e) {
                    std::cerr << "[MAIN] Exception while sending error response: " << e.what() << std::endl;
                }

                running = false;
            } catch (const std::exception& e) {
                std::cerr << "[MAIN] Unexpected exception in control loop: " << e.what() << std::endl;
                running = false;
            }

            // Reset time after control loop finishes
            control_time = 0.0;
        }

        // Cleanup - ensure threads exit gracefully
        std::cout << "[MAIN] Starting cleanup..." << std::endl;
        termination = true;  // Signal threads to exit

        // Wait for threads to complete
        try {
            if (msg_thread.joinable()) {
                msg_thread.join();
                std::cout << "[MAIN] Message thread joined" << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "[MAIN] Exception joining message thread: " << e.what() << std::endl;
        }

        try {
            if (state_pub_thread.joinable()) {
                state_pub_thread.join();
                std::cout << "[MAIN] State publisher thread joined" << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "[MAIN] Exception joining state publisher thread: " << e.what() << std::endl;
        }

        // Check if there was a thread exception
        auto thread_ex = getThreadException();
        if (thread_ex) {
            try {
                std::rethrow_exception(thread_ex);
            } catch (const std::exception& e) {
                std::cerr << "[MAIN] Thread exception: " << e.what() << std::endl;
            } catch (...) {
                std::cerr << "[MAIN] Unknown thread exception" << std::endl;
            }
        }

        if (global_shutdown) {
            std::cout << "Control node shutdown by user (Ctrl+C)" << std::endl;
        } else {
            std::cout << "Control node terminated successfully" << std::endl;
        }

    } catch (const franka::Exception& e) {
        std::cerr << "Franka exception: " << e.what() << std::endl;
        return -1;
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
