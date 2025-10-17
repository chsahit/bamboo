// Bamboo Control Node - Simplified version of deoxys control node
// Supports only Joint Impedance control with Min-Jerk interpolation
// Based on deoxys_control/deoxys/franka-interface/src/franka_control_node.cpp

#include <atomic>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

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

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <robot-ip> <zmq-port>" << std::endl;
        return -1;
    }

    const std::string robot_ip = argv[1];
    const std::string zmq_port = argv[2];

    // Control parameters - matching deoxys defaults
    const int policy_rate = 20;  // Hz
    const int traj_rate = 500;   // Hz
    const double traj_interpolator_time_fraction = 0.3;

    std::cout << "Bamboo Control Node Starting..." << std::endl;
    std::cout << "Robot IP: " << robot_ip << std::endl;
    std::cout << "ZMQ Port: " << zmq_port << std::endl;
    std::cout << "Policy rate: " << policy_rate << " Hz" << std::endl;
    std::cout << "Traj rate: " << traj_rate << " Hz" << std::endl;

    try {
        // Initialize ZMQ subscriber - based on deoxys zmq_utils
        zmq::context_t zmq_context(1);
        zmq::socket_t zmq_sub(zmq_context, ZMQ_SUB);
        zmq_sub.connect("tcp://localhost:" + zmq_port);
        zmq_sub.set(zmq::sockopt::subscribe, "");
        zmq_sub.set(zmq::sockopt::rcvtimeo, 10);  // 10ms timeout for non-blocking

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

        // Control state
        std::atomic_bool running{false};
        std::atomic_bool termination{false};
        double control_time = 0.0;
        int no_msg_counter = 0;

        // Message receiving thread - based on deoxys line 339-573
        std::thread msg_thread([&]() {
            while (!termination) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(int(1.0 / policy_rate * 1000.0))
                );

                // Try to receive message (non-blocking)
                zmq::message_t zmq_msg;
                auto result = zmq_sub.recv(zmq_msg, zmq::recv_flags::dontwait);

                if (!result || zmq_msg.size() == 0) {
                    if (running) {
                        no_msg_counter++;
                        if (no_msg_counter >= 20) {
                            std::cout << "No messages received for 20 steps, stopping" << std::endl;
                            running = false;
                            termination = true;
                        }
                    }
                    continue;
                }

                // Parse protobuf message
                FrankaControlMessage control_msg;
                std::string msg_str(static_cast<char*>(zmq_msg.data()), zmq_msg.size());

                if (!control_msg.ParseFromString(msg_str)) {
                    std::cerr << "Failed to parse control message" << std::endl;
                    continue;
                }

                no_msg_counter = 0;

                // Check for termination
                if (control_msg.termination()) {
                    std::cout << "Termination message received" << std::endl;
                    running = false;
                    termination = true;
                    continue;
                }

                // Extract joint impedance goal (based on deoxys line 525-528)
                FrankaJointImpedanceControllerMessage ji_msg;
                if (!control_msg.control_msg().UnpackTo(&ji_msg)) {
                    std::cerr << "Failed to unpack joint impedance message" << std::endl;
                    continue;
                }

                // Update goal position
                goal_q << ji_msg.goal().q1(), ji_msg.goal().q2(), ji_msg.goal().q3(),
                         ji_msg.goal().q4(), ji_msg.goal().q5(), ji_msg.goal().q6(),
                         ji_msg.goal().q7();

                std::cout << "New goal received: " << goal_q.transpose() << std::endl;

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
            }
        });

        // Main control loop - based on deoxys line 575-634
        std::cout << "Starting control loop..." << std::endl;

        while (!termination) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

            if (!running) {
                continue;
            }

            // Control callback - based on deoxys torque_callback.h
            control_time = 0.0;

            auto control_callback = [&](const franka::RobotState& robot_state,
                                        franka::Duration period) -> franka::Torques {

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

                // Check if we should stop (no running flag or close to goal)
                if (!running) {
                    return franka::MotionFinished(franka::Torques(tau_d_rate_limited));
                }

                return franka::Torques(tau_d_rate_limited);
            };

            // Execute control
            try {
                robot.control(control_callback);
            } catch (const franka::ControlException& e) {
                std::cerr << "Control exception: " << e.what() << std::endl;
                running = false;
            }

            // Reset time after control loop finishes
            control_time = 0.0;
        }

        // Cleanup
        msg_thread.join();
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
