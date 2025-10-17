// Minimal test program for joint impedance control with min-jerk interpolation
// Control architecture inspired by deoxys_control

#include <array>
#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include "joint_impedance_controller.h"
#include "min_jerk_interpolator.h"

// Helper function to print joint arrays
template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
    ostream << "[";
    for (size_t i = 0; i < N - 1; i++) {
        ostream << array[i] << ", ";
    }
    ostream << array[N - 1] << "]";
    return ostream;
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }

    // Policy parameters
    const int policy_rate = 20;  // Hz - policy outputs commands at this rate
    const int traj_rate = 500;   // Hz - interpolation frequency
    const double traj_interpolator_time_fraction = 0.3;  // Interpolation time as fraction of policy period

    try {
        // Connect to robot
        std::cout << "Connecting to robot at " << argv[1] << "..." << std::endl;
        franka::Robot robot(argv[1]);

        // Load robot model
        franka::Model model = robot.loadModel();

        // Read initial state
        franka::RobotState initial_state = robot.readOnce();
        std::cout << "Current joint positions [rad]: " << initial_state.q << std::endl;

        // Compute goal position: current_q + 0.1 for all joints
        Eigen::Matrix<double, 7, 1> q_current =
            Eigen::VectorXd::Map(initial_state.q.data(), 7);
        Eigen::Matrix<double, 7, 1> q_goal = q_current;
        for (int i = 0; i < 7; i++) {
            q_goal[i] += 0.1;  // Add 0.1 rad to each joint
        }

        std::cout << "Goal joint positions [rad]: " << q_goal.transpose() << std::endl;
        std::cout << std::endl;

        std::cout << "WARNING: This example will move the robot!" << std::endl
                  << "Please make sure to have the user stop button at hand!" << std::endl
                  << "Press Enter to continue..." << std::endl;
        std::cin.ignore();

        // Set collision behavior (from deoxys and libfranka examples)
        robot.setCollisionBehavior(
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}
        );

        // Create controller and interpolator
        JointImpedanceController controller(&model);
        MinJerkInterpolator interpolator;

        // Control loop variables
        double time = 0.0;
        bool interpolator_initialized = false;
        std::atomic_bool running{true};

        // Policy thread - runs at 20 Hz and sends constant goal
        std::thread policy_thread([&]() {
            while (running) {
                // In this simple test, the policy just maintains the same goal
                // In a real application, this could update q_goal based on some policy logic

                std::this_thread::sleep_for(
                    std::chrono::milliseconds(1000 / policy_rate)
                );
            }
        });

        // Control callback - adapted from deoxys torque_callback.h
        // See deoxys_control/deoxys/franka-interface/include/utils/control_callbacks/torque_callback.h:84-142
        auto control_callback = [&](const franka::RobotState& robot_state,
                                     franka::Duration period) -> franka::Torques {

            // Initialize interpolator on first call (line 112-119 in deoxys torque_callback.h)
            if (time == 0.0) {
                Eigen::Matrix<double, 7, 1> q_start =
                    Eigen::VectorXd::Map(robot_state.q.data(), 7);

                interpolator.Reset(
                    0.0,
                    q_start,
                    q_goal,
                    policy_rate,
                    traj_rate,
                    traj_interpolator_time_fraction
                );
            }

            // Update time (line 120 in deoxys torque_callback.h)
            time += period.toSec();

            // Get interpolated desired joint position (line 124-125 in deoxys torque_callback.h)
            Eigen::Matrix<double, 7, 1> desired_q;
            interpolator.GetNextStep(time, desired_q);

            // Compute control torques (line 129 in deoxys torque_callback.h)
            std::array<double, 7> tau_d = controller.Step(robot_state, desired_q);

            // Apply rate limiting for safety (line 131-132 in deoxys torque_callback.h)
            std::array<double, 7> tau_d_rate_limited =
                franka::limitRate(franka::kMaxTorqueRate, tau_d, robot_state.tau_J_d);

            // Check if motion is finished (all joints within threshold)
            const double position_threshold = 0.01;  // rad
            const double velocity_threshold = 0.01;  // rad/s
            bool motion_finished = true;

            for (size_t i = 0; i < 7; i++) {
                if (std::abs(q_goal[i] - robot_state.q[i]) > position_threshold ||
                    std::abs(robot_state.dq[i]) > velocity_threshold) {
                    motion_finished = false;
                    break;
                }
            }

            if (motion_finished) {
                std::cout << "\nMotion finished!" << std::endl;
                std::cout << "Final joint positions [rad]: " << robot_state.q << std::endl;
                std::cout << "Goal joint positions [rad]:  ";
                for (int i = 0; i < 7; i++) {
                    std::cout << q_goal[i];
                    if (i < 6) std::cout << ", ";
                }
                std::cout << std::endl;

                running = false;
                // Wrap std::array in franka::Torques before calling MotionFinished
                franka::Torques torques(tau_d_rate_limited);
                return franka::MotionFinished(torques);
            }

            return franka::Torques(tau_d_rate_limited);
        };

        // Start control loop
        std::cout << "Starting joint impedance control with min-jerk interpolation..." << std::endl;
        std::cout << "Policy rate: " << policy_rate << " Hz" << std::endl;
        std::cout << "Trajectory rate: " << traj_rate << " Hz" << std::endl;
        std::cout << "Interpolation time fraction: " << traj_interpolator_time_fraction << std::endl;
        std::cout << std::endl;

        robot.control(control_callback);

        // Clean up
        running = false;
        if (policy_thread.joinable()) {
            policy_thread.join();
        }

        std::cout << "Control completed successfully!" << std::endl;

    } catch (const franka::Exception& e) {
        std::cerr << "Franka exception: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
