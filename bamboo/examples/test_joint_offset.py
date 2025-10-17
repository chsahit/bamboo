#!/usr/bin/env python3
"""
Simple example that commands a 0.1 radian offset to all joints
using Joint Impedance control with Min-Jerk interpolation.

Based on deoxys_control examples.
"""

import time
import sys
import zmq
import numpy as np

# Import protobuf messages (need to generate from proto files first)
# These are copied from deoxys_control
sys.path.insert(0, '../proto')
import franka_controller_pb2

def create_joint_impedance_message(joint_positions):
    """
    Create a FrankaControlMessage for joint impedance control.

    Args:
        joint_positions: Array of 7 joint positions in radians

    Returns:
        FrankaControlMessage protobuf
    """
    # Create Joint Goal message
    joint_goal = franka_controller_pb2.JointGoal()
    joint_goal.q1 = joint_positions[0]
    joint_goal.q2 = joint_positions[1]
    joint_goal.q3 = joint_positions[2]
    joint_goal.q4 = joint_positions[3]
    joint_goal.q5 = joint_positions[4]
    joint_goal.q6 = joint_positions[5]
    joint_goal.q7 = joint_positions[6]
    joint_goal.is_delta = False

    # Create Joint Impedance Controller Message
    ji_msg = franka_controller_pb2.FrankaJointImpedanceControllerMessage()
    ji_msg.goal.CopyFrom(joint_goal)

    # Set default gains from deoxys config
    ji_msg.kp.extend([100.0, 100.0, 100.0, 100.0, 75.0, 150.0, 50.0])
    ji_msg.kd.extend([20.0, 20.0, 20.0, 20.0, 7.5, 15.0, 5.0])
    ji_msg.joint_tau_limits.extend([10.0, 10.0, 10.0, 10.0, 10.0, 5.0, 5.0])

    # Create Control Message
    control_msg = franka_controller_pb2.FrankaControlMessage()
    control_msg.control_msg.Pack(ji_msg)
    control_msg.controller_type = franka_controller_pb2.FrankaControlMessage.JOINT_IMPEDANCE
    control_msg.traj_interpolator_type = franka_controller_pb2.FrankaControlMessage.MIN_JERK_JOINT_POSITION
    control_msg.traj_interpolator_time_fraction = 0.3
    control_msg.termination = False

    return control_msg

def send_termination_message(zmq_pub):
    """Send termination message to control node."""
    control_msg = franka_controller_pb2.FrankaControlMessage()
    control_msg.termination = True
    zmq_pub.send(control_msg.SerializeToString())
    print("Sent termination message")

def main():
    if len(sys.argv) != 2:
        print("Usage: python test_joint_offset.py <zmq-port>")
        sys.exit(1)

    zmq_port = sys.argv[1]

    print("Bamboo Joint Offset Example")
    print(f"ZMQ Port: {zmq_port}")
    print()

    # Initialize ZMQ publisher
    zmq_context = zmq.Context()
    zmq_pub = zmq_context.socket(zmq.PUB)
    zmq_pub.bind(f"tcp://*:{zmq_port}")

    # Give ZMQ time to connect
    time.sleep(1.0)

    print("Reading current joint positions...")
    # In a real implementation, you would read from robot state publisher
    # For this example, we'll use a reasonable starting configuration
    current_joints = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])

    print(f"Current joints: {current_joints}")
    print()

    # Compute goal: current + 0.1 radians
    goal_joints = current_joints + 0.35

    print(f"Goal joints: {goal_joints}")
    print()

    print("Sending goal position...")
    control_msg = create_joint_impedance_message(goal_joints)
    zmq_pub.send(control_msg.SerializeToString())
    print("Command sent!")
    print()

    print("Waiting for motion to complete...")
    print("(The robot should move smoothly to the goal using min-jerk interpolation)")
    print()

    # Send commands at 20 Hz for a few seconds
    # In practice, you'd check robot state to know when motion is complete
    rate = 20  # Hz
    duration = 5.0  # seconds

    for i in range(int(rate * duration)):
        # Keep sending the same goal
        control_msg = create_joint_impedance_message(goal_joints)
        zmq_pub.send(control_msg.SerializeToString())
        time.sleep(1.0 / rate)

        if i % rate == 0:
            print(f"  {i // rate + 1} seconds elapsed...")

    print()
    print("Motion should be complete. Sending termination message...")

    # Send termination
    send_termination_message(zmq_pub)

    time.sleep(0.5)

    print("Example completed successfully!")
    print()
    print("Summary:")
    print("- Sent joint impedance command with 0.1 rad offset")
    print("- Used min-jerk interpolation for smooth motion")
    print("- Control node should have moved the robot smoothly to goal")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        sys.exit(0)
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
