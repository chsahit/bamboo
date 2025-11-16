#!/usr/bin/env python3

"""
Example script demonstrating how to receive robot joint angles from bamboo control node.
This script subscribes to the continuous robot state stream
"""

import zmq
import sys
import os
import time

# Add the build directory to path to import protobuf messages
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'build', 'proto_generated'))

try:
    import franka_controller_pb2
except ImportError:
    print("Error: Could not import protobuf messages. Make sure the project is built.")
    print("Run 'make' in the build directory first.")
    sys.exit(1)

def subscribe_robot_state(control_port="5556", num_samples=10):
    """
    Subscribe to the robot state stream from the control node.

    Args:
        control_port: Control port of the control node (state published on port+1)
        num_samples: Number of state samples to receive
    """
    # Set up ZMQ context and socket
    context = zmq.Context()

    # Subscriber to receive continuous state stream (control node publishes on port+1)
    sub_socket = context.socket(zmq.SUB)
    state_port = str(int(control_port) + 1)
    sub_socket.connect(f"tcp://localhost:{state_port}")
    sub_socket.setsockopt(zmq.SUBSCRIBE, b"")  # Subscribe to all messages
    sub_socket.setsockopt(zmq.RCVTIMEO, 5000)  # 5 second timeout

    print(f"Subscribing to robot state stream on port {state_port}...")
    print(f"Control node should be running on port {control_port}")
    print(f"Receiving {num_samples} samples...\n")

    try:
        for i in range(num_samples):
            # Wait for state message
            response_data = sub_socket.recv()

            # Parse response
            state_msg = franka_controller_pb2.FrankaRobotStateMessage()
            state_msg.ParseFromString(response_data)

            # Print joint angles
            print(f"Sample {i+1}/{num_samples}:")
            print(f"Joint angles ({len(state_msg.q)} joints):")
            for j, q in enumerate(state_msg.q):
                print(f"  Joint {j+1}: {q:.6f} rad ({q*180/3.14159:.2f} deg)")

            # Print end-effector position (compact format for continuous display)
            if len(state_msg.O_T_EE) == 16:
                x, y, z = state_msg.O_T_EE[12], state_msg.O_T_EE[13], state_msg.O_T_EE[14]
                print(f"End-effector position: X={x:.4f}m, Y={y:.4f}m, Z={z:.4f}m")

            # Print timing info
            if state_msg.HasField('time'):
                print(f"Timestamp: {state_msg.time.toSec:.6f} sec")

            print("-" * 50)

            # Small delay between prints
            time.sleep(0.1)

    except zmq.Again:
        print("Timeout: No state messages received within 5 seconds.")
        print("Make sure the control node is running and the state publisher is active.")
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error receiving state: {e}")

    # Cleanup
    sub_socket.close()
    context.term()

def print_latest_state(control_port="5556"):
    """
    Print just the latest robot state (single sample).
    """
    context = zmq.Context()
    sub_socket = context.socket(zmq.SUB)
    state_port = str(int(control_port) + 1)
    sub_socket.connect(f"tcp://localhost:{state_port}")
    sub_socket.setsockopt(zmq.SUBSCRIBE, b"")
    sub_socket.setsockopt(zmq.RCVTIMEO, 5000)

    print(f"Getting latest robot state from port {state_port}...")

    try:
        response_data = sub_socket.recv()
        state_msg = franka_controller_pb2.FrankaRobotStateMessage()
        state_msg.ParseFromString(response_data)

        print(f"Joint angles ({len(state_msg.q)} joints):")
        for i, q in enumerate(state_msg.q):
            print(f"  Joint {i+1}: {q:.6f} rad ({q*180/3.14159:.2f} deg)")

        print(f"\nJoint velocities:")
        for i, dq in enumerate(state_msg.dq):
            print(f"  Joint {i+1}: {dq:.6f} rad/s")

        print(f"\nJoint torques:")
        for i, tau in enumerate(state_msg.tau_J):
            print(f"  Joint {i+1}: {tau:.6f} Nm")

        # Print end-effector pose
        if len(state_msg.O_T_EE) == 16:
            print(f"\nEnd-effector pose (O_T_EE) - 4x4 matrix:")
            for row in range(4):
                row_values = []
                for col in range(4):
                    idx = row * 4 + col
                    row_values.append(f"{state_msg.O_T_EE[idx]:8.6f}")
                print(f"  [{' '.join(row_values)}]")

            # Extract position (last column, first 3 elements)
            x, y, z = state_msg.O_T_EE[12], state_msg.O_T_EE[13], state_msg.O_T_EE[14]
            print(f"\nEnd-effector position:")
            print(f"  X: {x:.6f} m")
            print(f"  Y: {y:.6f} m")
            print(f"  Z: {z:.6f} m")

        if state_msg.HasField('time'):
            print(f"\nTimestamp: {state_msg.time.toSec:.6f} sec")

    except zmq.Again:
        print("Timeout: No state messages received.")
        print("Make sure the control node is running.")
    except Exception as e:
        print(f"Error: {e}")

    sub_socket.close()
    context.term()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        control_port = sys.argv[1]
    else:
        control_port = "5556"  # Default port

    if len(sys.argv) > 2:
        if sys.argv[2] == "once":
            print_latest_state(control_port)
        else:
            try:
                num_samples = int(sys.argv[2])
                subscribe_robot_state(control_port, num_samples)
            except ValueError:
                print("Invalid number of samples. Using default.")
                subscribe_robot_state(control_port)
    else:
        subscribe_robot_state(control_port)
