#!/usr/bin/env python3

"""
Example script demonstrating how to:
1. Create a BambooFrankaClient
2. Get current joint angles
3. Add 0.1 to each joint angle
4. Send the modified angles as a 1-waypoint trajectory to the robot
"""

import argparse

import numpy as np

from bamboo import BambooFrankaClient


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Example joint trajectory execution with Bamboo.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--server-ip",
        default="localhost",
        help="Server IP address",
    )
    parser.add_argument(
        "--control-port",
        type=int,
        default=5555,
        help="Control port",
    )
    parser.add_argument(
        "--gripper-port",
        type=int,
        default=5559,
        help="Gripper port",
    )
    args = parser.parse_args()

    print("Creating BambooFrankaClient...")

    # Create the client with specified parameters
    with BambooFrankaClient(
        control_port=args.control_port,
        server_ip=args.server_ip,
        gripper_port=args.gripper_port,
    ) as client:
        # Get current joint angles
        print("\nGetting current joint angles...")
        current_joints = client.get_joint_positions()
        print(f"Current joint angles: {[f'{q:.4f}' for q in current_joints]}")

        waypoints = [current_joints]
        durations = [0.7]
        for _ in range(30):
            waypoint = [q + 0.01 for q in waypoints[-1]]
            waypoints.append(waypoint)
            durations.append(durations[-1] - 0.02)
        for _ in range(30):
            waypoint = [q - 0.01 for q in waypoints[-1]]
            waypoints.append(waypoint)
            durations.append(durations[-1] + 0.03)

        print("\nSending trajectory to robot...")
        result = client.execute_joint_impedance_path(np.array(waypoints), durations=durations)

        # Get final joint positions to calculate error
        print("\nGetting final joint angles...")
        final_joints = client.get_joint_positions()
        print(f"Final joint angles: {[f'{q:.4f}' for q in final_joints]}")

        # Calculate final position error (compare to last waypoint)
        position_error = np.linalg.norm(np.array(final_joints) - np.array(waypoints[-1]))
        print(f"Final position error: {position_error:.6f}")

        if result["success"]:
            print("✓ Trajectory executed successfully!")
        else:
            print(f"✗ Trajectory failed: {result.get('error', 'Unknown error')}")


if __name__ == "__main__":
    main()
