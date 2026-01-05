#!/usr/bin/env python3

"""
Example script demonstrating how to:
1. Create a BambooFrankaClient
2. Get current joint angles
3. Add 0.1 to each joint angle
4. Send the modified angles as a 1-waypoint trajectory to the robot
"""

import logging
import sys
from pathlib import Path

import numpy as np

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from bamboo_client import BambooFrankaClient


def main() -> int:
    # Set up logging
    logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

    print("Creating BambooFrankaClient...")

    try:
        # Create the client (uses default ports)
        with BambooFrankaClient() as client:
            print("✓ Client connected successfully")

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

    except Exception as e:
        print(f"Error: {e}")
        return 1

    return 0


if __name__ == "__main__":
    exit(main())
