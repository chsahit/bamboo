#!/usr/bin/env python3

"""
Example script demonstrating gripper control with BambooFrankaClient.
This script connects to a bamboo control node and gripper server to open/close the gripper.
"""

import sys
import time
import logging
from pathlib import Path

# Add parent directory to path for bamboo_client import
sys.path.insert(0, str(Path(__file__).parent.parent))

from bamboo_client import BambooFrankaClient


def main() -> int:
    # Set up logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )

    # Configuration
    robot_ip = "localhost"  # Change to robot's IP address
    control_port = 5556     # Bamboo control node port
    gripper_port = 5558     # Gripper server port

    print("=" * 60)
    print("Bamboo Gripper Control Example")
    print("=" * 60)
    print(f"Robot IP: {robot_ip}")
    print(f"Control port: {control_port}")
    print(f"Gripper port: {gripper_port}")
    print()

    try:
        # Create client with gripper enabled
        print("Connecting to bamboo control node and gripper server...")
        with BambooFrankaClient(
            control_port=control_port,
            server_ip=robot_ip,
            gripper_port=gripper_port,
            enable_gripper=True
        ) as client:
            print("✓ Connected successfully!")
            print()

            # Get initial gripper state
            print("Getting initial gripper state...")
            state_result = client.get_gripper_state()
            if state_result["success"]:
                state = state_result["state"]
                print(f"  Width: {state.get('width', 'unknown'):.4f}m")
                print(f"  Is grasped: {state.get('is_grasped', 'unknown')}")
                print(f"  Is moving: {state.get('is_moving', 'unknown')}")
            else:
                print(f"  Error getting state: {state_result.get('error', 'Unknown error')}")
            print()

            # Test sequence: close -> open -> close
            print("Starting gripper test sequence...")
            print()

            # Step 1: Close gripper
            print("1. Closing gripper...")
            result = client.close_gripper(speed=0.05, force=0.1)
            if result["success"]:
                print("   ✓ Close command sent successfully")
            else:
                print(f"   ✗ Close failed: {result.get('error', 'Unknown error')}")

            # Wait for movement to complete
            print("   Waiting for gripper movement...")
            time.sleep(3.0)

            # Check state after closing
            state_result = client.get_gripper_state()
            if state_result["success"]:
                width = state_result["state"].get('width', 0)
                print(f"   Gripper width after closing: {width:.4f}m")
            print()

            # Step 2: Open gripper
            print("2. Opening gripper...")
            result = client.open_gripper(speed=0.05, force=0.1)
            if result["success"]:
                print("   ✓ Open command sent successfully")
            else:
                print(f"   ✗ Open failed: {result.get('error', 'Unknown error')}")

            # Wait for movement to complete
            print("   Waiting for gripper movement...")
            time.sleep(3.0)

            # Check state after opening
            state_result = client.get_gripper_state()
            if state_result["success"]:
                width = state_result["state"].get('width', 0)
                print(f"   Gripper width after opening: {width:.4f}m")
            print()

            # Step 3: Close gripper again
            print("3. Closing gripper again...")
            result = client.close_gripper(speed=0.1, force=0.2)  # Different speed/force
            if result["success"]:
                print("   ✓ Close command sent successfully")
            else:
                print(f"   ✗ Close failed: {result.get('error', 'Unknown error')}")

            # Wait for movement to complete
            print("   Waiting for gripper movement...")
            time.sleep(3.0)

            # Final state check
            state_result = client.get_gripper_state()
            if state_result["success"]:
                state = state_result["state"]
                width = state.get('width', 0)
                is_grasped = state.get('is_grasped', False)
                print(f"   Final gripper width: {width:.4f}m")
                print(f"   Object grasped: {is_grasped}")
            print()

            print("✓ Gripper test sequence completed!")

    except KeyboardInterrupt:
        print("\n⚠ Test interrupted by user")
    except Exception as e:
        print(f"✗ Error during gripper test: {e}")
        return 1

    return 0


if __name__ == "__main__":
    exit(main())
