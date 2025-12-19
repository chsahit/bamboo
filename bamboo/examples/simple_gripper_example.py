#!/usr/bin/env python3

"""
Simple gripper control example with BambooFrankaClient.
"""

import sys
import time
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from bamboo_client import BambooFrankaClient

def main() -> None:
    # Connect to bamboo control node and gripper server
    with BambooFrankaClient(
        control_port=5555,      # Bamboo control node port
        server_ip="localhost",  # Robot IP (change as needed)
        gripper_port=5559,      # Gripper server port
        enable_gripper=True
    ) as client:

        print("Connected to bamboo client!")

        # Close gripper
        print("Closing gripper...")
        result = client.close_gripper(speed=0.05, force=0.1)
        print(f"Close result: {result}")

        # Open gripper
        print("Opening gripper...")
        result = client.open_gripper(speed=0.05, force=0.1)
        print(f"Open result: {result}")

        # Get gripper state
        print("Getting gripper state...")
        state = client.get_gripper_state()
        if state["success"]:
            width = state["state"]["width"]
            print(f"Gripper width: {width:.4f}m")
        else:
            print(f"Error: {state['error']}")

if __name__ == "__main__":
    main()
