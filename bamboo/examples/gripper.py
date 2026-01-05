#!/usr/bin/env python3


import argparse

from bamboo import BambooFrankaClient


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Simple gripper control example with Bamboo.",
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

    # Connect to bamboo control node and gripper server
    with BambooFrankaClient(
        control_port=args.control_port,
        server_ip=args.server_ip,
        gripper_port=args.gripper_port,
    ) as client:
        print("Connected to bamboo client!")

        # Close gripper
        print("Closing gripper...")
        result = client.close_gripper(speed=0.05, force=0.1, blocking=True)
        print(f"Close result: {result}")

        # Open gripper
        print("Opening gripper...")
        result = client.open_gripper(speed=0.05, force=0.1, blocking=True)
        print(f"Open result: {result}")

        # Get gripper state
        print("Getting gripper state...")
        state = client.get_gripper_state()
        width = state["state"]["width"]
        print(f"Gripper width: {width:.4f}m")


if __name__ == "__main__":
    main()
