#!/usr/bin/env python3

"""
Bamboo Client - A simple client for the bamboo control node.
Provides basic robot state querying functionality.
"""

import logging
import sys
import time
import zmq
import numpy as np
from pathlib import Path

# Add the examples directory to path to import protobuf messages
sys.path.insert(0, str(Path(__file__).parent / "examples"))

try:
    import franka_controller_pb2
except ImportError:
    raise ImportError(
        "Could not import protobuf messages. Make sure the bamboo project is built. "
        "Run 'make' in the build directory first."
    )


class BambooFrankaClient:
    """Client for communicating with the bamboo control node."""

    def __init__(self, control_port: int = 5556, server_ip: str = "localhost"):
        """Initialize Bamboo Franka Client.

        Args:
            control_port: TCP port of the bamboo control node (state published on port+1)
            server_ip: IP address of the bamboo control node server
        """
        self.control_port = control_port
        self.state_port = control_port + 1
        self.server_ip = server_ip

        # Set up ZMQ context and state subscriber
        self.context = zmq.Context()
        self.state_sub = self.context.socket(zmq.SUB)
        self.state_sub.connect(f"tcp://{self.server_ip}:{self.state_port}")
        self.state_sub.setsockopt(zmq.SUBSCRIBE, b"")  # Subscribe to all messages
        self.state_sub.setsockopt(zmq.RCVTIMEO, 1000)  # 1 second timeout

        # Test connection by trying to receive a state message
        self._test_connection()

    def _test_connection(self):
        """Test connection to the bamboo control node."""
        try:
            # Try to receive a state message to verify connection
            self._get_latest_state()
            logging.info(f"Successfully connected to bamboo control node at {self.server_ip}:{self.control_port}")
        except Exception as e:
            logging.warning(f"Could not connect to bamboo control node at {self.server_ip}:{self.control_port}: {e}")
            logging.warning("Make sure the bamboo control node is running and publishing state.")

    def _get_latest_state(self):
        """Get the latest robot state from the bamboo control node.

        Returns:
            FrankaRobotStateMessage: The latest robot state

        Raises:
            RuntimeError: If no state message is received
        """
        try:
            # Receive state message
            response_data = self.state_sub.recv()

            # Parse protobuf message
            state_msg = franka_controller_pb2.FrankaRobotStateMessage()
            state_msg.ParseFromString(response_data)

            return state_msg

        except zmq.Again:
            raise RuntimeError("Timeout: No state messages received from bamboo control node")
        except Exception as e:
            raise RuntimeError(f"Error receiving state from bamboo control node: {e}")

    def get_joint_states(self) -> dict:
        """Get current robot joint states.

        Returns:
            Dict with 'success', 'ee_pose', 'qpos', 'gripper_state', and 'error' if failed
            Format matches DeoxysFrankaServer.get_joint_states() for compatibility
        """
        try:
            # Get latest state from bamboo control node
            state_msg = self._get_latest_state()

            # Extract joint positions
            qpos = list(state_msg.q)  # Convert to list for JSON serialization

            # Extract end-effector pose (4x4 transformation matrix)
            if len(state_msg.O_T_EE) == 16:
                # Convert flat array to 4x4 matrix and transpose to match deoxys format
                ee_pose_flat = list(state_msg.O_T_EE)
                ee_pose_matrix = np.array(ee_pose_flat).reshape(4, 4)
                # Transpose to match deoxys format (column-major to row-major)
                ee_pose = ee_pose_matrix.T.tolist()
            else:
                # Fallback: identity matrix if pose data is missing
                ee_pose = np.eye(4).tolist()

            # Hardcoded gripper state as requested
            gripper_state = 0.0  # Hardcoded gripper width

            return {
                'success': True,  # Hardcoded to True as requested
                'ee_pose': ee_pose,
                'qpos': qpos,
                'gripper_state': gripper_state
            }

        except Exception as e:
            logging.error(f"Error in get_joint_states: {e}")
            return {
                "success": False,
                "error": str(e)
            }

    def close(self):
        """Clean up ZMQ resources."""
        if hasattr(self, 'state_sub'):
            self.state_sub.close()
        if hasattr(self, 'context'):
            self.context.term()

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()


def main():
    """Simple test of the BambooFrankaClient."""
    import argparse

    parser = argparse.ArgumentParser(description="Test Bamboo Franka Client")
    parser.add_argument("--port", default=5556, type=int,
                       help="Control port of bamboo control node")
    parser.add_argument("--ip", default="localhost", type=str,
                       help="IP address of bamboo control node server")
    parser.add_argument("--samples", default=5, type=int,
                       help="Number of state samples to fetch")
    args = parser.parse_args()

    # Set up logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )

    print(f"Testing BambooFrankaClient at {args.ip}:{args.port}")
    print(f"State subscriber will connect to {args.ip}:{args.port + 1}")

    try:
        with BambooFrankaClient(control_port=args.port, server_ip=args.ip) as client:
            print(f"\nFetching {args.samples} state samples:")
            print("-" * 60)

            for i in range(args.samples):
                result = client.get_joint_states()

                if result['success']:
                    print(f"Sample {i+1}:")
                    print(f"  Joint positions: {[f'{q:.4f}' for q in result['qpos']]}")
                    print(f"  EE position: [{result['ee_pose'][0][3]:.4f}, "
                          f"{result['ee_pose'][1][3]:.4f}, {result['ee_pose'][2][3]:.4f}]")
                    print(f"  Gripper state: {result['gripper_state']}")
                else:
                    print(f"Sample {i+1}: ERROR - {result.get('error', 'Unknown error')}")

                if i < args.samples - 1:
                    time.sleep(0.2)  # Small delay between samples

        print("\nTest completed successfully!")

    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Error during test: {e}")


if __name__ == "__main__":
    main()
