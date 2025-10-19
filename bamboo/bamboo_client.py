#!/usr/bin/env python3

"""
Bamboo Client - A simple client for the bamboo control node.
Provides basic robot state querying functionality.
"""

import json
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

    def __init__(self, control_port: int = 5556, server_ip: str = "localhost",
                 gripper_port: int = 5558, enable_gripper: bool = True):
        """Initialize Bamboo Franka Client.

        Args:
            control_port: TCP port of the bamboo control node (state published on port+1)
            server_ip: IP address of the bamboo control node server
            gripper_port: ZMQ port of the gripper server
            enable_gripper: Whether to enable gripper commands
        """
        self.control_port = control_port
        self.state_port = control_port + 1
        self.server_ip = server_ip

        # Set up ZMQ context and sockets
        self.context = zmq.Context()

        # State subscriber (for receiving robot state)
        self.state_sub = self.context.socket(zmq.SUB)
        self.state_sub.connect(f"tcp://{self.server_ip}:{self.state_port}")
        self.state_sub.setsockopt(zmq.SUBSCRIBE, b"")  # Subscribe to all messages
        self.state_sub.setsockopt(zmq.RCVTIMEO, 1000)  # 1 second timeout

        # Command publisher (for sending commands to control node)
        self.cmd_pub = self.context.socket(zmq.PUB)
        self.cmd_pub.connect(f"tcp://{self.server_ip}:{self.control_port}")

        # Give publisher time to connect
        time.sleep(0.1)

        # Set up gripper communication
        self.gripper_port = gripper_port
        self.enable_gripper = enable_gripper
        self.gripper_socket = None

        if enable_gripper:
            # Set up ZMQ socket for gripper commands (REQ socket for request-response)
            self.gripper_socket = self.context.socket(zmq.REQ)
            self.gripper_socket.connect(f"tcp://{self.server_ip}:{gripper_port}")
            self.gripper_socket.setsockopt(zmq.RCVTIMEO, 5000)  # 5 second timeout
            logging.info(f"Gripper client connected to {self.server_ip}:{gripper_port}")

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

            # Get gripper state if available, otherwise use default
            if self.enable_gripper and self.gripper_socket is not None:
                try:
                    gripper_result = self._send_gripper_command({"action": "get_state"})
                    if gripper_result.get("success") and "state" in gripper_result:
                        gripper_state = gripper_result["state"].get('width', 0.0)
                    else:
                        gripper_state = 0.0
                except Exception:
                    gripper_state = 0.0  # Fallback if gripper read fails
            else:
                gripper_state = 0.0  # No gripper enabled

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
        if hasattr(self, 'cmd_pub'):
            self.cmd_pub.close()
        if hasattr(self, 'gripper_socket') and self.gripper_socket is not None:
            self.gripper_socket.close()
        if hasattr(self, 'context'):
            self.context.term()

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()

    def get_joint_positions(self) -> list[float]:
        return self.get_joint_states()["qpos"]

    def _send_gripper_command(self, command: dict) -> dict:
        """Send a command to the gripper server.

        Args:
            command: Dict with command to send

        Returns:
            Dict with response from gripper server

        Raises:
            RuntimeError: If gripper communication fails
        """
        if not self.enable_gripper or self.gripper_socket is None:
            raise RuntimeError("Gripper not enabled or not connected")

        try:
            # Send command
            self.gripper_socket.send_string(json.dumps(command))

            # Receive response
            response_str = self.gripper_socket.recv_string()
            response = json.loads(response_str)

            return response

        except zmq.Again:
            raise RuntimeError("Timeout waiting for gripper server response")
        except Exception as e:
            raise RuntimeError(f"Gripper communication error: {e}")

    def _send_joint_impedance_command(self, joint_positions: list, gripper_open: bool = True):
        """Send a single joint impedance command to the control node.

        Args:
            joint_positions: List of 7 joint positions
            gripper_open: Whether gripper should be open (ignored for now)

        Raises:
            RuntimeError: If command sending fails
        """
        try:
            # Create joint impedance control message
            ji_msg = franka_controller_pb2.FrankaJointImpedanceControllerMessage()

            # Set goal joint positions
            goal = ji_msg.goal
            goal.is_delta = False  # Absolute positions
            goal.q1 = joint_positions[0]
            goal.q2 = joint_positions[1]
            goal.q3 = joint_positions[2]
            goal.q4 = joint_positions[3]
            goal.q5 = joint_positions[4]
            goal.q6 = joint_positions[5]
            goal.q7 = joint_positions[6]

            # Set default impedance parameters (matching deoxys defaults)
            for i in range(7):
                ji_msg.kp.append(600.0)  # Default stiffness
                ji_msg.kd.append(50.0)   # Default damping

            # Create main control message
            control_msg = franka_controller_pb2.FrankaControlMessage()
            control_msg.termination = False
            control_msg.controller_type = franka_controller_pb2.FrankaControlMessage.JOINT_IMPEDANCE
            control_msg.traj_interpolator_type = franka_controller_pb2.FrankaControlMessage.MIN_JERK_JOINT_POSITION
            control_msg.traj_interpolator_time_fraction = 0.1  # 10% of trajectory time for interpolation
            control_msg.timeout = 10.0

            # Pack the joint impedance message
            control_msg.control_msg.Pack(ji_msg)

            # Serialize and send
            msg_data = control_msg.SerializeToString()
            self.cmd_pub.send(msg_data)

        except Exception as e:
            raise RuntimeError(f"Failed to send joint impedance command: {e}")

    def execute_joint_impedance_path(self, joint_confs: list, gripper_isopen=True) -> dict:
        """Execute joint impedance trajectory.

        Args:
            joint_confs: List of joint configurations (7 values each)
            gripper_isopen: Bool or list of bools for gripper state (ignored for now)

        Returns:
            Dict with 'success' (bool) and 'error' (str) if failed
        """
        try:
            logging.info(f'Executing {len(joint_confs)} joint waypoints')

            # Send each joint configuration as a command
            for i, joint_conf in enumerate(joint_confs):
                if len(joint_conf) != 7:
                    raise ValueError(f"Joint configuration {i} must have 7 values, got {len(joint_conf)}")

                # Determine gripper state for this waypoint
                if isinstance(gripper_isopen, list):
                    gripper_open = gripper_isopen[i] if i < len(gripper_isopen) else gripper_isopen[-1]
                else:
                    gripper_open = gripper_isopen

                # Send command to control node
                self._send_joint_impedance_command(joint_conf, gripper_open)

            return {"success": True}

        except Exception as e:
            logging.error(f"Error in execute_joint_impedance_path: {e}")
            return {"success": False, "error": str(e)}

    def open_gripper(self, speed: float = 0.05, force: float = 0.1) -> dict:
        """Open the gripper.

        Args:
            speed: Gripper opening speed (0.0 to 1.0)
            force: Gripper force (0.0 to 1.0)

        Returns:
            Dict with 'success' (bool) and 'error' (str) if failed
        """
        try:
            command = {
                "action": "open",
                "speed": speed,
                "force": force
            }
            response = self._send_gripper_command(command)
            return response
        except Exception as e:
            logging.error(f"Error in open_gripper: {e}")
            return {"success": False, "error": str(e)}

    def close_gripper(self, speed: float = 0.05, force: float = 0.1) -> dict:
        """Close the gripper.

        Args:
            speed: Gripper closing speed (0.0 to 1.0)
            force: Gripper force (0.0 to 1.0)

        Returns:
            Dict with 'success' (bool) and 'error' (str) if failed
        """
        try:
            command = {
                "action": "close",
                "speed": speed,
                "force": force
            }
            response = self._send_gripper_command(command)
            return response
        except Exception as e:
            logging.error(f"Error in close_gripper: {e}")
            return {"success": False, "error": str(e)}

    def get_gripper_state(self) -> dict:
        """Get current gripper state.

        Returns:
            Dict with 'success', 'state', and 'error' (str) if failed
        """
        try:
            command = {"action": "get_state"}
            response = self._send_gripper_command(command)
            return response
        except Exception as e:
            logging.error(f"Error in get_gripper_state: {e}")
            return {"success": False, "error": str(e)}


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
    parser.add_argument("--gripper-port", default=5558, type=int,
                       help="ZMQ port of gripper server")
    parser.add_argument("--no-gripper", action="store_true",
                       help="Disable gripper commands")
    args = parser.parse_args()

    # Set up logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )

    print(f"Testing BambooFrankaClient at {args.ip}:{args.port}")
    print(f"State subscriber will connect to {args.ip}:{args.port + 1}")

    try:
        with BambooFrankaClient(control_port=args.port, server_ip=args.ip,
                               gripper_port=args.gripper_port,
                               enable_gripper=not args.no_gripper) as client:
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
