#!/usr/bin/env python3

"""
Bamboo gRPC Client - A client for the bamboo control node using gRPC.
Provides the same API as BambooFrankaClient but uses gRPC instead of ZMQ for arm control.
Gripper communication still uses ZMQ (unchanged).
"""

from __future__ import annotations

import json
import logging
import sys
import time
import zmq
import numpy as np
from pathlib import Path
import grpc
from typing import Optional, TypedDict

_log = logging.getLogger(__name__)

# Add the proto_gen directory to path to import protobuf messages
sys.path.insert(0, str(Path(__file__).parent / "proto_gen"))

try:
    import franka_controller_pb2
    import bamboo_service_pb2
    import bamboo_service_pb2_grpc
except ImportError:
    raise ImportError(
        "Could not import protobuf messages. Make sure the bamboo project is built. "
        "Run 'make' in the build directory first."
    )


class JointStates(TypedDict, total=False):
    """Type definition for joint states dictionary."""
    success: bool
    ee_pose: list[list[float]]
    qpos: list[float]
    gripper_state: float


class BambooFrankaClient:
    """Client for communicating with the bamboo control node via gRPC."""

    def __init__(self, control_port: int = 5555, server_ip: str = "localhost",
                 gripper_port: int = 5559, enable_gripper: bool = True):
        """Initialize Bamboo Franka Client with gRPC.

        Args:
            control_port: gRPC port of the bamboo control node (default: 50051)
            server_ip: IP address of the bamboo control node server
            gripper_port: ZMQ port of the gripper server (unchanged)
            enable_gripper: Whether to enable gripper commands
        """
        self.control_port = control_port
        self.server_ip = server_ip

        self.grpc_address = f"{self.server_ip}:{self.control_port}"
        options = [
            ('grpc.keepalive_time_ms', 10000),
            ('grpc.keepalive_timeout_ms', 5000),
            ('grpc.keepalive_permit_without_calls', True)
        ]
        self.channel = grpc.insecure_channel(self.grpc_address, options=options)
        self.stub = bamboo_service_pb2_grpc.BambooControlServiceStub(self.channel)

        self.gripper_port = gripper_port
        self.enable_gripper = enable_gripper
        self.gripper_socket = None
        self.zmq_context = None

        if enable_gripper:
            # Set up ZMQ socket for gripper commands (REQ socket for request-response)
            self.zmq_context = zmq.Context()
            self.gripper_socket = self.zmq_context.socket(zmq.REQ)
            self.gripper_socket.connect(f"tcp://{self.server_ip}:{gripper_port}")
            self.gripper_socket.setsockopt(zmq.RCVTIMEO, 5000)  # 5 second timeout
            _log.debug(f"Gripper client connected to {self.server_ip}:{gripper_port}")

        # Test connection by trying to receive a state message
        self._test_connection()

    def _test_connection(self) -> None:
        """Test connection to the bamboo control node and warm up the gRPC connection."""
        try:
            # Try to receive a state message to verify connection and warm up gRPC
            # This first call will be slow as it establishes the connection
            _log.debug(f"Establishing connection to bamboo control node at {self.grpc_address}...")
            self._get_latest_state()
            _log.info(f"Successfully connected to bamboo control node at {self.grpc_address}")
        except Exception as e:
            _log.warning(f"Could not connect to bamboo control node at {self.grpc_address}: {e}")
            _log.warning("Make sure the bamboo control node is running.")

    def _get_latest_state(self) -> bamboo_service_pb2.RobotStateRequest:
        """Get the latest robot state from the bamboo control node.

        Returns:
            FrankaRobotStateMessage: The latest robot state

        Raises:
            RuntimeError: If no state message is received
        """
        try:
            # Create request
            request = bamboo_service_pb2.RobotStateRequest()

            # Call gRPC service with timeout
            response = self.stub.GetRobotState(request, timeout=1.0)

            return response

        except grpc.RpcError as e:
            raise RuntimeError(f"gRPC error getting robot state: {e.code()} - {e.details()}")
        except Exception as e:
            # Check if channel is closed and attempt reconnection
            error_msg = str(e).lower()
            if 'closed channel' in error_msg:
                _log.warning("Channel closed, attempting to reconnect...")
                try:
                    # Close old channel
                    self.channel.close()
                except Exception as closing_err:
                    print(f"warning, exception hit: {closing_err}")

                # Recreate channel and stub with keep-alive options
                options = [
                    ('grpc.keepalive_time_ms', 10000),
                    ('grpc.keepalive_timeout_ms', 5000),
                    ('grpc.keepalive_permit_without_calls', True),
                ]
                self.channel = grpc.insecure_channel(self.grpc_address, options=options)
                self.stub = bamboo_service_pb2_grpc.BambooControlServiceStub(self.channel)

                # Retry the request once
                request = bamboo_service_pb2.RobotStateRequest()
                response = self.stub.GetRobotState(request, timeout=1.0)
                _log.info("Reconnected successfully")
                return response

            raise RuntimeError(f"Error receiving state from bamboo control node: {e}")


    def get_joint_states(self) -> JointStates:
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
            # Convert flat array to 4x4 matrix (column-major)
            ee_pose_flat = list(state_msg.O_T_EE)
            ee_pose_matrix = np.array(ee_pose_flat).reshape(4, 4)
            # Transpose from column-major to row-major
            ee_pose = ee_pose_matrix.T.tolist()
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
                'success': True,
                'ee_pose': ee_pose,
                'qpos': qpos,
                'gripper_state': gripper_state
            }

        except Exception as e:
            _log.error(f"Error in get_joint_states: {e}")
            return {
                "success": False,
            }

    def close(self) -> None:
        """Clean up gRPC and ZMQ resources."""
        if hasattr(self, 'channel'):
            self.channel.close()
        # if hasattr(self, 'gripper_socket') and self.gripper_socket is not None:
        #     # Set linger to 0 on the socket before closing
        #     self.gripper_socket.setsockopt(zmq.LINGER, 0)
        #     self.gripper_socket.close()
        # if hasattr(self, 'zmq_context') and self.zmq_context is not None:
        #     try:
        #         # Set linger to 0 to avoid hanging on context termination
        #         self.zmq_context.setsockopt(zmq.LINGER, 0)
        #         self.zmq_context.term()
        #     except Exception:
        #         # If context termination hangs, just destroy it
        #         self.zmq_context.destroy()

    def __enter__(self) -> BambooFrankaClient:
        """Context manager entry."""
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: object | None,
    ) -> None:
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

            return response  # type: ignore[no-any-return]

        except zmq.Again:
            raise RuntimeError("Timeout waiting for gripper server response")
        except Exception as e:
            raise RuntimeError(f"Gripper communication error: {e}")

    def execute_joint_impedance_path(self, joint_confs: list, joint_vels: Optional[list]=None, gripper_isopen: bool=True,
            durations: Optional[list]=None, default_duration:float=0.5) -> dict:
        """Execute joint impedance trajectory and wait for completion.

        Args:
            joint_confs: List of joint configurations (7 values each)
            joint_vels: List of joint velocities (7 values each) for each waypoint
            gripper_isopen: Bool or list of bools for gripper state (ignored for now)
            durations: Optional list of durations (in seconds) for each waypoint.
                      If None, all waypoints use default_duration.
                      If shorter than joint_confs, remaining waypoints use default_duration.
            default_duration: Default duration in seconds for waypoints (default: 0.5s)

        Returns:
            Dict with 'success' (bool) and 'error' (str) if failed
        """
        try:
            _log.debug(f'Executing {len(joint_confs)} joint waypoints')

            # Validate joint_vels parameter
            if joint_vels is None:
                joint_vels = [[0, 0, 0, 0, 0, 0, 0] ] * len(joint_confs)
            if joint_vels is not None and len(joint_vels) != len(joint_confs):
                raise ValueError(f"joint_vels length ({len(joint_vels)}) must match joint_confs length ({len(joint_confs)})")

            # Build trajectory request with all waypoints
            trajectory_request = bamboo_service_pb2.JointImpedanceTrajectoryRequest()
            trajectory_request.default_duration = default_duration

            # Create each waypoint as a TimedWaypoint
            for i, joint_conf in enumerate(joint_confs):
                if len(joint_conf) != 7:
                    raise ValueError(f"Joint configuration {i} must have 7 values, got {len(joint_conf)}")

                # Validate joint velocity if provided
                joint_vel = None
                if joint_vels is not None:
                    joint_vel = joint_vels[i]
                    if len(joint_vel) != 7:
                        raise ValueError(f"Joint velocity {i} must have 7 values, got {len(joint_vel)}")

                # Create joint impedance control message
                ji_msg = franka_controller_pb2.FrankaJointImpedanceControllerMessage()

                # Set goal joint positions
                goal = ji_msg.goal
                goal.is_delta = False  # Absolute positions
                goal.q1 = joint_conf[0]
                goal.q2 = joint_conf[1]
                goal.q3 = joint_conf[2]
                goal.q4 = joint_conf[3]
                goal.q5 = joint_conf[4]
                goal.q6 = joint_conf[5]
                goal.q7 = joint_conf[6]

                # Set default impedance parameters
                for _ in range(7):
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

                # Create timed waypoint
                timed_waypoint = bamboo_service_pb2.TimedWaypoint()
                timed_waypoint.waypoint.CopyFrom(control_msg)

                # Set duration for this waypoint
                if durations is not None and i < len(durations):
                    timed_waypoint.duration = durations[i]
                else:
                    timed_waypoint.duration = 0.0  # Use default_duration

                # Set velocity for this waypoint if provided
                if joint_vel is not None:
                    for vel in joint_vel:
                        timed_waypoint.velocity.append(vel)

                # Add to trajectory
                trajectory_request.waypoints.append(timed_waypoint)

            # Send trajectory to gRPC service and wait for completion
            _log.debug("Sending trajectory to control node...")
            response = self.stub.ExecuteJointImpedanceTrajectory(
                trajectory_request,
                timeout=60.0  # Long timeout for trajectory execution
            )

            if response.success:
                _log.debug("Trajectory completed successfully")
                return {"success": True}
            else:
                _log.error("Trajectory failed")
                return {"success": False, "error": "Trajectory execution failed"}

        except grpc.RpcError as e:
            _log.error(f"gRPC error in execute_joint_impedance_path: {e.code()} - {e.details()}")
            return {"success": False, "error": f"gRPC error: {e.details()}"}
        except Exception as e:
            _log.error(f"Error in execute_joint_impedance_path: {e}")
            return {"success": False, "error": str(e)}

    def open_gripper(self, speed: float = 0.05, force: float = 0.1, blocking: bool = True) -> dict:
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
                "force": force,
                "blocking": blocking
            }
            response = self._send_gripper_command(command)
            return response
        except Exception as e:
            _log.error(f"Error in open_gripper: {e}")
            return {"success": False, "error": str(e)}

    def close_gripper(self, speed: float = 0.05, force: float = 0.8, blocking: bool = True) -> dict:
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
                "force": force,
                "blocking": blocking
            }
            response = self._send_gripper_command(command)
            return response
        except Exception as e:
            _log.error(f"Error in close_gripper: {e}")
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
            _log.error(f"Error in get_gripper_state: {e}")
            return {"success": False, "error": str(e)}


def main() -> None:
    """Simple test of the BambooFrankaClient."""
    import argparse

    parser = argparse.ArgumentParser(description="Test Bamboo Franka Client (gRPC)")
    parser.add_argument("--port", default=5555, type=int,
                       help="gRPC port of bamboo control node")
    parser.add_argument("--ip", default="localhost", type=str,
                       help="IP address of bamboo control node server")
    parser.add_argument("--samples", default=5, type=int,
                       help="Number of state samples to fetch")
    parser.add_argument("--gripper-port", default=5559, type=int,
                       help="ZMQ port of gripper server")
    parser.add_argument("--no-gripper", action="store_true",
                       help="Disable gripper commands")
    args = parser.parse_args()

    # Set up logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )

    print(f"Testing BambooFrankaClient (gRPC) at {args.ip}:{args.port}")

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
