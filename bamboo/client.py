from __future__ import annotations

import logging
import time
from typing import TypedDict

import msgpack
import numpy as np
import zmq

_log = logging.getLogger(__name__)


class BambooConnectionError(Exception):
    """Raised when connection to bamboo control node fails."""

    pass


class BambooTimeoutError(Exception):
    """Raised when operation times out."""

    pass


class BambooGripperError(Exception):
    """Raised when gripper operation fails."""

    pass


class JointStates(TypedDict, total=False):
    """Type definition for joint states dictionary."""

    ee_pose: list[list[float]]
    qpos: list[float]
    gripper_state: float


class BambooFrankaClient:
    """Client for communicating with the bamboo control node."""

    def __init__(
        self,
        control_port: int = 5555,
        server_ip: str = "localhost",
        gripper_port: int = 5559,
        enable_gripper: bool = True,
    ):
        """Initialize Bamboo Franka Client.

        Args:
            control_port: Port of the bamboo control node (default: 5555)
            server_ip: IP address of the bamboo control node server
            gripper_port: ZMQ port of the gripper server (default: 5559)
            enable_gripper: Whether to enable gripper commands
        """
        self.control_port = control_port
        self.server_ip = server_ip

        # Set up ZMQ context and control socket
        self.zmq_context = zmq.Context()

        # Set up control socket (REQ socket for request-response)
        self.control_socket = self.zmq_context.socket(zmq.REQ)
        self.control_socket.connect(f"tcp://{self.server_ip}:{self.control_port}")
        self.control_socket.setsockopt(zmq.RCVTIMEO, 5000)  # 5 second timeout
        _log.debug(f"Control client connected to {self.server_ip}:{self.control_port}")

        self.gripper_port = gripper_port
        self.enable_gripper = enable_gripper
        self.gripper_socket = None

        if enable_gripper:
            # Set up ZMQ socket for gripper commands (REQ socket for request-response)
            self.gripper_socket = self.zmq_context.socket(zmq.REQ)
            self.gripper_socket.connect(f"tcp://{self.server_ip}:{gripper_port}")
            self.gripper_socket.setsockopt(zmq.RCVTIMEO, 5000)  # 5 second timeout
            _log.debug(f"Gripper client connected to {self.server_ip}:{gripper_port}")

        # Test connection by trying to receive a state message
        self._test_connection()

    def _test_connection(self) -> None:
        """Test connection to the bamboo control node.

        Raises:
            BambooConnectionError: If connection test fails
        """
        try:
            # Try to receive a state message to verify connection
            _log.debug(f"Establishing connection to bamboo control node at {self.server_ip}:{self.control_port}...")
            self._get_latest_state()
            _log.info(f"Successfully connected to bamboo control node at {self.server_ip}:{self.control_port}")
        except Exception as e:
            raise BambooConnectionError(
                f"Could not connect to bamboo control node at {self.server_ip}:{self.control_port}. "
                f"Make sure the bamboo control node is running. Error: {e}"
            ) from e

    def _get_latest_state(self) -> dict:
        """Get the latest robot state from the bamboo control node.

        Returns:
            dict: The latest robot state containing q, dq, tau_J, O_T_EE, time_sec

        Raises:
            BambooConnectionError: If server returns error
            BambooTimeoutError: If state read times out
        """
        try:
            # Create request
            request = {"command": "get_robot_state"}

            # Send request
            self.control_socket.send(msgpack.packb(request))

            # Receive response
            response_data = self.control_socket.recv()
            response = msgpack.unpackb(response_data, raw=False)

            if not response.get("success", False):
                raise BambooConnectionError(f"Failed to get robot state: {response.get('error', 'Unknown error')}")

            return response["data"]

        except zmq.Again:
            raise BambooTimeoutError("Timeout waiting for robot state response") from None

    def get_joint_states(self) -> JointStates:
        """Get current robot joint states.

        Returns:
            Dict with 'ee_pose', 'qpos', 'gripper_state'

        Raises:
            BambooConnectionError: If connection to controller fails
            BambooTimeoutError: If state read times out
            BambooGripperError: If gripper state read fails
        """
        # Get latest state from bamboo control node
        state_msg = self._get_latest_state()

        # Extract joint positions
        qpos = list(state_msg["q"])  # Convert to list for JSON serialization

        # Extract end-effector pose (4x4 transformation matrix)
        # Convert flat array to 4x4 matrix (column-major)
        ee_pose_flat = list(state_msg["O_T_EE"])
        ee_pose_matrix = np.array(ee_pose_flat).reshape(4, 4)
        # Transpose from column-major to row-major
        ee_pose = ee_pose_matrix.T.tolist()

        # Get gripper state if available, otherwise use default
        if self.enable_gripper and self.gripper_socket is not None:
            gripper_result = self._send_gripper_command({"action": "get_state"})
            if not gripper_result.get("success"):
                raise BambooGripperError(f"Failed to get gripper state: {gripper_result.get('error', 'Unknown error')}")
            gripper_state = gripper_result["state"]["width"]
        else:
            gripper_state = 0.0  # No gripper enabled

        return {"ee_pose": ee_pose, "qpos": qpos, "gripper_state": gripper_state}

    def close(self) -> None:
        """Clean up ZMQ resources."""
        if hasattr(self, "control_socket") and self.control_socket is not None:
            self.control_socket.close()
        if hasattr(self, "gripper_socket") and self.gripper_socket is not None:
            self.gripper_socket.close()
        if hasattr(self, "zmq_context") and self.zmq_context is not None:
            self.zmq_context.term()

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
            BambooGripperError: If gripper not enabled
            BambooTimeoutError: If gripper command times out
        """
        if not self.enable_gripper or self.gripper_socket is None:
            raise BambooGripperError("Gripper not enabled or not connected")

        try:
            # Send command
            self.gripper_socket.send(msgpack.packb(command))

            # Receive response
            response_data = self.gripper_socket.recv()
            response = msgpack.unpackb(response_data, raw=False)

            return response  # type: ignore[no-any-return]

        except zmq.Again:
            raise BambooTimeoutError("Timeout waiting for gripper server response") from None

    def execute_joint_impedance_path(
        self,
        joint_confs: np.ndarray,
        joint_vels: np.ndarray | None = None,
        durations: list | None = None,
        default_duration: float = 0.5,
    ) -> dict[str, bool | str]:
        """Execute joint impedance trajectory and wait for completion.

        Args:
            joint_confs: (n, 7) array of joint configurations
            joint_vels: (n, 7) array of of joint velocities for each waypoint
            durations: Optional list of durations (in seconds) for each waypoint.
                      If None, all waypoints use default_duration.
                      If shorter than joint_confs, remaining waypoints use default_duration.
            default_duration: Default duration in seconds for waypoints (default: 1.0s)

        Returns:
            Dict with 'success' (bool) and 'error' (str) if failed
        """
        try:
            # Validate joint_confs shape
            if joint_confs.ndim != 2:
                raise ValueError(f"joint_confs must be 2D array, got {joint_confs.ndim}D")
            if joint_confs.shape[1] != 7:
                raise ValueError(f"joint_confs must have 7 columns, got {joint_confs.shape[1]}")

            _log.debug(f"Executing {joint_confs.shape[0]} joint waypoints")

            # Validate joint_vels parameter
            if joint_vels is None:
                joint_vels = np.array([[0, 0, 0, 0, 0, 0, 0]] * joint_confs.shape[0])
            else:
                if joint_vels.ndim != 2:
                    raise ValueError(f"joint_vels must be 2D array, got {joint_vels.ndim}D")
                if joint_vels.shape[1] != 7:
                    raise ValueError(f"joint_vels must have 7 columns, got {joint_vels.shape[1]}")
                if joint_vels.shape[0] != joint_confs.shape[0]:
                    raise ValueError(
                        f"joint_vels rows ({joint_vels.shape[0]}) must match joint_confs rows ({joint_confs.shape[0]})"
                    )

            # Build trajectory request with all waypoints
            waypoints = []
            total_duration = 0.0

            # Create each waypoint as a TimedWaypoint
            for i, joint_conf in enumerate(joint_confs):
                joint_vel = joint_vels[i]

                # Get waypoint duration
                waypoint_duration = durations[i] if (durations is not None and i < len(durations)) else default_duration
                if waypoint_duration <= 0:
                    waypoint_duration = default_duration
                total_duration += waypoint_duration

                # Create timed waypoint
                waypoint = {
                    "q_goal": joint_conf.tolist(),
                    "velocity": joint_vel.tolist(),
                    "duration": waypoint_duration,
                    "kp": [600.0] * 7,  # Default stiffness
                    "kd": [50.0] * 7,  # Default damping
                }

                waypoints.append(waypoint)

            # Create trajectory request
            trajectory_request = {"waypoints": waypoints, "default_duration": default_duration, "default_velocity": []}

            # Create full request
            request = {"command": "execute_trajectory", "data": trajectory_request}

            # Calculate appropriate timeout (total duration + buffer for settling + safety margin)
            # Add 2 seconds for robot settling time plus 50% safety margin
            trajectory_timeout_ms = int((total_duration + 2.0) * 1.5 * 1000)
            # Minimum timeout of 100ms
            trajectory_timeout_ms = max(trajectory_timeout_ms, 100)

            # Temporarily set socket timeout for this trajectory
            old_timeout = self.control_socket.getsockopt(zmq.RCVTIMEO)
            self.control_socket.setsockopt(zmq.RCVTIMEO, trajectory_timeout_ms)
            _log.debug(
                f"Set trajectory timeout to {trajectory_timeout_ms/1000.0:.1f}s for {total_duration:.1f}s trajectory"
            )

            try:
                # Send trajectory to control node and wait for completion
                _log.debug("Sending trajectory to control node...")
                self.control_socket.send(msgpack.packb(request))

                # Receive response (this will block until trajectory completes)
                response_data = self.control_socket.recv()
                response = msgpack.unpackb(response_data, raw=False)
            finally:
                # Restore original timeout
                self.control_socket.setsockopt(zmq.RCVTIMEO, old_timeout)

            if response.get("success", False):
                _log.debug("Trajectory completed successfully")
                return {"success": True}
            else:
                error_msg = response.get("error", "Trajectory execution failed")
                _log.error(f"Trajectory failed: {error_msg}")
                return {"success": False, "error": error_msg}

        except zmq.Again:
            _log.error("Timeout waiting for trajectory completion")
            return {"success": False, "error": "Timeout waiting for trajectory completion"}
        except ValueError as e:
            # Validation errors from parameter checks
            error_msg = f"Invalid trajectory parameters: {e}"
            _log.error(error_msg)
            return {"success": False, "error": error_msg}

    def open_gripper(self, speed: float = 0.05, force: float = 0.1, blocking: bool = True) -> dict:
        """Open the gripper.

        Args:
            speed: Gripper opening speed (0.0 to 1.0)
            force: Gripper force (0.0 to 1.0)
            blocking: Whether to block until gripper finishes

        Returns:
            Dict with response from gripper server

        Raises:
            BambooGripperError: If gripper not enabled or command fails
            BambooTimeoutError: If gripper command times out
        """
        command = {"action": "open", "speed": speed, "force": force, "blocking": blocking}
        return self._send_gripper_command(command)

    def close_gripper(self, speed: float = 0.05, force: float = 0.8, blocking: bool = True) -> dict:
        """Close the gripper.

        Args:
            speed: Gripper closing speed (0.0 to 1.0)
            force: Gripper force (0.0 to 1.0)
            blocking: Whether to block until gripper finishes

        Returns:
            Dict with response from gripper server

        Raises:
            BambooGripperError: If gripper not enabled or command fails
            BambooTimeoutError: If gripper command times out
        """
        command = {"action": "close", "speed": speed, "force": force, "blocking": blocking}
        return self._send_gripper_command(command)

    def get_gripper_state(self) -> dict:
        """Get current gripper state.

        Returns:
            Dict with 'success' and 'state' from gripper server

        Raises:
            BambooGripperError: If gripper not enabled or command fails
            BambooTimeoutError: If gripper command times out
        """
        command = {"action": "get_state"}
        return self._send_gripper_command(command)


def main() -> None:
    """Simple test of the BambooFrankaClient."""
    import argparse

    parser = argparse.ArgumentParser(description="Test Bamboo Franka Client")
    parser.add_argument("--port", default=5555, type=int, help="Port of bamboo control node")
    parser.add_argument("--ip", default="localhost", type=str, help="IP address of bamboo control node server")
    parser.add_argument("--samples", default=5, type=int, help="Number of state samples to fetch")
    parser.add_argument("--gripper-port", default=5559, type=int, help="ZMQ port of gripper server")
    parser.add_argument("--no-gripper", action="store_true", help="Disable gripper commands")
    args = parser.parse_args()

    # Set up logging
    logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

    print(f"Testing BambooFrankaClient at {args.ip}:{args.port}")

    try:
        with BambooFrankaClient(
            control_port=args.port,
            server_ip=args.ip,
            gripper_port=args.gripper_port,
            enable_gripper=not args.no_gripper,
        ) as client:
            print(f"\nFetching {args.samples} state samples:")
            print("-" * 60)

            for i in range(args.samples):
                result = client.get_joint_states()
                print(f"Sample {i+1}:")
                print(f"  Joint positions: {[f'{q:.4f}' for q in result['qpos']]}")
                print(
                    f"  EE position: [{result['ee_pose'][0][3]:.4f}, "
                    f"{result['ee_pose'][1][3]:.4f}, {result['ee_pose'][2][3]:.4f}]"
                )
                print(f"  Gripper state: {result['gripper_state']}")

                if i < args.samples - 1:
                    time.sleep(0.2)  # Small delay between samples

        print("\nTest completed successfully!")

    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Error during test: {e}")


if __name__ == "__main__":
    main()
