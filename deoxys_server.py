import logging
import sys
import time
from argparse import ArgumentParser
from functools import wraps

import numpy as np
import zerorpc

from deoxys.experimental.motion_utils import reset_joints_to, follow_joint_traj
from deoxys.utils import YamlConfig, transform_utils
from hw_utils.interface import get_franka_interface
from robotiq_gripper_client import RobotiqGripperClient


class ColoredFormatter(logging.Formatter):
    """Custom formatter with colors for console output."""

    COLORS = {
        'DEBUG': '\033[36m',  # Cyan
        'INFO': '\033[32m',  # Green
        'WARNING': '\033[33m',  # Yellow
        'ERROR': '\033[31m',  # Red
        'CRITICAL': '\033[35m',  # Magenta
    }
    RESET = '\033[0m'

    def format(self, record):
        if hasattr(sys.stderr, 'isatty') and sys.stderr.isatty():
            levelname = record.levelname
            if levelname in self.COLORS:
                record.levelname = f"{self.COLORS[levelname]}{levelname}{self.RESET}"
        return super().format(record)


def log_rpc_call(func):
    """Decorator to log RPC method calls with parameters and results."""

    @wraps(func)
    def wrapper(*args, **kwargs):
        # args[0] is self, skip it for logging
        params = args[1:] if len(args) > 1 else []
        _log.info(f"RPC call: {func.__name__}, params: {params}, kwargs: {kwargs}")

        try:
            result = func(*args, **kwargs)
            if isinstance(result, dict) and "success" in result:
                if result["success"]:
                    _log.info(f"RPC call: {func.__name__} - SUCCESS")
                else:
                    _log.warning(f"RPC call: {func.__name__} - FAILED: {result.get('error', 'Unknown error')}")
            return result
        except Exception as e:
            _log.error(f"RPC call: {func.__name__} - EXCEPTION: {e}", exc_info=True)
            raise

    return wrapper


def setup_logging(log_file: str = "deoxys_server.log"):
    """Configure logging with file and console handlers."""
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)

    # File handler with detailed format
    file_handler = logging.FileHandler(log_file)
    file_handler.setLevel(logging.DEBUG)
    file_formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(funcName)s:%(lineno)d - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    file_handler.setFormatter(file_formatter)

    # Console handler with colors
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.INFO)
    console_formatter = ColoredFormatter(
        '%(asctime)s - %(levelname)s - %(message)s',
        datefmt='%H:%M:%S'
    )
    console_handler.setFormatter(console_formatter)

    logger.addHandler(file_handler)
    logger.addHandler(console_handler)


_log = logging.getLogger(__name__)


class DeoxysFrankaServer:
    def __init__(self, robot_index: int = 2,
                 config_root: str = "/home/labubu/workspace/deoxys_examples_kalm/robot_configs",
                 reset_on_start: bool = True, gripper_port: str = "/dev/ttyUSB0"):
        """Initialize Deoxys Franka Server.

        Args:
            robot_index: Robot index for franka interface
            config_root: Path to robot configuration files
            reset_on_start: Whether to reset robot to home position on startup
            gripper_port: Serial port for Robotiq gripper
        """
        _log.info(f"Connecting to Franka robot (index={robot_index})...")
        self.robot_interface = get_franka_interface(robot_index=robot_index, wait_for_state=True, auto_close=True)
        _log.info("Robot connected successfully!")

        # Initialize Robotiq gripper
        _log.info(f"Connecting to Robotiq gripper at {gripper_port}...")
        try:
            self.gripper = RobotiqGripperClient(comport=gripper_port)
            _log.info("Gripper connected successfully!")
        except Exception as e:
            _log.warning(f"Failed to connect to gripper: {e}. Gripper commands will not be available.")
            self.gripper = None

        self.config_root = config_root

        # Load controller configurations
        self.controller_type_imp = 'JOINT_IMPEDANCE'
        self.controller_cfg_imp = YamlConfig(config_root + '/joint-impedance-controller.yml').as_easydict()

        self.controller_type_free = 'OSC_POSE'
        self.controller_cfg_free = YamlConfig(config_root + '/osc-free-motion-controller.yml').as_easydict()


        # Home position
        self.home_q = np.array([0.0, -1 / 5 * np.pi, 0.0, -4 / 5 * np.pi, 0.0, 3 / 5 * np.pi, 0.0])

        # Wait for robot state
        while self.robot_interface.state_buffer_size == 0:
            _log.warning("Robot state not received, waiting...")
            time.sleep(0.5)

        if reset_on_start:
            _log.info("Resetting robot to home position...")
            reset_joints_to(self.robot_interface, self.home_q, gripper_open=False)
            if self.gripper:
                self.gripper.apply_gripper_command(width=0.085, speed=0.05, force=0.1)
            _log.info("Reset complete!")

        _log.info("Ready!")

    @log_rpc_call
    def move_to_joint_positions(self, qpos: list, gripper_open: bool = True) -> dict:
        """Reset robot to specified joint positions.

        Args:
            qpos: List of 7 joint positions
            gripper_open: Whether to open gripper

        Returns:
            Dict with 'success' (bool) and 'error' (str) if failed
        """
        try:
            reset_joints_to(self.robot_interface, np.array(qpos), gripper_open=False)
            if self.gripper:
                width = 0.085 if gripper_open else 0.0
                self.gripper.apply_gripper_command(width=width, speed=0.05, force=0.1)
            return {"success": True}
        except Exception as e:
            _log.error(f"Error in reset_joint_to: {e}", exc_info=True)
            return {"success": False, "error": str(e)}

    @log_rpc_call
    def get_joint_states(self) -> dict:
        """Get current robot joint states.

        Returns:
            Dict with 'success', 'ee_pose', 'qpos', 'gripper_state', and 'error' if failed
        """
        try:
            last_state = self.robot_interface._state_buffer[-1]

            # Get gripper state from Robotiq gripper
            if self.gripper:
                gripper_state_info = self.gripper.get_gripper_state()
                gripper_width = gripper_state_info.get('width', 0.0)
            else:
                gripper_width = 0.0

            return {
                'success': True,
                'ee_pose': np.array(last_state.O_T_EE).reshape(4, 4).T.tolist(),
                'qpos': np.array(last_state.q).tolist(),
                'gripper_state': float(gripper_width)
            }
        except Exception as e:
            _log.error(f"Error in get_joint_states: {e}", exc_info=True)
            return {"success": False, "error": str(e)}

    @log_rpc_call
    def open_gripper(self, speed: float = 0.05, force: float = 0.1) -> dict:
        try:
            if self.gripper is None:
                raise RuntimeError("Gripper not connected")
            max_gripper_width = 0.085
            self.gripper.apply_gripper_command(width=max_gripper_width, speed=speed, force=force)
            return {"success": True}
        except Exception as e:
            _log.error(f"Error in open_gripper: {e}", exc_info=True)
            return {"success": False, "error": str(e)}

    @log_rpc_call
    def close_gripper(self, speed: float = 0.05, force: float = 0.1) -> dict:
        try:
            if self.gripper is None:
                raise RuntimeError("Gripper not connected")
            self.gripper.apply_gripper_command(width=0.0, speed=speed, force=force)
            return {"success": True}
        except Exception as e:
            _log.error(f"Error in close_gripper: {e}", exc_info=True)
            return {"success": False, "error": str(e)}

    @log_rpc_call
    def get_gripper_state(self) -> dict:
        try:
            if self.gripper is None:
                raise RuntimeError("Gripper not connected")
            state = self.gripper.get_gripper_state()
            return {"success": True, "state": state}
        except Exception as e:
            _log.error(f"Error in get_gripper_state: {e}", exc_info=True)
            return {"success": False, "error": str(e)}

    @log_rpc_call
    def go_to_home(self, gripper_open: bool = True) -> dict:
        """Move robot to home position.

        Args:
            gripper_open: Whether to open gripper

        Returns:
            Dict with 'success' (bool) and 'error' (str) if failed
        """
        try:
            reset_joints_to(self.robot_interface, self.home_q, gripper_open=False)
            if self.gripper:
                width = 0.085 if gripper_open else 0.0
                self.gripper.apply_gripper_command(width=width, speed=0.05, force=0.1)
            return {"success": True}
        except Exception as e:
            _log.error(f"Error in go_to_home: {e}", exc_info=True)
            return {"success": False, "error": str(e)}

    @log_rpc_call
    def execute_joint_impedance_path(self, joint_confs: list, gripper_isopen) -> dict:
        """Execute joint impedance trajectory with Robotiq gripper control.

        Args:
            joint_confs: List of joint configurations (7 values each)
            gripper_isopen: Bool or list of bools for gripper state

        Returns:
            Dict with 'success' (bool) and 'error' (str) if failed
        """
        try:
            joint_confs = [np.array(conf) for conf in joint_confs]
            _log.info(f'Executing {len(joint_confs)} joint waypoints')

            # Execute trajectory without gripper commands
            follow_joint_traj(self.robot_interface, joint_confs, controller_cfg=self.controller_cfg_imp,
                              gripper_close=False, num_addition_steps=0)

            # Control Robotiq gripper separately
            # if self.gripper:
            #     if type(gripper_isopen) is bool:
            #         width = 0.085 if gripper_isopen else 0.0
            #         self.gripper.apply_gripper_command(width=width, speed=0.05, force=0.1)
            #     else:
            #         # Use final gripper state if list provided
            #         width = 0.085 if gripper_isopen[-1] else 0.0
            #         self.gripper.apply_gripper_command(width=width, speed=0.05, force=0.1)

            return {"success": True}
        except Exception as e:
            _log.error(f"Error in execute_joint_impedance_path: {e}", exc_info=True)
            return {"success": False, "error": str(e)}

    @log_rpc_call
    def free_motion_step(self, gripper_open: bool = True) -> dict:
        """Execute one step of free motion control.

        Maintains current pose with zero stiffness, allowing manual guidance.
        Call this repeatedly to keep the robot in free motion mode.

        Args:
            gripper_open: Whether to open gripper

        Returns:
            Dict with 'success' (bool), 'current_pose', and 'error' (str) if failed
        """
        try:
            # Get current end-effector pose
            current_ee_pose = self.robot_interface.last_eef_pose.copy()
            current_pos = current_ee_pose[:3, 3:]
            current_rot = current_ee_pose[:3, :3]
            current_quat = transform_utils.mat2quat(current_rot)
            current_axis_angle = transform_utils.quat2axisangle(current_quat)

            # Use current pose as target (position + axis-angle + gripper)
            target_pose = current_pos.flatten().tolist() + current_axis_angle.flatten().tolist()
            gripper_action = [-1.0] if gripper_open else [+1.0]
            action = target_pose + gripper_action

            # Execute control step
            self.robot_interface.control(
                controller_type=self.controller_type_free,
                action=action,
                controller_cfg=self.controller_cfg_free,
            )

            return {
                "success": True,
                "current_pose": {
                    "position": current_pos.flatten().tolist(),
                    "quaternion": current_quat.flatten().tolist()
                }
            }
        except Exception as e:
            _log.error(f"Error in free_motion_step: {e}", exc_info=True)
            return {"success": False, "error": str(e)}


def main():
    parser = ArgumentParser(description="ZeroRPC Server for Deoxys Franka Robot")
    parser.add_argument("--robot-index", default=2, type=int, help="Robot index")
    parser.add_argument("--port", default=4242, type=int, help="ZeroRPC server port")
    parser.add_argument("--bind", default="tcp://0.0.0.0", help="ZeroRPC bind address")
    parser.add_argument("--log-file", default="deoxys_server.log", help="Log file path")
    parser.add_argument("--config-root", default="/home/labubu/workspace/deoxys_examples_kalm/robot_configs",
                        help="Path to robot config files")
    parser.add_argument("--reset", action="store_true", help="Don't reset robot on startup")
    parser.add_argument("--gripper-port", default="/dev/ttyUSB0", help="Gripper serial port")
    args = parser.parse_args()

    # Setup logging
    print("=" * 60)
    print(f"Starting Deoxys Franka Robot ZeroRPC Server")
    print(f"Log file: {args.log_file}")
    print("=" * 60)

    setup_logging(args.log_file)

    try:
        robot_server = DeoxysFrankaServer(
            robot_index=args.robot_index,
            config_root=args.config_root,
            reset_on_start=args.reset,
            gripper_port=args.gripper_port
        )
        server = zerorpc.Server(robot_server)
        bind_address = f"{args.bind}:{args.port}"
        server.bind(bind_address)
        print(f"ZeroRPC server listening on {bind_address}")
        print("Press Ctrl+C to stop the server")
        server.run()
    except KeyboardInterrupt:
        print("\nShutting down server...")
    except Exception as e:
        _log.critical(f"Error starting server: {e}", exc_info=True)


if __name__ == "__main__":
    main()
