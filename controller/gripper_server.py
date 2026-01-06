#!/usr/bin/env python3

"""
Gripper Server - Controls Robotiq gripper hardware via ZMQ messages.
This runs on the robot computer and receives commands from remote BambooFrankaClients.
"""

import argparse
import logging
import time

import msgpack
import zmq

from controller.third_party.robotiq_gripper_client import RobotiqGripperClient


class GripperServer:
    """ZMQ server that controls Robotiq gripper hardware."""

    def __init__(self, gripper_port: str = "/dev/ttyUSB0", zmq_port: int = 5559):
        """Initialize Gripper Server.

        Args:
            gripper_port: Serial port for Robotiq gripper
            zmq_port: ZMQ port to listen for commands
        """
        self.zmq_port = zmq_port

        # Initialize Robotiq gripper
        logging.info(f"Connecting to Robotiq gripper at {gripper_port}...")
        try:
            self.gripper = RobotiqGripperClient(comport=gripper_port)
            logging.info("Gripper connected successfully!")
        except Exception as e:
            logging.error(f"Failed to connect to gripper: {e}")
            raise

        # Set up ZMQ server
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)  # Reply socket for request-response
        self.socket.bind(f"tcp://*:{zmq_port}")
        logging.info(f"Gripper server listening on port {zmq_port}")

        self.running = True

    def _spin_until_done(self, timeout: float = 5.0) -> None:
        done_time = time.time() + timeout
        while time.time() < done_time:
            time.sleep(0.02)  # poll at 50 Hz
            state = self.gripper.get_gripper_state()
            if not state["is_moving"]:
                return True
        return False

    def handle_command(self, command: dict) -> dict:
        """Handle a gripper command.

        Args:
            command: Dict with 'action' and parameters

        Returns:
            Dict with response
        """
        try:
            action = command.get("action")
            print(f"{action=}")

            if action == "open":
                speed = command.get("speed", 0.05)
                force = command.get("force", 0.1)
                max_gripper_width = 0.085
                success = self.gripper.apply_gripper_command(width=max_gripper_width, speed=speed, force=force)
                if command.get("blocking", True):
                    success = self._spin_until_done()
                return {"success": success}

            elif action == "close":
                speed = command.get("speed", 0.05)
                force = command.get("force", 0.1)
                success = self.gripper.apply_gripper_command(width=0.0, speed=speed, force=force)
                if command.get("blocking", True):
                    success = self._spin_until_done()
                return {"success": success}

            elif action == "get_state":
                state = self.gripper.get_gripper_state()
                return {"success": True, "state": state}

            elif action == "shutdown":
                self.running = False
                return {"success": True, "message": "Server shutting down"}

            else:
                return {"success": False, "error": f"Unknown action: {action}"}

        except Exception as e:
            logging.error(f"Error handling command {command}: {e}")
            return {"success": False, "error": str(e)}

    def run(self) -> None:
        """Main server loop."""
        logging.info("Gripper server ready to receive commands...")

        try:
            while self.running:
                # Wait for request (with timeout)
                try:
                    message = self.socket.recv(zmq.NOBLOCK)
                    print("MESSAGE RECEIVED")

                    # Parse command
                    try:
                        command = msgpack.unpackb(message, raw=False)
                    except (msgpack.exceptions.ExtraData, msgpack.exceptions.UnpackException):
                        response = {"success": False, "error": "Invalid msgpack"}
                    else:
                        print(f"{command=}")
                        response = self.handle_command(command)

                    # Send response
                    self.socket.send(msgpack.packb(response))

                except zmq.Again:
                    # No message received, continue
                    time.sleep(0.01)
                    continue

        except KeyboardInterrupt:
            logging.info("Received Ctrl+C, shutting down...")
        finally:
            self.cleanup()

    def cleanup(self) -> None:
        """Clean up resources."""
        logging.info("Cleaning up gripper server...")
        if hasattr(self, "socket"):
            self.socket.close()
        if hasattr(self, "context"):
            self.context.term()


def main() -> int:
    parser = argparse.ArgumentParser(description="Gripper Server for Robotiq control")
    parser.add_argument("--gripper-port", default="/dev/ttyUSB0", type=str, help="Serial port for Robotiq gripper")
    parser.add_argument("--zmq-port", default=5559, type=int, help="ZMQ port to listen on")
    parser.add_argument("--verbose", "-v", action="store_true", help="Enable verbose logging")
    args = parser.parse_args()

    # Set up logging
    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=log_level, format="%(asctime)s - %(levelname)s - %(message)s")

    print("=" * 60)
    print("Starting Gripper Server")
    print(f"Gripper port: {args.gripper_port}")
    print(f"ZMQ port: {args.zmq_port}")
    print("=" * 60)

    try:
        server = GripperServer(gripper_port=args.gripper_port, zmq_port=args.zmq_port)
        server.run()
    except Exception as e:
        logging.error(f"Failed to start gripper server: {e}")
        return 1

    return 0


if __name__ == "__main__":
    exit(main())
