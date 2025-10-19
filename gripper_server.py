#!/usr/bin/env python3

"""
Gripper Server - Controls Robotiq gripper hardware via ZMQ messages.
This runs on the robot computer and receives commands from remote BambooFrankaClients.
"""

import json
import logging
import time
import zmq
import argparse
from robotiq_gripper_client import RobotiqGripperClient


class GripperServer:
    """ZMQ server that controls Robotiq gripper hardware."""

    def __init__(self, gripper_port: str = "/dev/ttyUSB0", zmq_port: int = 5558):
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

    def handle_command(self, command: dict) -> dict:
        """Handle a gripper command.

        Args:
            command: Dict with 'action' and parameters

        Returns:
            Dict with response
        """
        try:
            action = command.get('action')
            print(f"{action=}")

            if action == 'open':
                speed = command.get('speed', 0.05)
                force = command.get('force', 0.1)
                max_gripper_width = 0.085
                self.gripper.apply_gripper_command(width=max_gripper_width, speed=speed, force=force)
                return {"success": True}

            elif action == 'close':
                speed = command.get('speed', 0.05)
                force = command.get('force', 0.1)
                self.gripper.apply_gripper_command(width=0.0, speed=speed, force=force)
                return {"success": True}

            elif action == 'get_state':
                state = self.gripper.get_gripper_state()
                return {"success": True, "state": state}

            elif action == 'shutdown':
                self.running = False
                return {"success": True, "message": "Server shutting down"}

            else:
                return {"success": False, "error": f"Unknown action: {action}"}

        except Exception as e:
            logging.error(f"Error handling command {command}: {e}")
            return {"success": False, "error": str(e)}

    def run(self):
        """Main server loop."""
        logging.info("Gripper server ready to receive commands...")

        try:
            while self.running:
                # Wait for request (with timeout)
                try:
                    message = self.socket.recv_string(zmq.NOBLOCK)
                    print("MESSAGE RECEIVED")

                    # Parse command
                    try:
                        command = json.loads(message)
                    except json.JSONDecodeError:
                        response = {"success": False, "error": "Invalid JSON"}
                    else:
                        print(f"{command=}")
                        response = self.handle_command(command)

                    # Send response
                    self.socket.send_string(json.dumps(response))

                except zmq.Again:
                    # No message received, continue
                    time.sleep(0.01)
                    continue

        except KeyboardInterrupt:
            logging.info("Received Ctrl+C, shutting down...")
        finally:
            self.cleanup()

    def cleanup(self):
        """Clean up resources."""
        logging.info("Cleaning up gripper server...")
        if hasattr(self, 'socket'):
            self.socket.close()
        if hasattr(self, 'context'):
            self.context.term()


def main():
    parser = argparse.ArgumentParser(description="Gripper Server for Robotiq control")
    parser.add_argument("--gripper-port", default="/dev/ttyUSB0", type=str,
                       help="Serial port for Robotiq gripper")
    parser.add_argument("--zmq-port", default=5558, type=int,
                       help="ZMQ port to listen on")
    parser.add_argument("--verbose", "-v", action="store_true",
                       help="Enable verbose logging")
    args = parser.parse_args()

    # Set up logging
    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        level=log_level,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )

    print("=" * 60)
    print(f"Starting Gripper Server")
    print(f"Gripper port: {args.gripper_port}")
    print(f"ZMQ port: {args.zmq_port}")
    print("=" * 60)

    try:
        server = GripperServer(
            gripper_port=args.gripper_port,
            zmq_port=args.zmq_port
        )
        server.run()
    except Exception as e:
        logging.error(f"Failed to start gripper server: {e}")
        return 1

    return 0


if __name__ == "__main__":
    exit(main())
