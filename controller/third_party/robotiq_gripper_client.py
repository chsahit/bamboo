# Copyright (c) Facebook, Inc. and its affiliates.

import logging
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.
import time

from controller.third_party.robotiq_2finger_grippers.robotiq_2f_gripper import (
    Robotiq2FingerGripper,
)

log = logging.getLogger(__name__)


class RobotiqGripperClient:
    """client that exposes controls of a Robotiq gripper.
    """

    def __init__(self, comport="/dev/ttyUSB0", hz=60):
        self.hz = hz

        # Connect to gripper
        self.gripper = Robotiq2FingerGripper(comport=comport)

        if not self.gripper.init_success:
            raise Exception(f"Unable to open comport to {comport}")

        if not self.gripper.getStatus():
            raise Exception(f"Failed to contact gripper on port {comport}... ABORTING")

        print("Activating gripper...")
        self.gripper.activate_emergency_release()
        self.gripper.sendCommand()
        time.sleep(1)
        self.gripper.deactivate_emergency_release()
        self.gripper.sendCommand()
        time.sleep(1)
        self.gripper.activate_gripper()
        self.gripper.sendCommand()
        time.sleep(3)

        if (
                self.gripper.is_ready()
                and self.gripper.sendCommand()
                and self.gripper.getStatus()
        ):
            print("Activated.")
        else:
            raise Exception(f"Unable to activate!")

        # Connect to server
        # self.channel = grpc.insecure_channel(f"{server_ip}:{server_port}")
        # self.connection = polymetis_pb2_grpc.GripperServerStub(self.channel)

        # Initialize connection to server
        # metadata = polymetis_pb2.GripperMetadata()
        # metadata.polymetis_version = polymetis.__version__
        # metadata.hz = self.hz
        # metadata.max_width = self.gripper.stroke

        # self.connection.InitRobotClient(metadata)
        # self.metadata = metadata

    def get_gripper_state(self):
        # state = polymetis_pb2.GripperState()
        state = {}

        if not self.gripper.getStatus():
            # NOTE: getStatus returns False and does not update state if modbus read fails
            state["error_code"] = 1
            log.warning(
                "Failed to read gripper state. Returning last observed state instead."
            )

        # state.timestamp.GetCurrentTime()
        state["width"] = self.gripper.get_pos()
        state["is_grasped"] = self.gripper.object_detected()
        state["is_moving"] = self.gripper.is_moving()

        return state

    def apply_gripper_command(self, width, speed, force):
        # if cmd.grasp:
        #     cmd.width = 0.0

        self.gripper.goto(pos=width, vel=speed, force=force)
        self.gripper.sendCommand()

        return True

    # def run(self):
    #     prev_timestamp = timestamp_pb2.Timestamp()
    #
    #     spinner = Spinner(self.hz)
    #     while True:
    #         # Retrieve state
    #         state = self.get_gripper_state()
    #
    #         # Query for command
    #         cmd = self.connection.ControlUpdate(state)
    #
    #         # Apply command if command is updated
    #         if cmd.timestamp != prev_timestamp:
    #             self.apply_gripper_command(cmd)
    #             prev_timestamp = cmd.timestamp
    #
    #         # Spin
    #         spinner.spin()


if __name__ == '__main__':
    client = RobotiqGripperClient()
    print("setup)")
    max_gripper_width = 0.085
    command = 0
    print("opening gripper")
    client.apply_gripper_command(width=max_gripper_width * (1 - command), speed=0.05, force=0.1)
    import time
    time.sleep(2.0)
    print("closing gripper")
    client.apply_gripper_command(width=max_gripper_width * (1 - 1), speed=0.05, force=0.1)

    time.sleep(2.0)

    print("opening gripper")
    client.apply_gripper_command(width=max_gripper_width * (1 - command), speed=0.05, force=0.1)



