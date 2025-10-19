#!/usr/bin/env python3
"""
Send termination message to the bamboo control node.
Use this to gracefully shut down the control node from Python.
"""

import sys
import zmq
sys.path.insert(0, '../proto')
import franka_controller_pb2

def send_termination_message(zmq_pub):
    """Send termination message to control node."""
    control_msg = franka_controller_pb2.FrankaControlMessage()
    control_msg.termination = True
    zmq_pub.send(control_msg.SerializeToString())
    print("Sent termination message")

def main():
    if len(sys.argv) != 2:
        print("Usage: python terminate_control_node.py <zmq-port>")
        sys.exit(1)

    zmq_port = sys.argv[1]

    print(f"Sending termination message to control node on port {zmq_port}...")

    # Initialize ZMQ publisher
    zmq_context = zmq.Context()
    zmq_pub = zmq_context.socket(zmq.PUB)
    zmq_pub.bind(f"tcp://*:{zmq_port}")

    # Give ZMQ time to connect
    import time
    time.sleep(0.5)

    # Send termination
    send_termination_message(zmq_pub)

    time.sleep(0.5)
    print("Control node should now terminate gracefully.")

if __name__ == "__main__":
    main()