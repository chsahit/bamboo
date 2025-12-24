# Bamboo Franka Controller

A lightweight package for controlling the Franka Emika FR3 and Panda with joint impedance control. 

A single real-time controller machine runs the control node and maintains the real-time link with the FR3/Panda.
Other machines can connect to this node using the Bamboo client via ZMQ to issue commands or receive robot state.

## Control Node Installation

Install the control node on the real-time control machine that is directly connected to the Franka robot.

### Prerequisites
Ensure that the system satisfies the [`libfranka` system requirements](https://github.com/frankarobotics/libfranka/tree/release-0.15.2?tab=readme-ov-file#1-system-requirements) and that the the [`libfranka` dependencies](https://github.com/frankarobotics/libfranka/tree/release-0.15.2?tab=readme-ov-file#1-system-requirements) are installed. 

Next, install `zmq` and related dependencies with
```bash
sudo apt install libzmq3-dev libmsgpack-dev libpoco-dev
```

These steps may require administrator privileges.

Make sure that the user is in the realtime group with `sudo usermod -a -G realtime $USER`

You do not need to install `libfranka` yourself — the included `InstallBambooController` script will clone, build, and set up libfranka.

### Build C++ Dependencies
```bash
bash InstallBambooController
```
You will be prompted to enter the version of libfranka to install. This can be determined by checking the FCI version in the Franka Desk (under Settings > Dashboard > Control) and then consulting the [FCI Compatability Table](https://frankarobotics.github.io/docs/compatibility.html) for a compatible `libfranka` version. 


### Install Python Package

The `InstallBambooController` script handles this automatically with server dependencies included. If you need to install manually:

```bash
conda create -n bamboo python=3.10
conda activate bamboo
pip install -e .[server]
```

### Compile Controller
```bash
mkdir bamboo/build && cd bamboo/build
cmake ..
make
```

## Bamboo Client Installation

You should install the Bamboo client on any machine that will talk to the control node. This installation only includes the client dependencies (numpy, pyzmq, msgpack) and not the hardware control dependencies.

**Install from GitHub repository:**

```bash
pip install git+https://github.com/chsahit/bamboo.git
```

**Install from source:**

```bash
git clone https://github.com/chsahit/bamboo.git
cd bamboo
pip install -e .
```

**If you need gripper server dependencies** (pyserial, pymodbus) on a non-control node machine:

```bash
pip install -e .[server]
```

## Usage

### Server-Side Robot Control

**Security Warning:** By default, the controller listens on all network interfaces (`*`), accepting commands from any IP address that can reach the machine. For security, consider restricting access by setting the 'listen address' 

**Easy Start (Recommended):** Use the provided script to start both control node and gripper server in tmux:

```bash
bash bamboo.sh
```

The script supports configuration flags:
```bash
bash bamboo.sh start --robot_addr 172.16.0.2 --control_port 5555 --listen_addr "*" --gripper_device /dev/ttyUSB0 --gripper_port 5559
```

Available options:
- `--robot_addr`: Robot IP address (default: 172.16.0.2)
- `--control_port`: Control node port (default: 5555)
- `--listen_addr`: Listen address (default: * for all interfaces)
- `--gripper_device`: Gripper device (default: /dev/ttyUSB0)
- `--gripper_port`: Gripper server port (default: 5559)

Other commands:
- `bash bamboo.sh status` - Check server status
- `bash bamboo.sh stop` - Stop all servers
- `bash bamboo.sh attach` - Attach to tmux session

**Manual Start:** If you need to run servers manually, first run the C++ control node:

```bash
conda activate bamboo
cd bamboo/build
./bamboo_control_node -r <robot-ip> -p <port> [-l <listen-address>]
```

Example:
```bash
./bamboo_control_node -r 172.16.0.2 -p 5555 -l "*"
```

Then in a new terminal, launch the gripper server:
```bash
conda activate bamboo
cd bamboo
python3 gripper_server.py --gripper-port <gripper-device> --zmq-port <zmq-port>
```

Example:
```bash
python3 gripper_server.py --gripper-port /dev/ttyUSB0 --zmq-port 5559
```

You may have to add the user to the `dialout` and `tty` groups to read from the robotiq grippers if this hasn't been done already:
```bash
sudo usermod -a -G dialout $USER
sudo usermod -a -G tty $USER
```

### Client-Side Interface with robot and gripper
You can verify the install by running some of the example scripts in a new terminal. 
To actuate the robot and print out its joint angles (*WARNING: THIS SCRIPT MOVES THE ROBOT WITHOUT DOING COLLISION CHECKING SO MAKE SURE THE NEARBY WORKSPACE IS CLEAR*):
```bash
conda activate bamboo
cd bamboo/examples
python example_joint_trajectory.py
```
To open and glose the gripper and print the width of the fingers:
```bash
conda activate bamboo
cd bamboo/examples
python simple_gripper_example.py
```

## Contributing 
For the python code we enforce style with `ruff` and typechecking with `mypy`. For the C++ code, we enforce style with `clang-tidy`. You can run all linting and checking steps with `pre-commit run --all-files`, they also will run automatically when you make a commit. 

To contribute, please create a fork of the repository, make a feature branch based on main, and commit your changes there. Then open a pull request from that branch.

## Acknowledgements

This work draws heavily from [deoxys\_control](https://github.com/UT-Austin-RPL/deoxys_control) and [drake-franka-driver](https://github.com/RobotLocomotion/drake-franka-driver).
Thanks to the developers for their open-source code!
