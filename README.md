# Bamboo Franka Controller

A lightweight package for controlling the Franka Emika FR3 and Panda with joint impedance control and controlling Robotiq grippers. 

A single real-time controller machine runs the control node and maintains the real-time link with the FR3/Panda.
Other machines can connect to this node using the Bamboo client via ZMQ to issue commands or receive robot state.

## Control Node Installation

Install the control node on the real-time control machine that is directly connected to the Franka robot.

### Prerequisites

1. Ensure that the [`libfranka` system requirements](https://github.com/frankarobotics/libfranka/tree/release-0.15.2?tab=readme-ov-file#1-system-requirements) are satisfied
2. Ensure that the [`libfranka` dependencies](https://github.com/frankarobotics/libfranka/tree/release-0.15.2?tab=readme-ov-file#1-system-requirements) are installed
3. **If using libfranka >= 0.14.0:** Install Pinocchio following the [libfranka dependency instructions](https://github.com/frankarobotics/libfranka/tree/release-0.15.2?tab=readme-ov-file#2-installing-dependencies) before running the installation script
4. Make sure you have set the inertial parameters for the Robotiq gripper in Franka Desk. You can follow the [instructions in DROID](https://droid-dataset.github.io/droid/software-setup/host-installation.html#updating-inertia-parameters-for-robotiq-gripper) for doing this.

### Build Controller
```bash
# Follow the instructions in the script
bash InstallBambooController
```

**Note:** This script builds `libfranka` locally and **will not override any system installations**. The installation script may request sudo privileges to add user groups and install system packages. You will be prompted before any sudo commands are executed.

You will also be prompted to enter the version of libfranka to install. This can be determined by:
- Checking the FCI version in the Franka Desk (under Settings > Dashboard > Control) and then consulting the [FCI Compatability Table](https://frankarobotics.github.io/docs/compatibility.html) for a compatible `libfranka` version
- Checking what libfranka versions you already have in other projects, you could run:
  ```bash
  locate libfranka.so
  ``` 

The `InstallBambooController` script will automatically handle:

- Adding your user to required groups (`realtime` for real-time kernel operations, `dialout` and `tty` for serial communication with Robotiq gripper)
- Installing system packages (`libzmq3-dev` for ZMQ networking, `libmsgpack-dev` for message serialization, `libpoco-dev` for Franka dependencies)
- Cloning and building `libfranka`

**Important:** If groups are added during installation, **you must log out and log back in** before running the controller.

### Manual Installation

If you prefer to install manually, refer to the steps in the [`InstallBambooController`](InstallBambooController) script.

## Bamboo Client Installation

You should install the Bamboo client on any machine that will talk to the control node. This installation only includes the client dependencies (numpy, pyzmq, msgpack) and not the hardware control dependencies.

**Install from PyPI:**

```bash
pip install bamboo-franka-client
```

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

**If you need Robotiq gripper server dependencies** (pyserial, pymodbus) on a non-control node machine:

```bash
pip install -e .[server]
```

## Usage

### Server-Side Robot Control

**Security Warning:** By default, the controller listens on all network interfaces (`*` or `0.0.0.0`), accepting commands from any IP address that can reach the machine. For security, consider restricting access by setting the listen address using the `--listen_ip` flag in `RunBambooController` (or the equivalent configuration option): for example, use `127.0.0.1` to accept commands only from the local machine, or a specific interface address such as `192.168.1.10` to accept commands only from that network. Avoid using `*`/`0.0.0.0` on untrusted or publicly accessible networks unless you have additional protections in place (VPN, firewall, etc.).

**Easy Start (Recommended):** Use the provided script to start both control node and gripper server in tmux:

```bash
bash RunBambooController
```

The script supports configuration flags:
```bash
bash RunBambooController start --robot_ip 172.16.0.2 --control_port 5555 --listen_ip "*" --gripper_device /dev/ttyUSB0 --gripper_port 5559 --conda_env bamboo
```

Available options:
- `--robot_ip`: Robot IP address (default: 172.16.0.2)
- `--control_port`: Control node ZMQ port (default: 5555)
- `--listen_ip`: ZMQ server listen address (default: * for all interfaces)
- `--gripper_device`: Gripper device (default: /dev/ttyUSB0)
- `--gripper_port`: Gripper server ZMQ port (default: 5559)
- `--conda_env`: Conda environment name (default: bamboo)

Other commands:
- `bash RunBambooController status` - Check server status
- `bash RunBambooController stop` - Stop all servers
- `bash RunBambooController attach` - Attach to tmux session

**Manual Start:** If you need to run servers manually, first run the C++ control node:

```bash
cd controller/build
./bamboo_control_node -r <robot-ip> -p <zmq-port> [-l <listen-address>] [-m]
```

Available flags:
- `-r`: Robot IP address (required)
- `-p`: Port number (required)
- `-l`: Listen address (default: * for all interfaces)
- `-m`: Use min-jerk interpolation (default: linear)
- `-h`: Show help

Example:
```bash
./bamboo_control_node -r 172.16.0.2 -p 5555 -l "*"
```

Then in a new terminal, launch the Robotiq gripper server:
```bash
conda activate bamboo
cd controller
python gripper_server.py --gripper-port <gripper-device> --zmq-port <zmq-port>
```

Example:
```bash
python gripper_server.py --gripper-port /dev/ttyUSB0 --zmq-port 5559
```

### Client-Side Interface with robot and gripper
You can verify the install by running some of the example scripts in a new terminal.
To actuate the robot and print out its joint angles (*WARNING: THIS SCRIPT MOVES THE ROBOT WITHOUT DOING COLLISION CHECKING SO MAKE SURE THE NEARBY WORKSPACE IS CLEAR*):
```bash
conda activate bamboo
python -m bamboo.examples.joint_trajectory
```
To open and close the gripper and print the width of the fingers:
```bash
conda activate bamboo
python -m bamboo.examples.gripper
```

## Development Setup

If you plan to contribute to Bamboo, you'll need to set up the development tools.

### Install Development Dependencies

Install the development dependencies including pre-commit, ruff, and mypy:

```bash
pip install -e .[dev]
```

### Set Up Pre-Commit Hooks

Install the pre-commit hooks to automatically run linting and formatting checks before each commit:

```bash
pre-commit install
```

Now, whenever you commit code, pre-commit will automatically:
- Format Python code with ruff
- Check Python code style with ruff

### Run Pre-Commit Manually

To run all pre-commit hooks on all files without making a commit:

```bash
pre-commit run --all-files
```

To run pre-commit on specific files:

```bash
pre-commit run --files path/to/file.py
```

## Contributing

For Python code, we enforce style with `ruff` and type checking with `mypy`. For C++ code, we enforce style with `clang-format`.

Pre-commit hooks will automatically run linting and formatting checks when you make a commit. You can also run them manually with `pre-commit run --all-files`.

To contribute:
1. Fork the repository
2. Create a feature branch based on `main`
3. Install development dependencies: `pip install -e .[dev]`
4. Set up pre-commit hooks: `pre-commit install`
5. Make your changes and commit them
6. Open a pull request from your feature branch

## Acknowledgements

This work draws heavily from [deoxys\_control](https://github.com/UT-Austin-RPL/deoxys_control) and [drake-franka-driver](https://github.com/RobotLocomotion/drake-franka-driver).
Thanks to the developers for their open-source code!
