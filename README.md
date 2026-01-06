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

Make sure that the user is in the realtime group and can read over USB interfaces with `sudo usermod -a -G realtime,dialout,tty $USER`. 

You do not need to install `libfranka` yourself — the included `InstallBambooController` script will clone, build, and set up libfranka.

### Build Controller
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
mkdir controller/build && cd controller/build
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
conda activate bamboo
cd controller/build
./bamboo_control_node -r <robot-ip> -p <zmq-port> [-l <listen-address>]
```

Example:
```bash
./bamboo_control_node -r 172.16.0.2 -p 5555 -l "*"
```

Then in a new terminal, launch the gripper server:
```bash
conda activate bamboo
cd controller
python3 gripper_server.py --gripper-port <gripper-device> --zmq-port <zmq-port>
```

Example:
```bash
python3 gripper_server.py --gripper-port /dev/ttyUSB0 --zmq-port 5559
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

For Python code, we enforce style with `ruff` and type checking with `mypy`. For C++ code, we enforce style with `clang-tidy`.

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
