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
First, run the C++ control node 

```bash
conda activate bamboo
cd bamboo/build
./bamboo_control_node <robot-ip> <zmq-port>
```

Example:
```bash
./bamboo_control_node 172.16.0.2 5555
```

Then in a new terminal, launch the gripper server
```bash
conda activate bamboo
cd bamboo
python3 gripper_server.py --gripper-port <gripper-device> --zmq-port <zmq-port>
```

Example:
```bash
python3 gripper_server.py --gripper-port /dev/ttyUSB0 --zmq-port 5559
```
You may have to add the user to the `dialout` and `tty` groups to read from the robotiq grippers if this hasn't been done already. It can be done with 
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
python joint_trajectory.py
```
To open and close the gripper and print the width of the fingers:
```bash
conda activate bamboo
cd bamboo/examples
python gripper.py
```

## Contributing 
For the python code we enforce style with `ruff` and typechecking with `mypy`. For the C++ code, we enforce style with `clang-tidy`. You can run all linting and checking steps with `pre-commit run --all-files`, they also will run automatically when you make a commit. 

To contribute, please create a fork of the repository, make a feature branch based on main, and commit your changes there. Then open a pull request from that branch.

## Acknowledgements

This work draws heavily from [deoxys\_control](https://github.com/UT-Austin-RPL/deoxys_control) and [drake-franka-driver](https://github.com/RobotLocomotion/drake-franka-driver).
Thanks to the developers for their open-source code!
