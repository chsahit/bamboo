# Bamboo Franka Controller

A lightweight package for controlling the Franka Emika FR3 with joint impedance control. 

A single real-time controller machine runs the control node and maintains the real-time link with the FR3. Other machines can connect to this node via gRPC to issue commands or receive robot state.

## Control Node Installation

### Prerequisites
Ensure that the system satisfies the [`libfranka` system requirements](https://github.com/frankarobotics/libfranka/tree/release-0.15.2?tab=readme-ov-file#1-system-requirements) and that the the [`libfranka` dependencies](https://github.com/frankarobotics/libfranka/tree/release-0.15.2?tab=readme-ov-file#1-system-requirements) are installed. These steps may require administrator privileges.

Make sure that the user is in the realtime group with `sudo usermod -a -G realtime $USER`

You do not need to install `libfranka` yourself — the included `InstallPackage` script will clone, build, and set up libfranka along with all additional dependencies at the user level.

### Build C++ Dependencies
```bash
bash InstallPackage
```
You will be prompted to enter the version of libfranka to install. This can be determined by checking the FCI version in the Franka Desk (under Settings > Dashboard > Control) and then consulting the [FCI Compatability Table](https://frankarobotics.github.io/docs/compatibility.html) for a compatible `libfranka` version. 

NB: Cloning and building `grpc` can take a long time due to the large number of submodules, this is expected behavior.

### Install Python Package
```bash
conda create -n bamboo python=3.10
conda activate bamboo
pip install -e .
```

### Compile Controller
```bash
mkdir bamboo/build && cd bamboo/build
cmake ..
make
```

## Client Installation
```bash
conda create -n bamboo python=3.10
conda activate bamboo
pip install bamboo-franka-controller
```

## Usage

### Server-Side Robot Control
First, run the C++ control node 

```bash
conda activate bamboo
cd bamboo/build
./bamboo_control_node <robot-ip> <grpc-port>
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
*TODO*

## Contributing 
For the python code we enforce style with `ruff` and typechecking with `mypy`. For the C++ code, we enforce style with `clang-tidy`. You can run all linting and checking steps with `pre-commit run --all-files`, they also will run automatically when you make a commit. 

To contribute, please create a fork of the repository, make a feature branch based on main, and commit your changes there. Then open a pull request from that branch.

## Acknowledgements
This work draws heavily from [deoxys\_control](https://github.com/UT-Austin-RPL/deoxys_control) and [drake-franka-driver](https://github.com/RobotLocomotion/drake-franka-driver). Thanks to developers for their open-source code!

## TODO
- [ ] Check that all the examples run after migrating
- [ ] move gripper to GRPC
- [ ] put gripper and robot in the same script
- [ ] delete unused proto defs
- [ ] make sure examples work
- [ ] git protections + release tag
- [ ] clean up comments
- [x] add compilation instructions for debugging / general compilation instructions (Quickstart)
- [x] Acks
- [x] document warnings that come up normally when building
- [x] mention caveat with needing sudo for install
- [x] contributing notes
- [x] update README
- [x] tune kp/kd to be stiffer
- [x] pre-commit + mypy
- [x] cleanup references to deoxys line numbers
- [x] protobuf build should not show up in examples/ directory
- [x] document control frequency parameters
- [x] remove minimal_joint_impedance example
- [x] move libfranka to third party and zmqpp to third_party? maybe?
- [x] move source files out of TLD
