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
To run 

#### For Development
```bash
# Install in development mode with all dependencies
pip install -e ".[all]"
```

#### For Usage Only
```bash
pip install bamboo-franka-controller
```

## Usage

### C++ Control Node
First, build and run the C++ control node:

```bash
cd bamboo/build
./bamboo_control_node <robot-ip> <zmq-port>
```

Example:
```bash
./bamboo_control_node 192.168.1.100 5555
```

### Python Scripts

#### Running directly
```bash
cd bamboo/examples
python test_joint_offset.py 5555
```

## Contributing 
For the python code we enforce style with `ruff` and typechecking with `mypy`. For the C++ code, we enforce style with `clang-tidy`. You can run all linting and checking steps with `pre-commit run --all-files`, they also will run automatically when you make a commit. 

To contribute, please create a fork of the repository, make a feature branch based on main, and commit your changes there. Then open a pull request from that branch.

## TODO
- [ ] add compilation instructions for debugging / general compilation instructions (Quickstart)
- [ ] document warnings that come up normally when building
- [ ] Check that all the examples run after migrating
- [ ] move gripper to GRPC
- [ ] put gripper and robot in the same script
- [ ] bamboo client is sending joint impedances that are not used?
- [ ] delete unused proto defs
- [ ] make sure examples work
- [ ] git protections + release tag
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
