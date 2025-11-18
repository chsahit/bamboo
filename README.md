# Bamboo Franka Controller

A Python package for controlling Franka robots using joint impedance control with GRPC to a C++ control node.

## Installation

### Prerequisites
Make sure user is part of realtime group and that realtime kernel is set up as per instructions on libfranka page.

### Build C++ Dependencies
```bash
bash InstallPackage
```

### Install Python Package

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

## Requirements

- Python >= 3.8
- Linux (for Franka robot communication)
- Built C++ control node (`bamboo_control_node`)
- Franka robot connected to network

## Contributing 
For the python code we enforce style with `ruff` and typechecking with `mypy`. For the C++ code, we enforce style with `clang-tidy`. You can run all linting and checking steps with `pre-commit run --all-files`, they also will run automatically when you make a commit. 

To contribute, please create a fork of the repository, make a feature branch based on main, and commit your changes there. Then open a pull request from that branch.

## TODO
- [ ] tune kp/kd to be stiffer
- [ ] add compilation instructions for debugging / general compilation instructions (Quickstart)
- [ ] mention caveat with needing sudo for install
- [ ] document warnings that come up normally when building
- [ ] Check that all the examples run after migrating
- [ ] move gripper to GRPC
- [ ] put gripper and robot in the same script
- [ ] bamboo client is sending joint impedances that are not used?
- [ ] delete unused proto defs
- [ ] make sure examples work
- [ ] update README
- [ ] git protections + release tag
- [ ] contributing notes
- [x] pre-commit + mypy
- [x] cleanup references to deoxys line numbers
- [x] protobuf build should not show up in examples/ directory
- [x] document control frequency parameters
- [x] remove minimal_joint_impedance example
- [x] move libfranka to third party and zmqpp to third_party? maybe?
- [x] move source files out of TLD
