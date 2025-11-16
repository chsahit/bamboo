# Bamboo Franka Controller

A Python package for controlling Franka robots using joint impedance control with ZMQ communication to C++ control nodes.

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

## Dependencies

### Core Dependencies
- **numpy**: Numerical computing
- **pyzmq**: ZeroMQ Python bindings for communication
- **protobuf**: Protocol Buffers for message serialization

### Optional Dependencies
- **robotics**: Robotics libraries (robotics-toolbox-python, spatialmath-python)
- **visualization**: Plotting and visualization tools
- **dev**: Development and testing tools

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

#### Using the installed package
```bash
bamboo-test-joint-offset 5555
```

#### Running directly
```bash
cd bamboo/examples
python test_joint_offset.py 5555
```

## Project Structure

```
bamboo/
├── bamboo/                 # Main Python package
│   ├── examples/           # Example scripts
│   │   ├── test_joint_offset.py
│   │   ├── franka_controller_pb2.py
│   │   └── franka_robot_state_pb2.py
│   └── __init__.py
├── bamboo/                 # C++ source (separate from Python package)
│   ├── src/
│   ├── include/
│   ├── proto/
│   └── CMakeLists.txt
├── pyproject.toml          # Python package configuration
└── README.md
```

## Requirements

- Python >= 3.8
- Linux (for Franka robot communication)
- Built C++ control node (`bamboo_control_node`)
- Franka robot connected to network

## Development

```bash
# Install development dependencies
pip install -e ".[dev]"

# Run tests
pytest

# Format code
black bamboo/
isort bamboo/

# Type checking
mypy bamboo/
```
