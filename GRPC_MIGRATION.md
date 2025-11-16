# gRPC Migration Summary

This branch (`grpc`) contains a gRPC-based implementation of the bamboo control node communication, replacing ZMQ for arm control while keeping gripper communication unchanged.

## What Changed

### 1. **New Protocol Buffer Service Definition**
- Added `bamboo/proto/bamboo_service.proto` with gRPC service definitions
- Defines `BambooControlService` with three RPCs:
  - `GetRobotState` - Get current robot state
  - `ExecuteJointImpedanceTrajectory` - Execute a sequence of waypoints
  - `Terminate` - Gracefully shutdown the control node

### 2. **New C++ gRPC Control Node**
- Created `bamboo/src/grpc_control_node.cpp`
- Implements the `BambooControlService` gRPC server
- Replaces ZMQ pub-sub pattern with synchronous RPC calls
- Usage: `./bamboo_grpc_control_node <robot-ip> <grpc-port>`
  - Default gRPC port: 50051 (instead of ZMQ port 5556)

### 3. **New Python gRPC Client**
- Created `bamboo/bamboo_grpc_client.py`
- Provides identical API to `BambooFrankaClient` but uses gRPC
- **Gripper communication still uses ZMQ** (unchanged)
- All public methods preserved:
  - `get_joint_states()`
  - `get_joint_positions()`
  - `execute_joint_impedance_path()`
  - `open_gripper()`, `close_gripper()`, `get_gripper_state()`

### 4. **Build System Updates**
- Updated `bamboo/CMakeLists.txt`:
  - Added gRPC dependency detection (local install or system)
  - Generate gRPC C++ and Python code from proto files
  - Build two executables: `bamboo_control_node` (ZMQ) and `bamboo_grpc_control_node` (gRPC)
  - Python protobuf files now generated to `bamboo/proto_gen/` (not `examples/`)

### 5. **Python Dependencies**
- Updated `pyproject.toml`:
  - Added `grpcio>=1.50.0`
  - Added `grpcio-tools>=1.50.0`

### 6. **Updated Import Paths**
- Both clients now import protobuf from `bamboo/proto_gen/`
- Updated `.gitignore` to ignore generated files

## Migration Benefits

### Single Trajectory Request
- **Before (ZMQ)**: Send N separate messages for N waypoints, wait for completion signal
- **After (gRPC)**: Send 1 message with all N waypoints, get single response when done

### Simpler State Queries
- **Before (ZMQ)**: Subscribe to continuous state stream, filter latest message
- **After (gRPC)**: Direct RPC call returns current state on demand

### Better Error Handling
- gRPC provides structured error codes and detailed error messages
- Timeout handling built into the protocol

### Type Safety
- Service interface clearly defined in proto file
- Compiler-checked client/server compatibility

## Files Added

```
bamboo/proto/bamboo_service.proto          # gRPC service definition
bamboo/src/grpc_control_node.cpp           # gRPC server implementation
bamboo/bamboo_grpc_client.py               # Python gRPC client
```

## Files Modified

```
bamboo/CMakeLists.txt                      # Added gRPC build support
bamboo/bamboo_client.py                    # Updated import path
pyproject.toml                             # Added grpcio dependencies
.gitignore                                 # Ignore proto_gen/
```

## Original ZMQ Implementation

The original ZMQ-based implementation is **preserved**:
- `bamboo/src/control_node.cpp` - Original ZMQ control node
- `bamboo/bamboo_client.py` - Original ZMQ client
- Build produces both `bamboo_control_node` and `bamboo_grpc_control_node`

## Testing

To test on your machine:

1. **Build the project** (requires gRPC C++ installed):
   ```bash
   cd build
   cmake ..
   make
   ```

2. **Start gRPC control node**:
   ```bash
   ./bamboo_grpc_control_node <robot-ip> 50051
   ```

3. **Test with Python client**:
   ```python
   from bamboo.bamboo_grpc_client import BambooFrankaClient

   with BambooFrankaClient(control_port=50051, server_ip="<robot-ip>") as client:
       state = client.get_joint_states()
       print(state)
   ```

## Notes

- **Gripper communication unchanged**: Still uses ZMQ on port 5559
- **Both implementations available**: Can switch between ZMQ and gRPC by changing which server/client you use
- **API compatible**: Drop-in replacement for existing code (just import `bamboo_grpc_client` instead)
