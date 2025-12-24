#!/bin/bash

SESSION="bamboo"
eval "$(conda shell.bash hook)"

# Default values
ROBOT_IP="172.16.0.2"
ROBOT_PORT="5555"
LISTEN_ADDR="*"
GRIPPER_DEVICE="/dev/ttyUSB0"
GRIPPER_PORT="5559"
COMMAND=""

# Parse arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --robot_addr)
      ROBOT_IP="$2"
      shift 2
      ;;
    --control_port)
      ROBOT_PORT="$2"
      shift 2
      ;;
    --listen_addr)
      LISTEN_ADDR="$2"
      shift 2
      ;;
    --gripper_device)
      GRIPPER_DEVICE="$2"
      shift 2
      ;;
    --gripper_port)
      GRIPPER_PORT="$2"
      shift 2
      ;;
    --help|-h)
      echo "Usage: $0 [COMMAND] [OPTIONS]"
      echo ""
      echo "Commands:"
      echo "  start     - Start servers (default if no command given and session not running)"
      echo "  stop      - Stop all servers"
      echo "  status    - Check server status"
      echo "  attach    - Attach to tmux session"
      echo ""
      echo "Options:"
      echo "  --robot_addr ADDR       Robot IP address (default: $ROBOT_IP)"
      echo "  --control_port PORT     Control node port (default: $ROBOT_PORT)"
      echo "  --listen_addr ADDR      Listen address (default: $LISTEN_ADDR)"
      echo "  --gripper_device DEV    Gripper device (default: $GRIPPER_DEVICE)"
      echo "  --gripper_port PORT     Gripper server port (default: $GRIPPER_PORT)"
      echo "  --help, -h              Show this help"
      echo ""
      echo "Examples:"
      echo "  $0 start --robot_addr 192.168.1.100 --control_port 6000"
      echo "  $0 --listen_addr 127.0.0.1 --gripper_device /dev/ttyUSB1"
      exit 0
      ;;
    start|stop|status|attach)
      if [[ -z "$COMMAND" ]]; then
        COMMAND="$1"
      else
        echo "Error: Multiple commands specified"
        exit 1
      fi
      shift
      ;;
    -*)
      echo "Unknown option: $1"
      echo "Use --help for usage information"
      exit 1
      ;;
    *)
      if [[ -z "$COMMAND" ]]; then
        COMMAND="$1"
      else
        echo "Error: Unknown argument: $1"
        exit 1
      fi
      shift
      ;;
  esac
done

# Default command if none specified
if [[ -z "$COMMAND" ]]; then
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    COMMAND="attach"
  else
    COMMAND="start"
  fi
fi

check_port_health() {
    local port=$1
    local max_wait=30
    local waited=0

    echo "  Waiting for port $port..."
    while [ $waited -lt $max_wait ]; do
        if lsof -i ":$port" -sTCP:LISTEN -t >/dev/null 2>&1; then
            echo "  ✅ Port $port is listening"
            return 0
        fi
        sleep 1
        waited=$((waited + 1))
    done

    echo "  ❌ Port $port didn't start within ${max_wait}s"
    return 1
}


start_servers() {
    # Check if session already exists
    if tmux has-session -t "$SESSION" 2>/dev/null; then
        echo "❌ Session '$SESSION' already exists. Stop it first with: $0 stop"
        return 1
    fi

    echo "Starting servers with configuration:"
    echo "  Robot IP: $ROBOT_IP"
    echo "  Control port: $ROBOT_PORT"
    echo "  Listen address: $LISTEN_ADDR"
    echo "  Gripper device: $GRIPPER_DEVICE"
    echo "  Gripper port: $GRIPPER_PORT"
    echo "Creating tmux session with multiple panes..."

    # Create new session with first pane for arm control node.
    tmux new-session -d -s "$SESSION" -n "servers" bash -c "eval \"\$(conda shell.bash hook)\" && conda activate bamboo && cd bamboo/build/ && ./bamboo_control_node -r \"$ROBOT_IP\" -p \"$ROBOT_PORT\" -l \"$LISTEN_ADDR\"; read"

    # Split horizontally to create gripper pane
    tmux split-window -h -p 50 -t "$SESSION:0"

    # Setup the gripper server pane
    tmux send-keys -t "$SESSION:0.1" "eval \"\$(conda shell.bash hook)\" && conda activate bamboo" C-m
    tmux send-keys -t "$SESSION:0.1" "cd bamboo" C-m
    tmux send-keys -t "$SESSION:0.1" "python gripper_server.py --gripper-port $GRIPPER_DEVICE --zmq-port $GRIPPER_PORT" C-m


    # Set pane titles
    tmux select-pane -t "$SESSION:0.0" -T "bamboo_control_node:$ROBOT_PORT"
    tmux select-pane -t "$SESSION:0.1" -T "gripper_server:$GRIPPER_PORT"

    # Select the control node pane by default
    tmux select-pane -t "$SESSION:0.0"

    echo "✅ All servers started successfully!"
    tmux attach -t "$SESSION"
}

stop_servers() {
    echo "Stopping servers..."
    if tmux has-session -t "$SESSION" 2>/dev/null; then
        tmux kill-session -t "$SESSION"
        echo "✅ Stopped $SESSION"
    else
        echo "⚠️  Session '$SESSION' is not running"
    fi
}

status_servers() {
    echo "Checking server status..."
    echo "Configuration:"
    echo "  Robot IP: $ROBOT_IP"
    echo "  Control port: $ROBOT_PORT"
    echo "  Listen address: $LISTEN_ADDR"
    echo "  Gripper device: $GRIPPER_DEVICE"
    echo "  Gripper port: $GRIPPER_PORT"
    echo ""

    if tmux has-session -t "$SESSION" 2>/dev/null; then
        echo "✅ Tmux session '$SESSION' is running"
        echo "   Attach with: tmux attach -t $SESSION"
        echo ""

        # Check individual ports
        echo "Port status:"
        if lsof -i ":$ROBOT_PORT" -sTCP:LISTEN -t >/dev/null 2>&1; then
            echo "  ✅ bamboo_control_node (port $ROBOT_PORT) - listening"
        else
            echo "  ❌ bamboo_control_node (port $ROBOT_PORT) - not responding"
        fi

        if lsof -i ":$GRIPPER_PORT" -sTCP:LISTEN -t >/dev/null 2>&1; then
            echo "  ✅ gripper_server (port $GRIPPER_PORT) - listening"
        else
            echo "  ❌ gripper_server (port $GRIPPER_PORT) - not responding"
        fi
    else
        echo "❌ Session '$SESSION' is not running"
        echo "   Start with: $0 start"
    fi
}

attach_servers() {
    if tmux has-session -t "$SESSION" 2>/dev/null; then
        tmux attach -t "$SESSION"
    else
        echo "❌ Session '$SESSION' is not running"
        echo "   Start with: $0 start"
    fi
}

case "$COMMAND" in
    start)
        start_servers
        ;;
    stop)
        stop_servers
        ;;
    status)
        status_servers
        ;;
    attach)
        attach_servers
        ;;
    *)
        echo "Unknown command: $COMMAND"
        echo "Use --help for usage information"
        exit 1
        ;;
esac
