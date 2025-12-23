#!/bin/bash

SESSION="bamboo"
eval "$(conda shell.bash hook)"

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
    echo "Creating tmux session with multiple panes..."

    # Create new session with first pane for arm control node.
    tmux new-session -d -s "$SESSION" -n "servers" bash -c "eval \"\$(conda shell.bash hook)\" && conda activate bamboo && cd bamboo/build/ && ./bamboo_control_node 172.16.0.2 5555; read"

    # Split horizontally to create gripper pane
    tmux split-window -h -p 50 -t "$SESSION:0"

    # Setup the gripper server pane
    tmux send-keys -t "$SESSION:0.1" "eval \"\$(conda shell.bash hook)\" && conda activate bamboo" C-m
    tmux send-keys -t "$SESSION:0.1" "cd bamboo" C-m
    tmux send-keys -t "$SESSION:0.1" "python gripper_server.py --gripper-port /dev/ttyUSB0 --zmq-port 5559" C-m


    # Set pane titles
    tmux select-pane -t "$SESSION:0.0" -T "bamboo_control_node:5555"
    tmux select-pane -t "$SESSION:0.1" -T "gripper_server:5559"

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

    if tmux has-session -t "$SESSION" 2>/dev/null; then
        echo "✅ Tmux session '$SESSION' is running"
        echo "   Attach with: tmux attach -t $SESSION"
        echo ""

        # Check individual ports
        echo "Port status:"
        if lsof -i ":5555" -sTCP:LISTEN -t >/dev/null 2>&1; then
            echo "  ✅ bamboo_control_node (port 5555) - listening"
        else
            echo "  ❌ bamboo_control_node (port 5555) - not responding"
        fi

        if lsof -i ":8123" -sTCP:LISTEN -t >/dev/null 2>&1; then
            echo "  ✅ gripper_server (port 5559) - listening"
        else
            echo "  ❌ gripper_server (port 5559) - not responding"
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

case "$1" in
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
    "")
        # Default behavior: start if not running, otherwise attach
        if tmux has-session -t "$SESSION" 2>/dev/null; then
            echo "Session '$SESSION' is already running. Attaching..."
            tmux attach -t "$SESSION"
        else
            echo "Session '$SESSION' not found. Starting servers..."
            start_servers
        fi
        ;;
    *)
        echo "Usage: $0 [start|stop|status|attach]"
        echo ""
        echo "With no arguments: starts servers if not running, otherwise attaches"
        echo ""
        echo "Commands:"
        echo "  (no args) - Smart start/attach (default)"
        echo "  start     - Explicitly start servers with workspace in tmux panes:"
        echo "              • bamboo_control_node (left)"
        echo "              • gripper_server (right)"
        echo "  stop      - Stop all servers"
        echo "  status    - Check if servers are running and ports are listening"
        echo "  attach    - Attach to the tmux session to view server logs and run demos"
        ;;
esac
