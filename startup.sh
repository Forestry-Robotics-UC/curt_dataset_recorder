#!/usr/bin/env bash
set -e

# Cleanup function
cleanup() {
    local exit_code=$?
    echo "=== RM3100 CAN0 ==="
    
    # Get container PID if still exists
    local pid
    pid=$(docker inspect -f '{{.State.Pid}}' rm3100 2>/dev/null || echo "")
    
    if [ ! -z "$pid" ] && [ "$pid" != "0" ] && [ "$pid" != "<no value>" ]; then
        echo "Container still running (PID: $pid)"
        
        # Try to move CAN back
        sudo nsenter -t $pid -n ip link set can0 down 2>/dev/null || true
        sudo ip link set can0 netns 1 2>/dev/null && {
            echo "can0 successfully returned to host"
            sudo ip link set can0 up
            #ip -details link show can0
            return $exit_code
        }
    fi
    
    # If we get here, we need to recreate CAN
    echo "Restoring CAN interface on host..."
    
    # Reload gs_usb driver
    echo "Reloading gs_usb driver..."
    sudo rmmod gs_usb 2>/dev/null || true
    sleep 1
    sudo modprobe gs_usb
    sleep 2
    
    # Configure CAN
    if ip link show can0 &>/dev/null; then
        echo "can0 reappeared, configuring..."
        sudo ip link set can0 type can bitrate 1000000
        sudo ip link set can0 up
        echo "=== RM3100 CAN0 Restored ==="
        #ip -details link show can0
    else
        echo "can0 did not reappear automatically"
        echo "Try physically reconnecting the USB-CAN adapter"
        echo "Then run: sudo ip link set can0 type can bitrate 500000 && sudo ip link set can0 up"
    fi
    
    # Eliminate containers
    docker compose down
    
    return $exit_code
}

# Set trap
trap cleanup EXIT

# Bring up the network connection (if needed)
sudo nmcli connection up Ouster

# Kill the ROS2 openzen IMU node from host (if needed)
sudo pkill -f "openzen_node" || true

# --- CONFIGURATION ---
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
cd $SCRIPT_DIR/Docker

# Start container
echo "Starting container..."
docker compose up -d --scale recorder=0

# Wait for container to be ready
sleep 4

# Get container PID
pid=$(docker inspect -f '{{.State.Pid}}' rm3100)

# Move can0 to container
#echo "Moving can0 to container namespace (PID: $pid)"
sudo ip link set can0 down
sudo ip link set can0 netns $pid 2>&1 | grep -v "Invalid argument" || true

# Verify it's in the container
#echo "=== CAN0 Status in Container ==="
#sudo nsenter -t $pid -n ip -details link show can0

# Run the recorder
echo "Starting recorder..."
docker compose run -i --rm recorder
