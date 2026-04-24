#!/usr/bin/env bash
# start_ros2_all.sh — Start all ROS2 services for the Jetbot simulation.
#
# Starts (if not already running):
#   1. ROS2 sidecar container (humble-desktop, --net=host)
#   2. Web dashboard container (port 8080)
#   3. Obstacle avoidance node inside the sidecar
#
# Prerequisites:
#   - Isaac Sim container running (isim-isaac-sim-1)
#   - ros2_jetbot.py already executed in Script Editor
#
# Run on instance: bash ~/start_ros2_all.sh
set -euo pipefail

PUBLIC_IP=$(curl -s ifconfig.me)
SIDECAR="ros2-sidecar"
DASHBOARD="ros2-dashboard"
ML_DETECTOR="yolo-detector"

echo "============================================"
echo "  Starting ROS2 services"
echo "============================================"
echo ""

# ── 1. ROS2 sidecar ──────────────────────────────────────────────────────────
echo "[1/4] ROS2 sidecar..."
if docker ps --format '{{.Names}}' | grep -q "^${SIDECAR}$"; then
    echo "  Already running."
else
    bash ~/ros2_sidecar.sh
fi

# ── 2. Web dashboard ─────────────────────────────────────────────────────────
echo "[2/4] Web dashboard..."
if docker ps --format '{{.Names}}' | grep -q "^${DASHBOARD}$"; then
    echo "  Already running at http://${PUBLIC_IP}:8080"
else
    bash ~/dashboard_launch.sh
fi

# ── 3. YOLO ML detector ───────────────────────────────────────────────────────
echo "[3/4] YOLO ML detector..."
if docker ps --format '{{.Names}}' | grep -q "^${ML_DETECTOR}$"; then
    echo "  Already running."
else
    # Build image if it doesn't exist yet (first build downloads ~4 GB)
    if ! docker image inspect yolo-detector:latest &>/dev/null; then
        echo "  Building yolo-detector image (first run — downloads PyTorch+CUDA, ~5 min)..."
        cd ~/ml_detector && docker build -t yolo-detector .
        echo "  Image built."
    fi
    docker rm -f "${ML_DETECTOR}" 2>/dev/null || true
    docker run -d --name "${ML_DETECTOR}" --net=host --gpus all yolo-detector
    echo "  YOLO detector started."
fi

# ── 4. Wait for /scan topic, then start obstacle avoidance ────────────────────
echo "[4/4] Obstacle avoidance node..."

# Check if already running
if docker exec "${SIDECAR}" bash -c "ps aux 2>/dev/null" | grep -q "obstacle_avoidance_node"; then
    echo "  Already running."
else
    echo "  Waiting for /scan topic from Isaac Sim..."
    TRIES=0
    MAX_TRIES=30
    while ! docker exec "${SIDECAR}" bash -c "source /opt/ros/humble/setup.bash && ros2 topic list 2>/dev/null" | grep -q "/scan"; do
        TRIES=$((TRIES + 1))
        if [[ $TRIES -ge $MAX_TRIES ]]; then
            echo "  WARNING: /scan topic not found after ${MAX_TRIES}s."
            echo "  Make sure ros2_jetbot.py is running in Isaac Sim Script Editor."
            echo "  You can start obstacle avoidance manually later:"
            echo "    docker exec -d ${SIDECAR} bash -c 'source /opt/ros/humble/setup.bash && python3 /ros2_ws/obstacle_avoidance_node.py'"
            exit 1
        fi
        sleep 1
    done
    echo "  /scan topic found. Starting obstacle avoidance..."
    docker exec -d "${SIDECAR}" bash -c \
        "source /opt/ros/humble/setup.bash && python3 /ros2_ws/obstacle_avoidance_node.py"
    sleep 1
    # Verify it started
    if docker exec "${SIDECAR}" bash -c "ps aux" | grep -q "obstacle_avoidance_node"; then
        echo "  Obstacle avoidance running."
    else
        echo "  WARNING: Failed to start obstacle avoidance. Check logs:"
        echo "    docker exec ${SIDECAR} bash -c 'source /opt/ros/humble/setup.bash && python3 /ros2_ws/obstacle_avoidance_node.py'"
    fi
fi

echo ""
echo "============================================"
echo "  All services running"
echo "============================================"
echo ""
echo "  Isaac Sim streaming : http://${PUBLIC_IP}:49100"
echo "  Web dashboard       : http://${PUBLIC_IP}:8080"
echo ""
echo "  ROS2 topics:"
docker exec "${SIDECAR}" bash -c "source /opt/ros/humble/setup.bash && ros2 topic list 2>/dev/null" | sed 's/^/    /'
echo ""
echo "  Useful commands:"
echo "    docker exec -it ${SIDECAR} bash                    # ROS2 shell"
echo "    docker logs -f ${DASHBOARD}                        # dashboard logs"
echo "    docker exec ${SIDECAR} bash -c 'kill \$(pgrep -f obstacle_avoidance)'  # stop avoidance"
echo ""
