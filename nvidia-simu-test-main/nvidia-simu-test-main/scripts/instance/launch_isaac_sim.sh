#!/usr/bin/env bash
# launch_isaac_sim.sh — Start Isaac Sim streaming on the instance.
# Run inside brev shell: bash ~/launch_isaac_sim.sh
set -euo pipefail

ISAAC_SIM_IMAGE="nvcr.io/nvidia/isaac-sim:5.1.0"
COMPOSE_FILE="$HOME/isaacsim-docker/tools/docker/docker-compose.yml"

if [[ ! -f "$COMPOSE_FILE" ]]; then
    echo "ERROR: docker-compose file not found at $COMPOSE_FILE"
    echo "  Run 'sudo bash ~/setup.sh' first."
    exit 1
fi

PUBLIC_IP=$(curl -s ifconfig.me)
echo "Starting Isaac Sim streaming..."
echo "  Image : $ISAAC_SIM_IMAGE"
echo "  Host  : $PUBLIC_IP"
echo ""

cd "$HOME/isaacsim-docker"
ISAACSIM_HOST="$PUBLIC_IP" \
ISAAC_SIM_IMAGE="$ISAAC_SIM_IMAGE" \
ROS_DISTRO=humble \
RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  docker compose -p isim -f tools/docker/docker-compose.yml up --build -d

echo ""
echo "Waiting for Isaac Sim to become ready (can take 1-2 min)..."
for i in $(seq 1 90); do
    STATUS=$(curl -s http://127.0.0.1:8011/v1/streaming/ready 2>/dev/null || true)
    if echo "$STATUS" | grep -q "Ready for connection"; then
        echo ""
    echo "Isaac Sim is READY."
    echo ""

    # Copy scripts into the container at a browsable path.
    # Container runs as user 'isaac-sim' (uid 1234), home = /isaac-sim/.
    CONTAINER="isim-isaac-sim-1"
    for f in /home/shadeform/docker/isaac-sim/data/hello_robot.py \
             /home/shadeform/docker/isaac-sim/data/ros2_jetbot.py \
              /home/shadeform/standalone_robot.py; do
        if [[ -f "$f" ]]; then
            docker cp "$f" "$CONTAINER:/isaac-sim/$(basename "$f")" && \
                echo "Copied $(basename "$f") -> container:/isaac-sim/$(basename "$f")"
        fi
    done

    # Copy poster image for wall textures (used by ros2_jetbot.py)
    if [[ -f /tmp/ossi.png ]]; then
        docker cp /tmp/ossi.png "$CONTAINER:/tmp/ossi.png" && \
            echo "Copied ossi.png -> container:/tmp/ossi.png"
    fi

    echo ""
    echo "On your Mac:"
    echo "  export ISAAC_SIM_IP=$PUBLIC_IP"
    echo "  bash scripts/launch_streaming.sh"
    echo ""
    echo "In Isaac Sim Script Editor (Window > Script Editor):"
    echo "  File > Open > /isaac-sim/hello_robot.py      # standalone obstacle avoidance"
    echo "  File > Open > /isaac-sim/ros2_jetbot.py      # ROS2 bridge mode"
    echo ""
    echo "For ROS2 mode, also start the sidecar on the instance:"
    echo "  bash ~/ros2_sidecar.sh"
    exit 0
    fi
    printf "\r  %ds elapsed..." "$((i * 10))"
    sleep 10
done

echo ""
echo "Timed out. Check logs: docker compose -p isim logs --tail=50"
exit 1
