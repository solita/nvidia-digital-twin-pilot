#!/usr/bin/env bash
# dashboard_launch.sh — Build and run the Isaac Sim web dashboard container.
# Run on instance: bash ~/dashboard_launch.sh
set -euo pipefail

CONTAINER_NAME="ros2-dashboard"
IMAGE_NAME="ros2-dashboard:latest"
DASHBOARD_PORT="${DASHBOARD_PORT:-8080}"

# Check if already running
if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    PUBLIC_IP=$(curl -s ifconfig.me)
    echo "Dashboard already running at http://${PUBLIC_IP}:${DASHBOARD_PORT}"
    echo "  Restart: docker rm -f ${CONTAINER_NAME} && bash ~/dashboard_launch.sh"
    exit 0
fi

# Remove stopped container if exists
docker rm -f "${CONTAINER_NAME}" 2>/dev/null || true

# Build image from ~/dashboard/
echo "Building dashboard image..."
docker build -t "${IMAGE_NAME}" ~/dashboard/

# Open firewall port
sudo ufw allow "${DASHBOARD_PORT}/tcp" 2>/dev/null || true

# Run — net=host for ROS2 DDS, mount docker socket for container monitoring,
# --gpus all so nvidia-smi is available inside the container.
echo "Starting dashboard on port ${DASHBOARD_PORT}..."
docker run -d \
    --name "${CONTAINER_NAME}" \
    --restart unless-stopped \
    --net=host \
    --pid=host \
    --gpus all \
    -v /var/run/docker.sock:/var/run/docker.sock \
    -e ROS_DOMAIN_ID=0 \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e DASHBOARD_PORT="${DASHBOARD_PORT}" \
    "${IMAGE_NAME}"

sleep 2
if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    PUBLIC_IP=$(curl -s ifconfig.me)
    echo ""
    echo "Dashboard running at http://${PUBLIC_IP}:${DASHBOARD_PORT}"
    echo ""
    echo "  Logs:    docker logs -f ${CONTAINER_NAME}"
    echo "  Stop:    docker rm -f ${CONTAINER_NAME}"
    echo "  Rebuild: docker rm -f ${CONTAINER_NAME} && bash ~/dashboard_launch.sh"
else
    echo "ERROR: Container failed to start. Check:"
    echo "  docker logs ${CONTAINER_NAME}"
    exit 1
fi
