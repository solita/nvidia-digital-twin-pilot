#!/usr/bin/env bash
# deploy_to_instance.sh — Inject NGC key into scripts and scp them to the instance.
# Run from repo root: bash scripts/deploy_to_instance.sh
# Requires: export ISAAC_SIM_IP=<instance-ip>
set -euo pipefail

if [[ -z "${ISAAC_SIM_IP:-}" ]]; then
    echo "ERROR: ISAAC_SIM_IP is not set. Export it before running:"
    echo "  export ISAAC_SIM_IP=<instance-public-ip>"
    exit 1
fi
INSTANCE_IP="$ISAAC_SIM_IP"
INSTANCE_USER="shadeform"
SSH_KEY="${HOME}/.brev/brev.pem"
SSH_OPTS="-i ${SSH_KEY} -o ControlPath=none -o StrictHostKeyChecking=no"
NGC_KEY_FILE="${HOME}/.ngc/key"
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

# --- Validate NGC key ---
if [[ ! -f "$NGC_KEY_FILE" ]]; then
    echo "ERROR: NGC key not found at $NGC_KEY_FILE"
    exit 1
fi
NGC_KEY=$(grep '^nvapi-' "$NGC_KEY_FILE" | head -1)
if [[ -z "$NGC_KEY" ]]; then
    echo "ERROR: No nvapi-... key found in $NGC_KEY_FILE"
    exit 1
fi

# --- Build patched setup.sh in /tmp (NGC key substituted) ---
TMP_SETUP="$(mktemp /tmp/setup.XXXXXXXXXX.sh)"
sed "s|NGC_KEY_PLACEHOLDER|${NGC_KEY}|g" \
    "$REPO_ROOT/scripts/instance/setup.sh" > "$TMP_SETUP"

echo "Deploying to ${INSTANCE_USER}@${INSTANCE_IP}..."

# shellcheck disable=SC2086
scp ${SSH_OPTS} "$TMP_SETUP"                                              "${INSTANCE_USER}@${INSTANCE_IP}:~/setup.sh"
# shellcheck disable=SC2086
scp ${SSH_OPTS} "$REPO_ROOT/scripts/instance/launch_isaac_sim.sh"         "${INSTANCE_USER}@${INSTANCE_IP}:~/launch_isaac_sim.sh"
# shellcheck disable=SC2086
scp ${SSH_OPTS} "$REPO_ROOT/scripts/instance/run_standalone.sh"           "${INSTANCE_USER}@${INSTANCE_IP}:~/run_standalone.sh"
# shellcheck disable=SC2086
scp ${SSH_OPTS} "$REPO_ROOT/scripts/instance/standalone_robot.py"         "${INSTANCE_USER}@${INSTANCE_IP}:~/standalone_robot.py"

# ROS2 files
# shellcheck disable=SC2086
scp ${SSH_OPTS} "$REPO_ROOT/scripts/instance/ros2_jetbot.py"              "${INSTANCE_USER}@${INSTANCE_IP}:/tmp/ros2_jetbot.py"
# shellcheck disable=SC2086
scp ${SSH_OPTS} "$REPO_ROOT/scripts/instance/ros2_sidecar.sh"             "${INSTANCE_USER}@${INSTANCE_IP}:~/ros2_sidecar.sh"
# shellcheck disable=SC2086
scp ${SSH_OPTS} "$REPO_ROOT/scripts/instance/obstacle_avoidance_node.py"  "${INSTANCE_USER}@${INSTANCE_IP}:~/obstacle_avoidance_node.py"
# shellcheck disable=SC2086
scp ${SSH_OPTS} "$REPO_ROOT/scripts/instance/start_ros2_all.sh"           "${INSTANCE_USER}@${INSTANCE_IP}:~/start_ros2_all.sh"

# Dashboard files
# shellcheck disable=SC2086
ssh ${SSH_OPTS} "${INSTANCE_USER}@${INSTANCE_IP}" "mkdir -p ~/dashboard"
# shellcheck disable=SC2086
scp ${SSH_OPTS} \
    "$REPO_ROOT/scripts/instance/dashboard/dashboard_server.py" \
    "$REPO_ROOT/scripts/instance/dashboard/index.html" \
    "$REPO_ROOT/scripts/instance/dashboard/Dockerfile" \
    "${INSTANCE_USER}@${INSTANCE_IP}:~/dashboard/"
# shellcheck disable=SC2086
scp ${SSH_OPTS} "$REPO_ROOT/scripts/instance/dashboard_launch.sh" "${INSTANCE_USER}@${INSTANCE_IP}:~/dashboard_launch.sh"

# ML detector files
# shellcheck disable=SC2086
ssh ${SSH_OPTS} "${INSTANCE_USER}@${INSTANCE_IP}" "mkdir -p ~/ml_detector"
# shellcheck disable=SC2086
scp ${SSH_OPTS} \
    "$REPO_ROOT/scripts/instance/ml_detector/Dockerfile" \
    "$REPO_ROOT/scripts/instance/ml_detector/yolo_detector_node.py" \
    "${INSTANCE_USER}@${INSTANCE_IP}:~/ml_detector/"

# hello_robot.py: scp to /tmp, sudo-copy to host staging dir, then docker cp into container
# shellcheck disable=SC2086
scp ${SSH_OPTS} "$REPO_ROOT/scripts/instance/hello_robot.py"  "${INSTANCE_USER}@${INSTANCE_IP}:/tmp/hello_robot.py"
# Poster image for wall textures
if [[ -f "$REPO_ROOT/ossi.png" ]]; then
    # shellcheck disable=SC2086
    scp ${SSH_OPTS} "$REPO_ROOT/ossi.png" "${INSTANCE_USER}@${INSTANCE_IP}:/tmp/ossi.png"
fi
# shellcheck disable=SC2086
ssh ${SSH_OPTS} "${INSTANCE_USER}@${INSTANCE_IP}" \
    "sudo mkdir -p /home/shadeform/docker/isaac-sim/data && \
     sudo cp /tmp/hello_robot.py /home/shadeform/docker/isaac-sim/data/hello_robot.py && \
     sudo cp /tmp/ros2_jetbot.py /home/shadeform/docker/isaac-sim/data/ros2_jetbot.py && \
     if docker ps --format '{{.Names}}' | grep -q isim-isaac-sim-1; then \
         docker cp /tmp/hello_robot.py isim-isaac-sim-1:/isaac-sim/hello_robot.py && \
         echo 'hello_robot.py -> container:/isaac-sim/hello_robot.py'; \
         docker cp /tmp/ros2_jetbot.py isim-isaac-sim-1:/isaac-sim/ros2_jetbot.py && \
         echo 'ros2_jetbot.py -> container:/isaac-sim/ros2_jetbot.py'; \
         if [ -f /tmp/ossi.png ]; then \
             docker cp /tmp/ossi.png isim-isaac-sim-1:/tmp/ossi.png && \
             echo 'ossi.png -> container:/tmp/ossi.png'; \
         fi; \
     else \
         echo 'Container not running — files staged on host, will be copied by launch_isaac_sim.sh'; \
     fi"

rm -f "$TMP_SETUP"

echo ""
echo "Deployed. Workflow:"
echo ""
echo "  1. On instance (brev shell):"
echo "       sudo bash ~/setup.sh          # if not run yet"
echo "       bash ~/launch_isaac_sim.sh    # start Isaac Sim streaming"
echo ""
echo "  2. On Mac (once instance reports READY):"
echo "       export ISAAC_SIM_IP=$INSTANCE_IP"
echo "       bash scripts/launch_streaming.sh   # tunnel + open Chrome"
echo ""
echo "  3. In Isaac Sim browser (http://localhost:8210):"
echo "       Window > Script Editor"
echo "       File > Open > /isaac-sim/hello_robot.py       # standalone mode"
echo "       File > Open > /isaac-sim/ros2_jetbot.py       # ROS2 mode"
echo "       Ctrl+Enter to run"
echo ""
echo "  4. For ROS2 mode — on instance (brev shell, 2nd terminal):"
echo "       bash ~/start_ros2_all.sh                      # starts sidecar + dashboard + obstacle avoidance"
echo ""
echo "  5. Web dashboard:"
echo "       → http://$INSTANCE_IP:8080"
