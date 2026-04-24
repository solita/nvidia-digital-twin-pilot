#!/usr/bin/env bash
# launch_streaming.sh — Mac-side: start SSH tunnel and poll Isaac Sim readiness.
# Usage: bash scripts/launch_streaming.sh
# Requires: export ISAAC_SIM_IP=<instance-ip>
set -euo pipefail

if [[ -z "${ISAAC_SIM_IP:-}" ]]; then
    echo "ERROR: ISAAC_SIM_IP is not set. Export it before running:"
    echo "  export ISAAC_SIM_IP=<instance-public-ip>"
    exit 1
fi
INSTANCE_IP="$ISAAC_SIM_IP"
SSH_KEY="$HOME/.brev/brev.pem"
SSH_USER="shadeform"

echo "=== Isaac Sim streaming launch ==="
echo "Instance: $SSH_USER@$INSTANCE_IP"
echo ""

# 1. Kill any stale tunnel on port 8210
if lsof -ti tcp:8210 &>/dev/null; then
    echo "[1/3] Killing stale process on port 8210..."
    lsof -ti tcp:8210 | xargs kill -9 2>/dev/null || true
fi

# 2. Open SSH tunnel for web viewer UI (port 8210 only)
echo "[1/3] Opening SSH tunnel: localhost:8210 -> $INSTANCE_IP:8210 ..."
ssh -i "$SSH_KEY" \
    -o StrictHostKeyChecking=no \
    -o ControlPath=none \
    -o ServerAliveInterval=30 \
    -L 8210:localhost:8210 \
    -N -f \
    "$SSH_USER@$INSTANCE_IP"
echo "      Tunnel open."

# 3. Poll Isaac Sim readiness (via direct SSH, not tunnel)
echo "[2/3] Polling Isaac Sim readiness (timeout: 15 min)..."
TIMEOUT=900
ELAPSED=0
INTERVAL=10

while true; do
    STATUS=$(ssh -i "$SSH_KEY" \
        -o StrictHostKeyChecking=no \
        -o ControlPath=none \
        -o ConnectTimeout=5 \
        "$SSH_USER@$INSTANCE_IP" \
        "curl -s http://127.0.0.1:8011/v1/streaming/ready 2>/dev/null" 2>/dev/null || echo "")

    if echo "$STATUS" | grep -q "Ready for connection"; then
        echo ""
        echo "[3/3] Isaac Sim is READY."
        break
    fi

    if [[ $ELAPSED -ge $TIMEOUT ]]; then
        echo ""
        echo "ERROR: Timed out after ${TIMEOUT}s. Check Docker Compose on the instance:"
        echo "  brev shell isaac-sim-2"
        echo "  docker compose -p isim ps"
        exit 1
    fi

    printf "\r      Waiting... %ds elapsed (status: %s)" "$ELAPSED" "${STATUS:-no response}"
    sleep $INTERVAL
    ELAPSED=$((ELAPSED + INTERVAL))
done

echo ""
echo "Open in Chrome: http://localhost:8210"
echo ""
# Open automatically on Mac
open -a "Google Chrome" "http://localhost:8210" 2>/dev/null || \
    open "http://localhost:8210" 2>/dev/null || \
    echo "(Could not open browser automatically — open http://localhost:8210 manually)"
