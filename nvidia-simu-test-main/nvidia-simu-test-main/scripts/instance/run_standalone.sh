#!/usr/bin/env bash
# run_standalone.sh — Run on the instance to execute the Part 3 standalone Jetbot script.
# Usage: bash ~/scripts/run_standalone.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_SCRIPT="$SCRIPT_DIR/standalone_robot.py"
IMAGE="nvcr.io/nvidia/isaac-sim:5.1.0"
CONTAINER_NAME="isim-standalone"
LOG_FILE="$SCRIPT_DIR/standalone_robot.log"

echo "=== Part 3: Standalone Jetbot simulation ==="

# Remove any previous run container
docker rm -f "$CONTAINER_NAME" 2>/dev/null && echo "Removed stale container." || true

# Check GPU memory available
GPU_FREE=$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits 2>/dev/null | head -1)
echo "GPU free memory: ${GPU_FREE} MiB"
if [[ -n "$GPU_FREE" && "$GPU_FREE" -lt 4000 ]]; then
    echo "WARNING: Only ${GPU_FREE} MiB GPU free. Standalone may fail if streaming container is using all GPU memory."
    echo "Consider stopping streaming first: docker compose -p isim down"
fi

echo "Starting container..."
docker run --rm --gpus all \
    -e ACCEPT_EULA=Y \
    -e PRIVACY_CONSENT=Y \
    -v ~/docker/isaac-sim/cache/main/ov:/isaac-sim/.cache/ov:rw \
    -v ~/docker/isaac-sim/cache/main/warp:/isaac-sim/.cache/warp:rw \
    -v ~/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw \
    -v ~/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw \
    -v ~/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw \
    -v ~/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw \
    -v "$SCRIPT_DIR":/scripts:rw \
    --name "$CONTAINER_NAME" \
    -u 1234:1234 \
    "$IMAGE" \
    /isaac-sim/python.sh /scripts/standalone_robot.py 2>&1 | tee "$LOG_FILE" | grep --line-buffered -E "pos=|Done\.|ERROR|Error|Traceback|assets_root|Wheel|steps"

echo ""
echo "Full log saved to: $LOG_FILE"
