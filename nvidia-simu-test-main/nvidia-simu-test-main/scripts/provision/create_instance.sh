#!/usr/bin/env bash
# create_instance.sh — Recreate the isaac-sim GPU instance on Hyperstack via Brev.
# Usage: bash scripts/provision/create_instance.sh
set -euo pipefail

INSTANCE_NAME="isaac-sim"
INSTANCE_TYPE="hyperstack_A4000"
STARTUP_SCRIPT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/startup.sh"

# Check the startup script exists
if [[ ! -f "$STARTUP_SCRIPT" ]]; then
    echo "ERROR: startup script not found at $STARTUP_SCRIPT"
    exit 1
fi

echo "=== Creating Brev instance ==="
echo "  Name:    $INSTANCE_NAME"
echo "  Type:    $INSTANCE_TYPE  (NVIDIA A4000 16GiB, 4 vCPU, 21GiB RAM, 100GB disk)"
echo "  Region:  oslo-norway-1 (hyperstack:shadeform)"
echo "  Cost:    \$0.18/hr"
echo "  Startup: $STARTUP_SCRIPT"
echo ""

yes | brev create "$INSTANCE_NAME" \
    --type "$INSTANCE_TYPE" \
    --startup-script "@$STARTUP_SCRIPT"

echo ""
echo "=== Instance created. Waiting for startup script to finish... ==="
echo ""
echo "The startup script (~15 min on first boot):"
echo "  1. Opens UFW ports 49100/tcp and 47998/udp"
echo "  2. Creates ~/docker/isaac-sim/ cache directory structure"
echo "  3. Sparse-checks out tools/docker from isaac-sim/IsaacSim"
echo "  4. Pulls nvcr.io/nvidia/isaac-sim:5.1.0"
echo ""
echo "Monitor startup log:"
echo "  brev shell $INSTANCE_NAME"
echo "  tail -f /var/log/isaac-sim-startup.log"
echo ""
echo "Once done, find the public IP:"
echo "  brev ls"
echo ""
echo "Then launch streaming (see INSTALL.md §3e):"
echo "  brev shell $INSTANCE_NAME"
echo "  cd ~/isaacsim-docker"
echo "  ISAACSIM_HOST=<public-ip> ISAAC_SIM_IMAGE=nvcr.io/nvidia/isaac-sim:5.1.0 \\"
echo "    docker compose -p isim -f tools/docker/docker-compose.yml up --build -d"
