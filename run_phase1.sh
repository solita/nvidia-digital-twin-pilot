#!/usr/bin/env bash
# run_phase1.sh — Run Phase 1 (all on Brev, separate terminals)
#
# This script starts Isaac Sim, then the vehicle controller, then
# publishes a single task via the Phase 1 warehouse manager.
#
# Usage: ./run_phase1.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

export ROS_DOMAIN_ID=42

echo "=== Phase 1: Minimal Loop ==="
echo ""
echo "This starts 3 processes. Use tmux or separate terminals."
echo ""

echo "--- Step 1: Start discovery server + Isaac Sim + Redis ---"
docker compose -f brev-compose.yml up -d discovery-server redis

echo "Waiting for discovery server..."
sleep 3

echo "--- Step 2: Start Isaac Sim ---"
docker compose -f brev-compose.yml up -d isaac-sim

echo "Waiting for Isaac Sim to initialize (shader compilation may take 2-5 min on first run)..."
echo "Monitor with: docker compose -f brev-compose.yml logs -f isaac-sim"
echo ""

echo "--- Step 3: Start Vehicle Controller ---"
docker compose -f brev-compose.yml up -d vehicle-controller

echo "--- Step 4: Start Warehouse Manager ---"
docker compose -f brev-compose.yml up -d warehouse-manager

echo ""
echo "=== All services started ==="
echo ""
echo "Monitor:"
echo "  docker compose -f brev-compose.yml logs -f"
echo "  docker compose -f brev-compose.yml ps"
echo ""
echo "Test (Phase 2 API):"
echo "  curl -X POST http://localhost:8000/orders -H 'Content-Type: application/json' -d '{\"items\": [\"box_001\"], \"priority\": 1}'"
echo "  curl http://localhost:8000/orders"
echo "  curl http://localhost:8000/forklifts"
echo "  curl http://localhost:8000/metrics"
echo ""
echo "Stop:"
echo "  docker compose -f brev-compose.yml down"
