#!/bin/bash
set -euo pipefail

# Load environment
source .env

# Resolve FastDDS config with actual Brev IP
export BREV_TAILSCALE_IP
envsubst < fastdds_client.xml > fastdds_client_resolved.xml

# Verify Brev connectivity
echo "Checking Brev connectivity..."
if ! nc -zw3 "$BREV_TAILSCALE_IP" 11811; then
    echo "ERROR: Cannot reach FastDDS discovery server at $BREV_TAILSCALE_IP:11811"
    echo "Is Tailscale connected? Is Brev running?"
    exit 1
fi
echo "Brev connectivity OK"

# Start services
docker compose -f local-compose.yml up --build -d

echo ""
echo "=== Local services started ==="
echo "Warehouse Manager API: http://localhost:8000"
echo "Warehouse Manager docs: http://localhost:8000/docs"
echo "Redis: localhost:6379"
echo ""
echo "To start dashboard (Phase 5+):"
echo "  docker compose -f local-compose.yml --profile dashboard up --build -d"
