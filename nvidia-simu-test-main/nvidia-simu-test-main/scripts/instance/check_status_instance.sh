#!/usr/bin/env bash
# check_status_instance.sh — Run ON the instance (via brev shell) to check status.
# Usage: bash ~/scripts/tutorial/check_status_instance.sh
set -euo pipefail

echo "=== Containers ==="
docker ps --format 'table {{.Names}}\t{{.Status}}'

echo ""
echo "=== Streaming readiness ==="
curl -s http://127.0.0.1:8011/v1/streaming/ready && echo ""

echo ""
echo "=== GPU memory ==="
nvidia-smi --query-gpu=name,memory.used,memory.free,memory.total --format=csv,noheader

echo ""
echo "=== Disk (docker volumes) ==="
du -sh ~/docker/isaac-sim/cache/main/ov 2>/dev/null || echo "(ov cache not found)"
du -sh ~/docker/isaac-sim/cache/main/warp 2>/dev/null || echo "(warp cache not found)"
