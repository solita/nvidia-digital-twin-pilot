#!/usr/bin/env bash
# check_status.sh — Print instructions to check Isaac Sim status via brev shell.
# Usage: bash scripts/check_status.sh
set -euo pipefail

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTANCE_SCRIPTS_DIR="~/scripts/tutorial"

echo "=== Deploying status script to instance ==="
echo ""

# Create the target directory using sftp (no TTY needed)
echo "mkdir -p ~/scripts/tutorial" | sftp -q isaac-sim

# Copy the status script via scp (no TTY needed)
scp "$SCRIPTS_DIR/instance/check_status_instance.sh" \
    "isaac-sim:$INSTANCE_SCRIPTS_DIR/check_status_instance.sh"

echo ""
echo "=== Script deployed. To check status, run: ==="
echo ""
echo "  brev shell isaac-sim"
echo "  bash $INSTANCE_SCRIPTS_DIR/check_status_instance.sh"
echo ""
