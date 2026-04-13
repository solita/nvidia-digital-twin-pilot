#!/usr/bin/env bash
set -euo pipefail

ISAAC_SIM_SH="/isaac-sim/isaac-sim.sh"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/02_core_scripts" && pwd)"
VALIDATE_SCRIPT="shotcrete_direct_cave_preview_validate_v1.py"

if [[ ! -f "$ISAAC_SIM_SH" ]]; then
  echo "Isaac Sim launcher not found:"
  echo "  $ISAAC_SIM_SH"
  exit 1
fi

if [[ ! -f "$SCRIPT_DIR/$VALIDATE_SCRIPT" ]]; then
  echo "Preview validation script not found:"
  echo "  $SCRIPT_DIR/$VALIDATE_SCRIPT"
  exit 1
fi

pushd "$SCRIPT_DIR" > /dev/null
"$ISAAC_SIM_SH" --no-window --exec "$VALIDATE_SCRIPT"
EXIT_CODE=$?
popd > /dev/null

exit $EXIT_CODE
