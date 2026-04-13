#!/usr/bin/env bash
# run_sim.sh — Generic Isaac Sim launch script.
# Rename and update LAUNCHER_SCRIPT to match your simulation's entry point.
set -euo pipefail

ISAAC_SIM_SH="/isaac-sim/isaac-sim.sh"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/02_core_scripts" && pwd)"
LAUNCHER_SCRIPT="launcher.py"   # <- update this to your launcher filename

if [[ ! -f "$ISAAC_SIM_SH" ]]; then
  echo "Isaac Sim launcher not found: $ISAAC_SIM_SH"
  exit 1
fi

if [[ ! -f "$SCRIPT_DIR/$LAUNCHER_SCRIPT" ]]; then
  echo "Launcher script not found: $SCRIPT_DIR/$LAUNCHER_SCRIPT"
  exit 1
fi

pushd "$SCRIPT_DIR" > /dev/null
"$ISAAC_SIM_SH" --exec "$LAUNCHER_SCRIPT"
EXIT_CODE=$?
popd > /dev/null

exit $EXIT_CODE
