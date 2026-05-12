#!/bin/bash
# Startup wrapper for Isaac Sim with automatic scene loading.
# The --exec flag runs the Python script inside the Kit process
# AFTER Isaac Sim is fully initialized (equivalent to waiting for
# "Isaac Sim Full Streaming App is loaded").
set -e

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib

DATA_DIR="/isaac-sim/.local/share/ov/data/nvidia-digital-twin-pilot"
SCENE_SCRIPT="${DATA_DIR}/simulations/forklift-warehouse/02_core_scripts/load_scene.py"

cd /isaac-sim

exec ./runheadless.sh \
    --exec "${SCENE_SCRIPT}" \
    --/app/livestream/publicEndpointAddress=${PUBLIC_IP} \
    --/app/livestream/port=49100
