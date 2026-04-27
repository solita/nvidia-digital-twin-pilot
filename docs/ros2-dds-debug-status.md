# ROS 2 DDS Debugging — Session Status

## Problem
Forklifts don't move in Isaac Sim. ROS 2 cmd_vel messages from vehicle-controller never arrive at Isaac Sim's OmniGraph twist_sub nodes.

## Root Cause Investigation

### What's been proven
1. **Isaac Sim 5.1.0 uses ROS 2 Jazzy internally** — confirmed by `/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/` directory
2. **All containers migrated from Humble → Jazzy** — discovery-server, vehicle-controller, warehouse-manager all now use `ros:jazzy` base
3. **FastDDS versions are compatible** — Isaac Sim has 2.14.5, vehicle-controller has 2.14.6 (minor patch diff, same wire protocol)
4. **Isaac Sim DDS env vars are correct** — `ROS_DISCOVERY_SERVER=localhost:11811`, `ROS_DOMAIN_ID=42`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
5. **Vehicle-controller DDS env vars are correct** — same settings, `ROS_DISTRO=jazzy`
6. **All services use `network_mode: host`** — no Docker network isolation
7. **OmniGraph bridge graphs exist and are valid** — 4 twist_sub nodes subscribed to /forklift_N/cmd_vel
8. **rclpy self-test PASSES inside Isaac Sim** — diag_dds3.py published+subscribed to /diag_dds3_test, received 60 messages in 6s
9. **External cmd_vel FAILS** — diag_dds3.py received 0 messages from vehicle-controller on /forklift_0/cmd_vel
10. **OmniGraph twist_sub outputs are all zeros** — diag_twist_check.py confirms [0,0,0] for all forklifts

### Key diagnostic: rclpy self-pub works but cross-container doesn't
- diag_dds3.py creates a fresh rclpy node inside Isaac Sim
- Self-publish to /diag_dds3_test → receives 60 msgs ✓
- Subscribe to /forklift_0/cmd_vel (from vehicle-controller container) → 0 msgs ✗
- Both BEST_EFFORT and RELIABLE QoS tested → 0 msgs

### What this means
The DDS participant inside Isaac Sim CAN discover and communicate locally, but messages from external processes (vehicle-controller, ros2 topic pub) are NOT delivered. This is NOT a Humble/Jazzy mismatch anymore — both sides are Jazzy.

## What's been done
1. ✅ Diagnosed initial issue (populate_scene.py not run)
2. ✅ Ran populate_scene.py + ros2_cmd_bridge.py (bridge works, but twist_sub outputs zero)
3. ✅ Created diag_bridge.py — confirmed OG graphs valid, outputs zero
4. ✅ Created diag_dds2.py — confirmed rclpy in Isaac Sim discovers topics but receives 0 msgs (was Humble era)
5. ✅ Migrated all containers from Humble → Jazzy (vehicle-controller/Dockerfile, warehouse-manager/Dockerfile, brev-compose.yml)
6. ✅ Rebuilt and restarted Jazzy containers
7. ✅ Created diag_dds3.py — self-test passes (60 msgs), external still fails (0 msgs)
8. ✅ Checked FastDDS versions (2.14.5 vs 2.14.6 — compatible)
9. ✅ Checked bridge extension config — `ros_distro = "system_default"`

## What to investigate next

### Hypothesis 1: Bridge extension using Humble libs, not Jazzy
- `ros_distro = "system_default"` in extension.toml — how does it resolve?
- Need to check: `docker exec nvidia-digital-twin-pilot-isaac-sim-1 bash -c 'grep -rn "system_default\|ros_distro" /isaac-sim/exts/isaacsim.ros2.bridge/isaacsim/ros2/bridge/impl/extension.py'`
- The extension has BOTH humble/ and jazzy/ lib dirs. If it loads humble libs, the OmniGraph nodes use humble DDS while rclpy uses jazzy DDS
- This would explain: rclpy self-test works (jazzy↔jazzy) but OG twist_sub fails (maybe humble↔jazzy)
- **ACTION**: Read extension.py to find distro resolution logic. Check Isaac Sim logs for which distro was loaded.

### Hypothesis 2: Discovery server stale participant
- Isaac Sim was NOT restarted when discovery-server was
- The old DDS participant might have stale registration
- diag_dds3.py self-test works because it creates a NEW rclpy node
- But OmniGraph twist_sub nodes were created before discovery restart
- **ACTION**: Re-run ros2_cmd_bridge.py to recreate OG graphs (fresh DDS subscribers). Already suggested but user may not have done this after Jazzy migration.

### Hypothesis 3: FASTRTPS_DEFAULT_PROFILES_FILE not set in Isaac Sim
- diag_dds3.py showed `FASTRTPS_DEFAULT_PROFILES_FILE=<not set>`
- But `ROS_DISCOVERY_SERVER=localhost:11811` IS set — in Jazzy, this should be sufficient
- The fastdds_client.xml in the repo uses `${BREV_TAILSCALE_IP}` (for remote access), NOT localhost
- If the OG bridge extension loads its own FastDDS profiles, it might bypass the env var
- **ACTION**: Check if OG twist_sub nodes are using discovery server or simple discovery

### Hypothesis 4: OmniGraph bridge runs in a separate DDS domain or process
- OG nodes might use a C++ DDS participant separate from rclpy
- The C++ bridge might not inherit env vars the same way
- **ACTION**: Check Isaac Sim logs for "ROS2" or "fastrtps" initialization messages

## Recommended next steps (in order)
1. **Check which ROS distro the bridge extension loaded**: 
   ```bash
   docker exec nvidia-digital-twin-pilot-isaac-sim-1 bash -c 'cat /isaac-sim/exts/isaacsim.ros2.bridge/isaacsim/ros2/bridge/impl/extension.py' | grep -A20 "system_default\|ros_distro\|_get_ros"
   ```
2. **Check Isaac Sim startup logs for bridge distro**:
   ```bash
   docker logs nvidia-digital-twin-pilot-isaac-sim-1 2>&1 | grep -i "ros2.*bridge\|ros_distro\|humble\|jazzy\|fastrtps" | head -20
   ```
3. **Re-run ros2_cmd_bridge.py** in Isaac Sim (Ctrl+Shift+P → Run File Remotely) to create fresh OG graphs
4. **Simultaneously publish cmd_vel** and check twist_sub outputs:
   ```bash
   docker exec nvidia-digital-twin-pilot-vehicle-controller-1 bash -c "source /opt/ros/jazzy/setup.bash && timeout 10 ros2 topic pub -r 10 /forklift_0/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}}'"
   ```
   Then run diag_twist_check.py remotely
5. If still failing, **try bypassing discovery server** — test with simple discovery (unset ROS_DISCOVERY_SERVER) between a test publisher on host and Isaac Sim

## Files created for diagnostics (can clean up later)
- `simulations/forklift-warehouse/02_core_scripts/diag_bridge.py`
- `simulations/forklift-warehouse/02_core_scripts/diag_dds2.py`  
- `simulations/forklift-warehouse/02_core_scripts/diag_dds3.py`
- `simulations/forklift-warehouse/02_core_scripts/diag_twist_check.py`

## Key files modified in this session
- `vehicle-controller/Dockerfile` — ros:humble → ros:jazzy, all package names updated
- `warehouse-manager/Dockerfile` — ros:humble → ros:jazzy, --break-system-packages
- `brev-compose.yml` — discovery-server/vehicle-controller/warehouse-manager all Jazzy
