"""
diag_bridge.py — Diagnostic: check OmniGraph twist_sub nodes and forklift prims.

Run via: Ctrl+Shift+P → Isaac Sim: Run File Remotely
"""
from __future__ import annotations
import carb
import omni.graph.core as og
import omni.usd
import omni.timeline

def _log(msg):
    tagged = f"[diag_bridge] {msg}"
    carb.log_warn(tagged)
    print(tagged, flush=True)

stage = omni.usd.get_context().get_stage()

# 1. Timeline state
tl = omni.timeline.get_timeline_interface()
_log(f"Timeline playing: {tl.is_playing()}")

# 2. Check forklift prims
for i in range(4):
    fid = f"forklift_{i}"
    body = stage.GetPrimAtPath(f"/World/{fid}/body")
    drive = stage.GetPrimAtPath(f"/World/{fid}/back_wheel_joints/back_wheel_drive")
    steer = stage.GetPrimAtPath(f"/World/{fid}/back_wheel_joints/back_wheel_swivel")
    _log(f"{fid}: body={body.IsValid()}, drive={drive.IsValid()}, steer={steer.IsValid()}")

# 3. List all graphs
_log("--- OmniGraph graphs ---")
for prim in stage.Traverse():
    if prim.GetTypeName() == "OmniGraph":
        _log(f"  Graph: {prim.GetPath()}")

# 4. Check ROS2CmdBridge graphs and twist_sub outputs
for i in range(4):
    fid = f"forklift_{i}"
    graph_path = f"/World/ROS2CmdBridge_{fid}"
    gp = stage.GetPrimAtPath(graph_path)
    _log(f"--- {graph_path} exists: {gp.IsValid()} ---")
    if not gp.IsValid():
        continue

    twist_sub_path = f"{graph_path}/twist_sub"
    tsp = stage.GetPrimAtPath(twist_sub_path)
    _log(f"  twist_sub prim valid: {tsp.IsValid()}")

    try:
        node = og.get_node_by_path(twist_sub_path)
        _log(f"  twist_sub OG node valid: {node.is_valid()}")
        if node.is_valid():
            # Check topic name
            topic = og.Controller.attribute("inputs:topicName", node).get()
            _log(f"  topicName: {topic}")
            # Check outputs
            lin = og.Controller.attribute("outputs:linearVelocity", node).get()
            ang = og.Controller.attribute("outputs:angularVelocity", node).get()
            _log(f"  outputs:linearVelocity = {lin}")
            _log(f"  outputs:angularVelocity = {ang}")
    except Exception as e:
        _log(f"  ERROR reading twist_sub: {e}")

# 5. Check the old bridge graphs too
for i in range(4):
    fid = f"forklift_{i}"
    old_path = f"/World/ROS2BridgeGraph_{fid}"
    op = stage.GetPrimAtPath(old_path)
    if op.IsValid():
        _log(f"OLD graph still present: {old_path}")

_log("=== Diagnostic complete ===")
