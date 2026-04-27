"""Quick diagnostic: read twist_sub output values for all forklifts."""
import omni.graph.core as og
import omni.usd

stage = omni.usd.get_context().get_stage()

for i in range(4):
    fid = f"forklift_{i}"
    graph_path = f"/World/ROS2CmdBridge_{fid}"
    node_path = f"{graph_path}/twist_sub"
    prim = stage.GetPrimAtPath(graph_path)
    print(f"[diag_twist] {fid}: graph exists={prim.IsValid()}", flush=True)
    if not prim.IsValid():
        continue
    try:
        node = og.Controller.node(node_path)
        lin = og.Controller.attribute("outputs:linearVelocity", node).get()
        ang = og.Controller.attribute("outputs:angularVelocity", node).get()
        topic = og.Controller.attribute("inputs:topicName", node).get()
        print(f"[diag_twist]   topic={topic}  lin={lin}  ang={ang}", flush=True)
    except Exception as e:
        print(f"[diag_twist]   ERROR: {e}", flush=True)

print("[diag_twist] Done", flush=True)
