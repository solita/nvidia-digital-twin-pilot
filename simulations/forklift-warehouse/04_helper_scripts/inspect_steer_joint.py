"""
inspect_steer_joint.py — Read joint body relationships and current state.
Writes output to the Isaac Sim console AND 04_current_outputs/steer_joint_latest.txt

Run via VS Code: Ctrl+Shift+P → Isaac Sim: Run File Remotely
"""
import omni.usd
from pxr import UsdPhysics

JOINTS = [
    "/World/forklift_b/back_wheel_joints/back_wheel_drive",
    "/World/forklift_b/back_wheel_joints/back_wheel_swivel",
]
OUTPUT = "/isaac-sim/.local/share/ov/data/nvidia-digital-twin-pilot/simulations/forklift-warehouse/04_current_outputs/steer_joint_latest.txt"

stage = omni.usd.get_context().get_stage()
lines = []

def pr(s=""):
    print(s)
    lines.append(str(s))

pr("=" * 70)
pr("JOINT BODY RELATIONSHIPS + FULL ATTRIBUTES")
pr("=" * 70)

for jpath in JOINTS:
    prim = stage.GetPrimAtPath(jpath)
    pr(f"\nJoint: {jpath}")
    if not prim.IsValid():
        pr("  ERROR: not found")
        continue
    pr(f"  Type: {prim.GetTypeName()}")

    # Body relationships (USD stores these as Rel, not Attr)
    for rel_name in ("physics:body0", "physics:body1"):
        rel = prim.GetRelationship(rel_name)
        if rel:
            targets = rel.GetTargets()
            pr(f"  {rel_name}: {targets}")
        else:
            pr(f"  {rel_name}: (no relationship)")

    # All attributes
    pr("  Attributes:")
    for attr in prim.GetAttributes():
        val = attr.Get()
        if val is not None:
            pr(f"    {attr.GetName()} = {val}")

pr("\n" + "=" * 70)

with open(OUTPUT, "w") as f:
    f.write("\n".join(lines))
print(f"[inspect] Written to {OUTPUT}")


lines = []
def pr(s=""):
    print(s)
    lines.append(s)

stage = omni.usd.get_context().get_stage()

steer_joint = stage.GetPrimAtPath(STEER_JOINT_PATH)
drive_joint = stage.GetPrimAtPath(DRIVE_JOINT_PATH)

pr("\n" + "="*60)
pr("STEER JOINT INSPECTION")
pr("="*60)

if steer_joint.IsValid():
    steer_api = UsdPhysics.DriveAPI(steer_joint, "angular")
    pr(f"Steer joint prim:       {STEER_JOINT_PATH}")
    pr(f"  targetPosition:       {steer_api.GetTargetPositionAttr().Get()}")
    pr(f"  stiffness:            {steer_api.GetStiffnessAttr().Get()}")
    pr(f"  damping:              {steer_api.GetDampingAttr().Get()}")
    pr(f"  maxForce:             {steer_api.GetMaxForceAttr().Get()}")
    lower = steer_joint.GetAttribute("physics:lowerLimit").Get()
    upper = steer_joint.GetAttribute("physics:upperLimit").Get()
    pr(f"  joint limits:         lower={lower}  upper={upper}")
    pr("\n  All joint attributes:")
    for attr in steer_joint.GetAttributes():
        val = attr.Get()
        if val is not None:
            pr(f"    {attr.GetName()}: {val}")
else:
    pr(f"ERROR: steer joint not found at {STEER_JOINT_PATH}")

pr("\n" + "="*60)
pr("DRIVE JOINT INSPECTION")
pr("="*60)

if drive_joint.IsValid():
    drive_api = UsdPhysics.DriveAPI(drive_joint, "angular")
    pr(f"Drive joint prim:       {DRIVE_JOINT_PATH}")
    pr(f"  targetVelocity:       {drive_api.GetTargetVelocityAttr().Get()}")
    pr(f"  stiffness:            {drive_api.GetStiffnessAttr().Get()}")
    pr(f"  damping:              {drive_api.GetDampingAttr().Get()}")
    pr("\n  All joint attributes:")
    for attr in drive_joint.GetAttributes():
        val = attr.Get()
        if val is not None:
            pr(f"    {attr.GetName()}: {val}")
else:
    pr(f"ERROR: drive joint not found at {DRIVE_JOINT_PATH}")

pr("="*60 + "\n")

os.makedirs(os.path.dirname(OUTPUT_FILE), exist_ok=True)
with open(OUTPUT_FILE, "w") as f:
    f.write("\n".join(lines))
print(f"[inspect_steer_joint] Written to {OUTPUT_FILE}")
