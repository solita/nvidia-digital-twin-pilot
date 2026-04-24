"""Part 3 — Headless standalone script: Jetbot drives a square path."""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import numpy as np
from isaacsim.core.api import World
from isaacsim.storage.native import get_assets_root_path
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.prims import Articulation

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

assets_root = get_assets_root_path()
if assets_root is None:
    print("ERROR: Cannot reach asset server")
    simulation_app.close()
    exit(1)

print(f"Assets root: {assets_root}")

stage_utils.add_reference_to_stage(
    usd_path=assets_root + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd",
    path="/World/Jetbot",
)

world.reset()

artic = Articulation("/World/Jetbot")
wheel_idx = artic.get_dof_indices(["left_wheel_joint", "right_wheel_joint"]).numpy()
print(f"Wheel joint indices: {wheel_idx}")

total_steps = 500
print(f"Running {total_steps} simulation steps...")

for step in range(total_steps):
    world.step(render=False)
    t = step * world.get_physics_dt()

    if t < 2.0:
        vl, vr = 10.0, 10.0      # forward
    elif t < 2.5:
        vl, vr = 10.0, -10.0     # turn right
    elif t < 4.5:
        vl, vr = 10.0, 10.0      # forward
    elif t < 5.0:
        vl, vr = 10.0, -10.0     # turn right
    else:
        vl, vr = 10.0, 10.0      # forward

    artic.set_dof_velocity_targets(
        np.array([[vl, vr]]), dof_indices=wheel_idx
    )

    if step % 60 == 0:
        pos = artic.get_local_poses()[0].numpy()[0]
        print(f"t={t:.2f}s  pos=({pos[0]:.3f}, {pos[1]:.3f})  cmd=({vl:.0f},{vr:.0f})")

simulation_app.close()
print("Done.")
