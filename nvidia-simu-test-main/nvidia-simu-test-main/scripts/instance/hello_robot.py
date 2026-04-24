"""
hello_robot.py — Jetbot playground with obstacle avoidance.

Walled arena with internal pillars. Jetbot has a forward-facing proximity sensor
(PhysX raycast). On obstacle detection, it turns a random direction and continues.

Script Editor: Window > Script Editor > File > Open > /isaac-sim/hello_robot.py
               Then Ctrl+Enter (or the Run button).
"""

import asyncio
import numpy as np
import carb


async def run_hello_robot():
    print("=== hello_robot.py starting ===")

    from isaacsim.core.api import World
    import isaacsim.core.experimental.utils.stage as stage_utils
    from isaacsim.storage.native import get_assets_root_path
    from isaacsim.core.experimental.prims import Articulation
    import omni.usd
    import omni.physx
    from pxr import UsdGeom, UsdPhysics, UsdShade, UsdLux, Sdf, Gf

    # ── World ──────────────────────────────────────────────────────────────────
    print("[1/5] Creating World...")
    world = World(stage_units_in_meters=1.0)
    await world.initialize_simulation_context_async()
    world.scene.add_default_ground_plane()
    stage = omni.usd.get_context().get_stage()

    # ── Even Lighting ────────────────────────────────────────────────────────────────
    # Remove default harsh lights and replace with dome + fill lights
    for p in ["/World/defaultGroundPlane/SphereLight", "/World/defaultDistantLight", "/World/DistantLight"]:
        if stage.GetPrimAtPath(p).IsValid():
            stage.RemovePrim(p)

    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.GetIntensityAttr().Set(800.0)
    dome.GetColorAttr().Set(Gf.Vec3f(0.95, 0.95, 1.0))

    for name, pos, intensity in [
        ("FillA", Gf.Vec3d(5, 5, 6), 400.0),
        ("FillB", Gf.Vec3d(-5, -5, 6), 400.0),
    ]:
        sl = UsdLux.SphereLight.Define(stage, f"/World/{name}")
        sl.GetIntensityAttr().Set(intensity)
        sl.GetRadiusAttr().Set(2.0)
        sl.GetColorAttr().Set(Gf.Vec3f(1.0, 0.98, 0.95))
        xf = UsdGeom.Xformable(sl.GetPrim())
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(pos)
    print("[1.5/5] Even lighting: dome + 2 fill lights.")

    # ── Arena ──────────────────────────────────────────────────────────────────
    print("[2/5] Building arena...")

    def add_box(path, pos, half_extents, opacity=1.0):
        """Create a static collision box. half_extents = (hx, hy, hz)."""
        cube = UsdGeom.Cube.Define(stage, path)
        cube.GetSizeAttr().Set(1.0)
        prim = cube.GetPrim()
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
        xf.AddScaleOp().Set(Gf.Vec3d(
            half_extents[0] * 2,
            half_extents[1] * 2,
            half_extents[2] * 2,
        ))
        UsdPhysics.CollisionAPI.Apply(prim)
        if opacity < 1.0:
            mat_path = path + "/Mat"
            mat = UsdShade.Material.Define(stage, mat_path)
            shader = UsdShade.Shader.Define(stage, mat_path + "/Shader")
            shader.CreateIdAttr("UsdPreviewSurface")
            shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.4, 0.6, 0.9))
            mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)

    A  = 4.0   # arena half-extent -> 8x8 m
    TW = 0.15  # wall half-thickness
    WH = 0.15  # wall half-height (low — rays at 0.08m still hit)
    OH = 0.12  # obstacle half-height

    # Perimeter walls
    add_box("/World/Arena/WallN", ( 0,  A, WH), (A,  TW, WH))
    add_box("/World/Arena/WallS", ( 0, -A, WH), (A,  TW, WH))
    add_box("/World/Arena/WallE", ( A,  0, WH), (TW, A,  WH))
    add_box("/World/Arena/WallW", (-A,  0, WH), (TW, A,  WH))

    # Internal obstacles (pillars + bar) — semi-transparent
    add_box("/World/Arena/P1", ( 2.0,  1.5, OH), (0.25, 0.25, OH), opacity=0.35)
    add_box("/World/Arena/P2", (-2.0, -1.5, OH), (0.25, 0.25, OH), opacity=0.35)
    add_box("/World/Arena/P3", ( 1.0, -2.5, OH), (0.25, 0.25, OH), opacity=0.35)
    add_box("/World/Arena/P4", (-1.0,  2.5, OH), (0.25, 0.25, OH), opacity=0.35)
    add_box("/World/Arena/P5", ( 0.0,  0.5, OH), (0.5,  0.1,  OH), opacity=0.35)
    print("[2/5] Arena ready: 4 walls + 5 obstacles.")

    # ── Jetbot ─────────────────────────────────────────────────────────────────
    print("[3/5] Adding Jetbot...")
    assets_root = get_assets_root_path()
    if assets_root is None:
        print("ERROR: assets_root is None -- check Nucleus")
        return
    print(f"[3/5] assets_root = {assets_root}")

    stage_utils.add_reference_to_stage(
        usd_path=assets_root + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd",
        path="/World/Jetbot",
    )
    await world.reset_async()
    print("[3/5] Reset done.")

    artic = Articulation("/World/Jetbot")
    print(f"[3/5] DOFs: {artic.dof_names}")
    wheel_idx = artic.get_dof_indices(["left_wheel_joint", "right_wheel_joint"]).numpy()
    print(f"[3/5] Wheel idx: {wheel_idx}")

    # ── Orbit camera (45° elevation, circles the robot) ─────────────────
    print("[3.5/5] Creating orbit camera...")
    cam = UsdGeom.Camera.Define(stage, "/World/FollowCam")
    cam_prim = cam.GetPrim()
    cam_xf = UsdGeom.Xformable(cam_prim)
    cam_xf.ClearXformOpOrder()
    cam_transform_op = cam_xf.AddTransformOp()         # full 4x4 matrix
    cam.GetFocalLengthAttr().Set(24.0)
    CAM_RADIUS   = 2.0    # horizontal orbit radius (m)
    CAM_HEIGHT   = 2.0    # height above robot (= radius for 45°)
    CAM_ORBIT_SPEED = 0.3 # radians per second

    def look_at_matrix(eye, target):
        """Build a USD-camera-compatible 4x4 look-at matrix.
        USD cameras look along local -Z, up = +Y."""
        fwd = target - eye
        fwd = fwd / np.linalg.norm(fwd)           # camera -Z direction
        world_up = np.array([0.0, 0.0, 1.0])
        right = np.cross(fwd, world_up)
        right = right / np.linalg.norm(right)     # camera +X
        up = np.cross(right, fwd)                  # camera +Y
        m = Gf.Matrix4d(
            right[0],  right[1],  right[2],  0,
            up[0],     up[1],     up[2],     0,
            -fwd[0],   -fwd[1],   -fwd[2],   0,
            eye[0],    eye[1],    eye[2],    1,
        )
        return m

    # Set as active viewport camera
    import omni.kit.viewport.utility as vp_utils
    viewport_api = vp_utils.get_active_viewport()
    viewport_api.camera_path = "/World/FollowCam"
    print("[3.5/5] Orbit camera active.")

    # ── 360° ray sweep + gap-based steering ─────────────────────────────────
    print("[4/5] Setting up forward proximity sweep...")
    pqi = omni.physx.get_physx_scene_query_interface()

    # Sweep parameters — forward 180° arc (-90° to +90°)
    RAY_OFFSET   = 0.18   # origin offset from robot centre (skip own body)
    RAY_HEIGHT   = 0.08   # ray z-height
    RAY_RANGE    = 0.35   # max sense distance
    SWEEP_STEP   = 10     # degrees between rays
    SWEEP_HALF   = 90     # half-arc in degrees (±90° from forward)
    # Rays from -90° to +90° in SWEEP_STEP increments
    ray_angles_deg = np.arange(-SWEEP_HALF, SWEEP_HALF + 1, SWEEP_STEP, dtype=np.float64)
    N_RAYS       = len(ray_angles_deg)  # 19 rays
    ray_offsets  = np.deg2rad(ray_angles_deg)
    CENTER_IDX   = N_RAYS // 2          # index of 0° (forward)

    # Steering parameters
    DRIVE_SPEED  = 8.0    # base wheel speed
    MAX_STEER    = 6.0    # max differential for turning
    STEER_LP     = 0.10   # low-pass filter coefficient (0=frozen, 1=instant)
    CLEAR_THRESH = 0.7    # fraction of RAY_RANGE to consider "clear"
    SLOW_THRESH  = 0.3    # fraction of RAY_RANGE to start slowing down
    FWD_DEADZONE = 2      # ±2 rays around centre = ±20° dead zone

    def get_pos_yaw():
        poses = artic.get_local_poses()
        pos = poses[0].numpy()[0]   # [x, y, z]
        q   = poses[1].numpy()[0]   # [w, x, y, z]
        w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        yaw = np.arctan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))
        return pos, yaw

    def sweep_fwd(pos, yaw):
        """Cast N_RAYS in the forward arc. Returns array of distances."""
        dists = np.full(N_RAYS, RAY_RANGE, dtype=np.float64)
        cx, cy = float(pos[0]), float(pos[1])
        for i in range(N_RAYS):
            angle = yaw + ray_offsets[i]
            dx, dy = float(np.cos(angle)), float(np.sin(angle))
            ox = cx + dx * RAY_OFFSET
            oy = cy + dy * RAY_OFFSET
            try:
                hit = pqi.raycast_closest(
                    carb.Float3(ox, oy, RAY_HEIGHT),
                    carb.Float3(dx, dy, 0.0),
                    RAY_RANGE,
                )
                if hit and hit.get("hit"):
                    dists[i] = float(hit["distance"])
            except Exception:
                pass
        return dists

    def find_best_gap(dists):
        """Find the widest contiguous run of clear rays.
        Returns (centre_index, width_in_rays). No wrap-around needed for a linear arc."""
        clear = dists > (RAY_RANGE * CLEAR_THRESH)
        best_start, best_len, cur_start, cur_len = 0, 0, 0, 0
        for i in range(N_RAYS):
            if clear[i]:
                if cur_len == 0:
                    cur_start = i
                cur_len += 1
                if cur_len > best_len:
                    best_len = cur_len
                    best_start = cur_start
            else:
                cur_len = 0
        centre = best_start + best_len / 2.0
        return centre, best_len

    # ── Trail line (thick colored line on the ground) ────────────────────────
    print("[4.5/5] Creating trail line...")
    TRAIL_HEIGHT = 0.02    # slightly above ground so it's visible
    TRAIL_MAX    = 2000    # max points to keep
    TRAIL_EVERY  = 6       # record a point every N ticks (~10 Hz at 60 fps)

    trail_curve = UsdGeom.BasisCurves.Define(stage, "/World/Trail")
    trail_curve.GetTypeAttr().Set("linear")
    trail_curve.GetWidthsAttr().Set([0.04])           # 4 cm wide line
    trail_curve.SetWidthsInterpolation("constant")
    # Orange color
    trail_curve.GetDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.45, 0.0)])
    trail_points = []   # list of Gf.Vec3f

    # ── Controller ─────────────────────────────────────────────────────────────
    state = {
        "t":         0.0,
        "ticks":     0,
        "steer_lp":  0.0,   # low-pass filtered steering command (-1..+1)
    }

    def on_step(dt: float):
        state["t"]     += dt
        state["ticks"] += 1
        t = state["t"]

        try:
            pos, yaw = get_pos_yaw()
        except Exception as e:
            print(f"  pose error: {e}")
            return

        # Update orbit camera
        orbit_angle = t * CAM_ORBIT_SPEED
        rx, ry = float(pos[0]), float(pos[1])

        # Record trail point
        if state["ticks"] % TRAIL_EVERY == 0:
            trail_points.append(Gf.Vec3f(rx, ry, TRAIL_HEIGHT))
            if len(trail_points) > TRAIL_MAX:
                del trail_points[:len(trail_points) - TRAIL_MAX]
            if len(trail_points) >= 2:
                trail_curve.GetPointsAttr().Set(trail_points)
                trail_curve.GetCurveVertexCountsAttr().Set([len(trail_points)])

        eye = np.array([
            rx + CAM_RADIUS * np.cos(orbit_angle),
            ry + CAM_RADIUS * np.sin(orbit_angle),
            CAM_HEIGHT,
        ])
        cam_transform_op.Set(look_at_matrix(eye, np.array([rx, ry, 0.15])))

        # Forward arc sweep
        dists = sweep_fwd(pos, yaw)
        front_dist = dists[CENTER_IDX]   # centre ray = forward

        # Find best gap direction
        gap_centre, gap_width = find_best_gap(dists)
        # Signed angle: CENTER_IDX = forward, lower = right, higher = left
        steer_angle = (gap_centre - CENTER_IDX) / (N_RAYS / 2.0)  # -1..+1

        # Proximity urgency: how close is the nearest obstacle in the forward arc?
        # 1.0 = obstacle touching, 0.0 = nothing within range
        min_front = float(np.min(dists))
        urgency = max(0.0, 1.0 - min_front / RAY_RANGE)

        # Blend steering intensity based on urgency:
        #   urgency ~0 (clear)  → gentle drift toward best gap (gain 0.15)
        #   urgency ~1 (close)  → hard steer toward best gap  (gain 2.0)
        gain = 0.15 + urgency * 1.85
        steer_cmd = max(-1.0, min(1.0, steer_angle * gain))

        # Adapt low-pass: respond faster when urgent
        lp = STEER_LP + urgency * 0.25

        # Low-pass filter
        state["steer_lp"] += lp * (steer_cmd - state["steer_lp"])
        s = state["steer_lp"]

        # Convert to differential drive
        # s=0 → straight, s>0 → turn left, s<0 → turn right
        vl = DRIVE_SPEED - s * MAX_STEER
        vr = DRIVE_SPEED + s * MAX_STEER

        # Slow down when obstacles are close ahead
        if front_dist < RAY_RANGE * SLOW_THRESH:
            speed_factor = max(0.3, front_dist / (RAY_RANGE * SLOW_THRESH))
            vl *= speed_factor
            vr *= speed_factor

        try:
            artic.set_dof_velocity_targets(
                np.array([[vl, vr]]),
                dof_indices=wheel_idx,
            )
        except Exception as e:
            print(f"  velocity error: {e}")
            return

        # Status every ~2 s
        if state["ticks"] % 120 == 0:
            yaw_deg = np.degrees(yaw)
            print(f"  t={t:.1f}s  pos=({pos[0]:.2f},{pos[1]:.2f})  yaw={yaw_deg:.0f}°  "
                  f"front={front_dist:.2f}m  gap={gap_width}×{SWEEP_STEP}°@{gap_centre:.0f}  "
                  f"steer={s:.2f}")

    print("[5/5] Starting simulation...")
    world.add_physics_callback("jetbot_drive", on_step)
    world.play()
    print("[5/5] Jetbot bouncing around arena. Press Stop (square) in toolbar to stop.")


asyncio.ensure_future(run_hello_robot())
