"""
ros2_jetbot.py — Jetbot arena with ROS2 bridge.

Same arena and Jetbot as hello_robot.py, but wired through OmniGraph to ROS2 topics:
  /cmd_vel  (Twist)      — subscriber → Differential Controller → wheels
  /odom     (Odometry)   — publisher  ← articulation root pose
  /scan     (LaserScan)  — publisher  ← 2D lidar on chassis
  /clock    (Clock)      — publisher  ← simulation time
  /camera/image/compressed — publisher ← front-facing RGB camera (JPEG)

Run in Script Editor: Window > Script Editor > File > Open > /isaac-sim/ros2_jetbot.py
                      Then Ctrl+Enter.
"""

import asyncio
import numpy as np


async def run_ros2_jetbot():
    print("=== ros2_jetbot.py starting ===")

    # ── Fix FastDDS shared-memory transport ──────────────────────────────────
    # Isaac Sim container runs with ipc=private, so SHM segments are invisible
    # to other containers. Force UDP-only transport for ROS2 DDS data.
    import os
    _fastdds_xml = "/tmp/fastdds_no_shm.xml"
    if not os.path.exists(_fastdds_xml):
        with open(_fastdds_xml, "w") as _f:
            _f.write('<?xml version="1.0" encoding="UTF-8" ?>\n'
                     '<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">\n'
                     '  <transport_descriptors>\n'
                     '    <transport_descriptor>\n'
                     '      <transport_id>udp_only</transport_id>\n'
                     '      <type>UDPv4</type>\n'
                     '      <sendBufferSize>1048576</sendBufferSize>\n'
                     '      <receiveBufferSize>1048576</receiveBufferSize>\n'
                     '    </transport_descriptor>\n'
                     '  </transport_descriptors>\n'
                     '  <participant profile_name="participant_profile" is_default_profile="true">\n'
                     '    <rtps>\n'
                     '      <useBuiltinTransports>false</useBuiltinTransports>\n'
                     '      <userTransports>\n'
                     '        <transport_id>udp_only</transport_id>\n'
                     '      </userTransports>\n'
                     '    </rtps>\n'
                     '  </participant>\n'
                     '</profiles>\n')
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = _fastdds_xml
    print(f"[pre] FastDDS SHM disabled via {_fastdds_xml}")

    from isaacsim.core.api import World
    import isaacsim.core.experimental.utils.stage as stage_utils
    from isaacsim.storage.native import get_assets_root_path
    from isaacsim.core.experimental.prims import Articulation
    import omni.usd
    import omni.kit.app
    from pxr import UsdGeom, UsdPhysics, UsdShade, UsdLux, Sdf, Gf

    # ── Step 0: Clean up previous run ─────────────────────────────────────────
    print("[0/7] Cleaning up previous scene...")
    import omni.graph.core as og
    # Delete any existing OmniGraph first (must happen before USD prim removal)
    for gp in ["/World/ROS2Graph"]:
        try:
            g = og.get_graph_by_path(gp)
            if g is not None and g.is_valid():
                og.Controller.delete_graph(gp)
                print(f"  Deleted graph {gp}")
        except Exception:
            pass
    # Open a fresh stage
    await omni.usd.get_context().new_stage_async()
    await omni.kit.app.get_app().next_update_async()

    # ── Step 1: Enable required extensions ────────────────────────────────────
    print("[1/8] Enabling extensions...")
    ext_mgr = omni.kit.app.get_app().get_extension_manager()
    ext_mgr.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
    ext_mgr.set_extension_enabled_immediate("isaacsim.robot.wheeled_robots", True)
    ext_mgr.set_extension_enabled_immediate("isaacsim.sensors.physx", True)
    ext_mgr.set_extension_enabled_immediate("omni.replicator.core", True)
    await omni.kit.app.get_app().next_update_async()
    print("[1/8] Extensions enabled.")

    # ── Step 2: World + ground ─────────────────────────────────────────────────
    print("[2/8] Creating World...")
    world = World(stage_units_in_meters=1.0)
    await world.initialize_simulation_context_async()
    world.scene.add_default_ground_plane()
    stage = omni.usd.get_context().get_stage()

    # Remove default distant light (creates harsh shadows) and add even lighting
    default_light = stage.GetPrimAtPath("/World/defaultGroundPlane/SphereLight")
    if default_light.IsValid():
        stage.RemovePrim("/World/defaultGroundPlane/SphereLight")
    for p in ["/World/defaultDistantLight", "/World/DistantLight"]:
        if stage.GetPrimAtPath(p).IsValid():
            stage.RemovePrim(p)

    # Dome light — soft, even ambient illumination from all directions
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.GetIntensityAttr().Set(800.0)
    dome.GetColorAttr().Set(Gf.Vec3f(0.95, 0.95, 1.0))

    # Two soft fill lights from opposite sides for gentle definition
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
    print("[2/8] Even lighting: dome + 2 fill lights.")

    # ── Step 3: Arena ──────────────────────────────────────────────────────────
    print("[3/8] Building arena...")

    def add_box(path, pos, half_extents, opacity=1.0):
        cube = UsdGeom.Cube.Define(stage, path)
        cube.GetSizeAttr().Set(1.0)
        prim = cube.GetPrim()
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
        xf.AddScaleOp().Set(Gf.Vec3d(
            half_extents[0] * 2, half_extents[1] * 2, half_extents[2] * 2,
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

    A, TW, WH, OH = 4.0, 0.15, 0.15, 0.12
    add_box("/World/Arena/WallN", (0, A, WH), (A, TW, WH))
    add_box("/World/Arena/WallS", (0, -A, WH), (A, TW, WH))
    add_box("/World/Arena/WallE", (A, 0, WH), (TW, A, WH))
    add_box("/World/Arena/WallW", (-A, 0, WH), (TW, A, WH))
    add_box("/World/Arena/P1", (2.0, 1.5, OH), (0.25, 0.25, OH), opacity=0.35)
    add_box("/World/Arena/P2", (-2.0, -1.5, OH), (0.25, 0.25, OH), opacity=0.35)
    add_box("/World/Arena/P3", (1.0, -2.5, OH), (0.25, 0.25, OH), opacity=0.35)
    add_box("/World/Arena/P4", (-1.0, 2.5, OH), (0.25, 0.25, OH), opacity=0.35)
    add_box("/World/Arena/P5", (0.0, 0.5, OH), (0.5, 0.1, OH), opacity=0.35)

    # ── Step 3b: Wall posters (textured quads for YOLO to detect) ─────────────
    print("[3b/8] Generating wall posters for ML detection...")
    from PIL import Image, ImageDraw, ImageFont
    import math as _math

    _poster_dir = "/tmp/posters"
    os.makedirs(_poster_dir, exist_ok=True)

    def _gen_clock(path, size=256):
        """Generate a realistic clock face — COCO class 'clock'."""
        img = Image.new("RGB", (size, size), (240, 235, 220))
        d = ImageDraw.Draw(img)
        cx, cy, r = size // 2, size // 2, size // 2 - 12
        # Outer ring
        d.ellipse([cx-r, cy-r, cx+r, cy+r], outline=(40, 40, 40), width=4)
        d.ellipse([cx-r+6, cy-r+6, cx+r-6, cy+r-6], outline=(80, 80, 80), width=2)
        # Hour markers + numbers
        for h in range(12):
            angle = _math.radians(h * 30 - 90)
            x1 = cx + int((r - 18) * _math.cos(angle))
            y1 = cy + int((r - 18) * _math.sin(angle))
            x2 = cx + int((r - 6) * _math.cos(angle))
            y2 = cy + int((r - 6) * _math.sin(angle))
            d.line([(x1, y1), (x2, y2)], fill=(30, 30, 30), width=3)
            # Number
            nx = cx + int((r - 34) * _math.cos(angle))
            ny = cy + int((r - 34) * _math.sin(angle))
            num = str(h if h > 0 else 12)
            d.text((nx - 5, ny - 6), num, fill=(20, 20, 20))
        # Minute ticks
        for m in range(60):
            if m % 5 == 0:
                continue
            angle = _math.radians(m * 6 - 90)
            x1 = cx + int((r - 8) * _math.cos(angle))
            y1 = cy + int((r - 8) * _math.sin(angle))
            x2 = cx + int((r - 3) * _math.cos(angle))
            y2 = cy + int((r - 3) * _math.sin(angle))
            d.line([(x1, y1), (x2, y2)], fill=(100, 100, 100), width=1)
        # Hands — 10:10 position (classic display time)
        # Hour hand
        ha = _math.radians(10 * 30 + 10 * 0.5 - 90)
        d.line([(cx, cy), (cx + int(r * 0.5 * _math.cos(ha)), cy + int(r * 0.5 * _math.sin(ha)))],
               fill=(20, 20, 20), width=5)
        # Minute hand
        ma = _math.radians(10 * 6 - 90)
        d.line([(cx, cy), (cx + int(r * 0.75 * _math.cos(ma)), cy + int(r * 0.75 * _math.sin(ma)))],
               fill=(20, 20, 20), width=3)
        # Second hand
        sa = _math.radians(30 * 6 - 90)
        d.line([(cx, cy), (cx + int(r * 0.78 * _math.cos(sa)), cy + int(r * 0.78 * _math.sin(sa)))],
               fill=(180, 30, 30), width=1)
        # Center dot
        d.ellipse([cx-5, cy-5, cx+5, cy+5], fill=(30, 30, 30))
        img.save(path)

    def _gen_stop_sign(path, size=256):
        """Generate a stop sign — COCO class 'stop sign'."""
        img = Image.new("RGB", (size, size), (200, 200, 200))
        d = ImageDraw.Draw(img)
        cx, cy, r = size // 2, size // 2, size // 2 - 10
        # Octagon
        pts = []
        for i in range(8):
            angle = _math.radians(i * 45 - 22.5)
            pts.append((cx + int(r * _math.cos(angle)), cy + int(r * _math.sin(angle))))
        d.polygon(pts, fill=(200, 20, 20), outline=(160, 10, 10))
        # White inner border
        inner_pts = []
        for i in range(8):
            angle = _math.radians(i * 45 - 22.5)
            inner_pts.append((cx + int((r - 12) * _math.cos(angle)), cy + int((r - 12) * _math.sin(angle))))
        d.polygon(inner_pts, fill=None, outline=(255, 255, 255))
        # "STOP" text
        d.text((cx - 28, cy - 10), "STOP", fill=(255, 255, 255))
        img.save(path)

    def _gen_sports_ball(path, size=256):
        """Generate a sports ball pattern — COCO class 'sports ball'."""
        img = Image.new("RGB", (size, size), (100, 160, 60))
        d = ImageDraw.Draw(img)
        cx, cy, r = size // 2, size // 2, size // 2 - 10
        # Ball
        d.ellipse([cx-r, cy-r, cx+r, cy+r], fill=(230, 230, 230), outline=(60, 60, 60), width=2)
        # Soccer/football pentagon pattern
        for angle_off in range(0, 360, 72):
            a = _math.radians(angle_off)
            pcx = cx + int(r * 0.45 * _math.cos(a))
            pcy = cy + int(r * 0.45 * _math.sin(a))
            pts = []
            for i in range(5):
                pa = _math.radians(angle_off + i * 72)
                pts.append((pcx + int(20 * _math.cos(pa)), pcy + int(20 * _math.sin(pa))))
            d.polygon(pts, fill=(30, 30, 30))
        # Center pentagon
        pts = []
        for i in range(5):
            a = _math.radians(i * 72 - 90)
            pts.append((cx + int(22 * _math.cos(a)), cy + int(22 * _math.sin(a))))
        d.polygon(pts, fill=(30, 30, 30))
        img.save(path)

    def _gen_cat_face(path, size=256):
        """Generate a cartoon cat face — COCO class 'cat'."""
        img = Image.new("RGB", (size, size), (180, 200, 180))
        d = ImageDraw.Draw(img)
        cx, cy = size // 2, size // 2 + 15
        # Face
        d.ellipse([cx-70, cy-55, cx+70, cy+55], fill=(220, 180, 120))
        # Ears (triangles)
        d.polygon([(cx-55, cy-50), (cx-70, cy-100), (cx-20, cy-65)], fill=(220, 180, 120), outline=(80, 60, 40))
        d.polygon([(cx+55, cy-50), (cx+70, cy-100), (cx+20, cy-65)], fill=(220, 180, 120), outline=(80, 60, 40))
        # Inner ears
        d.polygon([(cx-50, cy-52), (cx-62, cy-88), (cx-28, cy-62)], fill=(220, 140, 140))
        d.polygon([(cx+50, cy-52), (cx+62, cy-88), (cx+28, cy-62)], fill=(220, 140, 140))
        # Eyes
        d.ellipse([cx-35, cy-25, cx-12, cy+5], fill=(80, 180, 80))
        d.ellipse([cx+12, cy-25, cx+35, cy+5], fill=(80, 180, 80))
        d.ellipse([cx-28, cy-15, cx-18, cy+0], fill=(10, 10, 10))
        d.ellipse([cx+18, cy-15, cx+28, cy+0], fill=(10, 10, 10))
        # Nose
        d.polygon([(cx, cy+8), (cx-7, cy+18), (cx+7, cy+18)], fill=(220, 120, 120))
        # Mouth
        d.arc([cx-15, cy+15, cx, cy+32], 0, 180, fill=(80, 60, 40), width=2)
        d.arc([cx, cy+15, cx+15, cy+32], 0, 180, fill=(80, 60, 40), width=2)
        # Whiskers
        for dy, dx in [(-2, 0), (5, 3), (12, 6)]:
            d.line([(cx-15, cy+dy+15), (cx-65, cy+dx+12)], fill=(80, 60, 40), width=1)
            d.line([(cx+15, cy+dy+15), (cx+65, cy+dx+12)], fill=(80, 60, 40), width=1)
        img.save(path)

    poster_imgs = {
        "clock": os.path.join(_poster_dir, "clock.png"),
        "stop": os.path.join(_poster_dir, "stop.png"),
        "ball": os.path.join(_poster_dir, "ball.png"),
        "cat": os.path.join(_poster_dir, "cat.png"),
    }
    _gen_clock(poster_imgs["clock"])
    _gen_stop_sign(poster_imgs["stop"])
    _gen_sports_ball(poster_imgs["ball"])
    _gen_cat_face(poster_imgs["cat"])

    # Create textured poster quads on inner wall faces
    def add_poster(name, tex_path, center, normal, up, width, height):
        """Create a textured quad (poster) on a wall."""
        path = f"/World/Arena/Poster_{name}"
        mesh = UsdGeom.Mesh.Define(stage, path)

        # Compute right vector from normal × up
        n = np.array(normal, dtype=float)
        u = np.array(up, dtype=float)
        r = np.cross(n, u)
        r = r / np.linalg.norm(r)
        hw, hh = width / 2, height / 2
        c = np.array(center, dtype=float)

        # Quad vertices — offset slightly from wall (along normal)
        offset = c + n * 0.001
        v0 = offset - r * hw - u * hh
        v1 = offset + r * hw - u * hh
        v2 = offset + r * hw + u * hh
        v3 = offset - r * hw + u * hh

        mesh.GetPointsAttr().Set([Gf.Vec3f(*v0), Gf.Vec3f(*v1), Gf.Vec3f(*v2), Gf.Vec3f(*v3)])
        mesh.GetFaceVertexCountsAttr().Set([4])
        mesh.GetFaceVertexIndicesAttr().Set([0, 1, 2, 3])

        # UV coordinates
        texcoords = UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
        )
        texcoords.Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])

        # Material with texture
        mat_path = path + "/Mat"
        mat = UsdShade.Material.Define(stage, mat_path)
        shader = UsdShade.Shader.Define(stage, mat_path + "/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)

        # Texture reader
        tex_reader = UsdShade.Shader.Define(stage, mat_path + "/DiffuseTex")
        tex_reader.CreateIdAttr("UsdUVTexture")
        tex_reader.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(tex_path)
        tex_reader.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
        tex_reader.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
        tex_reader.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

        # ST reader for UV coords
        st_reader = UsdShade.Shader.Define(stage, mat_path + "/STReader")
        st_reader.CreateIdAttr("UsdPrimvarReader_float2")
        st_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        st_reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

        tex_reader.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
            st_reader.ConnectableAPI(), "result"
        )
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
            tex_reader.ConnectableAPI(), "rgb"
        )
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)

    PH = 0.18   # poster height
    PW = 0.18   # poster width
    PZ = 0.14   # poster center Z (above ground)
    WALL_INSET = A - TW  # inner wall face position

    #                   name     texture                   center                          normal        up          w    h
    add_poster("clock", poster_imgs["clock"], (0, WALL_INSET, PZ),    (0, -1, 0),  (0, 0, 1), PW, PH)
    add_poster("stop",  poster_imgs["stop"],  (0, -WALL_INSET, PZ),   (0, 1, 0),   (0, 0, 1), PW, PH)
    add_poster("ball",  poster_imgs["ball"],  (WALL_INSET, 0, PZ),    (-1, 0, 0),  (0, 0, 1), PW, PH)
    add_poster("cat",   poster_imgs["cat"],   (-WALL_INSET, 0, PZ),   (1, 0, 0),   (0, 0, 1), PW, PH)

    # Real photo poster — ossi.png on two walls (deployed to /tmp/ossi.png)
    _ossi_path = "/tmp/ossi.png"
    if os.path.exists(_ossi_path):
        # Taller poster to match image aspect ratio (444x655 ≈ 2:3)
        OPW, OPH = 0.16, 0.24
        add_poster("ossi_N", _ossi_path, (2.0, WALL_INSET, PZ + 0.03),  (0, -1, 0), (0, 0, 1), OPW, OPH)
        add_poster("ossi_S", _ossi_path, (-2.0, -WALL_INSET, PZ + 0.03), (0, 1, 0),  (0, 0, 1), OPW, OPH)
        print("[3b/8] + 2 ossi.png posters on N and S walls")
    else:
        print("[3b/8] ossi.png not found at /tmp/ossi.png — skipped")

    print("[3b/8] 4 wall posters: clock, stop sign, ball, cat")

    print("[3/8] Arena ready.")

    # ── Step 4: Jetbot ─────────────────────────────────────────────────────────
    print("[4/8] Adding Jetbot...")
    assets_root = get_assets_root_path()
    if assets_root is None:
        print("ERROR: assets_root is None")
        return

    stage_utils.add_reference_to_stage(
        usd_path=assets_root + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd",
        path="/World/Jetbot",
    )

    # ── Step 5: Add 2D PhysX Lidar to Jetbot chassis ─────────────────────────
    print("[5/8] Adding 2D PhysX Lidar sensor...")
    lidar_prim_path = "/World/Jetbot/chassis/Lidar"
    result, lidar = omni.kit.commands.execute(
        "RangeSensorCreateLidar",
        path="/Lidar",
        parent="/World/Jetbot/chassis",
        min_range=0.2,
        max_range=10.0,
        draw_points=False,
        draw_lines=False,
        horizontal_fov=360.0,
        vertical_fov=10.0,
        horizontal_resolution=1.0,
        vertical_resolution=10.0,
        rotation_rate=20.0,
        high_lod=False,
        yaw_offset=0.0,
        enable_semantics=False,
    )
    await omni.kit.app.get_app().next_update_async()
    lidar_prim_path = str(lidar.GetPath())
    # Position lidar 10cm above chassis
    lidar.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3d(0.0, 0.0, 0.1))
    print(f"[5/8] PhysX Lidar added at {lidar_prim_path}")

    await world.reset_async()
    print("[5/8] World reset done.")

    # ── Step 5b: Orbit camera (same as hello_robot.py) ────────────────────────
    print("[5b/8] Creating orbit camera...")
    artic = Articulation("/World/Jetbot")

    cam = UsdGeom.Camera.Define(stage, "/World/FollowCam")
    cam_prim = cam.GetPrim()
    cam_xf = UsdGeom.Xformable(cam_prim)
    cam_xf.ClearXformOpOrder()
    cam_transform_op = cam_xf.AddTransformOp()
    cam.GetFocalLengthAttr().Set(24.0)
    CAM_RADIUS = 2.0
    CAM_HEIGHT = 2.0
    CAM_ORBIT_SPEED = 0.3

    def look_at_matrix(eye, target):
        fwd = target - eye
        fwd = fwd / np.linalg.norm(fwd)
        world_up = np.array([0.0, 0.0, 1.0])
        right = np.cross(fwd, world_up)
        right = right / np.linalg.norm(right)
        up = np.cross(right, fwd)
        return Gf.Matrix4d(
            right[0], right[1], right[2], 0,
            up[0],    up[1],    up[2],    0,
            -fwd[0],  -fwd[1],  -fwd[2],  0,
            eye[0],   eye[1],   eye[2],   1,
        )

    import omni.kit.viewport.utility as vp_utils
    viewport_api = vp_utils.get_active_viewport()
    viewport_api.camera_path = "/World/FollowCam"

    # Trail line
    TRAIL_HEIGHT = 0.02
    TRAIL_MAX = 2000
    TRAIL_EVERY = 6
    trail_curve = UsdGeom.BasisCurves.Define(stage, "/World/Trail")
    trail_curve.GetTypeAttr().Set("linear")
    trail_curve.GetWidthsAttr().Set([0.04])
    trail_curve.SetWidthsInterpolation("constant")
    trail_curve.GetDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.45, 0.0)])
    trail_points = []

    _cam_state = {"t": 0.0, "ticks": 0}

    def _orbit_camera_step(dt: float):
        _cam_state["t"] += dt
        _cam_state["ticks"] += 1
        try:
            poses = artic.get_local_poses()
            pos = poses[0].numpy()[0]
            rx, ry = float(pos[0]), float(pos[1])
        except Exception:
            return

        # Trail
        if _cam_state["ticks"] % TRAIL_EVERY == 0:
            trail_points.append(Gf.Vec3f(rx, ry, TRAIL_HEIGHT))
            if len(trail_points) > TRAIL_MAX:
                del trail_points[:len(trail_points) - TRAIL_MAX]
            if len(trail_points) >= 2:
                trail_curve.GetPointsAttr().Set(trail_points)
                trail_curve.GetCurveVertexCountsAttr().Set([len(trail_points)])

        # Orbit camera
        orbit_angle = _cam_state["t"] * CAM_ORBIT_SPEED
        eye = np.array([
            rx + CAM_RADIUS * np.cos(orbit_angle),
            ry + CAM_RADIUS * np.sin(orbit_angle),
            CAM_HEIGHT,
        ])
        cam_transform_op.Set(look_at_matrix(eye, np.array([rx, ry, 0.15])))

    print("[5b/8] Orbit camera active.")

    # ── Step 5c: Front-facing RGB camera + ROS2 publisher ─────────────────────
    print("[5c/8] Setting up front camera + ROS2 image publisher...")
    import omni.replicator.core as rep

    # Create camera prim on Jetbot chassis — forward-facing, wide FOV
    front_cam = UsdGeom.Camera.Define(stage, "/World/Jetbot/chassis/FrontCamera")
    front_cam.GetFocalLengthAttr().Set(18.0)
    front_cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.1, 20.0))
    front_cam.GetHorizontalApertureAttr().Set(20.955)
    fc_xf = UsdGeom.Xformable(front_cam.GetPrim())
    fc_xf.ClearXformOpOrder()
    fc_xf.AddTranslateOp().Set(Gf.Vec3d(0.05, 0.0, 0.08))
    # USD camera looks down -Z with +Y up. We need forward=+X with up=+Z.
    # RotateXYZ(90,0,-90): first X+90 tilts up to +Z, then Z-90 swings forward to +X
    fc_xf.AddRotateXYZOp().Set(Gf.Vec3d(90, 0, -90))

    # Create RenderProduct — off-screen 480×320 render target
    _rp = rep.create.render_product(
        "/World/Jetbot/chassis/FrontCamera", (480, 320)
    )

    # Attach RGB annotator
    _rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
    _rgb_annot.attach([_rp])

    # Wait a few frames for the render product to initialize
    for _ in range(5):
        await omni.kit.app.get_app().next_update_async()

    # Initialize rclpy publisher for CompressedImage
    # The ROS2 bridge extension may already have initialized rclpy, so handle both cases
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import CompressedImage as CompressedImageMsg
    from builtin_interfaces.msg import Time as TimeMsg
    try:
        rclpy.init()
        print("[5c/8] rclpy initialized (new context)")
    except RuntimeError:
        print("[5c/8] rclpy already initialized (reusing context)")

    _cam_ros_node = rclpy.create_node("jetbot_camera")
    _cam_pub = _cam_ros_node.create_publisher(
        CompressedImageMsg,
        "/camera/image/compressed",
        QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        ),
    )
    _cam_frame_count = {"n": 0, "published": 0}
    CAM_PUBLISH_EVERY = 6  # every 6th physics tick ≈ 10 FPS at 60 Hz

    def _camera_step(dt: float):
        _cam_frame_count["n"] += 1
        if _cam_frame_count["n"] % CAM_PUBLISH_EVERY != 0:
            return
        try:
            data = _rgb_annot.get_data()
            if data is None or not hasattr(data, "shape"):
                return
            # data is RGBA uint8 [H, W, 4] — drop alpha
            rgb = np.ascontiguousarray(data[:, :, :3])

            # JPEG compress using Python's built-in approach
            from io import BytesIO
            from PIL import Image
            buf = BytesIO()
            Image.fromarray(rgb).save(buf, format="JPEG", quality=75)
            jpeg_bytes = buf.getvalue()

            msg = CompressedImageMsg()
            msg.format = "jpeg"
            msg.data = jpeg_bytes
            _cam_pub.publish(msg)
            _cam_frame_count["published"] += 1
            if _cam_frame_count["published"] == 1:
                print(f"[camera] First frame published: {rgb.shape} → {len(jpeg_bytes)} bytes JPEG")
        except Exception as exc:
            if _cam_frame_count["published"] == 0:
                print(f"[camera] Error: {exc}")

    print("[5c/8] Front camera ready: /camera/image/compressed (~10 FPS, 480×320 JPEG)")

    # ── Step 6: Build OmniGraph for ROS2 wiring ───────────────────────────────
    print("[6/8] Building OmniGraph...")
    import omni.graph.core as og
    import usdrt

    keys = og.Controller.Keys
    (graph, nodes, _, _) = og.Controller.edit(
        {"graph_path": "/World/ROS2Graph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick",       "omni.graph.action.OnPlaybackTick"),
                ("ROS2Context",          "isaacsim.ros2.bridge.ROS2Context"),
                # /cmd_vel → wheels (BreakVector3 decomposes double3 → scalar)
                ("SubscribeTwist",       "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("BreakLinVel",          "omni.graph.nodes.BreakVector3"),
                ("BreakAngVel",          "omni.graph.nodes.BreakVector3"),
                ("DiffController",       "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("ArticController",      "isaacsim.core.nodes.IsaacArticulationController"),
                # /odom publisher
                ("ComputeOdom",          "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishOdom",          "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                # /clock publisher
                ("ReadSimTime",          "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock",         "isaacsim.ros2.bridge.ROS2PublishClock"),
                # /scan publisher
                ("PublishLaserScan",     "isaacsim.ros2.bridge.ROS2PublishLaserScan"),
                ("IsaacReadLidar",       "isaacsim.sensors.physx.IsaacReadLidarBeams"),
            ],
            keys.SET_VALUES: [
                # Differential controller: Jetbot wheel params
                ("DiffController.inputs:wheelDistance", 0.1125),
                ("DiffController.inputs:wheelRadius", 0.0325),
                # Articulation controller target
                ("ArticController.inputs:jointNames", ["left_wheel_joint", "right_wheel_joint"]),
                # Twist subscriber topic
                ("SubscribeTwist.inputs:topicName", "/cmd_vel"),
                # Odom
                ("PublishOdom.inputs:topicName", "/odom"),
                ("PublishOdom.inputs:chassisFrameId", "base_link"),
                ("PublishOdom.inputs:odomFrameId", "odom"),
                # Clock
                ("ReadSimTime.inputs:resetOnStop", False),
                ("PublishClock.inputs:topicName", "/clock"),
                # Lidar
                ("PublishLaserScan.inputs:topicName", "/scan"),
                ("PublishLaserScan.inputs:frameId", "lidar"),
                # Target prims (target-type attrs need usdrt.Sdf.Path list)
                ("ComputeOdom.inputs:chassisPrim", [usdrt.Sdf.Path("/World/Jetbot")]),
                ("ArticController.inputs:targetPrim", [usdrt.Sdf.Path("/World/Jetbot")]),
                ("IsaacReadLidar.inputs:lidarPrim", [usdrt.Sdf.Path(lidar_prim_path)]),
            ],
            keys.CONNECT: [
                # Tick drives nodes
                ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ComputeOdom.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "IsaacReadLidar.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticController.inputs:execIn"),
                # Context
                ("ROS2Context.outputs:context", "SubscribeTwist.inputs:context"),
                ("ROS2Context.outputs:context", "PublishOdom.inputs:context"),
                ("ROS2Context.outputs:context", "PublishClock.inputs:context"),
                ("ROS2Context.outputs:context", "PublishLaserScan.inputs:context"),
                # cmd_vel → BreakVector3 → DiffController → ArticController
                ("SubscribeTwist.outputs:execOut", "DiffController.inputs:execIn"),
                ("SubscribeTwist.outputs:linearVelocity", "BreakLinVel.inputs:tuple"),
                ("SubscribeTwist.outputs:angularVelocity", "BreakAngVel.inputs:tuple"),
                ("BreakLinVel.outputs:x", "DiffController.inputs:linearVelocity"),
                ("BreakAngVel.outputs:z", "DiffController.inputs:angularVelocity"),
                ("DiffController.outputs:velocityCommand", "ArticController.inputs:velocityCommand"),
                # Odom compute → publish
                ("ComputeOdom.outputs:execOut", "PublishOdom.inputs:execIn"),
                ("ComputeOdom.outputs:position", "PublishOdom.inputs:position"),
                ("ComputeOdom.outputs:orientation", "PublishOdom.inputs:orientation"),
                ("ComputeOdom.outputs:linearVelocity", "PublishOdom.inputs:linearVelocity"),
                ("ComputeOdom.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),
                # Clock + timestamps for all publishers
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdom.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishLaserScan.inputs:timeStamp"),
                # Lidar → LaserScan
                ("IsaacReadLidar.outputs:execOut", "PublishLaserScan.inputs:execIn"),
                ("IsaacReadLidar.outputs:azimuthRange", "PublishLaserScan.inputs:azimuthRange"),
                ("IsaacReadLidar.outputs:depthRange", "PublishLaserScan.inputs:depthRange"),
                ("IsaacReadLidar.outputs:horizontalFov", "PublishLaserScan.inputs:horizontalFov"),
                ("IsaacReadLidar.outputs:horizontalResolution", "PublishLaserScan.inputs:horizontalResolution"),
                ("IsaacReadLidar.outputs:intensitiesData", "PublishLaserScan.inputs:intensitiesData"),
                ("IsaacReadLidar.outputs:linearDepthData", "PublishLaserScan.inputs:linearDepthData"),
                ("IsaacReadLidar.outputs:numCols", "PublishLaserScan.inputs:numCols"),
                ("IsaacReadLidar.outputs:numRows", "PublishLaserScan.inputs:numRows"),
                ("IsaacReadLidar.outputs:rotationRate", "PublishLaserScan.inputs:rotationRate"),
            ],
        },
    )
    print("[6/8] OmniGraph wired: /cmd_vel, /odom, /scan, /clock")

    # ── Step 7: Start simulation ───────────────────────────────────────────────
    print("[7/8] Starting simulation...")
    world.add_physics_callback("orbit_camera", _orbit_camera_step)
    world.add_physics_callback("front_camera", _camera_step)
    world.play()
    print("[7/8] Simulation running. ROS2 topics active.")
    print("")
    print("  From ROS2 sidecar (bash ~/ros2_sidecar.sh to start):")
    print("    ros2 topic list")
    print("    ros2 topic echo /odom")
    print("    ros2 topic echo /scan")
    print("    ros2 topic hz /camera/image/compressed")
    print("    ros2 run teleop_twist_keyboard teleop_twist_keyboard")
    print("    python3 /ros2_ws/obstacle_avoidance_node.py")

    # ── Debug: wait a few frames then dump OG diagnostics ──────────────────────
    for _ in range(10):
        await omni.kit.app.get_app().next_update_async()

    import omni.timeline
    tl = omni.timeline.get_timeline_interface()
    print(f"[DEBUG] timeline playing={tl.is_playing()} stopped={tl.is_stopped()} time={tl.get_current_time():.3f}")

    # Read attribute values using two-argument form: (attr_name, node_prim_path)
    def ogval(attr_name, node_path):
        try:
            a = og.Controller.attribute(attr_name, node_path)
            return og.Controller.get(a)
        except Exception as e:
            return f"ERR:{e}"

    GP = "/World/ROS2Graph"
    sim_time = ogval("outputs:simulationTime", f"{GP}/ReadSimTime")
    ctx_handle = ogval("outputs:context", f"{GP}/ROS2Context")
    tick_frame = ogval("outputs:frame", f"{GP}/OnPlaybackTick")
    tick_delta = ogval("outputs:deltaSeconds", f"{GP}/OnPlaybackTick")
    tick_time = ogval("outputs:time", f"{GP}/OnPlaybackTick")
    print(f"[DEBUG] simTime={sim_time} ctx={ctx_handle} tickFrame={tick_frame} tickDelta={tick_delta} tickTime={tick_time}")

    odom_pos = ogval("outputs:position", f"{GP}/ComputeOdom")
    odom_ori = ogval("outputs:orientation", f"{GP}/ComputeOdom")
    odom_linvel = ogval("outputs:linearVelocity", f"{GP}/ComputeOdom")
    print(f"[DEBUG] odom: pos={odom_pos} ori={odom_ori} linvel={odom_linvel}")

    lidar_depth = ogval("outputs:linearDepthData", f"{GP}/IsaacReadLidar")
    lidar_ncols = ogval("outputs:numCols", f"{GP}/IsaacReadLidar")
    lidar_hfov = ogval("outputs:horizontalFov", f"{GP}/IsaacReadLidar")
    depth_summary = "None"
    if isinstance(lidar_depth, np.ndarray):
        nz = np.count_nonzero(lidar_depth)
        depth_summary = f"len={len(lidar_depth)} nonzero={nz} min={lidar_depth.min():.2f} max={lidar_depth.max():.2f}"
    print(f"[DEBUG] lidar: ncols={lidar_ncols} hfov={lidar_hfov} depth=[{depth_summary}]")

    diff_velcmd = ogval("outputs:velocityCommand", f"{GP}/DiffController")
    diff_linvel = ogval("inputs:linearVelocity", f"{GP}/DiffController")
    diff_angvel = ogval("inputs:angularVelocity", f"{GP}/DiffController")
    diff_wdist = ogval("inputs:wheelDistance", f"{GP}/DiffController")
    diff_wrad = ogval("inputs:wheelRadius", f"{GP}/DiffController")
    artic_velcmd = ogval("inputs:velocityCommand", f"{GP}/ArticController")
    artic_joints = ogval("inputs:jointNames", f"{GP}/ArticController")
    print(f"[DEBUG] diff: linvel={diff_linvel} angvel={diff_angvel} wDist={diff_wdist} wRad={diff_wrad} velCmd={diff_velcmd}")
    print(f"[DEBUG] artic: velCmd={artic_velcmd} joints={artic_joints}")

    # Graph evaluation state
    g = og.get_graph_by_path("/World/ROS2Graph")
    if g and g.is_valid():
        print(f"[DEBUG] graph: valid evaluator={g.get_evaluator_name()} nodes={len(g.get_nodes())}")
    else:
        print("[DEBUG] graph NOT FOUND!")

    # Prims
    print(f"[DEBUG] prims: Jetbot={stage.GetPrimAtPath('/World/Jetbot').IsValid()}"
          f" Lidar={stage.GetPrimAtPath('/World/Jetbot/chassis/Lidar').IsValid()}"
          f" Arena={stage.GetPrimAtPath('/World/Arena').IsValid()}")

    # Wait more frames and check again to see if values change
    for _ in range(30):
        await omni.kit.app.get_app().next_update_async()
    sim_time2 = ogval("outputs:simulationTime", f"{GP}/ReadSimTime")
    tick_frame2 = ogval("outputs:frame", f"{GP}/OnPlaybackTick")
    odom_pos2 = ogval("outputs:position", f"{GP}/ComputeOdom")
    lidar_depth2 = ogval("outputs:linearDepthData", f"{GP}/IsaacReadLidar")
    depth2_summary = "None"
    if isinstance(lidar_depth2, np.ndarray):
        nz2 = np.count_nonzero(lidar_depth2)
        depth2_summary = f"len={len(lidar_depth2)} nonzero={nz2}"
    print(f"[DEBUG] +30frames: simTime={sim_time2} tickFrame={tick_frame2} (was {tick_frame}) odomPos={odom_pos2} lidarDepth=[{depth2_summary}]")
    print(f"[DEBUG] timeline now={tl.get_current_time():.3f}")


asyncio.ensure_future(run_ros2_jetbot())
