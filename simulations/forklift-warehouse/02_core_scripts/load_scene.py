import omni.usd

SCENE_PATH = (
    "/isaac-sim/.local/share/ov/data/nvidia-digital-twin-pilot"
    "/simulations/forklift-warehouse/01_scenes/scene_assembly.usd"
)

print(f"[load_scene] Opening scene: {SCENE_PATH}")
omni.usd.get_context().open_stage(SCENE_PATH)
print("[load_scene] Stage open requested")
