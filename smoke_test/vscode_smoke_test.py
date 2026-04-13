import omni.kit.commands
import omni.usd

PASS = "[PASS]"
FAIL = "[FAIL]"
TEST_PRIM_PATH = "/World/VSCodeConnectionTest"

print("\n=== Isaac Sim VS Code Smoke Test ===")

# ── 1. Stage check ────────────────────────────────────────────────────────────
ctx = omni.usd.get_context()
stage = ctx.get_stage()

if stage:
    print(f"{PASS} Stage is open")
    print(f"      Root layer : {stage.GetRootLayer().identifier}")
else:
    print(f"{FAIL} No stage found – open a scene in Isaac Sim first")
    raise SystemExit

# ── 2. Selection ──────────────────────────────────────────────────────────────
selection = ctx.get_selection().get_selected_prim_paths()
if selection:
    print(f"{PASS} Selected prims : {selection}")
else:
    print("[INFO] No prims selected (not a failure)")

# ── 3. Write test – create temp prim if absent ────────────────────────────────
if stage.GetPrimAtPath(TEST_PRIM_PATH).IsValid():
    print(f"[INFO] Test prim already exists at {TEST_PRIM_PATH}")
else:
    omni.kit.commands.execute(
        "CreatePrimWithDefaultXform",
        prim_type="Xform",
        prim_path=TEST_PRIM_PATH,
    )

if stage.GetPrimAtPath(TEST_PRIM_PATH).IsValid():
    print(f"{PASS} Wrote test prim  : {TEST_PRIM_PATH}")
else:
    print(f"{FAIL} Could not create test prim at {TEST_PRIM_PATH}")

print("=== End of Smoke Test ===\n")
