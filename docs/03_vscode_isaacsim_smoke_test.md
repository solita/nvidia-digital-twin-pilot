# Guide 03 — Connecting VS Code with Isaac Sim and Running the Smoke Test

This guide covers installing the Isaac Sim VS Code Edition extension, configuring it to communicate with the running Isaac Sim instance, enabling the extension inside Isaac Sim, and running the included smoke test to verify everything works end-to-end.

---

## Prerequisites

- Brev instance running with Isaac Sim container active (see [Guide 01](./01_brev_setup.md))
- VS Code connected to the Brev remote via SSH (see [Guide 02](./02_vscode_access.md))

---

## Step 1 — Install the Isaac Sim VS Code Edition Extension

In the **remote** VS Code window (connected to Brev):

1. Open the **Extensions** panel (`Ctrl+Shift+X`).
2. Search for **Isaac Sim VS Code Edition**.
3. Install `nvidia.isaacsim-vscode-edition` (v0.3.0 or later).

> Installing in the remote window ensures the extension runs on the Brev machine, not your local machine.

---

## Step 2 — Configure the Extension to Use the Brev Public IP

By default the extension tries to connect to `127.0.0.1:8226`. On a remote setup this does not route to the Isaac Sim process — you will get a **"No active editor"** or connection refused error.

Fix: point the extension at the Brev machine's public IP.

Run this in the **remote** VS Code terminal (or any SSH session into the Brev instance). It fetches the public IP automatically and writes the settings file:

```bash
PUBLIC_IP=$(curl -s ifconfig.me) && \
mkdir -p ~/.vscode-server/data/User && \
cat > ~/.vscode-server/data/User/settings.json << EOF
{
    "isaacsim-vscode-edition.remoteApplication": {
        "extensionIP": "$PUBLIC_IP",
        "extensionPort": 8226
    }
}
EOF
```

If `~/.vscode-server/data/User/settings.json` already contains other settings, do not overwrite it — merge manually instead (`cat` with `>` will replace the entire file).

After saving, reload VS Code: `Ctrl+Shift+P` → **Developer: Reload Window**.

---

## Step 3 — Enable the VS Code Extension Inside Isaac Sim

Port `8226` is not open until you explicitly enable the extension inside the running Isaac Sim GUI.

In the Isaac Sim WebRTC Client:
1. Go to **Window** → **Extensions**.
2. Search for **VS Code**.
3. Enable **VS CODE INTEGRATION**.

This starts a TCP server inside Isaac Sim that listens on port `8226`. VS Code will send Python scripts over this socket and receive their output.

> You must repeat this step each time Isaac Sim is restarted unless you set the extension to auto-load.

---

## Step 4 — Run the Smoke Test

> **Before running:** Make sure a USD scene is open in the Isaac Sim WebRTC client. The smoke test needs an active stage — if no scene is loaded it will print `[FAIL] No stage found` and exit. Open any `.usd` file in Isaac Sim first.

The smoke test is in `smoke_test/vscode_smoke_test.py` in the repo. If you have the multi-root workspace open (see [Guide 02](./02_vscode_access.md)), it is already visible under the `nvidia-digital-twin-pilot (repo)` tree — no copying needed.

**To run:**
1. Open `smoke_test/vscode_smoke_test.py` from the repo tree in the remote VS Code editor.
2. Press `Ctrl+Shift+P` → **Isaac Sim: Run Remotely**.  
   Alternatively, use the play button in the editor title bar and select **Run Remotely**.

**Expected output** (visible in `Ctrl+Shift+U` → Output → "Isaac Sim"):

```
=== Isaac Sim VS Code Smoke Test ===
[PASS] Stage is open
      Root layer : /isaac-sim/.local/share/ov/data/<your_scene>.usd
[PASS] Selected prims : ['/<some_prim_path>']
[PASS] Wrote test prim  : /World/VSCodeConnectionTest
=== End of Smoke Test ===
```

### What the smoke test verifies

| Test | What it checks |
|---|---|
| Stage open | A USD stage is active in Isaac Sim |
| Root layer path | Prints the path of the currently open `.usd` file |
| Selected prims | Prints any currently selected prim paths (not a failure if empty) |
| Write test | Creates `/World/VSCodeConnectionTest` (Xform prim) — confirms write access |

### Cleanup

After verifying, remove the test prim by running this remotely:

```python
import omni.kit.commands
omni.kit.commands.execute("DeletePrims", paths=["/World/VSCodeConnectionTest"])
```

---

## How "Run Remotely" Works

The VS Code extension sends the contents of the current Python file over a **TCP socket** to `extensionIP:extensionPort`. Isaac Sim's `omni.isaac.vscode` extension is listening on that port and executes the code inside the live Isaac Sim process. The output (stdout/stderr) is returned over the same socket and displayed in the VS Code **Output** panel.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| "No active editor" on Run Remotely | Extension IP still set to `127.0.0.1` | Update `settings.json` with the Brev public IP |
| Connection refused on port 8226 | `omni.isaac.vscode` not enabled | Enable it under Window → Extensions in Isaac Sim |
| Output panel shows nothing | Wrong output channel selected | `Ctrl+Shift+U` → dropdown → select "Isaac Sim" |
| `[FAIL] No stage found` | No scene is open in Isaac Sim | Open any `.usd` file in Isaac Sim before running |

---

**Next:** [Guide 04 — Running the Concrete Spray Simulation](./04_concrete_spray.md)
