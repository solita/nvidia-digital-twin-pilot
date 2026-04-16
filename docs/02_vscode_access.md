# Guide 02 — Opening a VS Code Workspace with Read/Write Access to Brev

This guide covers opening both the repo and the Isaac Sim data directory in a single VS Code window on Brev, and fixing the file-system permission issue that blocks access to container-created files.

---

## Prerequisites

- Brev instance is **Running** and SSH access is confirmed (see [Guide 01](./01_brev_setup.md))
- SSH config is set up on your local machine (Guide 01, Step 3, Option B)
- Remote - SSH extension installed in your local VS Code (`ms-vscode-remote.remote-ssh`)

---

## Step 1 — Clone the Repo and Open the Workspace

The repo is cloned **directly into the Isaac Sim data volume**. This means scene USD files are accessible to Isaac Sim without any copy-paste step — `git pull` is all you need to update them.

#### 1a. Clone the repo into the Isaac Sim data directory

In an SSH terminal on the Brev instance:

```bash
sudo mkdir -p /home/ubuntu/docker/isaac-sim/data
sudo chmod o+rwx /home/ubuntu/docker/isaac-sim/data

git clone <repo-url> /home/ubuntu/docker/isaac-sim/data/nvidia-digital-twin-pilot

sudo chown -R 1234:1234 /home/ubuntu/docker/isaac-sim/data/nvidia-digital-twin-pilot
sudo chmod -R o+rwx /home/ubuntu/docker/isaac-sim/data/nvidia-digital-twin-pilot
```

After the `chown`, the repo directory is owned by UID 1234 — not the `ubuntu` user. Git refuses to operate in directories it doesn't own, so VS Code's Source Control panel will show a "dubious ownership" error. Fix it once:

```bash
git config --global --add safe.directory /home/ubuntu/docker/isaac-sim/data/nvidia-digital-twin-pilot
```

#### 1b. Open the workspace in VS Code

1. Press `Cmd+Shift+P` (macOS) / `Ctrl+Shift+P` → **Remote-SSH: Connect to Host** → `<instance-name>`.
2. In the remote window: **File → Open Workspace from File**.
3. Navigate to `/home/ubuntu/docker/isaac-sim/data/nvidia-digital-twin-pilot/brev.code-workspace` and click **Open**.

VS Code opens the repo root in the Explorer. The integrated terminal runs on the Brev VM.

> Inside the Isaac Sim container the repo is visible at `/isaac-sim/data/nvidia-digital-twin-pilot/`.

---

## Step 2 — Fix the UID 1234 Permission Issue

The Isaac Sim container runs as **UID 1234**, not the `ubuntu` user (UID 1000). Any directories or files created by the container will be owned by UID 1234 and will have permissions `drwxr-x---` (750) — which gives no access to `ubuntu`.

**Symptom:** The `Isaac Sim Data` folder appears empty in VS Code even though Isaac Sim has created files inside it, or you get "Permission denied" when trying to save files.

**Fix:** Grant the `ubuntu` user access by adding `o+rwx` to the affected directory:

```bash
sudo chmod o+rwx /home/ubuntu/docker/isaac-sim/data/<your_working_directory>
```

For example, for a directory called `workshop2`:

```bash
sudo chmod o+rwx /home/ubuntu/docker/isaac-sim/data/workshop2
```

> Replace `workshop2` with the subdirectory Isaac Sim created. Run `ls -la /home/ubuntu/docker/isaac-sim/data/` to see the ownership of each directory.

**Why this works:** `o+rwx` adds read, write, and execute for "others" (anyone not in the owning group), which includes the `ubuntu` user. VS Code, which runs as `ubuntu`, can then browse and modify the directory normally.

---

## Step 3 — Verify Read/Write Access

In the VS Code terminal (connected to the remote), run:

```bash
echo "write test" > /home/ubuntu/docker/isaac-sim/data/<your_working_directory>/write_test.txt
cat /home/ubuntu/docker/isaac-sim/data/<your_working_directory>/write_test.txt
rm /home/ubuntu/docker/isaac-sim/data/<your_working_directory>/write_test.txt
```

All three commands should succeed without errors.

---

## Quick Reference

| Item | Value |
|---|---|
| Remote user | `ubuntu` (UID 1000) |
| Isaac Sim container user | UID `1234` |
| Volume-mounted data path (host) | `/home/ubuntu/docker/isaac-sim/data/` |
| Volume-mounted data path (container) | `/isaac-sim/.local/share/ov/data/` |
| Permission fix command | `sudo chmod o+rwx <directory>` |
| Repo + data path (host) | `/home/ubuntu/docker/isaac-sim/data/nvidia-digital-twin-pilot/` |
| Repo path (container) | `/isaac-sim/data/nvidia-digital-twin-pilot/` |
| Workspace file | `/home/ubuntu/docker/isaac-sim/data/nvidia-digital-twin-pilot/brev.code-workspace` |

---

**Next:** [Guide 03 — Connecting VS Code with Isaac Sim and Running the Smoke Test](./03_vscode_isaacsim_smoke_test.md)

