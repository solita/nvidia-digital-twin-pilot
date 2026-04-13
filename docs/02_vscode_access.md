# Guide 02 — Opening a VS Code Workspace with Read/Write Access to Brev

This guide covers connecting VS Code on your local machine to the Brev remote via SSH and fixing the file-system permission issue that blocks access to Isaac Sim data directories.

---

## Prerequisites

- Brev instance is **Running** (see [Guide 01](./01_brev_setup.md))
- VS Code with the **Remote - SSH** extension installed (`ms-vscode-remote.remote-ssh`)
- SSH access to the Brev instance (port 22 exposed)

---

## Step 1 — Get SSH Connection Details from Brev

In the Brev instance dashboard:
1. Click the **SSH** button (or find it under instance details).
2. Note the connection string — it will look like:

```
ssh ubuntu@<PUBLIC_IP>
```

Brev provisions instances with the `ubuntu` user (UID 1000).

---

## Step 2 — Connect VS Code via Remote - SSH

1. Open VS Code on your local machine.
2. Press `Ctrl+Shift+P` → type **Remote-SSH: Connect to Host** → Enter.
3. Enter the host to connect to:
   - If you added the `~/.ssh/config` entry in [Guide 01 — AWS SSH workaround](./01_brev_setup.md#aws-ssh-workaround), just type the host name (e.g. `isaac-sim-pilot`).
   - Otherwise enter the full connection string: `ubuntu@<PUBLIC_IP>`.
4. Select **Linux** as the platform when prompted.
5. VS Code will install the VS Code Server on the remote machine and open a remote window.

---

## Step 3 — Open the Isaac Sim Data Directory as a Workspace

Once connected:
1. In the remote VS Code window, go to **File → Open Folder**.
2. Navigate to:

```
/home/ubuntu/docker/isaac-sim/data/
```

3. Click **OK**.

You now have VS Code pointed at the directory that is volume-mounted into the Isaac Sim container as `/isaac-sim/.local/share/ov/data/`.

---

## Step 4 — Fix the UID 1234 Permission Issue

The Isaac Sim container runs as **UID 1234**, not the `ubuntu` user (UID 1000). Any directories or files created by the container will be owned by UID 1234 and will have permissions `drwxr-x---` (750) — which gives no access to `ubuntu`.

**Symptom:** Your working directory appears empty in VS Code even though Isaac Sim has created files inside it, or you get "Permission denied" when trying to save files.

**Fix:** Grant the `ubuntu` user (and VS Code) access by adding `o+rwx` to the affected directory:

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

## Step 5 — Verify Read/Write Access

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

---

**Next:** [Guide 03 — Connecting VS Code with Isaac Sim and Running the Smoke Test](./03_vscode_isaacsim_smoke_test.md)
