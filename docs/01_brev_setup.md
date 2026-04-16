# Guide 01 — Hosting Isaac Sim on Brev (AWS / Sweden)

This guide walks through provisioning an L40S GPU instance on NVIDIA Brev using the **AWS provider (Stockholm / eu-north-1)**, pulling the Isaac Sim Docker container, and launching it in headless WebRTC mode.

Official reference: [NVIDIA Brev Deployment](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_advanced_cloud_setup_brev.html)

---

## Prerequisites

| Requirement | Notes |
|---|---|
| NVIDIA Brev account | Sign up at [developer.nvidia.com/brev](https://developer.nvidia.com/brev) |
| Brev CLI | Installed on your **local machine** — see [Step 0](#step-0--install-the-brev-cli-locally) below |
| Isaac Sim WebRTC Streaming Client | Download from the [NVIDIA Isaac Sim releases page](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/manual_livestream_clients.html#isaac-sim-setup-livestream-webrtc) — install on your **local machine** |
| Your public IP | Needed to scope port exposure rules |

---

## Step 0 — Install the Brev CLI Locally

The Brev CLI lets you SSH into your instance directly from your local terminal — no need to go via the Jupyter Notebook interface in the browser.

### macOS

```bash
brew install brevdev/homebrew-brev/brev
brev --version
```

### Linux

```bash
sudo bash -c "$(curl -fsSL https://raw.githubusercontent.com/brevdev/brev-cli/main/bin/install-latest.sh)"
brev --version
```

### Windows (WSL)

```powershell
# Run in PowerShell as admin
wsl --install -d Ubuntu-22.04
```

Then inside Ubuntu (WSL):

```bash
sudo bash -c "$(curl -fsSL https://raw.githubusercontent.com/brevdev/brev-cli/main/bin/install-latest.sh)"
```

### Authenticate

```bash
brev login
```

This opens a browser window for OAuth. After logging in, the CLI downloads your SSH key to `~/.brev/brev.pem` and writes SSH config entries for all your instances to `~/.brev/ssh_config`. You connect by instance **name**, not IP — no manual key management needed.

---

## Step 1 — Create a Brev Instance

1. Go to [developer.nvidia.com/brev](https://developer.nvidia.com/brev) and sign in.
2. Click **Create New Instance**.
3. Under **Compute**, select **1× NVIDIA L40S GPU**.
4. Under **Cloud Provider**, choose **AWS** and select region **eu-north-1 (Stockholm)** for lowest latency from Europe.
5. Name the instance (e.g. `isaac-sim-pilot`) and click **Deploy**.
6. Wait for the VM status to change to **Built/Running** (typically 5–10 minutes).

---

## Step 2 — Expose Required Ports

Two ports must be open to your local IP for the WebRTC stream:

| Port | Purpose |
|---|---|
| `49100` | Isaac Sim WebRTC signalling |
| `47998` | Isaac Sim WebRTC media stream |

In the Brev instance dashboard:
1. Navigate to **Exposed Ports**.
2. Add port `49100` — scope it to **your IP only** (do not expose to `0.0.0.0/0`).
3. Add port `47998` — scope it to **your IP only**.

> **Security note:** Scoping these to your IP prevents the stream from being accessible by anyone on the internet.

If you also plan to use the VS Code remote extension (see [Guide 02](./02_vscode_access.md)), expose port **22 (SSH)** as well. However, this port is exposed by default so you don't have to do this manually — but make sure it's listed in the **Exposed Ports** section.

---

## Step 3 — Open a Remote Terminal

Once the instance status is **Running**, get the instance name and IP:

```bash
# Sync the latest instance list from the web console
brev refresh

# List instances — note the name and IP
brev ls
```

### Option A — Brev CLI (non-AWS providers only)

```bash
brev shell <instance-name>
```

> On **AWS**, all Brev CLI commands that route via the Brev backend (`brev shell`, `brev open ... code`) reliably hang or return `not_found`. Use Option B instead.

---

### Option B — Direct SSH (required for AWS, works everywhere)

This is the recommended path for AWS-hosted instances. It bypasses the Brev API entirely and connects directly over port 22.

#### 1. One-time SSH config setup

Open `~/.ssh/config` in a text editor and ensure it looks like this. **The order matters** — the `ControlPath` override must appear before the `Include` line, otherwise `.brev/ssh_config` sets it first and macOS hits a "path too long" error.

```
# ── Brev ControlPath fix ─────────────────────────────────────────────────────
# MUST be before the Include. SSH takes the first match per directive.
# %C expands to a short hash — avoids macOS's 104-char Unix socket limit.
Host <instance-name> <instance-name>-host
  ControlPath ~/.ssh/brev-ctl-%C

Include "/Users/<your-username>/.brev/ssh_config"

# ── Brev instance ─────────────────────────────────────────────────────────────
Host <instance-name>
    HostName <BREV_PUBLIC_IP>
    User ubuntu
    IdentityFile ~/.ssh/brev
```

Replace `<instance-name>` with the name from `brev ls`, `<your-username>` with your macOS username, and `<BREV_PUBLIC_IP>` with the instance IP.

> **Key location:** Brev writes your SSH private key to `~/.brev/brev.pem`. On some systems it is also linked to `~/.ssh/brev`. Check which exists:
> ```bash
> ls ~/.ssh/brev ~/.brev/brev.pem
> ```
> Use whichever path is present as the `IdentityFile` value.

> **Avoid duplicates:** Do not use `cat >>` to append entries repeatedly — it creates duplicate `Host` blocks that cause silent failures. Edit the file directly.

> **IP changes on restart:** AWS assigns a new public IP every time the instance is stopped and started. When that happens, update the `HostName` value in this block. Get the new IP from `~/.brev/ssh_config` after running `brev refresh`:
> ```bash
> brev refresh
> grep -A2 "yapa-l40s-gpu1" ~/.brev/ssh_config | grep Hostname
> ```
> Then update `~/.ssh/config` with the new IP — the VS Code extension `extensionIP` in `settings.json` (Guide 03) also needs updating.

#### 2. Verify the connection

```bash
ssh <instance-name>
```

Expected output:
```
Warning: Permanently added '...' (ED25519) to the list of known hosts.
ubuntu@brev-...~$
```

You are now inside the Brev VM. From this point, use `ssh <instance-name>` instead of `brev shell`.

All commands from this point run inside this remote shell (on the Brev VM).

---

### Option C — Open the Brev filesystem in VS Code (recommended for development)

With the SSH config in place (Option B above), you can open the remote filesystem directly in VS Code. The repo includes `brev.code-workspace` — a multi-root workspace that opens both the cloned repo and the Isaac Sim data directory in a single Remote-SSH window.

**Prerequisites:** Install the [Remote - SSH](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-ssh) extension in your local VS Code.

```
Extensions sidebar → search "Remote - SSH" → Install
Extension ID: ms-vscode-remote.remote-ssh
```

**Connect:**

1. Press `Cmd+Shift+P` (macOS) / `Ctrl+Shift+P` → **Remote-SSH: Connect to Host**.
2. Type `<instance-name>` and press Enter.
3. Select **Linux** when prompted for the platform.

You are now connected to the Brev VM in VS Code. The integrated terminal runs on the remote instance.

> For the full setup — cloning the repo into the Isaac Sim data volume and opening the workspace file — see [Guide 02 — VS Code Remote Access](./02_vscode_access.md).

---

## Step 4 — Get the Public IP

```bash
curl -s ifconfig.me
```

Note this IP — you will need it when starting Isaac Sim and when configuring the WebRTC client.

---

## Step 5 — Pull the Isaac Sim Container

```bash
docker pull nvcr.io/nvidia/isaac-sim:5.1.0
```

This is a large image (~20 GB). It will take several minutes on first pull.

---

## Step 6 — Create Volume Mount Directories

Isaac Sim uses persistent volume mounts for its cache, config, data, and logs. Run:

```bash
mkdir -p ~/docker/isaac-sim/cache/main/ov
mkdir -p ~/docker/isaac-sim/cache/main/warp
mkdir -p ~/docker/isaac-sim/cache/computecache
mkdir -p ~/docker/isaac-sim/config
mkdir -p ~/docker/isaac-sim/data/documents
mkdir -p ~/docker/isaac-sim/data/Kit
mkdir -p ~/docker/isaac-sim/logs
mkdir -p ~/docker/isaac-sim/pkg
sudo chown -R 1234:1234 ~/docker/isaac-sim
```

> The container runs as UID `1234`. The `chown` ensures the container can write to these directories without `Permission denied` errors.

---

## Step 7 — Start the Isaac Sim Container and Launch it in headless WebRTC mode

```bash
PUBLIC_IP=$(curl -s ifconfig.me) && \
docker run --name isaac-sim \
  --entrypoint bash \
  -it --gpus all \
  -e "ACCEPT_EULA=Y" \
  -e "PRIVACY_CONSENT=Y" \
  --rm --network=host \
  -v ~/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw \
  -v ~/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw \
  -v ~/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw \
  -v ~/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw \
  -u 1234:1234 \
  nvcr.io/nvidia/isaac-sim:5.1.0 \
  -lc "./runheadless.sh --/app/livestream/publicEndpointAddress=$PUBLIC_IP --/app/livestream/port=49100"
```

> By setting `-e "ACCEPT_EULA=Y"` you accept the [NVIDIA Omniverse License Agreement](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/common/NVIDIA_Omniverse_License_Agreement.html).  
> By setting `-e "PRIVACY_CONSENT=Y"` you opt-in to [data collection](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/common/data-collection.html). Omit this flag to opt out.  
> Add `--runtime=nvidia` if the container cannot detect the GPU.

You are now inside the container's bash shell.
Isaac Sim will take 2–5 minutes to fully load on first run (shader compilation). Subsequent starts are faster due to the cached volumes.

---

## Step 8 — Connect via WebRTC Client

### 8a — Download the Isaac Sim WebRTC Streaming Client

Go to the [Isaac Sim Download page](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html#isaac-sim-latest-release) and download **Isaac Sim WebRTC Streaming Client v1.1.5** for your platform:

| Platform | File type |
|---|---|
| macOS (Apple Silicon / arm64) | `.dmg` |
| macOS (Intel / x86_64) | `.dmg` |
| Linux (x86_64) | `.AppImage` |
| Windows | installer |

**macOS:**
1. Open the `.dmg` file.
2. Drag the **Isaac Sim WebRTC Streaming Client** app to the **Applications** folder.
3. Launch it from Applications.

> When streaming, use `Ctrl+C` / `Ctrl+V` to copy/paste within the streamed Isaac Sim app. To copy from your Mac host into the stream use `⌘C` then `Ctrl+V`.

**Linux (Ubuntu):**
```bash
chmod +x IsaacSimWebRTCStreamingClient-*.AppImage
./IsaacSimWebRTCStreamingClient-*.AppImage
```

> On Ubuntu 22.04 or later, `libfuse2` is required. Install it if the AppImage fails to launch:
> ```bash
> sudo apt install libfuse2
> ```

**Windows:**
1. Run the installer and follow the prompts.
2. If you cannot connect to the Isaac Sim instance, add both `kit.exe` and the Streaming Client app to the **Windows Firewall** allow list.

---

### 8b — Connect to Isaac Sim

1. Open the **Isaac Sim WebRTC Streaming Client** on your local machine.
2. Enter the **public IP** from Step 4 to the Server text box.
3. Click **Connect**.

Isaac Sim may take a few moments to appear. Look for this line in the Brev terminal to confirm Isaac Sim is fully loaded before connecting:

```
Isaac Sim Full Streaming App is loaded.
```

> If you see a blank screen after connecting, go to **View → Reload** in the client to re-establish the stream.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Container exits immediately | EULA flag missing | Ensure `-e "ACCEPT_EULA=Y"` is present |
| `ssh` connection timed out after restart | Instance got a new public IP | Run `brev refresh`, get new IP from `~/.brev/ssh_config`, update `HostName` in `~/.ssh/config` |
| GPU not detected | Missing runtime flag | Add `--runtime=nvidia` to the `docker run` command |
| WebRTC client cannot connect | Ports not exposed | Verify ports 49100 and 47998 are exposed to your IP in Brev |
| Stream connects but is black | Isaac Sim still loading | Wait up to 5 minutes and try reconnecting |
| `Permission denied` on volume writes | Wrong owner on host dirs | Re-run `sudo chown -R 1234:1234 ~/docker/isaac-sim` |

---

**Next:** [Guide 02 — Opening a VS Code Workspace with Read/Write Access](./02_vscode_access.md)
