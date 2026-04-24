# NVIDIA Robot Simulation Stack — Installation Guide
## GPU from Cloud + Local Setup

---

## 0. NVIDIA Omniverse — What It Is & Licensing

### What is Omniverse (2025–2026)?

Omniverse has **evolved significantly**. It is no longer primarily a desktop app — it is now a **collection of open libraries and microservices** for building physical AI applications (robotics simulation, digital twins, synthetic data). The old "Omniverse Launcher" desktop app is a legacy product.

The core components are:

| Library | Purpose | Access |
|---|---|---|
| **Omniverse Kit SDK** | App/microservice framework for building on Omniverse | Free, docs |
| **OpenUSD** | 3D scene description interchange format | Open source |
| **OpenUSD Exchange SDK** | USD-based data exchange tooling | Free |
| **ovRTX** (Early Access) | GPU rendering + sensor simulation library | GitHub |
| **ovPHYSICS / PhysX** | GPU physics engine | Open source, GitHub |
| **Newton Physics** | New open-source physics on NVIDIA Warp + USD | GitHub |
| **Isaac Sim** | Full robot simulation app built on Omniverse Kit | Free via NGC (EULA) |

### Do You Need a License?

**Short answer: No paid license required for development.**

| Scenario | License needed? | Cost |
|---|---|---|
| Isaac Sim (Docker / pip) | Accept NVIDIA Isaac Sim EULA | **Free** |
| Omniverse libraries (ovRTX, PhysX, etc.) | Open source / developer agreement | **Free** |
| NGC container registry | Free NVIDIA account | **Free** |
| Omniverse Enterprise (legacy Nucleus server for team collaboration) | Commercial license | Paid — contact NVIDIA sales |

- Isaac Sim is free for individual developers and research use. The EULA is accepted automatically with `-e "ACCEPT_EULA=Y"` in the Docker run command.
- The **Omniverse Enterprise** tier (with a Nucleus collaboration server for teams) requires a commercial license, but this is **not needed** for robot simulation development.
- All core libraries (PhysX, OpenUSD, Newton) are **open source** on GitHub.

### NGC Account Setup (Required to Pull Containers)

```bash
# 1. Create a free account at https://ngc.nvidia.com
# 2. Generate an API key: ngc.nvidia.com → Account → Setup → Generate API Key

# 3. Log in to the NVIDIA container registry
docker login nvcr.io
#   Username: $oauthtoken
#   Password: <paste your NGC API key>
```

### Omniverse Libraries — Install as Python Packages

Many Omniverse/Kit libraries are available as pip packages (no Docker needed):

```bash
pip install usd-core          # OpenUSD Python bindings
pip install warp-lang          # NVIDIA Warp (GPU-accelerated Python)

# Omniverse Kit SDK (for building custom apps)
# Full packages available at: https://docs.omniverse.nvidia.com
```

### Legacy Omniverse Launcher

The old Omniverse Launcher (used for USD Composer, Nucleus, etc.) is still accessible but **not needed** for robot simulation workflows. Isaac Sim is now best installed via Docker or pip directly. If you need it:

> Download from: https://www.nvidia.com/en-us/omniverse/download/

---

## 1. Cloud GPU Instance Setup — NVIDIA Brev

[NVIDIA Brev](https://brev.nvidia.com) is the recommended platform for this stack. Every instance comes with **NVIDIA drivers, CUDA, Docker, and NVIDIA Container Toolkit pre-installed** — nothing to configure manually.

**Chosen GPU:** NVIDIA A4000 — 16 GB GDDR6, Ampere (compute 8.6) — cheapest suitable GPU for Isaac Sim starter projects at **$0.18/hr**.

### 1a. Install the Brev CLI (on your Mac)

```bash
# Install via Homebrew
brew install brevdev/homebrew-brev/brev

# Verify
brev --version
```

### 1b. Log In

```bash
brev login
# Opens browser for authentication.
# Credentials and SSH keys are stored in ~/.brev/
```

### 1c. Browse Available GPUs

```bash
# Search all GPU types, sorted by price
brev search gpu --sort price

# Filter by GPU name
brev search gpu --gpu-name A4000
brev search gpu --gpu-name A10

# Filter by minimum VRAM (Isaac Sim needs 16 GB+)
brev search gpu --min-vram 16 --sort price
```

### 1d. Create the Instance

```bash
# A4000 — cheapest Ampere option ($0.18/hr), good for starter projects
brev create isaac-sim --type hyperstack_A4000

# A10G — more VRAM (22 GB), better for Isaac Lab RL ($1.21/hr)
brev create isaac-sim --type g5.xlarge

# Fallback chain: try A4000 first, fall back to A10G if unavailable
brev create isaac-sim --type hyperstack_A4000,g5.xlarge

# Takes 2-5 minutes to provision. Check status:
brev ls
```

> **Note:** You need credits in your Brev account before creating instances.
> Add them at: https://brev.nvidia.com/settings

### 1e. Connect to the Instance

```bash
# Shell into the instance
brev shell isaac-sim

# Or open directly in VS Code (installs Remote SSH extension automatically)
brev open isaac-sim
```

### 1f. Verify GPU on the Instance

```bash
# Verified on hyperstack_A4000 instance:
#   GPU:    NVIDIA RTX A4000, 15352 MiB VRAM
#   Driver: 570.195.03
#   CUDA:   12.8
#   Docker: 29.1.4
nvidia-smi

# Docker + NVIDIA Container Toolkit are pre-installed — verify:
docker run --rm --gpus all nvidia/cuda:12.1.0-base-ubuntu22.04 nvidia-smi
```

### 1g. Port Forwarding for browser streaming

The Docker Compose streaming setup (Section 3e) requires:

1. **Port 8210 (TCP)** — the web viewer UI — forwarded to your Mac via SSH tunnel
2. **Port 49100 (TCP)** — WebRTC signaling — opened directly in the instance firewall so the browser can reach it
3. **Port 47998 (UDP)** — WebRTC media stream — opened directly in the instance firewall so the browser can reach it

Ports 49100 and 47998 must be opened in the instance firewall (not SSH-tunnelled) because WebRTC media uses UDP, which SSH tunnels cannot carry.

```bash
# On the instance — open WebRTC ports in UFW (one-time):
brev shell isaac-sim
sudo ufw allow 49100/tcp
sudo ufw allow 47998/udp

# On your Mac — SSH tunnel only for the web viewer UI:
# Replace <instance-ip> with your instance's public IP
ssh -i ~/.ssh/id_ed25519 \
  -L 8210:localhost:8210 \
  shadeform@<instance-ip> -N -f

# Verify the web viewer is reachable:
curl -s http://localhost:8210/ | grep -o '<title>[^<]*</title>'
# Returns: <title>web-viewer</title>
```

Then open `http://localhost:8210` in Chrome (see Section 3e for the full setup).

### 1h. Stop / Resume to Save Costs

```bash
# Stop when not in use (pauses compute billing, disk is kept)
brev stop isaac-sim

# Resume later
brev start isaac-sim

# Delete completely when done
brev delete isaac-sim
```

### Suggested Brev Instances for Isaac Sim

| Type | GPU | VRAM | $/hr | Stoppable | Recommended for |
|---|---|---|---|---|---|
| `hyperstack_A4000` | A4000 | 16 GB | **$0.18** | ❌ | Starter projects, headless Python scripts |
| `g2-standard-4:nvidia-l4:1` | L4 | 24 GB | $0.85 | ✅ | GUI streaming, mid-range sim |
| `g5.xlarge` | A10G | 22 GB | $1.21 | ✅ | Isaac Lab RL, sensor simulation |
| `g5.2xlarge` | A10G | 22 GB | $1.45 | ✅ | A10G with more vCPUs (8) |

> **Stoppable** = `brev stop` pauses billing while keeping your disk. Non-stoppable instances must be deleted when not in use.
>
> Run `brev search gpu --min-vram 16 --sort price` anytime to see current live pricing.

---

## 2. System Prerequisites

> **Brev instances skip this section entirely** — NVIDIA drivers, CUDA, Docker, and NVIDIA Container Toolkit are all pre-installed. Jump straight to [Section 3](#3-install-isaac-sim-docker--recommended-for-cloud).
>
> These steps are only needed if you are setting up a **raw Ubuntu VM** (e.g. manually on AWS/GCP/Azure without Brev).

```bash
# Update system
sudo apt-get update && sudo apt-get upgrade -y

# Install essential dependencies
sudo apt-get install -y \
  curl wget git build-essential \
  libvulkan1 vulkan-tools \
  libgl1-mesa-glx libglu1-mesa \
  xorg xauth x11-xserver-utils \
  python3-pip python3.10-venv

# Verify NVIDIA driver (535+ recommended)
nvidia-smi
# If missing, install driver:
sudo apt-get install -y nvidia-driver-535
sudo reboot
```

### Install NVIDIA Container Toolkit (for Docker)

```bash
# Add NVIDIA repo
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Verify
docker run --rm --gpus all nvidia/cuda:12.1.0-base-ubuntu22.04 nvidia-smi
```

---

## 3. Install Isaac Sim (Docker — Recommended for Cloud)

Docker is the recommended approach for cloud deployments (headless or streaming).

> **Latest versions (March 2026):**
> - Stable: `nvcr.io/nvidia/isaac-sim:5.1.0`
> - Dev/Preview: `nvcr.io/nvidia/isaac-sim:6.0.0-dev2`
> - Check https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim for the latest tag.

### 3a. Pull Isaac Sim Container from NGC

```bash
# Log in to NVIDIA NGC (requires free account at ngc.nvidia.com — see Section 0)
docker login nvcr.io
# Username: $oauthtoken
# Password: <your NGC API key>

# Pull stable Isaac Sim
docker pull nvcr.io/nvidia/isaac-sim:5.1.0
```

### 3b. Create cache directories and set permissions

Run once on the instance to create persistent cache volumes. These survive container restarts and avoid re-downloading assets.

```bash
# Step 1: Connect to the instance
brev shell isaac-sim

# Step 2: Create cache directories
mkdir -p ~/docker/isaac-sim/{cache/main/ov,cache/main/warp,cache/computecache,config,data/documents,data/Kit,logs,pkg}

# Step 3: Set ownership to uid 1234 — Isaac Sim's container user
sudo chown -R 1234:1234 ~/docker/isaac-sim

# Step 4: Verify
ls -la ~/docker/isaac-sim/
```

> **Note:** The `chown` to uid 1234 is required. Isaac Sim inside the container runs as that UID, and without correct ownership it cannot write to the cache directories.

> **Stale cache causes stream failures.** If Isaac Sim reports ready but the stream never connects, wipe the cache and recreate:
> ```bash
> sudo rm -rf ~/docker/isaac-sim
> mkdir -p ~/docker/isaac-sim/{cache/main/ov,cache/main/warp,cache/computecache,config,data/documents,data/Kit,logs,pkg}
> sudo chown -R 1234:1234 ~/docker/isaac-sim
> ```

### 3c. Run Isaac Sim — Interactive shell (for scripting and debugging)

Opens a bash shell inside the container. Working directory inside is `/isaac-sim/`.

```bash
# Step 1: Connect to the instance
brev shell isaac-sim

# Step 2: Start an interactive container session
docker run --name isaac-sim --entrypoint bash -it --rm \
  --gpus all \
  -e "ACCEPT_EULA=Y" \
  -e "PRIVACY_CONSENT=Y" \
  -v ~/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw \
  -v ~/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw \
  -v ~/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw \
  -v ~/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw \
  -u 1234:1234 \
  nvcr.io/nvidia/isaac-sim:5.1.0

# You are now inside the container at /isaac-sim/
# Run a Python simulation script:
./python.sh /path/to/your_script.py
```

> **License note:** By using `-e "ACCEPT_EULA=Y"` you accept the
> [NVIDIA Isaac Sim Additional Software and Materials License](https://www.nvidia.com/en-us/agreements/enterprise-software/isaac-sim-additional-software-and-materials-license/) — free for development use.

> **Flags explained:**
> - `--gpus all` — enables GPU access. Do NOT also add `--runtime=nvidia`; they are mutually redundant with modern Docker.
> - `--entrypoint bash` — overrides the default entrypoint so you get a shell instead of auto-launching the app.
> - `-u 1234:1234` — runs as the Isaac Sim container user so cache writes succeed.
> - No `--network=host` needed here — only required when streaming (Section 3e).

### 3d. Run a Python script non-interactively (one-liner, for CI/pipelines)

```bash
# Mount your scripts directory read-only and run directly — no shell needed
docker run --rm \
  --gpus all \
  -e "ACCEPT_EULA=Y" \
  -e "PRIVACY_CONSENT=Y" \
  -v ~/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw \
  -v ~/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw \
  -v ~/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw \
  -v ~/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw \
  -v /path/to/my_scripts:/scripts:ro \
  -u 1234:1234 \
  nvcr.io/nvidia/isaac-sim:5.1.0 \
  /isaac-sim/python.sh /scripts/your_script.py
```

### 3e. WebRTC Streaming — Browser Access via Docker Compose

The recommended approach for cloud browser streaming is **Docker Compose**, which runs Isaac Sim and a purpose-built web viewer side-by-side. The web viewer is pre-configured with the correct signaling addresses and serves the streaming UI on port **8210**.

**Ports:**
| Port | Protocol | Purpose |
|---|---|---|
| `8210` | TCP | Web viewer — streaming UI served to the browser |
| `49100` | TCP | WebRTC signaling WebSocket (Isaac Sim) |
| `47998` | UDP | WebRTC media stream (Isaac Sim, stays on instance) |

**Important:**
- **First run:** Vulkan shader compilation takes ~10 minutes (GPU at ~35%). Do not stop — this is normal. Subsequent runs use the warm cache and take ~35 seconds.
- **Stale cache causes stream failures.** If the app is ready but the stream never starts, wipe the cache (see Section 3b) and restart.
- **Use Chrome or Edge only.** Firefox does not support WebRTC for this viewer.

---

**Step 1 — Clone `tools/docker/` on the instance (one-time)**

```bash
brev shell isaac-sim

mkdir -p ~/isaacsim-docker && cd ~/isaacsim-docker
git init
git remote add origin https://github.com/isaac-sim/IsaacSim.git
git fetch --depth 1 origin develop
git checkout FETCH_HEAD -- tools/docker
```

> This fetches only the `tools/docker/` directory (~1 MB) without cloning the full repository.

**Step 2 — Open WebRTC ports in the instance firewall (one-time)**

```bash
# While still on the instance:
sudo ufw allow 49100/tcp   # WebRTC signaling
sudo ufw allow 47998/udp   # WebRTC media
```

> These must be opened directly — not SSH-tunnelled — because WebRTC media uses UDP, which SSH tunnels cannot carry. The web viewer JavaScript connects to these ports from your Mac browser via the public IP.

**Step 3 — Launch Isaac Sim + web viewer**

```bash
# Replace <instance-ip> with your instance's public IP (brev ls):
cd ~/isaacsim-docker

ISAACSIM_HOST=<instance-ip> \
ISAAC_SIM_IMAGE=nvcr.io/nvidia/isaac-sim:5.1.0 \
docker compose -p isim -f tools/docker/docker-compose.yml up --build -d
```

> `ISAACSIM_HOST` must be the **public IP**, not `127.0.0.1`. The web viewer bakes this address into the JavaScript at build time. The browser uses it to reach Isaac Sim's signaling (TCP 49100) and media (UDP 47998) ports directly.

> `--build` compiles the web viewer image on first run (~1 minute). On subsequent runs it is skipped.

**Step 4 — Wait for Isaac Sim to be ready**

```bash
# Still on the instance — poll until ready:
watch -n 5 'curl -s http://127.0.0.1:8011/v1/streaming/ready'
# Wait for: {"statusMessage":"Status: Ready for connection"}
# First run with empty cache: ~10 minutes. Subsequent runs: ~35 seconds.
```

**Step 5 — Forward the web viewer port to your Mac**

On your Mac, open a terminal:

```bash
# SSH tunnel for the web viewer UI only (8210):
ssh -i ~/.ssh/id_ed25519 \
  -L 8210:localhost:8210 \
  shadeform@<instance-ip> -N -f

# Verify:
curl -s http://localhost:8210/ | grep -o '<title>[^<]*</title>'
# Returns: <title>web-viewer</title>
```

**Step 6 — Open the web viewer in Chrome**

Navigate to: **`http://localhost:8210`**

The stream connects automatically once Isaac Sim is ready. If it shows "Waiting for stream to begin", confirm Step 4 returned `Ready for connection` and refresh the page.

**Stopping and restarting**

```bash
# Stop (cache is preserved on disk):
ssh -i ~/.ssh/id_ed25519 shadeform@<instance-ip> \
  "cd ~/isaacsim-docker && docker compose -p isim -f tools/docker/docker-compose.yml down"

# Restart (fast — warm shader cache, ~35 seconds to ready):
ssh -i ~/.ssh/id_ed25519 shadeform@<instance-ip> \
  "cd ~/isaacsim-docker && ISAACSIM_HOST=<instance-ip> ISAAC_SIM_IMAGE=nvcr.io/nvidia/isaac-sim:5.1.0 docker compose -p isim -f tools/docker/docker-compose.yml up -d"
```

---

## 4. Install Isaac Lab (Robot Learning / RL)

Isaac Lab runs on top of Isaac Sim. Install it after Isaac Sim is working.

```bash
# Clone Isaac Lab
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab

# Install (using the Isaac Sim Python environment)
# Point to your Isaac Sim Python binary
export ISAACLAB_PATH=$(pwd)

# Run the install script
./isaaclab.sh --install

# Verify installation
./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py
```

---

## 5. Install ROS 2 Humble

```bash
# Add ROS 2 apt repo
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu \
  $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt-get update
sudo apt-get install -y ros-humble-desktop python3-colcon-common-extensions

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify
ros2 --version
```

---

## 6. Install Isaac ROS (Hardware-Accelerated ROS 2 Packages)

```bash
# Isaac ROS uses Docker — set up the workspace
mkdir -p ~/workspaces/isaac_ros-dev/src
cd ~/workspaces/isaac_ros-dev/src

# Clone a package (example: Isaac ROS Visual SLAM)
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git

# Launch the Isaac ROS Docker environment
cd ~/workspaces/isaac_ros-dev
docker pull nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_c6c6de01-2024-05-28-15-54-44

# Run (see Isaac ROS docs for full launch command per package)
```

---

## 7. Install cuRobo (GPU Motion Planning)

```bash
# Requires CUDA 11.8+ and PyTorch
pip install torch --index-url https://download.pytorch.org/whl/cu118

# Clone and install cuRobo
git clone https://github.com/NVlabs/curobo.git
cd curobo
pip install -e .[dev] --no-build-isolation

# Verify
python -c "import curobo; print('cuRobo OK')"
```

---

## 8. Verify Full Stack

```bash
# GPU check
nvidia-smi

# CUDA check
python3 -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"

# ROS 2 check
ros2 topic list

# Isaac Sim headless smoke test (inside container)
./python.sh -c "import omni; print('Isaac Sim OK')"
```

---

## 9. Recommended Stack by Use Case

| Goal | Primary Tools | Command Entry Point |
|---|---|---|
| General robot simulation | Isaac Sim + ROS 2 | Isaac Sim Docker + ROS bridge |
| RL / robot learning | Isaac Lab + Isaac Sim | `./isaaclab.sh -p <script>` |
| Robot perception pipeline | Isaac ROS + Isaac Sim | Isaac ROS Docker |
| Motion planning | cuRobo + Isaac Sim | Python API |
| Synthetic training data | Omniverse Replicator | Built into Isaac Sim |

---

## 10. Cloud Cost Tips

- **A4000 on Hyperstack** (`hyperstack_A4000`) is the cheapest Ampere option at $0.18/hr — but it is **not stoppable**, so delete (`brev delete isaac-sim`) when done.
- **A10G** (`g5.xlarge`) supports `brev stop` to pause billing while keeping disk — better for iterative work despite the higher hourly rate.
- Mount cache volumes so Isaac Sim asset downloads survive container restarts.
- Run `brev search gpu --min-vram 16 --sort price` anytime to check current pricing.

---

## References

- Isaac Sim: https://developer.nvidia.com/isaac-sim
- Isaac Lab: https://isaac-sim.github.io/IsaacLab
- NGC Catalog: https://catalog.ngc.nvidia.com
- Isaac ROS: https://github.com/NVIDIA-ISAAC-ROS
- cuRobo: https://curobo.org
- ROS 2 Humble: https://docs.ros.org/en/humble
