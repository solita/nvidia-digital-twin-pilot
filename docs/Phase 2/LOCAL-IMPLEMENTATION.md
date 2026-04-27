# Local Machine — Implementation Guide

> **Scope**: Everything that runs on the local Windows/WSL2 machine.
> **Current state (2026-04-27)**: Phases 1–3 run entirely on Brev. The local machine is used for code editing (VS Code Remote-SSH) and browsing the WebRTC viewer / API docs.
> **Phase 4+**: Warehouse manager, Redis, and React dashboard run locally. Connected to Brev via Tailscale VPN.

> **Host OS**: Windows with WSL2. All Docker and ROS 2 work runs inside WSL2.
> `network_mode: host` does NOT work on Docker Desktop for Windows in the traditional sense — it maps to the WSL2 VM's network, not the Windows host. This plan uses WSL2 as the primary environment.

---

## 0. Prerequisites & Environment Setup

### 0.1: WSL2 Setup

All local services run inside WSL2 (Ubuntu 22.04). This avoids Windows/Docker networking issues.

```powershell
# PowerShell (admin)
wsl --install -d Ubuntu-22.04
# Reboot if prompted
wsl --set-version Ubuntu-22.04 2
```

After install, open the Ubuntu terminal and configure:

```bash
sudo apt update && sudo apt upgrade -y
```

### 0.2: Docker Desktop with WSL2 Backend

Install Docker Desktop for Windows. In Settings:

1. **General** → "Use the WSL 2 based engine" ✅
2. **Resources → WSL Integration** → Enable for Ubuntu-22.04 ✅

Verify from inside WSL2:

```bash
docker --version   # Should show Docker version
docker run hello-world
```

**GOTCHA**: Do NOT install Docker Engine inside WSL2 separately if you're using Docker Desktop. They conflict. Use Docker Desktop's WSL2 integration.

**Alternative (no Docker Desktop)**: Install Docker Engine natively in WSL2:

```bash
# Inside WSL2
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER
# Log out and back into WSL2
```

This avoids Docker Desktop licensing requirements and gives true `network_mode: host` behavior inside WSL2.

### 0.3: Install Docker Compose v2 (if using native Docker)

Docker Desktop includes Compose v2. If using native Docker in WSL2:

```bash
sudo apt-get install -y docker-compose-plugin
docker compose version
```

### 0.4: Install Make

The project uses a `Makefile` for common commands (`make up`, `make dashboard`, etc.).

```bash
# WSL2
sudo apt-get install -y make
```

On Windows (without WSL2):

```powershell
# PowerShell (admin)
winget install GnuWin32.Make
```

### 0.5: Install Node.js (for Dashboard development)

```bash
# Inside WSL2
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt-get install -y nodejs
node --version  # v20.x
npm --version
```

### 0.6: Clone Repository

```bash
# Inside WSL2
cd ~
git clone <REPO_URL> nvidia-digital-twin-pilot
cd nvidia-digital-twin-pilot
```

**GOTCHA**: Clone into the WSL2 filesystem (`~/nvidia-digital-twin-pilot`), NOT into `/mnt/c/...`. Accessing Windows filesystem from WSL2 is 10-50x slower for file I/O. Docker volume mounts from the WSL2 filesystem are fast.

### 0.7: ROS 2 on Host — NOT Needed for Phases 1–3

All ROS 2 runs inside containers on Brev. For Phase 4+ debugging, you can optionally install ROS 2 Jazzy in WSL2:

```bash
# Inside WSL2 (OPTIONAL — only for Phase 4+ local debugging)
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**CRITICAL**: Use `ros-jazzy-*` packages, NOT `ros-humble-*`. The Brev side uses Jazzy — DDS discovery works cross-distro but message serialization does not.

---

## 1. Phases 1–3: Development Support (Current)

During Phases 1–3, all execution happens on Brev. The local machine is used for:

### 1.1: VS Code Remote-SSH

1. Install VS Code on Windows
2. Install the "Remote - SSH" extension
3. Connect to Brev:
   - `Ctrl+Shift+P` → "Remote-SSH: Connect to Host"
   - Host: `<brev-ssh-host>` (from Brev console)
   - Or configure `~/.ssh/config`:
     ```
     Host brev-warehouse
       HostName <brev-ip>
       User ubuntu
       IdentityFile ~/.ssh/id_ed25519
     ```
4. Open `/home/ubuntu/docker/isaac-sim/data/nvidia-digital-twin-pilot` on Brev

This gives full IDE features (IntelliSense, debugging, terminal) running on Brev hardware.

### 1.2: Accessing Brev Services

With VS Code Remote-SSH, VS Code auto-forwards ports. You can access:

| Service | URL | Notes |
|---|---|---|
| Isaac Sim WebRTC Viewer | `http://localhost:8210` | Live simulation view (via web-viewer) |
| Warehouse Manager API | `http://localhost:8000` | FastAPI endpoints |
| Warehouse Manager Swagger | `http://localhost:8000/docs` | Interactive API docs |
| Redis | `localhost:6379` | For debugging with `redis-cli` |

These ports are auto-forwarded by VS Code when connected via Remote-SSH.

Alternatively, access directly via Brev IP:
- `http://<brev-ip>:8210` — WebRTC viewer
- `http://<brev-ip>:8000/docs` — API docs

### 1.3: Running Commands on Brev

All Makefile commands run in the VS Code terminal (which is on Brev):

```bash
# In VS Code terminal (connected to Brev):
make brev-up          # Start all services
make brev-logs        # Tail logs
make brev-status      # Check container status
make brev-down        # Stop everything
```

### 1.4: Git Workflow During Phases 1–3

```bash
# On Brev (via VS Code Remote-SSH terminal):
cd ~/docker/isaac-sim/data/nvidia-digital-twin-pilot
git add -A
git commit -m "Phase 1: forklift controller working"
git push

# Periodically sync to local for backup/review:
# On local WSL2:
cd ~/nvidia-digital-twin-pilot
git pull
```

### 1.5: Local Testing (Without Sim)

Even during Phases 1–3, you can unit test warehouse manager logic locally:

```bash
cd ~/nvidia-digital-twin-pilot

# Install Python dependencies
pip3 install -r warehouse-manager/requirements.txt

# Run dispatcher tests
python3 -m pytest warehouse-manager/tests/

# Test SQLite schema creation
python3 -c "
import asyncio
from warehouse_manager.models import init_db
asyncio.run(init_db())
print('Schema OK')
"
```

---

## 2. Phase 4 — Split Deployment (Local Side)

> **Not yet implemented.** This section describes the planned setup for when the warehouse manager moves to the local machine.

### 2.1: Install Tailscale

Tailscale creates a VPN mesh that gives each machine a stable `100.x.y.z` IP. You need it on **both** the local machine and Brev.

#### Step 1 — Install Tailscale on Windows (host)

```powershell
# PowerShell (admin)
winget install --id Tailscale.Tailscale --accept-source-agreements --accept-package-agreements

# After install, open Tailscale from system tray and sign in.
tailscale ip -4
# Note this IP
```

#### Step 2 — Install Tailscale on the Brev instance

```bash
# SSH into Brev
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
tailscale ip -4
# Note this IP — this is the BREV_TAILSCALE_IP
```

#### Step 3 — Install Tailscale in WSL2 (local)

**Option A (recommended)**: Tailscale inside WSL2 directly. This gives the WSL2 instance its own Tailscale IP, and Docker containers with `network_mode: host` inherit it.

```bash
# Inside WSL2
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
tailscale ip -4
```

**GOTCHA**: Tailscale in WSL2 requires `systemd` to be enabled:

```bash
# Check if systemd is running
ps -p 1 -o comm=
# If not "systemd", enable it:
sudo tee /etc/wsl.conf << 'EOF'
[boot]
systemd=true
EOF
# Then restart WSL2 from PowerShell: wsl --shutdown
```

**Option B**: Tailscale on Windows host + WSL2 mirrored networking:

```powershell
# In %USERPROFILE%\.wslconfig:
[wsl2]
networkingMode=mirrored
```

**Recommendation**: Use Option A. It's self-contained.

#### Step 4 — Verify connectivity

```bash
ping <brev-tailscale-ip>
# Must succeed with <50ms latency
```

### 2.2: Environment File

Create `.env` in the repo root (add to `.gitignore`!):

```bash
# .env — local machine settings (DO NOT COMMIT)
BREV_TAILSCALE_IP=100.x.y.z
ROS_DOMAIN_ID=42
```

### 2.3: ROS 2 Cross-Network Configuration

The project includes `fastdds_client.xml` for local-side containers to connect to Brev's discovery server:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <profiles>
        <participant profile_name="discovery_client" is_default_profile="true">
            <rtps>
                <builtin>
                    <discovery_config>
                        <discoveryProtocol>CLIENT</discoveryProtocol>
                        <discoveryServersList>
                            <RemoteServer prefix="44.53.00.5f.45.50.52.4f.53.49.4d.41">
                                <metatrafficUnicastLocatorList>
                                    <locator>
                                        <udpv4>
                                            <address>${BREV_TAILSCALE_IP}</address>
                                            <port>11811</port>
                                        </udpv4>
                                    </locator>
                                </metatrafficUnicastLocatorList>
                            </RemoteServer>
                        </discoveryServersList>
                    </discovery_config>
                </builtin>
            </rtps>
        </participant>
    </profiles>
</dds>
```

**GOTCHA**: `${BREV_TAILSCALE_IP}` is NOT auto-substituted by FastDDS. The `local-start.sh` script uses `envsubst` to resolve it:

```bash
export BREV_TAILSCALE_IP
envsubst < fastdds_client.xml > fastdds_client_resolved.xml
```

**GOTCHA**: The `prefix` value `44.53.00.5f.45.50.52.4f.53.49.4d.41` must match the discovery server's `server-id 0` prefix. This is a fixed value.

### 2.4: Local Docker Compose — `local-compose.yml`

```yaml
services:
  redis:
    image: redis:7-alpine
    ports:
      - "6379:6379"
    volumes:
      - redis-data:/data
    command: redis-server --appendonly yes
    restart: unless-stopped
    healthcheck:
      test: ["CMD", "redis-cli", "ping"]
      interval: 5s
      timeout: 3s
      retries: 5

  warehouse-manager:
    build:
      context: .
      dockerfile: warehouse-manager/Dockerfile
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=42
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      - ROS_DISCOVERY_SERVER=${BREV_TAILSCALE_IP}:11811
      - FASTRTPS_DEFAULT_PROFILES_FILE=/config/fastdds_client.xml
      - REDIS_URL=redis://localhost:6379
      - DATABASE_PATH=/data/warehouse.db
    volumes:
      - ./fastdds_client_resolved.xml:/config/fastdds_client.xml:ro
      - warehouse-db:/data
    depends_on:
      redis:
        condition: service_healthy
    restart: unless-stopped

  dashboard:
    build:
      context: ./dashboard
      dockerfile: Dockerfile
    ports:
      - "3000:80"
    environment:
      - VITE_API_URL=http://localhost:8000
      - VITE_WS_URL=ws://localhost:8000/ws/live
    depends_on:
      - warehouse-manager
    restart: unless-stopped
    profiles:
      - dashboard  # Only starts when explicitly requested

volumes:
  redis-data:
  warehouse-db:
```

**CRITICAL**: `network_mode: host` for warehouse-manager is REQUIRED for ROS 2 DDS communication — DDS uses multicast/unicast on dynamic ports that can't be easily mapped with Docker bridge networking.

**GOTCHA**: With `network_mode: host`, the container shares WSL2's network. Port 8000 (FastAPI) is accessible at `localhost:8000` from within WSL2. Docker Desktop auto-forwards to Windows. Native Docker in WSL2 usually works via `localhost` in recent WSL2 versions.

**GOTCHA**: Redis uses bridge networking (default) with port mapping, while warehouse-manager uses host networking. Warehouse-manager reaches Redis at `localhost:6379` because the Redis port is published to the host.

### 2.5: Startup Script — `local-start.sh`

```bash
#!/bin/bash
set -euo pipefail

# Load environment
source .env

# Resolve FastDDS config with actual Brev IP
export BREV_TAILSCALE_IP
envsubst < fastdds_client.xml > fastdds_client_resolved.xml

# Verify Brev connectivity
echo "Checking Brev connectivity..."
if ! nc -zw3 "$BREV_TAILSCALE_IP" 11811; then
    echo "ERROR: Cannot reach FastDDS discovery server at $BREV_TAILSCALE_IP:11811"
    echo "Is Tailscale connected? Is Brev running?"
    exit 1
fi
echo "Brev connectivity OK"

# Start services
docker compose -f local-compose.yml up --build -d

echo ""
echo "=== Local services started ==="
echo "Warehouse Manager API: http://localhost:8000"
echo "Warehouse Manager docs: http://localhost:8000/docs"
echo "Redis: localhost:6379"
echo ""
echo "To start dashboard:"
echo "  docker compose -f local-compose.yml --profile dashboard up --build -d"
```

### 2.6: Makefile Commands (Local)

```bash
make up              # Start core services (redis + warehouse-manager)
make down            # Stop all services
make dashboard       # Start dashboard (http://localhost:3000)
make dashboard-dev   # Start dashboard in Vite dev mode (http://localhost:5173)
make logs            # Tail logs
make status          # Show running containers
make restart         # Restart all
make clean           # Stop + remove volumes
```

### 2.7: Warehouse Manager Dockerfile

The Dockerfile builds for `ros:jazzy` (matching Brev):

```dockerfile
FROM ros:jazzy

RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-jazzy-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

COPY warehouse-manager/requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir --break-system-packages -r /tmp/requirements.txt

WORKDIR /ros2_ws/src
COPY warehouse_msgs/ warehouse_msgs/

WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --packages-select warehouse_msgs && \
    echo 'source /ros2_ws/install/setup.bash' >> /etc/bash.bashrc"

COPY warehouse-manager/ /app/
WORKDIR /app
EXPOSE 8000
```

**NOTE**: The Dockerfile's default CMD still references `ros:humble`. The compose `command:` overrides this, but fix the CMD if running standalone:

```dockerfile
# Fix this line in warehouse-manager/Dockerfile:
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && uvicorn main:app --host 0.0.0.0 --port 8000"]
```

**GOTCHA**: `ros:jazzy` uses Python 3.12 — pip needs `--break-system-packages` flag.

**Python dependencies** (`requirements.txt`):
```
fastapi>=0.104.0
uvicorn[standard]>=0.24.0
aiosqlite>=0.19.0
redis>=5.0.0
pyyaml>=6.0
websockets>=12.0
scipy>=1.11.0
```

### 2.8: Phase 4 Validation

```bash
cd ~/nvidia-digital-twin-pilot
git pull  # Get latest code from Brev development

# Start local services
chmod +x local-start.sh
./local-start.sh

# Wait for services to start
docker compose -f local-compose.yml logs -f warehouse-manager
# Wait for "Uvicorn running on http://0.0.0.0:8000"
```

**Validation**:

```bash
# 1. API is accessible
curl http://localhost:8000/docs
# Should return Swagger UI HTML

# 2. ROS 2 topics visible from warehouse manager
docker compose -f local-compose.yml exec warehouse-manager \
  bash -c "source /opt/ros/jazzy/setup.bash && \
           source /ros2_ws/install/setup.bash && \
           ros2 topic list"
# Must show Brev topics: /clock, /forklift_*/odom, etc.

# 3. Create order → verify it reaches Brev
curl -X POST http://localhost:8000/orders \
  -H "Content-Type: application/json" \
  -d '{"items": ["box_001"], "priority": 1}'

# 4. Check order status
curl http://localhost:8000/orders
# Order should progress: pending → in_progress → completed

# 5. Forklift positions from API
curl http://localhost:8000/forklifts
# Should show forklift_0..3 with current positions

# 6. Metrics
curl http://localhost:8000/metrics

# 7. Persistence test
docker compose -f local-compose.yml restart warehouse-manager
sleep 5
curl http://localhost:8000/orders
# Previous orders must still be visible (loaded from SQLite)
```

### 2.9: Test ROS 2 Discovery from WSL2 (Optional, for debugging)

If you installed ROS 2 Jazzy locally (Section 0.7), test bare ROS 2 works across Tailscale:

```bash
# Inside WSL2
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=<brev-tailscale-ip>:11811
export FASTRTPS_DEFAULT_PROFILES_FILE=$(pwd)/fastdds_client_resolved.xml

ros2 topic list
# Expected: /clock, /forklift_0/odom, /forklift_0/status, etc.

ros2 topic echo /clock --once
ros2 topic delay /forklift_0/odom
# Expected: <50ms over Tailscale
```

**Troubleshooting** if `ros2 topic list` shows nothing:

1. Check Tailscale: `ping <brev-tailscale-ip>`
2. Check discovery server port: `nc -zv <brev-tailscale-ip> 11811`
3. Check `ROS_DOMAIN_ID` matches (must be 42)
4. Kill ROS 2 daemon: `ros2 daemon stop && ros2 daemon start`
5. Check firewall: Tailscale should bypass Windows firewall; verify WSL2 isn't blocking UDP

---

## 3. Dashboard

### 3.1: Tech Stack

| Package | Version | Purpose |
|---|---|---|
| React | 18.3.x | UI framework |
| TypeScript | 5.4.x | Type safety |
| Vite | 5.2.x | Build tool + dev server |
| Zustand | 4.5.x | State management |
| react-konva + konva | 9.3.x / 18.2.x | 2D canvas warehouse map |
| recharts | 2.12.x | Charts for metrics |
| reconnecting-websocket | 4.4.x | WebSocket with auto-reconnect |
| react-router-dom | 6.22.x | Client-side routing |

### 3.2: Project Structure

```
dashboard/
├── Dockerfile           ← Multi-stage: node:20 build → nginx:alpine serve
├── package.json
├── tsconfig.json
├── vite.config.ts
├── index.html
└── src/
    ├── App.tsx
    ├── config.ts        ← API_URL, WS_URL from env
    ├── main.tsx
    ├── vite-env.d.ts
    ├── components/
    │   ├── ConfigPanel/
    │   ├── FleetStatus/
    │   ├── MetricsPanel/
    │   ├── OrderQueue/
    │   └── WarehouseMap/
    ├── hooks/
    ├── stores/
    └── types/
```

### 3.3: Dashboard Dockerfile

```dockerfile
# Build stage
FROM node:20-alpine AS build
WORKDIR /app
COPY package.json package-lock.json* ./
RUN npm ci || npm install
COPY . .
RUN npm run build

# Production stage
FROM nginx:alpine
COPY --from=build /app/dist /usr/share/nginx/html
# Nginx SPA routing config
RUN printf 'server {\n\
    listen 80;\n\
    location / {\n\
    root /usr/share/nginx/html;\n\
    index index.html;\n\
    try_files $uri $uri/ /index.html;\n\
    }\n\
    }\n' > /etc/nginx/conf.d/default.conf
EXPOSE 80
```

**GOTCHA**: `try_files` is essential for client-side routing. Without it, refreshing on `/orders` returns 404.

**GOTCHA**: Vite env variables (`VITE_API_URL`, `VITE_WS_URL`) are baked in at build time. For runtime config, use a `config.js` loaded by `index.html`.

### 3.4: Development Workflow

```bash
# Production mode (via Docker, port 3000)
make dashboard
# Open http://localhost:3000

# Development mode (Vite hot reload, port 5173)
make dashboard-dev
# OR:
cd dashboard && npm install && npm run dev
# Open http://localhost:5173
```

### 3.5: WebSocket Connection

The dashboard connects to the warehouse manager's WebSocket at `ws://localhost:8000/ws/live` for real-time updates.

Message types:
- `forklift_update` — real-time forklift position/state (~10 Hz)
- `order_update` — order lifecycle changes
- `task_status` — task state transitions
- `metric_update` — periodic KPI snapshots
- `collision` — collision events from contact sensors

Uses `reconnecting-websocket` for auto-reconnect with backoff. Do NOT implement custom reconnection logic.

### 3.6: Warehouse Map Coordinate System

```
Warehouse: ~70m × 90m (real-world from USD)
  X range: -36 to +33
  Y range: -36 to +52

Dashboard abstract space: 50m × 30m
Scale factor: min(canvasWidth/50, canvasHeight/30)
Origin: bottom-left of warehouse = top-left of canvas (flip Y)

Forklift rendering:
  Triangle pointing in direction of yaw
  Color by state: green=IDLE, blue=NAVIGATING, yellow=PICKING/DROPPING, red=ERROR
```

### 3.7: REST API Endpoints

| Method | Endpoint | Purpose |
|---|---|---|
| `GET` | `/forklifts` | All forklift states |
| `GET` | `/orders` | All orders |
| `POST` | `/orders` | Create order `{"items": [...], "priority": N}` |
| `GET` | `/metrics` | KPI snapshots |
| `POST` | `/reset` | Cancel all tasks, reset forklifts |
| `PUT` | `/forklifts/{id}/pause` | Pause forklift |
| `PUT` | `/forklifts/{id}/resume` | Resume forklift |
| `DELETE` | `/tasks/{id}` | Cancel specific task |
| `POST` | `/config` | Change dispatch strategy (`nearest` / `batched`) |
| `WS` | `/ws/live` | Real-time event stream |
| `GET` | `/docs` | Swagger UI |

Commands go via REST (client → server) for better error handling. WebSocket is for real-time pushes only (server → client).

---

## 4. Key Files Reference

### 4.1: Existing Files (already in repo)

| File | Purpose | Status |
|---|---|---|
| `local-compose.yml` | Local Docker Compose (Phase 4+) | Ready, not yet active |
| `local-start.sh` | Startup script with Brev connectivity check | Ready, not yet active |
| `fastdds_client.xml` | Discovery client XML (needs `envsubst`) | Ready |
| `Makefile` | `make up`, `make down`, `make dashboard`, etc. | Ready |
| `dashboard/Dockerfile` | Multi-stage Node → Nginx build | Ready |
| `dashboard/package.json` | All dependencies declared | Ready |
| `warehouse-manager/Dockerfile` | `ros:jazzy` + FastAPI + warehouse_msgs | Ready (CMD needs humble→jazzy fix) |
| `warehouse-manager/requirements.txt` | fastapi, uvicorn, aiosqlite, redis, scipy, etc. | Ready |

### 4.2: Files to Create Before Phase 4

| File | Purpose |
|---|---|
| `.env` | `BREV_TAILSCALE_IP=100.x.y.z` and `ROS_DOMAIN_ID=42` |
| `.gitignore` entry | Add `.env` and `fastdds_client_resolved.xml` |

### 4.3: Known Issues to Fix Before Phase 4

1. **`warehouse-manager/Dockerfile` CMD**: Still says `source /opt/ros/humble/setup.bash` — change to `jazzy`. The compose `command:` overrides this, but standalone runs will break.
2. **`brev-compose-phase4.yml`**: Completely outdated — references `ros:humble`, uses `./python.sh` instead of `--exec`, missing `fastdds_no_shm.xml` volumes, missing `isaac-sim-init`, `web-viewer`, and `PRIVACY_CONSENT`. Must be rewritten to match current `brev-compose.yml` minus warehouse-manager and redis.
3. **Dashboard `VITE_*` env vars**: Need to point to Brev IP for WebRTC viewer URL if you want to embed the sim view in the dashboard.

---

## 5. Git Workflow

### 5.1: Branch Strategy

```
main              ← stable, always deployable
├── phase-1       ← Phase 1 work (complete, merged)
├── phase-2       ← Phase 2 work
├── phase-3       ← Phase 3 work
├── phase-4       ← Split deployment setup
├── phase-5       ← Dashboard
├── phase-6       ← Optimization
└── feature/*     ← Feature branches off phase branches
```

### 5.2: Code Sync Workflow (Phase 4+)

```bash
# Developer edits code on local machine (VS Code)
# For warehouse-manager or dashboard changes:
cd ~/nvidia-digital-twin-pilot
git add -A
git commit -m "warehouse-manager: add batch dispatcher"
git push

# Rebuild local services
docker compose -f local-compose.yml up --build -d warehouse-manager

# For vehicle-controller or sim-scripts changes:
# Push from local, then on Brev:
cd ~/docker/isaac-sim/data/nvidia-digital-twin-pilot
git pull
docker compose -f brev-compose.yml up --build -d vehicle-controller
# For Isaac Sim script changes, restart isaac-sim:
docker compose -f brev-compose.yml restart isaac-sim
```
