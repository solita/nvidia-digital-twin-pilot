# Local Machine — Implementation Plan

> **Scope**: Everything that runs on the local Windows/WSL2 machine.
> **Phases 1–3**: Local machine is for code editing only (VS Code Remote-SSH into Brev). All execution happens on Brev.
> **Phase 4+**: Warehouse manager, Redis, SQLite, and React dashboard run locally. Connected to Brev via Tailscale.

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

**⚠ GOTCHA**: Do NOT install Docker Engine inside WSL2 separately if you're using Docker Desktop. They conflict. Use Docker Desktop's WSL2 integration.

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

### 0.4: Install Make (for task shortcuts)

The project uses a `Makefile` for common commands (`make up`, `make dashboard`, etc.). Install `make` in WSL2:

```bash
sudo apt-get install -y make
make --version
```

On Windows (without WSL2), install via [Chocolatey](https://chocolatey.org/):

```powershell
# PowerShell (admin)
choco install make
```

Or via winget:

```powershell
winget install GnuWin32.Make
```

### 0.5: Install ROS 2 Humble in WSL2 (for debugging)

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install -y ros-humble-desktop python3-colcon-common-extensions
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 0.6: Install Tailscale

Tailscale creates a VPN mesh that gives each machine a stable `100.x.y.z` IP. You need it on **both** the local machine and Brev so ROS 2 DDS traffic can flow between them.

#### Step 1 — Install Tailscale on Windows (host)

```powershell
# PowerShell (admin) — install via winget
winget install --id Tailscale.Tailscale --accept-source-agreements --accept-package-agreements

# After install, open Tailscale from the Start menu or system tray and sign in.
# Alternatively, sign in from the command line:
tailscale up
tailscale ip -4
# Note this IP — it's your Windows host Tailscale IP.
```

If `winget` is not available, download the installer from https://tailscale.com/download/windows.

#### Step 2 — Install Tailscale on the Brev instance

```bash
# SSH into Brev
ssh <brev-ssh-host>

# Install Tailscale
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
tailscale ip -4
# Note this IP — this is the BREV_TAILSCALE_IP you put in .env
```

#### Step 3 — Install Tailscale in WSL2 (local)

**Option A (recommended)**: Tailscale inside WSL2 directly. This gives the WSL2 instance its own Tailscale IP, and Docker containers with `network_mode: host` inherit it.

```bash
# Inside WSL2
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
tailscale ip -4
# Note this IP — Brev needs it for cross-network ROS 2
```

**⚠ GOTCHA**: WSL2's default networking uses NAT. Tailscale in WSL2 creates a userspace tunnel that works regardless. However, Tailscale in WSL2 requires `systemd` to be enabled:

```bash
# Check if systemd is running
ps -p 1 -o comm=
# If not "systemd", enable it:
# Edit /etc/wsl.conf (create if missing):
sudo tee /etc/wsl.conf << 'EOF'
[boot]
systemd=true
EOF
# Then restart WSL2 from PowerShell:
# wsl --shutdown
# Re-open Ubuntu terminal
```

**Option B**: Tailscale on Windows host + WSL2 mirrored networking. This is simpler but requires WSL2 mirrored mode:

```powershell
# In %USERPROFILE%\.wslconfig:
[wsl2]
networkingMode=mirrored
```

With mirrored networking, WSL2 shares the Windows host's Tailscale network. Restart WSL2 after changing this.

**⚠ GOTCHA**: Mirrored networking mode changes WSL2's IP behavior significantly. Some Docker networking features may behave differently. Test thoroughly.

**Recommendation**: Use Option A (Tailscale in WSL2). It's self-contained and doesn't affect Windows networking.

### 0.7: Install Node.js (for Dashboard)

```bash
# Inside WSL2
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt-get install -y nodejs
node --version  # v20.x
npm --version
```

### 0.8: Clone Monorepo

```bash
# Inside WSL2
cd ~
git clone <REPO_URL> warehouse-sim
cd warehouse-sim
```

**⚠ GOTCHA**: Clone into the WSL2 filesystem (`~/warehouse-sim`), NOT into `/mnt/c/...`. Accessing Windows filesystem from WSL2 is slow (10-50x slower for file I/O). Docker volume mounts from the WSL2 filesystem are fast.

### 0.9: Build warehouse_msgs Locally (for IDE support)

Even though the warehouse manager runs in Docker, building messages locally gives IDE autocompletion:

```bash
cd ~/warehouse-sim
mkdir -p ros2_ws/src
ln -s ~/warehouse-sim/warehouse_msgs ros2_ws/src/warehouse_msgs

cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select warehouse_msgs
source install/setup.bash

# Verify
ros2 interface show warehouse_msgs/msg/TaskAssignment
```

---

## 1. Phases 1–3: Development Support

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
       User <brev-user>
       IdentityFile ~/.ssh/id_ed25519
     ```
4. Open `/root/warehouse-sim` on Brev

This gives full IDE features (IntelliSense, debugging, terminal) running on Brev hardware.

### 1.2: Git Workflow During Phases 1–3

```bash
# On Brev (via VS Code Remote-SSH terminal):
cd ~/warehouse-sim
git add -A
git commit -m "Phase 1: basic forklift controller"
git push

# Periodically sync to local for backup/review:
# On local WSL2:
cd ~/warehouse-sim
git pull
```

### 1.3: Local Testing of Warehouse Manager Code (Without Sim)

Even in Phase 1–3, you can unit test warehouse manager logic locally:

```bash
cd ~/warehouse-sim
pip3 install fastapi uvicorn aiosqlite redis pyyaml

# Run unit tests (if any)
python3 -m pytest warehouse-manager/tests/

# Test SQLite schema creation
python3 -c "
import sqlite3
conn = sqlite3.connect(':memory:')
# Execute schema from models.py
print('Schema OK')
"
```

---

## 2. Phase 4 — Split Deployment (Local Side)

### 2.1: Verify Tailscale Connectivity

```bash
# Inside WSL2
tailscale status
# Must show both local and Brev machines

ping <brev-tailscale-ip>
# Must succeed with <50ms latency

# Test FastDDS discovery port
nc -zv <brev-tailscale-ip> 11811
# Must show "Connection succeeded"
```

### 2.2: ROS 2 Cross-Network Configuration

Create `fastdds_client.xml` in the repo root (for local-side containers):

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

**⚠ GOTCHA**: The `prefix` in this XML MUST match the prefix used by the discovery server on Brev. The prefix `44.53.00.5f.45.50.52.4f.53.49.4d.41` corresponds to `server-id 0` (the default used by `fastdds discovery --server-id 0`).

**⚠ GOTCHA**: `${BREV_TAILSCALE_IP}` is NOT auto-substituted by FastDDS. You must replace it with the actual IP before use. Use `envsubst` or a wrapper script:

```bash
export BREV_TAILSCALE_IP=100.x.y.z  # Your Brev's Tailscale IP
envsubst < fastdds_client.xml > /tmp/fastdds_client_resolved.xml
```

### 2.3: Test ROS 2 Discovery from WSL2

Before Docker, verify bare ROS 2 works across Tailscale:

```bash
# Inside WSL2
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=<brev-tailscale-ip>:11811
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_client_resolved.xml

# Must show topics from Brev (Isaac Sim + vehicle controller)
ros2 topic list
# Expected: /clock, /forklift_0/odom, /forklift_0/status, /warehouse/task_status, ...

# Verify data flows
ros2 topic echo /clock --once
ros2 topic echo /forklift_0/status --once

# Measure latency
ros2 topic delay /forklift_0/status
# Expected: <50ms over Tailscale
```

**⚠ GOTCHA**: If `ros2 topic list` shows nothing:

1. Check Tailscale connectivity: `ping <brev-tailscale-ip>`
2. Check discovery server port: `nc -zv <brev-tailscale-ip> 11811`
3. Check `ROS_DOMAIN_ID` matches on both sides (must be 42)
4. Kill and restart the ROS 2 daemon: `ros2 daemon stop && ros2 daemon start`
5. Check firewall: Tailscale should bypass Windows firewall, but verify WSL2 isn't blocking UDP

### 2.4: Warehouse Manager Dockerfile

Create `warehouse-manager/Dockerfile`:

```dockerfile
FROM ros:humble

# System dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

# Python dependencies
COPY warehouse-manager/requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# Build warehouse_msgs
WORKDIR /ros2_ws/src
COPY warehouse_msgs/ warehouse_msgs/

WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --packages-select warehouse_msgs && \
    echo 'source /ros2_ws/install/setup.bash' >> /etc/bash.bashrc"

# Copy warehouse manager code
COPY warehouse-manager/ /app/
WORKDIR /app

EXPOSE 8000

# NOTE: use_sim_time=true is set in the WarehouseManagerNode constructor
# (parameter_overrides), NOT via CLI --ros-args. This means uvicorn can
# launch normally without passing ROS arguments.
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && uvicorn main:app --host 0.0.0.0 --port 8000"]
```

Create `warehouse-manager/requirements.txt`:

```
fastapi>=0.104.0
uvicorn[standard]>=0.24.0
aiosqlite>=0.19.0
redis>=5.0.0
pyyaml>=6.0
websockets>=12.0
```

### 2.5: Local Docker Compose

Create `local-compose.yml`:

```yaml
services:
  # --- Redis ---
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

  # --- Warehouse Manager ---
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

  # --- Dashboard (Phase 5+) ---
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
      - dashboard # Only starts when explicitly requested

volumes:
  redis-data:
  warehouse-db:
```

**⚠ CRITICAL**: `network_mode: host` for the warehouse-manager container. This is REQUIRED for ROS 2 DDS communication — DDS uses multicast/unicast on dynamic ports that can't be easily mapped with Docker bridge networking.

**⚠ GOTCHA**: With `network_mode: host`, the container shares WSL2's network. Port 8000 (FastAPI) is accessible at `localhost:8000` from within WSL2. To access from Windows browser, WSL2 must forward the port. Docker Desktop does this automatically. Native Docker in WSL2 requires `wsl --exec` or Windows `localhost` forwarding (usually works by default in recent WSL2 versions).

**⚠ GOTCHA**: The Redis container uses bridge networking (default), but warehouse-manager uses host networking. This means warehouse-manager reaches Redis at `localhost:6379` because the Redis port is published to the host. If Redis also used host networking, it would work the same way but without the port mapping.

### 2.6: Environment File

Create `.env` in the repo root (add to `.gitignore`!):

```bash
# .env — local machine settings (DO NOT COMMIT)
BREV_TAILSCALE_IP=100.x.y.z
ROS_DOMAIN_ID=42
```

Create a startup script `local-start.sh`:

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
echo "To start dashboard (Phase 5+):"
echo "  docker compose -f local-compose.yml --profile dashboard up --build -d"
```

### 2.7: Phase 4 Validation

```bash
cd ~/warehouse-sim
git pull  # Get latest code from Brev development

# Resolve FastDDS config
export BREV_TAILSCALE_IP=100.x.y.z
envsubst < fastdds_client.xml > fastdds_client_resolved.xml

# Start local services
chmod +x local-start.sh
./local-start.sh

# Wait for services to start
docker compose -f local-compose.yml logs -f warehouse-manager
# Wait for "Uvicorn running on http://0.0.0.0:8000"
```

Validation:

```bash
# 1. API is accessible
curl http://localhost:8000/docs
# Should return Swagger UI HTML

# 2. ROS 2 topics visible from warehouse manager
docker compose -f local-compose.yml exec warehouse-manager \
  bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"
# Must show Brev topics: /clock, /forklift_*/odom, etc.

# 3. Create order → verify it reaches Brev
curl -X POST http://localhost:8000/orders \
  -H "Content-Type: application/json" \
  -d '{"items": ["box_001"], "priority": 1}'

# 4. Watch task status flow back
curl http://localhost:8000/orders
# Order should progress: pending → in_progress → completed

# 5. WebSocket events
# Install websocat in WSL2:
cargo install websocat  # or download binary
websocat ws://localhost:8000/ws/live
# Should see real-time events as forklifts move

# 6. Forklift positions from API
curl http://localhost:8000/forklifts
# Should show all 4 forklifts with current positions

# 7. Metrics
curl http://localhost:8000/metrics
# Should show KPIs

# 8. Latency test
# From WSL2 host (not Docker):
export ROS_DOMAIN_ID=42
export ROS_DISCOVERY_SERVER=${BREV_TAILSCALE_IP}:11811
ros2 topic delay /warehouse/task_status
# Expected: <50ms

# 9. Persistence test
docker compose -f local-compose.yml restart warehouse-manager
sleep 5
curl http://localhost:8000/orders
# Previous orders must still be visible (loaded from SQLite)

# 10. Tailscale disconnect recovery
# Kill Tailscale temporarily:
sudo tailscale down
sleep 10
sudo tailscale up
# Warehouse manager should reconnect to Brev ROS 2 topics
# Check logs:
docker compose -f local-compose.yml logs --tail=50 warehouse-manager

# 11. Reset test
curl -X POST http://localhost:8000/reset
curl http://localhost:8000/orders
# In-flight orders should be cancelled
curl http://localhost:8000/forklifts
# All forklifts should be IDLE
```

---

## 3. Phase 5 — React Dashboard

### 3.1: Scaffold React App

```bash
cd ~/warehouse-sim/dashboard
npm create vite@latest . -- --template react-ts

npm install react-router-dom zustand
npm install react-konva konva           # 2D canvas for warehouse map
npm install recharts                     # Charts for metrics
npm install reconnecting-websocket      # WebSocket with auto-reconnect

# Dev dependencies
npm install -D @types/react @types/react-dom
```

### 3.2: Project Structure

```
dashboard/
├── src/
│   ├── components/
│   │   ├── WarehouseMap/
│   │   │   ├── WarehouseMap.tsx         # Main canvas component
│   │   │   ├── ForkliftSprite.tsx       # Forklift triangle + label
│   │   │   ├── ShelfRect.tsx            # Shelf rack rectangle
│   │   │   ├── DockRect.tsx             # Loading dock rectangle
│   │   │   └── PathLine.tsx             # Planned route overlay
│   │   ├── OrderQueue/
│   │   │   ├── OrderQueue.tsx           # Sortable order table
│   │   │   └── OrderRow.tsx             # Single order row
│   │   ├── FleetStatus/
│   │   │   ├── FleetStatus.tsx          # Fleet overview
│   │   │   └── ForkliftCard.tsx         # Individual forklift card
│   │   ├── MetricsPanel/
│   │   │   ├── MetricsPanel.tsx         # KPI dashboard
│   │   │   ├── ThroughputChart.tsx      # Orders/time chart
│   │   │   └── UtilizationChart.tsx     # Fleet utilization chart
│   │   ├── ConfigPanel/
│   │   │   └── ConfigPanel.tsx          # Order rate, layout controls
│   │   └── Layout/
│   │       ├── Header.tsx
│   │       └── Sidebar.tsx
│   ├── hooks/
│   │   ├── useWebSocket.ts             # WebSocket with reconnect + message parsing
│   │   └── useApi.ts                   # REST API wrapper (fetch-based)
│   ├── stores/
│   │   ├── warehouseStore.ts           # Zustand: forklifts, shelves, docks
│   │   ├── orderStore.ts              # Zustand: orders, tasks
│   │   └── metricsStore.ts           # Zustand: KPI data
│   ├── types/
│   │   ├── forklift.ts                # ForkliftStatus type
│   │   ├── order.ts                   # Order, Task types
│   │   ├── metrics.ts                 # Metrics types
│   │   └── websocket.ts              # WebSocket message types
│   ├── config.ts                      # API_URL, WS_URL from env
│   ├── App.tsx
│   └── main.tsx
├── public/
├── index.html
├── package.json
├── tsconfig.json
├── vite.config.ts
└── Dockerfile
```

### 3.3: Key TypeScript Types

Create `src/types/forklift.ts`:

```typescript
export enum ForkliftState {
  IDLE = 0,
  NAVIGATING_TO_SHELF = 1,
  PICKING = 2,
  NAVIGATING_TO_DOCK = 3,
  DROPPING = 4,
  ERROR = 5,
  RECOVERING = 6,
}

export interface ForkliftStatus {
  forklift_id: string;
  state: ForkliftState;
  pose: { x: number; y: number; z: number; yaw: number };
  battery_level: number;
  current_task_id: string | null;
}
```

Create `src/types/order.ts`:

```typescript
export interface Order {
  id: string;
  items: string[];
  priority: number;
  status: "pending" | "in_progress" | "completed" | "cancelled";
  assigned_forklift: string | null;
  created_at: string;
  completed_at: string | null;
}
```

Create `src/types/websocket.ts`:

```typescript
export type WSMessage =
  | { type: "forklift_update"; data: ForkliftStatus }
  | { type: "order_update"; data: Order }
  | {
      type: "task_status";
      data: { task_id: string; status: string; forklift_id: string };
    }
  | { type: "metric_update"; data: MetricSnapshot }
  | { type: "collision"; data: { forklift_id: string; timestamp: string } };
```

### 3.4: WebSocket Hook

Create `src/hooks/useWebSocket.ts`:

```typescript
import { useEffect, useRef, useCallback } from "react";
import ReconnectingWebSocket from "reconnecting-websocket";
import { WSMessage } from "../types/websocket";
import { useWarehouseStore } from "../stores/warehouseStore";
import { useOrderStore } from "../stores/orderStore";

const WS_URL = import.meta.env.VITE_WS_URL || "ws://localhost:8000/ws/live";

export function useWebSocket() {
  const wsRef = useRef<ReconnectingWebSocket | null>(null);
  const updateForklift = useWarehouseStore((s) => s.updateForklift);
  const updateOrder = useOrderStore((s) => s.updateOrder);

  useEffect(() => {
    const ws = new ReconnectingWebSocket(WS_URL, [], {
      maxRetries: Infinity,
      reconnectionDelayGrowFactor: 1.5,
      maxReconnectionDelay: 10000,
      minReconnectionDelay: 1000,
    });

    ws.onmessage = (event) => {
      const msg: WSMessage = JSON.parse(event.data);
      switch (msg.type) {
        case "forklift_update":
          updateForklift(msg.data);
          break;
        case "order_update":
          updateOrder(msg.data);
          break;
        // ... handle other types
      }
    };

    ws.onopen = () => console.log("WebSocket connected");
    ws.onclose = () => console.log("WebSocket disconnected, reconnecting...");

    wsRef.current = ws;
    return () => ws.close();
  }, [updateForklift, updateOrder]);

  // Send commands via WebSocket (for manual overrides)
  const send = useCallback((msg: object) => {
    wsRef.current?.send(JSON.stringify(msg));
  }, []);

  return { send };
}
```

**⚠ GOTCHA**: The `reconnecting-websocket` library handles reconnection automatically. Do NOT implement custom reconnection logic — it leads to duplicate connections. The library handles backoff, retry limits, and cleanup.

### 3.5: Warehouse Map Component

Create `src/components/WarehouseMap/WarehouseMap.tsx`:

Key implementation details:

```typescript
// Coordinate mapping:
// Warehouse: 50m × 30m (real world)
// Canvas: scale to fit the viewport
// Scale factor: min(canvasWidth/50, canvasHeight/30)
// Origin: bottom-left of warehouse = top-left of canvas (flip Y)

// Forklift rendering:
// - Triangle pointing in direction of yaw
// - Color by state: green=IDLE, blue=NAVIGATING, yellow=PICKING/DROPPING, red=ERROR
// - Label with forklift_id below

// Update rate: 10Hz from WebSocket is fine for visualization
// Use requestAnimationFrame for smooth rendering, batch WebSocket updates
```

**⚠ GOTCHA**: Konva (react-konva) re-renders the entire canvas on state change. For 4 forklifts at 10Hz this is fine. For higher forklift counts, use `React.memo` on individual sprites and only update the changed ones.

### 3.6: Manual Overrides

```typescript
// Pause forklift: PUT /forklifts/{id}/pause
// Resume forklift: PUT /forklifts/{id}/resume
// Cancel task: DELETE /tasks/{id}
// Create order: POST /orders
// Reset simulation: POST /reset

// All via REST API, not WebSocket
// WebSocket is for real-time pushes only (server → client)
// Commands go via REST (client → server) for better error handling
```

### 3.7: Dashboard Dockerfile

Create `dashboard/Dockerfile`:

```dockerfile
# Build stage
FROM node:20-alpine AS build
WORKDIR /app
COPY package.json package-lock.json ./
RUN npm ci
COPY . .
RUN npm run build

# Production stage
FROM nginx:alpine
COPY --from=build /app/dist /usr/share/nginx/html

# Nginx config for SPA routing
RUN echo 'server { \
    listen 80; \
    location / { \
        root /usr/share/nginx/html; \
        index index.html; \
        try_files $uri $uri/ /index.html; \
    } \
}' > /etc/nginx/conf.d/default.conf

EXPOSE 80
```

**⚠ GOTCHA**: The `try_files` directive is essential for client-side routing. Without it, refreshing on `/orders` returns 404.

**⚠ GOTCHA**: Environment variables (VITE_API_URL, VITE_WS_URL) are baked in at build time with Vite. For runtime configuration, use a `config.js` loaded by `index.html` or a `/config` endpoint.

### 3.8: Phase 5 Validation

```bash
# Start dashboard alongside other services
cd ~/warehouse-sim
docker compose -f local-compose.yml --profile dashboard up --build -d

# OR for development (hot reload):
cd ~/warehouse-sim/dashboard
npm run dev
# Opens at http://localhost:5173
```

Validation:

| Check                      | How                                                  | Expected                                       |
| -------------------------- | ---------------------------------------------------- | ---------------------------------------------- |
| Dashboard loads            | Open `http://localhost:3000` (prod) or `:5173` (dev) | Warehouse map visible                          |
| Forklift positions update  | Watch map for 10s                                    | Triangles move                                 |
| Order creation             | Click "New Order" → fill form → submit               | Order appears in queue                         |
| Order flows through system | Watch created order                                  | Status: pending → in_progress → completed      |
| Pause forklift             | Click forklift → "Pause"                             | Forklift stops moving in Isaac Sim             |
| Resume forklift            | Click paused forklift → "Resume"                     | Forklift resumes                               |
| Metrics populate           | Open metrics panel                                   | Charts show data points                        |
| WebSocket reconnect        | Kill warehouse-manager, restart                      | Dashboard reconnects automatically             |
| Reset works                | Click "Reset Simulation"                             | All orders cancelled, forklifts return to IDLE |

**Development workflow for dashboard**:

```bash
# Dashboard dev can happen independently of Brev
# 1. Run warehouse-manager locally (connects to Brev for ROS 2)
# 2. Run dashboard in dev mode with hot reload
cd ~/warehouse-sim/dashboard
npm run dev

# If Brev is not available, mock the API:
# Create a simple mock server for local development
# (useful when Brev instance is down)
```

---

## 4. Phase 6 — Optimization (Local Side)

### 4.1: Fleet Routing Optimization

Update `warehouse-manager/dispatcher.py`:

```python
# Batched optimization dispatcher
# 1. Collect orders for a configurable window (default 30s)
# 2. At end of window, solve assignment problem:
#    - Build cost matrix: cost[i][j] = distance from forklift_i to order_j's shelf
#    - Solve with scipy.optimize.linear_sum_assignment
#    - Assign optimally
# 3. Fall back to nearest-available for single orders outside the window

from scipy.optimize import linear_sum_assignment
import numpy as np

def batch_assign(orders: list, forklifts: list) -> dict:
    n = max(len(orders), len(forklifts))
    cost = np.full((n, n), 1e9)  # Large default cost
    for i, f in enumerate(forklifts):
        for j, o in enumerate(orders):
            cost[i][j] = euclidean_distance(f.position, o.shelf_position)
    row_ind, col_ind = linear_sum_assignment(cost)
    return {forklifts[i].id: orders[j].id
            for i, j in zip(row_ind, col_ind)
            if j < len(orders) and i < len(forklifts)}
```

Add `scipy` to `warehouse-manager/requirements.txt`.

### 4.2: Scenario Replay

Create `warehouse-manager/scenario.py`:

```python
# Record mode: intercept all TaskAssignment publishes, log to JSON
# {
#   "scenario_id": "run_001",
#   "start_time": "2026-04-23T10:00:00Z",
#   "events": [
#     {"t_offset_ms": 0, "type": "order", "data": {"items": ["box_001"], "priority": 1}},
#     {"t_offset_ms": 5000, "type": "order", "data": {"items": ["box_002"], "priority": 2}},
#     ...
#   ]
# }

# Replay mode: read JSON, inject orders at recorded timestamps
# POST /replay/start?scenario=run_001
# POST /replay/stop
```

### 4.3: A/B Testing Metrics

Add to `warehouse-manager/main.py`:

```python
# POST /config endpoint accepts dispatch_strategy: "nearest" | "batched"
# GET /metrics returns metrics tagged with current strategy
# Compare in dashboard: overlay two metric sets
```

---

## 5. Git Workflow

### 5.1: Branch Strategy

```
main              ← stable, always deployable on both sides
├── phase-1       ← Phase 1 work (merged to main when validated)
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
cd ~/warehouse-sim
git add -A
git commit -m "warehouse-manager: add batch dispatcher"
git push

# Rebuild local services
docker compose -f local-compose.yml up --build -d warehouse-manager

# For sim-scripts or vehicle-controller changes:
# SSH into Brev:
ssh brev-warehouse
cd ~/warehouse-sim
git pull
docker compose -f brev-compose.yml up --build -d vehicle-controller

# For warehouse_msgs changes (BOTH SIDES):
# 1. Commit and push from wherever you edited
git push
# 2. On Brev:
ssh brev-warehouse
cd ~/warehouse-sim && git pull
docker compose -f brev-compose.yml up --build
# 3. On local:
cd ~/warehouse-sim && git pull
docker compose -f local-compose.yml up --build
```

### 5.3: .gitignore

```gitignore
# Environment
.env
fastdds_client_resolved.xml

# Data
*.db
*.db-wal
*.db-shm

# Docker
isaac-sim-cache/

# Python
__pycache__/
*.pyc
.venv/

# ROS 2 build artifacts
ros2_ws/build/
ros2_ws/install/
ros2_ws/log/

# Node
dashboard/node_modules/
dashboard/dist/

# IDE
.vscode/
.idea/
```

---

## Troubleshooting

### Docker Desktop WSL2 — `network_mode: host` issues

```bash
# Symptom: Container can't reach Tailscale IPs
# Cause: Docker Desktop WSL2 backend uses its own network namespace

# Fix option 1: Use native Docker in WSL2 (uninstall Docker Desktop)
sudo apt-get install -y docker.io
sudo usermod -aG docker $USER

# Fix option 2: Use Docker Desktop but don't use network_mode: host
# Instead, map specific ports and use explicit container-to-container networking
# This requires configuring FastDDS with specific unicast locators and port ranges
```

### ROS 2 topics not visible from local

```bash
# Step 1: Verify Tailscale
ping <brev-tailscale-ip>

# Step 2: Verify discovery server port
nc -zv <brev-tailscale-ip> 11811

# Step 3: Check environment variables
echo $ROS_DOMAIN_ID        # Must be 42
echo $ROS_DISCOVERY_SERVER  # Must be <brev-ip>:11811
echo $RMW_IMPLEMENTATION    # Must be rmw_fastrtps_cpp

# Step 4: Check FastDDS XML is resolved (no ${BREV_TAILSCALE_IP} literal)
cat /tmp/fastdds_client_resolved.xml | grep address

# Step 5: Restart ROS 2 daemon
ros2 daemon stop && ros2 daemon start

# Step 6: Check if DDS traffic is blocked
# Tailscale should allow all traffic, but if not:
sudo tailscale up --accept-routes
```

### WebSocket connection fails from Windows browser

```bash
# Symptom: Dashboard WebSocket shows "disconnected" permanently
# Cause: WSL2 port forwarding not working

# Check if port 8000 is accessible from Windows:
# From PowerShell:
Test-NetConnection -ComputerName localhost -Port 8000

# If fails, WSL2 is not forwarding ports. Fix:
# Option 1: Docker Desktop auto-forwards published ports
# Option 2: Manual port forward:
netsh interface portproxy add v4tov4 listenaddress=0.0.0.0 listenport=8000 connectaddress=$(wsl hostname -I | awk '{print $1}') connectport=8000
```

### SQLite "database is locked" errors

```bash
# Symptom: warehouse-manager logs show SQLITE_BUSY
# Cause: Multiple threads writing simultaneously without WAL mode

# Fix: Ensure WAL mode is enabled (check models.py):
# conn.execute("PRAGMA journal_mode=WAL")
# conn.execute("PRAGMA busy_timeout=5000")

# Also: use a single connection pool, not per-request connections
```

### Dashboard shows stale data after Brev restart

```bash
# Symptom: Forklifts show last known position, not updating
# Cause: WebSocket connected but ROS 2 bridge lost

# Fix: Check warehouse-manager logs
docker compose -f local-compose.yml logs --tail=100 warehouse-manager | grep -i "ros\|dds\|discovery"

# If ROS 2 node lost discovery:
docker compose -f local-compose.yml restart warehouse-manager
```

### Redis connection refused

```bash
# Check Redis is running
docker compose -f local-compose.yml ps redis

# Check Redis is healthy
docker compose -f local-compose.yml exec redis redis-cli ping
# Must return PONG

# If warehouse-manager can't reach Redis:
# With network_mode: host, warehouse-manager reaches Redis at localhost:6379
# Verify the port is published:
docker compose -f local-compose.yml port redis 6379
```
