.PHONY: init check-ports dev dash dash-restart stop help
.DEFAULT_GOAL := help

# Ports required for streaming
STREAMING_PORTS := 49100 47998 8080

## Show available commands
help:
	@awk '/^## /{desc=substr($$0,4)} /^[a-zA-Z_-]+:/{if(desc){gsub(/:/,"",$$1); printf "  \033[36m%-20s\033[0m %s\n", $$1, desc; desc=""}}' $(MAKEFILE_LIST)

## One-time project initialization — creates dirs, pulls image, installs deps, checks ports
init:
	@echo "==> Creating Isaac Sim volume-mount directories..."
	@mkdir -p ~/docker/isaac-sim/cache/main/ov
	@mkdir -p ~/docker/isaac-sim/cache/main/warp
	@mkdir -p ~/docker/isaac-sim/cache/computecache
	@mkdir -p ~/docker/isaac-sim/config
	@mkdir -p ~/docker/isaac-sim/data/documents
	@mkdir -p ~/docker/isaac-sim/data/Kit
	@mkdir -p ~/docker/isaac-sim/logs
	@mkdir -p ~/docker/isaac-sim/pkg
	@echo "==> Setting ownership (requires sudo)..."
	@sudo chown -R 1234:1234 ~/docker/isaac-sim
	@echo "==> Restoring .git ownership for host user..."
	@if [ -d ~/docker/isaac-sim/data/nvidia-digital-twin-pilot/.git ]; then \
		sudo chown -R $$(id -u):$$(id -g) ~/docker/isaac-sim/data/nvidia-digital-twin-pilot/.git; \
	fi
	@echo "==> Pulling Isaac Sim Docker image (skip if already present)..."
	@docker pull nvcr.io/nvidia/isaac-sim:5.1.0
	@echo "==> Installing Python dependencies..."
	@pip install --quiet fastapi uvicorn
	@echo "==> Installing Isaac Sim VS Code Edition extension..."
	@code --install-extension nvidia.isaacsim-vscode-edition --force
	@echo "==> Enabling isaacsim.code_editor.vscode extension in Isaac Sim (autoload)..."
	@for profile in "Isaac-Sim Streaming" "Isaac-Sim Full"; do \
		CFG_DIR=~/docker/isaac-sim/data/Kit/"$$profile"/5.1; \
		mkdir -p "$$CFG_DIR"; \
		CFG="$$CFG_DIR/user.config.json"; \
		if [ -f "$$CFG" ]; then \
			python3 -c " \
import json, sys; \
cfg=json.load(open('$$CFG')); \
p=cfg.setdefault('persistent',{}); \
app=p.setdefault('app',{}); \
exts=app.setdefault('exts',{}); \
enabled=exts.setdefault('enabled',{}); \
ext_id='isaacsim.code_editor.vscode-1.1.0'; \
if ext_id not in enabled.values(): \
    enabled[str(len(enabled))]=ext_id; \
json.dump(cfg,open('$$CFG','w'),indent=2)"; \
		else \
			echo '{\"persistent\":{\"app\":{\"exts\":{\"enabled\":{\"0\":\"isaacsim.code_editor.vscode-1.1.0\"}}}}}' | python3 -m json.tool > "$$CFG"; \
		fi; \
	done
	@sudo chown -R 1234:1234 ~/docker/isaac-sim/data/Kit
	@echo "==> Checking streaming ports..."
	@$(MAKE) --no-print-directory check-ports
	@echo "==> Done. Run 'make dev' to start Isaac Sim."

## Verify required streaming ports are reachable from the internet
check-ports:
	@PUBLIC_IP=$$(curl -s ifconfig.me) && \
	echo "Public IP: $$PUBLIC_IP" && \
	echo "Checking ports: $(STREAMING_PORTS)" && \
	ALL_OK=true && \
	for port in $(STREAMING_PORTS); do \
		if curl -s --max-time 5 "https://portchecker.io/api/v1/query" \
			-H "Content-Type: application/json" \
			-d "{\"host\":\"$$PUBLIC_IP\",\"ports\":[$$port]}" 2>/dev/null \
			| grep -q '"status":true' 2>/dev/null; then \
			echo "  ✓ Port $$port — open"; \
		else \
			echo "  ✗ Port $$port — closed or filtered"; \
			ALL_OK=false; \
		fi; \
	done && \
	if [ "$$ALL_OK" = "false" ]; then \
		echo "" && \
		echo "╔══════════════════════════════════════════════════════════════╗" && \
		echo "║  ACTION REQUIRED — Open ports in the Brev dashboard        ║" && \
		echo "╠══════════════════════════════════════════════════════════════╣" && \
		echo "║  1. Go to https://developer.nvidia.com/brev                ║" && \
		echo "║  2. Open your instance → Exposed Ports                     ║" && \
		echo "║  3. Add these ports (scope to your IP for security):       ║" && \
		echo "║       49100  — WebRTC signalling                           ║" && \
		echo "║       47998  — WebRTC media stream                         ║" && \
		echo "║       8080   — Forklift dashboard                          ║" && \
		echo "║  4. Re-run:  make check-ports                              ║" && \
		echo "╚══════════════════════════════════════════════════════════════╝" && \
		echo ""; \
	else \
		echo "All streaming ports are open."; \
	fi

## Start Isaac Sim container with WebRTC streaming on port 49100
dev:
	@docker stop isaac-sim 2>/dev/null || true
	@docker rm -f isaac-sim 2>/dev/null || true
	@PUBLIC_IP=$$(curl -s ifconfig.me) && \
	echo "Starting Isaac Sim — WebRTC at $$PUBLIC_IP:49100" && \
	docker run --name isaac-sim \
		--entrypoint bash \
		-it --gpus all \
		-e "ACCEPT_EULA=Y" \
		-e "PRIVACY_CONSENT=Y" \
		-e "ROS_DISTRO=jazzy" \
		-e "RMW_IMPLEMENTATION=rmw_fastrtps_cpp" \
		--rm --network=host \
		-v ~/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw \
		-v ~/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw \
		-v ~/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw \
		-v ~/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw \
		-v ~/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw \
		-v ~/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw \
		-u 1234:1234 \
		nvcr.io/nvidia/isaac-sim:5.1.0 \
		-lc "export LD_LIBRARY_PATH=\$$LD_LIBRARY_PATH:/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib && ./runheadless.sh --/app/livestream/publicEndpointAddress=$$PUBLIC_IP --/app/livestream/port=49100"

## Start the forklift dashboard on port 8080 (auto-reloads on file changes)
dash:
	@cd simulations/forklift-warehouse/03_dashboard && python3 -m uvicorn dashboard:app --host 0.0.0.0 --port 8080 --reload

## Kill running dashboard, clear __pycache__, and restart fresh
dash-restart:
	@echo "==> Stopping existing dashboard on port 8080..."
	@fuser -k 8080/tcp 2>/dev/null || true
	@echo "==> Clearing Python cache..."
	@find simulations/forklift-warehouse/03_dashboard -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
	@echo "==> Starting dashboard with hot-reload..."
	@cd simulations/forklift-warehouse/03_dashboard && python3 -m uvicorn dashboard:app --host 0.0.0.0 --port 8080 --reload

## Stop the Isaac Sim container
stop:
	@echo "Stopping isaac-sim container..."
	@docker stop isaac-sim 2>/dev/null || true
	@docker rm -f isaac-sim 2>/dev/null || true
	@echo "isaac-sim container stopped."
