.PHONY: init check-ports dev dash stop

# Ports required for streaming
STREAMING_PORTS := 49100 47998 8080

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
	@echo "==> Pulling Isaac Sim Docker image (skip if already present)..."
	@docker pull nvcr.io/nvidia/isaac-sim:5.1.0
	@echo "==> Installing Python dependencies..."
	@pip install --quiet fastapi uvicorn
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

## Start the forklift dashboard on port 8080
dash:
	@cd simulations/forklift-warehouse/03_dashboard && python3 -m uvicorn dashboard:app --host 0.0.0.0 --port 8080

## Stop the Isaac Sim container
stop:
	@echo "Stopping isaac-sim container..."
	@docker stop isaac-sim 2>/dev/null || true
	@docker rm -f isaac-sim 2>/dev/null || true
	@echo "isaac-sim container stopped."
