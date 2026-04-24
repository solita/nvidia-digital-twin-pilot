#!/usr/bin/env bash
# setup.sh — Run on the instance after brev shell: bash ~/setup.sh
# Installs Docker + NVIDIA Container Toolkit, logs into NGC,
# checks out tools/docker, and pulls Isaac Sim image.
set -euo pipefail

ISAAC_SIM_IMAGE="nvcr.io/nvidia/isaac-sim:5.1.0"
NGC_KEY="NGC_KEY_PLACEHOLDER"
# Always use the shadeform home dir — $HOME becomes /root under sudo bash
USER_HOME="/home/shadeform"
LOG="$USER_HOME/setup.log"
exec > >(tee -a "$LOG") 2>&1

echo "[$(date)] setup.sh starting..."

# --- 1. Install Docker (if missing) ---
if ! command -v docker &>/dev/null; then
    echo "[$(date)] Installing Docker..."
    apt-get update -qq
    apt-get install -y ca-certificates curl gnupg lsb-release
    install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
        | gpg --dearmor -o /etc/apt/keyrings/docker.gpg
    chmod a+r /etc/apt/keyrings/docker.gpg
    echo \
        "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
        https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" \
        > /etc/apt/sources.list.d/docker.list
    apt-get update -qq
    apt-get install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin
    usermod -aG docker shadeform
    systemctl enable --now docker
    echo "[$(date)] Docker installed."
else
    echo "[$(date)] Docker already installed: $(docker --version)"
fi

# --- 2. Install NVIDIA Container Toolkit (if missing) ---
if ! dpkg -l nvidia-container-toolkit &>/dev/null; then
    echo "[$(date)] Installing NVIDIA Container Toolkit..."
    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
        | gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
    curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
        | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
        > /etc/apt/sources.list.d/nvidia-container-toolkit.list
    apt-get update -qq
    apt-get install -y nvidia-container-toolkit
    nvidia-ctk runtime configure --runtime=docker
    systemctl restart docker
    echo "[$(date)] NVIDIA Container Toolkit installed."
else
    echo "[$(date)] NVIDIA Container Toolkit already installed."
fi

# --- 3. Cache dir ownership ---
mkdir -p "$USER_HOME/docker/isaac-sim"
chown -R 1234:1234 "$USER_HOME/docker/isaac-sim"

# --- 4. NGC login ---
echo "[$(date)] Logging in to nvcr.io..."
echo "$NGC_KEY" | docker login nvcr.io --username '$oauthtoken' --password-stdin

# --- 5. Sparse-checkout tools/docker ---
if [[ ! -d "$USER_HOME/isaacsim-docker/tools/docker" ]]; then
    echo "[$(date)] Sparse-checkout tools/docker from IsaacSim..."
    mkdir -p "$USER_HOME/isaacsim-docker" && cd "$USER_HOME/isaacsim-docker"
    git init
    git remote add origin https://github.com/isaac-sim/IsaacSim.git
    git fetch --depth 1 origin develop
    git checkout FETCH_HEAD -- tools/docker
else
    echo "[$(date)] tools/docker already present."
fi

# --- 6. Pull Isaac Sim image (~10 min) ---
echo "[$(date)] Pulling $ISAAC_SIM_IMAGE ..."
docker pull "$ISAAC_SIM_IMAGE"

PUBLIC_IP=$(curl -s ifconfig.me)
echo ""
echo "[$(date)] Done. Launch Isaac Sim streaming:"
echo ""
echo "  cd ~/isaacsim-docker"
echo "  ISAACSIM_HOST=$PUBLIC_IP ISAAC_SIM_IMAGE=$ISAAC_SIM_IMAGE \\"
echo "    docker compose -p isim -f tools/docker/docker-compose.yml up --build -d"
