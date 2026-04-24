# Copilot Instructions — nvidia-simu-test

## Project purpose
Automate provisioning, setup, and operation of an NVIDIA Isaac Sim GPU instance 
(Ubuntu / Docker / NVIDIA A4000) via brev.dev. The goal is to run Isaac Sim with 
WebRTC streaming and execute Isaac Sim standalone Python scripts (Jetbot tutorial).

---

## Git: never commit or push without permission

Never run `git commit`, `git push`, or any other git write operation (e.g. `git tag`, 
`git reset`, `git revert`) without explicit permission from the user. Staging files 
with `git add` is allowed for review, but the actual commit and push must be 
confirmed by the human first.

---

## Golden rule: never touch the GPU instance directly

All work happens locally. Scripts are **authored and edited on the Mac**, then 
**deployed to the instance** via deployment script. The user runs deployed scripts manually 
inside `brev shell`. Never run commands against the instance in a terminal — never 
`ssh` to it to execute anything.

Deploy workflow:
```
# 1. Edit scripts locally in scripts/instance/
# 2. Run the deployer:
export ISAAC_SIM_IP=<instance-public-ip>
bash scripts/deploy_to_instance.sh
# 3. User runs in brev shell:
# brev shell isaac-sim-2
# sudo bash ~/setup.sh
```

---

## Environment variables (must be set before running Mac-side scripts)

| Variable | Purpose |
|---|---|
| `ISAAC_SIM_IP` | Public IP of the current GPU instance — **never hardcode this in scripts** |

Set it in your shell before running anything:
```bash
export ISAAC_SIM_IP=94.101.98.52   # example — update when instance changes
```

---

## Secrets and credentials

- **NGC key** is stored at `~/.ngc/key` (chmod 600), never in the repo.
- `scripts/instance/setup.sh` contains the literal string `NGC_KEY_PLACEHOLDER`.
- `scripts/deploy_to_instance.sh` reads `~/.ngc/key` and uses `sed` to substitute 
  the placeholder before uploading — the patched copy lives only in `/tmp`.
- `scripts/provision/startup.sh` is gitignored (it also contains the NGC key).
- The `.gitignore` excludes `*.key`, `*.pem`, `scripts/provision/startup.sh`.

---

## SSH / brev quirks

- **SSH key**: always `~/.brev/brev.pem`. Never use `~/.ssh/id_ed25519` for this instance.
- **brev SSH config** has `RequestTTY yes` + `ControlMaster auto`. Non-interactive 
  `ssh user@host "cmd"` **hangs**. Always use:
  - `scp -o ControlPath=none` for file transfers
  - `ssh -o ControlPath=none` when a non-interactive SSH call is genuinely needed
- **brev ls** requires a confirmation prompt bypass: `yes | brev ls`
- **brev shell** opens an interactive session — that's fine and expected.

---

## Instance details

- **Name**: `isaac-sim-2` (brev alias)
- **User**: `shadeform`
- **Type**: `hyperstack_A4000` (NVIDIA A4000 16 GiB, 4 vCPU, Oslo, Norway)
- **Docker image**: `nvcr.io/nvidia/isaac-sim:5.1.0`
- **Firewall (UFW)**: 22/tcp, 2222/tcp (SSH), 49100/tcp, 47998/udp (Isaac Sim streaming)

---

## Writing scripts and files

- Steps that require `root` on the instance (apt-get, systemctl, nvidia-ctk, ufw) 
  must be run with `sudo bash ~/setup.sh`, not plain `bash`.
- `set -euo pipefail` on all bash scripts.
- All Mac-side scripts accept instance IP exclusively via `$ISAAC_SIM_IP` — validate 
  it at the top of the script and exit with a clear error if unset.

---

## Launch sequence (reference)

```bash
# Mac — after instance is ready:
export ISAAC_SIM_IP=<ip>
bash scripts/deploy_to_instance.sh

# Instance — inside brev shell:
sudo bash ~/setup.sh          # installs Docker, NVIDIA CTK, pulls image (~10 min first run)

# Instance — launch streaming:
cd ~/isaacsim-docker
PUBLIC_IP=$(curl -s ifconfig.me)
ISAACSIM_HOST=$PUBLIC_IP ISAAC_SIM_IMAGE=nvcr.io/nvidia/isaac-sim:5.1.0 \
  docker compose -p isim -f tools/docker/docker-compose.yml up --build -d

# Mac — open web viewer:
bash scripts/launch_streaming.sh
# Opens tunnel on localhost:8210, polls readiness, opens Chrome
```
