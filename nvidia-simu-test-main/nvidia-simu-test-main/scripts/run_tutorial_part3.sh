#!/usr/bin/env bash
# run_tutorial_part3.sh — Deploy Part 3 scripts to the instance via scp,
# then print the command to run them inside brev shell.
# Usage: bash scripts/run_tutorial_part3.sh
set -euo pipefail

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTANCE_SCRIPTS_DIR="~/scripts/tutorial"

echo "=== Part 3: Deploying standalone Jetbot scripts to instance ==="
echo ""

# scp does not require TTY — works regardless of brev's RequestTTY yes setting
echo "[1/2] Copying scripts to instance..."
# Create the target directory first using sftp (no TTY needed)
echo "mkdir -p ~/scripts/tutorial" | sftp -q isaac-sim

scp "$SCRIPTS_DIR/instance/standalone_robot.py" \
    "$SCRIPTS_DIR/instance/run_standalone.sh" \
    "$SCRIPTS_DIR/instance/check_status_instance.sh" \
    "isaac-sim:$INSTANCE_SCRIPTS_DIR/"

echo "[2/2] Setting executable bit..."
# Write a tiny heredoc to a temp file and scp it as a setup script, then
# the user runs it — avoids ssh non-interactive TTY issue entirely.
cat > /tmp/isim_setup_perms.sh << 'EOF'
#!/usr/bin/env bash
chmod +x ~/scripts/tutorial/run_standalone.sh
chmod +x ~/scripts/tutorial/check_status_instance.sh
echo "Permissions set."
EOF
scp /tmp/isim_setup_perms.sh "isaac-sim:~/scripts/tutorial/setup_perms.sh"

echo ""
echo "=== Scripts deployed. Now open brev shell and run them: ==="
echo ""
echo "  brev shell isaac-sim"
echo ""
echo "  # Inside the instance:"
echo "  bash ~/scripts/tutorial/setup_perms.sh"
echo "  bash ~/scripts/tutorial/check_status_instance.sh   # optional: check GPU/containers"
echo "  bash ~/scripts/tutorial/run_standalone.sh           # run the Jetbot simulation"
echo ""
