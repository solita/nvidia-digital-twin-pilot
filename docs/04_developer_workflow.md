# Guide 04 — Developer Workflow

This guide covers how 2–3 developers collaborate on this repo: branching strategy, starting a new simulation from the template, deploying to Brev for testing, and committing changes back.

---

## Prerequisites

- Git installed locally
- Brev instance running with Isaac Sim (see [Guide 01](./01_brev_setup.md))
- VS Code connected to Brev (see [Guide 02](./02_vscode_access.md))
- Smoke test passing (see [Guide 03](./03_vscode_isaacsim_smoke_test.md))

---

## Branching Strategy

This repo uses a **feature-branch workflow**:

```
main                    ← stable, working state — never push directly
└── feature/<topic>     ← all development work
```

### Rules

| Rule | Reason |
|---|---|
| Never commit directly to `main` | Keeps `main` always in a runnable state for other devs |
| One branch per feature/simulation/experiment | Keeps PRs small and reviewable |
| Branch from the latest `main` | Avoids merging stale changes |
| Open a Pull Request to merge back | Gives the team a chance to review before `main` changes |

### Example branch names

```
feature/pipe-inspection-sim
feature/ground-support-scene
feature/update-cave-textures
fix/launcher-timeout
```

## Starting a New Simulation

### Step 1 — Copy the template

```bash
# From the repo root
cp -r simulations/_template simulations/<your_sim_name>
```

Use a short, lowercase, hyphenated name — e.g. `pipe-inspection`, `ground-support`, `rock-bolting`.

### Step 2 — Rename the launcher and run script

Inside `simulations/<your_sim_name>/02_core_scripts/`:

```bash
mv launcher.py <your_sim_name>_launcher_v1.py
```

Update `LAUNCHER_SCRIPT` at the top of `run_sim.sh` to match the new filename.

### Step 3 — Update the README

Open `simulations/<your_sim_name>/README.md` and replace all the placeholder text:
- Title and description
- Folder structure comment
- Expected outputs section

### Step 4 — Add your scene and scripts

- Place USD scene files in `01_scenes/`
- Place Isaac Sim Python logic in `02_core_scripts/`
- Update `SCENE_FILE` and `MAIN_SCRIPT` in your launcher to point to them

### Step 5 — Commit the skeleton before adding assets

```bash
git add simulations/<your_sim_name>/
git commit -m "feat: add <your_sim_name> simulation scaffold"
```

Committing the empty scaffold first means the folder structure is in version history before any large assets are added, making git history easier to follow.

---

## Deploying to Brev for Testing

Once you have scene files and scripts ready locally, deploy them to the Brev machine.

### Option A — rsync (recommended for iterative development)

`rsync` only transfers changed files, making it fast for script edits:

```bash
rsync -avz --progress \
  simulations/<your_sim_name>/ \
  ubuntu@<BREV_PUBLIC_IP>:/home/ubuntu/docker/isaac-sim/data/<your_sim_name>/
```

Run this from your local machine. Add `--exclude '__pycache__'` to skip Python cache files.

### Option B — Full copy (first-time or full reset)

From inside the Brev machine (after `brev shell` or `ssh isaac-sim-pilot`):

```bash
# If the repo is cloned on Brev — just pull and copy
git pull origin feature/your-branch

cp -r ~/path/to/simulations/<your_sim_name> \
      /home/ubuntu/docker/isaac-sim/data/
```

### Fix permissions after deploying

Always run this so Isaac Sim (UID 1234) and VS Code (`ubuntu`) both have access:

```bash
sudo chown -R 1234:1234 /home/ubuntu/docker/isaac-sim/data/<your_sim_name>
sudo chmod -R o+rwx /home/ubuntu/docker/isaac-sim/data/<your_sim_name>
```

---

## Iterating on Scripts via VS Code

Because VS Code is connected to the Brev file system (see [Guide 02](./02_vscode_access.md)), you can edit scripts directly on the remote and run them immediately with **Isaac Sim: Run Remotely** — no re-deploy needed.

**Workflow for fast iteration:**

1. Edit a script in VS Code (connected to Brev remote)
2. Press `Ctrl+Shift+P` → **Isaac Sim: Run Remotely**
3. Check output in `Ctrl+Shift+U` → "Isaac Sim"
4. Iterate
5. When happy, copy the file back to your local repo and commit:

```bash
# From local machine
rsync -avz \
  ubuntu@<BREV_PUBLIC_IP>:/home/ubuntu/docker/isaac-sim/data/<your_sim_name>/02_core_scripts/ \
  simulations/<your_sim_name>/02_core_scripts/

git add simulations/<your_sim_name>/02_core_scripts/
git commit -m "feat: update launcher with tested changes"
```

---

## Saving Reference Milestones

When a run produces a result worth keeping (a good parameter set, a baseline output, etc.), save it under `05_reference_milestones/` with a descriptive subfolder name:

```
05_reference_milestones/
└── A_baseline_first_run/
    ├── run_output.json
    └── run_output.txt
```

These are tracked by git (unlike `04_current_outputs/` which is gitignored), so they act as a shared, versioned record of significant results.

---

## Pull Request Checklist

Before opening a PR to `main`:

- [ ] Simulation runs on Brev without errors
- [ ] `README.md` in the simulation folder is filled in (no placeholder text remains)
- [ ] No new binary assets added without discussion (keep repo size in check)
- [ ] `04_current_outputs/` is empty / only contains `.gitkeep`
- [ ] `__pycache__/` is not staged

---

**Back to:** [Repository README](../README.md)
