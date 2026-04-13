# NVIDIA Digital Twin Pilot

**BuDev · Emerging Business Area · NVIDIA Isaac Sim on Brev**

A proof-of-concept repository for running NVIDIA Isaac Sim simulations on a cloud GPU (Brev / AWS) and developing against it from VS Code.

---

## Guides

| # | Guide | Description |
|---|---|---|
| 01 | [Hosting Isaac Sim on Brev (AWS / Sweden)](docs/01_brev_setup.md) | Provision an L40S GPU on Brev, pull the Isaac Sim Docker container, and launch in headless WebRTC mode |
| 02 | [VS Code Workspace with Read/Write Access](docs/02_vscode_access.md) | Connect VS Code via SSH and fix UID 1234 permission issues |
| 03 | [VS Code ↔ Isaac Sim Connection + Smoke Test](docs/03_vscode_isaacsim_smoke_test.md) | Install the Isaac Sim VS Code extension, configure it for remote execution, and verify with the smoke test |
| 04 | [Developer Workflow](docs/04_developer_workflow.md) | Branching strategy, starting a new simulation, deploying to Brev, iterating on scripts |

---

## Simulations

| Simulation | Description |
|---|---|
| [Concrete Spray (Shotcrete)](simulations/concrete_spray/README.md) | Real-time digital twin of a shotcrete operation in a mining cave — USD particle simulation driven by a microburst proxy data pipeline |

> To add a new simulation, copy `simulations/_template/` and follow [Guide 04](docs/04_developer_workflow.md).

---

## Repository Structure

```
nvidia-digital-twin-pilot/
├── docs/
│   ├── 01_brev_setup.md
│   ├── 02_vscode_access.md
│   ├── 03_vscode_isaacsim_smoke_test.md
│   └── 04_developer_workflow.md
├── smoke_test/
│   └── vscode_smoke_test.py
├── simulations/
│   ├── _template/               ← copy this to start a new simulation
│   │   ├── README.md
│   │   ├── 01_scenes/
│   │   ├── 02_core_scripts/
│   │   │   └── launcher.py      ← rename and update for your sim
│   │   ├── 03_preview_donor_scripts/
│   │   ├── 04_current_outputs/  ← gitignored (generated at runtime)
│   │   ├── 05_reference_milestones/
│   │   ├── 06_master_handoff/
│   │   └── run_sim.sh
│   └── concrete_spray/          ← example simulation
│       ├── README.md
│       ├── 01_scenes/
│       ├── 02_core_scripts/
│       ├── 03_preview_donor_scripts/
│       ├── 04_current_outputs/  ← gitignored (generated at runtime)
│       ├── 05_reference_milestones/
│       ├── 06_master_handoff/
│       ├── run_direct_cave_preview_v1.sh
│       └── run_direct_cave_preview_validate_v1.sh
└── .gitignore
```

---

## Quick Start

1. Follow [Guide 01](docs/01_brev_setup.md) to provision Brev and start Isaac Sim.
2. Follow [Guide 02](docs/02_vscode_access.md) to connect VS Code with full file-system access.
3. Follow [Guide 03](docs/03_vscode_isaacsim_smoke_test.md) to verify the VS Code ↔ Isaac Sim link.
4. Deploy and run the [Concrete Spray simulation](simulations/concrete_spray/README.md) as a first real test.
5. Read [Guide 04](docs/04_developer_workflow.md) before starting new development work.
