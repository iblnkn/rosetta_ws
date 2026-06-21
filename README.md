# rosetta_ws

A ready-to-use devcontainer workspace for **[Rosetta](src/action/rosetta/README.md)**, the ROS2 to LeRobot bridge.

## Why This Workspace?

Getting ROS2 and LeRobot installed together is not trivial. This workspace is a working example, so you can clone it, open it in VS Code, and get started.

Based on Allison Thackston's excellent [ROS2 VS Code devcontainer template](https://github.com/athackst/vscode_ros2_workspace) ([blog post](https://www.allisonthackston.com/articles/vscode-docker-ros2.html)).

## Quick Start

```bash
git clone https://github.com/iblnkn/rosetta_ws.git
cd rosetta_ws
code .  # VS Code: "Reopen in Container"
```

The devcontainer:
- Installs ROS2, LeRobot, and all dependencies
- Builds and sources the workspace
- Mounts credentials from host (HuggingFace, W&B, SSH)
- Configures GPU access (NVIDIA runtime)

## Policy Backends & Environments

One contract drives **LeRobot**, **TRI vla_foundry**, or **starVLA**. The
frameworks pin incompatible `torch` versions, so the ROS-side backends live in
their own pixi environments (sharing the ROS 2 Jazzy base). starVLA's model is
hosted out-of-process, so its leaf stays in `default`:

| Environment | Stack | Use |
|---|---|---|
| `default` | ROS 2 + **LeRobot** (torch 2.10) + **starVLA** client leaf | `pixi run ...` |
| `vla` | ROS 2 + **vla_foundry** (torch 2.7) | `pixi run -e vla ...` |
| `starvla` | **starVLA** policy server only (torch 2.6, Linux/CUDA, no ROS) | `pixi run -e starvla ...` |

The LeRobot/vla_foundry environments build their own colcon overlay
(`install/` vs `install_vla/`):

```bash
pixi run build           # default overlay -> install/ (lerobot + starvla leaves)
pixi run -e vla build-vla # vla overlay -> install_vla/
```

The backend is selected by name. Convert bags to a dataset:

```bash
pixi run        rosetta_port --backend lerobot --raw-dir ... --contract ... --repo-id ...
pixi run -e vla rosetta_port --backend vla     --raw-dir ... --contract ... --repo-id ...
pixi run        rosetta_port --backend starvla --raw-dir ... --contract ... --repo-id ...
```

Deploy a policy (the client node's `backend` param picks the runner):

```bash
pixi run        ros2 launch rosetta rosetta_client_launch.py backend:=lerobot ...
pixi run -e vla ros2 launch rosetta rosetta_client_launch.py backend:=vla \
                    pretrained_name_or_path:=/path/to/vla_foundry/checkpoint
```

**starVLA** is split. The dataset writer and the deploy client run in `default`,
but the model is served by starVLA's own websocket server in the `starvla` env
(Linux/CUDA). The `starvla` writer emits a LeRobot v3.0 dataset plus a GR00T
`meta/modality.json` and a paste-ready `rosetta_dataconfig.py`. Training runs
from the starVLA repo itself (`libs/starvla`). To deploy, start the server on a
GPU host, then launch the ROS client against it:

```bash
# one-time, on the GPU host:
pixi run -e starvla setup-starvla       # installs starVLA's requirements + editable pkg
#   then install a matching CUDA torch (see pixi.toml [feature.starvla] comment)
# Rosetta's wrapper normalizes input state server-side (no train/deploy skew):
PYTHONPATH=libs/starvla pixi run -e starvla python scripts/rosetta_starvla_server.py \
                    --ckpt_path <checkpoint.pt> --port 10093 --use_bf16
#   (vision+language-only model? starVLA's stock deployment/model_server/server_policy.py also works.)

# on the robot:
pixi run ros2 launch rosetta rosetta_client_launch.py backend:=starvla \
                    server_host:=<gpu-host> server_port:=10093
```

Backends register under the `rosetta.dataset_writers` / `rosetta.policy_runners`
entry-point groups; `rosetta` core imports neither framework. Both names appear
in each environment, but only the env-matching backend is importable.

## Workspace Structure

```
rosetta_ws/
├── src/action/              # ROS2 packages (from repos/src.repos)
│   ├── rosetta/             # Core package - see rosetta/README.md
│   ├── rosetta_interfaces/  # ROS2 action/service definitions
│   ├── lerobot_robot_rosetta/      # LeRobot backend leaf (default env)
│   ├── lerobot_teleoperator_rosetta/
│   ├── vla_foundry_rosetta/        # vla_foundry backend leaf (vla env)
│   ├── starvla_rosetta/            # starVLA backend leaf (default env; ws client + writer)
│   └── rosetta_rl/
├── libs/                    # Python libraries (from repos/libs.repos)
│   ├── lerobot/             # LeRobot (editable, default env)
│   ├── vla_foundry/         # TRI vla_foundry (editable, vla env)
│   └── starvla/             # starVLA (editable, starvla env — server only)
├── models/                  # Trained policies
│   ├── act/
│   └── pi05/
├── datasets/
│   ├── bags/                # Raw rosbag recordings
│   └── lerobot/             # Converted LeRobot datasets
├── repos/                   # VCS import definitions
│   ├── src.repos            # ROS2 package sources
│   └── libs.repos           # Python library sources
├── scripts/                 # Build and workflow scripts
├── docker/                  # Dockerfiles (x86, Jetson)
└── .devcontainer/           # VS Code devcontainer configs
    ├── x86/
    └── jetson/
```

## Directories

### `src/action/`

ROS2 packages. Currently embedded in the workspace for development. In production, these will be pulled from separate repositories via `repos/src.repos`:

```yaml
# repos/src.repos (not yet active)
repositories:
  action/rosetta:
    type: git
    url: https://github.com/iblnkn/rosetta.git
    version: main
```

For documentation on the packages themselves, see [rosetta/README.md](src/action/rosetta/README.md).

### `libs/`

Python libraries installed in editable mode. LeRobot is cloned here so you can modify it if needed.

```yaml
# repos/libs.repos
repositories:
  lerobot:
    type: git
    url: https://github.com/huggingface/lerobot.git
    version: main
```

The `COLCON_IGNORE` file prevents colcon from treating this as a ROS2 package.

### `models/`

Store trained policies here. Organized by policy type:

```
models/
├── act/
│   └── act_pen_in_cup/
│       └── 050000/
│           └── pretrained_model/
└── pi05/
    └── my_pi05_policy/
```

Reference in launch files:
```bash
ros2 launch rosetta rosetta_client_launch.py \
    pretrained_name_or_path:=/workspaces/rosetta_ws/models/act/my_policy/pretrained_model
```

Or upload to HuggingFace Hub and reference by repo ID.

### `datasets/`

Two subdirectories for the recording → training pipeline:

| Directory | Contents | Created By |
|-----------|----------|------------|
| `datasets/bags/` | Raw rosbag2 recordings (MCAP) | Episode Recorder |
| `datasets/lerobot/` | Converted LeRobot datasets | `port_bags.py` |

**Workflow:**
1. Record episodes → `datasets/bags/my_robot_YYYYMMDD_HHMMSS/`
2. Convert → `datasets/lerobot/my_dataset/`
3. Train → `models/act/my_policy/`

### `repos/`

VCS import files for `vcs import`. To pull external sources:

```bash
vcs import src < repos/src.repos
vcs import libs < repos/libs.repos
```

Currently commented out since packages are embedded for development.

### `scripts/`

Workflow automation scripts:

| Script | Description |
|--------|-------------|
| `build.sh` | Build workspace with colcon |
| `setup.sh` | First-time setup (rosdep, pip installs) |
| `convert_bags_parallel.sh` | Parallel bag-to-dataset conversion |
| `train_policy.sh` | Training with policy selection and multi-GPU |
| `test.sh` | Run tests |

## Docker

Two Dockerfiles for different platforms:

| File | Platform | Base Image |
|------|----------|------------|
| `docker/Dockerfile.x86` | x86_64 with NVIDIA GPU | `nvidia/cuda:12.x` + ROS2 |
| `docker/Dockerfile.jetson` | NVIDIA Jetson (ARM64) | `dustynv/ros:jazzy-...` |

### Devcontainer (Recommended)

Open in VS Code and select "Reopen in Container". The devcontainer:
- Uses the appropriate Dockerfile for your platform
- Mounts the workspace at `/workspaces/rosetta_ws`
- Shares credentials from host (`~/.cache/huggingface`, `~/.config/wandb`, `~/.ssh`)
- Enables GPU passthrough

## VS Code Tasks

The workspace includes pre-configured tasks (`.vscode/tasks.json`):

| Task | Description |
|------|-------------|
| `build` | Build with colcon |
| `convert bags to lerobot` | Convert recordings to dataset |
| `convert bags to lerobot (parallel)` | Parallel conversion (faster) |
| `train policy` | Interactive policy training |
| `resume training` | Resume from checkpoint |
| `upload trained policy` | Push to HuggingFace Hub |

Access via `Ctrl+Shift+P` → "Tasks: Run Task".

## Credentials

The devcontainer mounts credentials from your host machine:

| Host Path | Container Path | Purpose |
|-----------|----------------|---------|
| `~/.cache/huggingface` | `/home/ros/.cache/huggingface` | HuggingFace Hub token |
| `~/.config/wandb` | `/home/ros/.config/wandb` | Weights & Biases |
| `~/.netrc` | `/home/ros/.netrc` | Git credentials |
| `~/.ssh` | `/home/ros/.ssh` | SSH keys (read-only) |

Login on your host before opening the container:
```bash
huggingface-cli login
wandb login
```

## Building

```bash
# First time setup
./scripts/setup.sh

# Build packages
./scripts/build.sh
# or
colcon build --symlink-install

# Source
source install/setup.bash
```

## License

[Apache-2.0](LICENSE)
