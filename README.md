# rosetta_ws

The [pixi](https://pixi.sh)-powered workspace for **[Rosetta](src/action/rosetta/README.md)**,
the ROS2 to LeRobot bridge.

## Why This Workspace?

Getting ROS2 and LeRobot installed together is not trivial — the ML frameworks
pin incompatible torch versions and the ROS python stack has its own numpy and
opencv opinions. This workspace resolves all of it into committed, reproducible
[pixi](https://pixi.sh) environments: one `pixi run setup` gives you ROS 2
Jazzy (via [RoboStack](https://robostack.github.io/)), LeRobot, and every tool
the workflow needs — no Docker, no system ROS, no venv juggling.

## Quick Start

Install pixi ([one-liner](https://pixi.sh/latest/#installation), v0.72+), then:

```bash
git clone https://github.com/iblnkn/rosetta_ws.git
cd rosetta_ws
pixi run --frozen setup    # clone package/library repos + install the default env
pixi run build             # colcon build -> install/
pixi run ros2 launch rosetta rosetta_client_launch.py
```

(`--frozen` is only needed for that first `setup`: until it clones `libs/`,
pixi can't validate the lockfile's editable path dependencies. Every later
command is just `pixi run ...`.)

`pixi shell` drops you into an activated environment (ROS sourced, overlay
sourced, Zenoh RMW selected) for interactive work.

### Tab completion

Two one-time additions to `~/.bashrc` (zsh users: `pixi completion --shell
zsh` and argcomplete's zsh hooks are the equivalents):

```bash
# pixi itself: subcommands, and task names for `pixi run <TAB>`
eval "$(pixi completion --shell bash)"

# ros2/colcon argument completion (subcommands, packages, launch files),
# registered lazily on the first TAB press. Eager registration here won't
# work: `pixi shell` sources ~/.bashrc BEFORE activating the env, so
# register-python-argcomplete isn't on PATH yet at rc time. The stub looks
# it up at completion time instead, swaps in the real argcomplete hook, and
# re-dispatches — outside the env it returns 1 and you get plain filename
# completion.
_pixi_lazy_argcomplete() {
  local reg realfn
  reg=$(command -v register-python-argcomplete) || return 1
  eval "$("$reg" "$1")"
  realfn=$(complete -p "$1" 2>/dev/null | sed -E 's/.*-F ([^ ]+).*/\1/')
  [ -n "$realfn" ] && [ "$realfn" != "_pixi_lazy_argcomplete" ] && "$realfn" "$@"
}
complete -o nospace -o bashdefault -o default -F _pixi_lazy_argcomplete ros2 colcon
```

Know what each layer can and can't complete:

- `pixi run <TAB>` completes **pixi task names only** (`build`, `test`,
  `start-zenoh`, ...). It will never complete env binaries like `ros2`, and it
  can't complete their arguments either — argcomplete has to run the target
  command in a shell where the environment is active, which is never true for
  the outer shell driving `pixi run`.
- For `ros2 launch <pkg><TAB>` and friends, work inside **`pixi shell`** —
  with the stub above, full ros2/colcon completion works there.

Notes: argcomplete ships with the env, so nothing extra needs installing.
Registering from a pixi activation script doesn't work either — activation
propagates environment variables, not interactive shell hooks. And
argcomplete's `activate-global-python-argcomplete` hook is a dead end here:
RoboStack's `ros2`/`colcon` entry-point scripts lack the
`PYTHON_ARGCOMPLETE_OK` marker it keys on. See
[pixi#2366](https://github.com/prefix-dev/pixi/issues/2366) for background.

Prefer a container? The devcontainer (below) is a thin wrapper that runs the
exact same `pixi run setup`. Want the packages without pixi at all? See
[docs/NON_PIXI.md](docs/NON_PIXI.md).

## Environments

| Environment | Stack | Use |
|---|---|---|
| `default` | ROS 2 Jazzy + **LeRobot** (torch 2.10) | `pixi run ...` — daily driver |
| `ci` | ROS 2 only (torch-free, COPY install) | CI builds |
| `bootstrap` | vcstool + git only | powers `pixi run setup` on a fresh clone |

Convert bags to a dataset and deploy a policy:

```bash
pixi run convert-bags --raw-dir ... --contract ... --repo-id ...
pixi run ros2 launch rosetta rosetta_client_launch.py backend:=lerobot ...
```

Backends register under the `rosetta.dataset_writers` / `rosetta.policy_runners`
entry-point groups; `rosetta` core imports no ML framework directly.

## Tasks

Everything routes through pixi tasks (`pixi task list` shows all of them; the
VS Code tasks in `.vscode/tasks.json` are one-line wrappers around these):

| Task | Env | Description |
|---|---|---|
| `setup` | bootstrap | Fresh-clone setup: `vcs import` + `pixi install` + colcon mixins (run with `--frozen` the first time) |
| `build` | default | colcon build → `install/` |
| `build-with-tests` | default | build with `BUILD_TESTING=ON` |
| `test` | default | `colcon test` + result report |
| `clean` | default | remove build/install/log |
| `lint` | default | ruff (`ruff.toml`) over `src/action` + `scripts`; pre-commit hooks over workspace files |
| `start-zenoh` | default | Zenoh RMW router (own terminal, leave running) |
| `convert-bags` / `convert-bags-parallel` | default | bag → LeRobot dataset conversion |
| `train` / `resume-train` | default | LeRobot policy training (`scripts/train_policy.py`) |
| `export-repos` | default | pin current checkouts back into `repos/*.repos` |

Colcon behavior (merge-install, symlink-install, cmake-args, `base-paths: src`)
is centralized in [`.colcon/defaults.yaml`](.colcon/defaults.yaml), activated
via `COLCON_DEFAULTS_FILE` — so a bare `colcon build` inside `pixi shell`
behaves exactly like `pixi run build`. Extra args pass straight through:
`pixi run build --packages-select rosetta`, `pixi run build --mixin debug`
(mixins come from `.colcon/mixin/`, registered by `setup`).

## Workspace Structure

```
rosetta_ws/
├── pixi.toml                # THE environment definition (envs, deps, tasks)
├── pixi.lock                # committed lockfile — reproducible everywhere
├── .colcon/                 # colcon defaults + local mixins
├── src/action/              # ROS2 packages (from repos/src.repos)
│   ├── rosetta/             # Core package - see rosetta/README.md
│   ├── rosetta_interfaces/  # ROS2 action/service definitions
│   ├── lerobot_robot_rosetta/        # LeRobot backend leaf
│   └── lerobot_teleoperator_rosetta/
├── libs/                    # Python libraries (from repos/libs.repos, editable installs)
│   └── lerobot/             # LeRobot
├── models/                  # Trained policies
├── datasets/                # bags/ (recordings) + lerobot/ (converted datasets)
├── repos/                   # vcs import manifests (src.repos, libs.repos)
├── scripts/                 # activation script + ML workflow implementations
├── docker/                  # Toolchain-only devcontainer image
└── .devcontainer/x86/       # VS Code devcontainer (thin wrapper: mounts workspace, runs pixi)
```

`src/action` packages and `libs/` libraries are each their own git repo,
imported by `pixi run setup`; `repos/*.repos` pins what a fresh clone gets
(`pixi run export-repos` updates the pins). `libs/COLCON_IGNORE` keeps colcon
out of the python libraries.

## Devcontainer (optional)

Open in VS Code → "Reopen in Container". The image
([docker/Dockerfile](docker/Dockerfile)) is toolchain-only — CUDA, pixi, and
X11 client libs; no source, no dependencies, no build baked in.
`postCreateCommand` runs the same `pixi run setup` as the host path, into the
bind-mounted workspace:

- `src/` and `libs/` live on the **host** (the workspace folder is a bind
  mount), so branches and uncommitted work persist across container rebuilds.
- `.pixi` lives in a named volume, so the installed environments survive
  rebuilds too. Rebuilding the image only refreshes the toolchain.
- Credentials (`~/.cache/huggingface`, `~/.config/wandb`, `~/.netrc`, `~/.ssh`)
  are mounted from the host — log in with `hf auth login` / `wandb login` on
  the host first.

## Contributing notes

- **`pixi.lock` is committed** (and marked generated in `.gitattributes`).
  After changing `pixi.toml`, run `pixi lock` and commit both. The lock can
  only solve when `libs/` is populated (editable path deps) — run
  `pixi run --frozen setup` first on a fresh clone.
- CI (`.github/workflows/ci.yaml`) builds the torch-free `ci` environment with
  `frozen: true`; an out-of-date lockfile fails the build.
- CI and `setup` use `--frozen` rather than `--locked`: pixi's lock
  up-to-date check currently mis-reads lerobot's `[tool.uv.sources]` cu128
  index pin and reports a false mismatch (the lock itself is correct —
  `pixi lock` regenerates it byte-identical).
- Non-pixi consumers: keep `package.xml` rosdep metadata honest — see
  [docs/NON_PIXI.md](docs/NON_PIXI.md).

## License

[Apache-2.0](LICENSE)
