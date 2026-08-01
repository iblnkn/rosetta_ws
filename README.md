# rosetta_ws

The [pixi](https://pixi.sh)-powered workspace for **[Rosetta](https://github.com/iblnkn/rosetta)**,
which connects ROS 2 robots to robot-learning frameworks like LeRobot.
Full documentation: **https://iblnkn.github.io/rosetta/**

This workspace has two jobs:

1. **Get you running fast.** One `pixi run setup` installs ROS 2 Jazzy,
   LeRobot, and every Rosetta package into a committed, reproducible
   environment. Nothing installs system-wide, and you don't need Docker or
   a stack of virtualenvs.
2. **Show a complete stack.** The workspace is a working example of Rosetta
   built into a ROS 2 system: the package repos, the launch files, and the
   full record, train, deploy workflow wired together with tasks.

If you want Rosetta inside an existing ROS 2 install instead, see the
[installation guide](https://iblnkn.github.io/rosetta/installation.html).
Rosetta itself is a set of plain ROS 2 packages and doesn't require pixi.

## Quick start

Install pixi ([one-liner](https://pixi.sh/latest/#installation), v0.72+), then:

```bash
git clone https://github.com/iblnkn/rosetta_ws.git
cd rosetta_ws
pixi run --frozen setup    # clone package/library repos + install the default env
pixi run build             # colcon build -> install/
pixi run ros2 launch rosetta episode_recorder_launch.py contract_path:=<your contract>
```

(`--frozen` is only needed for that first `setup`: until it clones `libs/`,
pixi can't validate the lockfile's editable path dependencies. Every later
command is plain `pixi run ...`.)

`pixi shell` drops you into an activated environment (ROS sourced, overlay
sourced, Zenoh RMW selected) for interactive work.

From here, the [first-policy tutorial](https://iblnkn.github.io/rosetta/tutorials/first-policy.html)
walks the full loop: write a contract, record demos, train, deploy.

## Why pixi

Installing ROS 2 and LeRobot together is a real dependency problem: the ML
frameworks pin torch versions that conflict with each other, and the ROS
python stack has its own numpy and opencv opinions. Docker is the usual
answer, but it trades one problem for another: GPU and hardware passthrough,
X11 forwarding, image rebuild cycles. Pixi resolves the whole dependency
graph natively instead.

If you know the traditional ROS setup, the mental model is a direct swap.
`apt` installs Debian packages from Ubuntu, with packages.ros.org layered on
top. Pixi installs conda packages from
[conda-forge](https://conda-forge.org/), with
[RoboStack](https://robostack.github.io/) layered on top in exactly the same
way: the ROS ecosystem repackaged as conda packages (`ros-jazzy-desktop`,
`ros-jazzy-cv-bridge`, and the rest). The difference is that nothing is
installed system-wide and nothing is tied to one OS:

- Everything lives in the workspace. ROS, torch, CUDA-enabled wheels,
  and compilers all install under `.pixi/`, isolated per project. Delete the
  folder and they're gone.
- `pixi.lock` pins every dependency, transitive ones included, and is
  committed. Your laptop, CI, and the robot solve to byte-identical
  environments.
- ROS and ML dependencies are one solve. The numpy/opencv/torch conflicts
  between ROS and LeRobot are resolved once, in `pixi.toml`, for everyone,
  not per machine at install time.
- Tasks replace README run-books. The workflow lives in `pixi.toml`
  (`pixi run build`, `pixi run train`, `pixi run convert-bags`), not in
  tribal knowledge. The [Tasks](#tasks) section lists all of them.

Two caveats worth knowing: RoboStack is a community-maintained (Tier 3) ROS
platform, not an Open Robotics product; and `rosdep` doesn't work inside a
pixi environment (it shells out to apt/pip), so dependencies are added with
`pixi add` / `pixi.toml` instead. The `package.xml` files stay
rosdep-complete regardless, so the plain-ROS install path keeps working.

## Devcontainer

Pixi already provides the isolation and reproducibility most people reach
for Docker for, with native performance and no hardware passthrough
friction. But if your team standardizes on containers, or you want a pinned
CUDA userland, the workspace ships a devcontainer that wraps the same setup.
Open in VS Code → "Reopen in Container". The image
([docker/Dockerfile](docker/Dockerfile)) is toolchain-only: CUDA, pixi, and
X11 client libs. No source, no dependencies, and no build are baked in.
`postCreateCommand` runs the exact same `pixi run setup` as the host path,
into the bind-mounted workspace:

- `src/` and `libs/` live on the host (the workspace folder is a bind
  mount), so branches and uncommitted work persist across container rebuilds.
- `.pixi` lives in a named volume, so the installed environments survive
  rebuilds too. Rebuilding the image only refreshes the toolchain. (The
  volume also dodges case-insensitive-filesystem corruption on macOS/Windows
  hosts: some conda packages ship files that differ only in case.)
- Credentials (`~/.cache/huggingface`, `~/.config/wandb`, `~/.netrc`, `~/.ssh`)
  are mounted from the host. Log in with `hf auth login` or `wandb login` on
  the host first.

Because the environment definition is `pixi.toml` either way, the container
and host paths can't drift apart: the Dockerfile has nothing to keep in
sync.

## Tasks

Everything routes through pixi tasks. `pixi task list` shows all of them,
and the VS Code tasks in `.vscode/tasks.json` are one-line wrappers around
them:

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
| `docs-build` / `docs-serve` / `docs-linkcheck` | default | Sphinx site from `src/action/rosetta/doc` (build with warnings-as-errors, live-reload server, external link check) |
| `export-repos` | default | pin current checkouts back into `repos/*.repos` |

The ML workflow end-to-end, as tasks:

```bash
pixi run convert-bags --raw-dir ... --contract ... --repo-id ...
pixi run train --help
pixi run ros2 launch rosetta policy_runner_launch.py \
    contract_path:=... pretrained_name_or_path:=...
```

Colcon behavior (merge-install, symlink-install, cmake-args, `base-paths: src`)
is centralized in [`.colcon/defaults.yaml`](.colcon/defaults.yaml), activated
via `COLCON_DEFAULTS_FILE`, so a bare `colcon build` inside `pixi shell`
behaves exactly like `pixi run build`. Extra args pass straight through:
`pixi run build --packages-select rosetta`, `pixi run build --mixin debug`
(mixins come from `.colcon/mixin/`, registered by `setup`).

## Environments

| Environment | Stack | Use |
|---|---|---|
| `default` | ROS 2 Jazzy + LeRobot (torch 2.10) | daily driver for `pixi run ...` |
| `ci` | ROS 2 only (torch-free, COPY install) | CI builds |
| `bootstrap` | vcstool + git only | powers `pixi run setup` on a fresh clone |

Plain `pixi run <task>` picks the right environment automatically
(`default-environment` in `pixi.toml`); `-e` is only needed to override.

## Workspace structure

```
rosetta_ws/
├── pixi.toml                # THE environment definition (envs, deps, tasks)
├── pixi.lock                # committed lockfile (reproducible everywhere)
├── .colcon/                 # colcon defaults + local mixins
├── src/action/              # ROS2 packages (from repos/src.repos)
│   ├── rosetta/             # Core package (contracts, recorder, porter, policy runner)
│   ├── rosetta_interfaces/  # ROS2 action/service definitions
│   ├── lerobot_rosetta/     # rosetta -> LeRobot adapter (dataset writer, policy runner, servers)
│   ├── lerobot_robot_rosetta/        # LeRobot-discovered Robot plugin
│   └── lerobot_teleoperator_rosetta/ # LeRobot-discovered Teleoperator plugin
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
out of the python libraries. Each package README says what that package is
and where it fits; the [rosetta docs](https://iblnkn.github.io/rosetta/)
cover how it all works.

## VS Code

Works out of the box, container or not:

- The Python extension auto-detects pixi environments via the `.pixi` folder;
  if it doesn't, point "Python: Select Interpreter" at
  `.pixi/envs/default/bin/python` (the devcontainer pre-configures this).
- `.vscode/tasks.json` wraps the pixi tasks, so build/test/lint are available
  from the task picker with problem matchers wired up.
- The community [Pixi extension](https://marketplace.visualstudio.com/items?itemName=renan-r-santos.pixi-code)
  adds a task explorer and dependency management in the UI (installed
  automatically in the devcontainer).

## Tab completion

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
# re-dispatches; outside the env it returns 1 and you get plain filename
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
  can't complete their arguments either: argcomplete has to run the target
  command in a shell where the environment is active, which is never true for
  the outer shell driving `pixi run`.
- For `ros2 launch <pkg><TAB>` and friends, work inside **`pixi shell`**,
  where the lazy stub gives full ros2/colcon completion.

Notes: argcomplete ships with the env, so nothing extra needs installing.
Registering from a pixi activation script doesn't work either: activation
propagates environment variables, not interactive shell hooks. And
argcomplete's `activate-global-python-argcomplete` hook is a dead end here:
RoboStack's `ros2`/`colcon` entry-point scripts lack the
`PYTHON_ARGCOMPLETE_OK` marker it keys on. See
[pixi#2366](https://github.com/prefix-dev/pixi/issues/2366) for background.

## Contributing notes

- `pixi.lock` is committed (and marked generated in `.gitattributes`).
  After changing `pixi.toml`, run `pixi lock` and commit both. The lock can
  only solve when `libs/` is populated (editable path deps), so run
  `pixi run --frozen setup` first on a fresh clone.
- CI (`.github/workflows/ci.yaml`) builds the torch-free `ci` environment with
  `frozen: true`; an out-of-date lockfile fails the build.
- CI and `setup` use `--frozen` rather than `--locked`: pixi's lock
  up-to-date check currently mis-reads lerobot's `[tool.uv.sources]` cu128
  index pin and reports a false mismatch (the lock itself is correct;
  `pixi lock` regenerates it byte-identical).
- Non-pixi consumers: keep `package.xml` rosdep metadata honest. The plain-ROS
  path in Rosetta's
  [installation guide](https://iblnkn.github.io/rosetta/installation.html)
  depends on rosdep resolving everything from `package.xml` alone.

## License

[Apache-2.0](LICENSE)
