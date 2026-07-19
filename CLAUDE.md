This file provides guidance to AI agents when working with code in this repository.

## Workspace Layout

`rosetta_ws` is the pixi-based workspace for **Rosetta**, a ROS 2 ↔ LeRobot bridge. Setup and quick-start live in [README.md](README.md); don't duplicate them here.

- **`src/` and `libs/` are gitignored.** `pixi run setup` populates them via `vcs import` from `repos/src.repos` / `repos/libs.repos`. Each package under `src/` is its own git repository — branch, commit, and diff inside the package repo, not in `rosetta_ws`. `pixi run export-repos` pins the current checkouts back into `repos/*.repos`.
- `libs/lerobot` is stock upstream lerobot pinned at `v0.6.0` — see `repos/libs.repos` for the one local feature branch and why.
- Every colcon invocation shares `.colcon/defaults.yaml` (pixi activation exports `COLCON_DEFAULTS_FILE`). `BUILD_TESTING` is **off** in those defaults: run `pixi run build-with-tests` before `pixi run test`, or there are no tests to run. A CLI `--cmake-args` replaces the defaults list wholesale — restate the `Python_FIND_VIRTUALENV` flags if you override (see the `build-with-tests` task for the full list).

Key tasks (`pixi run <task>`): `build`, `build-with-tests`, `test` (colcon test + `test-result --verbose`), `lint` (ruff over `src/action` and `scripts` with the workspace `ruff.toml`, then pre-commit), `clean`, `start-zenoh` (Zenoh RMW router — own terminal, leave running).

## Testing Standards

Adapted from the [ROS2 Testing Workshop's AI Testing Guide](https://github.com/Ekumen-OS/ros2_testing_workshop_roscon_es_25/blob/main/AI_TESTING_GUIDE.md) for this workspace's stack: Python / `ament_python` / pytest, not C++ / `ament_cmake` / gtest.

### Testable Design

Algorithmic logic is decoupled from the ROS 2 middleware. Nodes are thin wrappers over ROS interfaces; application logic lives in its own class/module, importable and testable without `rclpy`. Inject dependencies and config through constructors so tests can substitute fakes. Core functions take and return plain `numpy` arrays or dataclasses — never a `*_msgs` type (e.g. a detector takes `ranges: np.ndarray`, not a `sensor_msgs.msg.LaserScan`); message ↔ plain-type conversion happens in the node class. Avoiding `rclpy` imports alone isn't enough — a function that takes a ROS message as an argument is coupled to the message API even if it never touches `rclpy`.

### Testing Pyramid

- **Static analysis**: `ruff check` / `ruff format` via `pixi run lint` or each package's CI lint job.
- **Unit tests (the majority)**: pytest over ROS-independent logic only — no `rclpy` import, no `*_msgs` types in signatures. Use `@pytest.mark.parametrize` to cover an algorithm's edge cases instead of duplicating near-identical tests.
- **ROS component tests**: node interfaces (topics, services, parameters) in isolation — patterns below.
- **Integration tests**: multi-node interaction via `launch_testing` (Python launch files).
- **End-to-end**: full system against a replayed mission — none exists yet; patterns below for when one is written.

### ROS Component Test Patterns

- **Fixture lifecycle**: a session/module-scoped fixture calls `rclpy.init()`/`rclpy.shutdown()` once (expensive, global); a function-scoped fixture creates/destroys the node under test per test (cheap, isolated). Never init/shutdown `rclpy` per test.
- **Parameters**: construct the node with `parameter_overrides=[...]` and assert `node.get_parameter("name").value` — verifies declaration and propagation without spinning.
- **Interface registration**: `node.get_topic_names_and_types()` / `node.get_service_names_and_types()` confirm a node exposes the right names and types, with no traffic and no executor.
- **Node pipelines** (one process, multiple nodes — the layer between a unit test and full `launch_testing`): a test node (publishes inputs, subscribes to outputs) plus the DUT on one `SingleThreadedExecutor`. Drive it explicitly: loop `executor.spin_once(timeout_sec=...)` until the expected condition or a timeout; `spin_until_future_complete(future, timeout_sec=...)` for service/action calls; for timer-driven logic set `use_sim_time` and publish `/clock` from the test node to advance time deterministically.

### End-to-End Test Patterns

An E2E test runs the full stack against a replayed mission and asserts on final state. When one is written:

- **Purpose-built fixture bags**: `ros2 bag record -o <name> <specific topics>`, not `-a`.
- **Deterministic replay**: `ros2 bag play --clock <bag>` publishes `/clock`, so nodes with `use_sim_time=True` replay on the bag's timeline — the same mechanism as the timer case above, scaled to a full mission.
- **Same framework, bigger**: `generate_test_description()` launches the full system plus `ros2 bag play <episode>` as an `ExecuteProcess`; the test case subscribes to a result/status topic, waits for the bag to finish, asserts final state within tolerance.
- Evaluate [`replay_testing`](https://github.com/polymathrobotics/replay_testing) (Polymath Robotics) — a purpose-built wrapper for exactly this pattern — before hand-rolling the scaffolding.

### Determinism and Reliability

- **No arbitrary sleeps.** Wait on the actual condition (a received message, a service response, a state transition) with a bounded timeout.
- **Subscriber-readiness race in `launch_testing` tests**: `ReadyToTest()` means the launch *process* completed, not that subscriptions are live — publishing immediately after launch drops the message. Either poll `publisher.get_subscription_count() > 0` (with `spin_once` and a bounded timeout) before publishing, or use a continuously repeating publisher (`ros2 topic pub -r <hz> ...`) so a late subscriber still catches a message — see `rosetta/test/test_bridge_launch.py` for the latter.
- **Test isolation**: there is no `ament_python` equivalent of `ament_add_ros_isolated_gtest`. Give each test module a unique `ROS_DOMAIN_ID` (pytest fixture setting the env var before `rclpy.init()`) to prevent cross-talk in parallel runs. Until that's in place everywhere, `colcon test --executor sequential` is the pragmatic fallback.
- **No EXPECT/ASSERT split in pytest**: plain `assert` always aborts the test (gtest's `ASSERT_*`). Don't add a soft-assertion library — keep each test narrow enough that one assert failing is exactly as informative as intended; five narrow tests beat one test with five stacked asserts.

## Local Development

- Pre-commit hooks (ruff, trailing-whitespace, etc. — see `.pre-commit-config.yaml`) catch style issues before CI.
- **`launch_testing` pytest-plugin gotcha**: `lerobot_robot_rosetta`'s `setup.cfg` disables the `launch_ros`/`launch_testing` pytest plugins (`-p no:launch_ros -p no:launch_testing`); `rosetta`'s doesn't, and has a working `launch_testing` test. This matches a known pytest ≥ 9.1 / `launch_testing` incompatibility (the workspace pins pytest `< 9.1` for it). Before adding integration tests to a package that disables these plugins, revisit that `addopts` line first — otherwise `generate_test_description` silently isn't collected.
- **macOS build gotcha**: on recent macOS SDKs (observed on macOS 26.5/Tahoe), robostack-jazzy's bundled clang/`ld` fails to link any `ament_cmake` dylib with `Undefined symbols ... ___assert_rtn / ___stack_chk_fail / ___stack_chk_guard` — the bundled linker can't resolve implicit libSystem symbols against the new SDK's `.tbd` format. Fixed in `pixi.toml`'s `[target.osx-arm64.activation]`, which routes linking through the system linker via `scripts/macos-ld-wrapper.sh` (Apple's `ld` also requires the `-lto_library` basename to be exactly `libLTO.dylib`, which conda's clang doesn't pass). No action needed unless that env var is removed.

## Continuous Integration

Each package repo (`rosetta`, `rosetta_interfaces`, `lerobot_rosetta`, `lerobot_robot_rosetta`, `lerobot_teleoperator_rosetta`) has its own standalone, self-contained `.github/workflows/ci.yml`:

- **`industrial_ci`** (`ros-industrial/industrial_ci@master`) builds and tests the package inside an official ROS Docker image, so there's no "is ROS actually installed on this runner" class of bug (a bare runner + `ros-tooling/setup-ros` was tried first and hit exactly that). `rosdep` resolves deps from `package.xml`; `UPSTREAM_WORKSPACE` (e.g. `"github:iblnkn/rosetta_interfaces#main"`) pulls direct upstream deps inline. `colcon test-result --verbose` and the `ament_cmake` lint tests run automatically — no custom scripting.
- **`ROSDEP_SKIP_KEYS: ament_python`** is required on every `ament_python` package's job. `ament_python` has no rosdep key (pure-Python build support ships with base ROS, not as an apt package), and `industrial_ci` treats any unresolved key as fatal. This is a known ecosystem-wide rosdep asymmetry, not a `package.xml` bug.
- **`lint`** is a separate job running `ruff check` against the shared `ruff.toml` (fetched from `rosetta_ws`) — `industrial_ci` predates ruff.

`rosetta_ws`'s own CI (`.github/workflows/ci.yaml`) is a separate pixi-based cross-package integration build — it validates the whole pinned workspace together (including the non-ROS pixi/lerobot/torch stack), and is not the primary gate for any individual package.
