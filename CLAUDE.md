This file provides guidance to AI agents when working with code in this repository.

## Testing Standards

Adapted from the [ROS2 Testing Workshop's AI Testing Guide](https://github.com/Ekumen-OS/ros2_testing_workshop_roscon_es_25/blob/main/AI_TESTING_GUIDE.md) for this workspace's stack: Python / `ament_python` / pytest, not C++ / `ament_cmake` / gtest.

### Testable Design

Algorithmic logic must be decoupled from the ROS2 middleware:

- **Single Responsibility**: ROS2 nodes are thin wrappers over ROS interfaces. Application logic lives in its own class/module, importable and testable without `rclpy`.
- **Dependency Injection**: Inject dependencies/config into a logic class's constructor rather than constructing them internally, so tests can substitute fakes/mocks.
- **Interface Segregation**: Depend on abstractions, not concrete implementations, so real components can be swapped for fakes in unit tests.
- **ROS-agnostic signatures, not just ROS-agnostic imports**: core algorithm functions/classes should take and return plain `numpy` arrays or dataclasses, never a `*_msgs` type directly (e.g. a detector takes `ranges: np.ndarray`, not a `sensor_msgs.msg.LaserScan`). The node class is where message ↔ plain-type conversion happens. Avoiding `rclpy` imports alone isn't enough — a function that still takes a ROS message as its argument is coupled to the message API even if it never touches `rclpy`.

### Testing Pyramid

- **Static Analysis (foundation)**: `ruff check` / `ruff format` — via `pixi run lint` or each package's own CI lint job. Catches style and correctness issues before runtime.
- **Unit Tests (majority)**: pytest, Arrange-Act-Assert. Target ROS-independent logic only — no `rclpy` import, and no `*_msgs` types in the function signature (see above). Fast, deterministic, aim for high coverage on core algorithms. Use `@pytest.mark.parametrize` to cover an algorithm's edge cases instead of duplicating near-identical test functions.
- **ROS Unit/Component Tests**: Validate node interfaces (topics, services, parameters) in isolation. `rclpy.init()`/`shutdown()` happen once per test class/session (expensive, global); the node itself is created/destroyed per test (cheap, isolated) — see patterns below.
- **Integration Tests**: Multi-node interaction via `launch_testing` (same framework as C++, used with Python launch files).
- **End-to-End**: Full system behavior against simulation or real hardware — see patterns below. No test at this level exists in the codebase yet.

### ROS Unit/Component Test Patterns

- **Fixture lifecycle**: a session/module-scoped fixture calls `rclpy.init()`/`rclpy.shutdown()` once; a function-scoped fixture creates/destroys the node under test per test case. Don't init/shutdown `rclpy` per test — it's the expensive global step the fixture split exists to avoid repeating.
- **Parameter testing**: construct the node with `parameter_overrides=[...]` and assert `node.get_parameter("name").value` — verifies declaration and propagation without spinning anything.
- **Topic/service registration checks**: `node.get_topic_names_and_types()` / `node.get_service_names_and_types()` confirm a node exposes the interfaces it's supposed to (name + type), with no message traffic and no executor needed.
- **Node pipeline tests** (the layer between a pure unit test and a full `launch_testing` integration test — one process, multiple ROS nodes talking to each other): a test node (publishes inputs, subscribes to outputs) plus the DUT node, both added to one `rclpy.executors.SingleThreadedExecutor`. Drive it explicitly, never sleep-and-hope:
  - Loop `executor.spin_once(timeout_sec=...)` until the expected condition is observed or a timeout elapses.
  - Use `executor.spin_until_future_complete(future, timeout_sec=...)` for service/action calls.
  - For timer-driven logic, set `use_sim_time` and publish `/clock` from the test node to advance time deterministically instead of waiting in real time.

### End-to-End Test Patterns

An E2E test runs the full stack against a replayed mission and asserts on final state — the level above `launch_testing` integration tests, which only check that a subset of nodes communicate correctly. This project already has rosbag/MCAP infrastructure (see the Rerun MCAP ingestion notes) but no test at this level yet; when one is written:

- **Purpose-built fixture bags**: `ros2 bag record -o <name> <specific topics>`, not `-a` — record exactly what the test needs, not everything.
- **Deterministic mission replay**: `ros2 bag play --clock <bag>` publishes `/clock`, so any node with `use_sim_time=True` replays on the bag's original timeline instead of wall-clock — the same mechanism as the single-timer case in the Node Pipeline pattern above, scaled to a full mission.
- **The pattern is `launch_testing`, just bigger**: `generate_test_description()` launches the full system (policy runner + robot interface) plus `ros2 bag play <episode>` as an `ExecuteProcess`; the `unittest.TestCase` creates a temporary node, subscribes to a result/status topic, waits for the bag to finish, and asserts final state within tolerance. No new framework — the same tools as the integration-test layer.
- **`replay_testing` (Polymath Robotics)**: a purpose-built wrapper for exactly this pattern (launch system + bag together, sync `/clock`, check a completion condition). Worth evaluating before hand-rolling this scaffolding, given how bag-centric this project already is.

### Determinism and Reliability

- **No arbitrary sleeps** — they make tests flaky and non-deterministic. Wait on the actual condition (a received message, a service response, a state transition) with a bounded timeout instead.
- **Subscriber-readiness race in launch_testing integration tests**: `ReadyToTest()` only means the launch *process* completed — it does not mean a node's subscriptions are live yet. Publishing immediately after launch is a classic flaky-test cause: the message gets dropped because nothing is subscribed. Two valid fixes, both already used or documented in this workspace: (1) after creating the test's publisher, loop `spin_once()` while polling `publisher.get_subscription_count() > 0` with a bounded timeout, *then* publish; or (2) use a continuously-repeating publisher (`ros2 topic pub -r <hz> ...`) instead of a one-shot publish, so a late subscriber still catches a message within the poll window — see `rosetta/test/test_bridge_launch.py` for a working example of (2).
- **Test isolation**: there is no Python/`ament_python` equivalent of `ament_add_ros_isolated_gtest`. Give each test module/session a unique `ROS_DOMAIN_ID` (e.g. via a pytest fixture setting the env var before `rclpy.init()`) to prevent cross-talk when tests run in parallel on the same network. Until that's in place everywhere, `colcon test --executor sequential` is a pragmatic fallback — no code changes, guarantees no cross-talk, costs wall-clock time.
- **No EXPECT/ASSERT split in pytest**: gtest distinguishes `EXPECT_*` (record failure, keep running — collects multiple mismatches) from `ASSERT_*` (abort immediately — for preconditions where continuing would be meaningless). Plain `assert` in pytest always behaves like `ASSERT_*`; there's no built-in soft-assertion mode. Don't reach for a library to emulate `EXPECT_*` — keep each test narrow enough that one `assert` failure is exactly as informative as the assertion pyramid intends. A test with five stacked asserts loses information when the first one aborts it; five narrow tests don't.

### Local Development

Pre-commit hooks (ruff, trailing-whitespace, etc. — see `.pre-commit-config.yaml`) catch style issues before they reach CI.

`launch_testing`/`launch_ros` pytest-plugin gotcha: `lerobot_robot_rosetta`'s `setup.cfg` disables both (`-p no:launch_ros -p no:launch_testing`) while `rosetta`'s doesn't, and `rosetta` has a working `launch_testing` test. This lines up with a known pytest≥9.1/`launch_testing` incompatibility (the workspace pins pytest `<9.1` for this reason). Before adding integration tests to any package that disables these plugins, that `addopts` line needs to be revisited first — otherwise `generate_test_description` just won't be collected, silently.

### Continuous Integration

Each package repo (`rosetta`, `rosetta_interfaces`, `lerobot_robot_rosetta`, `lerobot_teleoperator_rosetta`, `starvla_rosetta`, `vla_foundry_rosetta`) has its own standalone `.github/workflows/ci.yml`, self-contained (no cross-repo reusable-workflow reference):

- **`industrial_ci`** (`ros-industrial/industrial_ci@master`) builds and tests the package. It's Docker-native — pulls an official ROS image and builds inside it, so there's no "is ROS actually installed on this runner" class of bug to hand-roll around (unlike a bare GitHub runner + `ros-tooling/setup-ros`, which was tried first and hit exactly that: missing `rosdep`, missing `ament_package`, missing `launch`). `rosdep` resolves deps straight from `package.xml`; `UPSTREAM_WORKSPACE` (e.g. `"github:iblnkn/rosetta_interfaces#main"`) pulls in direct upstream deps inline, no separate `.repos` file needed. `colcon test` also runs `ament_lint_cmake`/`ament_xmllint` automatically for `ament_cmake` packages, and `colcon test-result --verbose` runs automatically — no custom scripting for either.
- **`ROSDEP_SKIP_KEYS: ament_python`** is required on every `ament_python` package's `industrial_ci` job. `ament_python` has no rosdep key of its own — pure-Python build support ships with the base ROS install rather than as a separate apt package — and `industrial_ci` treats any unresolved rosdep key as fatal (unlike `action-ros-ci`, which tolerated it silently). This is not a bug in this repo's `package.xml`; it's a known, ecosystem-wide rosdep-database asymmetry (`ament_cmake` does resolve; `ament_python` doesn't).
- **`lint`** is a separate job running `ruff check` against the shared `ruff.toml` (fetched from `rosetta_ws`) — `industrial_ci` predates ruff and has no native support for it.

`rosetta_ws`'s own CI (`.github/workflows/ci.yaml`) is a separate, pixi-based cross-package integration build — it validates the whole pinned workspace together (including the non-ROS pixi/lerobot/torch stack), not a single package's `colcon` build, and is not the primary gate for any individual package.
