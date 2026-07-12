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
- **ROS Unit/Component Tests**: Validate node interfaces (topics, services, parameters) in isolation using pytest fixtures that manage `rclpy.init()`/`shutdown()` around each test.
- **Integration Tests**: Multi-node interaction via `launch_testing` (same framework as C++, used with Python launch files).
- **End-to-End**: Full system behavior against simulation or real hardware.

### Determinism and Reliability

- **No arbitrary sleeps** — they make tests flaky and non-deterministic. Wait on the actual condition (a received message, a service response, a state transition) with a bounded timeout instead.
- **Test isolation**: there is no Python/`ament_python` equivalent of `ament_add_ros_isolated_gtest`. Give each test module/session a unique `ROS_DOMAIN_ID` (e.g. via a pytest fixture setting the env var before `rclpy.init()`) to prevent cross-talk when tests run in parallel on the same network.
- **No EXPECT/ASSERT split in pytest**: gtest distinguishes `EXPECT_*` (record failure, keep running — collects multiple mismatches) from `ASSERT_*` (abort immediately — for preconditions where continuing would be meaningless). Plain `assert` in pytest always behaves like `ASSERT_*`; there's no built-in soft-assertion mode. Don't reach for a library to emulate `EXPECT_*` — keep each test narrow enough that one `assert` failure is exactly as informative as the assertion pyramid intends. A test with five stacked asserts loses information when the first one aborts it; five narrow tests don't.

### Local Development

Pre-commit hooks (ruff, trailing-whitespace, etc. — see `.pre-commit-config.yaml`) catch style issues before they reach CI.

### Continuous Integration

Each package repo (`rosetta`, `rosetta_interfaces`, `lerobot_robot_rosetta`, `lerobot_teleoperator_rosetta`, `starvla_rosetta`, `vla_foundry_rosetta`) has its own standalone CI via the shared `reusable-ros-ci.yml` workflow in this repo: build + `colcon test` + `colcon test-result` via `ros-tooling/action-ros-ci`, plus a standalone `ruff` lint job. `rosetta_ws`'s own CI is the separate cross-package integration build (see `.github/workflows/ci.yaml`), not the primary gate for any single package.
