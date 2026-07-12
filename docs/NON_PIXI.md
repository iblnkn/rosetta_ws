# Building the Rosetta packages without pixi

The **workspace repo** (`rosetta_ws`) is pixi-first: its lockfile, tasks, and
devcontainer all assume pixi. The **packages** it assembles are plain ROS 2
packages, though — you can build them in any standard ROS 2 Jazzy environment
(apt-installed ROS on Ubuntu 24.04, or your own workspace layout). This page is
that path.

## 1. Get the sources

Each package lives in its own repository (see [`repos/src.repos`](../repos/src.repos)).
Import them into any colcon workspace:

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws
pip install vcstool   # or apt install python3-vcstool
curl -LO https://raw.githubusercontent.com/iblnkn/rosetta_ws/main/repos/src.repos
vcs import src --input src.repos --recursive
```

## 2. Install ROS dependencies

The packages declare their ROS and system dependencies in `package.xml`, so
rosdep handles them:

```bash
sudo apt update
rosdep update --rosdistro=jazzy
rosdep install --from-paths src --ignore-src -y
```

## 3. Install the python frameworks (backend-dependent)

The ML frameworks are deliberately **not** in `package.xml` — `rosetta` core
imports none of them, and the backend leaves discover them at runtime via
entry points. Install only what you need:

| You want to use | Install |
|---|---|
| `rosetta`, `rosetta_interfaces` (core bridge) | nothing extra |
| `lerobot_robot_rosetta`, `lerobot_teleoperator_rosetta` | `pip install 'lerobot[dataset,async]==0.6.0'` |

> LeRobot and the ROS python stack have overlapping numpy/opencv constraints;
> a virtualenv created with `--system-site-packages` on top of the ROS python
> is the least painful arrangement.

## 4. Build

```bash
cd ~/ros2_ws
colcon build --merge-install --symlink-install
source install/setup.bash
```

## Notes

- The default RMW in the pixi workspace is `rmw_zenoh_cpp`
  (`apt install ros-jazzy-rmw-zenoh-cpp`, run `ros2 run rmw_zenoh_cpp rmw_zenohd`
  as the router). Any RMW works for the core bridge; set `RMW_IMPLEMENTATION`
  accordingly.
- If a `package.xml` is missing a dependency rosdep should have installed,
  please file an issue on the package repo — the pixi workspace masks that
  class of bug, and this path is how we catch it.
