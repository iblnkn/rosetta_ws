---
name: verify
description: Build, launch, and drive the rosetta_ws ROS2 stack to verify changes at runtime (client node, policy server, action goals).
---

# Verifying rosetta_ws changes at runtime

## Build

```bash
pixi run build --packages-select <pkg...>     # colcon; installs to install/, python copied to build/
```

`pixi run python` imports packages from `build/<pkg>/` (copies, not symlinks) — **rebuild after editing sources** or the env runs stale code. Check with `python -c "import X; print(X.__file__)"`.

Fast pytest iteration without rebuilding:

```bash
PYTHONPATH="$PWD/src/action/<pkg>:$PWD/src/action/rosetta" pixi run python -m pytest src/action/<pkg>/test/ -q
```

## RMW / discovery gotchas

- `pixi.toml` activation sets `RMW_IMPLEMENTATION=rmw_zenoh_cpp` and **overrides any value you export before `pixi run`**. To use cyclonedds, set it *inside* the env: `pixi run bash -c 'RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 ...'`.
- Zenoh needs a router: `pixi run start-zenoh` (port 7447; "Address already in use" means one is already running — fine).
- All communicating processes must share the same RMW.

## Drive the policy inference flow

```bash
# Standalone policy server (preloads model before binding port):
pixi run python -m lerobot_robot_rosetta.policy_server --host=127.0.0.1 --port=8098 \
    --policy-type=act --pretrained-name-or-path=iblnk/act-turtlebot3_demo --policy-device=cuda

# Client node (configure spawns + preloads the server; blocks until model loaded):
pixi run ros2 launch rosetta policy_runner_launch.py \
    contract_path:=$PWD/src/action/rosetta/contracts/turtlebot3.yaml

# The RunPolicy action resolves to /run_policy (NOT /policy_runner/run_policy):
timeout -s INT 20 pixi run ros2 action send_goal /run_policy \
    rosetta_interfaces/action/RunPolicy "{prompt: 'demo'}" --feedback
# SIGINT on send_goal cancels the goal cleanly ("Finished: Cancelled" in node log).
```

- Policy-server subprocess logs go to `/tmp/rosetta_policy_server_*.log` (path printed in node log). Grep for `Policy loaded` / `Reusing loaded policy` / `Handshake done`.
- For observations without a sim, publish fake turtlebot3 topics (`/camera/image_raw/compressed`, `/overhead_camera/image_raw/compressed` as JPEG CompressedImage; `/joint_states` with wheel_{left,right}_joint; `/imu`; `/odom`) at ~30 Hz with rclpy + PIL. Enough for the bridge to warm and the control loop to run; synthetic images may still trip model-input key errors server-side — that's the fake rig, not the node.
- Model `iblnk/act-turtlebot3_demo` downloads from HF on first use (~7s warm-GPU load, no token needed).
