#!/usr/bin/env bash
set -e

# Source ROS 2
: "${ROS_DISTRO:=jazzy}"
source "/opt/ros/$ROS_DISTRO/setup.bash" --

# Activate venv if present and add to PYTHONPATH for ROS2 compatibility
# See: https://docs.ros.org/en/jazzy/How-To-Guides/Using-Python-Packages.html
if [ -n "$VIRTUAL_ENV" ] && [ -f "$VIRTUAL_ENV/bin/activate" ]; then
    source "$VIRTUAL_ENV/bin/activate"
    export PYTHONPATH="${VIRTUAL_ENV}/lib/python3.12/site-packages:${PYTHONPATH}"
fi

# Source workspace if built
if [ -f "${WORKSPACE:-/workspaces/rosetta_ws}/install/setup.bash" ]; then
    source "${WORKSPACE:-/workspaces/rosetta_ws}/install/setup.bash" --
fi

exec "$@"