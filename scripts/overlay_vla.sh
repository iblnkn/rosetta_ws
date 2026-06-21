#!/usr/bin/env bash
# Source the vla-environment colcon overlay (built against the vla env python).
# Guarded so activation works before the first build.
_root="${PIXI_PROJECT_ROOT:-$PWD}"
if [ -f "${_root}/install_vla/setup.bash" ]; then
    set +u
    source "${_root}/install_vla/setup.bash"
    set -u
fi
