#!/usr/bin/env bash
# Source the default-environment colcon overlay if it has been built.
# Guarded so activation works before the first build.
_root="${PIXI_PROJECT_ROOT:-$PWD}"
if [ -f "${_root}/install/setup.bash" ]; then
    set +u
    # shellcheck disable=SC1091
    source "${_root}/install/setup.bash"
    set -u
fi
