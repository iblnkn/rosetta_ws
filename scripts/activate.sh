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
if [ "$(uname)" = "Darwin" ] && [ -n "${CONDA_PREFIX:-}" ]; then
    for _lib in "${_root}"/install/lib/*__rosidl_*.dylib; do
        [ -e "${_lib}" ] || continue
        _dest="${CONDA_PREFIX}/lib/$(basename "${_lib}")"
        # Only create/refresh symlinks; never clobber a real file the env ships.
        if [ ! -e "${_dest}" ] || [ -L "${_dest}" ]; then
            ln -sf "${_lib}" "${_dest}"
        fi
    done
    unset _lib _dest
fi
