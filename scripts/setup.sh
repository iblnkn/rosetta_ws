#!/bin/bash
# setup.sh - Workspace setup script for ROS 2 Jazzy
# Handles repository import, dependency installation, and environment setup
#
# Works in two modes:
#   Docker/Linux:  ./scripts/setup.sh          (uses apt, rosdep, venv)
#   pixi (any OS): pixi run setup              (deps managed by pixi.toml)
set -e

# ============================================================================
# Configuration
# ============================================================================
WORKSPACE="${WORKSPACE:-$(pwd)}"
if command -v nproc &> /dev/null; then
    PARALLEL_WORKERS=$(($(nproc) / 2))
else
    PARALLEL_WORKERS=$(($(sysctl -n hw.ncpu) / 2))
fi

IS_PIXI="${PIXI_PROJECT_ROOT:+true}"
IS_LINUX=$([[ "$(uname -s)" == "Linux" ]] && echo true || echo false)

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log_info()  { echo -e "${GREEN}[INFO]${NC} $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }

# ============================================================================
# Repository Management
# ============================================================================
import_repos() {
    local repos_file="$1"
    local target_dir="$2"

    if [[ ! -f "${repos_file}" ]]; then
        log_warn "Repos file not found: ${repos_file}"
        return 0
    fi

    if [[ ! -s "${repos_file}" ]]; then
        log_warn "Repos file is empty: ${repos_file}"
        return 0
    fi

    if ! command -v vcs &> /dev/null; then
        # Fallback: clone repos with plain git when vcs is not available
        # Parses .repos YAML to respect directory structure (e.g. action/rosetta)
        log_warn "'vcs' not found, falling back to git clone..."
        mkdir -p "${target_dir}"
        local current_path="" current_url="" current_version=""
        while IFS= read -r line; do
            if [[ "${line}" =~ ^[[:space:]]{2}([a-zA-Z0-9_/-]+):$ ]]; then
                # Commit previous entry if we have one
                if [[ -n "${current_path}" && -n "${current_url}" ]]; then
                    if [[ ! -d "${target_dir}/${current_path}" ]]; then
                        mkdir -p "$(dirname "${target_dir}/${current_path}")"
                        git clone ${current_version:+--branch "${current_version}"} "${current_url}" "${target_dir}/${current_path}" || true
                    fi
                fi
                current_path="${BASH_REMATCH[1]}"
                current_url=""
                current_version=""
            elif [[ "${line}" =~ url:[[:space:]]*(.+) ]]; then
                current_url="${BASH_REMATCH[1]}"
            elif [[ "${line}" =~ version:[[:space:]]*(.+) ]]; then
                current_version="${BASH_REMATCH[1]}"
            fi
        done < "${repos_file}"
        # Commit last entry
        if [[ -n "${current_path}" && -n "${current_url}" && ! -d "${target_dir}/${current_path}" ]]; then
            mkdir -p "$(dirname "${target_dir}/${current_path}")"
            git clone ${current_version:+--branch "${current_version}"} "${current_url}" "${target_dir}/${current_path}" || true
        fi
        return 0
    fi

    log_info "Importing from ${repos_file} into ${target_dir}/"
    mkdir -p "${target_dir}"
    vcs import "${target_dir}" < "${repos_file}" --recursive -w "${PARALLEL_WORKERS}" || true
}

fix_detached_heads() {
    local base_dir="$1"

    [[ ! -d "${base_dir}" ]] && return 0

    log_info "Fixing detached HEAD repositories in ${base_dir}..."

    while IFS= read -r -d '' git_dir; do
        local repo_dir
        repo_dir=$(dirname "${git_dir}")

        (
            cd "${repo_dir}" || exit 0

            # Skip if already on a branch
            git symbolic-ref -q HEAD > /dev/null 2>&1 && exit 0

            # Get default branch
            local default_branch
            default_branch=$(git symbolic-ref refs/remotes/origin/HEAD 2>/dev/null | sed 's@^refs/remotes/origin/@@') || default_branch="main"

            log_info "  Checking out ${default_branch} in $(basename "${repo_dir}")"

            # Stash changes if needed
            if ! git diff-index --quiet HEAD -- 2>/dev/null; then
                log_warn "    Stashing uncommitted changes"
                git stash -q 2>/dev/null || true
            fi

            git checkout -q "${default_branch}" 2>/dev/null || \
            git checkout -q "main" 2>/dev/null || \
            git checkout -q "master" 2>/dev/null || true
        )
    done < <(find "${base_dir}" -type d -name ".git" -print0 2>/dev/null)
}

pull_repos() {
    local repos_file="$1"
    local target_dir="$2"

    if [[ ! -f "${repos_file}" ]] || [[ ! -s "${repos_file}" ]]; then
        return 0
    fi

    if ! command -v vcs &> /dev/null; then
        return 0
    fi

    log_info "Pulling latest changes for ${repos_file}..."
    vcs pull "${target_dir}" --workers "${PARALLEL_WORKERS}" || true
}

# ============================================================================
# Dependency Management
# ============================================================================
install_system_deps() {
    # apt/rosdep only applies on Linux outside of pixi
    if [[ "${IS_PIXI}" == "true" ]] || [[ "${IS_LINUX}" == "false" ]]; then
        log_info "Skipping apt/rosdep (not applicable — deps managed by pixi or not on Linux)"
        return 0
    fi

    log_info "Updating apt package lists..."
    sudo apt-get update -qq

    log_info "Updating rosdep database..."
    rosdep update --rosdistro=jazzy

    log_info "Installing ROS dependencies..."
    rosdep install \
        --from-paths src \
        --ignore-src \
        --rosdistro=jazzy \
        -y -r \
        --skip-keys "catkin roscpp lerobot trimesh[easy] simple-parsing cupy-cuda12x ctl_system_interface numpy_lessthan_2 ament_python"
}

setup_python_venv() {
    # pixi manages Python deps via pixi.toml
    if [[ "${IS_PIXI}" == "true" ]]; then
        log_info "Pixi environment detected — skipping venv setup (Python deps managed by pixi.toml)"
        return 0
    fi

    local venv_path="/home/ros/colcon_venv/venv"

    if [[ -f "${venv_path}/bin/activate" ]]; then
        log_info "Installing Python dependencies in venv..."
        source "${venv_path}/bin/activate"
        python3 -m pip install --upgrade pip --quiet
        python3 -m pip install pyserial feetech-servo-sdk --quiet
        log_info "Python dependencies installed"
    elif [[ -n "${VIRTUAL_ENV:-}" ]]; then
        log_info "Installing Python dependencies in active venv..."
        python3 -m pip install --upgrade pip --quiet
        python3 -m pip install pyserial feetech-servo-sdk --quiet
    else
        log_warn "No venv found. Install Python deps manually:"
        log_warn "  python3 -m pip install pyserial feetech-servo-sdk"
    fi
}

# ============================================================================
# Main
# ============================================================================
main() {
    cd "${WORKSPACE}"

    log_info "Setting up workspace at ${WORKSPACE}"

    # Clone repositories first (before pixi, since pixi.toml references libs/lerobot)
    import_repos "repos/libs.repos" "libs"
    import_repos "repos/src.repos" "src"

    # If pixi is available but env is not active, re-exec through pixi now that
    # repos are cloned (pixi.toml has an editable pypi dep on libs/lerobot)
    if command -v pixi &> /dev/null && [[ -z "${PIXI_PROJECT_ROOT:-}" ]]; then
        log_info "Repos cloned. Activating pixi environment..."
        exec pixi run "$0" "$@"
    fi

    # Fix detached heads before pulling
    fix_detached_heads "libs"
    fix_detached_heads "src"

    # Pull latest changes
    pull_repos "repos/libs.repos" "libs"
    pull_repos "repos/src.repos" "src"

    # Install dependencies
    install_system_deps
    setup_python_venv

    log_info "Setup complete! Run ./scripts/build.sh to build the workspace."
}

# Run if executed directly (not sourced)
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi