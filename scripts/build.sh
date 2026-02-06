#!/bin/bash
# build.sh - Modern ROS 2 Jazzy build script with mixin support
#
# Usage:
#   ./scripts/build.sh                    # Default dev build
#   ./scripts/build.sh --mixin release    # Release build
#   ./scripts/build.sh --mixin debug test # Debug with tests
#   ./scripts/build.sh --list-mixins      # Show available mixins
#   ./scripts/build.sh --packages-select rosetta  # Build specific package
#
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${WORKSPACE:-$(dirname "${SCRIPT_DIR}")}"
MIXIN_DIR="${WORKSPACE}/colcon/mixin"

# ============================================================================
# Help & Mixin Listing
# ============================================================================
show_help() {
    cat << 'EOF'
ROS 2 Jazzy Build Script

Usage: ./scripts/build.sh [OPTIONS] [-- COLCON_ARGS]

Options:
  --mixin NAME [NAME...]   Use specified mixin(s) (can combine multiple)
  --list-mixins            List available mixins and exit
  --clean                  Clean build (cmake-clean-cache)
  --this                   Build only packages in current directory
  -h, --help               Show this help

Common mixins:
  dev          Development (debug, no tests, symlink-install) [DEFAULT]
  debug        Debug build with full symbols
  release      Optimized release build
  test         Enable testing
  ci           CI build (release + tests + linting)
  prod         Production (optimized, no debug/tests/tracing)
  asan         AddressSanitizer
  tsan         ThreadSanitizer

Examples:
  ./scripts/build.sh                           # Default dev build
  ./scripts/build.sh --mixin release           # Release build
  ./scripts/build.sh --mixin debug test        # Debug with tests
  ./scripts/build.sh --mixin asan              # With AddressSanitizer
  ./scripts/build.sh --clean --mixin release   # Clean release build
  ./scripts/build.sh -- --packages-select foo  # Pass args to colcon
EOF
}

list_mixins() {
    echo "Available mixins in ${MIXIN_DIR}:"
    echo ""
    if command -v yq &> /dev/null; then
        yq -r '.[] | "  \(.name)\t\(.description // "")"' "${MIXIN_DIR}/build.mixin.yaml" | column -t -s $'\t'
    else
        # Fallback: parse with grep/sed
        grep -E "^- name:|^  description:" "${MIXIN_DIR}/build.mixin.yaml" | \
        sed 'N;s/- name: \(.*\)\n  description: "\(.*\)"/  \1\t\2/' | \
        column -t -s $'\t'
    fi
    echo ""
    echo "Combine mixins: --mixin debug test asan"
}

# ============================================================================
# Argument Parsing
# ============================================================================
MIXINS=()
CLEAN_BUILD=false
BUILD_THIS=false
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --mixin)
            shift
            while [[ $# -gt 0 && ! "$1" =~ ^-- ]]; do
                MIXINS+=("$1")
                shift
            done
            ;;
        --list-mixins)
            list_mixins
            exit 0
            ;;
        --clean)
            CLEAN_BUILD=true
            shift
            ;;
        --this)
            BUILD_THIS=true
            shift
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        --)
            shift
            EXTRA_ARGS+=("$@")
            break
            ;;
        *)
            EXTRA_ARGS+=("$1")
            shift
            ;;
    esac
done

# Default mixin if none specified
[[ ${#MIXINS[@]} -eq 0 ]] && MIXINS=("dev")

# ============================================================================
# Virtual Environment Setup
# ============================================================================
setup_venv() {
    local venv_paths=(
        "/home/ros/colcon_venv/venv"
        "${VIRTUAL_ENV:-}"
    )
    
    for venv in "${venv_paths[@]}"; do
        if [[ -n "${venv}" && -f "${venv}/bin/activate" ]]; then
            source "${venv}/bin/activate"
            export PATH="${venv}/bin:$PATH"
            return 0
        fi
    done
    return 1
}

ensure_python_deps() {
    [[ -z "${VIRTUAL_ENV:-}" ]] && return 0
    
    local deps=("serial:pyserial" "feetech_servo_sdk:feetech-servo-sdk")
    for dep in "${deps[@]}"; do
        local module="${dep%%:*}"
        local package="${dep##*:}"
        if ! python3 -c "import ${module}" 2>/dev/null; then
            echo "Installing ${package} in venv..."
            python3 -m pip install --quiet "${package}"
        fi
    done
}

setup_venv || true
ensure_python_deps

# ============================================================================
# Install lerobot from libs/
# ============================================================================
if [[ -d "${WORKSPACE}/libs/lerobot" ]]; then
    echo "[INFO] Installing lerobot..."
    # Use constraints to prevent NumPy 2.x (NGC PyTorch compiled against NumPy 1.x)
    # and pin tokenizers for transformers compatibility
    pip install "${WORKSPACE}/libs/lerobot"
        # --constraint <(echo -e "numpy<2\ntokenizers>=0.21,<0.22") \
fi

# ============================================================================
# ROS 2 Environment
# ============================================================================
source /opt/ros/jazzy/setup.bash
[[ -f "${WORKSPACE}/install/setup.bash" ]] && source "${WORKSPACE}/install/setup.bash"

# ============================================================================
# Build
# ============================================================================
cd "${WORKSPACE}"

# Build mixin arguments
MIXIN_ARGS=()
if [[ -f "${MIXIN_DIR}/build.mixin.yaml" ]]; then
    MIXIN_ARGS+=("--mixin-files" "${MIXIN_DIR}"/*.yaml)
    MIXIN_ARGS+=("--mixin" "${MIXINS[@]}")
fi

# Clean build if requested
CLEAN_ARGS=()
${CLEAN_BUILD} && CLEAN_ARGS+=("--cmake-clean-cache")

# Build specific directory if --this
THIS_ARGS=()
${BUILD_THIS} && THIS_ARGS+=("--paths" "$(pwd)")

echo "════════════════════════════════════════════════════════════════════"
echo "Building with mixin(s): ${MIXINS[*]}"
echo "════════════════════════════════════════════════════════════════════"

colcon build \
    --continue-on-error \
    --parallel-workers "$(nproc)" \
    --merge-install \
    --event-handlers console_cohesion+ \
    --base-paths src \
    "${MIXIN_ARGS[@]}" \
    "${CLEAN_ARGS[@]}" \
    "${THIS_ARGS[@]}" \
    --packages-skip \
    "${EXTRA_ARGS[@]}"

echo ""
echo "Build complete. Source with: source install/setup.bash"