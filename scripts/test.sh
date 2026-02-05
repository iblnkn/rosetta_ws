#!/bin/bash
# test.sh - Run tests for ROS 2 Jazzy workspace
#
# Usage:
#   ./scripts/test.sh                     # Run all tests
#   ./scripts/test.sh rosetta             # Test specific package
#   ./scripts/test.sh rosetta --verbose   # With live output
#   ./scripts/test.sh --build             # Build with tests first
#   ./scripts/test.sh --build rosetta     # Build and test package
#
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${WORKSPACE:-$(dirname "${SCRIPT_DIR}")}"
MIXIN_DIR="${WORKSPACE}/colcon/mixin"

# ============================================================================
# Argument Parsing
# ============================================================================
BUILD_FIRST=false
VERBOSE=false
PACKAGE=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --build|-b)
            BUILD_FIRST=true
            shift
            ;;
        --verbose|-v)
            VERBOSE=true
            shift
            ;;
        -*)
            echo "Unknown option: $1"
            exit 1
            ;;
        *)
            PACKAGE="$1"
            shift
            ;;
    esac
done

# ============================================================================
# Environment
# ============================================================================
source /opt/ros/jazzy/setup.bash
[[ -f "${WORKSPACE}/install/setup.bash" ]] && source "${WORKSPACE}/install/setup.bash"

cd "${WORKSPACE}"

# ============================================================================
# Build with tests if requested
# ============================================================================
if ${BUILD_FIRST}; then
    echo "════════════════════════════════════════════════════════════════════"
    echo "Building with tests enabled..."
    echo "════════════════════════════════════════════════════════════════════"
    
    BUILD_ARGS=("--mixin" "debug" "test")
    [[ -n "${PACKAGE}" ]] && BUILD_ARGS+=("--" "--packages-select" "${PACKAGE}")
    
    ./scripts/build.sh "${BUILD_ARGS[@]}"
    echo ""
fi

# ============================================================================
# Run tests
# ============================================================================
echo "════════════════════════════════════════════════════════════════════"
echo "Running tests${PACKAGE:+ for ${PACKAGE}}..."
echo "════════════════════════════════════════════════════════════════════"

PKG_ARGS=()
[[ -n "${PACKAGE}" ]] && PKG_ARGS+=("--packages-select" "${PACKAGE}")

EVENT_HANDLER="console_cohesion+"
${VERBOSE} && EVENT_HANDLER="console_direct+"

colcon test \
    --merge-install \
    --event-handlers "${EVENT_HANDLER}" \
    "${PKG_ARGS[@]}"

echo ""
echo "════════════════════════════════════════════════════════════════════"
echo "Test Results"
echo "════════════════════════════════════════════════════════════════════"

colcon test-result --verbose "${PKG_ARGS[@]}" || true