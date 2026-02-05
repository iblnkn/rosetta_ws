#!/bin/bash
# Parallel ROS2 bag to LeRobot dataset conversion
# Uses sharding to process multiple bags concurrently

set -e

# Default values
RAW_DIR="${1:-/workspaces/rosetta_ws/datasets/bags}"
CONTRACT="${2:-/workspaces/rosetta_ws/src/action/rosetta/contracts/so_101.yaml}"
REPO_ID="${3:-so_101_dataset}"
ROOT="${4:-/workspaces/rosetta_ws/datasets/lerobot}"
NUM_SHARDS="${5:-4}"

echo "========================================"
echo "Parallel Bag Conversion"
echo "========================================"
echo "Raw dir:    $RAW_DIR"
echo "Contract:   $CONTRACT"
echo "Repo ID:    $REPO_ID"
echo "Root:       $ROOT"
echo "Shards:     $NUM_SHARDS"
echo "========================================"

# Count bags
NUM_BAGS=$(find "$RAW_DIR" -name "metadata.yaml" | wc -l)
echo "Found $NUM_BAGS bags to convert"

if [ "$NUM_BAGS" -eq 0 ]; then
    echo "Error: No bags found in $RAW_DIR"
    exit 1
fi

# Ensure PYTHONPATH includes rosetta
export PYTHONPATH="/workspaces/rosetta_ws/src/action/rosetta:${PYTHONPATH}"

# Use LEROBOT_VIDEO_CODEC if set, otherwise default to libx264 for speed
export LEROBOT_VIDEO_CODEC="${LEROBOT_VIDEO_CODEC:-libx264}"
echo "Video codec: $LEROBOT_VIDEO_CODEC"

# Create temporary directory for shard datasets
SHARD_ROOT="${ROOT}/.shards"
mkdir -p "$SHARD_ROOT"

echo ""
echo "Phase 1: Converting bags in parallel ($NUM_SHARDS shards)..."
echo ""

# Launch parallel shard processes
pids=()
for ((i=0; i<NUM_SHARDS; i++)); do
    SHARD_REPO_ID="${REPO_ID}_shard_${i}"
    echo "Starting shard $i -> $SHARD_REPO_ID"

    python3 -m rosetta.port_bags \
        --raw-dir "$RAW_DIR" \
        --contract "$CONTRACT" \
        --repo-id "$SHARD_REPO_ID" \
        --root "$SHARD_ROOT" \
        --num-shards "$NUM_SHARDS" \
        --shard-index "$i" \
        2>&1 | sed "s/^/[shard-$i] /" &

    pids+=($!)
done

echo ""
echo "Waiting for ${#pids[@]} shard processes..."

# Wait for all shards and check for failures
failed=0
for pid in "${pids[@]}"; do
    if ! wait "$pid"; then
        echo "Shard process $pid failed"
        ((failed++))
    fi
done

if [ "$failed" -gt 0 ]; then
    echo "Error: $failed shard(s) failed"
    exit 1
fi

echo ""
echo "Phase 2: Aggregating shard datasets..."
echo ""

# Aggregate datasets using lerobot
python3 << EOF
import sys
import logging
sys.path.insert(0, "/workspaces/rosetta_ws/src/action/rosetta")

logging.basicConfig(level=logging.INFO, format="%(message)s")

from pathlib import Path
from lerobot.datasets.aggregate import aggregate_datasets
from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata

shard_root = Path("$SHARD_ROOT")
output_root = Path("$ROOT")
repo_id = "$REPO_ID"
num_shards = $NUM_SHARDS

# List shard repo IDs
shard_repo_ids = [f"{repo_id}_shard_{i}" for i in range(num_shards)]

# Filter to only existing shards
# Each shard dataset is at shard_root/shard_id/, so we pass shard_root as the root
existing_shards = []
for shard_id in shard_repo_ids:
    shard_path = shard_root / shard_id / "meta" / "info.json"
    if shard_path.exists():
        existing_shards.append(shard_id)
        print(f"Found shard: {shard_id}")
    else:
        print(f"Warning: Shard {shard_id} not found at {shard_path.parent.parent}, skipping")

if not existing_shards:
    print("Error: No shard datasets found")
    sys.exit(1)

print(f"Aggregating {len(existing_shards)} shards into {repo_id}...")

# Each root must point to the actual dataset directory (shard_root/shard_id)
# LeRobotDatasetMetadata expects root to be the dataset dir, not parent
# aggr_root must also be the full output path (output_root/repo_id)
aggregate_datasets(
    repo_ids=existing_shards,
    aggr_repo_id=repo_id,
    roots=[shard_root / shard_id for shard_id in existing_shards],
    aggr_root=output_root / repo_id,
)

print(f"Done! Dataset saved to: {output_root / repo_id}")
EOF

echo ""
echo "Phase 3: Cleanup..."
echo ""

# Auto-cleanup if not interactive, otherwise prompt
if [ -t 0 ]; then
    read -p "Remove temporary shard directories? [y/N] " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf "$SHARD_ROOT"
        echo "Cleaned up shard directories"
    else
        echo "Shards kept at: $SHARD_ROOT"
    fi
else
    # Non-interactive: auto cleanup
    rm -rf "$SHARD_ROOT"
    echo "Cleaned up shard directories"
fi

echo ""
echo "========================================"
echo "Conversion complete!"
echo "Dataset: $ROOT/$REPO_ID"
echo "========================================"
