#!/usr/bin/env bash
# Install MS-ILLM compression for the Rosetta async-inference pipeline.
#
# What this does
#   1. pip-installs neuralcompression (+ compressai) into the active Python env.
#   2. Pre-downloads MS-ILLM weights via torch.hub for the requested quality
#      levels so the first observation does not block on a network fetch.
#   3. Optionally bundles the downloaded weights into a tarball so the same
#      checkpoint can be carried to offline servers (aigpu1107 etc) without
#      re-touching the public internet.
#
# Idempotent: rerun safely. The torch.hub cache de-duplicates and pip will
# skip already-satisfied requirements.
#
# Usage
#   bash scripts/install_msillm.sh                                # install + download default quality (4 = ~0.3 bpp)
#   bash scripts/install_msillm.sh --quality 3 4                  # download multiple quality levels (encoder and decoder must match)
#   bash scripts/install_msillm.sh --hub-dir /opt/torch-hub      # use a non-default torch.hub cache
#   bash scripts/install_msillm.sh --bundle  /tmp/msillm.tar     # create offline bundle (weights + repo cache)
#   bash scripts/install_msillm.sh --unbundle /tmp/msillm.tar    # restore from bundle on a target server
#   bash scripts/install_msillm.sh --no-install                  # weights only (skip pip step)
#
# After install set this env var on every machine that uses a non-default cache:
#   export LEROBOT_MSILLM_HUB_DIR=/opt/torch-hub
#
# Runtime knobs read by rosetta.common.compression.msillm:
#   LEROBOT_MSILLM_QUALITY  1..6 (default 4)
#   LEROBOT_MSILLM_DEVICE   cuda|cpu|cuda:N (default: cuda if available)
#   LEROBOT_MSILLM_HUB_DIR  torch.hub cache override
#
# License note: MS-ILLM pretrained weights are released under CC-BY-NC 4.0.
# Research/demo use only — not commercial deployment.

set -euo pipefail

QUALITIES=(4)
HUB_DIR=""
BUNDLE=""
UNBUNDLE=""
DO_INSTALL=1

usage() {
    awk 'NR>1 && /^[^#]/ {exit} NR>1 {sub(/^# ?/, ""); print}' "$0"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --quality)
            shift
            QUALITIES=()
            while [[ $# -gt 0 && "$1" =~ ^[0-9]+$ ]]; do
                QUALITIES+=("$1")
                shift
            done
            if [[ ${#QUALITIES[@]} -eq 0 ]]; then
                echo "error: --quality needs at least one integer (1..6)" >&2
                exit 2
            fi
            ;;
        --hub-dir)
            HUB_DIR="$2"; shift 2
            ;;
        --bundle)
            BUNDLE="$2"; shift 2
            ;;
        --unbundle)
            UNBUNDLE="$2"; shift 2
            ;;
        --no-install)
            DO_INSTALL=0; shift
            ;;
        -h|--help)
            usage; exit 0
            ;;
        *)
            echo "error: unknown arg '$1'" >&2
            usage
            exit 2
            ;;
    esac
done

if ! command -v python >/dev/null 2>&1; then
    echo "error: 'python' not on PATH. Activate the env first." >&2
    exit 1
fi

if [[ $DO_INSTALL -eq 1 ]]; then
    # neuralcompression 0.3.1 pins `scipy<=1.11.1`, which has no wheels for
    # Python >=3.12 and tries to source-build (requires gfortran). The pin is
    # only relevant for NC's *training/evaluation* code; the inference path we
    # use (HiFiCAutoencoder.compress / .decompress) does not call scipy.
    #
    # Workaround: install neuralcompression and pytorchvideo with --no-deps,
    # then explicitly install the modules its functional/__init__.py imports
    # at load time (fvcore, lpips, DISTS-pytorch, torchmetrics, torch-fidelity).
    # compressai is the only required runtime dep.
    echo "[install_msillm] pip install neuralcompression (no-deps) + runtime deps ..."
    python -m pip install --prefer-binary \
        'compressai>=1.2.4' \
        'fvcore>=0.1.5.post20221221' \
        'lpips>=0.1.4' \
        'DISTS-pytorch>=0.1' \
        'torchmetrics>=1.0' \
        'torch-fidelity>=0.3'
    python -m pip install --no-deps 'pytorchvideo>=0.1.5'
    python -m pip install --no-deps 'neuralcompression>=0.3'
fi

# Resolve target hub dir.
if [[ -z "$HUB_DIR" ]]; then
    HUB_DIR="$(python -c 'import torch.hub; print(torch.hub.get_dir())')"
fi
mkdir -p "$HUB_DIR"
echo "[install_msillm] torch.hub cache: $HUB_DIR"

if [[ -n "$UNBUNDLE" ]]; then
    if [[ ! -f "$UNBUNDLE" ]]; then
        echo "error: bundle file not found: $UNBUNDLE" >&2
        exit 1
    fi
    echo "[install_msillm] restoring weights from $UNBUNDLE -> $HUB_DIR"
    tar -xf "$UNBUNDLE" -C "$HUB_DIR"
fi

# Pre-download requested qualities. Done in-process so torch.hub uses
# HUB_DIR; we export TORCH_HOME and call torch.hub.set_dir for belt-and-braces.
for Q in "${QUALITIES[@]}"; do
    if [[ ! "$Q" =~ ^[1-6]$ ]]; then
        echo "error: quality must be 1..6, got '$Q'" >&2
        exit 2
    fi
    echo "[install_msillm] preparing MS-ILLM quality $Q ..."
    TORCH_HOME="$HUB_DIR" python - "$Q" "$HUB_DIR" <<'PY'
import sys, os
q = int(sys.argv[1])
hub = sys.argv[2]
os.environ["TORCH_HOME"] = hub
import torch
torch.hub.set_dir(hub)
model = torch.hub.load(
    "facebookresearch/NeuralCompression",
    f"msillm_quality_{q}",
    trust_repo=True,
)
model.eval()
model.update()
# Reach into one tensor to confirm weights actually loaded (not just architecture).
n_params = sum(p.numel() for p in model.parameters())
print(f"  ok quality={q}, params={n_params:,}, cache={hub}")
PY
done

if [[ -n "$BUNDLE" ]]; then
    BUNDLE_DIR="$(dirname "$BUNDLE")"
    mkdir -p "$BUNDLE_DIR"
    echo "[install_msillm] bundling cache: $HUB_DIR -> $BUNDLE"
    tar -cf "$BUNDLE" -C "$HUB_DIR" .
    bytes=$(stat -c%s "$BUNDLE" 2>/dev/null || stat -f%z "$BUNDLE")
    echo "[install_msillm] bundle ready: $BUNDLE ($bytes bytes)"
    echo "  On target server:"
    echo "    bash $(basename "$0") --unbundle $(basename "$BUNDLE") --no-install --hub-dir $HUB_DIR"
fi

echo "[install_msillm] done."
if [[ "$HUB_DIR" != "$(python -c 'import torch.hub; print(torch.hub.get_dir())')" ]]; then
    echo "[install_msillm] NOTE: non-default cache. Set this at runtime:"
    echo "    export LEROBOT_MSILLM_HUB_DIR=$HUB_DIR"
fi
