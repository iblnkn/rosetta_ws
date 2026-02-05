#!/bin/bash
# Train LeRobot policy with optional multi-GPU and PEFT/LoRA support
#
# Usage: ./scripts/train_policy.sh <dataset_repo_id> <policy_type> <policy_repo_id> \
#        <output_dir> <job_name> <batch_size> <steps> <wandb> <multi_gpu_mode> \
#        <pretrained_config>
#
# Pretrained config format:
#   - none                    : Train from scratch (no pretrained model)
#   - default                 : Full fine-tune with default pretrained model for policy
#   - lora                    : LoRA fine-tune with default pretrained, rank 64
#   - lora:32                 : LoRA fine-tune with default pretrained, rank 32
#   - lora:org/model          : LoRA fine-tune with custom model, rank 64
#   - lora:org/model:32       : LoRA fine-tune with custom model, rank 32
#   - org/model               : Full fine-tune with custom pretrained model
#
# Supported policies and their default pretrained models:
#   - act: Action Chunking Transformer (no pretrained available)
#   - diffusion: Diffusion Policy (no pretrained available)
#   - smolvla: SmolVLA -> lerobot/smolvla_base
#   - pi0: Pi0 VLA -> lerobot/pi0_base
#   - pi0fast: Pi0Fast VLA -> lerobot/pi0fast_base
#   - pi05: Pi0.5 VLA -> lerobot/pi05_base
#   - gr00t_n1: NVIDIA GR00T N1.5 (requires custom path)
#   - xvla: X-VLA -> lerobot/xvla-base
#   - wall_x: WALL-OSS -> x-square-robot/wall-oss-flow
#   - vqbet: VQ-BeT (no pretrained available)
#   - tdmpc: TDMPC (no pretrained available)

set -e

DATASET_REPO_ID="$1"
POLICY_TYPE="$2"
POLICY_REPO_ID="$3"
OUTPUT_DIR_BASE="$4"
JOB_NAME="$5"
BATCH_SIZE="$6"
STEPS="$7"
WANDB_ENABLE="$8"
MULTI_GPU_MODE="$9"
PRETRAINED_CONFIG="${10}"

# Validate required args
if [ -z "$DATASET_REPO_ID" ] || [ -z "$POLICY_TYPE" ] || [ -z "$POLICY_REPO_ID" ]; then
    echo "Error: dataset_repo_id, policy_type, and policy_repo_id are required"
    exit 1
fi

# Map policy type to human-readable name for output directory
get_policy_display_name() {
    case "$1" in
        act) echo "ACT" ;;
        diffusion) echo "DiffusionPolicy" ;;
        smolvla) echo "SmolVLA" ;;
        pi0) echo "Pi0" ;;
        pi0fast) echo "Pi0Fast" ;;
        pi05) echo "Pi05" ;;
        gr00t_n1) echo "GR00T" ;;
        xvla) echo "XVLA" ;;
        wall_x) echo "WALLOSS" ;;
        vqbet) echo "VQBeT" ;;
        tdmpc) echo "TDMPC" ;;
        *) echo "$1" ;;
    esac
}

# Map policy type to default pretrained model
get_default_pretrained() {
    case "$1" in
        smolvla) echo "lerobot/smolvla_base" ;;
        pi0) echo "lerobot/pi0_base" ;;
        pi0fast) echo "lerobot/pi0fast_base" ;;
        pi05) echo "lerobot/pi05_base" ;;
        xvla) echo "lerobot/xvla-base" ;;
        wall_x) echo "x-square-robot/wall-oss-flow" ;;
        # Policies without pretrained models
        act|diffusion|vqbet|tdmpc|gr00t_n1) echo "" ;;
        *) echo "" ;;
    esac
}

# Check if pretrained model is compatible with policy type
check_compatibility() {
    local policy="$1"
    local pretrained="$2"

    # If no pretrained, always compatible
    if [ -z "$pretrained" ]; then
        return 0
    fi

    # Check for obvious mismatches
    case "$policy" in
        smolvla)
            if [[ "$pretrained" == *"pi0"* ]] || [[ "$pretrained" == *"xvla"* ]] || [[ "$pretrained" == *"wall"* ]]; then
                echo "Error: Cannot use $pretrained with $policy policy"
                echo "Use 'lerobot/smolvla_base' or 'default' for SmolVLA"
                return 1
            fi
            ;;
        pi0|pi0fast)
            if [[ "$pretrained" == *"smolvla"* ]] || [[ "$pretrained" == *"xvla"* ]] || [[ "$pretrained" == *"wall"* ]]; then
                echo "Error: Cannot use $pretrained with $policy policy"
                echo "Use 'lerobot/pi0_base', 'lerobot/pi0fast_base', or 'default'"
                return 1
            fi
            ;;
        pi05)
            if [[ "$pretrained" == *"smolvla"* ]] || [[ "$pretrained" == *"xvla"* ]] || [[ "$pretrained" == *"wall"* ]]; then
                echo "Error: Cannot use $pretrained with $policy policy"
                echo "Use 'lerobot/pi05_base' or 'default'"
                return 1
            fi
            ;;
        xvla)
            if [[ "$pretrained" == *"smolvla"* ]] || [[ "$pretrained" == *"pi0"* ]] || [[ "$pretrained" == *"wall"* ]]; then
                echo "Error: Cannot use $pretrained with $policy policy"
                echo "Use 'lerobot/xvla-base' or 'default'"
                return 1
            fi
            ;;
        wall_x)
            if [[ "$pretrained" == *"smolvla"* ]] || [[ "$pretrained" == *"pi0"* ]] || [[ "$pretrained" == *"xvla"* ]]; then
                echo "Error: Cannot use $pretrained with $policy policy"
                echo "Use 'x-square-robot/wall-oss-flow' or 'default'"
                return 1
            fi
            ;;
    esac
    return 0
}

# Parse pretrained config
# Returns: USE_LORA (0/1), PRETRAINED_PATH, LORA_RANK
parse_pretrained_config() {
    local config="$1"
    local policy="$2"
    local default_pretrained
    default_pretrained=$(get_default_pretrained "$policy")

    # Initialize outputs
    USE_LORA=0
    PRETRAINED_PATH=""
    LORA_RANK=64

    case "$config" in
        none|"")
            # Train from scratch
            USE_LORA=0
            PRETRAINED_PATH=""
            ;;
        default)
            # Full fine-tune with default pretrained
            USE_LORA=0
            if [ -z "$default_pretrained" ]; then
                echo "Note: Policy '$policy' has no default pretrained model. Training from scratch."
                PRETRAINED_PATH=""
            else
                PRETRAINED_PATH="$default_pretrained"
            fi
            ;;
        lora)
            # LoRA with default pretrained, default rank
            USE_LORA=1
            LORA_RANK=64
            if [ -z "$default_pretrained" ]; then
                echo "Error: LoRA requires a pretrained model, but '$policy' has no default."
                echo "Policies with pretrained models: smolvla, pi0, pi0fast, pi05, xvla, wall_x"
                exit 1
            fi
            PRETRAINED_PATH="$default_pretrained"
            ;;
        lora:*)
            # LoRA with custom settings
            USE_LORA=1
            local rest="${config#lora:}"

            # Check if it's just a rank number
            if [[ "$rest" =~ ^[0-9]+$ ]]; then
                # lora:32 format - default model, custom rank
                LORA_RANK="$rest"
                if [ -z "$default_pretrained" ]; then
                    echo "Error: LoRA requires a pretrained model, but '$policy' has no default."
                    exit 1
                fi
                PRETRAINED_PATH="$default_pretrained"
            elif [[ "$rest" =~ : ]]; then
                # lora:org/model:rank format
                PRETRAINED_PATH="${rest%:*}"
                LORA_RANK="${rest##*:}"
            else
                # lora:org/model format - custom model, default rank
                PRETRAINED_PATH="$rest"
                LORA_RANK=64
            fi
            ;;
        *)
            # Custom pretrained path for full fine-tuning
            USE_LORA=0
            PRETRAINED_PATH="$config"
            ;;
    esac
}

# Get display name and parse config
POLICY_DISPLAY_NAME=$(get_policy_display_name "$POLICY_TYPE")
parse_pretrained_config "$PRETRAINED_CONFIG" "$POLICY_TYPE"

# Check compatibility
if [ -n "$PRETRAINED_PATH" ]; then
    if ! check_compatibility "$POLICY_TYPE" "$PRETRAINED_PATH"; then
        exit 1
    fi
fi

# Construct output directory with policy name
OUTPUT_DIR="${OUTPUT_DIR_BASE}/${POLICY_DISPLAY_NAME}"

# Build base arguments
ARGS=(
    "--dataset.repo_id=$DATASET_REPO_ID"
    "--policy.repo_id=$POLICY_REPO_ID"
    "--output_dir=$OUTPUT_DIR"
    "--job_name=$JOB_NAME"
    "--batch_size=$BATCH_SIZE"
    "--steps=$STEPS"
    "--wandb.enable=$WANDB_ENABLE"
)

# Handle training mode
if [ "$USE_LORA" -eq 1 ]; then
    # PEFT/LoRA fine-tuning
    ARGS+=(
        "--policy.path=$PRETRAINED_PATH"
        "--policy.output_features=null"
        "--policy.input_features=null"
        "--policy.optimizer_lr=1e-3"
        "--policy.scheduler_decay_lr=1e-4"
        "--peft.method_type=LORA"
        "--peft.r=$LORA_RANK"
    )
    TRAINING_MODE="LoRA (r=$LORA_RANK)"
else
    # Regular training
    ARGS+=("--policy.type=$POLICY_TYPE")
    if [ -n "$PRETRAINED_PATH" ]; then
        ARGS+=("--policy.path=$PRETRAINED_PATH")
        TRAINING_MODE="Full fine-tuning"
    else
        TRAINING_MODE="From scratch"
    fi
fi

# Build command based on multi-GPU mode
case "$MULTI_GPU_MODE" in
    "single")
        ARGS+=("--policy.device=cuda")
        CMD="lerobot-train"
        ;;
    "2-gpu-fp16")
        CMD="accelerate launch --multi_gpu --num_processes=2 --mixed_precision=fp16 $(which lerobot-train)"
        ;;
    "2-gpu-bf16")
        CMD="accelerate launch --multi_gpu --num_processes=2 --mixed_precision=bf16 $(which lerobot-train)"
        ;;
    "4-gpu-fp16")
        CMD="accelerate launch --multi_gpu --num_processes=4 --mixed_precision=fp16 $(which lerobot-train)"
        ;;
    "4-gpu-bf16")
        CMD="accelerate launch --multi_gpu --num_processes=4 --mixed_precision=bf16 $(which lerobot-train)"
        ;;
    "8-gpu-fp16")
        CMD="accelerate launch --multi_gpu --num_processes=8 --mixed_precision=fp16 $(which lerobot-train)"
        ;;
    "8-gpu-bf16")
        CMD="accelerate launch --multi_gpu --num_processes=8 --mixed_precision=bf16 $(which lerobot-train)"
        ;;
    *)
        ARGS+=("--policy.device=cuda")
        CMD="lerobot-train"
        ;;
esac

echo "=========================================="
echo "LeRobot Policy Training"
echo "=========================================="
echo "Dataset:      $DATASET_REPO_ID"
echo "Policy:       $POLICY_DISPLAY_NAME ($POLICY_TYPE)"
echo "Training:     $TRAINING_MODE"
if [ -n "$PRETRAINED_PATH" ]; then
    echo "Pretrained:   $PRETRAINED_PATH"
fi
echo "Output Dir:   $OUTPUT_DIR"
echo "Policy Repo:  $POLICY_REPO_ID"
echo "Steps:        $STEPS"
echo "Batch Size:   $BATCH_SIZE"
echo "GPU Mode:     $MULTI_GPU_MODE"
echo "W&B:          $WANDB_ENABLE"
echo "=========================================="
echo ""
echo "Command:"
echo "$CMD ${ARGS[*]}"
echo ""

# Execute training
exec $CMD "${ARGS[@]}"
