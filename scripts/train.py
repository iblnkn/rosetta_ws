#!/usr/bin/env python3
"""Train a LeRobot policy from a YAML configuration file.

Reads a per-model YAML config, translates it to lerobot-train CLI arguments,
and handles multi-GPU wrapping, resume, and Hub upload transparently.

Usage:
    python scripts/train.py configs/training/act.yaml --dataset my-org/data
    python scripts/train.py configs/training/smolvla.yaml --dataset data --steps 5000
    python scripts/train.py --resume models/ACT
    python scripts/train.py configs/training/pi0.yaml --dataset data --dry-run
"""

import argparse
import os
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path

import yaml

# ── Display names for output directory ───────────────────────────────────────

DISPLAY_NAMES = {
    "act": "ACT",
    "diffusion": "DiffusionPolicy",
    "smolvla": "SmolVLA",
    "pi0fast": "Pi0Fast",
    "pi0_fast": "Pi0Fast",
    "pi05": "Pi05",
    "pi0": "Pi0",
    "gr00t": "GR00T",
    "groot": "GR00T",
    "xvla": "XVLA",
    "wall_x": "WALLOSS",
    "wall_oss": "WALLOSS",
    "vqbet": "VQBeT",
    "tdmpc": "TDMPC",
}

# Keys the script handles directly (everything else under `policy:` is passthrough)
MANAGED_KEYS = {
    "dataset",
    "output_dir",
    "hub_repo_id",
    "batch_size",
    "steps",
    "device",
    "num_gpus",
    "mixed_precision",
    "wandb",
    "job_name",
    "policy_type",
    "pretrained_path",
    "lora_rank",
    "policy",
    "extra_args",
}


# ── Config loading ───────────────────────────────────────────────────────────


def load_config(yaml_path: str) -> dict:
    """Load a training config YAML."""
    path = Path(yaml_path)
    if not path.exists():
        sys.exit(f"Error: Config file not found: {path}")
    with open(path) as f:
        config = yaml.safe_load(f)
    if config is None:
        sys.exit(f"Error: Empty config file: {path}")
    return config


def apply_cli_overrides(config: dict, args: argparse.Namespace) -> dict:
    """Override config values with CLI arguments (if provided)."""
    override_map = {
        "dataset": "dataset",
        "steps": "steps",
        "batch_size": "batch_size",
        "device": "device",
        "num_gpus": "num_gpus",
        "mixed_precision": "mixed_precision",
        "wandb": "wandb",
        "hub_repo_id": "hub_repo_id",
        "push_to_hub": "push_to_hub",
        "lora_rank": "lora_rank",
        "job_name": "job_name",
        "output_dir": "output_dir",
    }
    for config_key, arg_key in override_map.items():
        val = getattr(args, arg_key, None)
        if val is not None:
            config[config_key] = val
    return config


# ── Validation ───────────────────────────────────────────────────────────────


def validate_config(config: dict) -> None:
    """Validate the config before building arguments."""
    has_type = bool(config.get("policy_type"))
    has_path = bool(config.get("pretrained_path"))

    if has_type and has_path:
        sys.exit(
            "Error: Config has both 'policy_type' and 'pretrained_path'.\n"
            "Use 'policy_type' for training from scratch, or "
            "'pretrained_path' for fine-tuning. Not both."
        )
    if not has_type and not has_path:
        sys.exit(
            "Error: Config must have either 'policy_type' (train from scratch) "
            "or 'pretrained_path' (fine-tune)."
        )

    dataset = config.get("dataset", "")
    if not dataset:
        sys.exit(
            "Error: 'dataset' is required.\n"
            "  Set it in the YAML or pass --dataset on the command line.\n"
            "  Examples:\n"
            "    --dataset /path/to/lerobot/dataset\n"
            "    --dataset my-org/my-dataset"
        )

    lora_rank = config.get("lora_rank")
    if lora_rank and int(lora_rank) > 0 and not has_path:
        sys.exit(
            "Error: 'lora_rank' requires 'pretrained_path'.\n"
            "LoRA can only be used when fine-tuning a pretrained model."
        )


# ── Argument building ────────────────────────────────────────────────────────


def get_display_name(config: dict) -> str:
    """Get display name for the model (used in output directory)."""
    if config.get("policy_type"):
        return DISPLAY_NAMES.get(config["policy_type"], config["policy_type"])
    # Infer from pretrained path (check longest keys first to avoid partial matches)
    path = config.get("pretrained_path", "")
    normalized = path.lower().replace("-", "_")
    for key in sorted(DISPLAY_NAMES, key=len, reverse=True):
        if key in normalized:
            return DISPLAY_NAMES[key]
    return Path(path).name


def build_lerobot_args(config: dict) -> list[str]:
    """Translate config dict to lerobot-train CLI arguments."""
    args = []

    # Dataset
    args.append(f"--dataset.repo_id={config['dataset']}")

    # Policy type or pretrained path (mutually exclusive)
    if config.get("policy_type"):
        args.append(f"--policy.type={config['policy_type']}")
    if config.get("pretrained_path"):
        args.append(f"--policy.path={config['pretrained_path']}")

    # Output directory: models/<ModelName>/<dataset>_<MMDD> (with _2, _3, ... if exists)
    display_name = get_display_name(config)
    dataset_short = Path(config["dataset"]).name
    date_tag = datetime.now().strftime("%m%d")
    output_dir = Path(config.get("output_dir", "models"))
    base = output_dir / display_name / f"{dataset_short}_{date_tag}"
    full_output_dir = base
    n = 2
    while full_output_dir.exists():
        full_output_dir = base.parent / f"{base.name}_{n}"
        n += 1
    args.append(f"--output_dir={full_output_dir}")

    # Hub upload
    hub_repo_id = config.get("hub_repo_id", "")
    push_to_hub = config.get("push_to_hub")
    if hub_repo_id:
        args.append(f"--policy.repo_id={hub_repo_id}")
        if push_to_hub is not False:
            args.append("--policy.push_to_hub=true")
        else:
            args.append("--policy.push_to_hub=false")
    else:
        args.append("--policy.push_to_hub=false")

    # Training parameters
    args.append(f"--batch_size={config.get('batch_size', 8)}")
    args.append(f"--steps={config.get('steps', 100000)}")
    args.append(f"--wandb.enable={str(config.get('wandb', False)).lower()}")

    # Device (single GPU only; multi-GPU handled by accelerate)
    num_gpus = int(config.get("num_gpus", 1))
    if num_gpus <= 1:
        args.append(f"--policy.device={config.get('device', 'cuda')}")

    # Job name
    job_name = config.get("job_name", "")
    if job_name:
        args.append(f"--job_name={job_name}")

    # LoRA / PEFT
    lora_rank = config.get("lora_rank")
    if lora_rank and int(lora_rank) > 0:
        args.append("--peft.method_type=LORA")
        args.append(f"--peft.r={lora_rank}")
        args.append("--policy.output_features=null")
        args.append("--policy.input_features=null")

    # Policy-specific passthrough (policy: dict → --policy.<key>=<value>)
    policy_overrides = config.get("policy", {})
    if policy_overrides and isinstance(policy_overrides, dict):
        for key, value in policy_overrides.items():
            args.append(f"--policy.{key}={value}")

    # Extra verbatim args
    extra = config.get("extra_args", [])
    if isinstance(extra, str):
        extra = extra.split()
    if extra:
        args.extend(extra)

    return args


def build_command(config: dict, lerobot_args: list[str]) -> list[str]:
    """Build the full command, wrapping with accelerate if multi-GPU."""
    num_gpus = int(config.get("num_gpus", 1))

    if num_gpus > 1:
        mixed_precision = config.get("mixed_precision", "fp16")
        lerobot_bin = shutil.which("lerobot-train")
        if not lerobot_bin:
            sys.exit("Error: lerobot-train not found in PATH")
        cmd = [
            "accelerate", "launch",
            "--multi_gpu",
            f"--num_processes={num_gpus}",
        ]
        if mixed_precision:
            cmd.append(f"--mixed_precision={mixed_precision}")
        cmd.append(lerobot_bin)
    else:
        cmd = ["lerobot-train"]

    cmd.extend(lerobot_args)
    return cmd


# ── Resume ───────────────────────────────────────────────────────────────────


def find_checkpoint(model_dir: str) -> Path:
    """Find the last checkpoint's train_config.json in a model directory.

    Searches for the standard LeRobot checkpoint layout:
      <model_dir>/checkpoints/last/pretrained_model/train_config.json
    """
    base = Path(model_dir)
    candidates = [
        base / "checkpoints" / "last" / "pretrained_model" / "train_config.json",
        base / "last" / "pretrained_model" / "train_config.json",
        base / "pretrained_model" / "train_config.json",
    ]
    for path in candidates:
        if path.exists():
            return path

    sys.exit(
        f"Error: Could not find train_config.json in {base}\n"
        f"Expected: {base}/checkpoints/last/pretrained_model/train_config.json\n"
        f"Is this a valid model output directory?"
    )


def build_resume_command(model_dir: str, args: argparse.Namespace) -> list[str]:
    """Build resume command from a model directory."""
    config_path = find_checkpoint(model_dir)

    lerobot_args = [
        f"--config_path={config_path}",
        "--resume=true",
    ]

    num_gpus = getattr(args, "num_gpus", None) or 1
    if num_gpus > 1:
        mixed_precision = getattr(args, "mixed_precision", None) or "fp16"
        lerobot_bin = shutil.which("lerobot-train")
        if not lerobot_bin:
            sys.exit("Error: lerobot-train not found in PATH")
        cmd = [
            "accelerate", "launch",
            "--multi_gpu",
            f"--num_processes={num_gpus}",
            f"--mixed_precision={mixed_precision}",
            lerobot_bin,
        ]
    else:
        cmd = ["lerobot-train"]

    cmd.extend(lerobot_args)
    return cmd


# ── Summary ──────────────────────────────────────────────────────────────────


def print_summary(config: dict, command: list[str], *, is_resume: bool = False) -> None:
    """Print a formatted summary before execution."""
    w = 60
    print()
    print("=" * w)
    if is_resume:
        print("  RESUME TRAINING")
    else:
        print("  LEROBOT POLICY TRAINING")
    print("=" * w)

    if not is_resume:
        display_name = get_display_name(config)
        pretrained = config.get("pretrained_path", "")
        lora_rank = config.get("lora_rank")

        if pretrained:
            if lora_rank and int(lora_rank) > 0:
                mode = f"LoRA fine-tuning (r={lora_rank})"
            else:
                mode = "Full fine-tuning"
        else:
            mode = "From scratch"

        print(f"  Model:        {display_name}")
        print(f"  Mode:         {mode}")
        if pretrained:
            print(f"  Pretrained:   {pretrained}")
        print(f"  Dataset:      {config['dataset']}")
        print(f"  Steps:        {config.get('steps', 100000)}")
        print(f"  Batch size:   {config.get('batch_size', 8)}")

        num_gpus = int(config.get("num_gpus", 1))
        if num_gpus > 1:
            mp = config.get("mixed_precision", "fp16")
            print(f"  GPUs:         {num_gpus} ({mp})")
        else:
            print(f"  Device:       {config.get('device', 'cuda')}")

        print(f"  W&B:          {config.get('wandb', False)}")

        hub = config.get("hub_repo_id", "")
        if hub:
            print(f"  Hub upload:   {hub}")

    print("-" * w)
    # Print command wrapped at reasonable width
    cmd_str = " ".join(command)
    print(f"  {cmd_str}")
    print("=" * w)
    print()


# ── Main ─────────────────────────────────────────────────────────────────────


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Train a LeRobot policy from a YAML config file.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
examples:
  %(prog)s configs/training/act.yaml --dataset my-org/data
  %(prog)s configs/training/smolvla.yaml --dataset data --steps 5000
  %(prog)s --resume models/ACT
  %(prog)s configs/training/pi0.yaml --dataset data --dry-run
""",
    )

    parser.add_argument(
        "config",
        nargs="?",
        default=None,
        help="Path to training config YAML (e.g., configs/training/act.yaml)",
    )
    parser.add_argument(
        "--resume",
        metavar="MODEL_DIR",
        help="Resume training from a model output directory (e.g., models/ACT)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the command without executing",
    )

    # CLI overrides (all optional; values come from YAML by default)
    overrides = parser.add_argument_group("overrides (optional)")
    overrides.add_argument("--dataset", help="Dataset: local path or HF repo ID")
    overrides.add_argument("--steps", type=int, help="Training steps")
    overrides.add_argument("--batch_size", type=int, help="Batch size")
    overrides.add_argument("--device", help="Device: cuda, cpu, cuda:0, mps")
    overrides.add_argument("--num_gpus", type=int, help="Number of GPUs (>1 uses accelerate)")
    overrides.add_argument("--mixed_precision", help="Mixed precision: fp16 or bf16")
    overrides.add_argument("--wandb", type=lambda v: v.lower() == "true", help="Enable W&B: true/false")
    overrides.add_argument("--hub_repo_id", help="HuggingFace Hub repo for upload")
    overrides.add_argument("--push_to_hub", type=lambda v: v.lower() == "true", help="Push to Hub: true/false")
    overrides.add_argument("--lora_rank", type=int, help="LoRA rank (0 to disable)")
    overrides.add_argument("--job_name", help="Job name for logging")
    overrides.add_argument("--output_dir", help="Base output directory")

    return parser.parse_args()


def main():
    args = parse_args()

    # Set CUDA memory config
    os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", "expandable_segments:True")

    if args.resume:
        # ── Resume mode ──────────────────────────────────────────────
        command = build_resume_command(args.resume, args)
        print_summary({}, command, is_resume=True)

        if args.dry_run:
            print("[dry-run] Would execute the above command.")
            return

        result = subprocess.run(command)
        sys.exit(result.returncode)

    elif args.config:
        # ── Normal training mode ─────────────────────────────────────
        config = load_config(args.config)
        config = apply_cli_overrides(config, args)
        validate_config(config)

        lerobot_args = build_lerobot_args(config)
        command = build_command(config, lerobot_args)
        print_summary(config, command)

        if args.dry_run:
            print("[dry-run] Would execute the above command.")
            return

        result = subprocess.run(command)
        sys.exit(result.returncode)

    else:
        print(
            "Error: Provide a config YAML or --resume.\n"
            "\n"
            "  Train:  python scripts/train.py configs/training/act.yaml --dataset my-org/data\n"
            "  Resume: python scripts/train.py --resume models/ACT\n"
            "\n"
            "Available configs:",
            file=sys.stderr,
        )
        configs_dir = Path(__file__).parent.parent / "configs" / "training"
        if configs_dir.exists():
            for f in sorted(configs_dir.glob("*.yaml")):
                # Read the first comment line for description
                with open(f) as fh:
                    for line in fh:
                        line = line.strip()
                        if line.startswith("#") and not line.startswith("# =="):
                            desc = line.lstrip("# ").strip()
                            if desc:
                                print(f"  {f.name:<20s} {desc}", file=sys.stderr)
                                break
        sys.exit(1)


if __name__ == "__main__":
    main()
