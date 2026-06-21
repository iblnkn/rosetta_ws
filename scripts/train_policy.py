#!/usr/bin/env python3
# Copyright 2025 Isaac Blankenau
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Train a LeRobot policy with optional multi-GPU and PEFT/LoRA support.

Any unrecognized arguments are forwarded directly to lerobot-train as
policy overrides (e.g. --policy.chunk_size=50 --policy.dim_model=256).

Examples:

  # Train ACT from scratch
  python scripts/train_policy.py --dataset iblnk/data --policy act --repo iblnk/model

  # Fine-tune SmolVLA from its default base
  python scripts/train_policy.py --dataset iblnk/data --policy smolvla --repo iblnk/model \\
      --pretrained default

  # LoRA fine-tune Pi0 with rank 32
  python scripts/train_policy.py --dataset iblnk/data --policy pi0 --repo iblnk/model \\
      --lora --lora-rank 32

  # Fine-tune from a specific checkpoint with architecture overrides
  python scripts/train_policy.py --dataset iblnk/data --policy act --repo iblnk/model \\
      --pretrained iblnk/act-base --policy.chunk_size=50 --policy.dim_model=256

  # Multi-GPU training
  python scripts/train_policy.py --dataset iblnk/data --policy act --repo iblnk/model \\
      --gpus 2 --precision bf16
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys

# ── Policy registry ──────────────────────────────────────────────────────────

POLICIES: dict[str, dict] = {
    "act":       {"display": "ACT",             "base": None},
    "diffusion": {"display": "DiffusionPolicy", "base": None},
    "smolvla":   {"display": "SmolVLA",         "base": "lerobot/smolvla_base"},
    "pi0":       {"display": "Pi0",             "base": "lerobot/pi0_base"},
    "pi0fast":   {"display": "Pi0Fast",         "base": "lerobot/pi0fast_base"},
    "pi05":      {"display": "Pi05",            "base": "lerobot/pi05_base"},
    "gr00t_n1":  {"display": "GR00T",           "base": None},
    "xvla":      {"display": "XVLA",            "base": "lerobot/xvla-base"},
    "wall_x":    {"display": "WALLOSS",         "base": "x-square-robot/wall-oss-flow"},
    "vqbet":     {"display": "VQBeT",           "base": None},
    "tdmpc":     {"display": "TDMPC",           "base": None},
}

# ── Helpers ──────────────────────────────────────────────────────────────────


def resolve_pretrained(policy: str, pretrained: str | None) -> str | None:
    """Resolve 'default' to the policy's base model, or return the path as-is."""
    if pretrained is None:
        return None
    if pretrained == "default":
        base = POLICIES[policy]["base"]
        if base is None:
            print(f"Note: '{policy}' has no default pretrained model. Training from scratch.")
            return None
        return base
    return pretrained


def build_args(ns: argparse.Namespace, passthrough: list[str]) -> list[str]:
    """Build the lerobot-train CLI arguments."""
    policy = ns.policy
    pretrained = resolve_pretrained(policy, ns.pretrained)
    display = POLICIES[policy]["display"]

    output_dir = ns.output_dir or "outputs/train"
    output_dir = os.path.join(output_dir, display)

    args: list[str] = [
        f"--dataset.repo_id={ns.dataset}",
        f"--policy.repo_id={ns.repo}",
        f"--output_dir={output_dir}",
        f"--batch_size={ns.batch_size}",
        f"--steps={ns.steps}",
        f"--wandb.enable={str(ns.wandb).lower()}",
    ]

    if ns.job_name:
        args.append(f"--job_name={ns.job_name}")

    # ── Training mode ────────────────────────────────────────────────────
    if ns.lora:
        if pretrained is None:
            base = POLICIES[policy]["base"]
            if base is None:
                print(f"Error: LoRA requires a pretrained model, but '{policy}' has no default.", file=sys.stderr)
                print(f"Pass --pretrained <path> or use a policy with a base model.", file=sys.stderr)
                sys.exit(1)
            pretrained = base

        args += [
            f"--policy.path={pretrained}",
            "--policy.output_features=null",
            "--policy.input_features=null",
            "--policy.optimizer_lr=1e-3",
            "--policy.scheduler_decay_lr=1e-4",
            "--peft.method_type=LORA",
            f"--peft.r={ns.lora_rank}",
        ]
    else:
        args.append(f"--policy.type={policy}")
        if pretrained:
            args.append(f"--policy.path={pretrained}")

    # ── Device / multi-GPU ───────────────────────────────────────────────
    if ns.gpus <= 1:
        args.append("--policy.device=cuda")

    # ── Passthrough overrides (e.g. --policy.chunk_size=50) ──────────────
    args += passthrough

    return args


def build_command(ns: argparse.Namespace, train_args: list[str]) -> list[str]:
    """Build the full command (with optional accelerate launcher)."""
    if ns.gpus > 1:
        lerobot_train = shutil.which("lerobot-train")
        if lerobot_train is None:
            print("Error: lerobot-train not found on PATH", file=sys.stderr)
            sys.exit(1)
        return [
            "accelerate", "launch",
            "--multi_gpu",
            f"--num_processes={ns.gpus}",
            f"--mixed_precision={ns.precision}",
            lerobot_train,
        ] + train_args
    else:
        return ["lerobot-train"] + train_args


def print_summary(ns: argparse.Namespace, cmd: list[str]) -> None:
    policy = ns.policy
    pretrained = resolve_pretrained(policy, ns.pretrained)
    display = POLICIES[policy]["display"]

    if ns.lora:
        mode = f"LoRA (r={ns.lora_rank})"
    elif pretrained:
        mode = "Full fine-tuning"
    else:
        mode = "From scratch"

    gpu_mode = f"{ns.gpus}-GPU {ns.precision}" if ns.gpus > 1 else "single GPU"

    print("=" * 50)
    print("LeRobot Policy Training")
    print("=" * 50)
    print(f"  Dataset:    {ns.dataset}")
    print(f"  Policy:     {display} ({policy})")
    print(f"  Training:   {mode}")
    if pretrained:
        print(f"  Pretrained: {pretrained}")
    print(f"  Repo:       {ns.repo}")
    print(f"  Steps:      {ns.steps}")
    print(f"  Batch size: {ns.batch_size}")
    print(f"  GPU:        {gpu_mode}")
    print(f"  W&B:        {ns.wandb}")
    print("=" * 50)
    print()
    print("Command:")
    print("  " + " ".join(cmd))
    print()


# ── Main ─────────────────────────────────────────────────────────────────────


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Train a LeRobot policy.",
        epilog=(
            "Any unrecognized arguments are forwarded to lerobot-train.\n"
            "Example: --policy.chunk_size=50 --policy.dim_model=256"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # ── Required ─────────────────────────────────────────────────────────
    parser.add_argument("--dataset", required=True, help="Dataset repo ID (e.g. iblnk/my_data)")
    parser.add_argument("--policy", required=True, choices=POLICIES.keys(), help="Policy architecture")
    parser.add_argument("--repo", required=True, help="HF repo ID for the trained model")

    # ── Training config ──────────────────────────────────────────────────
    parser.add_argument("--steps", type=int, default=100_000, help="Training steps (default: 100000)")
    parser.add_argument("--batch-size", type=int, default=8, help="Batch size (default: 8)")
    parser.add_argument("--output-dir", default=None, help="Output directory (default: outputs/train)")
    parser.add_argument("--job-name", default=None, help="Job name for logging")
    parser.add_argument("--wandb", action="store_true", default=False, help="Enable W&B logging")

    # ── Pretrained / fine-tuning ─────────────────────────────────────────
    parser.add_argument(
        "--pretrained", default=None, metavar="PATH",
        help="Pretrained model path or HF repo ID. Use 'default' for the policy's base model.",
    )

    # ── LoRA ─────────────────────────────────────────────────────────────
    parser.add_argument("--lora", action="store_true", default=False, help="Enable LoRA fine-tuning")
    parser.add_argument("--lora-rank", type=int, default=64, help="LoRA rank (default: 64)")

    # ── GPU ──────────────────────────────────────────────────────────────
    parser.add_argument("--gpus", type=int, default=1, help="Number of GPUs (default: 1)")
    parser.add_argument(
        "--precision", default="bf16", choices=["fp16", "bf16"],
        help="Mixed precision mode for multi-GPU (default: bf16)",
    )

    ns, passthrough = parser.parse_known_args()

    train_args = build_args(ns, passthrough)
    cmd = build_command(ns, train_args)

    print_summary(ns, cmd)

    os.execvp(cmd[0], cmd)


if __name__ == "__main__":
    sys.exit(main())
