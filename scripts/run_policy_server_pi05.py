"""Wrapper around lerobot.async_inference.policy_server that pre-imports the
PI05 processor module so its registry entries (e.g.
``pi05_prepare_state_tokenizer_processor_step``) are available before any
processor pipeline is constructed.

Why this is needed
------------------
``PolicyProcessorPipeline.from_pretrained`` resolves each step by name through
``ProcessorStepRegistry``. The PI05 processor steps register themselves on
import via ``@ProcessorStepRegistry.register(...)`` decorators. The async
policy server does not import policy-specific processor modules, so when it
tries to load a checkpoint whose processor pipeline references a PI05 step,
registry lookup raises and processors silently fall back — leaving the policy
running on un-normalized raw observations.

Usage (drop-in replacement for ``python -m lerobot.async_inference.policy_server``)::

    python -m rosetta.scripts.run_policy_server_pi05 --host=127.0.0.1 --port=8080

Args are passed through to the upstream module unchanged.
"""
from __future__ import annotations

import runpy
import sys

# Side-effect import: registers PI05 processor steps with ProcessorStepRegistry.
import lerobot.policies.pi05.processor_pi05  # noqa: F401

# Install the image-decompression hook so any CompressedImagePayload entries in
# incoming observations are decoded back to numpy arrays right after
# pickle.loads. The hook is a no-op for uncompressed payloads, so this is safe
# to install unconditionally — the policy server works whether or not the
# client side enables compression.
from rosetta.common.compression.integration import wrap_policy_server

wrap_policy_server()


def main() -> None:
    runpy.run_module(
        "lerobot.async_inference.policy_server",
        run_name="__main__",
        alter_sys=True,
    )


if __name__ == "__main__":
    sys.exit(main())
