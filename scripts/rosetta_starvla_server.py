#!/usr/bin/env python
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
Rosetta's starVLA policy server: starVLA's server_policy.py plus server-side
normalization of the input state.

Stock starVLA un-normalizes the output actions but does not normalize the input
state; its eval clients normalize before sending. The Rosetta ROS runner is
starVLA-free (websocket client in the `default` env) and sends raw state, so a
state-conditioned model trained on normalized state would see skew. We fix it by
normalizing the input state here using the same ComposedModalityTransform the
model was trained with (PolicyNormProcessor.transform.apply), which covers
min_max / mean_std / percentile / binary / rotation.

Run in the `starvla` env (starVLA on PYTHONPATH), not the ROS env:

    pixi run -e starvla python scripts/rosetta_starvla_server.py \
        --ckpt_path <checkpoint.pt> --port 10093 --use_bf16

The ROS client (backend:=starvla) is unchanged: it sends raw state and this
server normalizes it. No effect for vision+language-only models, which send no
state.
"""

from __future__ import annotations

import argparse
import logging
import socket
from typing import Any, Dict, List, Optional

import numpy as np
import torch

from deployment.model_server.policy_wrapper import PolicyServerWrapper
from deployment.model_server.tools.websocket_policy_server import WebsocketPolicyServer


def _normalize_state(proc, state: Any) -> np.ndarray:
    """Normalize a raw state vector with the training-time transform.

    The forward direction of PolicyNormProcessor.unapply_actions: split the flat
    state by per-key dims (declaration order), run the composed transform's apply,
    concatenate back. Output keeps the input shape (1-D -> 1-D, (T, S) -> (T, S)).
    """
    arr = np.asarray(state, dtype=np.float32)
    squeeze = arr.ndim == 1
    if squeeze:
        arr = arr[None, :]  # (1, S)

    state_keys: List[str] = list(proc.state_keys)
    key_dims: Dict[str, int] = dict(getattr(proc, '_state_key_dims', {}))

    data: Dict[str, torch.Tensor] = {}
    cursor = 0
    for key in state_keys:
        dim = key_dims.get(key, 1)
        data[key] = torch.as_tensor(arr[..., cursor:cursor + dim], dtype=torch.float32)
        cursor += dim
    if cursor != arr.shape[-1]:
        raise ValueError(
            f'state width {arr.shape[-1]} != sum of state_key_dims {cursor} '
            f'(state_keys={state_keys}, dims={key_dims}).'
        )

    out = proc.transform.apply(data)  # training-time forward transform

    parts: List[np.ndarray] = []
    for key in state_keys:
        v = out[key]
        if isinstance(v, torch.Tensor):
            v = v.detach().cpu().numpy()
        parts.append(np.asarray(v, dtype=np.float32))
    norm = np.concatenate(parts, axis=-1)
    return norm[0] if squeeze else norm


class RosettaPolicyServerWrapper(PolicyServerWrapper):
    """starVLA server wrapper that also normalizes input state."""

    def predict_action(self, examples: List[dict], unnorm_key: Optional[str] = None, **kwargs):
        key = unnorm_key if unnorm_key is not None else self._default_unnorm_key
        if key is None and len(self._available_unnorm_keys) == 1:
            key = self._available_unnorm_keys[0]

        if key is not None:
            proc = self._get_processor(key)
            if proc.state_keys:  # only state-conditioned models carry state
                for ex in examples:
                    if ex.get('state') is not None:
                        ex['state'] = _normalize_state(proc, ex['state'])

        return super().predict_action(examples, unnorm_key=unnorm_key, **kwargs)


def build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ckpt_path', type=str, required=True)
    p.add_argument('--port', type=int, default=10093)
    p.add_argument('--use_bf16', action='store_true')
    p.add_argument('--unnorm_key', type=str, default=None)
    p.add_argument('--idle_timeout', type=int, default=1800, help='-1 = never close')
    return p


def main() -> None:
    logging.basicConfig(level=logging.INFO, force=True)
    args = build_argparser().parse_args()

    wrapper = RosettaPolicyServerWrapper(
        ckpt_path=args.ckpt_path,
        device='cuda',
        use_bf16=args.use_bf16,
        unnorm_key=args.unnorm_key,
    )
    hostname = socket.gethostname()
    logging.info('Rosetta starVLA server (input-state normalization ON): host=%s', hostname)
    logging.info('metadata=%s', wrapper.metadata)

    server = WebsocketPolicyServer(
        policy=wrapper,
        host='0.0.0.0',
        port=args.port,
        idle_timeout=args.idle_timeout,
        metadata=wrapper.metadata,
    )
    server.serve_forever()


if __name__ == '__main__':
    main()
