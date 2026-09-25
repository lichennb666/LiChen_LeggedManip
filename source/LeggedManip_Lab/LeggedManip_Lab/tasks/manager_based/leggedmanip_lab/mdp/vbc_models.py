# Copyright (c) 2025-2026, Junjie Zhu.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""RSL-RL models used only by the paper-aligned Go2-Piper VBC tasks."""

from __future__ import annotations

import copy

import torch
import torch.nn as nn
from tensordict import TensorDict

from rsl_rl.models import MLPModel
from rsl_rl.modules import HiddenState, MLP


class VBCShapeEncoderModel(MLPModel):
    """MLP policy with the VBC paper's dedicated object-shape branch.

    The frozen 1024-D PointNet++ feature is encoded by ``1024 -> 512 -> 128``.
    Only the 128-D latent is concatenated with privileged robot/object state,
    keeping the large offline descriptor out of the main policy trunk.
    """

    def __init__(
        self,
        obs: TensorDict,
        obs_groups: dict[str, list[str]],
        obs_set: str,
        output_dim: int,
        hidden_dims: tuple[int, ...] | list[int] = (512, 256, 128),
        activation: str = "elu",
        obs_normalization: bool = False,
        distribution_cfg: dict | None = None,
        shape_group: str = "shape",
        shape_hidden_dim: int = 512,
        shape_latent_dim: int = 128,
    ) -> None:
        self.shape_group = shape_group
        self.shape_hidden_dim = shape_hidden_dim
        self.shape_latent_dim = shape_latent_dim
        super().__init__(
            obs=obs,
            obs_groups=obs_groups,
            obs_set=obs_set,
            output_dim=output_dim,
            hidden_dims=hidden_dims,
            activation=activation,
            obs_normalization=obs_normalization,
            distribution_cfg=distribution_cfg,
        )
        self.shape_encoder = MLP(
            self.shape_obs_dim,
            self.shape_latent_dim,
            [self.shape_hidden_dim],
            activation,
        )

    def _get_obs_dim(
        self,
        obs: TensorDict,
        obs_groups: dict[str, list[str]],
        obs_set: str,
    ) -> tuple[list[str], int]:
        active_obs_groups = obs_groups[obs_set]
        if self.shape_group not in active_obs_groups:
            raise ValueError(
                f"[VBC-SHAPE] Observation set '{obs_set}' must include "
                f"the '{self.shape_group}' group."
            )
        if len(obs[self.shape_group].shape) != 2:
            raise ValueError(
                f"[VBC-SHAPE] Expected a 1-D shape feature, got {obs[self.shape_group].shape}."
            )

        state_groups = [group for group in active_obs_groups if group != self.shape_group]
        state_dim = 0
        for group in state_groups:
            if len(obs[group].shape) != 2:
                raise ValueError(
                    f"[VBC-SHAPE] State group '{group}' must be 1-D, got {obs[group].shape}."
                )
            state_dim += obs[group].shape[-1]
        self.shape_obs_dim = obs[self.shape_group].shape[-1]
        if self.shape_obs_dim != 1024:
            raise ValueError(
                f"[VBC-SHAPE] Expected 1024-D PointNet++ input, got {self.shape_obs_dim}."
            )
        return state_groups, state_dim

    def _get_latent_dim(self) -> int:
        return self.obs_dim + self.shape_latent_dim

    def get_latent(
        self,
        obs: TensorDict,
        masks: torch.Tensor | None = None,
        hidden_state: HiddenState = None,
    ) -> torch.Tensor:
        del masks, hidden_state
        state = torch.cat([obs[group] for group in self.obs_groups], dim=-1)
        state = self.obs_normalizer(state)
        shape_latent = self.shape_encoder(obs[self.shape_group])
        return torch.cat((state, shape_latent), dim=-1)

    def update_normalization(self, obs: TensorDict) -> None:
        if self.obs_normalization:
            state = torch.cat([obs[group] for group in self.obs_groups], dim=-1)
            self.obs_normalizer.update(state)

    def as_jit(self) -> nn.Module:
        return _TorchVBCShapeEncoderModel(self)

    def as_onnx(self, verbose: bool = False) -> nn.Module:
        return _OnnxVBCShapeEncoderModel(self, verbose)


class _TorchVBCShapeEncoderModel(nn.Module):
    """Export wrapper accepting concatenated ``[state, shape]`` input."""

    def __init__(self, model: VBCShapeEncoderModel) -> None:
        super().__init__()
        self.state_dim = model.obs_dim
        self.obs_normalizer = copy.deepcopy(model.obs_normalizer)
        self.shape_encoder = copy.deepcopy(model.shape_encoder)
        self.mlp = copy.deepcopy(model.mlp)
        if model.distribution is not None:
            self.deterministic_output = model.distribution.as_deterministic_output_module()
        else:
            self.deterministic_output = nn.Identity()

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        state = self.obs_normalizer(x[..., : self.state_dim])
        shape = self.shape_encoder(x[..., self.state_dim :])
        return self.deterministic_output(self.mlp(torch.cat((state, shape), dim=-1)))

    @torch.jit.export
    def reset(self) -> None:
        pass


class _OnnxVBCShapeEncoderModel(_TorchVBCShapeEncoderModel):
    def __init__(self, model: VBCShapeEncoderModel, verbose: bool) -> None:
        super().__init__(model)
        self.verbose = verbose
        self.input_size = model.obs_dim + model.shape_obs_dim

    def get_dummy_inputs(self) -> tuple[torch.Tensor]:
        return (torch.zeros(1, self.input_size),)

    @property
    def input_names(self) -> list[str]:
        return ["obs"]

    @property
    def output_names(self) -> list[str]:
        return ["actions"]
