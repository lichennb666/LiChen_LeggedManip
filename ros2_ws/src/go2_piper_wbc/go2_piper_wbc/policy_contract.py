"""Single, validated control contract shared by deployment adapters."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class PolicyContract:
    version: int
    joint_names: tuple[str, ...]
    default_angles: np.ndarray
    stiffness: np.ndarray
    damping: np.ndarray
    effort_limits: np.ndarray
    action_scale: float
    action_clip: float
    observation_clip: float
    history_depth: int
    base_angular_velocity_scale: float
    joint_velocity_scale: float
    physics_rate_hz: float
    policy_rate_hz: float
    command_frame: str
    quaternion_order: str

    @classmethod
    def from_dict(cls, data: dict) -> "PolicyContract":
        names = tuple(str(name) for name in data["policy_joint_names"])

        def vector(key: str) -> np.ndarray:
            value = np.asarray(data[key], dtype=np.float32)
            if value.shape != (len(names),):
                raise ValueError(f"{key} must have {len(names)} entries, got {value.shape}")
            if not np.all(np.isfinite(value)):
                raise ValueError(f"{key} contains non-finite values")
            return value

        contract = cls(
            version=int(data["version"]),
            joint_names=names,
            default_angles=vector("default_angles"),
            stiffness=vector("stiffness"),
            damping=vector("damping"),
            effort_limits=vector("effort_limits"),
            action_scale=float(data["action_scale"]),
            action_clip=float(data["action_clip"]),
            observation_clip=float(data["observation_clip"]),
            history_depth=int(data["history_depth"]),
            base_angular_velocity_scale=float(data["base_angular_velocity_scale"]),
            joint_velocity_scale=float(data["joint_velocity_scale"]),
            physics_rate_hz=float(data["physics_rate_hz"]),
            policy_rate_hz=float(data["policy_rate_hz"]),
            command_frame=str(data["command_frame"]),
            quaternion_order=str(data["quaternion_order"]),
        )
        contract.validate()
        return contract

    def validate(self) -> None:
        if self.version != 1:
            raise ValueError(f"unsupported policy contract version {self.version}")
        if len(self.joint_names) != 18 or len(set(self.joint_names)) != 18:
            raise ValueError("policy_joint_names must contain 18 unique names")
        if self.history_depth != 3:
            raise ValueError("the exported policy requires three history frames")
        if min(self.action_scale, self.action_clip, self.observation_clip) <= 0.0:
            raise ValueError("clip and scale values must be positive")
        if min(self.physics_rate_hz, self.policy_rate_hz) <= 0.0:
            raise ValueError("control rates must be positive")
        ratio = self.physics_rate_hz / self.policy_rate_hz
        if abs(ratio - round(ratio)) > 1.0e-6:
            raise ValueError("physics rate must be an integer multiple of policy rate")
        if np.any(self.stiffness <= 0.0) or np.any(self.damping < 0.0):
            raise ValueError("invalid PD gains")
        if np.any(self.effort_limits <= 0.0):
            raise ValueError("effort limits must be positive")
        if self.command_frame != "link0_mixed_world_z":
            raise ValueError(f"unsupported command frame {self.command_frame}")
        if self.quaternion_order != "policy_wxyz_ros_xyzw":
            raise ValueError(f"unsupported quaternion convention {self.quaternion_order}")


def default_contract_path() -> Path:
    source_path = Path(__file__).resolve().parents[1] / "config" / "policy_contract.yaml"
    if source_path.is_file():
        return source_path
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("go2_piper_wbc")) / "config" / "policy_contract.yaml"
    except (ImportError, LookupError):
        raise FileNotFoundError("cannot locate go2_piper_wbc policy_contract.yaml")


def load_policy_contract(path: str | Path | None = None) -> PolicyContract:
    contract_path = Path(path) if path is not None else default_contract_path()
    with contract_path.open("r", encoding="utf-8") as stream:
        return PolicyContract.from_dict(json.load(stream))


def indices_for_names(
    source_names: tuple[str, ...] | list[str],
    target_names: tuple[str, ...] | list[str],
) -> list[int]:
    """Indices which reorder a source vector into target-name order."""
    if len(set(source_names)) != len(source_names):
        raise ValueError("source names are not unique")
    missing = [name for name in target_names if name not in source_names]
    if missing:
        raise ValueError(f"target names absent from source: {missing}")
    return [source_names.index(name) for name in target_names]
