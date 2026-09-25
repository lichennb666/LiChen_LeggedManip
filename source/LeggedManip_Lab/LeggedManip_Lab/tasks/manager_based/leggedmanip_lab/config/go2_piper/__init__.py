import gymnasium as gym
from . import agents

# Flat
gym.register(
    id="GO2-PIPER-Flat",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.flat_env_cfg:Go2PiperFlatEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:Go2PiperFlatPPORunnerCfg",
    },
)

gym.register(
    id="GO2-PIPER-Flat-Play",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.flat_env_cfg:Go2PiperFlatEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:Go2PiperFlatPPORunnerCfg",
    },
)

# WBC
gym.register(
    id="GO2-PIPER-WBC",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.wbc_env_cfg:Go2PiperWBCEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:Go2PiperWBCPPORunnerCfg",
    },
)

gym.register(
    id="GO2-PIPER-WBC-Play",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.wbc_env_cfg:Go2PiperWBCEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:Go2PiperWBCPPORunnerCfg",
    },
)

# Stairs
gym.register(
    id="GO2-PIPER-Stairs",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.stairs_env_cfg:Go2PiperStairsEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:Go2PiperStairsV5PPORunnerCfg",
    },
)

gym.register(
    id="GO2-PIPER-Stairs-Play",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.stairs_env_cfg:Go2PiperStairsEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:Go2PiperStairsV5PPORunnerCfg",
    },
)

# [VBC-NEW] Hierarchical VBC teacher. This registration is additive: the
# original Flat/WBC/Stairs tasks and their checkpoints keep their old IDs.
gym.register(
    id="GO2-PIPER-VBC-Teacher",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.vbc_env_cfg:Go2PiperVBCTeacherEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:Go2PiperVBCTeacherPPORunnerCfg",
    },
)

gym.register(
    id="GO2-PIPER-VBC-Teacher-Play",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.vbc_env_cfg:Go2PiperVBCTeacherEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:Go2PiperVBCTeacherPPORunnerCfg",
    },
)

# [VBC-VISION-STUDENT] Additive visual student rollout.  It is intentionally
# a separate task because its CNN observation contract is not compatible with
# the old 210-D WBC actor or the privileged teacher actor.
gym.register(
    id="GO2-PIPER-VBC-Student",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.vbc_env_cfg:Go2PiperVBCStudentEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:Go2PiperVBCStudentDistillationRunnerCfg",
    },
)

# [VBC-SHAPE] Paper-aligned multi-object teacher.  Kept separate from the
# original 65-D single-cube teacher so old checkpoints remain loadable.
gym.register(
    id="GO2-PIPER-VBC-Teacher-Shape",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.vbc_env_cfg:Go2PiperVBCShapeTeacherEnvCfg",
        "rsl_rl_cfg_entry_point": (
            f"{agents.__name__}.rsl_rl_ppo_cfg:Go2PiperVBCShapeTeacherPPORunnerCfg"
        ),
    },
)

# [VBC-MASK-DEPTH] Deployable visual student distilled from the shape teacher.
gym.register(
    id="GO2-PIPER-VBC-Student-MaskDepth",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": (
            f"{__name__}.vbc_env_cfg:Go2PiperVBCMaskDepthStudentEnvCfg"
        ),
        "rsl_rl_cfg_entry_point": (
            f"{agents.__name__}.rsl_rl_ppo_cfg:"
            "Go2PiperVBCMaskDepthStudentDistillationRunnerCfg"
        ),
    },
)
