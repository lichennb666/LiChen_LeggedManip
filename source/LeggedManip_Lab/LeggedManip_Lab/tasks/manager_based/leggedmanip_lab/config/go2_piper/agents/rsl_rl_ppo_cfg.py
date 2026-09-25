# Copyright (c) 2025-2026, Junjie Zhu.
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

from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import (
    RslRlCNNModelCfg,
    RslRlDistillationAlgorithmCfg,
    RslRlDistillationRunnerCfg,
    RslRlOnPolicyRunnerCfg,
    RslRlMLPModelCfg,
    RslRlPpoAlgorithmCfg,
)


@configclass
class VBCShapeEncoderModelCfg(RslRlMLPModelCfg):
    """Dedicated 1024-D PointNet++ branch used by the privileged teacher."""

    class_name: str = (
        "LeggedManip_Lab.tasks.manager_based.leggedmanip_lab.mdp.vbc_models:"
        "VBCShapeEncoderModel"
    )
    shape_group: str = "shape"
    shape_hidden_dim: int = 512
    shape_latent_dim: int = 128

# Flat
@configclass
class Go2PiperFlatPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 24
    check_for_nan = True
    max_iterations = 10000
    save_interval = 1000

    experiment_name = "go2_piper_flat"
    actor = RslRlMLPModelCfg(
        hidden_dims=[512, 256, 128],
        activation="elu",
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(
            init_std=1.0,
            std_type="log",
        ),
    )
    critic = RslRlMLPModelCfg(
        hidden_dims=[512, 256, 128],
        activation="elu",
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(
            init_std=1.0,
            std_type="log",
        ),
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.005,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=5e-4,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )

# WBC
@configclass
class Go2PiperWBCPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 24
    check_for_nan = True
    max_iterations = 4000
    save_interval = 250
    # Fine-tune from the exported WBC policy: use a conservative learning rate
    # so the domain-adapted run does not destroy the trained behaviour.
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.005,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=3e-4,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )

    experiment_name = "go2_piper_wbc"
    actor = RslRlMLPModelCfg(
        hidden_dims=[512, 256, 128],
        activation="elu",
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(
            init_std=1.0,
            std_type="log",
        ),
    )
    critic = RslRlMLPModelCfg(
        hidden_dims=[512, 256, 128],
        activation="elu",
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(
            init_std=1.0,
            std_type="log",
        ),
    )

# Stairs V4 - resume from V2.5 model, same network, improved terrain/rewards
@configclass
class Go2PiperStairsPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 24
    check_for_nan = True
    max_iterations = 15000
    save_interval = 500
    resume = True
    load_run = "2026-06-12_05-30-49"
    load_checkpoint = "model_9999.pt"

    experiment_name = "go2_piper_stairs"
    actor = RslRlMLPModelCfg(
        hidden_dims=[512, 256, 128],
        activation="elu",
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(
            init_std=1.0,
            std_type="log",
        ),
    )
    critic = RslRlMLPModelCfg(
        hidden_dims=[512, 256, 128],
        activation="elu",
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(
            init_std=1.0,
            std_type="log",
        ),
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.005,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=3e-4,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )


# Stairs V5 - fresh training with 8-subterrain mix + stair rewards.
# No hardcoded resume: warm-start from a flat policy via --finetune_policy if desired.
@configclass
class Go2PiperStairsV5PPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 24
    check_for_nan = True
    max_iterations = 8000
    save_interval = 500

    experiment_name = "go2_piper_stairs_v5"
    actor = RslRlMLPModelCfg(
        hidden_dims=[512, 256, 128],
        activation="elu",
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(
            init_std=1.0,
            std_type="log",
        ),
    )
    critic = RslRlMLPModelCfg(
        hidden_dims=[512, 256, 128],
        activation="elu",
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(
            init_std=1.0,
            std_type="log",
        ),
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.005,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=3e-4,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )


# [VBC-NEW] Teacher PPO configuration. The actor emits 10 high-level command
# values; the action term executes the frozen 18-D WBC underneath it.
@configclass
class Go2PiperVBCTeacherPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 32
    check_for_nan = True
    max_iterations = 10000
    save_interval = 250

    experiment_name = "go2_piper_vbc_teacher"
    actor = RslRlMLPModelCfg(
        hidden_dims=[256, 256, 128],
        activation="elu",
        obs_normalization=True,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(
            init_std=0.5,
            std_type="log",
        ),
    )
    critic = RslRlMLPModelCfg(
        hidden_dims=[256, 256, 128],
        activation="elu",
        obs_normalization=True,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(
            init_std=0.5,
            std_type="log",
        ),
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.005,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1e-4,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )


# [VBC-VISION-STUDENT] Paper-aligned online behavior cloning/DAgger entry.
# The student receives policy + images; the teacher receives the privileged
# 65-D state group and loads the actor from a trained VBC teacher checkpoint.
@configclass
class Go2PiperVBCStudentDistillationRunnerCfg(RslRlDistillationRunnerCfg):
    num_steps_per_env = 32
    check_for_nan = True
    max_iterations = 10000
    save_interval = 500
    experiment_name = "go2_piper_vbc_student"
    obs_groups = {
        "student": ["policy", "images"],
        "teacher": ["teacher"],
    }
    student = RslRlCNNModelCfg(
        hidden_dims=[256, 256, 128],
        activation="elu",
        obs_normalization=True,
        distribution_cfg=RslRlCNNModelCfg.GaussianDistributionCfg(
            init_std=0.10,
            std_type="log",
        ),
        cnn_cfg=RslRlCNNModelCfg.CNNCfg(
            output_channels=[32, 64, 128],
            kernel_size=[5, 3, 3],
            stride=[2, 2, 2],
            padding="zeros",
            norm="none",
            activation="elu",
            max_pool=False,
            global_pool="avg",
            flatten=True,
        ),
    )
    teacher = RslRlMLPModelCfg(
        hidden_dims=[256, 256, 128],
        activation="elu",
        obs_normalization=True,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(
            init_std=0.5,
            std_type="log",
        ),
    )
    algorithm = RslRlDistillationAlgorithmCfg(
        num_learning_epochs=1,
        learning_rate=1.0e-4,
        gradient_length=16,
        loss_type="mse",
        optimizer="adam",
        max_grad_norm=1.0,
    )


# [VBC-SHAPE] Paper-aligned teacher: the 1024-D frozen PointNet++ descriptor
# is compressed to 128-D before fusion with the 68-D privileged task state.
@configclass
class Go2PiperVBCShapeTeacherPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 32
    check_for_nan = True
    max_iterations = 10000
    save_interval = 250
    experiment_name = "go2_piper_vbc_shape_teacher"
    obs_groups = {
        "actor": ["policy", "shape"],
        "critic": ["critic", "shape"],
    }
    actor = VBCShapeEncoderModelCfg(
        hidden_dims=[512, 256, 128],
        activation="elu",
        obs_normalization=True,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(
            init_std=0.5,
            std_type="log",
        ),
    )
    critic = VBCShapeEncoderModelCfg(
        hidden_dims=[512, 256, 128],
        activation="elu",
        obs_normalization=True,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(
            init_std=0.5,
            std_type="log",
        ),
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.005,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1e-4,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )


# [VBC-MASK-DEPTH] Final high-level deployment policy.  Only this CNN student
# is exported; the shape teacher remains a training-time privileged expert.
@configclass
class Go2PiperVBCMaskDepthStudentDistillationRunnerCfg(RslRlDistillationRunnerCfg):
    num_steps_per_env = 32
    check_for_nan = True
    max_iterations = 10000
    save_interval = 500
    experiment_name = "go2_piper_vbc_mask_depth_student"
    obs_groups = {
        "student": ["policy", "images"],
        "teacher": ["teacher", "shape"],
    }
    student = RslRlCNNModelCfg(
        hidden_dims=[256, 256, 128],
        activation="elu",
        obs_normalization=True,
        distribution_cfg=RslRlCNNModelCfg.GaussianDistributionCfg(
            init_std=0.10,
            std_type="log",
        ),
        cnn_cfg=RslRlCNNModelCfg.CNNCfg(
            output_channels=[32, 64, 128],
            kernel_size=[5, 3, 3],
            stride=[2, 2, 2],
            padding="zeros",
            norm="none",
            activation="elu",
            max_pool=False,
            global_pool="avg",
            flatten=True,
        ),
    )
    teacher = VBCShapeEncoderModelCfg(
        hidden_dims=[512, 256, 128],
        activation="elu",
        obs_normalization=True,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(
            init_std=0.5,
            std_type="log",
        ),
    )
    algorithm = RslRlDistillationAlgorithmCfg(
        num_learning_epochs=1,
        learning_rate=1.0e-4,
        gradient_length=16,
        loss_type="mse",
        optimizer="adam",
        max_grad_norm=1.0,
    )
