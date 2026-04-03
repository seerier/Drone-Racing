# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.utils import configclass

from .rl_cfg import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg


@configclass
class QuadcopterPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    # Longer rollouts give better GAE estimates; 64 steps ≈ ~1.3 s at 50 Hz policy rate
    num_steps_per_env = 64
    max_iterations = 20000
    save_interval = 25
    experiment_name = "quadcopter_direct"
    empirical_normalization = False
    wandb_project = "ese651_quadcopter"  # Wandb project name for logging
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.8,
        # Wider actor: 28-dim obs → 256 → 256 → 128 → 4-dim action
        actor_hidden_dims=[256, 256, 128],
        # Deeper critic: better value estimates drive better advantage signals
        critic_hidden_dims=[512, 512, 256, 128],
        activation="elu",
        min_std=0.01,
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        # Positive entropy bonus encourages exploration and diverse racing lines
        entropy_coef=0.008,
        num_learning_epochs=5,
        # 8 mini-batches with 64 steps × 8192 envs → ~65k samples each, stable gradient
        num_mini_batches=8,
        learning_rate=3.0e-4,
        schedule="adaptive",
        # Higher gamma for longer-horizon lap-time optimisation (~4.6 s half-life at 50 Hz)
        gamma=0.997,
        lam=0.95,
        desired_kl=0.008,
        max_grad_norm=1.0,
    )
