# Autonomous Drone Racing with Deep Reinforcement Learning

![Python](https://img.shields.io/badge/Python-3.10+-3776AB?logo=python&logoColor=white)
![PyTorch](https://img.shields.io/badge/PyTorch-2.x-EE4C2C?logo=pytorch&logoColor=white)
![Isaac Sim](https://img.shields.io/badge/NVIDIA-Isaac%20Lab-76B900?logo=nvidia&logoColor=white)
![RL](https://img.shields.io/badge/RL-PPO-blue)
![Envs](https://img.shields.io/badge/Parallel%20Envs-8192-orange)

> I designed a complete reinforcement learning pipeline -- observation space, multi-component reward function, three-stage curriculum, and a novel arc maneuver -- to train a PPO agent that races a Crazyflie 2.1 quadrotor through a 7-gate 3D circuit, completing 3 full laps (21 gate passages) at maximum speed. Built in NVIDIA Isaac Lab with 8,192 parallel environments and domain randomization for policy robustness, designed for future sim-to-real deployment. This project tackles a core challenge in embodied AI: designing RL systems that produce agile, robust behavior for autonomous agents operating in complex 3D environments.

**Horizontal Bypass Approach:**


https://github.com/user-attachments/assets/e2a53d59-dc04-474b-be40-fd2a3bbf1dff


**Vertical Bypass Approach:**



https://github.com/user-attachments/assets/20d34114-196e-410d-8120-a9f9068dbfe0


> Local files: [`results/horizontal.mp4`](results/horizontal.mp4) | [`results/vertical.mp4`](results/vertical.mp4)

---

## What I Built

- **Full circuit completion** -- the agent reliably completes all 21 gate passages (3 laps) at high speed
- **Solved the shared-gate problem** -- gates 3 and 6 occupy the same physical position but face opposite directions; I designed a 3-waypoint arc maneuver that lets the agent approach from the correct side each time
- **10-component reward function** -- mixed sparse gate-passage signals with dense shaping terms, carefully balancing exploration incentives and penalty schedules to avoid reward hacking
- **Three-stage curriculum** -- independently schedules domain randomization, approach distance, and speed scaling, each progressing at its natural rate to avoid cascading instability
- **Robustness via domain randomization** -- systematic randomization of thrust, drag, and PID gains across 8,192 parallel environments in Isaac Lab, designed for future sim-to-real transfer

---

## Approach

### Environment and Task

The environment is built on **NVIDIA Isaac Lab** (Isaac Sim 4.5), which provides GPU-accelerated rigid-body simulation, parallel scene rendering, and USD-based asset composition. This enables training across 8,192 environments simultaneously on a single GPU — each with independent physics, randomized dynamics, and gate configurations. The track is a 7-gate "powerloop" circuit; the agent must pass all gates in order for 3 laps.

| | |
|---|---|
| **Simulation platform** | NVIDIA Isaac Lab (Isaac Sim 4.5) |
| **Action space** | 4D continuous -- Collective Thrust + Body Rates (CTBR) |
| **Observation space** | 31-dimensional (ego state, 3-gate lookahead, arc guidance, temporal context) |
| **Parallel envs** | 8,192 |
| **Policy rate** | 50 Hz |

I chose CTBR over position control because direct thrust and body-rate commands enable the aggressive maneuvers (sharp banking, rapid altitude changes) that racing demands.

The 31-dimensional observation includes body-frame velocities, gate-relative positions for the next 3 gates, the current gate's approach direction, distance and altitude, lap progress, previous actions, and an arc phase signal. Gate-relative encoding makes the observation invariant to global position, improving generalization across gates.

### Reward Design

I designed a 10-component reward function from scratch, mixing sparse gate-passage signals with dense shaping terms:

**Sparse Rewards**

| Component | Scale | Formula | Rationale |
|---|---|---|---|
| Gate pass | +15.0 | Binary trigger on correct-side passage within aperture | Primary training signal — large to dominate over dense shaping noise |
| Gate speed bonus | 1.0 | `v² / 10`, clamped [0, 10] | Rewards aggressive traversals; quadratic scaling incentivizes high-speed entries |

**Dense Shaping**

| Component | Scale | Formula | Rationale |
|---|---|---|---|
| Progress toward gate | 2.0 | `clamp(d_{t-1} - d_t, -5, 5)`; 1.5x boost during arc | Potential-based shaping — dense gradient between sparse gate rewards |
| Velocity toward gate | 0.5 | `clamp(v · dir, -5, 10)`; scaled by speed curriculum | Encourages fast racing; curriculum-gated to avoid premature aggression |
| Time penalty | -0.05 | Constant per step; curriculum ramps 0.5x to 1.5x | Small cost that grows over training to push speed over caution |
| Angular velocity penalty | -0.01 | `-||omega_body||` | Discourages excessive spinning for flight stability |
| Crash penalty | -0.5 | Triggered when contact force > 1e-8 N | Per-timestep contact penalty |
| Wrong-side proximity | -0.5 | Dense penalty for approaching gate from exit side | Gentle nudge (reduced from -2.0) to avoid creating a fear barrier at gate planes |
| Exit-side repulsion | -0.5 | Repulsive field near exit side of ALL gates | Prevents brush-and-turn exploits at gates 2→3 and elsewhere |
| Action smoothness | -0.03 | `-0.03 × ||a_t - a_{t-1}||` (added directly) | Reduces actuator jitter for policy robustness |

**Terminal Rewards**

| Event | Reward | Rationale |
|---|---|---|
| Death (crash / out of bounds) | -5.0 | Intentionally cheap — encourages exploration of risky zones (e.g., the powerloop after gate 2) |
| Wrong-side gate entry | -15.0 (stacks with death: -20 total) | Must be much more expensive than crashing, otherwise the agent learns to exploit wrong-side passages as a "cheaper" alternative |

**Dense Penalty Warmup:** Wrong-side proximity and exit-side repulsion are disabled for the first 500 iterations. This lets the agent discover gate-passing behavior without a fear barrier, then the penalties activate to shape approach trajectories.

### Curriculum Learning

I designed three independent curricula that each progress at their natural rate:

| Curriculum | Start | End | Duration | Why |
|---|---|---|---|---|
| Domain randomization | 20% range | 100% range | 2,000 iter | Learn basic flight before perturbed physics |
| Approach distance | 0.5-1.5 m | 1.2-3.5 m | 2,000 iter | Master close-range accuracy first |
| Speed scaling | 0.5x | 1.5x | 3,000 iter | Route learning before speed optimization |

Decoupling these was a key design decision -- coupling them caused training instability when one axis became too difficult before the others were ready. The speed curriculum deliberately finishes after DR, so the agent masters robust dynamics before racing at full speed.

### Key Innovation: Arc Maneuver

Gates 3 and 6 occupy the same physical position but face opposite directions. A naive policy targeting the next gate's position gets confused here -- the positional signal is identical for both passages.

I solved this with a 3-waypoint clockwise arc at constant altitude. The observation space includes an arc phase signal that guides the agent through intermediate waypoints around the shared gate, ensuring it approaches from the correct direction each time. The arc is encoded as an **observation signal** rather than hard-coded actions, so the policy learns the maneuver shape end-to-end and produces smoother trajectories.

### Domain Randomization

To ensure the trained policy is robust to manufacturing tolerances, battery voltage variations, and environmental disturbances, I randomize physical parameters each episode. This domain randomization strategy is designed for future sim-to-real transfer to real Crazyflie hardware:

| Parameter | Range |
|---|---|
| Thrust-to-weight ratio | +/- 5% |
| Drag coefficients | 0.5x -- 2.0x |
| PID gains (proportional, integral) | +/- 15% |
| PID gains (derivative) | +/- 30% |

---

## Architecture and Hyperparameters

I use an asymmetric actor-critic PPO design where the critic is deliberately larger:

| Network | Architecture | Activation | Rationale |
|---|---|---|---|
| **Actor** | 31 -> 256 -> 256 -> 128 -> 4 | ELU + Tanh output | Compact — only needs to map observations to 4D actions |
| **Critic** | 31 -> 512 -> 512 -> 256 -> 128 -> 1 | ELU | 3x more parameters — value estimation across a 21-gate circuit with curriculum-dependent dynamics is a harder regression problem than the policy itself |

### PPO Hyperparameters

| Parameter | Value | Rationale |
|---|---|---|
| Parallel environments | 8,192 | Massive parallelism for stable gradient estimation |
| Steps per env per update | 64 | ~1.3s at 50 Hz; sufficient horizon for GAE |
| Mini-batches | 8 | ~65K samples each, balancing compute and gradient noise |
| Learning epochs | 5 | |
| Learning rate | 3e-4 | Adaptive schedule via KL divergence targeting |
| Clip range (epsilon) | 0.2 | Standard PPO clipping |
| Entropy coefficient | 0.008 | Maintains exploration for diverse racing lines |
| Discount (gamma) | 0.997 | Half-life ~4.6s at 50 Hz; credits rewards 100+ steps ahead for long-horizon lap optimization |
| GAE lambda | 0.95 | |
| Desired KL | 0.008 | LR auto-reduces when policy updates become too aggressive |
| Max gradient norm | 1.0 | Gradient clipping for training stability |
| Initial noise std | 0.8 | Wide initial exploration over the action space |
| Minimum noise std | 0.01 | Prevents premature convergence to deterministic policy |

**Effective batch size:** 8,192 envs x 64 steps = **524,288 transitions per update**.

---

## Project Structure

```
DroneRacing/
├── pyproject.toml                             # Project metadata and Isaac Lab dependencies
├── setup.py                                   # Package installation
├── LICENSE                                    # BSD-3-Clause
│
├── config/
│   └── extension.toml                         # Isaac Lab extension registration
│
├── scripts/
│   ├── rsl_rl/
│   │   ├── train_race.py                      # Training entry point — reward scales, env init, PPO runner
│   │   ├── play_race.py                       # Evaluation — checkpoint loading, video recording, model export
│   │   ├── cli_args.py                        # CLI argument parsing for RSL-RL integration
│   │   └── batch_training.sh                  # Multi-seed batch training (3 seeds)
│   ├── slurm/
│   │   ├── train.sh                           # SLURM GPU job submission for training
│   │   └── play.sh                            # SLURM job submission for evaluation
│   └── util/
│       ├── env.sh                             # Environment setup (Isaac Sim 4.5 installation)
│       ├── drone_run.sh                       # Convenience launcher for training
│       └── drone_play.sh                      # Convenience launcher for evaluation
│
├── results/
│   ├── horizontal_best.mp4                    # Best run — side view
│   └── vertical_best.mp4                      # Best run — top view
│
├── src/
│   ├── isaac_quad_sim2real/                    # Custom Isaac Lab extension
│   │   └── tasks/race/config/crazyflie/
│   │       ├── quadcopter_env.py              # Environment — rigid-body physics, PID control, gate geometry
│   │       ├── quadcopter_strategies.py       # Strategy — reward computation, observations, resets, curricula, arc maneuver
│   │       └── agents/
│   │           ├── rl_cfg.py                  # Base RL configuration dataclasses
│   │           └── rsl_rl_ppo_cfg.py          # PPO hyperparameters — network architecture, algorithm config
│   │
│   └── third_parties/
│       └── rsl_rl_local/                      # Custom RSL-RL v2.2.3 — modified PPO implementation
│           └── rsl_rl/
│               ├── algorithms/ppo.py          # PPO algorithm — loss computation, gradient updates
│               ├── modules/
│               │   ├── actor_critic.py        # Actor-critic network — policy and value function heads
│               │   └── normalizer.py          # Input/output normalization utilities
│               ├── runners/
│               │   └── on_policy_runner.py    # Training loop — rollout collection, logging, checkpointing
│               ├── storage/
│               │   └── rollout_storage.py     # Trajectory buffer — GAE advantage computation
│               └── utils/
│                   ├── wandb_utils.py         # Weights & Biases logging integration
│                   └── utils.py               # General utilities — checkpointing, device management
│
└── usd/
    ├── cf2x.usda                              # Crazyflie 2.1 quadrotor USD model (physics, mesh, joints)
    └── gate.usda                              # Racing gate USD mesh
```

---

## Getting Started

### Prerequisites

- NVIDIA Isaac Lab (Isaac Sim 4.5+)
- Python 3.10+
- CUDA GPU with sufficient VRAM for 8,192 parallel environments

### Train

```bash
python scripts/rsl_rl/train_race.py \
    --task Isaac-Quadcopter-Race-v0 \
    --num_envs 8192 \
    --max_iterations 5000 \
    --headless
```

### Evaluate

```bash
python scripts/rsl_rl/play_race.py \
    --task Isaac-Quadcopter-Race-v0 \
    --num_envs 1 \
    --load_run <run_dir> \
    --checkpoint best_model.pt \
    --video
```

---

## Design Decisions

- **Terminal penalty stacking** (-20 for wrong-side vs -5 for crash) prevents the agent from exploiting wrong-side passages as a cheaper alternative to correct approaches
- **Independent curricula over coupled** -- decoupling distance, speed, and DR schedules lets each progress at its natural rate without cascading instability
- **All-gate scanning** every timestep prevents agents from "backdooring" non-target gates for free progress reward
- **Arc phase as observation signal** rather than reward shaping -- produces smoother trajectories since the policy learns the maneuver end-to-end
- **Asymmetric actor-critic** (3x more critic parameters) stabilizes training for this long-horizon, curriculum-dependent task

---

**Tech Stack:** NVIDIA Isaac Lab (Isaac Sim 4.5) | PyTorch | PPO (RSL-RL) | USD Scene Composition | Crazyflie 2.1
