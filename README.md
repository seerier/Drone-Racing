# Autonomous Drone Racing with Deep Reinforcement Learning

![Python](https://img.shields.io/badge/Python-3.10+-3776AB?logo=python&logoColor=white)
![PyTorch](https://img.shields.io/badge/PyTorch-2.x-EE4C2C?logo=pytorch&logoColor=white)
![Isaac Sim](https://img.shields.io/badge/NVIDIA-Isaac%20Lab-76B900?logo=nvidia&logoColor=white)
![RL](https://img.shields.io/badge/RL-PPO-blue)
![Envs](https://img.shields.io/badge/Parallel%20Envs-8192-orange)

> I designed a complete reinforcement learning pipeline -- observation space, multi-component reward function, three-stage curriculum, and a novel arc maneuver -- to train a PPO agent that races a Crazyflie 2.1 quadrotor through a 7-gate 3D circuit, completing 3 full laps (21 gate passages) at maximum speed. Built in NVIDIA Isaac Lab with 8,192 parallel environments and domain randomization for policy robustness, then **deployed zero-shot to a real Crazyflie 2.1**. This project tackles a core challenge in embodied AI: designing RL systems that produce agile, robust behavior for autonomous agents operating in complex 3D environments.

## Real-World Deployment (Stage 3)

The same policy trained in Isaac Lab transfers directly to a physical Crazyflie 2.1 quadrotor, flying the full 7-gate powerloop circuit -- including the shared-gate arc maneuver -- with no real-world fine-tuning. Onboard PID accepts collective-thrust + body-rate commands at 50 Hz from the policy network; observations are reconstructed from VICON motion-capture state in the 36D Vineet body-frame format used during training.



https://github.com/user-attachments/assets/87ce08ea-6633-47c1-8a01-a9de1610ed7a



> Local file: [`results/powerloop.mp4`](results/powerloop.mp4)

**Sim videos -- Horizontal Bypass Approach:**


https://github.com/user-attachments/assets/e2a53d59-dc04-474b-be40-fd2a3bbf1dff


**Sim videos -- Vertical Bypass Approach:**



https://github.com/user-attachments/assets/20d34114-196e-410d-8120-a9f9068dbfe0


> Local files: [`results/horizontal.mp4`](results/horizontal.mp4) | [`results/vertical.mp4`](results/vertical.mp4)

---

## What I Built

- **Full circuit completion in sim and on hardware** -- the agent reliably completes all 21 gate passages (3 laps of a 7-gate powerloop) in Isaac Lab, and the same checkpoint flies a physical Crazyflie 2.1 zero-shot
- **Solved the shared-gate problem** -- gates 3 and 6 share the same `(x, y, z)` but face opposite directions; a 3-waypoint horizontal arc steers the agent around the colocated pair on the correct side each lap
- **11-component reward function** -- sparse gate-passage signals plus dense progress, velocity, smoothness, and exit-side penalties, with a 500-iteration warmup before the directional penalties switch on
- **Three independent curricula** -- domain randomization, approach distance, and speed scaling each progress on their own schedule, decoupled to avoid cascading instability
- **Sim-to-real bridge** -- the sim emits the same 36D body-frame "Vineet" observation that the real Crazyflie controller reconstructs from VICON, and the same `[512, 512, 256, 128]` ELU MLP runs on both ends — no fine-tuning, no distillation

---

## Approach

### Environment and Task

The environment is built on **NVIDIA Isaac Lab** (Isaac Sim 4.5), which provides GPU-accelerated rigid-body simulation, parallel scene rendering, and USD-based asset composition. The track (`quadcopter_env.py:416-424`) is a 7-gate `powerloop` circuit where gates 3 and 6 sit at the same `(0.625, 0, 0.75)` position with opposite yaw, forcing the agent to thread the same physical opening from two different directions on each lap. An evaluation episode is 3 full laps = 21 gate passages.

| | |
|---|---|
| **Simulation platform** | NVIDIA Isaac Lab (Isaac Sim 4.5), USD scene composition |
| **Action space** | 4D continuous CTBR (collective thrust + roll/pitch/yaw body rates), clamped to [-1, 1] |
| **Body-rate scaling** | ±100°/s roll, ±100°/s pitch, ±200°/s yaw (matches the Crazyflie `crtpRateSetpoint`, deg/s — *not* rad/s) |
| **Observation space** | 36D Vineet body-frame format -- identical in sim and on the real drone |
| **Default parallel envs** | 4,096 (config); training runs use 8,192 on an A40 |
| **Sim rate / policy rate / inner PID rate** | 500 Hz / 50 Hz / 500 Hz |
| **Episode length** | 30 s (1,500 policy steps) |

CTBR was chosen over position control because direct thrust and body-rate commands let the policy execute the sharp banking and altitude transitions a powerloop demands; the inner PID still runs at 500 Hz on the IMU loop.

The 36D observation (`quadcopter_strategies.py:416-475`) is constructed identically in sim and on the real drone, which is what makes zero-shot transfer possible:

| Block | Dim | Source in sim | Source on real Crazyflie |
|---|---|---|---|
| Body-frame linear velocity | 3 | `root_com_lin_vel_b` | VICON-derived `state['v_b']` |
| Rotation matrix (row-major) | 9 | `matrix_from_quat(root_quat_w)` | VICON-derived `state['R']` |
| Current gate, 4 corner positions in body frame | 12 | gate verts → `subtract_frame_transforms` | identical numpy reimplementation |
| Next gate, 4 corner positions in body frame | 12 | same as above | same |

Encoding gate **corners** rather than centers gives the policy direct geometric cues for size, orientation, and aperture — and stays well-defined even when the drone is inside a gate plane.

### Reward Design

The reward dictionary is assembled in `train_race.py:108-158`; the per-component formulas live in `quadcopter_strategies.py:get_rewards`. Action smoothness is added on top of the dictionary sum.

**Sparse**

| Component | Scale | Formula | Rationale |
|---|---|---|---|
| Gate pass | +15.0 | Triggers when gate-frame x crosses positive→non-positive within the 1.0 m aperture | Primary training signal — large enough to dominate dense shaping noise |
| Gate speed bonus | +0.3 | `(clamp(-v · n_gate, 0, 10))² / 10`, applied at the moment of passage | Quadratic in passage speed; peaks at +0.3 × 10 = +3 for a 10 m/s clean traversal |

**Dense shaping**

| Component | Scale | Formula | Notes |
|---|---|---|---|
| Progress toward gate | +2.0 | `clamp(d_{t-1} - d_t, -5, 5)`; multiplied by 1.5 while in the arc | Potential-based shaping — gives a dense gradient between sparse gate rewards |
| Velocity toward gate | +0.3 | `clamp(v · dir_to_gate, -5, 10) × speed_curriculum` | Curriculum-scaled 0.5×→1.5× over 3,000 iter |
| Time penalty | -0.02 | Constant per step, scaled 0.5×→1.5× by speed curriculum | Small cost that grows over training to push speed over loitering |
| Angular-velocity penalty | -0.01 | `-‖ω_body‖` | Discourages excessive spinning |
| Crash penalty | -0.5 | Per step where contact-sensor force > 1e-8 N (after the first 100 steps) | Per-timestep contact penalty, accumulates into termination at 100 hits |
| Wrong-side proximity | -0.5 | `(curr_x < 0) × clamp(1 - d/3, 0, 1)`; only after iter 500 | Gentle nudge against approaching the current gate from its exit side |
| Exit-side repulsion | -0.5 | Same form, applied to **every** non-target gate; only after iter 500 | Prevents brush-and-turn exploits across the whole circuit |
| Action smoothness | -0.03 | `-0.03 × ‖a_t - a_{t-1}‖`, added directly to the per-step reward | Reduces actuator jitter, important for sim-to-real |

**Terminal**

| Event | Reward | Rationale |
|---|---|---|
| Death (crash / altitude bound / off-track) | -5.0 | Intentionally cheap so the agent still explores risky zones like the post-gate-2 transition |
| Wrong-side gate entry (any gate, distance < 1 m) | -15.0, stacks with death (-20 total) and **terminates the episode** | Wrong-side passes are a DQ at evaluation; must be strictly worse than a crash so the policy never trades them |

**Dense-penalty warmup.** Both directional penalties (`wrong_side_prox`, `exit_repulsion`) are gated by `iter >= 500`. This lets the agent discover gate-passing behavior without a "fear barrier" near gate planes during initial exploration; the warmup ends once the policy reliably finds gates, after which the penalties shape approach geometry. Termination on wrong-side entry is always on, so the warmup never opens an exploit.

### Curriculum Learning

Three independent curricula, each on its own iteration schedule (`quadcopter_strategies.py:reset_idx`, `get_rewards`):

| Curriculum | Start | End | Duration | Why this schedule |
|---|---|---|---|---|
| Domain randomization width | 20 % of full range | 100 % of full range | 2,000 iter | Learn basic flight on near-nominal physics first; widen perturbations only once the policy is competent |
| Reset approach distance | 0.5–1.5 m | 1.2–3.5 m | 2,000 iter | Close-range gate accuracy first, then long approaches |
| Speed scaling (velocity reward + time penalty) | 0.5× | 1.5× | 3,000 iter | Route learning before speed optimization; finishes *after* DR so the policy masters robust dynamics before racing at full speed |

Decoupling matters: when these were coupled in earlier iterations, one axis becoming too hard before the others stalled training entirely.

### Key Innovation: Horizontal Arc Maneuver

Gates 3 and 6 occupy the same `(x, y, z)`. A policy that just chases gate 3's center after passing gate 2 either flies straight back into gate 6 or learns a wrong-side pass and gets terminated. The fix is a **reward-shaping override** that steers the agent through a clockwise horizontal arc at constant altitude before re-entering gate 3 from the +Y side.

Three hard-coded waypoints (`quadcopter_strategies.py:238-240`) define the arc:

| Waypoint | Position `(x, y, z)` | Role |
|---|---|---|
| WP1 | `(0.6, -0.6, 0.75)` | Pull rightward into the arc start |
| WP2 | `(1.8, 0.0, 0.75)` | Cross Y=0 at a safe X distance from both gates |
| WP3 | `(0.625, 0.5, 0.75)` | Set up the gate-3 approach from +Y |

The arc is implemented as **reward shaping, not as an observation signal** — the policy never sees a phase variable. When the target is gate 3 *and* the drone is on gate 3's exit side (`gate3_frame_x < 0`), an arc state machine advances through phases 1→2→3 based on proximity to WP1/WP2, and the `progress` and `velocity_gate` rewards are computed against the active arc waypoint instead of the gate. To stabilise learning, **30 %** of episode resets spawn the drone near gate 2's exit with a +X / -Y momentum and target gate 3, so the arc distribution is well covered.

### Domain Randomization

Each episode resamples physical parameters from these ranges (scaled by the DR curriculum). This is what makes the zero-shot real-world flight in the video above possible:

| Parameter | Range (at full curriculum) |
|---|---|
| Thrust-to-weight ratio | nominal ± 5 % |
| Drag coefficients (XY and Z) | 0.5× — 2.0× |
| PID gains kp, ki (roll/pitch and yaw) | nominal ± 15 % |
| PID gains kd (roll/pitch and yaw) | nominal ± 30 % |

---

## Architecture and Hyperparameters

The actor and critic share an identical MLP topology, defined in `agents/rsl_rl_ppo_cfg.py`:

| Network | Architecture | Activation | Output |
|---|---|---|---|
| **Actor** | 36 → 512 → 512 → 256 → 128 → 4 | ELU (hidden) | Tanh, then PPO-Gaussian noise (init std 0.8 → min 0.01) |
| **Critic** | 36 → 512 → 512 → 256 → 128 → 1 | ELU (hidden) | linear |

The actor is what gets shipped to the real Crazyflie — exact same architecture, exact same input layout, weights loaded from the trained checkpoint (`sim2real/src/controller/controller/controller_simple_policy.py:Actor`).

### PPO Hyperparameters

| Parameter | Value | Notes |
|---|---|---|
| Parallel environments | 8,192 (training) / 4,096 (config default) | A40 fits 8,192 comfortably |
| Steps per env per update | 64 | ~1.28 s at 50 Hz; sufficient horizon for GAE |
| Mini-batches | 8 | ~65k samples each |
| Learning epochs | 5 | |
| Learning rate | 3e-4 | Adaptive schedule via target KL |
| Clip range | 0.2 | Standard PPO |
| Entropy coefficient | 0.008 | Keeps exploration alive late in training |
| Value-loss coefficient | 1.0 | |
| Discount γ | 0.997 | ~4.6 s half-life at 50 Hz; credits rewards ~230 steps ahead |
| GAE λ | 0.95 | |
| Desired KL | 0.008 | LR auto-reduces when policy updates outpace this |
| Max gradient norm | 1.0 | |
| Initial noise std | 0.8 | Wide initial action exploration |
| Minimum noise std | 0.01 | Prevents premature convergence to a deterministic policy |
| Default `max_iterations` | 20,000 | Typical runs use 5k–8k |

**Effective batch size:** 8,192 envs × 64 steps = **524,288 transitions per update**.

---

## Project Structure

```
DroneRacing/
├── pyproject.toml                              # Project metadata and Isaac Lab dependencies
├── setup.py                                    # Package installation
├── LICENSE                                     # BSD-3-Clause
│
├── config/
│   └── extension.toml                          # Isaac Lab extension registration
│
├── scripts/
│   ├── rsl_rl/
│   │   ├── train_race.py                       # Training entry — reward scales, env init, PPO runner
│   │   ├── play_race.py                        # Evaluation — checkpoint load, video, JIT/ONNX export
│   │   └── cli_args.py                         # RSL-RL CLI argument plumbing
│   ├── slurm/                                  # SLURM submission scripts (Penn GRASP cluster)
│   └── util/
│       ├── env.sh                              # Environment setup (Isaac Sim 4.5)
│       ├── drone_run.sh                        # Convenience launcher for training
│       └── drone_play.sh                       # Convenience launcher for evaluation
│
├── results/
│   ├── powerloop.mp4                           # Real-world Crazyflie 2.1 powerloop deployment (Stage 3)
│   ├── featured.png                            # Thumbnail for the deployment video
│   ├── horizontal.mp4                          # Sim — horizontal bypass approach
│   ├── vertical.mp4                            # Sim — vertical bypass approach
│   ├── bestcircle.mp4                          # Sim — Phase 2 Stage 1 circle track
│   └── circlereal.mp4                          # Real — Phase 2 Stage 2 circle track
│
├── src/
│   ├── isaac_quad_sim2real/                    # Custom Isaac Lab extension (Direct RL)
│   │   └── tasks/race/config/crazyflie/
│   │       ├── quadcopter_env.py               # Environment — rigid-body sim, motor + PID, gate geometry, tracks
│   │       ├── quadcopter_strategies.py        # Strategy — rewards, 36D observations, resets, curricula, arc state machine
│   │       └── agents/
│   │           ├── rl_cfg.py                   # Base RL config dataclasses
│   │           └── rsl_rl_ppo_cfg.py           # PPO hyperparameters and actor/critic [512,512,256,128] ELU
│   │
│   └── third_parties/
│       └── rsl_rl_local/                       # Vendored RSL-RL with project-specific tweaks
│
├── sim2real/                                   # ROS2 stack used to deploy the trained policy on hardware
│   └── src/
│       ├── controller/controller/
│       │   ├── controller_simple_policy.py     # Real-world inference — same Actor + 36D obs as sim
│       │   ├── controller_params.py            # ROS2 YAML param loader (waypoints, body-rate limits, gate_side)
│       │   ├── controller_node.py              # ROS2 node — subscribes VICON, publishes CTBR
│       │   └── controller_fsm.py               # Takeoff / armed / racing / land state machine
│       ├── jirl_bringup/launch/                # ROS2 launch files (controller, vicon, crazyradio)
│       ├── crazyflie-lib-python/               # Crazyflie radio + crtp protocol (vendored)
│       ├── crazyradio_driver_cpp/              # Native crazyradio driver
│       └── motion_capture_system/              # VICON ROS2 bridge
│   └── bin/
│       └── process_bag_with_br_pos_export.py   # Post-flight rosbag analysis (sim-vs-real plots)
│
└── usd/
    ├── cf2x.usda                               # Crazyflie 2.1 USD model (physics, meshes, joints)
    └── gate.usda                               # Racing gate USD mesh
```

---

## Getting Started

### Prerequisites

- NVIDIA Isaac Lab (Isaac Sim 4.5+)
- Python 3.10+
- CUDA GPU with sufficient VRAM for thousands of parallel environments (training was done on a single A40)

### Train

```bash
python scripts/rsl_rl/train_race.py \
    --task Isaac-Quadcopter-Race-v0 \
    --num_envs 8192 \
    --max_iterations 8000 \
    --headless --logger wandb
```

Reward scales live in `scripts/rsl_rl/train_race.py`. Network architecture and PPO hyperparameters live in `src/isaac_quad_sim2real/tasks/race/config/crazyflie/agents/rsl_rl_ppo_cfg.py`. Track choice (`powerloop`, `circle`, `lemniscate`, `complex`) is set by `track_name` in `quadcopter_env.py`.

### Evaluate in sim and export the policy

```bash
python scripts/rsl_rl/play_race.py \
    --task Isaac-Quadcopter-Race-v0 \
    --num_envs 1 \
    --load_run <run_dir> \
    --checkpoint best_model.pt \
    --video --video_length 2500
```

`play_race.py` loads the checkpoint, exports the policy as both **TorchScript (`policy.pt`) and ONNX (`policy.onnx`)** under `<run_dir>/exported/`, then runs the deterministic policy for evaluation video capture.

### Deploy on a real Crazyflie 2.1

The ROS2 controller in `sim2real/src/controller/` consumes the exported checkpoint plus a YAML param file:

```yaml
gate_side: 1.0
policy:
  max_roll_br: 100.0     # deg/s — must match sim body_rate_scale_xy
  max_pitch_br: 100.0    # deg/s
  max_yaw_br: 200.0      # deg/s — must match sim body_rate_scale_z
  initial_waypoint: 0
  waypoints: [<flat 6*N list of x, y, z, roll, pitch, yaw per gate>]
```

VICON publishes drone state, the controller node reconstructs the 36D observation in numpy with the exact same layout as the sim, and the policy outputs CTBR commands sent to the Crazyflie via Crazyradio. After a flight, post-process the rosbag with:

```bash
python sim2real/bin/process_bag_with_br_pos_export.py <bag_path> <crazyflie_id>
```

---

## Design Decisions

- **Terminal penalty stacking** (−20 for wrong-side vs −5 for plain crash) — the agent never learns to "trade" a wrong-side pass for a cheaper crash, because wrong-side is strictly worse *and* immediately ends the episode.
- **Independent curricula over coupled** — DR width, approach distance, and speed scaling each progress on their own iteration schedule, so one axis becoming hard never stalls the others.
- **Dense-penalty warmup until iter 500** — the agent first learns to find gates without a fear barrier near gate planes; directional penalties activate later to shape the *approach geometry*, not gate finding itself.
- **All-gate wrong-side scanning** — every timestep checks reverse passes through every gate (excluding the current target and any colocated partner), so the policy can never sneak free progress by reversing through earlier gates.
- **Arc as reward shaping, not as observation** — the 3-waypoint horizontal arc is implemented by re-routing the `progress` and `velocity_gate` rewards through arc waypoints when in arc state, with 30 % of resets spawned in the arc distribution. Keeping the observation pure makes the same 36D format usable on hardware with no extra plumbing.
- **Symmetric actor and critic** (`[512, 512, 256, 128]` ELU on both) — chosen to keep the deployed actor architecture identical to the trained one; sim-to-real fidelity is more valuable here than a hypothetically better critic.

---

**Tech Stack:** NVIDIA Isaac Lab (Isaac Sim 4.5) | PyTorch | PPO (RSL-RL, vendored) | USD scene composition | ROS2 + VICON + Crazyradio | Crazyflie 2.1
