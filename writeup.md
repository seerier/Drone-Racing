# Autonomous Drone Racing with Reinforcement Learning

## Complete Technical Manual & Strategy Report

---

## 1. Project Overview

This repository implements a reinforcement learning system for autonomous drone racing using Proximal Policy Optimization (PPO). A simulated Crazyflie 2.1 quadrotor learns to race through the **powerloop** track — a 7-gate 3D circuit requiring 3 complete laps in minimum time.

**Platform:** NVIDIA Isaac Lab (Isaac Sim 4.5), 8192 parallel environments, 50 Hz policy rate.

**Action space:** 4-dimensional Collective Thrust and Body Rates (CTBR): `[T, ṗ, θ̇, ψ̇] ∈ [-1, 1]⁴`. The policy controls *what* the drone should do (thrust + rotate); an onboard PID controller converts these to motor commands.

**Objective:** Complete 3 laps (21 gate passages) as fast as possible, starting from the ground with randomized position and dynamics perturbations. Entering any gate from the wrong side results in immediate disqualification.

**Core challenge:** The track requires passing through a shared physical gate (gates 3 and 6) in opposite directions, necessitating a direction-reversal maneuver between gates 2 and 3.

---

## 2. Track Geometry

The powerloop track has 7 logical gates. Each gate is a 1.0m × 1.0m square frame (half-side = 0.5m).

| Gate | X (m) | Y (m) | Z (m) | Yaw (rad) | Notes |
|------|-------|-------|-------|-----------|-------|
| 0 | 2.000 | 3.500 | 0.75 | -π/2 | Start gate |
| 1 | -1.500 | 3.500 | 2.00 | +π/4 | High gate, angled |
| 2 | -0.625 | 0.000 | 0.75 | +π/2 | Pre-arc gate |
| 3 | 0.625 | 0.000 | 0.75 | +π/2 | **Shared with gate 6** |
| 4 | -1.500 | -3.500 | 2.00 | +3π/4 | High gate, angled |
| 5 | 2.000 | -3.500 | 0.75 | -π/2 | Chicane entry |
| 6 | 0.625 | 0.000 | 0.75 | -π/2 | **Shared with gate 3** |

**Gate-frame coordinate system:** The x-axis of each gate frame points in the approach direction. A correct gate passage is a sign change from positive to non-positive x in the gate frame. A wrong-side entry is the reverse (negative to positive x).

**Critical constraints:**
- **Gates 3 & 6** share position (0.625, 0, 0.75) with opposite yaw (±π/2). A correct pass through one looks like a wrong-side entry through the other.
- **Chicane proximity:** During gate 5→6 transit, the drone passes within 1.25m of gate 2. Wrong-side detection must not trigger falsely.

---

## 3. Horizontal Arc Maneuver (Core Innovation)

### 3.1 The Problem

After passing gate 2 (traveling in the -Y direction), the drone must reverse to pass gate 3 (requiring approach from +Y). This is the hardest part of the track.

### 3.2 Solution: Three-Waypoint Clockwise Sweep

The drone executes a **horizontal arc at constant altitude Z = 0.75m**, sweeping clockwise from the -Y side (post-gate-2) to the +Y side (pre-gate-3):

| Waypoint | Position (X, Y, Z) | Purpose | Phase trigger |
|----------|-------------------|---------|---------------|
| WP1 | (0.6, -0.6, 0.75) | Pull rightward after gate 2 | Default (phase 1) |
| WP2 | (1.8, 0.0, 0.75) | Cross Y=0 at safe X distance | d(drone, WP1) < 0.5m |
| WP3 | (0.625, 0.5, 0.75) | Approach gate 3 from +Y | d(drone, WP2) < 0.6m & Y ≥ -0.1 |

### 3.3 Phase Management

The arc operates in 4 phases (0–3), advanced by distance-based triggers with a **forward-only ratchet** — phases never regress. This prevents the drone from oscillating between waypoints.

- **Phase 0:** Not in arc range. Progress reward targets gate 3 directly.
- **Phase 1:** In arc, far from WP1. Progress targets WP1.
- **Phase 2:** Near WP1 (< 0.5m), not yet near WP2. Progress targets WP2.
- **Phase 3:** Near WP2 (< 0.6m) and Y ≥ -0.1. Progress targets WP3.

**Arc entry condition:** `target == gate3 AND gate3_frame_x < 0` (drone is on the -Y side, approaching from the wrong direction). Once the drone crosses Y=0 (gate-3-frame x becomes positive), it exits arc mode and standard gate-targeting takes over.

### 3.4 Transition Spike Prevention

When the drone exits the arc or advances between phases, the distance reference `_last_distance_to_goal` is reinitialized to the current distance. Without this, the stored distance from the previous waypoint (~0.5m) vs. the new target distance (~1.4m) creates a spurious -0.9 progress penalty at the exact moment the drone completes the maneuver.

### 3.5 Safety Margin

WP2 at X=1.8 is 1.175m from gate 3's center. The wrong-side detection threshold is 1.0m, providing a **0.175m margin**. This is sufficient because the constant-altitude arc eliminates vertical drift, the 1.5× progress boost keeps the drone committed, and the forward-only ratchet prevents backtracking.

### 3.6 Progress Boost

A symmetric **1.5× progress boost** applies during the arc (both positive and negative progress). The symmetry is deliberate: an asymmetric clamp (clamping negative progress to 0) would create a ratchet exploit where the drone earns reward by oscillating without net displacement.

---

## 4. Reward Design

The reward function has 10 components computed per timestep, plus a terminal reward on episode termination.

### 4.1 Sparse Rewards

| Component | Scale | Description |
|-----------|-------|-------------|
| **Gate pass** | +15.0 | Binary reward when drone crosses gate from correct side within aperture (|y| < 0.6m, |z| < 0.6m) |
| **Gate speed bonus** | ×1.0 | At gate crossing: quadratic function of gate-normal velocity, `v²/10`, clamped [0, 10] m/s. Disproportionately rewards fast crossings. |

### 4.2 Dense Rewards

| Component | Scale | Description |
|-----------|-------|-------------|
| **Progress toward gate** | ×2.0 | `clamp(d_{t-1} - d_t, -5, 5)`. During arc, targets current arc waypoint with 1.5× boost. |
| **Velocity toward gate** | ×0.5 | `clamp(v · dir_to_target, -5, 10)`. During arc, direction points to current waypoint. Scaled by speed curriculum (0.5× → 1.5× over 3000 iters). |
| **Time penalty** | ×-0.05 | Constant per step. Scaled by speed curriculum. Over 30s at 50Hz = -75 total. |
| **Angular velocity penalty** | ×-0.01 | `-0.01 * ‖ω_body‖`. Discourages aggressive spinning. |
| **Crash penalty** | ×-0.5 | Activated when contact sensor detects force > 1e-8 N after 100 timesteps (avoids penalizing takeoff). |
| **Wrong-side entry** | ×-15.0 | Computed but only reaches gradient through terminal reward mechanism. |
| **Wrong-side proximity** | ×-0.5 | Dense penalty for being on exit side of current gate. Curriculum: disabled iter 0–500, active after. |
| **Exit-side repulsion** | ×-0.5 | Dense penalty near exit side of ALL non-target gates. Same curriculum. |
| **Action smoothness** | -0.03 | `-0.03 * ‖a_t - a_{t-1}‖`. Added directly to reward sum (not in reward dict). Encourages smooth control for sim-to-real robustness. |

### 4.3 Terminal Reward

On episode termination, the entire step reward is replaced:
- Normal death: **-5** (death cost)
- Wrong-side death: **-5 + (-15) = -20** (stacked penalties)

This stacking is critical. Without it, the wrong-side penalty (-15) computed in the reward dictionary is overwritten by the generic death cost (-5), making wrong-side entries no more costly than normal crashes.

---

## 5. Wrong-Side Detection System

### 5.1 Primary Detection (Current Target Gate)

Each timestep, the drone's position is projected into the current target gate's frame. Wrong-side entry is detected when:
1. Gate-frame x transitions from negative to positive (exit-to-approach crossing)
2. 3D distance to gate center < **1.0m**

The 1.0m threshold is chosen to:
- **Catch** real wrong-side passes: max aperture corner distance ≈ 0.85m
- **Allow** the arc's Y=0 crossing at X=1.8 (distance 1.175m > 1.0m)
- **Allow** chicane 5→6 transit past gate 2 (distance 1.25m > 1.0m)

### 5.2 All-Gate Detection

A loop checks every gate each timestep, projecting the drone into each gate's frame and detecting negative-to-positive x transitions within 1.0m. This catches reverse passes through non-target gates (e.g., reversing through gate 2 after passing it).

Two exceptions prevent false positives:
- **Current target gate** is skipped (handled by primary detection)
- **Colocated gates** are skipped: when targeting gate 3, gate 6 is excluded (and vice versa), because a correct pass through one appears as wrong-side through the other

Colocated pairs are precomputed at initialization by checking gate pairs with position distance < 0.1m.

### 5.3 Dense Penalty Curriculum

Wrong-side proximity and exit repulsion are **disabled for iterations 0–500**, allowing the policy to discover gate-passing without a fear barrier. Wrong-side **termination** is always active from iteration 0, ensuring the policy never learns to exploit wrong-side passes. After iteration 500, dense penalties activate to shape approach trajectories.

---

## 6. Observation Space (31 Dimensions)

| Dims | Component | Frame | Rationale |
|------|-----------|-------|-----------|
| 3 | Linear velocity | Body | Speed/direction for thrust control |
| 3 | Angular velocity | Body | Attitude stabilization (body rates) |
| 3 | Gravity vector | Body | Encodes roll/pitch tilt compactly (3rd column of rotation matrix) |
| 3 | Position in current gate frame | Gate | Primary navigation target |
| 3 | Position in next gate frame | Gate+1 | Racing line planning |
| 3 | Position in gate+2 frame | Gate+2 | Arc setup and chicane planning |
| 3 | Gate normal in body frame | Body | Traversal direction; disambiguates gates 3 vs 6 |
| 1 | Distance to current gate | Scalar | Value function estimation |
| 1 | Altitude (z) | World | Ground avoidance, ceiling awareness |
| 1 | Lap progress | Scalar | Fraction through 7-gate circuit |
| 4 | Previous actions (CTBR) | — | Temporal context for smooth control |
| 1 | Velocity toward gate | Body | Scalar approach speed (dot product of velocity and gate normal) |
| 1 | Arc phase indicator | Scalar | Normalized arc phase ∈ [0, 1]; helps policy adapt to each arc phase |
| 1 | Episode time fraction | Scalar | Remaining time awareness for 3-lap planning |
| **31** | **Total** | | |

**Design notes:**
- **Three-gate lookahead** allows the policy to plan racing lines through upcoming gates, not just the immediate target.
- **Gate normal in body frame** is the negated gate normal vector rotated into body frame. Essential for disambiguating gates 3 and 6.
- **Gravity in body frame** (not quaternion) encodes tilt in 3 dims without redundancy. Yaw is implicit in gate-frame observations.
- **Arc phase indicator** is computed in `get_rewards()` (runs before `get_observations()`), so it reflects the current step's phase.

---

## 7. Domain Randomization

At each episode reset, dynamics parameters are re-sampled from uniform distributions:

| Parameter | Range | Notes |
|-----------|-------|-------|
| Thrust-to-weight ratio | Nominal ±5% | |
| Aerodynamic drag (xy) | 0.5× – 2.0× | xy coupled (same value for x and y) |
| Aerodynamic drag (z) | 0.5× – 2.0× | Independent from xy |
| PID roll/pitch kp, ki | ±15% | |
| PID roll/pitch kd | ±30% | |
| PID yaw kp, ki | ±15% | |
| PID yaw kd | ±30% | |

### DR Curriculum

Ranges start at **20% of full extent** and widen to **100% over 2000 iterations**:

```
dr_frac = min(1.0, 0.2 + 0.8 * iteration / 2000)
```

| Parameter | Iter 0 (f=0.2) | Iter 2000+ (f=1.0) |
|-----------|---------------|-------------------|
| TWR | ±1% | ±5% |
| Drag | [0.9, 1.2]× | [0.5, 2.0]× |
| PID kp, ki | ±3% | ±15% |
| PID kd | ±6% | ±30% |

This allows the policy to master control under near-nominal dynamics before gradually adapting to the full perturbation range.

---

## 8. Training Curriculum

### 8.1 Reset Distribution

| Category | Fraction | Description |
|----------|----------|-------------|
| **Ground starts** | 20% | z=0.05m near gate 0. x_local ∈ [-3, -0.5], y_local ∈ [-1, 1]. Zero velocity. Matches evaluation conditions. |
| **Arc starts** | 30% | Near gate 2 exit, target=gate 3. Position offset from gate 2: ΔX ∈ [-0.2, 0.4], ΔY ∈ [-0.8, -0.1], Z ∈ [0.5, 1.0]. Velocity biased into clockwise arc: vx ∈ [0, 1.5], vy ∈ [-2.0, -0.5] m/s. |
| **Random gate** | 50% | Uniformly random gate, approach distance scaled by curriculum. Small velocity noise (±0.3 m/s) and yaw noise (±0.2 rad). |

### 8.2 Approach Distance Curriculum

Spawn distance grows with training progress (controlled by `iteration / 2000`):
- **d_min:** 0.5 → 1.2m
- **d_max:** 1.5 → 3.5m

Early training places the drone close to gates for easy gate-pass discovery. Late training requires full gate-to-gate navigation.

### 8.3 Speed Curriculum

Velocity-toward-gate and time-penalty rewards scale from **0.5× to 1.5×** over 3000 iterations:

```
speed_frac = min(1.0, iteration / 3000)
scale = 0.5 + 1.0 * speed_frac
```

Early training focuses on learning the route; late training pushes for lap speed.

### 8.4 Dense Penalty Curriculum

Wrong-side proximity and exit repulsion are disabled for iterations 0–500 (see Section 5.3).

---

## 9. PPO Configuration

### 9.1 Network Architecture

- **Actor:** 31 → 256 → 256 → 128 → 4 (ELU activations, Tanh output)
- **Critic:** 31 → 512 → 512 → 256 → 128 → 1 (ELU activations)

The critic is deliberately larger. It must estimate expected returns across a complex state space (gate approaches, arc phases, chicane, DR variations, 3-lap horizon). Better value estimates drive better advantage signals without adding cost to policy inference.

### 9.2 Hyperparameters

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Environments | 8192 | Massive parallelism for stable gradients |
| Steps per env | 64 | ~1.3s at 50Hz; sufficient for GAE estimation |
| Mini-batches | 8 | ~65K samples each |
| Learning epochs | 5 | |
| Learning rate | 3e-4 | Adaptive via KL targeting |
| Clip ε | 0.2 | Standard PPO clip |
| Entropy coef | 0.008 | Maintains exploration for diverse racing lines |
| γ (discount) | 0.997 | Half-life ~4.6s at 50Hz; credits rewards 100+ steps ahead |
| λ (GAE) | 0.95 | |
| Desired KL | 0.008 | LR auto-reduces when updates are too aggressive |
| Max grad norm | 1.0 | |
| Init noise std | 0.8 | |
| Min noise std | 0.01 | |

---

## 10. W&B Monitoring

All `Episode_Reward/` variables are **per-second rates** (episodic sum / max episode length in seconds), averaged over resetting environments.

### 10.1 Reward Metrics

| Variable | Interpretation | Target |
|----------|---------------|--------|
| `gate_pass` | Gate passages per episode. 10.5 = 3 full laps (21 gates). | ≥ 10.5 |
| `progress_goal` | Distance-reduction reward rate. Positive = making progress. | Positive |
| `velocity_gate` | Velocity-toward-target rate. Higher = faster approach. | Increasing |
| `gate_speed_bonus` | Speed at gate crossings. Higher = faster, confident passes. | Increasing |
| `time_penalty` | More negative = longer episodes (surviving but slow). | Less negative |
| `crash` | Crash frequency. | → 0 |
| `wrong_side` | Wrong-side entry frequency. | → 0 |
| `ang_vel_penalty` | Body rate magnitude. | Small |
| `wrong_side_prox` | Activates at iter 500. | → 0 |
| `exit_repulsion` | Activates at iter 500. | → 0 |
| `arc_active` | Fraction of steps in arc mode. Diagnostic only. | Moderate |
| `arc_progress` | Progress during arc steps. Positive = arc working. | Positive |

### 10.2 Termination & Loss Metrics

| Variable | Meaning |
|----------|---------|
| `Episode_Termination/died` | Terminated episodes per reset batch |
| `Episode_Termination/time_out` | Episodes reaching max length (good = surviving) |
| `Loss/surrogate` | PPO clipped surrogate loss |
| `Loss/value_function` | Critic MSE loss |
| `Loss/entropy` | Policy entropy (should slowly decrease) |
| `Loss/learning_rate` | Adaptive LR (starts 3e-4) |
| `Policy/mean_noise_std` | Action noise std (decreases as policy gains confidence) |

### 10.3 Reading the Learning Curve

A typical successful run shows:
1. `gate_pass` climbs from 0 to ~1.5 (3 gates) in the first few hundred iterations
2. Plateau at ~1.5–2.0 as the policy attempts the horizontal arc
3. Jump to ~3.5+ (7+ gates) once the arc is learned
4. Continued climb toward 10.5 as multi-lap performance improves
5. A possible dip around iteration 1000–2000 as DR ranges widen (expected and recoverable)

---

## 11. Key Design Decisions & Lessons

### 11.1 Explicit Waypoint Guidance

The gate 2→3 transition cannot be learned from gate-pass rewards alone — the drone has no incentive to deviate from a straight line until it discovers the correct bypass trajectory by chance. Intermediate waypoints provide a continuous reward signal that guides the drone through the maneuver step by step. Without waypoint-based shaping, the policy converges to crashing after gate 2 as a local optimum.

### 11.2 Curriculum Is Everything

Three curricula work together to make the bypass learnable:
- **Targeted respawning (30% arc starts):** Provides dense experience for a transition encountered only once per lap.
- **Distance curriculum:** Start close to gates (0.5–1.5m), widen to 0.5–3.5m. Learn gate-passing before full gate-to-gate navigation.
- **Penalty curriculum:** Dense directional penalties disabled for iter 0–500 to avoid fear barriers that prevent gate-passing discovery.

Without any one of these, the policy either fails to discover the bypass or learns it too slowly.

### 11.3 Wrong-Side Penalty Stacking

A critical bug existed where the wrong-side penalty (-15) was overwritten by the death cost (-5) in the terminal reward logic, making wrong-side entries no worse than normal crashes. Stacking both penalties (-20 total) was essential for correct deterrence.

### 11.4 Speed Curriculum Interaction with DR

The DR curriculum reaches full range at iteration 2000; the speed curriculum reaches full scale at iteration 3000. This sequencing is deliberate: first learn to handle dynamic perturbations, then optimize speed under those perturbations.

---

## 12. Usage

### Training

```bash
python scripts/rsl_rl/train_race.py \
    --task Isaac-Quadcopter-Race-v0 \
    --num_envs 8192 \
    --max_iterations 8000 \
    --headless \
    --logger wandb
```

### Playing / Video Recording

```bash
python scripts/rsl_rl/play_race.py \
    --task Isaac-Quadcopter-Race-v0 \
    --num_envs 1 \
    --load_run <YYYY-MM-DD_HH-MM-SS> \
    --checkpoint best_model.pt \
    --headless \
    --video \
    --video_length 800
```

Videos are saved to `logs/rsl_rl/quadcopter_direct/<run>/videos/play/`.

### Key Files

| File | Purpose |
|------|---------|
| `src/.../quadcopter_strategies.py` | Rewards, observations, resets, arc maneuver, wrong-side detection |
| `src/.../agents/rsl_rl_ppo_cfg.py` | Network architecture & PPO hyperparameters |
| `scripts/rsl_rl/train_race.py` | Training script & reward scales |
| `scripts/rsl_rl/play_race.py` | Evaluation & video rendering |
| `src/.../quadcopter_env.py` | Environment physics (not modifiable) |
| `src/third_parties/rsl_rl_local/rsl_rl/algorithms/ppo.py` | PPO algorithm implementation |
| `src/third_parties/rsl_rl_local/rsl_rl/storage/rollout_storage.py` | Rollout storage & GAE computation |

### SLURM (Cluster)

```bash
sbatch --partition=kostas-compute --qos=kd-med scripts/slurm/train.sh
```
