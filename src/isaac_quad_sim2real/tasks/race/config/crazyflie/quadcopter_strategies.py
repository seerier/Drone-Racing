# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Modular strategy classes for quadcopter environment rewards, observations, and resets."""

from __future__ import annotations

import torch
import numpy as np
from typing import TYPE_CHECKING, Dict, Optional, Tuple

from isaaclab.utils.math import subtract_frame_transforms, quat_from_euler_xyz, euler_xyz_from_quat, wrap_to_pi, matrix_from_quat

if TYPE_CHECKING:
    from .quadcopter_env import QuadcopterEnv

D2R = np.pi / 180.0
R2D = 180.0 / np.pi


class DefaultQuadcopterStrategy:
    """Default strategy implementation for quadcopter environment."""

    def __init__(self, env: QuadcopterEnv):
        """Initialize the default strategy.

        Args:
            env: The quadcopter environment instance.
        """
        self.env = env
        self.device = env.device
        self.num_envs = env.num_envs
        self.cfg = env.cfg

        # Track gate-frame x-coordinate for ALL gates (for all-gate wrong-side detection)
        num_gates = env._waypoints.shape[0]
        self._prev_x_all_gates = torch.ones(self.num_envs, num_gates, device=self.device)

        # Precompute colocated gate pairs (gates at same position, e.g., gates 3 and 6)
        # Used to skip false wrong-side triggers when passing one of a colocated pair
        self._colocated_gates = {}
        wp_pos = env._waypoints[:, :3]
        for i in range(num_gates):
            colocated = []
            for j in range(num_gates):
                if i != j and torch.linalg.norm(wp_pos[i] - wp_pos[j]) < 0.1:
                    colocated.append(j)
            if colocated:
                self._colocated_gates[i] = colocated

        # Initialize episode sums for logging if in training mode
        if self.cfg.is_train and hasattr(env, 'rew'):
            keys = [key.split("_reward_scale")[0] for key in env.rew.keys() if key != "death_cost"]
            self._episode_sums = {
                key: torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
                for key in keys
            }
            # Extra monitoring metrics (not rewards, just diagnostics)
            self._episode_sums["arc_active"] = torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
            self._episode_sums["arc_progress"] = torch.zeros(self.num_envs, dtype=torch.float, device=self.device)

        # Track horizontal arc state for transition spike fix
        self._was_in_arc = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self._arc_phase = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self._prev_arc_phase = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)

        # Initialize fixed parameters once (no domain randomization)
        # These parameters remain constant throughout the simulation
        # Aerodynamic drag coefficients
        self.env._K_aero[:, :2] = self.env._k_aero_xy_value
        self.env._K_aero[:, 2] = self.env._k_aero_z_value

        # PID controller gains for angular rate control
        # Roll and pitch use the same gains
        self.env._kp_omega[:, :2] = self.env._kp_omega_rp_value
        self.env._ki_omega[:, :2] = self.env._ki_omega_rp_value
        self.env._kd_omega[:, :2] = self.env._kd_omega_rp_value

        # Yaw has different gains
        self.env._kp_omega[:, 2] = self.env._kp_omega_y_value
        self.env._ki_omega[:, 2] = self.env._ki_omega_y_value
        self.env._kd_omega[:, 2] = self.env._kd_omega_y_value

        # Motor time constants (same for all 4 motors)
        self.env._tau_m[:] = self.env._tau_m_value

        # Thrust to weight ratio
        self.env._thrust_to_weight[:] = self.env._twr_value

    def get_rewards(self) -> torch.Tensor:
        """get_rewards() is called per timestep. This is where you define your reward structure and compute them
        according to the reward scales you tune in train_race.py. The following is an example reward structure that
        causes the drone to hover near the zeroth gate. It will not produce a racing policy, but simply serves as proof
        if your PPO implementation works. You should delete it or heavily modify it once you begin the racing task."""

        # TODO ----- START ----- Define the tensors required for your custom reward structure

        num_gates = self.env._waypoints.shape[0]
        gate_half = self.env.cfg.gate_model.gate_side / 2.0  # 0.5 m

        # --- Gate-crossing detection via sign change of gate-frame x-coordinate ---
        curr_x = self.env._pose_drone_wrt_gate[:, 0]
        curr_y = self.env._pose_drone_wrt_gate[:, 1]
        curr_z = self.env._pose_drone_wrt_gate[:, 2]
        prev_x = self.env._prev_x_drone_wrt_gate

        # Correct direction: crossed from positive to non-positive x within gate aperture
        gate_passed = (
            (prev_x > 0.0)
            & (curr_x <= 0.0)
            & (torch.abs(curr_y) < gate_half + 0.1)
            & (torch.abs(curr_z) < gate_half + 0.1)
        )
        ids_gate_passed = torch.where(gate_passed)[0]

        # --- Wrong-side gate entry: crossed from negative to positive x (DQ in eval!) ---
        # Distance-based check: 1.0m catches any real wrong-side pass (max aperture
        # corner distance ~0.85m) while allowing the horizontal arc Y=0 crossing at
        # X≈1.8 (distance to gate 3 = 1.175m > 1.0m, margin=0.175m).
        # Also prevents false DQ at gate 2 during chicane 5→6 (dist=1.25m > 1.0m).
        dist_to_target_gate = torch.linalg.norm(self.env._pose_drone_wrt_gate, dim=1)
        wrong_side_entry = (
            (prev_x < 0.0)
            & (curr_x >= 0.0)
            & (dist_to_target_gate < 1.0)
        )

        # --- All-gate wrong-side detection: catch reverse passes through ANY gate ---
        # Fixes exploit: after passing gate 2, drone reverses through gate 2 undetected
        # because curr_x/prev_x only track the current target gate.
        wrong_side_any_gate = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        for i in range(num_gates):
            gate_pos_i = self.env._waypoints[i, :3].unsqueeze(0).expand(self.num_envs, -1)
            gate_quat_i = self.env._waypoints_quat[i, :].unsqueeze(0).expand(self.num_envs, -1)
            pos_in_gate_i, _ = subtract_frame_transforms(
                gate_pos_i, gate_quat_i,
                self.env._robot.data.root_link_pos_w[:, :3],
            )
            curr_x_i = pos_in_gate_i[:, 0]

            # Skip current target gate (already handled above)
            is_current = (self.env._idx_wp == i)

            # Skip gates colocated with current target (e.g., skip gate 6 when targeting gate 3)
            # Gates 3&6 share position but opposite yaw; correct pass through one looks like
            # wrong-side entry through the other without this skip.
            is_colocated_with_target = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
            if i in self._colocated_gates:
                for j in self._colocated_gates[i]:
                    is_colocated_with_target = is_colocated_with_target | (self.env._idx_wp == j)

            dist_to_gate_i = torch.linalg.norm(pos_in_gate_i, dim=1)
            wrong_side_i = (
                (self._prev_x_all_gates[:, i] < 0.0)
                & (curr_x_i >= 0.0)
                & (dist_to_gate_i < 1.0)
                & (~is_current)
                & (~is_colocated_with_target)
            )
            wrong_side_any_gate = wrong_side_any_gate | wrong_side_i

            self._prev_x_all_gates[:, i] = curr_x_i

        # Merge: wrong-side through ANY gate triggers termination
        wrong_side_entry = wrong_side_entry | wrong_side_any_gate

        # Compute world-frame velocity before gate advancement (needed for speed bonus)
        rot_mat = matrix_from_quat(self.env._robot.data.root_quat_w)  # (N,3,3)
        lin_vel_w = torch.bmm(
            rot_mat, self.env._robot.data.root_com_lin_vel_b.unsqueeze(-1)
        ).squeeze(-1)

        # --- Gate speed bonus: reward velocity aligned with gate normal at passage ---
        gate_pass_speed_bonus = torch.zeros(self.num_envs, device=self.device)
        if len(ids_gate_passed) > 0:
            # Gate index they just passed (before incrementing)
            passed_gate_idx = self.env._idx_wp[ids_gate_passed]
            gate_normals = self.env._normal_vectors[passed_gate_idx]  # (K, 3)
            vel_at_pass = lin_vel_w[ids_gate_passed]
            # Speed through gate = dot(velocity, gate_normal), clamp positive
            # Quadratic bonus: disproportionately rewards faster crossings
            # 2m/s→0.4, 5m/s→2.5, 10m/s→10 (same max as before)
            speed_through_raw = (-vel_at_pass * gate_normals).sum(dim=1).clamp(0.0, 10.0)
            speed_through = speed_through_raw ** 2 / 10.0
            gate_pass_speed_bonus[ids_gate_passed] = speed_through

        # Advance waypoint index and update counters for envs that passed a gate
        if len(ids_gate_passed) > 0:
            self.env._n_gates_passed[ids_gate_passed] += 1
            self.env._idx_wp[ids_gate_passed] = (
                self.env._idx_wp[ids_gate_passed] + 1
            ) % num_gates
            self.env._desired_pos_w[ids_gate_passed, :3] = (
                self.env._waypoints[self.env._idx_wp[ids_gate_passed], :3]
            )
            # Recompute gate-relative pose using the NEW target gate
            new_pose, _ = subtract_frame_transforms(
                self.env._waypoints[self.env._idx_wp[ids_gate_passed], :3],
                self.env._waypoints_quat[self.env._idx_wp[ids_gate_passed], :],
                self.env._robot.data.root_link_pos_w[ids_gate_passed],
            )
            self.env._pose_drone_wrt_gate[ids_gate_passed] = new_pose
            self.env._prev_x_drone_wrt_gate[ids_gate_passed] = new_pose[:, 0]
            self.env._last_distance_to_goal[ids_gate_passed] = torch.linalg.norm(
                new_pose, dim=1
            )

            # Horizontal arc: for envs that just advanced to gate 3, override
            # _last_distance_to_goal to use ARC_WP1 so the progress reward
            # guides the drone into the clockwise horizontal arc.
            new_targets = self.env._idx_wp[ids_gate_passed]
            entering_arc = ids_gate_passed[new_targets == 3]
            if len(entering_arc) > 0:
                arc_wp1_init = torch.tensor([0.6, -0.6, 0.75], device=self.device)
                drone_pos_arc = self.env._robot.data.root_link_pos_w[entering_arc, :3]
                self.env._last_distance_to_goal[entering_arc] = torch.linalg.norm(
                    drone_pos_arc - arc_wp1_init.unsqueeze(0), dim=1
                )

        # Update prev_x for envs that did NOT just pass a gate
        self.env._prev_x_drone_wrt_gate[~gate_passed] = curr_x[~gate_passed]

        # --- Three-phase horizontal arc waypoints ---
        # After gate 2 exit (drone at Y<0), sweep clockwise at Z≈0.75
        # to gate 3's approach side (Y>0). Three waypoints guide the arc:
        #   WP1: pull rightward into arc start
        #   WP2: Y=0 crossing at safe X distance from gates
        #   WP3: gate 3 approach setup from +Y side
        arc_wp1 = torch.tensor([0.6, -0.6, 0.75], device=self.device)
        arc_wp2 = torch.tensor([1.8,  0.0, 0.75], device=self.device)
        arc_wp3 = torch.tensor([0.625, 0.5, 0.75], device=self.device)

        is_target_gate3 = (self.env._idx_wp == 3)
        gate3_frame_x = self.env._pose_drone_wrt_gate[:, 0]
        in_arc = is_target_gate3 & (gate3_frame_x < 0.0)

        # Phase advancement: distance-based with forward-only ratchet
        drone_pos_w_all = self.env._robot.data.root_link_pos_w[:, :3]
        dist_to_arc_wp1 = torch.linalg.norm(drone_pos_w_all - arc_wp1.unsqueeze(0), dim=1)
        dist_to_arc_wp2 = torch.linalg.norm(drone_pos_w_all - arc_wp2.unsqueeze(0), dim=1)
        near_wp1 = (dist_to_arc_wp1 < 0.5)
        near_wp2 = (dist_to_arc_wp2 < 0.6) & (drone_pos_w_all[:, 1] >= -0.1)

        new_phase = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        new_phase[in_arc & ~near_wp1] = 1
        new_phase[in_arc & near_wp1 & ~near_wp2] = 2
        new_phase[in_arc & near_wp2] = 3
        self._arc_phase = torch.where(in_arc, torch.max(self._arc_phase, new_phase),
                                      torch.zeros_like(self._arc_phase))

        in_arc_phase1 = in_arc & (self._arc_phase == 1)
        in_arc_phase2 = in_arc & (self._arc_phase == 2)
        in_arc_phase3 = in_arc & (self._arc_phase == 3)

        # Detect arc→normal transition and phase advances for spike prevention
        just_exited_arc = self._was_in_arc & (~in_arc)
        phase_just_advanced = in_arc & (self._arc_phase > self._prev_arc_phase)

        # --- Dense potential-based progress reward ---
        dist_to_gate = torch.linalg.norm(self.env._pose_drone_wrt_gate, dim=1)
        if in_arc_phase1.any():
            dist_to_gate[in_arc_phase1] = torch.linalg.norm(
                drone_pos_w_all[in_arc_phase1] - arc_wp1.unsqueeze(0), dim=1)
        if in_arc_phase2.any():
            dist_to_gate[in_arc_phase2] = torch.linalg.norm(
                drone_pos_w_all[in_arc_phase2] - arc_wp2.unsqueeze(0), dim=1)
        if in_arc_phase3.any():
            dist_to_gate[in_arc_phase3] = torch.linalg.norm(
                drone_pos_w_all[in_arc_phase3] - arc_wp3.unsqueeze(0), dim=1)

        # Reinit last_distance on arc exit and phase transitions to prevent spike
        if just_exited_arc.any():
            self.env._last_distance_to_goal[just_exited_arc] = dist_to_gate[just_exited_arc]
        if phase_just_advanced.any():
            self.env._last_distance_to_goal[phase_just_advanced] = dist_to_gate[phase_just_advanced]

        progress = (self.env._last_distance_to_goal - dist_to_gate).clamp(-5.0, 5.0)
        # Symmetric 1.5x arc boost: backtracking IS penalized (no ratchet exploit)
        if in_arc.any():
            progress[in_arc] = progress[in_arc] * 1.5
        self.env._last_distance_to_goal = dist_to_gate.clone()

        # --- Velocity-toward-gate reward ---
        gate_pos_w = self.env._waypoints[self.env._idx_wp, :3].clone()
        if in_arc_phase1.any():
            gate_pos_w[in_arc_phase1] = arc_wp1.unsqueeze(0)
        if in_arc_phase2.any():
            gate_pos_w[in_arc_phase2] = arc_wp2.unsqueeze(0)
        if in_arc_phase3.any():
            gate_pos_w[in_arc_phase3] = arc_wp3.unsqueeze(0)
        drone_pos_w = self.env._robot.data.root_link_pos_w
        vec_to_gate = gate_pos_w - drone_pos_w
        dist_for_vel = torch.norm(vec_to_gate, dim=1, keepdim=True).clamp(min=1e-6)
        dir_to_gate = vec_to_gate / dist_for_vel
        vel_toward_gate = (lin_vel_w * dir_to_gate).sum(dim=1).clamp(-5.0, 10.0)

        # --- Crash detection: sustained contact force for > 100 timesteps ---
        contact_forces = self.env._contact_sensor.data.net_forces_w
        crashed = (torch.norm(contact_forces, dim=-1) > 1e-8).squeeze(1).int()
        mask = (self.env.episode_length_buf > 100).int()
        self.env._crashed = self.env._crashed + crashed * mask

        # --- Time penalty: constant per-step cost to encourage speed ---
        time_penalty = torch.ones(self.num_envs, device=self.device)

        # --- Angular velocity penalty: penalize excessive body rates for stability ---
        ang_vel_penalty = torch.norm(
            self.env._robot.data.root_ang_vel_b, dim=1
        )

        # --- Wrong-side entry always terminates episode (matches eval DQ rule) ---
        self.env.reset_terminated = self.env.reset_terminated | wrong_side_entry

        # --- Dense directional penalties: curriculum ---
        # Iter 0-500:  No dense penalties — drone learns to pass gates without fear barrier
        # Iter 500+:   Enable dense penalties — drone learns correct approach direction
        # Termination is always active above, so wrong-side exploit is impossible
        # During eval (is_train=False): always enable all penalties
        dense_penalty_active = (not self.cfg.is_train) or (self.env.iteration >= 500)

        # --- Wrong-side proximity: dense penalty for being on exit side of current gate ---
        if dense_penalty_active:
            on_wrong_side = (curr_x < 0.0).float()
            on_wrong_side[ids_gate_passed] = 0.0  # don't penalize correct gate passages
            proximity_scale = torch.clamp(1.0 - dist_to_gate / 3.0, 0.0, 1.0)
            wrong_side_proximity = on_wrong_side * proximity_scale
        else:
            wrong_side_proximity = torch.zeros(self.num_envs, device=self.device)

        # --- Exit-side repulsion for ALL gates ---
        if dense_penalty_active:
            exit_repulsion = torch.zeros(self.num_envs, device=self.device)
            for i in range(num_gates):
                gate_pos_i = self.env._waypoints[i, :3].unsqueeze(0).expand(self.num_envs, -1)
                gate_quat_i = self.env._waypoints_quat[i, :].unsqueeze(0).expand(self.num_envs, -1)
                pos_in_gate_i, _ = subtract_frame_transforms(
                    gate_pos_i, gate_quat_i,
                    self.env._robot.data.root_link_pos_w[:, :3],
                )
                x_i = pos_in_gate_i[:, 0]
                dist_i = torch.linalg.norm(pos_in_gate_i, dim=1)
                # Skip current target gate (already covered by wrong_side_prox)
                is_current = (self.env._idx_wp == i)
                on_exit = (x_i < 0.0).float() * (~is_current).float()
                prox = torch.clamp(1.0 - dist_i / 1.5, 0.0, 1.0)
                exit_repulsion += on_exit * prox
        else:
            exit_repulsion = torch.zeros(self.num_envs, device=self.device)

        # TODO ----- END -----

        if self.cfg.is_train:
            # TODO ----- START ----- Compute per-timestep rewards by multiplying with your reward scales (in train_race.py)
            # Speed curriculum: ramp velocity/time rewards as policy matures
            speed_frac = min(1.0, self.env.iteration / 3000.0)
            vel_curriculum_scale = 0.5 + 1.0 * speed_frac    # 0.5 → 1.5
            time_curriculum_scale = 0.5 + 1.0 * speed_frac   # 0.5 → 1.5

            rewards = {
                "gate_pass":        gate_passed.float()         * self.env.rew['gate_pass_reward_scale'],
                "progress_goal":    progress                    * self.env.rew['progress_goal_reward_scale'],
                "velocity_gate":    vel_toward_gate             * self.env.rew['velocity_gate_reward_scale'] * vel_curriculum_scale,
                "crash":            crashed.float()             * self.env.rew['crash_reward_scale'],
                "wrong_side":       wrong_side_entry.float()    * self.env.rew['wrong_side_reward_scale'],
                "gate_speed_bonus": gate_pass_speed_bonus       * self.env.rew['gate_speed_bonus_reward_scale'],
                "time_penalty":     time_penalty                * self.env.rew['time_penalty_reward_scale'] * time_curriculum_scale,
                "ang_vel_penalty":  ang_vel_penalty              * self.env.rew['ang_vel_penalty_reward_scale'],
                "wrong_side_prox":  wrong_side_proximity         * self.env.rew['wrong_side_prox_reward_scale'],
                "exit_repulsion":   exit_repulsion               * self.env.rew['exit_repulsion_reward_scale'],
            }
            reward = torch.sum(torch.stack(list(rewards.values())), dim=0)

            # Action smoothness: penalize jerky control for sim-to-real robustness
            action_smoothness = -0.03 * torch.norm(self.env._actions - self.env._previous_actions, dim=1)
            reward = reward + action_smoothness

            # Fix 1: wrong-side deaths get death_cost + wrong_side penalty (-5 + -15 = -20)
            # Other deaths get just death_cost (-5). Previously, wrong_side -15 was
            # computed but overwritten by death_cost, making wrong-side = normal death.
            terminal_reward = (self.env.rew['death_cost']
                               + wrong_side_entry.float() * self.env.rew['wrong_side_reward_scale'])
            reward = torch.where(self.env.reset_terminated, terminal_reward, reward)

            # Logging
            for key, value in rewards.items():
                self._episode_sums[key] += value
            # Arc diagnostics: track how often the drone is in arc mode
            # and the progress it makes through the horizontal arc
            self._episode_sums["arc_active"] += in_arc.float()
            arc_progress_step = torch.zeros(self.num_envs, device=self.device)
            arc_progress_step[in_arc] = progress[in_arc]
            self._episode_sums["arc_progress"] += arc_progress_step
        else:   # This else condition implies eval is called with play_race.py. Can be useful to debug at test-time
            reward = torch.zeros(self.num_envs, device=self.device)
            # TODO ----- END -----

        # Track arc state for next step's transition detection
        self._was_in_arc = in_arc.clone()
        self._prev_arc_phase = self._arc_phase.clone()

        return reward

    def get_observations(self) -> Dict[str, torch.Tensor]:
        """Get observations. Read reset_idx() and quadcopter_env.py to see which drone info is extracted from the sim.
        The following code is an example. You should delete it or heavily modify it once you begin the racing task."""

        # TODO ----- START ----- Define tensors for your observation space. Be careful with frame transformations

        # Body-frame linear velocity: direct speed/direction signal for thrust control
        drone_lin_vel_b = self.env._robot.data.root_com_lin_vel_b   # (N, 3)

        # Body-frame angular velocity (body rates): essential for attitude stabilization
        drone_ang_vel_b = self.env._robot.data.root_ang_vel_b       # (N, 3)

        # Gravity vector projected into body frame: encodes roll and pitch tilt compactly.
        rot_mat_obs = matrix_from_quat(self.env._robot.data.root_quat_w)  # (N, 3, 3)
        gravity_b = -rot_mat_obs[:, 2, :]                                  # (N, 3)

        # Current target gate: drone position in gate's local frame.
        drone_pos_gate_frame = self.env._pose_drone_wrt_gate          # (N, 3)

        # Next gate look-ahead (gate+1): lets the policy plan smooth racing lines
        num_gates_obs = self.env._waypoints.shape[0]
        next_gate_idx = (self.env._idx_wp + 1) % num_gates_obs
        drone_pos_next_gate_frame, _ = subtract_frame_transforms(
            self.env._waypoints[next_gate_idx, :3],
            self.env._waypoints_quat[next_gate_idx, :],
            self.env._robot.data.root_link_pos_w,
        )  # (N, 3)

        # Second look-ahead (gate+2): critical for horizontal arc setup and chicane planning
        next_next_gate_idx = (self.env._idx_wp + 2) % num_gates_obs
        drone_pos_next_next_gate_frame, _ = subtract_frame_transforms(
            self.env._waypoints[next_next_gate_idx, :3],
            self.env._waypoints_quat[next_next_gate_idx, :],
            self.env._robot.data.root_link_pos_w,
        )  # (N, 3)

        # Gate normal direction in body frame: tells agent which direction to fly through gate.
        # Essential for disambiguating gates 3 vs 6 (same physical gate, opposite directions).
        gate_normal_w = -self.env._normal_vectors[self.env._idx_wp]  # (N, 3) negated: points in traversal direction
        gate_normal_b = torch.bmm(
            rot_mat_obs.transpose(1, 2), gate_normal_w.unsqueeze(-1)
        ).squeeze(-1)  # (N, 3)

        # Scalar distance to current gate: easier for value function estimation
        dist_to_gate = torch.linalg.norm(
            self.env._pose_drone_wrt_gate, dim=1, keepdim=True
        )  # (N, 1)

        # Height above ground: helps takeoff from ground starts and avoid floor crashes
        height = self.env._robot.data.root_link_pos_w[:, 2:3]  # (N, 1)

        # Lap progress: fraction through current lap for value estimation over 3-lap horizon
        lap_progress = (
            (self.env._n_gates_passed.float() % num_gates_obs) / num_gates_obs
        ).unsqueeze(-1)  # (N, 1)

        # Previous policy outputs (CTBR): temporal context for smooth control
        prev_actions = self.env._previous_actions                     # (N, 4)

        # Velocity toward current gate (scalar): helps policy optimize approach speed
        vel_toward_gate_obs = torch.sum(
            drone_lin_vel_b * gate_normal_b, dim=1, keepdim=True
        )  # (N, 1)

        # Arc phase indicator: normalized to [0, 1], disambiguates horizontal arc position
        arc_phase_obs = (self._arc_phase.float() / 3.0).unsqueeze(-1)  # (N, 1)

        # Normalized episode time: helps value function estimate remaining time budget
        episode_time_frac = (
            self.env.episode_length_buf.float() / self.env.max_episode_length
        ).unsqueeze(-1)  # (N, 1)

        # TODO ----- END -----

        obs = torch.cat(
            # TODO ----- START ----- List your observation tensors here to be concatenated together
            [
                drone_lin_vel_b,                # 3  body-frame linear velocity
                drone_ang_vel_b,                # 3  body-frame angular velocity
                gravity_b,                      # 3  gravity in body frame
                drone_pos_gate_frame,           # 3  drone pos in current gate frame
                drone_pos_next_gate_frame,      # 3  drone pos in next gate frame
                drone_pos_next_next_gate_frame, # 3  drone pos in gate+2 frame
                gate_normal_b,                  # 3  gate normal in body frame
                dist_to_gate,                   # 1  scalar distance to gate
                height,                         # 1  altitude
                lap_progress,                   # 1  fraction through current lap
                prev_actions,                   # 4  previous CTBR output
                vel_toward_gate_obs,            # 1  velocity toward gate (scalar)
                arc_phase_obs,                  # 1  arc phase indicator [0, 1]
                episode_time_frac,              # 1  normalized episode time
            ],
            # TODO ----- END -----
            dim=-1,
        )
        observations = {"policy": obs}

        return observations

    def reset_idx(self, env_ids: Optional[torch.Tensor]):
        """Reset specific environments to initial states."""
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self.env._robot._ALL_INDICES

        # Logging for training mode
        if self.cfg.is_train and hasattr(self, '_episode_sums'):
            extras = dict()
            for key in self._episode_sums.keys():
                episodic_sum_avg = torch.mean(self._episode_sums[key][env_ids])
                extras["Episode_Reward/" + key] = episodic_sum_avg / self.env.max_episode_length_s
                self._episode_sums[key][env_ids] = 0.0
            self.env.extras["log"] = dict()
            self.env.extras["log"].update(extras)
            extras = dict()
            extras["Episode_Termination/died"] = torch.count_nonzero(self.env.reset_terminated[env_ids]).item()
            extras["Episode_Termination/time_out"] = torch.count_nonzero(self.env.reset_time_outs[env_ids]).item()
            self.env.extras["log"].update(extras)

        # Call robot reset first
        self.env._robot.reset(env_ids)

        # Initialize model paths if needed
        if not self.env._models_paths_initialized:
            num_models_per_env = self.env._waypoints.size(0)
            model_prim_names_in_env = [f"{self.env.target_models_prim_base_name}_{i}" for i in range(num_models_per_env)]

            self.env._all_target_models_paths = []
            for env_path in self.env.scene.env_prim_paths:
                paths_for_this_env = [f"{env_path}/{name}" for name in model_prim_names_in_env]
                self.env._all_target_models_paths.append(paths_for_this_env)

            self.env._models_paths_initialized = True

        n_reset = len(env_ids)
        if n_reset == self.num_envs and self.num_envs > 1:
            self.env.episode_length_buf = torch.randint_like(self.env.episode_length_buf,
                                                             high=int(self.env.max_episode_length))

        # Reset action buffers
        self.env._actions[env_ids] = 0.0
        self.env._previous_actions[env_ids] = 0.0
        self.env._previous_yaw[env_ids] = 0.0
        self.env._motor_speeds[env_ids] = 0.0
        self.env._previous_omega_meas[env_ids] = 0.0
        self.env._previous_omega_err[env_ids] = 0.0
        self.env._omega_err_integral[env_ids] = 0.0

        # Reset joints state
        joint_pos = self.env._robot.data.default_joint_pos[env_ids]
        joint_vel = self.env._robot.data.default_joint_vel[env_ids]
        self.env._robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)

        default_root_state = self.env._robot.data.default_root_state[env_ids]

        # TODO ----- START ----- Define the initial state during training after resetting an environment.

        # ----------------------------------------------------------------
        # Domain randomization: re-sample dynamics each episode so the policy
        # learns to be robust to the perturbation ranges used in evaluation.
        # ----------------------------------------------------------------
        if self.cfg.is_train:
            # DR curriculum: start narrow, widen to full range over training
            dr_frac = min(1.0, 0.2 + 0.8 * (self.env.iteration / 2000.0))

            # Thrust-to-weight ratio: ±5% scaled by dr_frac
            twr_delta = 0.05 * dr_frac
            self.env._thrust_to_weight[env_ids] = torch.empty(
                n_reset, device=self.device
            ).uniform_(self.env._twr_value * (1.0 - twr_delta), self.env._twr_value * (1.0 + twr_delta))

            # Aerodynamic drag coefficients: range scaled by dr_frac
            drag_lo = 1.0 - 0.5 * dr_frac   # 1.0 → 0.5
            drag_hi = 1.0 + 1.0 * dr_frac   # 1.0 → 2.0
            k_xy = torch.empty(n_reset, device=self.device).uniform_(
                self.env._k_aero_xy_value * drag_lo, self.env._k_aero_xy_value * drag_hi
            )
            self.env._K_aero[env_ids, 0] = k_xy
            self.env._K_aero[env_ids, 1] = k_xy
            self.env._K_aero[env_ids, 2] = torch.empty(n_reset, device=self.device).uniform_(
                self.env._k_aero_z_value * drag_lo, self.env._k_aero_z_value * drag_hi
            )

            # PID gains – roll/pitch: kp/ki ±15%, kd ±30%, scaled by dr_frac
            kpki_rp_delta = 0.15 * dr_frac
            kd_rp_delta = 0.30 * dr_frac
            kp_rp = torch.empty(n_reset, device=self.device).uniform_(
                self.env._kp_omega_rp_value * (1.0 - kpki_rp_delta), self.env._kp_omega_rp_value * (1.0 + kpki_rp_delta)
            )
            ki_rp = torch.empty(n_reset, device=self.device).uniform_(
                self.env._ki_omega_rp_value * (1.0 - kpki_rp_delta), self.env._ki_omega_rp_value * (1.0 + kpki_rp_delta)
            )
            kd_rp = torch.empty(n_reset, device=self.device).uniform_(
                self.env._kd_omega_rp_value * (1.0 - kd_rp_delta), self.env._kd_omega_rp_value * (1.0 + kd_rp_delta)
            )
            self.env._kp_omega[env_ids, 0] = kp_rp
            self.env._kp_omega[env_ids, 1] = kp_rp
            self.env._ki_omega[env_ids, 0] = ki_rp
            self.env._ki_omega[env_ids, 1] = ki_rp
            self.env._kd_omega[env_ids, 0] = kd_rp
            self.env._kd_omega[env_ids, 1] = kd_rp

            # PID gains – yaw: kp/ki ±15%, kd ±30%, scaled by dr_frac
            kpki_y_delta = 0.15 * dr_frac
            kd_y_delta = 0.30 * dr_frac
            kp_y = torch.empty(n_reset, device=self.device).uniform_(
                self.env._kp_omega_y_value * (1.0 - kpki_y_delta), self.env._kp_omega_y_value * (1.0 + kpki_y_delta)
            )
            ki_y = torch.empty(n_reset, device=self.device).uniform_(
                self.env._ki_omega_y_value * (1.0 - kpki_y_delta), self.env._ki_omega_y_value * (1.0 + kpki_y_delta)
            )
            kd_y = torch.empty(n_reset, device=self.device).uniform_(
                self.env._kd_omega_y_value * (1.0 - kd_y_delta), self.env._kd_omega_y_value * (1.0 + kd_y_delta)
            )
            self.env._kp_omega[env_ids, 2] = kp_y
            self.env._ki_omega[env_ids, 2] = ki_y
            self.env._kd_omega[env_ids, 2] = kd_y

        # ----------------------------------------------------------------
        # Curriculum starting positions: randomly sample ANY gate on the track.
        # Approach distance grows with training (curriculum).
        # 20% of resets start from ground near gate 0 to match eval conditions.
        # ----------------------------------------------------------------
        num_gates_reset = self.env._waypoints.shape[0]
        waypoint_indices = torch.randint(
            0, num_gates_reset, (n_reset,), device=self.device, dtype=self.env._idx_wp.dtype
        )

        # Gate world pose
        x0_wp = self.env._waypoints[waypoint_indices, 0]
        y0_wp = self.env._waypoints[waypoint_indices, 1]
        z_wp  = self.env._waypoints[waypoint_indices, 2]
        theta = self.env._waypoints[waypoint_indices, -1]

        # Curriculum: approach distance grows with training progress
        progress_frac = min(1.0, self.env.iteration / 2000.0)
        min_approach = 0.5 + 0.7 * progress_frac   # 0.5 → 1.2
        max_approach = 1.5 + 2.0 * progress_frac   # 1.5 → 3.5
        approach_d = torch.empty(n_reset, device=self.device).uniform_(min_approach, max_approach)
        lat_offset = torch.empty(n_reset, device=self.device).uniform_(-0.35, 0.35)
        z_offset   = torch.empty(n_reset, device=self.device).uniform_(-0.25, 0.25)

        # Rotate from gate local frame to world frame
        cos_theta = torch.cos(theta)
        sin_theta = torch.sin(theta)
        dx_w = cos_theta * approach_d - sin_theta * lat_offset
        dy_w = sin_theta * approach_d + cos_theta * lat_offset

        initial_x = x0_wp + dx_w
        initial_y = y0_wp + dy_w
        initial_z = (z_wp + z_offset).clamp(0.15, self.env.cfg.max_altitude - 0.2)

        default_root_state[:, 0] = initial_x
        default_root_state[:, 1] = initial_y
        default_root_state[:, 2] = initial_z
        # Small initial velocity noise for robustness
        default_root_state[:, 7:10] = torch.empty(n_reset, 3, device=self.device).uniform_(-0.3, 0.3)
        default_root_state[:, 10:] = 0.0  # zero angular velocity

        # Point drone toward chosen gate + small yaw noise
        initial_yaw = torch.atan2(y0_wp - initial_y, x0_wp - initial_x)
        quat = quat_from_euler_xyz(
            torch.zeros(n_reset, device=self.device),
            torch.zeros(n_reset, device=self.device),
            initial_yaw + torch.empty(n_reset, device=self.device).uniform_(-0.20, 0.20),
        )
        default_root_state[:, 3:7] = quat

        # ----------------------------------------------------------------
        # Ground starts: 20% of resets start from ground near gate 0.
        # This mimics the TA evaluation condition (drone starts at z≈0.05
        # with x_local ∈ [-3, -0.5], y_local ∈ [-1, 1] relative to gate 0).
        # ----------------------------------------------------------------
        if self.cfg.is_train:
            ground_start_mask = torch.rand(n_reset, device=self.device) < 0.2
            gs_ids = ground_start_mask.nonzero(as_tuple=True)[0]
            n_gs = len(gs_ids)
            if n_gs > 0:
                waypoint_indices[gs_ids] = 0
                x_local_gs = torch.empty(n_gs, device=self.device).uniform_(-3.0, -0.5)
                y_local_gs = torch.empty(n_gs, device=self.device).uniform_(-1.0, 1.0)
                x0_wp_gs = self.env._waypoints[0, 0]
                y0_wp_gs = self.env._waypoints[0, 1]
                theta_gs = self.env._waypoints[0, -1]
                cos_t, sin_t = torch.cos(theta_gs), torch.sin(theta_gs)
                x_rot = cos_t * x_local_gs - sin_t * y_local_gs
                y_rot = sin_t * x_local_gs + cos_t * y_local_gs
                default_root_state[gs_ids, 0] = x0_wp_gs - x_rot
                default_root_state[gs_ids, 1] = y0_wp_gs - y_rot
                default_root_state[gs_ids, 2] = 0.05
                default_root_state[gs_ids, 7:] = 0.0  # zero velocity for ground start
                # Point toward gate 0
                yaw_gs = torch.atan2(
                    y0_wp_gs - default_root_state[gs_ids, 1],
                    x0_wp_gs - default_root_state[gs_ids, 0],
                )
                quat_gs = quat_from_euler_xyz(
                    torch.zeros(n_gs, device=self.device),
                    torch.zeros(n_gs, device=self.device),
                    yaw_gs,
                )
                default_root_state[gs_ids, 3:7] = quat_gs

            # ----------------------------------------------------------------
            # Horizontal arc starts: 30% of resets spawn near gate 2 exit
            # with target=gate 3. Velocity has +X (rightward into arc) and
            # -Y (post-gate-2 momentum) components for the clockwise arc.
            # 0.375 of non-ground resets ≈ 30% of total.
            # ----------------------------------------------------------------
            arc_mask = (~ground_start_mask) & (torch.rand(n_reset, device=self.device) < 0.375)
            arc_ids = arc_mask.nonzero(as_tuple=True)[0]
            n_arc = len(arc_ids)
            if n_arc > 0:
                waypoint_indices[arc_ids] = 3  # target gate 3
                gate2_pos = self.env._waypoints[2, :3]
                # Spawn just past gate 2 exit with slight offsets
                default_root_state[arc_ids, 0] = gate2_pos[0] + torch.empty(n_arc, device=self.device).uniform_(-0.2, 0.4)
                default_root_state[arc_ids, 1] = gate2_pos[1] + torch.empty(n_arc, device=self.device).uniform_(-0.8, -0.1)
                default_root_state[arc_ids, 2] = torch.empty(n_arc, device=self.device).uniform_(0.5, 1.0)
                # Post-gate-2 velocity: +X (rightward into arc) and -Y (through-gate momentum)
                default_root_state[arc_ids, 7] = torch.empty(n_arc, device=self.device).uniform_(0.0, 1.5)
                default_root_state[arc_ids, 8] = torch.empty(n_arc, device=self.device).uniform_(-2.0, -0.5)
                default_root_state[arc_ids, 9] = torch.empty(n_arc, device=self.device).uniform_(-0.5, 0.5)
                default_root_state[arc_ids, 10:] = 0.0
                yaw_arc = torch.empty(n_arc, device=self.device).uniform_(-0.3, 0.8)  # biased toward +X
                quat_arc = quat_from_euler_xyz(
                    torch.zeros(n_arc, device=self.device),
                    torch.zeros(n_arc, device=self.device),
                    yaw_arc,
                )
                default_root_state[arc_ids, 3:7] = quat_arc

        # TODO ----- END -----

        # Handle play mode initial position
        if not self.cfg.is_train:
            # x_local and y_local are randomly sampled
            x_local = torch.empty(1, device=self.device).uniform_(-3.0, -0.5)
            y_local = torch.empty(1, device=self.device).uniform_(-1.0, 1.0)

            x0_wp = self.env._waypoints[self.env._initial_wp, 0]
            y0_wp = self.env._waypoints[self.env._initial_wp, 1]
            theta = self.env._waypoints[self.env._initial_wp, -1]

            # rotate local pos to global frame
            cos_theta, sin_theta = torch.cos(theta), torch.sin(theta)
            x_rot = cos_theta * x_local - sin_theta * y_local
            y_rot = sin_theta * x_local + cos_theta * y_local
            x0 = x0_wp - x_rot
            y0 = y0_wp - y_rot
            z0 = 0.05

            # point drone towards the zeroth gate
            yaw0 = torch.atan2(y0_wp - y0, x0_wp - x0)

            default_root_state = self.env._robot.data.default_root_state[0].unsqueeze(0)
            default_root_state[:, 0] = x0
            default_root_state[:, 1] = y0
            default_root_state[:, 2] = z0

            quat = quat_from_euler_xyz(
                torch.zeros(1, device=self.device),
                torch.zeros(1, device=self.device),
                yaw0
            )
            default_root_state[:, 3:7] = quat
            waypoint_indices = self.env._initial_wp

        # Set waypoint indices and desired positions
        self.env._idx_wp[env_ids] = waypoint_indices

        self.env._desired_pos_w[env_ids, :2] = self.env._waypoints[waypoint_indices, :2].clone()
        self.env._desired_pos_w[env_ids, 2] = self.env._waypoints[waypoint_indices, 2].clone()

        self.env._n_gates_passed[env_ids] = 0

        # Write state to simulation
        self.env._robot.write_root_link_pose_to_sim(default_root_state[:, :7], env_ids)
        self.env._robot.write_root_com_velocity_to_sim(default_root_state[:, 7:], env_ids)

        # Reset variables
        self.env._yaw_n_laps[env_ids] = 0

        self.env._pose_drone_wrt_gate[env_ids], _ = subtract_frame_transforms(
            self.env._waypoints[self.env._idx_wp[env_ids], :3],
            self.env._waypoints_quat[self.env._idx_wp[env_ids], :],
            self.env._robot.data.root_link_state_w[env_ids, :3]
        )

        # Fix 7: use 3D gate-frame distance (AFTER pose recomputation)
        self.env._last_distance_to_goal[env_ids] = torch.linalg.norm(
            self.env._pose_drone_wrt_gate[env_ids], dim=1
        )

        # For arc spawns (target=gate3), init distance to ARC_WP1
        if self.cfg.is_train and isinstance(waypoint_indices, torch.Tensor):
            is_gate3 = (waypoint_indices == 3)
            if is_gate3.any():
                arc_wp1_reset = torch.tensor([0.6, -0.6, 0.75], device=self.device)
                g3_ids = env_ids[is_gate3]
                drone_pos_g3 = self.env._robot.data.root_link_state_w[g3_ids, :3]
                self.env._last_distance_to_goal[g3_ids] = torch.linalg.norm(
                    drone_pos_g3 - arc_wp1_reset.unsqueeze(0), dim=1
                )

        self.env._prev_x_drone_wrt_gate[env_ids] = 1.0
        self._prev_x_all_gates[env_ids] = 1.0
        # Reset arc state trackers
        self._was_in_arc[env_ids] = False
        self._arc_phase[env_ids] = 0
        self._prev_arc_phase[env_ids] = 0

        self.env._crashed[env_ids] = 0