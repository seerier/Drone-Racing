import time
import numpy as np
from jirl_interfaces.msg import CommandCTBR, Trajectory, Observations
from nav_msgs.msg import Odometry

from rotorpy.trajectories.hover_traj import HoverTraj

from scipy.spatial.transform import Rotation as R

def send_ctbr_command(self, thrust_pwm, thrust_N, roll_rate, pitch_rate, yaw_rate):
    """
    Send CTBR command
    """
    command_msg = CommandCTBR()
    command_msg.crazyflie_name = self.get_namespace().split('/')[-1]
    command_msg.thrust_pwm = thrust_pwm
    command_msg.thrust_n = thrust_N
    command_msg.roll_rate = float(roll_rate)
    command_msg.pitch_rate = float(pitch_rate)
    command_msg.yaw_rate = float(yaw_rate)

    self.cmd_pub.publish(command_msg)

def send_trajectory(self, traj):
    """
    Send trajectory
    """
    traj_msg = Trajectory()
    traj_msg.x = np.array(traj['x'], dtype=np.float64)
    traj_msg.x_dot = np.array(traj['x_dot'], dtype=np.float64)
    traj_msg.x_ddot = np.array(traj['x_ddot'], dtype=np.float64)
    traj_msg.x_dddot = np.array(traj['x_dddot'], dtype=np.float64)
    traj_msg.x_ddddot = np.array(traj['x_ddddot'], dtype=np.float64)
    traj_msg.yaw = float(traj['yaw'])
    traj_msg.yaw_dot = float(traj['yaw_dot'])
    traj_msg.yaw_ddot = float(traj['yaw_ddot'])

    self.traj_pub.publish(traj_msg)

def single_update(self, msg: Odometry):
    p = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
    v_w = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
    w_w = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])
    quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    R_mat = R.from_quat(quat).as_matrix()
    v_b = R_mat.T @ v_w
    w_b = R_mat.T @ w_w

    self.mocap_pose['x'] = p
    self.mocap_pose['R'] = R_mat
    self.mocap_pose['q'] = quat
    self.mocap_pose['yaw'] = R.from_matrix(R_mat).as_euler('zyx')[0]
    self.mocap_pose['v_b'] = v_b
    self.mocap_pose['w_b'] = w_b
    self.mocap_pose['v_w'] = v_w
    self.mocap_pose['w_w'] = w_w

    thrust_pwm_min = self.low_level_controller_thrust_pwm_min
    thrust_pwm_max = self.low_level_controller_thrust_pwm_max

    if self.fsm.state == 'racing':
        control, obs = self.policy.update(self.mocap_pose)

        # Lap timing: detect when idx_wp transitions from 0 to 1 (completed gate 0)
        if hasattr(self, 'prev_idx_wp') and self.prev_idx_wp is not None:
            if self.policy.idx_wp == 1 and self.prev_idx_wp == 0:
                lap_time = time.time() - self.lap_start_time
                total_time = time.time() - self.race_start_time
                self.lap_count += 1
                print(f"[LAP {self.lap_count}] Lap: {lap_time:.3f}s | Total: {total_time:.3f}s")
                self.lap_start_time = time.time()
            self.prev_idx_wp = self.policy.idx_wp

        obs_msg = Observations()
        obs_msg.lin_vel = obs[0:3]
        obs_msg.rot = obs[3:12]
        obs_msg.corners_pos_b_curr = obs[12:24]
        obs_msg.corners_pos_b_next = obs[24:36]

        self.obs_pub.publish(obs_msg)

        thrust_des_perc = control['cmd_thrust']
        thrust_pwm = int(thrust_pwm_min + thrust_des_perc * (thrust_pwm_max - thrust_pwm_min))

        thrust_des_newtons = thrust_des_perc * (0.038 * 9.81 * 3.15)
    else:
        if self.fsm.state == 'landed':
            return
        elif self.fsm.state == 'taking_off':
            x0 = [self.p0[0], self.p0[1], self.takeoff_height]
            self.flat_output = HoverTraj(x0=x0, yaw0=self.mocap_pose['yaw']).update(0)

            if (time.time() - self.t0 > 3.0):
                # Change FSM state
                self.fsm.in_position()
                self.get_logger().info("[FSM] Hovering")
        elif self.fsm.state == 'hovering':
            pass
        elif self.fsm.state == 'landing':
            if (time.time() - self.t0 < 1.0):
                x0 = [self.p0[0], self.p0[1], 0.15]
            else:
                x0 = [self.p0[0], self.p0[1], 0.05]
            self.flat_output = HoverTraj(x0=x0).update(0)

            if (time.time() - self.t0 > 3.0):
                for _ in range(30):
                    self.send_ctbr_command(0, 0.0, 0.0, 0.0, 0.0)
                    time.sleep(0.1)

                self.get_logger().info("[FSM] Landed")
                self.fsm.landing_complete()

                return 0, 0.0, 0.0, 0.0
        elif self.fsm.state == 'flying':
            if self.dt > self.traj_duration:
                self.get_logger().info(f"Finished circular trajectory")
                self.flat_output = HoverTraj(x0=p).update(0)
                self.fsm.stop()
            else:
                self.dt = time.time() - self.t0
                self.flat_output = self.trajectory.update(self.dt)

        # Publish trajectory
        self.send_trajectory(self.flat_output)

        # Apply control
        control = self.se3_controller.update(0, self.mocap_pose, self.flat_output)

        c1 = self.low_level_controller_c1
        c2 = self.low_level_controller_c2
        c3 = self.low_level_controller_c3

        thrust_des_newtons = control['cmd_thrust']
        thrust_des_grams = thrust_des_newtons / 9.81 * 1000
        if c3 + thrust_des_grams < 0:
            thrust_des_grams = 0
        thrust_pwm = c1 + c2 * (c3 + thrust_des_grams)**.5
        thrust_pwm = thrust_pwm * thrust_pwm_max + thrust_pwm_min * 1.0
        thrust_pwm = int(min(max(thrust_pwm_min, thrust_pwm), thrust_pwm_max))

    w_des = control['cmd_w']        # deg/s

    # Publish command msg
    self.send_ctbr_command(thrust_pwm, thrust_des_newtons, w_des[0], w_des[1], w_des[2])

    return thrust_pwm, w_des[0], w_des[1], w_des[2]
