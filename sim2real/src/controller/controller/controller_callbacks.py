import numpy as np
import time

from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

from jirl_interfaces.msg import OdometryArray
from jirl_interfaces.srv import StartTrajectory

from rotorpy.trajectories.hover_traj import HoverTraj
from rotorpy.trajectories.circular_traj import CircularTraj

def update_setpoint_clbk(self, request, response):
    x0 = np.array([request.x, request.y, request.z])

    if self.fsm.state == 'hovering':
        self.get_logger().info(f"Received new setpoint: {x0}")
    elif self.fsm.state == 'flying':
        self.fsm.hovering()                         # FIXME
        self.get_logger().info(f"[FSM] Hovering at new setpoint: {x0}")
    elif self.fsm.state == 'landed':
        if not request.is_global:
            self.get_logger().error("Setpoint must be global for manual launch")
            response.success = False
            return response
        self.fsm.launch()
        self.get_logger().info(f"[FSM] Waiting for launch...")
    elif self.fsm.state == 'racing':
        self.fsm.stop()
        self.get_logger().info(f"[FSM] Drone is stopping...")
    else:
        self.get_logger().info("Cannot update setpoint from current state")
        response.success = False
        return response

    with self.traj_lock:
        if request.is_global:
            new_pos = x0
            new_yaw = request.yaw
        else:
            new_pos = self.flat_output['x'] + x0
            new_yaw = self.mocap_pose['yaw'] + request.yaw
        self.flat_output = HoverTraj(x0=new_pos, yaw0=new_yaw).update(0)

    response.success = True
    return response

def trajectory_clbk(self, request, response):
    if self.fsm.state != 'hovering':
        self.get_logger().info("Cannot start trajectory from current state")
        response.success = False
        return response

    state = self.mocap_pose
    self.t0 = time.time()
    self.dt = 0.0

    if request.trajectory_type == StartTrajectory.Request.CIRCLE:
        radius = request.radius
        center = np.array([state['x'][0] - radius, state['x'][1], state['x'][2]])
        freq = request.freq
        yaw_bool = request.direction
        if request.plane == StartTrajectory.Request.PLANE_XY:
            plane = 'XY'
        elif request.plane == StartTrajectory.Request.PLANE_YZ:
            plane = 'YZ'
        elif request.plane == StartTrajectory.Request.PLANE_XZ:
            plane = 'XZ'
        direction = 'CW' if request.direction == StartTrajectory.Request.DIR_CW else 'CCW'
        self.traj_duration = request.duration

        self.trajectory = CircularTraj(center=center, radius=radius, freq=freq, yaw_bool=yaw_bool, plane=plane, direction=direction)

        self.get_logger().info(f"Starting circular trajectory")

    self.fsm.move()

    response.success = True
    return response

def takeoff_clbk(self, _, response):
    if self.mocap_pose == {}:
        self.get_logger().info("Mocap data not available yet")
        response.success = False
        return response

    if self.fsm.state != 'landed':
        self.get_logger().info("Cannot takeoff from current state")
        response.success = False
        return response

    self.t0 = time.time()
    self.p0 = self.mocap_pose['x']

    # Change FSM state
    self.fsm.takeoff()
    self.get_logger().info("[FSM] Taking off to %.2f, %.2f, %.2f" % (self.p0[0], self.p0[1], self.takeoff_height))

    response.success = True
    return response

def landing_clbk(self, _, response):
    if self.fsm.state != 'hovering':
        self.get_logger().info("Cannot land from current state")
        response.success = False
        return response

    self.t0 = time.time()
    self.p0 = self.mocap_pose['x']

    self.get_logger().info("[FSM] Landing")
    self.fsm.land()

    response.success = True
    return response

def race_clbk(self, _, response):
    if self.fsm.state not in ['hovering', 'landed']:
        self.get_logger().info("Cannot start racing from current state")
        response.success = False
        return response

    self.get_logger().info("[FSM] Start racing")
    self.fsm.race()

    # Initialize lap timing
    self.race_start_time = time.time()
    self.lap_start_time = time.time()
    self.lap_count = -1
    self.prev_idx_wp = self.policy.idx_wp

    response.success = True
    return response

def logger_clbk(self):
    for log_entry in self.sync_logger.next():
        print(log_entry)
        timestamp = log_entry[0]
        data = log_entry[1]
        name = log_entry[2]

        self.get_logger().info('[%d][%s]: %.3s' % (timestamp, name, data))

def mocap_clbk(self, msg: Odometry):
    """
    Mocap odometry callback
    """
    self.single_update(msg)

def multi_mocap_clbk(self, msg_array: OdometryArray):
    """
    Multi mocap odometry callback
    """
    # Process control commands for each drone
    for msg in msg_array.odom_array:
        cmds = self.single_update(msg)
        if cmds is not None:
            thrust_pwm, roll_rate, pitch_rate, yaw_rate = cmds
        else:
            return

        cf_name = msg.child_frame_id.split('/')[0]
        if cf_name in self.scf_dict:
            scf = self.scf_dict[cf_name]
            scf.cf.extpos.send_extpose(
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            )

        if cf_name in self.scf_dict:
            scf = self.scf_dict[cf_name]
            scf.cf.commander.send_setpoint(roll_rate, pitch_rate, -yaw_rate, thrust_pwm)


def stop_clbk(self, _: Empty):
    if self.fsm.state == 'hovering':
        return

    self.get_logger().info(f"[FSM] Drone is stopping...")
    x0 = self.mocap_pose['x']
    x0[2] = 0.5
    self.fsm.stop()
    self.flat_output = HoverTraj(x0=x0, yaw0=self.mocap_pose['yaw']).update(0)