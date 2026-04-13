from threading import Lock

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from jirl_interfaces.msg import CommandCTBR, Trajectory, Observations, OdometryArray
from jirl_interfaces.srv import UpdateSetpoint, StartTrajectory

from rotorpy.controllers.quadrotor_control import SE3ControlCTBR

from rotorpy.vehicles.crazyflie_params import quad_params as crazyflie_params

import torch

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie import Crazyflie
from cflib.utils import uri_helper

from .controller_qos import *
from .controller_fsm import ControllerFSM
from .controller_simple_policy import SimpleRacingPolicy

class ControllerNode(Node):

    # Import methods
    from .controller_params import init_parameters
    from .controller_callbacks import mocap_clbk, multi_mocap_clbk, logger_clbk, update_setpoint_clbk, landing_clbk, takeoff_clbk, trajectory_clbk, race_clbk, stop_clbk
    from .controller_utils import send_ctbr_command, send_trajectory, single_update

    traj_lock = Lock()
    mocap_pose = {}
    device = torch.device('cpu')

    scf_dict = {}

    def __init__(self):
        super().__init__('controller')

        self.init_parameters()
        self.init_crazyflie()
        self.init_fsm()
        self.init_publishers()
        self.init_controllers()
        self.init_callback_groups()
        self.init_services()
        self.init_subscriptions()

        self.get_logger().warn('Node initialized')

    def cleanup(self):
        for scf in self.scf_dict.values():
            scf.close_link()

    def init_crazyflie(self):
        """
        Init crazyflie
        """
        if self.driver_enable:
            cflib.crtp.init_drivers()

            for crazyradio_uri, crazyflie_name in zip(self.driver_uris, self.driver_names):
                if crazyflie_name in self.scf_dict:
                    connected = self.scf_dict[crazyflie_name].is_link_open()
                    if not connected:
                        self.get_logger().error('Crazyflie %s disconnected' % (crazyflie_name))
                        self.scf_dict.pop(crazyflie_name)
                else:
                    self.get_logger().warn('Trying to connect to Crazyflie %s...' % crazyflie_name)
                    try:
                        URI = uri_helper.uri_from_env(default=crazyradio_uri)

                        self.scf_dict[crazyflie_name] = SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache'))
                        self.scf_dict[crazyflie_name].open_link()

                        self.get_logger().warn('Sending zero command to %s' % crazyradio_uri)
                        self.scf_dict[crazyflie_name].cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
                        self.get_logger().warn('Connected to %s' % crazyradio_uri)

                        self.scf_dict[crazyflie_name].cf.param.set_value('stabilizer.estimator', '2')
                        self.scf_dict[crazyflie_name].param.set_value('locSrv.extQuatStdDev', 0.06)
                    except Exception as e:
                        continue

    def init_fsm(self):
        """
        Init FSM
        """
        self.fsm = ControllerFSM()

    def print_state(self):
        self.get_logger().info(f'Entering state: {self.fsm.state}')

    def init_controllers(self):
        """
        Init controllers
        """
        self.policy = SimpleRacingPolicy(crazyflie_params, self.policy_path, self.params, device=self.device, use_cond=False)
        self.se3_controller = SE3ControlCTBR(crazyflie_params)

    def init_callback_groups(self):
        """
        Init callback groups
        """
        # Subscribers
        self.mocap_cgroup = MutuallyExclusiveCallbackGroup()
        self.stop_cgroup = MutuallyExclusiveCallbackGroup()

        # Timers
        self.cmd_cgroup = MutuallyExclusiveCallbackGroup()

    def init_publishers(self):
        """
        Init publishers
        """
        # CTBR command
        if not self.driver_enable:
            if self.driver_ext == 'cpp':
                topic_name = 'ctbr_cmd'
            elif self.driver_ext in ['py', 'python']:
                topic_name = '/ctbr_cmd'
        else:
            topic_name = '/ctbr_cmd'

        self.cmd_pub = self.create_publisher(
            CommandCTBR,
            topic_name,
            qos_best_effort
        )

        # Trajectory data
        self.traj_pub = self.create_publisher(
            Trajectory,
            'trajectory',
            qos_best_effort
        )

        # Observations
        self.obs_pub = self.create_publisher(
            Observations,
            'observations',
            qos_best_effort
        )

    def init_subscriptions(self):
        """
        Init subscriptions
        """
        if self.driver_enable:
            # Multi mocap odometry
            self.mocap_sub = self.create_subscription(
                OdometryArray,
                '/multi_odometry',
                self.multi_mocap_clbk,
                qos_best_effort,
                callback_group=self.mocap_cgroup
            )
        else:
            # Mocap odometry
            self.mocap_sub = self.create_subscription(
                Odometry,
                '/mocap',
                self.mocap_clbk,
                qos_best_effort,
                callback_group=self.mocap_cgroup
            )

        self.stop_sub = self.create_subscription(
                Empty,
                'stop',
                self.stop_clbk,
                qos_best_effort,
                callback_group=self.stop_cgroup
            )

    def init_services(self):
        """
        Init services
        """
        # Setpoint update
        self.update_setpoint_srv = self.create_service(
            UpdateSetpoint,
            'update_setpoint',
            self.update_setpoint_clbk)

        # StartTrajectory
        self.trajectory_srv = self.create_service(
            StartTrajectory,
            'trajectory',
            self.trajectory_clbk)

        # Landing
        self.land_srv = self.create_service(
            Trigger,
            'land',
            self.landing_clbk)

        # Takeoff
        self.takeoff_srv = self.create_service(
            Trigger,
            'takeoff',
            self.takeoff_clbk)

        # Start racing
        self.racing_srv = self.create_service(
            Trigger,
            'race',
            self.race_clbk)