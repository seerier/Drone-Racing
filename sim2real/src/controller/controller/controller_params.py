import numpy as np
from scipy.spatial.transform import Rotation as R

def init_parameters(self):
    """
    Init parameters
    """
    # Declare parameters
    self.declare_parameters(
        namespace='',
        parameters=[('crazyradio_driver.enable', True),
                    ('crazyradio_driver.crazyflie_names', ['']),
                    ('crazyradio_driver.crazyradio_uris', ['']),
                    ('crazyradio_driver.ext_driver', ''),
                    ('gate_side', 0.0),
                    ('low_level_controller.c1', 0.0),
                    ('low_level_controller.c2', 0.0),
                    ('low_level_controller.c3', 0.0),
                    ('low_level_controller.thrust_pwm_min', 0),
                    ('low_level_controller.thrust_pwm_max', 0),
                    ('policy.waypoints', [0.0]),
                    ('policy.initial_waypoint', 0),
                    ('takeoff_height', 0.0),
                    ('policy.max_roll_br', 0.0),
                    ('policy.max_pitch_br', 0.0),
                    ('policy.max_yaw_br', 0.0),
                   ])

    # Get parameters
    self.driver_enable = self.get_parameter('crazyradio_driver.enable').value
    self.driver_names = self.get_parameter('crazyradio_driver.crazyflie_names').value
    self.driver_uris = self.get_parameter('crazyradio_driver.crazyradio_uris').value
    self.driver_ext = self.get_parameter('crazyradio_driver.ext_driver').value
    self.gate_side = self.get_parameter('gate_side').value
    self.low_level_controller_c1 = self.get_parameter('low_level_controller.c1').value
    self.low_level_controller_c2 = self.get_parameter('low_level_controller.c2').value
    self.low_level_controller_c3 = self.get_parameter('low_level_controller.c3').value
    self.low_level_controller_thrust_pwm_min = self.get_parameter('low_level_controller.thrust_pwm_min').value
    self.low_level_controller_thrust_pwm_max = self.get_parameter('low_level_controller.thrust_pwm_max').value
    self.policy_max_roll_br = self.get_parameter('policy.max_roll_br').value
    self.policy_max_pitch_br = self.get_parameter('policy.max_pitch_br').value
    self.policy_max_yaw_br = self.get_parameter('policy.max_yaw_br').value
    self.takeoff_height = self.get_parameter('takeoff_height').value
    waypoints_flat = np.array(self.get_parameter('policy.waypoints').value, dtype=np.float32)
    self.waypoints = waypoints_flat.reshape(-1, 6)
    self.initial_waypoint = self.get_parameter('policy.initial_waypoint').value

    # Read per-drone policy paths from YAML (ROS2 doesn't support dict parameters directly)
    import yaml
    yaml_file = '/home/neo/workspace/src/jirl_bringup/config/config.yaml'
    with open(yaml_file, 'r') as f:
        config_data = yaml.safe_load(f)
    policy_config = config_data.get("/*/controller", {}).get("ros__parameters", {}).get("policy", {})
    self.policy_paths_per_drone = policy_config.get("paths_per_drone", {})

    # Get ego drone name from namespace
    self.ego_name = self.get_namespace().strip('/')

    # Get drone-specific policy path (required)
    if self.ego_name in self.policy_paths_per_drone:
        self.policy_path = self.policy_paths_per_drone[self.ego_name]
        self.get_logger().info(f'Using policy path for {self.ego_name}: {self.policy_path}')
    else:
        self.get_logger().error(f'No policy path configured for drone {self.ego_name} in paths_per_drone!')
        raise ValueError(f'No policy path configured for drone {self.ego_name}. Please add it to paths_per_drone in config.yaml')

    # Print parameters
    self.get_logger().info(f'crazyradio_enable: {self.driver_enable}')
    self.get_logger().info(f'crazyradio_uris: {self.driver_names}')
    self.get_logger().info(f'crazyradio_driver: {self.driver_uris}')
    self.get_logger().info(f'crazyradio_external_driver: {self.driver_ext}')

    self.get_logger().info(f'c1: {self.low_level_controller_c1}')
    self.get_logger().info(f'c2: {self.low_level_controller_c2}')
    self.get_logger().info(f'c3: {self.low_level_controller_c3}')
    self.get_logger().info(f'gate_side: {self.gate_side}')
    self.get_logger().info(f'thrust_pwm_min: {self.low_level_controller_thrust_pwm_min}')
    self.get_logger().info(f'thrust_pwm_max: {self.low_level_controller_thrust_pwm_max}')
    self.get_logger().info(f'policy_path: {self.policy_path}')
    self.get_logger().info(f'policy_waypoints:\n{self.waypoints}')
    self.get_logger().info(f'policy_max_roll_br: {self.policy_max_roll_br}')
    self.get_logger().info(f'policy_max_pitch_br: {self.policy_max_pitch_br}')
    self.get_logger().info(f'policy_max_yaw_br: {self.policy_max_yaw_br}')
    self.get_logger().info(f'initial_waypoint: {self.initial_waypoint}')
    self.get_logger().info(f'takeoff_height: {self.takeoff_height}')
    self.get_logger().info(f'ego_name: {self.ego_name}')

    #
    self.waypoints_quat = np.zeros((self.waypoints.shape[0], 4), dtype=np.float32)
    self.params = {
        "waypoints": self.waypoints,
        "waypoints_quat": self.waypoints_quat,
        "gate_side": self.gate_side,
        "initial_waypoint": self.initial_waypoint,
        "max_roll_br": self.policy_max_roll_br,
        "max_pitch_br": self.policy_max_pitch_br,
        "max_yaw_br": self.policy_max_yaw_br,
    }

    for i, waypoint_data in enumerate(self.waypoints):
        euler_np = waypoint_data[3:6]
        rot_from_euler = R.from_euler('xyz', euler_np)
        self.waypoints_quat[i, :] = rot_from_euler.as_quat(scalar_first=True)
