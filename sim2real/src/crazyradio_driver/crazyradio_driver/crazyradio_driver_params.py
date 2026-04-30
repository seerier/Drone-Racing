def init_parameters(self):
    """
    Init parameters
    """
    # Declare parameters
    self.declare_parameters(
        namespace='',
        parameters=[
            ('crazyflie_names', ['']),
            ('crazyradio_uris', ['']),
            ('logger_period_ms', 0),
            ('reconnection_period_ms', 0),
        ])

    # Get parameters
    self.crazyflie_names = self.get_parameter('crazyflie_names').value
    self.crazyradio_uris = self.get_parameter('crazyradio_uris').value
    self.logger_period_ms = self.get_parameter('logger_period_ms').value
    self.reconnection_period_ms = self.get_parameter('reconnection_period_ms').value

    # Print parameters
    self.get_logger().info('crazyflie_names: %s' % self.crazyflie_names)
    self.get_logger().info('crazyradio_uris: %s' % self.crazyradio_uris)
    self.get_logger().info('logger_period_ms: %s' % self.logger_period_ms)
    self.get_logger().info('reconnection_period_ms: %s' % self.reconnection_period_ms)