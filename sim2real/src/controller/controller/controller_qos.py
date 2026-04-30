import rclpy.qos as qos

qos_best_effort = qos.QoSProfile(
    depth=10,
    history=qos.HistoryPolicy.KEEP_LAST,
    reliability=qos.ReliabilityPolicy.BEST_EFFORT,
    durability=qos.DurabilityPolicy.VOLATILE)

qos_reliable = qos.QoSProfile(
    depth=10,
    history=qos.HistoryPolicy.KEEP_LAST,
    reliability=qos.ReliabilityPolicy.RELIABLE,
    durability=qos.DurabilityPolicy.VOLATILE)