import socket
import struct
import select
import rclpy
from rclpy.node import Node
from jirl_interfaces.msg import CommandCTBR

class SetpointReceiverNode(Node):
    def __init__(self):
        super().__init__('setpoint_receiver')

        self.publisher = self.create_publisher(CommandCTBR, '/cf/ctbr_command', 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 1234))
        self.sock.setblocking(False)

        self.timer = self.create_timer(0.001, self.read_socket)

    def read_socket(self):
        readable, _, _ = select.select([self.sock], [], [], 0)
        if readable:
            try:
                data, _ = self.sock.recvfrom(1024)
                roll, pitch, yawrate, thrust = struct.unpack('<ffff', data)

                msg = CommandCTBR()
                msg.thrust_pwm = int(thrust)
                msg.roll_rate = roll
                msg.pitch_rate = pitch
                msg.yaw_rate = yawrate

                self.publisher.publish(msg)
            except Exception as e:
                self.get_logger().warn(f"Socket error: {e}")

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SetpointReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
