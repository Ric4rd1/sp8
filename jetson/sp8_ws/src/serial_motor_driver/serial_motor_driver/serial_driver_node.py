import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial


class SerialMotorDriver(Node):

    def __init__(self):
        super().__init__('serial_motor_driver')

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        # Serial connection
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f"Connected to {port}")
        except Exception as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            self.ser = None

        # Subscriber
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )
        # Publisher


    def cmd_callback(self, msg: Twist):
        if self.ser is None:
            return

        linear = msg.linear.x
        angular = msg.angular.z

        # Example format → customize for your motor controller
        command = f"linear: {linear}, angular: {angular}\n"
        self.get_logger().info(f'sending {command}')

        try:
            self.ser.write(command.encode())
        except Exception as e:
            self.get_logger().error(f"Write failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SerialMotorDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
        main()