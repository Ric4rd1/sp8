import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from rclpy import qos
import serial


class SerialMotorDriver(Node):

    def __init__(self):
        super().__init__('serial_motor_driver')

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        self.msg_l = Float32()
        self.msg_r = Float32()

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

        # Publishers
        self.l_wheel_pub = self.create_publisher(
            Float32,
            '/wl',
            qos_profile=qos.qos_profile_sensor_data
        )

        self.r_wheel_pub = self.create_publisher(
            Float32,
            '/wr',
            qos_profile=qos.qos_profile_sensor_data
        )


        # Timer (reading serial wl and wr)
        timer_period = 0.045
        self.timer = self.create_timer(timer_period, self.timer_cb)
    
    def timer_cb(self):
        if self.ser is None:
            return

        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                #self.get_logger().info(f"Received: {line}")

                # Expected:
                # ang_vel_l: 1.2345, ang_vel_r: 1.5678

                parts = line.split(',')

                wl = None
                wr = None

                for part in parts:
                    key, value = part.split(':')
                    key = key.strip()
                    value = value.strip()

                    if key == 'ang_vel_l':
                        wl = float(value)
                    elif key == 'ang_vel_r':
                        wr = float(value)

                if wl is not None and wr is not None:
                    
                    self.msg_l.data = wl   # left wheel
                    self.msg_r.data = wr   # right wheel

                    self.l_wheel_pub.publish(self.msg_l)
                    self.r_wheel_pub.publish(self.msg_r)

        except Exception as e:
            self.get_logger().warn(f"Read/parse error: {e}")

    def cmd_callback(self, msg: Twist):
        if self.ser is None:
            return

        linear = msg.linear.x
        angular = msg.angular.z

        # command format 
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