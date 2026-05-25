import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
import numpy as np
import transforms3d


class Odom(Node):
    def __init__(self):
        super().__init__('odom')

        self.namespace = self.get_namespace().strip('/')

        # Sub
        self.l_wheel_sub = self.create_subscription(Float32, 'wl', self.wl_cb, qos_profile=qos_profile_sensor_data)
        self.r_wheel_sub = self.create_subscription(Float32, 'wr', self.wr_cb, qos_profile=qos_profile_sensor_data)
        # Pub
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        self.odom = Odometry()

        # timer
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.timer_cb)

        # Const
        self.declare_parameter('wheel_rad', 0.052)
        self.declare_parameter('wheel_base', 0.185)
        self.declare_parameter('kr', 0.45) # Tuning parameter for right wheel noise
        self.declare_parameter('kl', 0.4) # Tuning parameter for left wheel noise
        self.wheel_rad = self.get_parameter('wheel_rad').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.kr = self.get_parameter('kr').value
        self.kl = self.get_parameter('kl').value

        # Var
        self.linear_vel = 0.0
        self.ang_vel = 0.0
        self.wl = 0.0
        self.wr = 0.0
        self.sigma = np.zeros((3, 3))

        self.A = 0.0008285985222 # xx
        self.B = 0.0004207512315 # xy
        self.C = 11.52564103 # 00
        #self.C = .00002152564103

        self.Q_k_static = np.array([
            [self.A, self.B, self.A/10],
            [self.B, self.A, self.A/10],
            [self.C/20, self.C/20, self.C]
        ])

        # Robot vel and pose
        self.x_dot = 0.0 # vel along x
        self.y_dot = 0.0 # vel along y
        self.theta_dot = 0.0 # ang vel
        self.x = 0.0 # position in x
        self.y = 0.0 # position in y
        self.theta = 0.0 # angle position

        self.prev_time_ns = self.get_clock().now().nanoseconds #Previous time in nanoseconds 


    def wl_cb(self, msg):
        self.wl = msg.data
    def wr_cb(self, msg):
        self.wr = msg.data
        
    def timer_cb(self):
        current_time = self.get_clock().now().nanoseconds
        dt = (current_time - self.prev_time_ns) * 1e-9 # seconds

        prev_theta = self.theta

        # Compute linear and ang vel
        self.linear_vel = self.wheel_rad*((self.wr+self.wl)/2)
        self.ang_vel = self.wheel_rad*((self.wr-self.wl)/self.wheel_base)

        # Update cartisean vel
        self.x_dot = self.linear_vel*np.cos(self.theta)
        self.y_dot = self.linear_vel*np.sin(self.theta)
        self.theta_dot = self.ang_vel

        # Estimate cartinean pose
        self.x += self.x_dot*dt
        self.y += self.y_dot*dt
        self.theta += self.theta_dot*dt
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta)) # Crop this angle to [-pi,pi] 

        # -- Covariance calculation
        H = np.array([
            [1.0, 0.0, -dt * self.linear_vel * np.sin(prev_theta)],
            [0.0, 1.0,  dt * self.linear_vel * np.cos(prev_theta)],
            [0.0, 0.0,  1.0]
        ])

        Sigma_delta = np.array([
            [self.kr * abs(self.wr), 0.0],
            [0.0, self.kl * abs(self.wl)]
        ])

        Nabla_w = (0.5 * self.wheel_rad * dt) * np.array([
            [np.cos(prev_theta), np.cos(prev_theta)],
            [np.sin(prev_theta), np.sin(prev_theta)],
            [2.0 / self.wheel_base, -2.0 / self.wheel_base]
        ])

        Q_k = Nabla_w @ Sigma_delta @ Nabla_w.T

        self.sigma = H @ self.sigma @ H.T + Q_k

        # Update odom message
        self.odom.header.stamp = self.get_clock().now().to_msg()
        self.odom.header.frame_id = f'{self.namespace}/odom'
        self.odom.child_frame_id = f'{self.namespace}/base_footprint'
        self.odom.pose.pose.position.x = self.x
        self.odom.pose.pose.position.y = self.y
        self.odom.pose.pose.position.z = 0.0
        q = transforms3d.euler.euler2quat(0, 0, self.theta) 
        self.odom.pose.pose.orientation.x = q[1]
        self.odom.pose.pose.orientation.y = q[2]
        self.odom.pose.pose.orientation.z = q[3]
        self.odom.pose.pose.orientation.w = q[0]

        # Map 3x3 Sigma to the 36-element flat array
        # Order: x, y, z, roll, pitch, yaw (theta)
        cov = np.zeros(36)
        cov[0]  = self.sigma[0, 0] # x-x
        cov[1]  = self.sigma[0, 1] # x-y
        cov[5]  = self.sigma[0, 2] # x-yaw
        
        cov[6]  = self.sigma[1, 0] # y-x
        cov[7]  = self.sigma[1, 1] # y-y
        cov[11] = self.sigma[1, 2] # y-yaw
        
        cov[30] = self.sigma[2, 0] # yaw-x
        cov[31] = self.sigma[2, 1] # yaw-y
        cov[35] = self.sigma[2, 2] # yaw-yaw

        self.odom.pose.covariance = cov.tolist()

        # Publish
        self.odom_pub.publish(self.odom)

        self.prev_time_ns = current_time

def main():
    rclpy.init()

    node = Odom()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:

        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()