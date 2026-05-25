import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from rclpy.qos import qos_profile_sensor_data
import transforms3d
import numpy as np

class JointStatePub(Node):
    def __init__(self):
        super().__init__('Joint_state_publisher')

        self.namespace = self.get_namespace().strip('/')

        # Puzzlebot Initial Pose
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.q_x = 0.0
        self.q_y = 0.0
        self.q_z = 0.0
        self.q_w = 1.0

        # Wheel ang vel and position
        self.wr = 0.0
        self.wl = 0.0
        self.theta_l = 0.0  # left wheel angle
        self.theta_r = 0.0  # right wheel angle
        self.prev_time = self.get_clock().now().nanoseconds

        # Define Transform Broadcasters
        self.tf_br_base_footprint = TransformBroadcaster(self)

        # Joint publisher
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Odom subscriber
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.wl_sub = self.create_subscription(Float32, 'wl', self.wl_cb, qos_profile=qos_profile_sensor_data)
        self.wr_sub = self.create_subscription(Float32, 'wr', self.wr_cb, qos_profile=qos_profile_sensor_data)

        # Initialize message to be published
        self.ctrlJoints = JointState()
        self.ctrlJoints.header.stamp = self.get_clock().now().to_msg()
        self.ctrlJoints.name = ['left_wheel_joint', 'right_wheel_joint'] 
        self.ctrlJoints.position = [0.0] * 2
        self.ctrlJoints.velocity = [0.0] * 2
        self.ctrlJoints.effort = [0.0] * 2     
        #Define Transformations
        self.define_TF()

        #Create a Timer
        timer_period = 0.02 #seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)

    def wl_cb(self, msg):
        self.wl = msg.data

    def wr_cb(self, msg):
        self.wr = msg.data

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        self.q_x = msg.pose.pose.orientation.x
        self.q_y = msg.pose.pose.orientation.y
        self.q_z = msg.pose.pose.orientation.z
        self.q_w = msg.pose.pose.orientation.w

    
    def timer_cb(self):
        current_time = self.get_clock().now().nanoseconds
        dt = (current_time - self.prev_time) * 1e-9

        # Calculate wheel angles
        self.theta_l += self.wl * dt
        self.theta_r += self.wr * dt
        self.theta_l = np.arctan2(np.sin(self.theta_l), np.cos(self.theta_l))
        self.theta_r = np.arctan2(np.sin(self.theta_r), np.cos(self.theta_r))

        self.base_footprint_link_tf.header.stamp = self.get_clock().now().to_msg()
        self.base_footprint_link_tf.transform.translation.x = self.x
        self.base_footprint_link_tf.transform.translation.y = self.y
        self.base_footprint_link_tf.transform.translation.z = self.z   
        self.base_footprint_link_tf.transform.rotation.x = self.q_x
        self.base_footprint_link_tf.transform.rotation.y = self.q_y
        self.base_footprint_link_tf.transform.rotation.z = self.q_z
        self.base_footprint_link_tf.transform.rotation.w = self.q_w

        self.ctrlJoints.header.stamp = self.get_clock().now().to_msg()
        self.ctrlJoints.position[0] = self.theta_l
        self.ctrlJoints.position[1] = self.theta_r
        self.ctrlJoints.velocity[0] = self.wl
        self.ctrlJoints.velocity[1] = self.wr

        # Broadcast transforms
        self.tf_br_base_footprint.sendTransform(self.base_footprint_link_tf)

        # Public dynamic joints
        self.joint_pub.publish(self.ctrlJoints)

        self.prev_time = current_time

    def define_TF(self):

        # Base_footprint_Link
        self.base_footprint_link_tf = TransformStamped()
        self.base_footprint_link_tf.header.stamp = self.get_clock().now().to_msg()
        self.base_footprint_link_tf.header.frame_id = f'{self.namespace}/odom'
        self.base_footprint_link_tf.child_frame_id = f'{self.namespace}/base_footprint'
        self.base_footprint_link_tf.transform.translation.x = 0.0
        self.base_footprint_link_tf.transform.translation.y = 0.0
        self.base_footprint_link_tf.transform.translation.z = 0.0
        q = transforms3d.euler.euler2quat(0,0,0)       
        self.base_footprint_link_tf.transform.rotation.x = q[1]
        self.base_footprint_link_tf.transform.rotation.y = q[2]
        self.base_footprint_link_tf.transform.rotation.z = q[3]
        self.base_footprint_link_tf.transform.rotation.w = q[0]

        self.tf_br_base_footprint.sendTransform(self.base_footprint_link_tf)





def main():
    rclpy.init()

    node = JointStatePub()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()