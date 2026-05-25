import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    xacro_file_name = 'sp8.urdf.xacro'
    xacro_file = os.path.join(
        get_package_share_directory('sp8_robot'),
        'description',
        xacro_file_name)
    
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    namespace = LaunchConfiguration("namespace")
    gazebo = LaunchConfiguration("gazebo")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value=""
    )

    declare_gazebo_cmd = DeclareLaunchArgument(
    "gazebo",
    default_value="false",  # default: not simulation
    description="Launch in Gazebo simulation"
)

    robot_state_pub_node = Node(
                            package='robot_state_publisher',
                            executable='robot_state_publisher',
                            name='robot_state_publisher',
                            output='screen',
                            parameters=[{'robot_description': robot_description_raw,
                                         'use_sim_time': gazebo}],
                            namespace=namespace
                            )
    
    odom_node = Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{'robot_description': robot_description_raw,
                                    'use_sim_time': gazebo}],
                    namespace=namespace
                    )
    

    '''
    # Define joint_state_publisher node (for simulation)
    joint_state_publisher_node = Node(
                                package='joint_state_publisher_gui',
                                executable='joint_state_publisher_gui',
                                output='screen'
                            )
    '''
    l_d = LaunchDescription([declare_namespace_cmd, declare_gazebo_cmd, robot_state_pub_node])

    return l_d