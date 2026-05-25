from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import TimerAction

from launch.actions import  EmitEvent, LogInfo, RegisterEventHandler, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, LocalSubstitution, LaunchConfiguration
#import xacro

def generate_launch_description():

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the micro-ROS agent'
    )

    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for the rplidar a1'
    )

    serial_port = LaunchConfiguration('serial_port')
    lidar_port = LaunchConfiguration('lidar_port')
    '''
    xacro_file_name = 'sp8.urdf.xacro'
    xacro_file = os.path.join(
        get_package_share_directory('sp8_robot'),
        'description',
        xacro_file_name)
    
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    '''
    urdf_file_name = 'sp8.urdf'
    urdf_file = os.path.join(
        get_package_share_directory('sp8_robot'),
        'description',
        urdf_file_name)
    
    # Read the file natively using standard Python
    with open(urdf_file, 'r') as infp:
        robot_description_raw = infp.read()

    motor_controller_node = Node(
                                package='serial_motor_driver',
                                executable='serial_driver_node',
                                name='serial_motor_driver',
                                parameters=[{
                                    'port': serial_port,
                                    'baudrate': 115200,
                                }],
                                output='screen'
                            )
    
    ydlidar_launch_path = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'),
        'launch',
        'ydlidar_launch.py'
    )

    ydlidar_params_file = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'),
        'params',
        'X4jetson.yaml'
    )

    ydlidar_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ydlidar_launch_path),
        launch_arguments={
            'port': lidar_port,
            'params_file': ydlidar_params_file
        }.items()
    )
    
    odom_node = Node(name='odom',
                    package='serial_motor_driver',
                    executable='odom',
                    )
    
    joint_publisher_node = Node(name='joint_publisher',
                                package='serial_motor_driver',
                                executable='joint_state_publisher'
                                )
                        
    robot_state_pub_node = Node(
                            package='robot_state_publisher',
                            executable='robot_state_publisher',
                            name='robot_state_publisher',
                            output='screen',
                            parameters=[{'robot_description': robot_description_raw}],
                            )

    l_d = LaunchDescription([serial_port_arg, lidar_port_arg, motor_controller_node, odom_node, joint_publisher_node, robot_state_pub_node, ydlidar_driver_launch])

    return l_d