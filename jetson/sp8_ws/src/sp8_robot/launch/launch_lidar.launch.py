import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():

    ydlidar_param_file =  os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'params', 'X4jetson.yaml')

    ydlidar_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ydlidar_ros2_driver'),'launch','ydlidar_launch.py'
                )]), launch_arguments={'params_file': ydlidar_param_file}.items()
    )

    return LaunchDescription([
        ydlidar_launch
    ])