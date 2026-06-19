import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('imu_mag_fusion'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='imu_mag_fusion',
            executable='imu_mag_fusion.py',
            name='imu_mag_fusion',
            output='screen',
            parameters=[config]
        )
    ])