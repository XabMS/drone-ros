"""Hito S2: config_manager + mission_manager.

    ros2 launch drone_mission s2.launch.py city:=toulouse mission_id:=tls_demo_01
    ros2 service call /mission_manager/command drone_interfaces/srv/MissionCommand "{command: 1}"   # START
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params = os.path.join(get_package_share_directory('drone_mission'), 'config', 'mission_manager.yaml')
    city = LaunchConfiguration('city')
    mission_id = LaunchConfiguration('mission_id')
    ops_dir = LaunchConfiguration('ops_dir')
    missions_dir = LaunchConfiguration('missions_dir')
    return LaunchDescription([
        DeclareLaunchArgument('city', default_value='toulouse'),
        DeclareLaunchArgument('mission_id', default_value='tls_demo_01'),
        DeclareLaunchArgument('ops_dir', default_value=os.environ.get('OPS_DIR', '/ops')),
        DeclareLaunchArgument('missions_dir', default_value=os.environ.get('MISSIONS_DIR', '/missions')),
        Node(
            package='drone_mission', executable='config_manager', name='config_manager', output='screen',
            parameters=[{'ops_dir': ops_dir, 'city': city}],
        ),
        Node(
            package='drone_mission', executable='mission_manager', name='mission_manager', output='screen',
            parameters=[params, {'ops_dir': ops_dir, 'missions_dir': missions_dir, 'mission_id': mission_id,
                                 'simulated_drop': True}],   # S2: sin payload_manager
        ),
    ])
