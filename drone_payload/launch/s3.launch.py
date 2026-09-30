"""Hito S3: config_manager + mission_manager + payload_manager.

    ros2 launch drone_payload s3.launch.py city:=toulouse mission_id:=tls_demo_01
    ros2 service call /payload_manager/confirm drone_interfaces/srv/ConfirmDrop "{drop_zone_id: tls_dz_01}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    mission_params = os.path.join(get_package_share_directory('drone_mission'), 'config', 'mission_manager.yaml')
    payload_params = os.path.join(get_package_share_directory('drone_payload'), 'config', 'payload_manager.yaml')
    city = LaunchConfiguration('city')
    mission_id = LaunchConfiguration('mission_id')
    ops_dir = LaunchConfiguration('ops_dir')
    missions_dir = LaunchConfiguration('missions_dir')
    mavlink_url = LaunchConfiguration('mavlink_url')
    return LaunchDescription([
        # Enlace del companion por el que llega la confirmación del piloto (por defecto, el enlace onboard de PX4 SITL)
        DeclareLaunchArgument('mavlink_url', default_value='udpin:0.0.0.0:14540'),
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
            parameters=[mission_params, {'ops_dir': ops_dir, 'missions_dir': missions_dir, 'mission_id': mission_id}],
        ),
        Node(
            package='drone_payload', executable='payload_manager', name='payload_manager', output='screen',
            parameters=[payload_params, {'ops_dir': ops_dir, 'city': city}],
        ),
        Node(
            package='drone_gcs_bridge', executable='pilot_confirm_bridge', name='pilot_confirm_bridge',
            output='screen', parameters=[{'mavlink_url': mavlink_url}],
        ),
    ])
