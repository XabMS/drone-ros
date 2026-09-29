"""Lanza el nodo de demostración S1 con parámetros configurables.

    ros2 launch drone_s1_demo s1_demo.launch.py altitude_m:=10.0 hover_s:=15.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    altitude = LaunchConfiguration('altitude_m')
    hover = LaunchConfiguration('hover_s')
    topic_status = LaunchConfiguration('topic_vehicle_status')
    topic_position = LaunchConfiguration('topic_local_position')
    return LaunchDescription([
        DeclareLaunchArgument('altitude_m', default_value='10.0'),
        DeclareLaunchArgument('hover_s', default_value='15.0'),
        DeclareLaunchArgument('topic_vehicle_status', default_value='/fmu/out/vehicle_status_v1'),
        DeclareLaunchArgument('topic_local_position', default_value='/fmu/out/vehicle_local_position_v1'),
        Node(
            package='drone_s1_demo',
            executable='takeoff_hover_land',
            name='takeoff_hover_land',
            output='screen',
            parameters=[{
                'altitude_m': altitude,
                'hover_s': hover,
                'topic_vehicle_status': topic_status,
                'topic_local_position': topic_position,
            }],
        ),
    ])
