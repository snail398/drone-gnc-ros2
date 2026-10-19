from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_mission = Path(get_package_share_directory('gnc_ros')) / 'config' / 'square_mission.yaml'

    return LaunchDescription([
        DeclareLaunchArgument(
            'mission', default_value=str(default_mission),
            description='YAML file with mission_node parameters'),
        Node(
            package='gnc_ros',
            executable='mission_node',
            parameters=[LaunchConfiguration('mission')],
            output='screen',
            emulate_tty=True),
    ])
