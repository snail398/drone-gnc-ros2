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
        DeclareLaunchArgument(
            'controller', default_value='px4',
            description="Position controller: 'px4' (PX4's own) or 'own' (gnc_core)"),
        Node(
            package='gnc_ros',
            executable='mission_node',
            # Later entries override earlier ones: the command-line choice wins over the file.
            parameters=[LaunchConfiguration('mission'),
                        {'controller': LaunchConfiguration('controller')}],
            output='screen',
            emulate_tty=True),
    ])
