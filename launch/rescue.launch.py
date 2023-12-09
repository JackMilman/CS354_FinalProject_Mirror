"""Launch the Zeta rescue system 

You can make any changes you want to this launch file, but it must
accept the time_limit and use_sim_time command line arguments.

"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    this_path = get_package_share_directory('zeta_competition')
    default_map_path = os.path.join(this_path, 'maps', 'room_practice.yaml')
    default_pose = os.path.join(this_path, 'config', 'sim_initial_pose.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map_path),
        DeclareLaunchArgument('initial_pose', default_value=default_pose),
        DeclareLaunchArgument('time_limit', default_value='360'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        
        Node(
            package="zeta_rescue",
            executable="rescue_node",
            output="screen",
            parameters=[{'map': LaunchConfiguration('map'),
                        #  'initial_pose': LaunchConfiguration('initial_pose'),
                         'time_limit':  LaunchConfiguration('time_limit'),
                         'use_sim_time': LaunchConfiguration('use_sim_time'),}]
        ),

        # Start initial pose setter
        Node(
            package="zeta_competition",
            executable="set_initial_pose",
            output="screen",
            parameters=[LaunchConfiguration('initial_pose')]
        )
    ])

if __name__ == "__main__":
    generate_launch_description()
