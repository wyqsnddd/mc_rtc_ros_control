# usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def launch_setup(context, *args, **kwargs):
    robot = context.launch_configurations['robot']
    frame_id = context.launch_configurations['frame_id']

    pkg_share = get_package_share_directory('mc_rtc_ros_control')
    rviz_config = os.path.join(pkg_share, 'rviz', 'surface_visualization.rviz')

    surface_node = Node(
        package='mc_rtc_ros_control',
        executable='mc_surface_visualization',
        name='mc_surface_visualization',
        output='screen',
        parameters=[
            {
                'robot': robot,
                'frame_id': frame_id,
            }
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return [surface_node, rviz_node]


def generate_launch_description():

    robot = DeclareLaunchArgument(
        'robot',
        default_value='JVRC1',
        description='Robot module name'
    )

    frame_id = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='TF frame for markers'
    )

    declare_arguments = [robot, frame_id]

    return LaunchDescription(declare_arguments + [
        OpaqueFunction(function=launch_setup),
    ])
