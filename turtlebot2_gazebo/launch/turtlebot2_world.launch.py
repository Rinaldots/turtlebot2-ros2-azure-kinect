import argparse
import os
from textwrap import indent

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    turtlebot2_gazebo_package = FindPackageShare(
        package="turtlebot2_gazebo").find("turtlebot2_gazebo")

    gazebo_ros_package = FindPackageShare(
        package="gazebo_ros").find("gazebo_ros")

    
    # ============================================================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_package, "launch", "gazebo.launch.py"),
        )
    )

    spawn_tb2_5 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot2_gazebo_package, "launch",
                         "turtlebot2_spawn_robot.launch.py")
        ),
        launch_arguments={'namespace': namespace}.items()
    )

    ld = LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(
                turtlebot2_gazebo_package, 'worlds', 'virtual07.world'), ''],
            description='SDF world file'),

        DeclareLaunchArgument(
            'namespace',
            default_value='turtlebot',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_namespace',
            default_value='true',
            description='Whether to apply a namespace to the navigation stack'),
        

    ])

    ld.add_action(gazebo)
    ld.add_action(spawn_tb2_5)

    return ld
