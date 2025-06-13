import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    turtlebot2_gazebo_package = FindPackageShare(
        package="turtlebot2_gazebo").find("turtlebot2_gazebo")
    # Alternative using get_package_share_directory for consistency:
    # turtlebot2_gazebo_pkg_share = get_package_share_directory('turtlebot2_gazebo')

    gazebo_ros_package = FindPackageShare(
        package="gazebo_ros").find("gazebo_ros")
    # Alternative using get_package_share_directory for consistency:
    # gazebo_ros_pkg_share = get_package_share_directory('gazebo_ros')

    # Launch Configurations
    namespace_lc = LaunchConfiguration('namespace')
    world_lc = LaunchConfiguration('world')
    use_sim_time_lc = LaunchConfiguration('use_sim_time')
    # use_namespace_lc = LaunchConfiguration('use_namespace') # Declared but not directly used to pass to children

    # ============================================================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_package, "launch", "gazebo.launch.py"),
        ),
        launch_arguments={'world': world_lc,
                          'verbose': 'true'}.items()
    )

    spawn_tb2_5 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot2_gazebo_package, "launch",
                         "turtlebot2_spawn_robot.launch.py")
        ),
        launch_arguments={'namespace': namespace_lc,
                          'use_sim_time': use_sim_time_lc}.items()
    )

    ld = LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                turtlebot2_gazebo_package, 'worlds', 'test2.world'),
            description='SDF world file'),

        DeclareLaunchArgument(
            'namespace',
            default_value='turtlebot',
            description='Top-level namespace for the robot'),

        DeclareLaunchArgument(
            'use_namespace',
            default_value='true',
            description='Whether to apply a namespace (primarily for Nav2 stack context)'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
    ])

    ld.add_action(gazebo)
    ld.add_action(spawn_tb2_5)

    return ld
