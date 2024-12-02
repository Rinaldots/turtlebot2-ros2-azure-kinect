import os
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory, get_package_share_path

import launch
import launch.launch_description_sources
import launch.substitutions
import launch_ros
import launch_ros.substitutions
import yaml



def generate_launch_description():
    
    rviz_decision = launch.actions.DeclareLaunchArgument('rviz', default_value='false', description='Open RViz.')
    
    kobuki_package = launch_ros.substitutions.FindPackageShare(package='kobuki_node').find('kobuki_node')
    turtlebot2_bringup_package = launch_ros.substitutions.FindPackageShare(package='turtlebot2_bringup').find('turtlebot2_bringup')
    turtlebot_description_package = launch_ros.substitutions.FindPackageShare(package='turtlebot2_description').find('turtlebot2_description')

    ekf_config_params = os.path.join(turtlebot2_bringup_package,'config/ekf_config.yaml')
    
    params_file = os.path.join(kobuki_package, 'config', 'kobuki_node_params.yaml')

    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['kobuki_ros_node']['ros__parameters']


    kobuki_node_launch = launch_ros.actions.Node(
        package='kobuki_node',
        executable='kobuki_ros_node',
        parameters=[params],
        namespace='turtlebot',
        remappings=[("commands/velocity", "cmd_vel",),
                    ("odom", "odom_turtle")],
    )

    ekf_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        output='screen',
        parameters=[ekf_config_params],
        remappings=[("odometry/filtered", "odom")],
        namespace='turtlebot'
    )
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': launch.substitutions.Command(['xacro ',os.path.join(turtlebot_description_package,'robots/kobuki_hexagons_kinect.urdf.xacro')])}],
        namespace='turtlebot'
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace='turtlebot'
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d',os.path.join(turtlebot2_bringup_package,'rviz/bringup.rviz')],
        namespace='turtlebot',
        condition=launch.conditions.IfCondition(LaunchConfiguration('rviz'))
    )


    return launch.LaunchDescription([
        rviz_decision,

        kobuki_node_launch,
        robot_state_publisher_node,
        joint_state_publisher_node,
        
        rviz_node,
        ekf_node
    ])
