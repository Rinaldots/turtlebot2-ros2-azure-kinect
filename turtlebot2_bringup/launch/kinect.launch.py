import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Arguments
    namespace = LaunchConfiguration('namespace')  
    use_rviz = LaunchConfiguration('rviz')
    pkg_bringup = get_package_share_directory('turtlebot2_bringup')
    default_rviz_config_path = os.path.join(pkg_bringup, "config/pointcloud.rviz")
    param_config = os.path.join(get_package_share_directory('turtlebot2_bringup'), 'config', 'param.yaml')
    

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='turtlebot',
        description='Top-level namespace',)
    use_rviz_argument = DeclareLaunchArgument(
        'rviz', default_value='false', 
        description='Open RViz.',)
    

    bringup_cmd_group = GroupAction([
        # Include kinect
        Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            output='screen',
            arguments=["0", "0", "0", "-1.57", "0", "-1.57", "camera_link", "kinect_rgb"]        ),
        Node(
            package="kinect_ros2",
            executable="kinect_ros2_node",
            name="kinect_ros2",
            namespace=namespace,),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', default_rviz_config_path],
            condition=IfCondition(use_rviz),
            namespace=namespace,),
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            remappings=[('depth', 'depth/image_raw'),
                        ('depth_camera_info', 'depth/camera_info'),
                        ('image', 'image_raw'),
                        ('scan', 'scan')],
            parameters=[param_config],
            )
        ])
    
    ld = LaunchDescription()
    
     # Declare the launch options
    ld.add_action(declare_namespace_cmd),
    ld.add_action(use_rviz_argument),
    
        
    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group),
    
    return ld
    