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
    pkg_kinect = get_package_share_directory('kinect_ros2')
    default_rviz_config_path = os.path.join(pkg_kinect, "rviz/pointcloud.rviz")
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Top-level namespace',)
    
    use_rviz_argument = DeclareLaunchArgument(
        'rviz', default_value='false', 
        description='Open RViz.',)

    bringup_cmd_group = GroupAction([
        # Include kinect
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
        ])
    
    ld = LaunchDescription()
    
     # Declare the launch options
    ld.add_action(declare_namespace_cmd),
    ld.add_action(use_rviz_argument),
        
    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group),
    
    return ld
    