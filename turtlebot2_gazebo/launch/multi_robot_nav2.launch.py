import os
from unicodedata import name

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString, RewrittenYaml

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('turtlebot2_gazebo'),
            'maps',
            'map_1661002108.yaml'))
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('turtlebot2_gazebo'),
            'config',
            'nav2_params.yaml'))

    namespace1 = LaunchConfiguration('namespace1')
    use_namespace = LaunchConfiguration('use_namespace')
    rviz_config_file = LaunchConfiguration('rviz_config')

    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')

    turtlebot2_gazebo_dir = get_package_share_directory('turtlebot2_gazebo')

    namespaced_params1= ReplaceString(
        source_file=param_dir, replacements={"/namespace":("/",namespace1)}
    )

    namespaced_rviz_config_file1 = ReplaceString(
        source_file=rviz_config_file, replacements={"/tb2": ("/", namespace1)})
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'namespace1',
            default_value='tb2_5',
            description='Top-level namespace'),
            
        DeclareLaunchArgument(
            'namespace2',
            default_value='tb2_6',
            description='top-level namespace for 2nd robot'),

        DeclareLaunchArgument(
            'use_namespace',
            default_value='true',
            description='Whether to apply a namespace to the navigation stack'),

        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                turtlebot2_gazebo_dir, 'rviz', 'namespaced_nav2.rviz'),
            description='Full path to the RVIZ config file to use'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([turtlebot2_world_launch]),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'use_namespace': use_namespace}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': namespaced_params}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            namespace=namespace,
            arguments=['-d', namespaced_rviz_config_file],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'),
                        ('/tf_static', 'tf_static'),
                        ('/goal_pose', 'goal_pose'),
                        ('/clicked_point', 'clicked_point'),
                        ('/initialpose', 'initialpose')]),
    ])