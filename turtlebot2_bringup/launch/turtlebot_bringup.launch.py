import os
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import launch
import launch.launch_description_sources
import launch.substitutions
import launch_ros
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_ros.substitutions
import yaml



def generate_launch_description():
    
    rviz_decision = launch.actions.DeclareLaunchArgument('rviz', default_value='false', description='Open RViz.')
    
    # Obter caminhos de diretório de compartilhamento de pacotes
    kobuki_node_share_dir = get_package_share_directory('kobuki_node')
    kobuki_auto_docking_share_dir = get_package_share_directory('kobuki_auto_docking')
    turtlebot2_bringup_share_dir = get_package_share_directory('turtlebot2_bringup')
    turtlebot_description_share_dir = get_package_share_directory('turtlebot2_description')

    ekf_config_params = os.path.join(turtlebot2_bringup_share_dir,'config/ekf_config.yaml')
    
    # Parâmetros do Kobuki Node
    params_file_kobuki = os.path.join(kobuki_node_share_dir, 'config', 'kobuki_node_params.yaml')
    with open(params_file_kobuki, 'r') as f:
        params_kobuki = yaml.safe_load(f)['kobuki_ros_node']['ros__parameters']

    # Parâmetros do Kobuki Auto Docking
    params_file_auto_docking = os.path.join(kobuki_auto_docking_share_dir, 'config', 'auto_docking.yaml')
    with open(params_file_auto_docking, 'r') as f:
        params_auto_docking = yaml.safe_load(f)['kobuki_auto_docking']['ros__parameters']

    # Definição do Robot Description para o Robot State Publisher
    robot_description_content = launch.substitutions.Command([
        'xacro ',
        os.path.join(turtlebot_description_share_dir,'robots/kobuki_hexagons_kinect.urdf.xacro')
    ])

    # Contêiner para os nós compostos do TurtleBot
    turtlebot_container = ComposableNodeContainer(
        name='turtlebot_container',
        namespace='turtlebot',
        package='rclcpp_components',
        executable='component_container', # ou 'component_container_mt' para multi-threaded
        composable_node_descriptions=[
            ComposableNode(
                package='kobuki_node',
                plugin='kobuki_node::KobukiRos',
                name='kobuki_ros_node',
                namespace='turtlebot',
                parameters=[params_kobuki],
                remappings=[
                    ("commands/velocity", "cmd_vel"),
                    ("odom", "odom_turtle")
                ],
            ),
            ComposableNode(
                package='kobuki_auto_docking',
                plugin='kobuki_auto_docking::AutoDockingROS',
                name='kobuki_auto_docking',
                namespace='turtlebot',
                parameters=[params_auto_docking],
                remappings=[
                    ("commands/velocity", "cmd_vel"),
                    ("odom", "odom_turtle")
                ],
            ),
            ComposableNode(
                package='robot_state_publisher',
                plugin='robot_state_publisher::RobotStatePublisher',
                name='robot_state_publisher',
                namespace='turtlebot',
                parameters=[{'robot_description': robot_description_content}],
            ),
            ComposableNode(
                package='joint_state_publisher',
                plugin='joint_state_publisher::JointStatePublisher',
                name='joint_state_publisher',
                namespace='turtlebot',
                parameters=[{'use_gui': False}], # Exemplo, ajuste se necessário
            ),
        ],
        output='screen',
    )

    # Nós que permanecem como processos separados
    ekf_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        output='screen',
        parameters=[ekf_config_params],
        remappings=[("odometry/filtered", "turtlebot/odom")],
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(turtlebot2_bringup_share_dir,'rviz/bringup.rviz')],
        namespace='turtlebot',
        condition=launch.conditions.IfCondition(LaunchConfiguration('rviz'))
    )

    return launch.LaunchDescription([
        rviz_decision,
        turtlebot_container,
        ekf_node,
        rviz_node,
    ])
