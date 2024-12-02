from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.actions import OpaqueFunction
import os
import xacro

def robot_state_publisher_launch(context, *args, **kwargs):
    turtlebot2_description_package = FindPackageShare(package="turtlebot2_description").find("turtlebot2_description")
    launch_namespace = LaunchConfiguration('namespace').perform(context)
    urdf = (xacro.process_file(os.path.join(turtlebot2_description_package,
                                            "robots/kobuki_hexagons_kinect.urdf.xacro"),
                               mappings={'namespace': launch_namespace}))
    pretty_urdf = urdf.toprettyxml(indent='   ')

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=LaunchConfiguration('namespace'),
        remappings=[('/odom', 'odom')],
        parameters=[
            {"robot_description": pretty_urdf},
            {"use_sim_time": True},
        ],
    )

    return [robot_state_publisher_node]

def generate_launch_description():
    
    declare_namespace_cmd = DeclareLaunchArgument('namespace', default_value='turtlebot', description='Top-level namespace')
    declare_use_timer = DeclareLaunchArgument('use_sim_time', default_value='true', description='Whether to use Gazebo clock')
    declare_x_pose = DeclareLaunchArgument('x_pose', default_value='0', description='Initial x position of the robot')

    namespace = LaunchConfiguration('namespace')
    x_pose = LaunchConfiguration('x_pose')

    turtlebot2_description_package = get_package_share_directory("turtlebot2_description")
    kobuki_description_package = get_package_share_directory("kobuki_description")
    param_config = os.path.join(get_package_share_directory('turtlebot2_bringup'), 'config', 'param.yaml')

    param2= {'use_sim_time': LaunchConfiguration('use_sim_time') }

    install_dir1 = get_package_prefix("turtlebot2_description")
    install_dir2 = get_package_prefix("kobuki_description")

    gazebo_models_path1 = os.path.join(turtlebot2_description_package, "meshes")
    gazebo_models_path2 = os.path.join(kobuki_description_package, "meshes")

    if "GAZEBO_MODEL_PATH" in os.environ:os.environ["GAZEBO_MODEL_PATH"] = (os.environ["GAZEBO_MODEL_PATH"]+ ":"+ install_dir2+ "/share"+ ":"+ gazebo_models_path2)
    else:os.environ["GAZEBO_MODEL_PATH"] = (install_dir2 + "/share" + ":" + gazebo_models_path2)
    if "GAZEBO_MODEL_PATH" in os.environ:os.environ["GAZEBO_MODEL_PATH"] = (os.environ["GAZEBO_MODEL_PATH"]+ ":"+ install_dir1+ "/share"+ ":"+ gazebo_models_path1)
    else:os.environ["GAZEBO_MODEL_PATH"] = (install_dir1 + "/share" + ":" + gazebo_models_path1)
    if "GAZEBO_PLUGIN_PATH" in os.environ:os.environ["GAZEBO_PLUGIN_PATH"] = (os.environ["GAZEBO_PLUGIN_PATH"] + ":" + install_dir1 + "/lib")
    else:os.environ["GAZEBO_PLUGIN_PATH"] = install_dir1 + "/lib"
    
    tf_prefix = Node(package='tf2_ros', executable='static_transform_publisher', 
            output='screen',arguments=["0", "0", "0", "-1.57", "0", "-1.57", "camera_link", "kinect_rgb"],
            parameters=[param2],) 
    
    gazebo_node = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_entity",
            namespace=namespace,
            output="screen",
            remappings=[('/odom','odom')],
            arguments=["-entity", (namespace,"_robot"),"-topic", ("/", namespace, "/robot_description"), "-x", x_pose, "-y", "0",],
        )

    depthimage_to_laserscan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        namespace=namespace,
        remappings=[('depth', 'depth/image_raw'),
                    ('depth_camera_info', 'depth/camera_info'),
                    ('image', 'image_raw'),
                    ('scan', 'scan')],
        parameters=[param_config,param2],
    )


    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        parameters=[param2]
    )

    return LaunchDescription([
        
        declare_namespace_cmd,
        declare_use_timer,
        declare_x_pose,
        gazebo_node,
        tf_prefix,
        
        depthimage_to_laserscan_node,
        OpaqueFunction(function=robot_state_publisher_launch),
        joint_state_publisher_node,
        
    ])