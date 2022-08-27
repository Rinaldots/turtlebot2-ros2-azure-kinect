import argparse
import os
from textwrap import indent

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression, Command, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
from launch.launch_context import LaunchContext
from launch.actions import OpaqueFunction


def robot_state_publisher_launch(context, *args, **kwargs):
    turtlebot2_description_package = FindPackageShare(
        package="turtlebot2_description").find("turtlebot2_description")
    launch_namespace = LaunchConfiguration('namespace').perform(context)
    urdf = (xacro.process_file(os.path.join(turtlebot2_description_package,
                                            "robots/kobuki_hexagons_hokuyo.urdf.xacro"),
                               mappings={'namespace': launch_namespace}))
    pretty_urdf = urdf.toprettyxml(indent='   ')

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=LaunchConfiguration('namespace'),
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static')],
        parameters=[
            {"robot_description": pretty_urdf},
            {"use_sim_time": LaunchConfiguration('use_sim_time')},
        ],
    )

    return [robot_state_publisher_node]


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    turtlebot2_gazebo_package = FindPackageShare(
        package="turtlebot2_gazebo").find("turtlebot2_gazebo")

    turtlebot2_description_package = FindPackageShare(
        package="turtlebot2_description").find("turtlebot2_description")

    gazebo_ros_package = FindPackageShare(
        package="gazebo_ros").find("gazebo_ros")

    kobuki_description_package = FindPackageShare(
        package="kobuki_description").find("kobuki_description")

    install_dir1 = get_package_prefix("turtlebot2_description")
    install_dir2 = get_package_prefix("kobuki_description")
    gazebo_models_path1 = os.path.join(
        turtlebot2_description_package, "meshes")
    gazebo_models_path2 = os.path.join(kobuki_description_package, "meshes")

    if "GAZEBO_MODEL_PATH" in os.environ:
        os.environ["GAZEBO_MODEL_PATH"] = (
            os.environ["GAZEBO_MODEL_PATH"]
            + ":"
            + install_dir2
            + "/share"
            + ":"
            + gazebo_models_path2
        )
    else:
        os.environ["GAZEBO_MODEL_PATH"] = (
            install_dir2 + "/share" + ":" + gazebo_models_path2
        )

    if "GAZEBO_MODEL_PATH" in os.environ:
        os.environ["GAZEBO_MODEL_PATH"] = (
            os.environ["GAZEBO_MODEL_PATH"]
            + ":"
            + install_dir1
            + "/share"
            + ":"
            + gazebo_models_path1
        )
    else:
        os.environ["GAZEBO_MODEL_PATH"] = (
            install_dir1 + "/share" + ":" + gazebo_models_path1
        )

    if "GAZEBO_PLUGIN_PATH" in os.environ:
        os.environ["GAZEBO_PLUGIN_PATH"] = (
            os.environ["GAZEBO_PLUGIN_PATH"] + ":" + install_dir1 + "/lib"
        )
    else:
        os.environ["GAZEBO_PLUGIN_PATH"] = install_dir1 + "/lib"

    print("GAZEBO MODELS PATH==" + str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH==" + str(os.environ["GAZEBO_PLUGIN_PATH"]))

    # ============================================================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_package, "launch", "gazebo.launch.py"),
        )
    )

    spawn_tb2 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_entity",
        namespace=namespace,
        output="screen",
        arguments=[
            "-entity",
            "turtlebot2",
            "-topic",
            ("/", namespace, "/robot_description"),
            "-x",
            "0",
            "-y",
            "0",
        ],
    )

    ld = LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(
                turtlebot2_gazebo_package, 'worlds', 'test2.world'), ''],
            description='SDF world file'),

        DeclareLaunchArgument(
            'namespace',
            default_value='tb2_5',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_namespace',
            default_value='true',
            description='Whether to apply a namespace to the navigation stack'),

        OpaqueFunction(function=robot_state_publisher_launch)
    ])

    ld.add_action(gazebo)
    ld.add_action(spawn_tb2)

    return ld
