from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    declare_use_timer = DeclareLaunchArgument('use_sim_time', default_value='true', description='Whether to use Gazebo clock')
    use_sim_time = LaunchConfiguration('use_sim_time')

    parameters = [{
        'frame_id':'base_footprint'
    }]

    remappings = [
        ('rgb/image', 'image_raw'),
        ('rgb/camera_info', 'depth/camera_info'),
        ('depth/image', 'depth/image_raw'),
    ]

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='turtlebot', description='Top-level namespace')

    tf = Node(package='tf2_ros', executable='static_transform_publisher',
              arguments=["0", "0", "0", "-1.57", "0", "-1.57", 'camera_rgb_frame', 'kinect_rgb'], output='screen')
    tf2 = Node(package='tf2_ros', executable='static_transform_publisher',
               arguments=["0", "0", "0", "-1.57", "0", "-1.57", 'camera_depth_frame', 'kinect_depth'], output='screen')
    

    rtabmap_sync = Node(
        package='rtabmap_sync', executable='rgbd_sync', output='screen',
        parameters=parameters,
        remappings=remappings,
        namespace=namespace)

    

    return LaunchDescription([
        tf,
        tf2,
        declare_namespace_cmd,
        declare_use_timer,
        rtabmap_sync,

    ])