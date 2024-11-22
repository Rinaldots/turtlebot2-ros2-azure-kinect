# Requirements:
#   A kinect for xbox 360
#   Install kinect_ros2 package (use this fork: https://github.com/matlabbe/kinect_ros2)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    parameters=[{
          'frame_id':'base_link',
          'use_sin_time':True,
          'subscribe_depth':False,
          'subscribe_odom':False,
          'subscribe_rgb':False,
          'subscribe_scan':True,
          'use_action_for_goal':True,
          'qos_scan':1,
          'qos_imu':1,
          'approx_sync':True,
          'qos':1,
          'sync_queue_size': 10,
          'topic_queue_size': 1,
          }]
    
    namespace = LaunchConfiguration('namespace')
    
    remappings=[
          ('rgb/image', 'image_raw'),
          ('rgb/camera_info', 'camera_info'),
          ('depth/image', 'depth/image_raw'),
                ('depth/camera_info', 'depth/camera_info'),
                ('scan', 'scan'),
                
                    
          ]

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='turtlebot',
        description='Top-level namespace',)
    
    
    rtabmap = Node(
                package='rtabmap_slam', executable='rtabmap', output='screen',
                parameters=parameters,
                arguments=['-d'],
                namespace=namespace,
            )
    rtabmap_viz = Node(
                package='rtabmap_viz', executable='rtabmap_viz', output='screen',
                parameters=parameters,
                namespace=namespace,
            )

    return LaunchDescription([

        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=["0", "0", "0", "-1.57", "0", "-1.57", "camera_link", "kinect_rgb"],
            ),


        declare_namespace_cmd,
        rtabmap,
        rtabmap_viz,
    ])