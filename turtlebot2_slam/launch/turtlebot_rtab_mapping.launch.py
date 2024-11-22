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
          'subscribe_rgbd':True,
          'subscribe_odom':True,
          'odom_frame_id':'odom',
          'odom_tf_linear_variance':0.001,
          'odom_tf_angular_variance':0.001,
          'approx_sync':True,
          'qos':1,
          'sync_queue_size': 10,
          
          'approx_sync_max_interval': 0.01,
          'use_action_for_goal':True,
          'Reg/Force3DoF':'true',
          'Grid/RayTracing':'true', # Fill empty space
          'Grid/3D':'false', # Use 2D occupancy
          'Grid/NormalsSegmentation':'false', # Use passthrough filter to detect obstacles
          'Grid/MaxGroundHeight':'0.05', # All points above 5 cm are obstacles
          'Grid/MaxObstacleHeight':'1',  # All points over 1 meter are ignored

          
          }]
    
    namespace = LaunchConfiguration('namespace')
    
    remappings=[
          ('rgb/image', 'image_raw'),
          ('rgb/camera_info', 'depth/camera_info'),
          ('depth/image', 'depth/image_raw'),
          ('rgbd_image', 'rgbd_image'),          
          ]

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='turtlebot',
        description='Top-level namespace',)
    rotate_node = Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=["0", "0", "0", "0", "0", "0", "camera_link", "kinect_rgb"])
    
    rtabmap_sync = Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=parameters,
            remappings=remappings,
            namespace=namespace,)


    rtabmap_odom = Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            namespace=namespace,
            remappings=remappings,
            )
    rtabmap_slam = Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            arguments=['-d'],
            namespace=namespace,
            remappings=remappings,
            )
    rtabmap_viz = Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            namespace=namespace,
            remappings=remappings,
            )

    return LaunchDescription([

        declare_namespace_cmd,
        rotate_node,
        rtabmap_sync,
        #rtabmap_sync,
        rtabmap_odom,
        rtabmap_slam,
        rtabmap_viz,
    ])