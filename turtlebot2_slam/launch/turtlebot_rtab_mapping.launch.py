from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    declare_use_timer = DeclareLaunchArgument('use_sim_time', default_value='true', description='Whether to use Gazebo clock')
    use_sim_time = LaunchConfiguration('use_sim_time')

    parameters=[{
          'frame_id':'base_link',
          'subscribe_rgbd':True,
          'subscribe_odom':True,
          'approx_sync':True,
          'qos':1,
          'sync_queue_size': 10,
          'approx_sync_max_interval': 0.01,
          'odom_frame_id':'odom',
          'odom_tf_linear_variance':0.001,
          'odom_tf_angular_variance':0.001,
          'use_action_for_goal':'true',
          'RGBD/ProximityBySpace':'true',
          'RGBD/OptimizeFromGraphEnd':'false',
          'Reg/Strategy':'0',
          "Icp/CorrespondenceRatio":'0.3',
          'Vis/MinInliers':'15',
          'Vis/InlierDistance':'0.1',
          'RGBD/ProximityPathMaxNeighbors':'0',
          'Rtabmap/TImeThr':'0.0',
          'Mem/RehearsalSimilarity':'0.3',
          'Reg/Force3DoF':'true',
          'GrigGlobal/MinSize':'20',
          'Grid/3D':'false', # Use 2D occupancy
          'Grid/NormalsSegmentation':'false', # Use passthrough filter to detect obstacles
          'Grid/MaxGroundHeight':'0.05', # All points above 5 cm are obstacles
          'Grid/MaxObstacleHeight':'1',  # All points over 1 meter are ignored
          'Rtabmap/DatabasePath':'/home/rinaldo/Documents/devel/src/turtlebot2-ros2-azure-kinect/turtlebot2_slam/maps/rtabmap.db',
          }]
    
    remappings=[
          ('rgb/image', 'image_raw'),
          ('rgb/camera_info', 'depth/camera_info'),
          ('depth/image', 'depth/image_raw'),          
          ]

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='turtlebot',
        description='Top-level namespace',)
    tf = Node(package='tf2_ros', executable='static_transform_publisher', arguments=["0", "0", "0", "-1.57", "0", "-1.57", 'camera_rgb_frame', 'kinect_rgb'], output='screen')
    tf2 = Node(package='tf2_ros', executable='static_transform_publisher', arguments=["0", "0", "0", "-1.57", "0", "-1.57", 'camera_depth_frame', 'kinect_depth'], output='screen')
    
    rtabmap_sync = Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=parameters,
            remappings=remappings,
            namespace=namespace,)

    rtabmap_odom = Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            namespace=namespace,
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
        tf,
        tf2,
        declare_namespace_cmd,
        declare_use_timer,
        rtabmap_sync,
        rtabmap_odom,
        rtabmap_slam,
        rtabmap_viz,
    ])