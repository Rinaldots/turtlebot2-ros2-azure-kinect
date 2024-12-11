from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    declare_use_timer = DeclareLaunchArgument('use_sim_time', default_value='true', description='Whether to use Gazebo clock')
    use_sim_time = LaunchConfiguration('use_sim_time')

    parameters = [{
        'frame_id':'base_link',
        'subscribe_rgbd':True,
        'subscribe_odom':True,
        'approx_sync':True,
        'use_sim_time': use_sim_time,
        'qos':1,
        'sync_queue_size': 10,
        'approx_sync_max_interval': 0.01,
        'publish_tf_map': 'true',
        'imu_topic':'sensor/imu_data',
        'odom_frame_id':'odom',
        'odom_tf_linear_variance':0.001,
        'odom_tf_angular_variance':0.001,
        

        'use_action_for_goal':True,
        'max_update_rate': '30',

        'RGBD/ProximityBySpace':'true',
        'RGBD/OptimizeFromGraphEnd':'false',
        'RGBD/ProximityPathMaxNeighbors':'0',
        ''

        'Reg/Strategy':'0',
        'Reg/Force3DoF':'true',

        "Icp/CorrespondenceRatio":'0.3',

        'Vis/MinInliers':'15',
        'Vis/InlierDistance':'0.1',
        
        'Rtabmap/TImeThr':'0.0',

        'Mem/RehearsalSimilarity':'0.3',
        'GrigGlobal/MinSize':'20',
        
        'Grid/3D':'false', # Use 2D occupancy
        'Grid/NormalsSegmentation':'false', # Use passthrough filter to detect obstacles
        'Grid/MaxGroundHeight':'0.05', # All points above 5 cm are obstacles
        'Grid/MaxObstacleHeight':'0.6',  # All points over 1 meter are ignored
    }]

    remappings = [
        ('rgb/image', 'image_raw'),
        ('rgb/camera_info', 'depth/camera_info'),
        ('depth/image', 'depth/image_raw'),
        ('odom', 'odom'),
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

    rtabmap_odom = Node(
        package='rtabmap_odom', executable='rgbd_odometry', output='screen',
        arguments=['-d'],
        parameters=parameters,
        namespace=namespace)

    rtabmap_slam = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=parameters,
        arguments=['-d'],
        namespace=namespace,
        remappings=remappings)

    rtabmap_viz = Node(
        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        parameters=parameters,
        namespace=namespace,
        remappings=remappings)

    rtabmap_util = Node(
        package='rtabmap_util', executable='point_cloud_xyz', output='screen',
        parameters=[{'decimation': 2,
                     'max_depth': 3.0,
                     'voxel_size': 0.02}],
        remappings=[('depth/image', 'depth/image_raw'),
                    ('cloud', 'depth/cloud')])

    rtabmap_util2 = Node(
        package='rtabmap_util', executable='obstacles_detection', output='screen',
        parameters=parameters,
        remappings=[('cloud', 'depth/cloud'),
                    ('obstacles', 'depth/obstacles'),
                    ('ground', 'depth/ground')])

    return LaunchDescription([
        tf,
        tf2,
        declare_namespace_cmd,
        declare_use_timer,
        rtabmap_sync,
        rtabmap_odom,
        rtabmap_slam,
        # rtabmap_viz,
        rtabmap_util,
        rtabmap_util2,
    ])