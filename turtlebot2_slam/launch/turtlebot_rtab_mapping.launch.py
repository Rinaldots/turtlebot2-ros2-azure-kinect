from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    declare_use_timer = DeclareLaunchArgument('use_sim_time', default_value='true', description='Whether to use Gazebo clock')
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='turtlebot', description='Top-level namespace')

    parameters = [{
        'frame_id':'base_footprint',
        'subscribe_rgbd':True,
        'subscribe_odom':True,
        #'subscribe_sensor_data':True,

        'approx_sync':True,
        'qos':1,
        'sync_queue_size': 10,
        'approx_sync_max_interval': 0.01,
        'publish_tf_map': True,
        'imu_topic':'sensor/imu_data',
        'odom_frame_id':'odom',
        'odom_tf_linear_variance':0.001,
        'odom_tf_angular_variance':0.001,
        'max_update_rate': 5.0,
        'min_update_rate': 3.0,

        'RGBD/ProximityBySpace':'true',
        'RGBD/OptimizeFromGraphEnd':'false',
        'RGBD/ProximityPathMaxNeighbors':'0',
        
        'Reg/Strategy':'0',
        'Reg/Force3DoF':'true',

        "Icp/CorrespondenceRatio":'0.3',

        'Vis/MinInliers':'15',
        'Vis/InlierDistance':'0.1',
        
        'Rtabmap/TImeThr':'0.0',

        'Mem/RehearsalSimilarity':'0.3',
        'map_always_update':True,

        'GrigGlobal/MinSize':'20',
        'Grid/RayTracing':'true',
        'Grid/3D':'false', # Use 2D occupancy
        'Grid/NormalsSegmentation':'false', # Use passthrough filter to detect obstacles
        'Grid/MaxGroundHeight':'0.05', # All points above 5 cm are obstacles
        'Grid/MaxObstacleHeight':'0.6',  # All points over 1 meter are ignored
    }]

    remappings = [
        ('odom_info', 'odom_filtered'),
        ('odom', 'odom_rtab'),
        ('map', '/map'),
        ('imu', 'sensors/imu_data'),
        ('rgb/image', 'image_raw'),
        ('rgb/camera_info', 'depth/camera_info'),
        ('depth/image', 'depth/image_raw'),
    ]
    rtabmap_odom = Node(
        package='rtabmap_odom', executable='rgbd_odometry', output='screen',
        arguments=['-d'],
        parameters=parameters,
        remappings=remappings,
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

    return LaunchDescription([
        declare_namespace_cmd,
        declare_use_timer,
        rtabmap_odom,
        rtabmap_slam,
        rtabmap_viz,

    ])