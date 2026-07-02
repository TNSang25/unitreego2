import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """Generate launch description for Go2 pure navigation (Nav2 & AMCL only)"""
    
    # Package paths
    nav_package_dir = get_package_share_directory('go2_navigation')
    
    # Default map path in the new package
    default_map_path = os.path.join(nav_package_dir, 'maps', 'chieu2306.yaml')
    map_file = os.getenv('MAP_FILE', default_map_path)

    config_paths = {
        'nav2': os.path.join(nav_package_dir, 'config', 'nav2_params.yaml'),
        'rviz': os.path.join(nav_package_dir, 'rviz', 'nav2.rviz'),
    }
    
    print(f"🧭 Go2 Pure Navigation Bringup")
    print(f"   Map File: {map_file}")
    print(f"   Note: Hardware drivers (WebRTC, LiDAR, state publisher) must be running separately.")
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_arg = LaunchConfiguration('map')
    with_rviz = LaunchConfiguration('rviz', default='true')
    
    urdf_file = os.path.join(get_package_share_directory('go2_robot_sdk'), 'urdf', 'go2.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
        
    robot_ip = LaunchConfiguration('robot_ip', default=os.getenv('ROBOT_IP', ''))
    robot_token = LaunchConfiguration('robot_token', default=os.getenv('ROBOT_TOKEN', 'af4195d67dd4d585f161f7e0932c2aa8'))
    conn_type = LaunchConfiguration('conn_type', default=os.getenv('CONN_TYPE', 'webrtc'))
    
    launch_args = [
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map yaml file for navigation'
        ),
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz2'),
    ]
    
    core_nodes = [
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='go2_robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }],
        ),
        # Main robot driver
        Node(
            package='go2_robot_sdk',
            executable='go2_driver_node',
            name='go2_driver_node',
            output='screen',
            remappings=[
                ('cmd_vel_out', 'cmd_vel'),
            ],
            parameters=[{
                'robot_ip': robot_ip,
                'token': robot_token,
                'conn_type': conn_type,
                'enable_video': True,
                'lidar_publish_rate': 15.0,   # chặn firehose LiDAR
                'lidar_voxel_size': 0.06,     # khớp pipeline lúc quét map
            }],
        ),
        # LƯU Ý: lidar_to_pointcloud chỉ cần ở MULTI mode (thêm bên dưới). Single mode
        # driver đã publish /point_cloud2 -> thêm node này sẽ tự-lặp gây firehose.
        # Point cloud aggregator - KHỚP mapping.launch.py để scan trùng với map đã lưu
        Node(
            package='lidar_processor_cpp',
            executable='pointcloud_aggregator_node',
            name='pointcloud_aggregator',
            remappings=[
                ('cloud_in', '/point_cloud2'),
            ],
            parameters=[{
                'max_range': 10.0,
                'min_range': 0.15,
                'height_filter_min': 0.2,   # cùng lát cao 0.2-0.4m như lúc quét map -> AMCL khớp
                'height_filter_max': 0.4,
                'downsample_rate': 1,
                'publish_rate': 20.0,
                'voxel_leaf_size': 0.05,
                'sor_enable': True,
                'sor_mean_k': 12,
                'sor_std_dev': 1.0,
                'ror_enable': True,
                'ror_radius': 0.15,
                'ror_min_neighbors': 3,
            }],
        ),
        # PointCloud to LaserScan - KHỚP mapping (để AMCL khớp map)
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='go2_pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/pointcloud/filtered'),
                ('scan', '/scan'),
            ],
            parameters=[{
                'target_frame': 'base_link',
                'max_height': 2.0,    # rộng: lọc cao đã làm ở aggregator (lát 0.2-0.4m)
                'min_height': -1.0,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.00872665,
                'scan_time': 0.1,
                'range_min': 0.35,    # loại chân/thân robot
                'range_max': 10.0,
                'use_inf': True,
                'concurrency_level': 2,
            }],
            output='screen',
        ),
    ]

    # Visualization nodes
    viz_nodes = [
        Node(
            package='rviz2',
            executable='rviz2',
            condition=IfCondition(with_rviz),
            name='go2_nav2_rviz2',
            output='screen',
            arguments=['-d', config_paths['rviz']],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ]
    
    # Include launches
    include_launches = [
        # Nav2 Bringup (Localization + Navigation + Lifecycle Manager)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('nav2_bringup'),
                            'launch', 'bringup_launch.py')
            ]),
            launch_arguments={
                'map': map_arg,
                'params_file': config_paths['nav2'],
                'use_sim_time': use_sim_time,
                'autostart': 'true',
            }.items(),
        ),
    ]
    
    return LaunchDescription(
        launch_args +
        core_nodes +
        viz_nodes +
        include_launches
    )
