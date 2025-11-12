import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.actions import SetParameter

def generate_launch_description():
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')
    
    # Get package directories
    g1_pkg_dir = get_package_share_directory('g1_sdk')
    
    # Load URDF file
    urdf_file = os.path.join(g1_pkg_dir, 'urdf', 'g1_23dof.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # RTAB-Map parameters
    parameters = [{
        'frame_id': 'base_link',  # For humanoid, typically base_link or pelvis
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',
        'use_sim_time': use_sim_time,
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_scan': False,
        'subscribe_scan_cloud': True,
        'approx_sync': True,
        'queue_size': 10,
        'qos_image': qos,
        'qos_scan': qos,
        'qos_odom': qos,
        
        # RTAB-Map parameters
        'Rtabmap/DetectionRate': '1.0',
        'Rtabmap/CreateIntermediateNodes': 'true',
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityMaxGraphDepth': '0',
        'RGBD/ProximityPathMaxNeighbors': '10',
        'RGBD/AngularUpdate': '0.01',
        'RGBD/LinearUpdate': '0.01',
        'RGBD/OptimizeFromGraphEnd': 'false',
        
        # Loop closure detection
        'Mem/STMSize': '30',
        'Mem/LaserScanVoxelSize': '0.1',
        'Mem/LaserScanNormalK': '10',
        'Mem/LaserScanRadius': '0.0',  # Keep at 0 to use K-NN
        
        # Graph optimization
        'Optimizer/Strategy': '1',  # 1=GTSAM without gravity constraints
        'Optimizer/GravitySigma': '0.0',  # No gravity constraint without IMU
        
        # 3D mapping with LiDAR
        'Grid/3D': 'true',
        'Grid/RangeMax': '20.0',
        'Grid/ClusterRadius': '0.1',
        'Grid/GroundIsObstacle': 'false',
        'Grid/RayTracing': 'true',
        'Grid/3DGroundIsObstacle': 'false',
        
        # ICP parameters for LiDAR (tuned for humanoid motion)
        # FIXED: Set either PointToPlaneK or PointToPlaneRadius to 0, not both
        'Icp/VoxelSize': '0.1',  # Finer voxel size for detailed matching
        'Icp/MaxCorrespondenceDistance': '1.5',
        'Icp/MaxTranslation': '3.0',  # Increased for dynamic walking
        'Icp/MaxRotation': '1.57',  # Increased for humanoid body motion
        'Icp/Iterations': '50',  # More iterations for complex motion
        'Icp/Epsilon': '0.0001',
        'Icp/PointToPlane': 'true',
        'Icp/PointToPlaneK': '10',
        'Icp/PointToPlaneRadius': '0.0',  # FIXED: Set to 0 to use K-NN only (avoids PCL error)
        'Icp/PointToPlaneGroundNormalsUp': '0.0',  # Disabled - humanoid doesn't maintain fixed ground orientation
        
        # Registration
        'Reg/Strategy': '1',  # 1=ICP
        'Reg/Force3DoF': 'false',  # Allow full 6DOF for humanoid
        
        # Map assembly
        'GridGlobal/MinSize': '20.0',
        'GridGlobal/MaxNodes': '0',
        
        # Visualization
        'Rtabmap/TimeThr': '0',
        'Rtabmap/MemoryThr': '0',
    }]
    
    # Remappings
    remappings = [
        ('scan_cloud', '/utlidar/cloud_livox_mid360'),
        ('odom', '/odom')
        # IMU removed - not required
    ]
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='false',
            description='Use simulation time'
        ),
        
        DeclareLaunchArgument(
            'qos', 
            default_value='2',
            description='QoS profile (1=Reliable, 2=Best effort)'
        ),
        
        DeclareLaunchArgument(
            'localization', 
            default_value='false',
            description='True for localization mode, false for mapping'
        ),
        
        DeclareLaunchArgument(
            'database_path',
            default_value='~/.ros/rtabmap.db',
            description='Path to RTAB-Map database'
        ),
        
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz for visualization'
        ),
        
        # G1 Joint State Publisher Node
        Node(
            package='g1_sdk',
            executable='g1_jointstate',
            name='g1_jointstate',
            output='screen',
        ),
        
        # G1 Odometry Node
        Node(
            package='g1_sdk',
            executable='g1_odom',
            name='g1_odom',
            output='screen',
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        # Joint State Publisher (commented out as in original - likely because g1_jointstate provides this)
        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     output='screen',
        # ),
        
        # Remove Livox driver node since it's already running
        # The driver publishes to /utlidar/cloud_livox_mid360 and /utlidar/imu_livox_mid360
        
        # RTAB-Map node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=[
                '--delete_db_on_start',
                '--Vis/EstimationType', '0',  # 0=3D->3D for full 6DOF humanoid motion
            ],
            condition=UnlessCondition(localization)
        ),
        
        # RTAB-Map localization mode
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                **parameters[0],
                'Mem/IncrementalMemory': 'false',
                'Mem/InitWMWithAllNodes': 'true',
            }],
            remappings=remappings,
            condition=IfCondition(localization)
        ),
        
        # RTAB-Map visualization node
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=parameters,
            remappings=remappings,
            condition=IfCondition(LaunchConfiguration('rviz'))
        ),
        
        # Point cloud assembler for map visualization
        Node(
            package='rtabmap_util',
            executable='point_cloud_assembler',
            name='point_cloud_assembler',
            output='screen',
            parameters=[{
                'max_clouds': 10,
                'fixed_frame': 'map',
                'queue_size': 10,
                'circular_buffer': False,
                'wait_for_transform': 0.1,
            }],
            remappings=[
                ('cloud', '/utlidar/cloud_livox_mid360'),
                ('assembled_cloud', '/rtabmap/assembled_cloud')
            ]
        ),
        
        # Remove static transform publisher - transforms are already published by your system
        # The TF tree shows base_footprint -> base_link -> other frames are already connected
        
        # Map assembler node
        Node(
            package='rtabmap_util',
            executable='map_assembler',
            name='map_assembler',
            output='screen',
            parameters=[{
                'regenerate_local_grids': True,
                'grid_size': 0.05,
                'grid_eroded': False,
                'grid_cell_size': 0.05,
            }],
            remappings=[
                ('mapData', '/rtabmap/mapData'),
                ('grid_map', '/map')
            ]
        ),
    ])