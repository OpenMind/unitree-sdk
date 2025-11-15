import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    qos = LaunchConfiguration("qos")
    localization = LaunchConfiguration("localization")
    database_path = LaunchConfiguration("database_path")

    # Get package directories
    g1_pkg_dir = get_package_share_directory("g1_sdk")

    # Default database path in package maps folder
    default_db_path = os.path.join(g1_pkg_dir, "maps", "rtabmap.db")

    # RTAB-Map parameters
    parameters = [
        {
            "frame_id": "base_link",  # For humanoid, typically base_link or pelvis
            "odom_frame_id": "odom",
            "map_frame_id": "map",
            "use_sim_time": use_sim_time,
            "subscribe_depth": False,
            "subscribe_rgb": False,
            "subscribe_scan": False,
            "subscribe_scan_cloud": True,
            "approx_sync": True,
            "queue_size": 10,
            "qos_image": qos,
            "qos_scan": qos,
            "qos_odom": qos,
            "database_path": database_path,
            # Database disk-based saving
            # Force continuous disk writes instead of memory-only operation
            "Db/Sqlite3/InMemory": "false",  # Critical: Write to disk continuously
            "Db/Sqlite3/JournalMode": "WAL",  # Write-Ahead Logging for crash safety
            "Db/Sqlite3/CacheSize": "10000",  # Cache size (lower = more frequent writes)
            "Db/Sqlite3/Synchronous": "NORMAL",  # Balance between safety and speed
            # RTAB-Map parameters
            "Rtabmap/DetectionRate": "1.0",
            "Rtabmap/CreateIntermediateNodes": "true",
            "Rtabmap/MemoryThr": "500",  # ADDED: Trigger database maintenance at 500 nodes
            "RGBD/NeighborLinkRefining": "true",
            "RGBD/ProximityBySpace": "true",
            "RGBD/ProximityMaxGraphDepth": "0",
            "RGBD/ProximityPathMaxNeighbors": "10",
            "RGBD/AngularUpdate": "0.01",
            "RGBD/LinearUpdate": "0.01",
            "RGBD/OptimizeFromGraphEnd": "false",
            # Loop closure detection
            "Mem/STMSize": "30",
            "Mem/LaserScanVoxelSize": "0.1",
            "Mem/LaserScanNormalK": "10",
            "Mem/LaserScanRadius": "0.0",  # Keep at 0 to use K-NN
            # Graph optimization
            "Optimizer/Strategy": "1",  # 1=GTSAM without gravity constraints
            "Optimizer/GravitySigma": "0.0",  # No gravity constraint without IMU
            # 3D mapping with LiDAR
            "Grid/3D": "true",
            "Grid/RangeMax": "20.0",
            "Grid/ClusterRadius": "0.1",
            "Grid/GroundIsObstacle": "false",
            "Grid/RayTracing": "true",
            "Grid/3DGroundIsObstacle": "false",
            # ICP parameters for LiDAR (tuned for humanoid motion)
            "Icp/VoxelSize": "0.1",  # Finer voxel size for detailed matching
            "Icp/MaxCorrespondenceDistance": "1.5",
            "Icp/MaxTranslation": "3.0",  # Increased for dynamic walking
            "Icp/MaxRotation": "1.57",  # Increased for humanoid body motion
            "Icp/Iterations": "50",  # More iterations for complex motion
            "Icp/Epsilon": "0.0001",
            "Icp/PointToPlane": "true",
            "Icp/PointToPlaneK": "10",
            "Icp/PointToPlaneRadius": "0.0",  # Set to 0 to use K-NN only (avoids PCL error)
            "Icp/PointToPlaneGroundNormalsUp": "0.0",  # Disabled - humanoid doesn't maintain fixed ground orientation
            # Registration
            "Reg/Strategy": "1",  # 1=ICP
            "Reg/Force3DoF": "false",  # Allow full 6DOF for humanoid
            # Map assembly
            "GridGlobal/MinSize": "20.0",
            "GridGlobal/MaxNodes": "0",
            # Visualization
            "Rtabmap/TimeThr": "0",
            # MemoryThr moved up to trigger saves
        }
    ]

    # Remappings
    remappings = [
        ("scan_cloud", "/utlidar/cloud_livox_mid360"),
        ("odom", "/odom"),
        # IMU removed - not required
    ]

    # Load URDF for robot_state_publisher
    urdf_file = os.path.join(g1_pkg_dir, "urdf", "g1_23dof.urdf")
    with open(urdf_file, "r") as infp:
        robot_desc = infp.read()

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use simulation time"
            ),
            DeclareLaunchArgument(
                "qos",
                default_value="2",
                description="QoS profile (1=Reliable, 2=Best effort)",
            ),
            DeclareLaunchArgument(
                "localization",
                default_value="false",
                description="True for localization mode, false for mapping",
            ),
            DeclareLaunchArgument(
                "database_path",
                default_value=default_db_path,
                description="Path to RTAB-Map database",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Launch RViz for visualization",
            ),
            # Mapping mode node (with delete_db_on_start)
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                name="rtabmap",
                output="screen",
                parameters=parameters,
                remappings=remappings,
                arguments=[
                    "--delete_db_on_start",  # Fresh map for SLAM sessions
                    "--Vis/EstimationType",
                    "0",
                ],
                condition=UnlessCondition(localization),
            ),
            # Localization mode node (no delete, uses existing map)
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                name="rtabmap",
                output="screen",
                parameters=[
                    {
                        **parameters[0],
                        "Mem/IncrementalMemory": "false",  # Don't add new nodes
                        "Mem/InitWMWithAllNodes": "true",  # Load entire map
                    }
                ],
                remappings=remappings,
                condition=IfCondition(localization),
            ),
            Node(
                package="rtabmap_viz",
                executable="rtabmap_viz",
                name="rtabmap_viz",
                output="screen",
                parameters=parameters,
                remappings=remappings,
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
            Node(
                package="rtabmap_util",
                executable="point_cloud_assembler",
                name="point_cloud_assembler",
                output="screen",
                parameters=[
                    {
                        "max_clouds": 10,
                        "fixed_frame": "map",
                        "queue_size": 10,
                        "circular_buffer": False,
                        "wait_for_transform": 0.1,
                    }
                ],
                remappings=[
                    ("cloud", "/utlidar/cloud_livox_mid360"),
                    ("assembled_cloud", "/rtabmap/assembled_cloud"),
                ],
            ),
            Node(
                package="rtabmap_util",
                executable="map_assembler",
                name="map_assembler",
                output="screen",
                parameters=[
                    {
                        "regenerate_local_grids": True,
                        "grid_size": 0.05,
                        "grid_eroded": False,
                        "grid_cell_size": 0.05,
                    }
                ],
                remappings=[("mapData", "/rtabmap/mapData"), ("grid_map", "/map")],
            ),
            # G1-specific nodes and robot_state_publisher
            Node(
                package="g1_sdk",
                executable="cmd_vel_to_g1",
                name="cmd_vel_to_g1",
                output="screen",
            ),
            Node(
                package="g1_sdk",
                executable="g1_loco_action",
                name="g1_loco_action",
                output="screen",
            ),
            Node(
                package="g1_sdk",
                executable="g1_jointstate",
                name="g1_jointstate",
                output="screen",
            ),
            Node(
                package="g1_sdk",
                executable="g1_odom",
                name="g1_odom",
                output="screen",
            ),
            Node(
                package="g1_sdk",
                executable="waypoint_manager",
                name="waypoint_manager",
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_desc}],
            ),
            Node(
                package="joy", executable="joy_node", name="joy_node", output="screen"
            ),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                name="teleop_twist_joy_node",
                output="screen",
                parameters=[
                    {
                        "axis_linear.x": 1,
                        "axis_linear.y": 0,
                        "axis_angular.z": 3,
                        "enable_button": 7,
                        "scale_linear.x": 0.5,
                        "scale_angular.z": 1.0,
                        "enable_turbo_button": 6,
                        "scale_turbo_linear.x": 1.5,
                        "scale_turbo_angular.z": 2.0,
                    }
                ],
            ),
        ]
    )
