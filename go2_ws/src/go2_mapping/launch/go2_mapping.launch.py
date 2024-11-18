from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package share directory
    pkg_share = os.path.join(
        get_package_share_directory('go2_mapping'))

    # Node to convert PointCloud2 to LaserScan
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/utlidar/cloud_deskewed'),  # Replace if your topic is different
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.1,
            'min_height': -0.1,
            'max_height': 0.1,
            'angle_min': -3.14,
            'angle_max': 3.14,
            'angle_increment': 0.01,
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 20.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
        output='screen',
    )

    # Node to run slam_toolbox
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',  # Use 'async_slam_toolbox_node' for async SLAM
        name='slam_toolbox',
        parameters=[
            os.path.join(pkg_share, 'config', 'mapper_params_online_sync.yaml'),
        ],
        remappings=[
            ('scan', '/scan'),
        ],
        output='screen',
    )

    return LaunchDescription([
        pointcloud_to_laserscan_node,
        slam_toolbox_node,
    ])
