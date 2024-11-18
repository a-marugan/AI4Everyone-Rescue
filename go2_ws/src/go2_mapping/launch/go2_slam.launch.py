import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Parameters for pointcloud_to_laserscan
    params = {
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
    }
    
    rviz_config_file = os.path.join(
        get_package_share_directory('go2_mapping'),
        'rviz',
        'go2_mapping.rviz'
    )
    
    mapper_params = os.path.join(
        get_package_share_directory('go2_mapping'),
        'config',
        'mapper_params_online_async.yaml'
    )

    return LaunchDescription([
        # PointCloud2 to LaserScan conversion
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/lidar/point_cloud'),
                ('scan', '/scan'),
            ],
            parameters=[params],
            output='screen'
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[mapper_params],
            remappings=[('/scan', '/scan')],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
    ])
