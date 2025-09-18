from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='rover',
            description='Namespace for sample topics'
        ),
        
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='camera_to_base_transform',
        #     arguments=[
        #         '--x', '0.3', '--y', '0', '--z', '1.0',  # Modifica queste coordinate!
        #         '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
        #         '--frame-id', 'rover/base_footprint', '--child-frame-id', 'rover/camera_depth_optical_frame'
        #     ],
        # ),
        
        # PointCloud to LaserScan node - MODIFICATO per il tuo topic
        Node(
            package='pointcloud_to_laserscan', 
            executable='pointcloud_to_laserscan_node',
            remappings=[
                ('cloud_in', '/depth/image_raw/points'),
            ],
            parameters=[{
                'target_frame': 'rover/lidar_link',
                'transform_tolerance': 0.01,  # Ridotto da 0.1
                'min_height': -0.2,
                'max_height': 0.5,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0174533,
                'scan_time': 0.033,  # ~30 Hz, corrispondente alla frequenza della camera
                'range_min': 0.0,
                'range_max': 20.0,
                'use_inf': False,
                'inf_epsilon': 0.1,
                'queue_size': 10,  # Aggiunto: dimensione della coda
            }],
            name='pointcloud_to_laserscan'
        )
    ])