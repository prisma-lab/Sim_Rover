from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 256000,
                'frame_id': 'rover/laser',
                'inverted': True,
                'angle_compensate': True,
                'scan_qos.reliability': 'SYSTEM_DEFAULT',
                'scan_qos.history': 'keep_all',
                'scan_qos.depth': 100,
                'scan_qos.durability': 'volatile'
            }],
            remappings=[
                ('scan', 'rover/scan')
            ]
        ),
    ])
