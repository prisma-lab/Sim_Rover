from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    this_package_name = 'yolov11_ros2'
    
    # Run the yolov11 node with OpenVINO
    yolov11_node = Node(
        package=this_package_name,
        executable='yolov11_node',
        output='screen',
        parameters=[
            {'device': 'CPU',  # OpenVINO device
             'model': 'yolo11n-seg.pt'},
        ],
        remappings=[
            ('/rover/color/image_raw/compressed', '/out/compressed'), #Added for simulation
        ],
    )
    
    return LaunchDescription([
        yolov11_node,
    ])