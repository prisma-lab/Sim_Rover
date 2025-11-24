from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package = "tf2_ros", 
             executable = "static_transform_publisher",
             arguments = ["17", "5", "1.5", "0", "0", "0", "map", "exp00"]),
        Node(package = "tf2_ros", 
             executable = "static_transform_publisher",
             arguments = ["8", "9", "1.5", "0", "0", "0", "map", "exp10"]),
     #    Node(package = "tf2_ros", 
     #         executable = "static_transform_publisher",
     #         arguments = ["4", "2", "1.5", "0", "0", "0", "map", "exp20"]),
     #    Node(package = "tf2_ros", 
     #         executable = "static_transform_publisher",
     #         arguments = ["6", "0", "1.5", "0", "0", "0", "map", "exp30"]),
        Node(package = "tf2_ros", 
             executable = "static_transform_publisher",
             arguments = ["3", "3", "1.5", "0", "0", "0", "map", "exp01"]),
        Node(package = "tf2_ros", 
             executable = "static_transform_publisher",
             arguments = ["7", "3", "1.5", "0", "0", "0", "map", "exp11"]),
     #    Node(package = "tf2_ros", 
     #         executable = "static_transform_publisher",
     #         arguments = ["2", "3", "1.5", "0", "0", "0", "map", "exp21"]),
        Node(
            package='rover_manager',
            executable='rover_manager',
            #name='mimic',
            name='rover_manager',
            output="screen"
        ),
    ])
