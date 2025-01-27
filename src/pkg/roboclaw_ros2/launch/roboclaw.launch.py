'''<launch>

    <node pkg="roboclaw_ros2" exec="roboclaw_node" name="roboclaw" output="screen">
        <param name="~serial_port" value="/dev/ttyACM1"/>
        <param name="~baudrate" value="115200"/>
        <param name="~roboclaws" value="1"/>
    </node>


</launch>'''
from launch_ros.actions import Node , SetParameter
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='roboclaw_ros2',
            executable='roboclaw_node',
            parameters=[
                {"serial_port": "/dev/ttyACM0"},
                {"baudrate": 115200},
                {"roboclaws": 1}
            ],
            output='screen',
            emulate_tty=True
        )
    ])