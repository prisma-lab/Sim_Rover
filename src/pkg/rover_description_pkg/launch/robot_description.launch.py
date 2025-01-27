from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments


    declared_arguments.append(
        DeclareLaunchArgument(
            "rover_description",
            default_value=PathJoinSubstitution(
                [FindPackageShare("rover_description_pkg"), "urdf", "rover.xacro"]
            ),
            description="URDF/XACRO description file (absolute path) with the robot.",
        )
    )


#    declared_arguments.append(
#        DeclareLaunchArgument(
#            "rviz_config_file",
#            default_value=PathJoinSubstitution(
#                [FindPackageShare("ur_description"), "rviz", "view_robot.rviz"]
#            ),
#            description="RViz config file (absolute path) to use when launching rviz.",
#        )
#   )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='rover/',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "frame_prefix",
            default_value='rover/',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )

    # Initialize Arguments
    rover_description = LaunchConfiguration("rover_description")
#    tf_prefix = LaunchConfiguration("tf_prefix")
    frame_prefix = LaunchConfiguration("frame_prefix")
#    rviz_config_file = LaunchConfiguration("rviz_config_file")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            rover_description,
            " ",
            "frame_prefix:=",  
            frame_prefix,
        ]
    )
    robot_description = {"robot_description": robot_description_content, "frame_prefix":frame_prefix}

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
#    rviz_node = Node(
#        package="rviz2",
#        executable="rviz2",
#        name="rviz2",
#        output="log",
#        arguments=["-d", rviz_config_file],
#    )

    nodes_to_start = [
        joint_state_publisher_node,
        robot_state_publisher_node,
#        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)