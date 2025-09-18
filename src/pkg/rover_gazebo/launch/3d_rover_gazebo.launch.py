import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():
    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf = os.path.join(get_package_share_directory(
        'rover_description_pkg'), 'urdf', 'rover.xacro')
    world = LaunchConfiguration('world')
    frame_prefix = LaunchConfiguration("frame_prefix")
    tf_prefix = LaunchConfiguration("tf_prefix")   

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='',
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
  

    robot_desc = ParameterValue(Command(
            ['xacro ', urdf,
             " ",            
            "tf_prefix:=",  
            tf_prefix,
            " ",
            "frame_prefix:=",  
            frame_prefix]),value_type=str)
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='maze.sdf',
        description='World file to use in Gazebo')
    
    gz_world_arg = PathJoinSubstitution([
        get_package_share_directory('rover_gazebo'), 'worlds', world])

    # Include the gz sim launch file  
    gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args" : gz_world_arg 
        }.items()
    )
    
    # Spawn Rover Robot
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "prisma_rover",
            "-allow_renaming", "true",
            "-x", "7.0",
            "-y", "0.5",
            "-z", "1.2",
        ]
    )
    
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/odom/wheels@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",          # @ is for a 1 to 1 conversion, [ is for a 1 to multiples conversion
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/color/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/depth/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/color/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/depth/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/depth/image_raw/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
            '/livox/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/livox/scan/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
        ],
        parameters=[{
            # 'qos_overrides./clock.publisher.reliability': 'best_effort',
            # 'qos_overrides./cmd_vel.publisher.reliability': 'best_effort',
            # 'qos_overrides./cmd_vel.subscription.reliability': 'best_effort',
            # 'qos_overrides./color/camera_info.publisher.reliability': 'best_effort',
            # 'qos_overrides./color/camera_info.subscription.reliability': 'best_effort',
            # 'qos_overrides./color/image_raw.publisher.reliability': 'best_effort',
            # 'qos_overrides./color/image_raw.subscription.reliability': 'best_effort',
            # 'qos_overrides./depth/camera_info.publisher.reliability': 'best_effort',
            # 'qos_overrides./depth/camera_info.subscription.reliability': 'best_effort',
            # 'qos_overrides./depth/color/points.publisher.reliability': 'best_effort',
            # 'qos_overrides./depth/color/points.subscription.reliability': 'best_effort',
            # 'qos_overrides./joint_states.publisher.reliability': 'best_effort',            
            # 'qos_overrides./imu/data.publisher.reliability': 'best_effort',
            # 'qos_overrides./imu/data.subscription.reliability': 'best_effort',
            # 'qos_overrides./livox/scan.publisher.reliability': 'best_effort',
            # 'qos_overrides./livox/scan.subscription.reliability': 'best_effort',
            # 'qos_overrides./livox/scan/points.publisher.reliability': 'best_effort',
            # 'qos_overrides./livox/scan/points.subscription.reliability': 'best_effort',
            # 'qos_overrides./odom/wheels.publisher.reliability': 'best_effort',
            # 'qos_overrides./odom/wheels.subscription.reliability': 'best_effort',
            # 'qos_overrides./parameter_events.publisher.reliability': 'best_effort',
            # 'qos_overrides./scan.publisher.reliability': 'best_effort',
            # 'qos_overrides./scan.subscription.reliability': 'best_effort',
            # 'qos_overrides./tf.publisher.reliability': 'best_effort'
        }]
    )

    # Robot state publisher
    params = {'use_sim_time': use_sim_time, 'robot_description': robot_desc, "tf_prefix":tf_prefix,"frame_prefix":frame_prefix}
    start_robot_state_publisher_cmd = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[params],
            arguments=[])

    # Create the launch description and populate
    ld = LaunchDescription(declared_arguments)

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)

    # Launch Gazebo
    ld.add_action(gz_sim)
    ld.add_action(gz_spawn_entity)
    ld.add_action(gz_ros2_bridge)

    # Launch Robot State Publisher
    ld.add_action(start_robot_state_publisher_cmd)

    return ld
