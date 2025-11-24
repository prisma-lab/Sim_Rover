import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # FORZA GAZEBO GARDEN
    set_gz_version = SetEnvironmentVariable(
        name='GZ_VERSION',
        value='garden'
    )
    
    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf = os.path.join(get_package_share_directory(
        'rover_description_pkg'), 'urdf', 'rover.xacro')
    world = LaunchConfiguration('world')
    frame_prefix = LaunchConfiguration("frame_prefix")
    tf_prefix = LaunchConfiguration("tf_prefix")
    namespace = LaunchConfiguration("namespace")   

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value='rover',
            description="",
        )
    )
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
            namespace,
            " ",
            "frame_prefix:=",
            namespace,
            " ",
            "namespace:=",  
            namespace]),value_type=str)
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=['leonardo_race.sdf', ' -r'],
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
        name='urdf_spawner_2',
        arguments=[
            # "-world", "leonardo_race",
            "-topic", "/rover/robot_description",
            "-name", "prisma_rover",
            # "-allow_renaming", "true",
            "-z", "0.15",
        ]
    )
    
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/rover/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/odom/wheels@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            # '/rover/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/rover/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/rover/depth/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/rover/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/rover/depth/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/rover/depth/image_raw/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        ],
        parameters=[{
            'qos_overrides./scan.publisher.reliability': 'best_effort',
            'qos_overrides./scan.subscription.reliability': 'best_effort',
        }]
    )

    # Robot state publisher
    params = {'use_sim_time': use_sim_time, 'robot_description': robot_desc, "tf_prefix":[namespace, '/'],"frame_prefix":[namespace, '/']}
    start_robot_state_publisher_cmd = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            output='both',
            parameters=[params],
            arguments=[])

    start_joint_state_publisher_cmd = Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            namespace=namespace,
        )

    image_compressed_republisher = Node(
        package="image_transport",
        executable="republish",
        arguments=[
            "raw", "compressed",
            "--ros-args",
            "-r", "in:=/rover/color/image_raw",
            "-r", "out:=/rover/color/image_raw/compressed",
        ]
    )    

    # Create the launch description and populate
    ld = LaunchDescription(declared_arguments)

    # AGGIUNGI PER PRIMO IL SET DELLA VERSIONE
    ld.add_action(set_gz_version)
    
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)

    # Launch Gazebo
    ld.add_action(gz_sim)
    ld.add_action(gz_spawn_entity)
    ld.add_action(gz_ros2_bridge)

    #Republisher image per yolo
    ld.add_action(image_compressed_republisher)

    # Launch Robot State Publisher
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_cmd)

    return ld
