from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('node_name', default_value='rover_aruco_detector'),
        DeclareLaunchArgument('image_width', default_value='1280'),
        DeclareLaunchArgument('image_height', default_value='720'),
        DeclareLaunchArgument('image_fps', default_value='30'),
        DeclareLaunchArgument('camera', default_value='/rover/color/image_raw'),
        DeclareLaunchArgument('camera_info', default_value='/rover/color/camera_info'),
        DeclareLaunchArgument('camera_rate_subsampling', default_value='5'),
        DeclareLaunchArgument('topic_queue_size', default_value='1'),
        DeclareLaunchArgument('tf_prefix', default_value=[LaunchConfiguration('node_name'), '/marker_id']),
        DeclareLaunchArgument('show_detections', default_value='false'),
        DeclareLaunchArgument('marker_size', default_value='0.10'),
        DeclareLaunchArgument('dictionary_name', default_value='DICT_ARUCO_ORIGINAL'),
        DeclareLaunchArgument('blur_window_size', default_value='7'),
        DeclareLaunchArgument('num_detected', default_value='25'),
        DeclareLaunchArgument('min_prec_value', default_value='80'),
        DeclareLaunchArgument('enable_blur', default_value='true'),
        DeclareLaunchArgument('is_grey', default_value='false'),
        DeclareLaunchArgument('min_id', default_value='100'),
        DeclareLaunchArgument('max_id', default_value='300'),
        DeclareLaunchArgument('publish_tf', default_value='true'),

        # Node definition
        Node(
            package='aruco_detector_ocv_ros2',
            executable='aruco_detector_node',
            name=LaunchConfiguration('node_name'),  # Correctly use 'node_name'
            output='screen',
            parameters=[
                {'node_name': LaunchConfiguration('node_name')},
                {'camera': LaunchConfiguration('camera')},
                {'camera_info': LaunchConfiguration('camera_info')},
                {'image_transport': 'theora'},
                {'is_grey': LaunchConfiguration('is_grey')},
                {'tf_prefix': LaunchConfiguration('tf_prefix')},
                {'show_detections': LaunchConfiguration('show_detections')},
                {'marker_size': LaunchConfiguration('marker_size')},
                {'dictionary_name': LaunchConfiguration('dictionary_name')},
                {'image_width': LaunchConfiguration('image_width')},
                {'image_height': LaunchConfiguration('image_height')},
                {'image_fps': LaunchConfiguration('image_fps')},
                {'camera_rate_subsampling': LaunchConfiguration('camera_rate_subsampling')},
                {'enable_blur': LaunchConfiguration('enable_blur')},
                {'blur_window_size': LaunchConfiguration('blur_window_size')},
                {'num_detected': LaunchConfiguration('num_detected')},
                {'min_prec_value': LaunchConfiguration('min_prec_value')},
                {'topic_queue_size': LaunchConfiguration('topic_queue_size')},
                {'min_id': LaunchConfiguration('min_id')},
                {'max_id': LaunchConfiguration('max_id')},
                {'publish_tf': LaunchConfiguration('publish_tf')},
            ]
        )
    ])
