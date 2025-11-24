#include <csignal>
#include <iostream>
#include <map> // used for hashmap to give certainty
#include <vector> // used in hashmap
#include <numeric> // used for summing a vector

// ROS 2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "image_geometry/pinhole_camera_model.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/imgproc.hpp"
#include "Eigen/Dense"
#include "eigen3/Eigen/Dense"

using namespace std;
using namespace sensor_msgs;
using namespace cv;

// Publisher
image_transport::Publisher result_img_pub_;
rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr tf_list_pub_;
rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr aruco_info_pub_;

#define SSTR(x) (std::ostringstream() << x).str()
#define ROUND2(x) std::round(x * 100) / 100
#define ROUND3(x) std::round(x * 1000) / 1000

// Define global variables
bool camera_model_computed = false;
bool show_detections;
float marker_size;
image_geometry::PinholeCameraModel camera_model;
Mat distortion_coefficients;
Matx33d intrinsic_matrix;
Ptr<aruco::DetectorParameters> detector_params;
Ptr<cv::aruco::Dictionary> dictionary;
string marker_tf_prefix;
int blur_window_size = 7;
int image_fps = 30;
int image_width = 640;
int image_height = 480;
float text_pt =1.0;
int text_y = 10;
int text_x = 30;
bool enable_blur = true;
bool is_grey = false;
int cnt=0;
int camera_subsampling;
int queue_size = 10;
// hashmap used for uncertainty:
int num_detected = 10;  // =0 -> not used
int min_prec_value = 80; // min precentage value to be a detected marker.
map<int,  std::vector<int>  > ids_hashmap;   // key: ids, value: number within last 100 imgs
int _min_id =0;
int _max_id = 10000;
string node_name;
bool publish_tf = true;

void int_handler(int x) {
    // Disconnect and exit gracefully
    if (show_detections) {
        cv::destroyAllWindows();
    }
    rclcpp::shutdown();
    exit(0);
}

geometry_msgs::msg::Vector3 cv_vector3d_to_geometry_vector3(const Vec3d &vec) {
    geometry_msgs::msg::Vector3 vector;
    vector.x = vec[0];
    vector.y = vec[1];
    vector.z = vec[2];
    return vector;
}

double getPrec(std::vector<int> ids, int i){
    vector<int> current_vector(num_detected);
   current_vector = ids_hashmap[ids[i]];
       int num_detections = std::accumulate(current_vector.begin(), current_vector.end(), 0);
   return (double) num_detections/num_detected *100;
}

tf2::Quaternion cv_vector3d_to_tf_quaternion(const Vec3d &rotation_vector) {
    Mat rotation_matrix;
    auto ax = rotation_vector[0], ay = rotation_vector[1], az = rotation_vector[2];
    auto angle = sqrt(ax * ax + ay * ay + az * az);
    if (angle < 1e-6) { // Avoid division by zero for very small angles
        return tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
    }
    auto cosa = cos(angle * 0.5);
    auto sina = sin(angle * 0.5);
    auto qx = ax * sina / angle;
    auto qy = ay * sina / angle;
    auto qz = az * sina / angle;
    auto qw = cosa;
    tf2::Quaternion q;
    q.setValue(qx, qy, qz, qw);
    return q;
}

geometry_msgs::msg::Transform create_transform(const Vec3d &tvec, const Vec3d &rotation_vector) {
    geometry_msgs::msg::Transform transform;
    transform.translation = cv_vector3d_to_geometry_vector3(tvec);
    tf2::Quaternion quaternion = cv_vector3d_to_tf_quaternion(rotation_vector);
    transform.rotation.x = quaternion.x();
    transform.rotation.y = quaternion.y();
    transform.rotation.z = quaternion.z();
    transform.rotation.w = quaternion.w();
    return transform;
}

void callback_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (camera_model_computed) {
        return;
    }
    camera_model.fromCameraInfo(*msg);
    camera_model.distortionCoeffs().copyTo(distortion_coefficients);
    intrinsic_matrix = camera_model.intrinsicMatrix();
    camera_model_computed = true;
    RCLCPP_INFO(rclcpp::get_logger(node_name), "Camera model is computed");
}

void callback(const sensor_msgs::msg::Image::SharedPtr image_msg, const rclcpp::Node::SharedPtr& node) {
    if (++cnt % camera_subsampling != 0) 
        return;

    cnt = 0;   
    if (!camera_model_computed) {
        RCLCPP_INFO(rclcpp::get_logger(node_name), "Camera model is not computed yet");
        return;
    }

    string frame_id = image_msg->header.frame_id;
    auto stamp = image_msg->header.stamp;

    auto image = cv_bridge::toCvShare(image_msg)->image;
    cv::Mat display_image(image);

    // Smooth the image to improve detection results
    if (enable_blur) {
        GaussianBlur(image, image, Size(blur_window_size, blur_window_size), 0, 0);
    }

    // Detect the markers
    vector<int> ids;
    vector<int> ids_not_tresholded;
    vector<vector<Point2f>> corners, rejected;
    aruco::detectMarkers(image, dictionary, corners, ids_not_tresholded, detector_params, rejected);
    std_msgs::msg::Int32MultiArray ar_msg;

    for (int i = 0; i < ids_not_tresholded.size(); i++) {
        if (ids_not_tresholded[i] >= _min_id && ids_not_tresholded[i] <= _max_id) {
            ar_msg.data.push_back(ids_not_tresholded[i]);
            ids.push_back(ids_not_tresholded[i]);
        } else {
            corners.erase(corners.begin() + i);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "AR %s discard id %d", node_name.c_str(), ids_not_tresholded[i]);
        }
    }

    if (ids.size() > 0) {
        aruco_info_pub_->publish(ar_msg); // Publish id list, ONLY IF THERE IS A DETECTION !!
        
        // Compute poses of markers
        vector<Vec3d> rotation_vectors, translation_vectors;
        aruco::estimatePoseSingleMarkers(corners, marker_size, intrinsic_matrix, distortion_coefficients, rotation_vectors, translation_vectors);
        for (auto i = 0; i < rotation_vectors.size(); ++i) {
            aruco::drawAxis(display_image, intrinsic_matrix, distortion_coefficients, rotation_vectors[i], translation_vectors[i], marker_size * 0.5f);
        }

        // Draw marker poses
        aruco::drawDetectedMarkers(display_image, corners, ids);

        if (result_img_pub_.getNumSubscribers() > 0) {
            cv::putText(display_image, node_name + "   " + SSTR(image_width) + "x" + SSTR(image_height) + "@" + SSTR(image_fps) + "fps", 
                        cv::Point(text_y, text_x), cv::FONT_HERSHEY_SIMPLEX, text_pt, CV_RGB(255, 0, 0), 2);

            for (int i = 0; i < ids.size(); i++) {
                double prec = getPrec(ids, i);
                Vec3d distance_z_first = translation_vectors[i];
                double distance_z = ROUND3(distance_z_first[2]);
                cv::putText(display_image, "id: " + SSTR(ids[i]) + " z dis: " + SSTR(distance_z) + " m  ", 
                            cv::Point(text_y, text_x * 2 + i * text_x), cv::FONT_HERSHEY_SIMPLEX, text_pt, CV_RGB(0, 255, 0), 2);
            }

            std_msgs::msg::Header hdr;
            hdr.stamp = stamp;
            hdr.frame_id = frame_id;
            if (is_grey) {
                result_img_pub_.publish(cv_bridge::CvImage(hdr, "mono8", display_image).toImageMsg());
            } else {
                result_img_pub_.publish(cv_bridge::CvImage(hdr, "bgr8", display_image).toImageMsg());
            }
        }

        if (show_detections) {
            imshow("markers", display_image); // OpenCV imshow
            auto key = waitKey(1);
            if (key == 27) {
                RCLCPP_INFO(rclcpp::get_logger(node_name), "ESC pressed, exit the program");
                rclcpp::shutdown();
            }
        }

        // Publish TFs for each of the markers
        geometry_msgs::msg::TransformStamped tf_msg;
        
        static std::shared_ptr<tf2_ros::TransformBroadcaster> br = nullptr;
        if (!br) {
            br = std::make_shared<tf2_ros::TransformBroadcaster>(node);
        }
        for (auto i = 0; i < rotation_vectors.size(); ++i) {
            auto translation_vector = translation_vectors[i];
            auto rotation_vector = rotation_vectors[i];
            auto transform = create_transform(translation_vector, rotation_vector);
            tf_msg.header.stamp = stamp;
            tf_msg.header.frame_id = frame_id;
            stringstream ss;
            ss << marker_tf_prefix << ids[i];
            tf_msg.child_frame_id = ss.str();
            tf_msg.transform = transform;
            if( publish_tf ) {
                br->sendTransform(tf_msg);
            }
        }

        tf_list_pub_->publish(tf_msg);
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("aruco_detector_ocv");

    map<string, aruco::PREDEFINED_DICTIONARY_NAME> dictionary_names = {
        {"DICT_4X4_50", aruco::DICT_4X4_50},
        {"DICT_4X4_100", aruco::DICT_4X4_100},
        {"DICT_4X4_250", aruco::DICT_4X4_250},
        {"DICT_4X4_1000", aruco::DICT_4X4_1000},
        {"DICT_5X5_50", aruco::DICT_5X5_50},
        {"DICT_5X5_100", aruco::DICT_5X5_100},
        {"DICT_5X5_250", aruco::DICT_5X5_250},
        {"DICT_5X5_1000", aruco::DICT_5X5_1000},
        {"DICT_6X6_50", aruco::DICT_6X6_50},
        {"DICT_6X6_100", aruco::DICT_6X6_100},
        {"DICT_6X6_250", aruco::DICT_6X6_250},
        {"DICT_6X6_1000", aruco::DICT_6X6_1000},
        {"DICT_7X7_50", aruco::DICT_7X7_50},
        {"DICT_7X7_100", aruco::DICT_7X7_100},
        {"DICT_7X7_250", aruco::DICT_7X7_250},
        {"DICT_7X7_1000", aruco::DICT_7X7_1000},
        {"DICT_ARUCO_ORIGINAL", aruco::DICT_ARUCO_ORIGINAL}
    };

    signal(SIGINT, int_handler);

    // Declare parameters
    string rgb_topic, rgb_info_topic, dictionary_name;
    node->declare_parameter("camera", "/rover/color/image_raw");
    node->declare_parameter("camera_info", "/rover/color/camera_info");
    node->declare_parameter("show_detections", false);
    node->declare_parameter("tf_prefix", "marker");
    node->declare_parameter("marker_size", 0.09f);
    node->declare_parameter("enable_blur", true);
    node->declare_parameter("blur_window_size", 7);
    node->declare_parameter("image_fps", 30);
    node->declare_parameter("image_width", 640);
    node->declare_parameter("image_height", 480);
    node->declare_parameter("num_detected", 50);
    node->declare_parameter("min_prec_value", 80);
    node->declare_parameter("node_name", "aruco_detector");
    node->declare_parameter("is_grey", false);
    node->declare_parameter("camera_rate_subsampling", 1);
    node->declare_parameter("topic_queue_size", 1);
    node->declare_parameter("min_id", 100);
    node->declare_parameter("max_id", 300);
    node->declare_parameter("dictionary_name", "DICT_ARUCO_ORIGINAL");
    node->declare_parameter("publish_tf", false);

    // Get parameters
    node->get_parameter("camera", rgb_topic);
    node->get_parameter("camera_info", rgb_info_topic);
    node->get_parameter("show_detections", show_detections);
    node->get_parameter("tf_prefix", marker_tf_prefix);
    node->get_parameter("marker_size", marker_size);
    node->get_parameter("enable_blur", enable_blur);
    node->get_parameter("blur_window_size", blur_window_size);
    node->get_parameter("image_fps", image_fps);
    node->get_parameter("image_width", image_width);
    node->get_parameter("image_height", image_height);
    node->get_parameter("num_detected", num_detected);
    node->get_parameter("min_prec_value", min_prec_value);
    node->get_parameter("node_name", node_name);
    node->get_parameter("is_grey", is_grey);
    node->get_parameter("camera_rate_subsampling", camera_subsampling);
    node->get_parameter("topic_queue_size", queue_size);
    node->get_parameter("min_id", _min_id);
    node->get_parameter("max_id", _max_id);
    node->get_parameter("dictionary_name", dictionary_name);
    node->get_parameter("publish_tf", publish_tf);

    // Initialize ARUCO detector parameters
    text_pt = float(image_width * image_height) / 307200.0 * 0.7;
    text_y = image_width / 64;
    text_x = (image_width / 48) * 3;

    detector_params = aruco::DetectorParameters::create();
    dictionary = aruco::getPredefinedDictionary(dictionary_names[dictionary_name]);

    if (show_detections) {
        namedWindow("markers", cv::WINDOW_KEEPRATIO);
    }

    // Subscribers
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.best_effort();  // Imposta come best effort
    auto rgb_sub = node->create_subscription<sensor_msgs::msg::Image>(
        rgb_topic, qos, [node](const sensor_msgs::msg::Image::SharedPtr msg) {
            callback(msg, node);
        });
    auto rgb_info_sub = node->create_subscription<sensor_msgs::msg::CameraInfo>(
        rgb_info_topic, queue_size, callback_camera_info);

    // Publishers
    auto image_transport = image_transport::create_publisher(node.get(), node_name + "/result_img");
    result_img_pub_ = image_transport;
    tf_list_pub_ = node->create_publisher<geometry_msgs::msg::TransformStamped>(node_name + "/tf_list", queue_size);
    aruco_info_pub_ = node->create_publisher<std_msgs::msg::Int32MultiArray>(node_name + "/aruco_list", queue_size);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
