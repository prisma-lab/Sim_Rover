#pragma once

#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

class RoverManager : public rclcpp::Node
{
public:
    RoverManager();

    // Metodo per fermare subito il robot
    inline void all_stop(){
        RCLCPP_WARN(this->get_logger(), "Emergency stop!");
        if(client_ptr_) client_ptr_->async_cancel_all_goals();
        geometry_msgs::msg::Twist tw;
        tw.linear.x = 0.0; tw.linear.y = 0.0; tw.angular.z = 0.0;
        cmd_vel_pub_->publish(tw);
    }

private:
    // ================== Parametri e stati ==================
    std::string home_pose_;
    std::string current_command;
    std::string new_command;
    std::vector<std::string> cv;
    bool command_running;
    bool nav2_running;

    // Action client NAV2
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_ptr_;
    std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle_;

    // Publisher / Subscriber
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rover_feedback_pb_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr coverage_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ================== Funzioni principali ==================
    void command_callback(const std_msgs::msg::String::SharedPtr msg);
    void command_manager_callback();
    void execute_command(const std::string& cmd);

    bool parse_xyyaw(const std::string &s, double &x, double &y, double &yaw);
    bool send_goto_goal(double x, double y, double yaw);

    void nav2_goal_response_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> future);
    void nav2_feedback_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>>, const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);
    void nav2_result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result);

    geometry_msgs::msg::TransformStamped get_tf(const std::string& source_frame, const std::string& target_frame);

    std::vector<std::string> instance2vector(std::string schemaInstance);

    // ================== Coverage ==================
    bool coverage_active;
    std::vector<std::pair<double,double>> coverage_area;   // poligono di copertura
    std::vector<std::pair<double,double>> coverage_path;   // percorso bustrofedon
    size_t current_waypoint_index;
    double swath_width;
    bool navigation_in_progress;

    void generate_boustrophedon_path();
    std::pair<double,double> rotate_point(const std::pair<double,double>& p, double angle);
    std::vector<std::pair<double,double>> rotate_points(const std::vector<std::pair<double,double>>& pts, double angle);
    std::vector<double> find_polygon_intersections(const std::vector<std::pair<double,double>>& pts, double y);
    std::vector<std::pair<double, double>> parse_coverage_points(const std::string& cmd);
    void execute_coverage_step();
};
