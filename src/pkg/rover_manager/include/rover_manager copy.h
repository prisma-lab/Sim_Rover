#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_msgs/msg/string.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/polygon.hpp>

using namespace std::chrono_literals;

class RoverManager : public rclcpp::Node
{
public:
    //using NavigateToPose = nav2_msgs::action::NavigateToPose;
    //using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    RoverManager();
    
    inline void all_stop(){
      std::cout<<"STOPPING ROBOT!"<<std::endl;
      this->client_ptr_->async_cancel_all_goals();
    }

private:
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    //rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;
    //std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>* goal_handle_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr coverage_publisher_;

    std::string current_command;
    std::string new_command;
    bool command_running;
    bool nav2_running;
    bool coverage_active;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::vector<std::string> cv;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rover_feedback_pb_;



    std::vector<std::string> instance2vector(std::string schemaInstance);

    geometry_msgs::msg::TransformStamped get_tf(std::string source_frame, std::string target_frame);

    void command_callback(const std_msgs::msg::String::SharedPtr msg);

    void command_manager_callback();

    //bool cancel_goal();

    void execute_command(std::string cmd);

    bool start_coverage_mission(const std::string& area_spec);

    //void nav2_goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> future);
    void nav2_goal_response_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> future);

    //void nav2_feedback_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>,
    //                       const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);
    void nav2_feedback_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>>,
                           const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);

    void nav2_result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result);
};
