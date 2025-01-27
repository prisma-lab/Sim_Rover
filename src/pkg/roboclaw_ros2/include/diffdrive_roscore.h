/**
*
* Copyright (c) 2018 Carroll Vance.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
        * the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
* THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
        * DEALINGS IN THE SOFTWARE.
*/

#ifndef PROJECT_DIFFDRIVE_ROSCORE_H
#define PROJECT_DIFFDRIVE_ROSCORE_H

// #include "ros/ros.h"
#include "rclcpp/rclcpp.hpp"

// #include "ros/package.h"

#include "roboclaw_ros2/msg/roboclaw_encoder_steps.hpp"
#include "roboclaw_ros2/msg/roboclaw_motor_velocity.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace roboclaw {

    class diffdrive_roscore : public rclcpp::Node {
    public:
        diffdrive_roscore();
    private:

        //ros::NodeHandle nh;
        //ros::NodeHandle nh_private;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

        rclcpp::Publisher<roboclaw_ros2::msg::RoboclawMotorVelocity>::SharedPtr motor_pub;
        

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub;
        rclcpp::Subscription<roboclaw_ros2::msg::RoboclawEncoderSteps>::SharedPtr encoder_sub;

        int last_steps_1;
        int last_steps_2;

        double last_x;
        double last_y;
        double last_theta;

        double base_width;
        double steps_per_meter;

        bool swap_motors;
        bool invert_motor_1;
        bool invert_motor_2;
        bool use_runge_kutta;

        double var_pos_x;
        double var_pos_y;
        double var_theta_z;

        std::string odom_tf_name;
        std::string base_tf_name;

        void twist_callback(const geometry_msgs::msg::Twist &msg);

        void encoder_callback(const roboclaw_ros2::msg::RoboclawEncoderSteps &msg);
        void encoder_callback_runge_kutta(const roboclaw_ros2::msg::RoboclawEncoderSteps &msg); 
    };

}

#endif //PROJECT_DIFFDRIVE_ROSCORE_H
