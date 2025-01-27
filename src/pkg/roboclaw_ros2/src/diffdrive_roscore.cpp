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

#include <cmath>
#include <iostream>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "diffdrive_roscore.h"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/transform_datatypes.h" 
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


namespace roboclaw {

    diffdrive_roscore::diffdrive_roscore() : Node("diffdrive_node") {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Inizio ciclo");

        Node::declare_parameter("use_runge_kutta",rclcpp::PARAMETER_BOOL);
        Node::declare_parameter("base_width", rclcpp::PARAMETER_DOUBLE);
        Node::declare_parameter("steps_per_meter", rclcpp::PARAMETER_DOUBLE);
        Node::declare_parameter("swap_motors", rclcpp::PARAMETER_BOOL);
        Node::declare_parameter("invert_motor_1", rclcpp::PARAMETER_BOOL);
        Node::declare_parameter("invert_motor_2", rclcpp::PARAMETER_BOOL);
        Node::declare_parameter("var_pos_x", rclcpp::PARAMETER_DOUBLE);
        Node::declare_parameter("var_pos_y", rclcpp::PARAMETER_DOUBLE);
        Node::declare_parameter("var_pos_z", rclcpp::PARAMETER_DOUBLE);
        Node::declare_parameter("odom_tf_name",rclcpp::PARAMETER_STRING);
        Node::declare_parameter("base_tf_name",rclcpp::PARAMETER_STRING);


        //odom_pub = nh.advertise<nav_msgs::Odometry>(std::string("odom"), 10);
        odom_pub = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        //motor_pub = nh.advertise<roboclaw::RoboclawMotorVelocity>(std::string("motor_cmd_vel"), 10);
        motor_pub = create_publisher<roboclaw_ros2::msg::RoboclawMotorVelocity>("motor_cmd_vel", 10);
        
        if(Node::get_parameter("use_runge_kutta", use_runge_kutta)){
            use_runge_kutta = false;
        }

        if(use_runge_kutta)
        {
            //encoder_sub = nh.subscribe(std::string("motor_enc"), 10, &diffdrive_roscore::encoder_callback_runge_kutta, this); //Runge kutta 2 order integration
            encoder_sub = create_subscription<roboclaw_ros2::msg::RoboclawEncoderSteps>("motor_enc", 10,std::bind(&diffdrive_roscore::encoder_callback_runge_kutta, this,std::placeholders::_1));
            //ROS_INFO("Odometry with Runge-Kutta integration method");
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Odometry with Runge-Kutta integration method");
        }
        else
        {
            //encoder_sub = nh.subscribe(std::string("motor_enc"), 10, &diffdrive_roscore::encoder_callback, this); //Forward euler integration
            encoder_sub = create_subscription<roboclaw_ros2::msg::RoboclawEncoderSteps>("motor_enc", 10,std::bind(&diffdrive_roscore::encoder_callback, this,std::placeholders::_1));
            //ROS_INFO("Odometry with Forward Euler integration method");
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Odometry with Forward Euler integration method");
        }
        
        //twist_sub = nh.subscribe(std::msg:string("cmd_vel"), 10, &diffdrive_roscore::twist_callback, this);
        twist_sub = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,std::bind(&diffdrive_roscore::twist_callback, this,std::placeholders::_1));
        
        last_x = 0.0;
        last_y = 0.0;
        last_theta = 0.0;
        last_steps_1 = 0;
        last_steps_2 = 0;

        if(!Node::get_parameter("base_width", base_width)){
            throw std::runtime_error("Must specify base_width!");
        }
        if(!Node::get_parameter("steps_per_meter", steps_per_meter)) {
            throw std::runtime_error("Must specify steps_per_meter!");
        }

        if(!Node::get_parameter("swap_motors", swap_motors))
            swap_motors = true;
        if(!Node::get_parameter("invert_motor_1", invert_motor_1))
            invert_motor_1 = false;
        if(!Node::get_parameter("invert_motor_2", invert_motor_2))
            invert_motor_2 = false;

        if(!Node::get_parameter("var_pos_x", var_pos_x)){
            var_pos_x = 0.01;
        }
        if(!Node::get_parameter("var_pos_y", var_pos_y)){
            var_pos_y = 0.01;
        }
        if(!Node::get_parameter("var_theta_z", var_theta_z)){
            var_theta_z = 0.01;
        }
        if(!Node::get_parameter("odom_tf_name", odom_tf_name)){
            //odom_tf_name = "odom";
            throw std::runtime_error("Must specify odom tf");
        }
        if(!Node::get_parameter("base_tf_name", base_tf_name)){
            //base_tf_name = "base_link";
            throw std::runtime_error("Must specify base link tf");
        }

    }

    void diffdrive_roscore::twist_callback(const geometry_msgs::msg::Twist &msg) {

        roboclaw_ros2::msg::RoboclawMotorVelocity motor_vel;
        motor_vel.index = 0;
        motor_vel.mot1_vel_sps = 0;
        motor_vel.mot2_vel_sps = 0;

        // Linear
        motor_vel.mot1_vel_sps += (int) (steps_per_meter * msg.linear.x);
        motor_vel.mot2_vel_sps += (int) (steps_per_meter * msg.linear.x);

        if(msg.linear.y > 0){
            motor_vel.mot2_vel_sps += (int) (steps_per_meter * msg.linear.y);
        }else if(msg.linear.y < 0){
            motor_vel.mot1_vel_sps += (int) (steps_per_meter * msg.linear.y);
        }

        // Angular
        motor_vel.mot1_vel_sps += (int) -(steps_per_meter * msg.angular.z * base_width/2);
        motor_vel.mot2_vel_sps += (int) (steps_per_meter * msg.angular.z * base_width/2);

        if (invert_motor_1)
            motor_vel.mot1_vel_sps = -motor_vel.mot1_vel_sps;

        if (invert_motor_2)
            motor_vel.mot2_vel_sps = -motor_vel.mot2_vel_sps;

        if (swap_motors){
            int tmp = motor_vel.mot1_vel_sps;
            motor_vel.mot1_vel_sps = motor_vel.mot2_vel_sps;
            motor_vel.mot2_vel_sps = tmp;
        }

        motor_pub->publish(motor_vel);
    }

    void diffdrive_roscore::encoder_callback(const roboclaw_ros2::msg::RoboclawEncoderSteps &msg) {

        // static tf::TransformBroadcaster br;
        std::unique_ptr<tf2_ros::TransformBroadcaster> br;
        br = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        int delta_1 = msg.mot1_enc_steps - last_steps_1;
        int delta_2 = msg.mot2_enc_steps - last_steps_2;
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Motor1 enc: %d\n",delta_1);

        last_steps_1 = msg.mot1_enc_steps;
        last_steps_2 = msg.mot2_enc_steps;

        if (invert_motor_1)
            delta_1 = -delta_1;

        if (invert_motor_2)
            delta_1 = -delta_2;

        if (swap_motors){
            int tmp = delta_1;
            delta_1 = delta_2;
            delta_2 = tmp;
        }

        double u_w = ((delta_1 + delta_2) / steps_per_meter) / 2.0;
        double u_p = ((delta_2 - delta_1) / steps_per_meter);

        double delta_x = u_w * cos(last_theta);
        double delta_y = u_w * sin(last_theta);
        double delta_theta = u_p / base_width;

        double cur_x = last_x + delta_x;
        double cur_y = last_y + delta_y;
        double cur_theta = last_theta + delta_theta;

        nav_msgs::msg::Odometry odom;

        odom.header.frame_id = odom_tf_name;
        odom.child_frame_id = base_tf_name;
        

        // Time
        //odom.header.stamp = ros::Time::now();
        rclcpp::Clock steady_clock = rclcpp::Clock(RCL_STEADY_TIME);
        odom.header.stamp = steady_clock.now();

        // Position
        odom.pose.pose.position.x = cur_x;
        odom.pose.pose.position.y = cur_y;

        // Velocity
        odom.twist.twist.linear.x = cur_x - last_x;
        odom.twist.twist.linear.y = cur_y - last_y;
        odom.twist.twist.angular.z = cur_theta - last_theta;

        //tf::Quaternion quaternion = tf::createQuaternionFromRPY(0.0, 0.0, cur_theta);
        double roll = 0.0;   // Angolo di roll in radianti
        double pitch = 0.0;  // Angolo di pitch in radianti
        double yaw = cur_theta;  // Angolo di yaw in radianti

        tf2::Quaternion quaternion;
        quaternion.setRPY(roll, pitch, yaw);
        odom.pose.pose.orientation.w = quaternion.w();
        odom.pose.pose.orientation.x = quaternion.x();
        odom.pose.pose.orientation.y = quaternion.y();
        odom.pose.pose.orientation.z = quaternion.z();
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Quaternione settato");

        // Pos_x Variance
        odom.pose.covariance[0] = var_pos_x;

        // Pos_y Variance
        odom.pose.covariance[7] = var_pos_y;

        // Theta_z Variance
        odom.pose.covariance[35] = var_theta_z;
        tf2::Transform transform;
        transform.setOrigin(tf2::Vector3(last_x, last_y, 0.0));
        transform.setRotation(quaternion);
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Rotazione settata ");

        //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), odom_tf_name, base_tf_name));
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = rclcpp::Clock().now();
        transformStamped.header.frame_id = odom_tf_name;
        transformStamped.child_frame_id = base_tf_name;
        transformStamped.transform.translation.x = transform.getOrigin().getX();
        transformStamped.transform.translation.y = transform.getOrigin().getY();
        transformStamped.transform.translation.z = transform.getOrigin().getZ();
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"TF lineare settata ");

        quaternion.setRPY(0, 0, cur_theta);
        transformStamped.transform.rotation.x = quaternion.x();
        transformStamped.transform.rotation.y = quaternion.y();
        transformStamped.transform.rotation.z = quaternion.z();
        transformStamped.transform.rotation.w = quaternion.w();
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"TF angolare settata ");

        br->sendTransform(transformStamped);
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Pulish odom");
        odom_pub->publish(odom);

        last_x = cur_x;
        last_y = cur_y;
        last_theta = cur_theta;

    }

    void diffdrive_roscore::encoder_callback_runge_kutta(const roboclaw_ros2::msg::RoboclawEncoderSteps &msg)
    {
        //static tf::TransformBroadcaster br;
        std::unique_ptr<tf2_ros::TransformBroadcaster> br;

        int delta_1 = msg.mot1_enc_steps - last_steps_1;
        int delta_2 = msg.mot2_enc_steps - last_steps_2;

        last_steps_1 = msg.mot1_enc_steps;
        last_steps_2 = msg.mot2_enc_steps;

        if (invert_motor_1)
            delta_1 = -delta_1;

        if (invert_motor_2)
            delta_1 = -delta_2;

        if (swap_motors){
            int tmp = delta_1;
            delta_1 = delta_2;
            delta_2 = tmp;
        }

        double u_w = ((delta_1 + delta_2) / steps_per_meter) / 2.0;
        double u_p = ((delta_2 - delta_1) / steps_per_meter);

        double delta_theta = u_p / base_width;
        double delta_x = u_w * cos(last_theta + delta_theta/2.0);
        double delta_y = u_w * sin(last_theta + delta_theta/2.0);

        double cur_x = last_x + delta_x;
        double cur_y = last_y + delta_y;
        double cur_theta = last_theta + delta_theta;

        nav_msgs::msg::Odometry odom;

        odom.header.frame_id = odom_tf_name;
        odom.child_frame_id = base_tf_name;

        // Time
        rclcpp::Clock steady_clock = rclcpp::Clock(RCL_STEADY_TIME);
        odom.header.stamp = steady_clock.now();

        // Position
        odom.pose.pose.position.x = cur_x;
        odom.pose.pose.position.y = cur_y;

        // Velocity
        odom.twist.twist.linear.x = cur_x - last_x;
        odom.twist.twist.linear.y = cur_y - last_y;
        odom.twist.twist.angular.z = cur_theta - last_theta;

        //tf::Quaternion quaternion = tf::createQuaternionFromRPY(0.0, 0.0, cur_theta);
        double roll = 0.0;   // Angolo di roll in radianti
        double pitch = 0.0;  // Angolo di pitch in radianti
        double yaw = cur_theta;  // Angolo di yaw in radianti
        tf2::Quaternion quaternion;
        quaternion.setRPY(roll, pitch, yaw);
        odom.pose.pose.orientation.w = quaternion.w();
        odom.pose.pose.orientation.x = quaternion.x();
        odom.pose.pose.orientation.y = quaternion.y();
        odom.pose.pose.orientation.z = quaternion.z();

        // Pos_x Variance
        odom.pose.covariance[0] = var_pos_x;

        // Pos_y Variance
        odom.pose.covariance[7] = var_pos_y;

        // Theta_z Variance
        odom.pose.covariance[35] = var_theta_z;

        tf2::Transform transform;
        transform.setOrigin(tf2::Vector3(last_x, last_y, 0.0));
        transform.setRotation(quaternion);
        //br->sendTransform(tf2::StampedTransform(transform, rclcpp::Time::now(), odom_tf_name, base_tf_name));
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = rclcpp::Clock().now();
        transformStamped.header.frame_id = odom_tf_name;
        transformStamped.child_frame_id = base_tf_name;
        transformStamped.transform.translation.x = transform.getOrigin().getX();
        transformStamped.transform.translation.y = transform.getOrigin().getY();
        transformStamped.transform.translation.z = transform.getOrigin().getZ();

        quaternion.setRPY(0, 0, cur_theta);
        transformStamped.transform.rotation.x = quaternion.x();
        transformStamped.transform.rotation.y = quaternion.y();
        transformStamped.transform.rotation.z = quaternion.z();
        transformStamped.transform.rotation.w = quaternion.w();

        br->sendTransform(transformStamped);

        odom_pub->publish(odom);

        last_x = cur_x;
        last_y = cur_y;
        last_theta = cur_theta;
    }


}
