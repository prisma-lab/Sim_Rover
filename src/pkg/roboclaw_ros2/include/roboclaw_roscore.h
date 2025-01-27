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

#ifndef PROJECT_ROBOCLAW_ROSCORE_H
#define PROJECT_ROBOCLAW_ROSCORE_H
#pragma once
//#include "ros/ros.h"
#include "rclcpp/rclcpp.hpp"
#include <boost/shared_ptr.hpp>
#include <chrono>

// #include "ros/package.h"

#include "roboclaw_driver.h"

#include "roboclaw_ros2/msg/roboclaw_encoder_steps.hpp"
#include "roboclaw_ros2/msg/roboclaw_motor_velocity.hpp"

namespace roboclaw {

    class roboclaw_roscore : public rclcpp::Node {
    public:
        //rclcpp::executors::MultiThreadedExecutor executor;
        //roboclaw_roscore(std::shared_ptr<rclcpp::Node>  nh, std::shared_ptr<rclcpp::Node>  nh_private);
        roboclaw_roscore();
        ~roboclaw_roscore();

        //void run(std::shared_ptr<roboclaw::roboclaw_roscore> rover_node);
        

    private:

        driver *roboclaw;

        std::map<int, unsigned char> roboclaw_mapping;

        //ros::NodeHandle nh;
        //ros::NodeHandle nh_private;
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<roboclaw_ros2::msg::RoboclawEncoderSteps>::SharedPtr encoder_pub;

        rclcpp::Subscription<roboclaw_ros2::msg::RoboclawMotorVelocity>::SharedPtr velocity_sub;


        rclcpp::Time last_message;

        void velocity_callback(const roboclaw_ros2::msg::RoboclawMotorVelocity &msg);
        void run_callback();
    };


}

#endif //PROJECT_ROBOCLAW_ROSCORE_H
