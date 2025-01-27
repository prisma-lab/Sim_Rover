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

#include "roboclaw_roscore.h"

#include <map>
#include <string>

#include <iostream>

namespace roboclaw {
    using namespace std::chrono_literals;

   // roboclaw_roscore::roboclaw_roscore(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<rclcpp::Node>  nh_private) {
    roboclaw_roscore::roboclaw_roscore() :Node("roboclaw_node") {

        std::string serial_port;
        int baudrate;
        int num_roboclaws;

        Node::declare_parameter("serial_port", rclcpp::PARAMETER_STRING);
        Node::declare_parameter("baudrate", rclcpp::PARAMETER_INTEGER);
        Node::declare_parameter("roboclaws", rclcpp::PARAMETER_INTEGER);
 
        if(!(Node::get_parameter("serial_port", serial_port)))
           throw std::runtime_error("Must specify serial port");
        
        if(!(Node::get_parameter("baudrate", baudrate)))
         //  baudrate = (int) driver::DEFAULT_BAUDRATE;
        throw std::runtime_error("Must specify boud rate");

        if(!(Node::get_parameter("roboclaws", num_roboclaws)))
        //     num_roboclaws = 1;
        throw std::runtime_error("Must specify roboclas number");

        roboclaw_mapping = std::map<int, unsigned char>();

        // Create address map
        if (num_roboclaws > 1) {

            for (int r = 0; r < num_roboclaws; r++)
                roboclaw_mapping.insert(std::pair<int, unsigned char>(r, driver::BASE_ADDRESS + r));

        } else {
            num_roboclaws = 1;

            roboclaw_mapping.insert(std::pair<int, unsigned char>(0, driver::BASE_ADDRESS));
        }
        roboclaw = new driver(serial_port, baudrate);
        for (int r = 0; r < roboclaw_mapping.size(); r++){
         //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"%d\n",roboclaw_mapping[r]);
            roboclaw->reset_encoders(roboclaw_mapping[r]);
        }
        timer_= timer_ = this->create_wall_timer(100ms, std::bind(&roboclaw_roscore::run_callback, this));

        //encoder_pub = nh.advertise<roboclaw::RoboclawEncoderSteps>(std::string("motor_enc"), 10);
        encoder_pub = create_publisher<roboclaw_ros2::msg::RoboclawEncoderSteps>("motor_enc", 10);
  
        //velocity_sub = nh.subscribe(std::string("motor_cmd_vel"), 10, &roboclaw_roscore::velocity_callback, this);
        velocity_sub = Node::create_subscription<roboclaw_ros2::msg::RoboclawMotorVelocity>("motor_cmd_vel", 10,std::bind(&roboclaw_roscore::velocity_callback, this,std::placeholders::_1));
 
    }

    roboclaw_roscore::~roboclaw_roscore() {
        for (int r = 0; r < roboclaw_mapping.size(); r++)
            roboclaw->set_duty(roboclaw_mapping[r], std::pair<int, int>(0, 0));
    }

    void roboclaw_roscore::velocity_callback(const roboclaw_ros2::msg::RoboclawMotorVelocity &msg) {
       // last_message = ros::Time.now();

        rclcpp::Clock steady_clock = rclcpp::Clock(RCL_STEADY_TIME);

        last_message = steady_clock.now();
   

        try {
            roboclaw->set_velocity(roboclaw_mapping[msg.index], std::pair<int, int>(msg.mot1_vel_sps, msg.mot2_vel_sps));
        } catch(roboclaw::crc_exception &e){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"RoboClaw CRC error during set velocity!");
        } catch(timeout_exception &e){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"RoboClaw timout during set velocity!");
        }

    }

    /*void roboclaw_roscore::run(std::shared_ptr<roboclaw::roboclaw_roscore> rover_node ) {

        //last_message = ros::Time.now();
        rclcpp::Clock steady_clock = rclcpp::Clock(RCL_STEADY_TIME);
        last_message = steady_clock.now();
        rclcpp::Rate update_rate(10);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Inizio ciclo");
        while (rclcpp::ok()) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"sto nel ciclo");
            //ros::spinOnce();
            //rclcpp::spin(rover_node);
            //executor.spin();
            update_rate.sleep();

            // Publish encoders
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Publish encoders");
            for (int r = 0; r < roboclaw_mapping.size(); r++) {
                std::pair<int, int> encs = std::pair<int, int>(0, 0);
                try {
                    encs = roboclaw->get_encoders(roboclaw_mapping[r]);
                } catch(roboclaw::crc_exception &e){
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"RoboClaw CRC error during getting encoders!");
                    continue;
                } catch(timeout_exception &e){
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"RoboClaw timout during getting encoders!");
                    continue;
                }

                //RoboclawEncoderSteps enc_steps;
                roboclaw_ros2::msg::RoboclawEncoderSteps enc_steps;
                enc_steps.index = r;
                enc_steps.mot1_enc_steps = enc  : Node("turtle_tf2_frame_publisher")s.first;
                enc_steps.mot2_enc_steps = encs.second;
                encoder_pub->publish(enc_steps);

            }

            if (steady_clock.now() - last_message > rclcpp::Duration(std::chrono::seconds(5))) {
                for (int r = 0; r < roboclaw_mapping.size(); r++) {
                    try {
                        roboclaw->set_duty(roboclaw_mapping[r], std::pair<int, int>(0, 0));
                    } catch(roboclaw::crc_exception &e){
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"RoboClaw CRC error setting duty cyrcle!");
                    } catch(timeout_exception &e) {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"RoboClaw timout during setting duty cycle!");
                    }
                }
            }

        }
    }*/
    void roboclaw_roscore::run_callback() {

            // Publish encoders
           // RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Publish encoders");
            for (int r = 0; r < roboclaw_mapping.size(); r++) {
                std::pair<int, int> encs = std::pair<int, int>(0, 0);
                try {
                    encs = roboclaw->get_encoders(roboclaw_mapping[r]);
                } catch(roboclaw::crc_exception &e){
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"RoboClaw CRC error during getting encoders!");
                    continue;
                } catch(timeout_exception &e){
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"RoboClaw timout during getting encoders!");
                    continue;
                }

                //RoboclawEncoderSteps enc_steps;
                roboclaw_ros2::msg::RoboclawEncoderSteps enc_steps;
                enc_steps.index = r;
                enc_steps.mot1_enc_steps = encs.first;
                enc_steps.mot2_enc_steps = encs.second;
                encoder_pub->publish(enc_steps);

            }

        
    }

}
