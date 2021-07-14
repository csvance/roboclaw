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
#include <ros/console.h>

namespace roboclaw {

    roboclaw_roscore::roboclaw_roscore(ros::NodeHandle nh, ros::NodeHandle nh_private) {

        std::string serial_port;
        int baudrate;
        int num_roboclaws;

        this->nh = nh;
        this->nh_private = nh_private;

        if(!nh_private.getParam("serial_port", serial_port))
            throw std::runtime_error("Must specify serial port");

        if(!nh_private.getParam("baudrate", baudrate))
            baudrate = (int) driver::DEFAULT_BAUDRATE;
        if(!nh_private.getParam("roboclaws", num_roboclaws))
            num_roboclaws = 1;

        roboclaw_mapping = std::map<int, unsigned char>();
        error_blocking = false;

        // Create address map
        if (num_roboclaws > 1) {

            for (int r = 0; r < num_roboclaws; r++)
                roboclaw_mapping.insert(std::pair<int, unsigned char>(r, driver::BASE_ADDRESS + r));

        } else {
            num_roboclaws = 1;

            roboclaw_mapping.insert(std::pair<int, unsigned char>(0, driver::BASE_ADDRESS));
        }

        roboclaw = new driver(serial_port, baudrate);

        for (int r = 0; r < roboclaw_mapping.size(); r++)
            roboclaw->reset_encoders(roboclaw_mapping[r]);

        encoder_pub = nh.advertise<roboclaw::RoboclawEncoderSteps>(std::string("motor_enc"), 10);
        input_voltage_pub = nh.advertise<roboclaw::RoboclawInputVoltageMessage>(std::string("roboclaw_input_voltage"), 10);
        velocity_sub = nh.subscribe(std::string("motor_cmd_vel"), 10, &roboclaw_roscore::velocity_callback, this);
    }

    roboclaw_roscore::~roboclaw_roscore() {
        for (int r = 0; r < roboclaw_mapping.size(); r++)
            roboclaw->set_duty(roboclaw_mapping[r], std::pair<int, int>(0, 0));
    }

    void roboclaw_roscore::velocity_callback(const roboclaw::RoboclawMotorVelocity &msg) {
        last_message = ros::Time::now();
        motor_1_vel_cmd = msg.mot1_vel_sps;
        motor_2_vel_cmd = msg.mot2_vel_sps;

        try {
            if (error_blocking == true) {
                // Set zero duty if blocking is detected
                try {
                        roboclaw->set_velocity(roboclaw_mapping[msg.index], std::pair<int, int>(0, 0));
                } catch(roboclaw::crc_exception &e){
                        ROS_ERROR("RoboClaw CRC error setting duty cycle!");
                } catch(timeout_exception &e) {
                        ROS_ERROR("RoboClaw timout during setting duty cycle!");
                }
                try {
                        roboclaw->set_duty(roboclaw_mapping[msg.index], std::pair<int, int>(0, 0));
                } catch(roboclaw::crc_exception &e){
                        ROS_ERROR("RoboClaw CRC error setting duty cycle!");
                } catch(timeout_exception &e) {
                        ROS_ERROR("RoboClaw timout during setting duty cycle!");
                }
                ROS_DEBUG("RoboClaw error while executing velocity command. Roboclaw will not respond");
                return;
            }
            roboclaw->set_velocity(roboclaw_mapping[msg.index], std::pair<int, int>(motor_1_vel_cmd, motor_2_vel_cmd));
        } catch(roboclaw::crc_exception &e){
            ROS_ERROR("RoboClaw CRC error during set velocity!");
        } catch(timeout_exception &e){
            ROS_ERROR("RoboClaw timout during set velocity!");
        }
    }

    void roboclaw_roscore::run() {
        error_it_count = 0;
        old_err = 0;
        last_message = ros::Time::now();
        loop_rate = 50;

        int same_err_count = 0;
        int speed_error_count = 0;
        double speed_error_factor = 0.85;

        ros::Rate update_rate(loop_rate);

        while (ros::ok()) {

            ros::spinOnce();
            update_rate.sleep();

            // Publish input voltage
            for (int r = 0; r < roboclaw_mapping.size(); r++) {
                int input_v = 0;
                try {
                    input_v = roboclaw->get_voltage(roboclaw_mapping[r]);

                } catch(roboclaw::crc_exception &e){
                    ROS_ERROR("RoboClaw CRC error during getting input voltage!");
                    continue;
                } catch(timeout_exception &e){
                    ROS_ERROR("RoboClaw timout during getting inputvoltage!");
                    continue;
                }

                RoboclawInputVoltageMessage input_voltage_m;
                input_voltage_m.index = r;
                input_voltage_m.input_voltage = (float) input_v/10;

                input_voltage_pub.publish(input_voltage_m);
            }

            // Publish encoders
            for (int r = 0; r < roboclaw_mapping.size(); r++) {
                std::pair<int, int> encs = std::pair<int, int>(0, 0);
                try {
                    encs = roboclaw->get_encoders(roboclaw_mapping[r]);
                } catch(roboclaw::crc_exception &e){
                    ROS_ERROR("RoboClaw CRC error during getting encoders!");
                    continue;
                } catch(timeout_exception &e){
                    ROS_ERROR("RoboClaw timout during getting encoders!");
                    continue;
                }

                RoboclawEncoderSteps enc_steps;
                enc_steps.index = r;
                enc_steps.mot1_enc_steps = encs.first;
                enc_steps.mot2_enc_steps = encs.second;
                encoder_pub.publish(enc_steps);
            }


            // Print errors and stop motors if errors occur
            for (int r = 0; r < roboclaw_mapping.size(); r++) {
                // If previous error was present, count to 10 seconds and continue blocking motors.
                // ToDo: Adapt for multiple RoboClaws. Only 1 in robot so currently no use and no possibility to test


                if ((error_blocking == true) && (error_it_count < loop_rate*10)) {
                    error_it_count++;
                } else {
                    error_blocking = false;

                    // If blocking period has expired, old error must be cleared to make sure that new errors are displayed again
                    old_err = 0;
                    same_err_count = 0;
                    error_it_count = 0;
                }

                int err;
                try {
                   err = roboclaw->read_err(roboclaw_mapping[r]);

                } catch(roboclaw::crc_exception &e){
                    ROS_ERROR("RoboClaw CRC error during getting error!");
                    continue;
                } catch(timeout_exception &e){
                    ROS_ERROR("RoboClaw timout during getting error!");
                    continue;
                }

                if (err != 0) {
                    // Error found. First block velocity publisher
                    error_blocking = true;

                    // Set velocity and duty to 0. Only setting velocity to zero results in holding torque which is not desired.
                    try {
                        roboclaw->set_velocity(roboclaw_mapping[r], std::pair<int, int>(0, 0));
                    } catch(roboclaw::crc_exception &e){
                        ROS_ERROR("RoboClaw CRC error setting duty cycle!");
                    } catch(timeout_exception &e) {
                        ROS_ERROR("RoboClaw timout during setting duty cycle!");
                    }

                    try {
                        roboclaw->set_duty(roboclaw_mapping[r], std::pair<int, int>(0, 0));
                    } catch(roboclaw::crc_exception &e){
                        ROS_ERROR("RoboClaw CRC error setting duty cycle!");
                    } catch(timeout_exception &e) {
                        ROS_ERROR("RoboClaw timout during setting duty cycle!");
                    }

                    // Start counting error iterations for blocking. Every new error blocks motors 10 seconds
                    error_it_count = 0;

                    // Start counting duplicate errors to not spam the terminal
                    if (old_err == err){
                        same_err_count++;
                    } else {
                        same_err_count = 0;
                    }
                    old_err = err;

                    // Display error when detected and for every second afterwards
                    if ( (same_err_count % loop_rate) == 0){
                        if (err == 65536) {
                            ROS_ERROR_STREAM("RoboClaw error. Velocity commands will be blocked for 10 seconds. Error number 10000 = M1 over current!");
                        } else if (err == 131072) {
                            ROS_ERROR_STREAM("RoboClaw error. Velocity commands will be blocked for 10 seconds. Error number 20000 = M2 over current!");
                        } else {
                            ROS_ERROR_STREAM("RoboClaw error. Velocity commands will be blocked for 10 seconds. Error number (see RoboClaw manual page 73): " << std::hex << err);
                        }
                    }
                }
            }

            // Detect large speed differences in desired velocity and actual velocity
            for (int r = 0; r < roboclaw_mapping.size(); r++) {
                // Get instantaneous velocity (1/300th of a second)
                std::pair<int, int> velocity = std::pair<int, int>(0, 0);
                try {
                    velocity = roboclaw->get_velocity(roboclaw_mapping[r]);
                } catch(roboclaw::crc_exception &e){
                    ROS_ERROR("RoboClaw CRC error during getting velocity");
                    continue;
                } catch(timeout_exception &e){
                    ROS_ERROR("RoboClaw timout during getting velocity!");
                    continue;
                }
                int motor_1_speed = abs(velocity.first);
                int motor_2_speed = abs(velocity.second);

                if (( motor_1_speed < abs(  speed_error_factor * motor_1_vel_cmd )) || (motor_2_speed  < abs( speed_error_factor * motor_2_vel_cmd ))){
                    // if 1 of 2 absolute motor speeds is more than 1 - speed_error_factor different
                    speed_error_count++;
                    if (speed_error_count >= loop_rate*2){
                        // Difference must be present for more than 2 seconds

                        error_blocking = true;
                        error_it_count = 0;

                        if (speed_error_count % loop_rate == 0){
                            // Execute once per second and at beginning
                            ROS_ERROR("Difference in cmd vel and actual wheel vel has been more than 15 percent for 2 seconds! Blocking velocity commands for 10 seconds.");

                            // Set velocity and duty to 0. Only setting velocity to zero results in holding torque which is not desired.
                            try {
                                roboclaw->set_velocity(roboclaw_mapping[r], std::pair<int, int>(0, 0));
                            } catch(roboclaw::crc_exception &e){
                                ROS_ERROR("RoboClaw CRC error setting duty cycle!");
                            } catch(timeout_exception &e) {
                                ROS_ERROR("RoboClaw timout during setting duty cycle!");
                            }

                            try {
                                roboclaw->set_duty(roboclaw_mapping[r], std::pair<int, int>(0, 0));
                            } catch(roboclaw::crc_exception &e){
                                ROS_ERROR("RoboClaw CRC error setting duty cycle!");
                            } catch(timeout_exception &e) {
                                ROS_ERROR("RoboClaw timout during setting duty cycle!");
                            }
                        }
                    }
                } else {
                    speed_error_count = 0;
                }
                if (speed_error_count > loop_rate*10) {
                    // If error is generated again (i.e. by navigation), this unblocks execution after 10 seconds
                    ROS_INFO("Resetting speed error count. Trying to move again");
                    speed_error_count = 0;
                    error_blocking = false;
                }
            }

            if (ros::Time::now() - last_message > ros::Duration(5)) {
                for (int r = 0; r < roboclaw_mapping.size(); r++) {
                    try {
                        roboclaw->set_duty(roboclaw_mapping[r], std::pair<int, int>(0, 0));
                    } catch(roboclaw::crc_exception &e){
                        ROS_ERROR("RoboClaw CRC error setting duty cycle!");
                    } catch(timeout_exception &e) {
                        ROS_ERROR("RoboClaw timout during setting duty cycle!");
                    }
                }
            }
        }
    }
}
