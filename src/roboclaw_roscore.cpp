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

        if(!nh_private.getParam("speed_error_factor", speed_error_factor))
            speed_error_factor = 0.85;

        if(!nh_private.getParam("error_loop_rate", loop_rate))
            loop_rate = 50;

        if(!nh_private.getParam("error_blocking_time", blocking_time))
            blocking_time = 10.0;

        if(!nh_private.getParam("speed_diff_time", speed_diff_time))
            speed_diff_time = 2.0;

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

        motor_1_vel_cmd = 0;
        motor_2_vel_cmd = 0;
    }

    roboclaw_roscore::~roboclaw_roscore() {
        for (int r = 0; r < roboclaw_mapping.size(); r++)
            roboclaw->set_duty(roboclaw_mapping[r], std::pair<int, int>(0, 0));
    }

    void roboclaw_roscore::set_velocity_duty_zero(int index) {
        // Sets velocity and duty to 0. Only setting velocity to zero results in holding torque which is not desired if an error is detected.
        try {
            roboclaw->set_velocity(roboclaw_mapping[index], std::pair<int, int>(0, 0));
        } catch(roboclaw::crc_exception &e){
            ROS_ERROR("RoboClaw CRC error setting duty cycle!");
        } catch(timeout_exception &e) {
            ROS_ERROR("RoboClaw timout during setting duty cycle!");
        }
        try {
            roboclaw->set_duty(roboclaw_mapping[index], std::pair<int, int>(0, 0));
        } catch(roboclaw::crc_exception &e){
            ROS_ERROR("RoboClaw CRC error setting duty cycle!");
        } catch(timeout_exception &e) {
            ROS_ERROR("RoboClaw timout during setting duty cycle!");
        }
    }

    void roboclaw_roscore::velocity_callback(const roboclaw::RoboclawMotorVelocity &msg) {
        if (error_blocking) {
                set_velocity_duty_zero(msg.index);
                ROS_DEBUG("RoboClaw error while executing velocity command. Roboclaw will not respond");
                return;
            }

        last_message = ros::Time::now();
        motor_1_vel_cmd = msg.mot1_vel_sps;
        motor_2_vel_cmd = msg.mot2_vel_sps;

        try {
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

        int same_err_count = 0;
        int speed_error_count = 0;

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
                // If previous error was present, count to blocking time seconds and continue blocking motors.
                // ToDo: Adapt for multiple RoboClaws. Only 1 in robot so currently no use and no possibility to test

                if (error_blocking && (error_it_count < loop_rate*blocking_time)) {
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

                    set_velocity_duty_zero(r);

                    // Start counting error iterations for blocking. Every new error blocks motors blocking_time seconds
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
                        switch (err) {
                            case 1:
                                ROS_ERROR("RoboClaw error. Velocity commands will be blocked for %.1f seconds. Error number 0x000001 = E-stop!",blocking_time);
                                break;
                            case 2:
                                ROS_ERROR("RoboClaw error. Velocity commands will be blocked for %.1f seconds. Error number 0x000002 = Temperature Error",blocking_time);
                                break;
                            case 4:
                                ROS_ERROR("RoboClaw error. Velocity commands will be blocked for %.1f seconds. Error number 0x000004 = Temperature 2 Error",blocking_time);
                                break;
                            case 8:
                                ROS_ERROR("RoboClaw error. Velocity commands will be blocked for %.1f seconds. Error number 0x000008 = Main Voltage High Error!",blocking_time);
                                break;
                            case 16:
                                ROS_ERROR("RoboClaw error. Velocity commands will be blocked for %.1f seconds. Error number 0x000010 = Logic Voltage High Error!",blocking_time);
                                break;
                            case 32:
                                ROS_ERROR("RoboClaw error. Velocity commands will be blocked for %.1f seconds. Error number 0x000020 = Logic Voltage Low Error!",blocking_time);
                                break;
                            case 64:
                                ROS_ERROR("RoboClaw error. Velocity commands will be blocked for %.1f seconds. Error number 0x000040 = M1 Driver Fault Error",blocking_time);
                                break;
                            case 128:
                                ROS_ERROR("RoboClaw error. Velocity commands will be blocked for %.1f seconds. Error number 0x000080 = M2 Driver Fault Error!",blocking_time);
                                break;
                            case 256:
                                ROS_ERROR("RoboClaw error. Velocity commands will be blocked for %.1f seconds. Error number 0x000100 = M1 Speed Error",blocking_time);
                                break;
                            case 512:
                                ROS_ERROR("RoboClaw error. Velocity commands will be blocked for %.1f seconds. Error number 0x000200 = M2 Speed Error",blocking_time);
                                break;
                            case 1024:
                                ROS_ERROR("RoboClaw error. Velocity commands will be blocked for %.1f seconds. Error number 0x000400 = M1 Position Error",blocking_time);
                                break;
                            case 2048:
                                ROS_ERROR("RoboClaw error. Velocity commands will be blocked for %.1f seconds. Error number 0x000800 = M2 Position Error",blocking_time);
                                break;
                            case 4096:
                                ROS_ERROR("RoboClaw error. Velocity commands will be blocked for %.1f seconds. Error number 0x001000 = M1 Current Error",blocking_time);
                                break;
                            case 8192:
                                ROS_ERROR("RoboClaw error. Velocity commands will be blocked for %.1f seconds. Error number 0x002000 = M2 Current Error",blocking_time);
                                break;
                            case 65536:
                                ROS_ERROR("RoboClaw error. Velocity commands will be blocked for %.1f seconds. Error number 0x010000 = M1 Over Current Warning",blocking_time);
                                break;
                            case 131072:
                                ROS_ERROR("RoboClaw error. Velocity commands will be blocked for %.1f seconds. Error number 0x020000 = M2 Over Current Warning",blocking_time);
                                break;
                            case 262144:
                                ROS_ERROR("RoboClaw error. Velocity commands will be blocked for %.1f seconds. Error number 0x040000 = Main Voltage High Warning",blocking_time);
                                break;
                            case 524288:
                                ROS_ERROR("RoboClaw error. Velocity commands will be blocked for %.1f seconds. Error number 0x080000 = Main Voltage Low Warning",blocking_time);
                                break;
                            case 1048576:
                                ROS_ERROR("RoboClaw error. Velocity commands will be blocked for %.1f seconds. Error number 0x100000 = Temperature Warning",blocking_time);
                                break;
                            case 2097152:
                                ROS_ERROR("RoboClaw error. Velocity commands will be blocked for %.1f seconds. Error number 0x200000 = Temperature 2 Warning",blocking_time);
                                break;
                            case 4194304:
                                ROS_ERROR("RoboClaw error. Velocity commands will be blocked for %.1f seconds. Error number 0x400000 = S4 signal triggered",blocking_time);
                                break;
                            case 8388608:
                                ROS_ERROR("RoboClaw error. Velocity commands will be blocked for %.1f seconds. Error number 0x800000 = S5 signal triggered",blocking_time);
                                break;
                            case 16777216:
                                ROS_ERROR("RoboClaw error. Velocity commands will be blocked for %.1f seconds. Error number 0x01000000 = Speed Error Limit Warning",blocking_time);
                                break;
                            case 33554432:
                                ROS_ERROR("RoboClaw error. Velocity commands will be blocked for %.1f seconds. Error number 0x02000000 = Position Error Limit Warning",blocking_time);
                                break;
                            default:
                                ROS_ERROR("RoboClaw error. Velocity commands will be blocked for %.1f seconds. Error number (see RoboClaw manual page 73): %#010x",blocking_time,err);
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

                int motor_1_criteria = (int) abs(speed_error_factor * motor_1_vel_cmd);
                int motor_2_criteria = (int) abs(speed_error_factor * motor_2_vel_cmd);

                if (( motor_1_speed < motor_1_criteria ) || (motor_2_speed  < motor_2_vel_cmd )){
                    // if 1 of 2 absolute motor speeds is more than 1 - speed_error_factor different
                    speed_error_count++;
                    if (speed_error_count >= loop_rate*speed_diff_time){
                        // Difference must be present for more than speed_diff_time seconds

                        error_blocking = true;
                        error_it_count = 0;

                        if (speed_error_count % loop_rate == 0){
                            // Execute once per second and at beginning

                            ROS_ERROR("Difference in cmd vel and actual wheel vel has been more than %.2f percent for %.1f seconds! Blocking velocity commands for %.1f seconds.",(1-speed_error_factor)*100,speed_diff_time,blocking_time);
                            ROS_ERROR("Motor 1 speed: %d // motor_1_vel_cmd %d // motor_1_criteria %d ",motor_1_speed,motor_1_vel_cmd,motor_1_criteria);
                            ROS_ERROR("Motor 2 speed: %d // motor_2_vel_cmd %d // motor_2_criteria %d ",motor_2_speed,motor_2_vel_cmd,motor_2_criteria);

                            set_velocity_duty_zero(r);
                        }
                    }
                } else {
                    speed_error_count = 0;
                }
                if (speed_error_count > loop_rate*blocking_time) {
                    // If error is generated continuously (i.e. by navigation), this unblocks execution after blocking_time seconds
                    ROS_INFO("Resetting speed error count. Trying to move again");
                    speed_error_count = 0;
                    error_blocking = false;
                }
            }

            if (ros::Time::now() - last_message > ros::Duration(0.25)) {
                for (int r = 0; r < roboclaw_mapping.size(); r++) {
                    set_velocity_duty_zero(r);

                    // Also set command velocity to zero to satisfy the speed difference checker
                    motor_1_vel_cmd = 0;
                    motor_2_vel_cmd = 0;

                }
            }
        }
    }
}
