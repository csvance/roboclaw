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

namespace roboclaw {

    roboclaw_roscore::roboclaw_roscore(ros::NodeHandle nh, ros::NodeHandle nh_private) {

        std::string serial_port;
        int baudrate;
        int num_roboclaws;

        this->nh = nh;
        this->nh_private = nh_private;

        nh_private.param("serial_port", serial_port);
        nh_private.param("baudrate", baudrate, (int) driver::DEFAULT_BAUDRATE);
        nh_private.param("roboclaws", num_roboclaws, 1);

        roboclaw_mapping = std::map<int, unsigned char>();

        // Create address map
        if (num_roboclaws > 1) {

            for (int r = 0; r < num_roboclaws; r++)
                roboclaw_mapping.insert(std::pair<int, unsigned char>(r, driver::BASE_ADDRESS + r));

        } else {
            num_roboclaws = 1;

            roboclaw_mapping.insert(std::pair<int, unsigned char>(0, driver::BASE_ADDRESS));
        }

        roboclaw = new driver(serial_port);
        roboclaw->set_baud((unsigned int) baudrate);

        for (int r = 0; r < roboclaw_mapping.size(); r++)
            roboclaw->reset_encoders(roboclaw_mapping[r]);

        encoder_pub = nh_private.advertise<roboclaw::RoboclawEncoderSteps>(std::string("motor_enc"), 10);
        velocity_sub = nh.subscribe(std::string("motor_vel_cmd"), 10, &roboclaw_roscore::velocity_callback, this);

    }

    void roboclaw_roscore::velocity_callback(const roboclaw::RoboclawMotorVelocity &msg) {
        roboclaw->set_velocity(roboclaw_mapping[msg.index], std::pair<int, int>(msg.mot1_vel_sps, msg.mot2_vel_sps));
    }

    void roboclaw_roscore::run() {

        last_message = ros::Time::now();

        ros::Rate update_rate(10);
        while (ros::ok()) {

            // Publish encoders
            for (int r = 0; r < roboclaw_mapping.size(); r++) {

                std::pair<int, int> encs = roboclaw->get_encoders(roboclaw_mapping[r]);

                RoboclawEncoderSteps enc_steps;
                enc_steps.index = r;
                enc_steps.mot1_enc_steps = encs.first;
                enc_steps.mot2_enc_steps = encs.second;
                encoder_pub.publish(enc_steps);

            }

            if (ros::Time::now() - last_message > ros::Duration(5))
                for (int r = 0; r < roboclaw_mapping.size(); r++) {
                    roboclaw->set_duty(roboclaw_mapping[r], std::pair<int, int>(0, 0));

            update_rate.sleep();
        }
    }

}