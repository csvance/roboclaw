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
#include <sstream>

namespace roboclaw {

    roscore::roscore(ros::NodeHandle nh, ros::NodeHandle nh_private) {

        std::string serial_port;
        int baudrate;
        int num_roboclaws;

        this->nh = nh;
        this->nh_private = nh_private;

        nh_private.param("serial_port", serial_port);
        nh_private.param("baudrate", baudrate, (int) driver::DEFAULT_BAUDRATE);
        nh_private.param("roboclaws", num_roboclaws, 0);

        roboclaw_mapping = std::map<int, unsigned char>();

        if (num_roboclaws > 0) {

            // Create address map
            for (int i = 0; i < num_roboclaws; i++) {
                int address = 0;

                std::stringstream param_name;
                param_name << "roboclaw_address_" << i;

                nh_private.param(param_name.str(), address);

                roboclaw_mapping.insert(std::pair<int, unsigned char>(i, driver::BASE_ADDRESS + i));
            }

            roboclaw = new driver(serial_port);

        } else {
            num_roboclaws = 1;
            roboclaw = new driver(serial_port);

            roboclaw_mapping.insert(std::pair<int, unsigned char>(0, driver::BASE_ADDRESS));
        }

        roboclaw->set_baud((unsigned int) baudrate);

        encoder_pub = nh_private.advertise<roboclaw::RoboclawEncoderSteps>(std::string("enc_steps"), 10);
        velocity_sub = nh.subscribe(std::string("vel_cmd"), 10, &roscore::velocity_callback, this);

    }

    void roscore::velocity_callback(const roboclaw::RoboclawMotorVelocity &msg){

    }

    void roscore::run(){

    }

}