//
// Created by Carroll Vance on 2018-12-26.
//

#include "../include/roboclaw_roscore.h"

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

        if (num_roboclaws > 0) {

            std::shared_ptr<std::map<int, unsigned char>> roboclaw_mapping(new std::map<int, unsigned char>());

            // Create address map
            for (int i = 0; i < num_roboclaws; i++) {
                int address = 0;

                std::stringstream param_name;
                param_name << "roboclaw_address_" << i;

                nh_private.param(param_name.str(), address);

                roboclaw_mapping->insert(std::pair<int, unsigned char>(i, (unsigned char) address));
            }

            roboclaw = new driver(serial_port, roboclaw_mapping);

        } else {
            num_roboclaws = 1;
            roboclaw = new driver(serial_port);
        }

        roboclaw->set_baud((unsigned int) baudrate);

    }

}