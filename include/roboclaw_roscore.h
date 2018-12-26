//
// Created by Carroll Vance on 2018-12-26.
//

#ifndef PROJECT_ROBOCLAW_ROSCORE_H
#define PROJECT_ROBOCLAW_ROSCORE_H

#include "ros/ros.h"
#include "ros/package.h"

#include "roboclaw_driver.h"

namespace roboclaw {

    class roscore {
    public:
        roscore(ros::NodeHandle nh, ros::NodeHandle nh_private);
    private:
        driver* roboclaw;

        ros::NodeHandle nh;
        ros::NodeHandle nh_private;
    };


}

#endif //PROJECT_ROBOCLAW_ROSCORE_H
