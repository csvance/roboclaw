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


#include "diffdrive_roscore.h"
#include "roboclaw/RoboclawMotorVelocity.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

#include <cmath>


namespace roboclaw{

    diffdrive_roscore::diffdrive_roscore(ros::NodeHandle nh, ros::NodeHandle nh_private) {

        this->nh = nh;
        this->nh_private = nh_private;

        odom_pub = nh_private.advertise<nav_msgs::Odometry>(std::string("odom"), 10);
        motor_pub = nh_private.advertise<roboclaw::RoboclawMotorVelocity>(std::string("motor_vel_cmd"), 10);

        encoder_sub = nh.subscribe(std::string("motor_enc"), 10, &diffdrive_roscore::encoder_callback, this);
        twist_sub = nh.subscribe(std::string("vel_cmd"), 10, &diffdrive_roscore::twist_callback, this);

        last_theta = 0.0;
        last_steps_1 = 0;
        last_steps_2 = 0;

        nh_private.param("base_width", base_width);
        nh_private.param("steps_per_meter", steps_per_meter);
        nh_private.param("wheel_radius", wheel_radius);

    }

    void diffdrive_roscore::twist_callback(const geometry_msgs::Twist &msg){

        // TODO: Calculate velocities
        roboclaw::RoboclawMotorVelocity motor_vel;

        if(abs(msg.linear.x) + abs(msg.linear.y) + abs(msg.linear.z) != 0){

        }else if(abs(msg.angular.x) + abs(msg.angular.y) + abs(msg.angular.z) != 0){

        }else{

        }
    }

    void diffdrive_roscore::encoder_callback(const roboclaw::RoboclawEncoderSteps &msg){

        int delta_1 = last_steps_1 - msg.mot1_enc_steps;
        int delta_2 = last_steps_2 - msg.mot2_enc_steps;

        double u_w = (delta_1 + delta_2) / 2.0;
        double u_p = (delta_2 - delta_1);

        double delta_x = wheel_radius * u_w * cos(last_theta);
        double delta_y = wheel_radius * u_w * sin(last_theta);
        double delta_theta = wheel_radius / base_width * u_p;

        last_x += delta_x;
        last_y += delta_y;
        last_theta += delta_theta;

        nav_msgs::Odometry odom;

        odom.header.stamp = ros::Time::now();
        odom.pose.pose.position.x = last_x;
        odom.pose.pose.position.y = last_y;

        tf::Quaternion quaternion = tf::createQuaternionFromRPY(0.0, 0.0, last_theta);
        odom.pose.pose.orientation.w = quaternion.w();
        odom.pose.pose.orientation.x = quaternion.x();
        odom.pose.pose.orientation.y = quaternion.y();
        odom.pose.pose.orientation.z = quaternion.z();

        odom_pub.publish(odom);

    }


}