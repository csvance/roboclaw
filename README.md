# Roboclaw
Roboclaw is an extensible series of [Roboclaw][roboclaw] nodes for [ROS][ros]

## Features

- The base node "roboclaw_node" supports up to 8 Roboclaw controllers using packet serial mode
- Drive systems and odometry are decoupled from the base Roboclaw node
- A differential drive node is supported out of the box
- Written in roscpp for effecient memory usage and performance

## Requirements
- ROS Kinetic or Melodic
- C++11 or higher 

## Planned

- Support for the [NASA JPL Open Source Rover][jpl]'s drive system
- Exposing more of the Roboclaw's functionality via messages / services



[roboclaw]: http://www.basicmicro.com
[ros]: http://www.ros.org
[jpl]: https://opensourcerover.jpl.nasa.gov