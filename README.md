# Roboclaw
Roboclaw is an extensible series of [Roboclaw][roboclaw] nodes for [ROS][ros]

## Features

- The base node "roboclaw_node" supports up to 8 Roboclaw controllers using packet serial mode
- Drive systems and odometry are decoupled from the base Roboclaw node
- A differential drive node is supported out of the box
- Written in roscpp for effecient memory usage and performance

## Requirements
- ROS Kinetic/Lunar/Melodic

## Nodes

### roboclaw_node

#### Parameters

| Param | Type  | Description  |
| :------------- |:-------------| :-----|
| serial_port | string | Path to the serial port to use |
| baudrate | int | baudrate of the serial port |

#### Topics
| Action | Topic | Type |
| :------------- |:-------------| :-----|
| publish | odom | Odometry |
| subscribe | cmd_vel | Twist |

### diffdrive_node

#### Parameters

| Param | Type  | Description  |
| :------------- |:-------------| :-----|
| steps_per_meter | string | Path to the serial port to use |
| base_width | int | baudrate of the serial port |
| swap_motors | bool | Swap motor1 with motor2
| invert_motor_1 | bool | Invert drive and odometry for motor1
| invert_motor_2 | bool | Invert drive and odometry for motor2

#### Topics
| Action | Topic | Type |
| :------------- |:-------------| :-----|
| publish | motor_enc_steps | roboclaw/RoboclawEncoderSteps |
| subscribe | motor_cmd_vel | roboclaw/RoboclawMotorVelocity |

## Planned

- Support for the [NASA JPL Open Source Rover][jpl]'s drive system
- Exposing more of the Roboclaw's functionality via messages / services



[roboclaw]: http://www.basicmicro.com
[ros]: http://www.ros.org
[jpl]: https://opensourcerover.jpl.nasa.gov