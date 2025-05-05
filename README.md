# sojourner-spud
A program for controlling a VEX V5 robot with a dual-motor tank drive, single-motor single-stage elevator, and claw mechanism. This project was for a school assignment.

## Description
The robot is designed to stack cups filled with water on to a raised tray fully autonomously, with pre-defined boundaries and starting positions. This is meant to be easily extensible and upgradeable for future additions or modifications to the robot.

### Port assignments
All ports are assigned to their respective devices, where the front of the robot is defined as the side with the claw and counterclockwise is positive.
These are the current ports for every device connected to the robot brain:
|Device|Port|
|------|----|
|motor-left_drive|3|
|motor-right_drive|4|
|sensor-inertial|6|
|sensor-color|7|
|sensor-distance|8|
|motor-elevator|2|
|switch-upper_bound|A|
|switch-lower_bound|B|
|motor-intake|1|
|switch-surface|C|
