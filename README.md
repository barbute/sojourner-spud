# sojourner-spud
A program for controlling a [VEX V5](https://www.vexrobotics.com/v5?srsltid=AfmBOor7LT-k8MatAaz2pKX20vpmfsGfgkzMzOUpa_GgX34ZiYLnXh1O) robot with a dual-motor tank drive, single-motor single-stage elevator, and claw mechanism. This project was for a school assignment. For more information about VEX V5, see [the documentation](https://api.vex.com/v5/home/cpp/index.html).

## Description
The robot is designed to stack cups filled with water on to a raised tray fully autonomously, with pre-defined boundaries and starting positions. The robot program adopted a subsystem-based organization model, where hardware components are separated into "subsystems" and can be interacted with there. The `main.cpp` file is where actions can be defined for the autonomous period.

The robot also has button bindings to perform similar actions as the autonomous period, as well as with manual control over the claw and elevator separately.

### Port assignments
All ports are assigned to their respective devices, where the front of the robot is defined as the side with the claw and counterclockwise is positive.
These are the current ports for every device connected to the robot brain:
|Device|Port|
|------|----|
|motor-left_drive|3|
|motor-right_drive|4|
|sensor-inertial|6|
|sensor-front_optical|7|
|sensor-left_optical|8|
|sensor-right_optical|9|
|sensor-distance|10|
|motor-elevator|2|
|switch-upper_bound|E|
|switch-lower_bound|F|
|motor-intake|1|
|switch-surface|D|

> When operating the robot, it may attempt to continue performing an action due to an unforseen edge case. If seen, immediately disable the robot by holding down the power button on the controller to stop the robot program.

### Controller bindings
|Action|Button|
|------|------|
|drive-fwd|axis3-up|
|drive-rev|axis3-down|
|drive-left|axis1-left|
|drive-right|axis1-right|
|auto-score_cup_in_box|y-press|
|auto-pickup_cup|a-press|
|elevator-up|x-hold|
|elevator-down|b-hold|
|claw-open|L1-hold|
|claw-hold_cup|R1-hold|