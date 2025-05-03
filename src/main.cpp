// Copyright (c) 2025 barbute
// 
// This file is part of sojourner-spud and is licensed under the MIT License.
// See the LICENSE file in the root of the project for more information.
//
// File: main.cpp
// Description: Main file for robot program. All execution occurs here.

#include "vex.h"
#include "subsystems/drive.h"
#include "subsystems/elevator.h"
#include "subsystems/intake.h"
#include "lib/telemetry.h"
#include <iostream>

using namespace vex;
using signature = vision::signature;
using code = vision::code;

vex::brain Brain;

std::string driveName = "D";
vex::motor leftMotor(vex::PORT3, vex::gearSetting::ratio18_1, true);
vex::motor rightMotor(vex::PORT4, vex::gearSetting::ratio18_1, false);
vex::inertial inertialSensor(vex::PORT6);

vex::color colorSensor(vex::PORT7);
vex::distance distanceSensor(vex::PORT8);

std::string elevatorName = "E";
vex::motor elevatorMotor(vex::PORT2, vex::gearSetting::ratio18_1, false);
vex::digital_in upperLimitSwitch(Brain.ThreeWirePort.A);
vex::digital_in lowerLimitSwitch(Brain.ThreeWirePort.B);

std::string intakeName = "I";
vex::motor intakeMotor(vex::PORT1, vex::gearSetting::ratio18_1, false);
// TODO Add this limit switch as it's currently not on the robot yet
vex::digital_in surfaceLimitSwitch(Brain.ThreeWirePort.C);

// TODO Get these constants
const double DRIVE_SPEED_RPM = 150.0;

// TODO Get these setpoints
const double PICKUP_DISTANCE_MM = 40.0;
const double PREP_PLACE_DISTANCE_MM = 20.0;
const double PLACE_DISTANCE_MM = 15.0;

// TODO Get these setpoints
const double PICKUP_HEIGHT_MM = 0.0;
const double CLEAR_TOP_BOX_HEIGHT_MM = 600.0;
const double PLACE_CUP_HEIGHT_MM = 550.0;

// TODO Get these setpoints
const double CLAW_OPEN_ROTATIONS = 100.0;
const double CLAW_CLOSED_ROTATIONS = 0.0;

// Set to true before running graded performance
const bool RUN_AUTONOMOUS = false;

int main() {
  // Initialization routine for devices that need it
  Brain.Screen.print("Device initialization...");
  Brain.Screen.setCursor(2, 1);
  // calibrate the drivetrain inertial
  wait(200, msec);
  inertialSensor.startCalibration(1);
  Brain.Screen.print("Calibrating Gyro for Drivetrain");
  // wait for the inertial calibration process to finish
  while (inertialSensor.isCalibrating()) {
    wait(25, msec);
  }
  // reset the screen now that the calibration is complete
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  wait(50, msec);
  Brain.Screen.clearScreen();

  // Instantiate subsystems
  subsystems::Drive drive(driveName, leftMotor, rightMotor, inertialSensor);
  subsystems::Elevator elevator(elevatorName, elevatorMotor, 
    upperLimitSwitch, lowerLimitSwitch);
  subsystems::Intake intake(intakeName, intakeMotor, surfaceLimitSwitch);

  if (RUN_AUTONOMOUS) {
    // Prep Open claw
    intake.setPositionRotations(CLAW_OPEN_ROTATIONS);

    // Drive until cup is in front of distance sensor
    while (distanceSensor.objectDistance(vex::inches) < PICKUP_DISTANCE_MM) {
      drive.drive(vex::forward, DRIVE_SPEED_RPM, vex::rpm);
    }
    drive.stop();

    // Close claw
    intake.setPositionRotations(CLAW_CLOSED_ROTATIONS);

    // Turn to boxes
    drive.turnToAngle(vex::left, 90.0, vex::degrees);

    // Drive until stack of boxes is in front of robot
    while (distanceSensor.objectDistance(vex::inches) < PREP_PLACE_DISTANCE_MM) {
      drive.drive(vex::forward, DRIVE_SPEED_RPM, vex::rpm);

      wait(5, vex::msec);
    }
    drive.stop();

    // Lift elevator to clearence level
    elevator.setPositionMM(CLEAR_TOP_BOX_HEIGHT_MM);

    // Drive forward slightly
    drive.driveDistance(vex::forward, PLACE_DISTANCE_MM, vex::mm);

    // Lower elevator into box
    elevator.setPositionMM(PLACE_CUP_HEIGHT_MM);

    // Drop cup
    intake.setPositionRotations(CLAW_OPEN_ROTATIONS);

    // Close claw
    intake.setPositionRotations(CLAW_CLOSED_ROTATIONS);

    // Drive backward slightly
    drive.driveDistance(vex::reverse, PREP_PLACE_DISTANCE_MM, vex::mm);

    // Lower elevator to pickup oosition
    elevator.setPositionMM(PICKUP_HEIGHT_MM);
  } else {
    // Run periodics which will call telemetry, useful to ensure data
    // reporting is good
    while (true) {
      drive.periodic();
      elevator.periodic();
      intake.periodic();

      wait(5, vex::msec);
    }
  }

  return 0;
}
