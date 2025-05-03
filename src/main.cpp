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
#include "lib/telemetry.h"
#include <iostream>

using namespace vex;
using signature = vision::signature;
using code = vision::code;

brain Brain;

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

vex::motor intakeMotor(vex::PORT1, vex::gearSetting::ratio18_1, false);

// TODO Get these setpoints
const double PICKUP_HEIGHT_MM = 0.0;
const double CLEAR_TOP_BOX_HEIGHT_MM = 600.0;
const double PLACE_CUP_HEIGHT_MM = 550.0;

int main() {
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

  subsystems::Drive drive(driveName, leftMotor, rightMotor, inertialSensor);
  subsystems::Elevator elevator(elevatorName, elevatorMotor, upperLimitSwitch, lowerLimitSwitch);

  // std::cout << "run forward";

  // drive.driveDistance(forward, 12, inches);

  // std::cout << "stopping";

  // drive.stop();

  // while (true) {
  //   elevator.periodic();

  //   wait(5, msec);
  // }

  elevator.moveToHeight(100);

  return 0;
}
