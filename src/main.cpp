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
#include <sstream>
#include <array>

using namespace vex;
using signature = vision::signature;
using code = vision::code;

vex::brain Brain;

std::string systemName = "S";
// Set to true before running graded performance
const bool RUN_AUTONOMOUS = true;
// Set to true before running the graded auto
const bool RUN_MAIN_AUTO = true;

std::string driveName = "D";
vex::motor leftMotor(vex::PORT3, vex::gearSetting::ratio18_1, true);
vex::motor rightMotor(vex::PORT4, vex::gearSetting::ratio18_1, false);
vex::inertial inertialSensor(vex::PORT6);

vex::optical topOpticalSensor(vex::PORT7);
vex::optical leftOpticalSensor(vex::PORT8);
vex::optical rightOpticalSensor(vex::PORT9);
vex::distance distanceSensor(vex::PORT10);

std::string elevatorName = "E";
vex::motor elevatorMotor(vex::PORT2, vex::gearSetting::ratio18_1, false);
vex::digital_in upperLimitSwitch(Brain.ThreeWirePort.E);
vex::digital_in lowerLimitSwitch(Brain.ThreeWirePort.F);

std::string intakeName = "I";
vex::motor intakeMotor(vex::PORT1, vex::gearSetting::ratio18_1, false);
// TODO Add this limit switch as it's currently not on the robot yet
vex::digital_in surfaceLimitSwitch(Brain.ThreeWirePort.D);

const double DRIVE_SPEED_PCT = 20.0;

const std::string ROW = "1"; // 1 or 2
const std::string COLOR = "GREEN"; // GREEN, BLUE, or PINK
const bool TARGET_SINGLE_COLOR = true;

// TODO Get these constants
const double SINGLE_COLOR_DISTANCE_MM = 1040; // 320.0;
const std::array<double, 3> SINGLE_COLOR_TARGET = {1650.0, 1750.0, 1400.0};
const std::array<double, 3> WHITE_COLOR = {6700.0, 4300.0, 4700.0};
const double CENTER_BOARD_DISTANCE_MM = 550.0;
const double DOOR_DISTANCE_MM = 2480.0;

// TODO Get these constants
const double BOARD_COLOR_ROW_1_DISTANCE_MM = 1040.0; //1375.0;
const double BOARD_COLOR_ROW_2_DISTANCE_MM = 700.0; // 950.0;

// TODO Get these constants
const std::array<double, 3> COLOR_ROW_1_GREEN = {6600.0, 4950.0, 3380.0};
const std::array<double, 3> COLOR_ROW_1_BLUE = {2600.0, 2750.0, 3300.0};
const std::array<double, 3> COLOR_ROW_1_PINK = {7500.0, 1750.0, 3500.0};

// TODO Get these constants
const std::array<double, 3> COLOR_ROW_2_GREEN = {1650.0, 1750.0, 1400.0};
const std::array<double, 3> COLOR_ROW_2_BLUE = {5550.0, 4700.0, 5300.0};
const std::array<double, 3> COLOR_ROW_2_PINK = {7300.0, 1700.0, 2910.0};

const double PICKUP_DISTANCE_MM = 77.0;
const double PREP_PLACE_DISTANCE_MM = 250.0;
const double PLACE_DISTANCE_MM = 17.0;

const double PICKUP_HEIGHT_MM = 0.0;
const double CLEAR_TOP_BOX_HEIGHT_MM = 582.706;
const double PLACE_CUP_HEIGHT_MM = 450.0;
const double STOW_ELEVATOR_MM = 150.0;

const double CLAW_OPEN_ROTATIONS = 0.8;
const double CLAW_CLOSED_ROTATIONS = 0.31;

std::string labelDistance = "distance";
std::string labelColorRed = "colorRed";
std::string labelColorGreen = "colorGreen";
std::string labelColorBlue = "colorBlue";

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
  // Turn on color sensor light
  topOpticalSensor.setLight(vex::ledState::on);
  leftOpticalSensor.setLight(vex::ledState::on);
  rightOpticalSensor.setLight(vex::ledState::on);

  // Instantiate subsystems
  subsystems::Drive drive(driveName, leftMotor, rightMotor, inertialSensor);
  subsystems::Elevator elevator(elevatorName, elevatorMotor, 
    upperLimitSwitch, lowerLimitSwitch);
  subsystems::Intake intake(intakeName, intakeMotor, surfaceLimitSwitch);

  // NOTE Actions are waiting/blocking by default, meaning the code program
  // will wait for that action to complete before moveing on, thus why all
  // actions below can be ordered sequentially with little transition or
  // scheduling logic
  if (RUN_AUTONOMOUS) {
    if (RUN_MAIN_AUTO) {
      // Grab constants based on where the robot is running
      double initialDistanceMM = 0.0;
      std::array<double, 3> colorToSeek;

      initialDistanceMM = SINGLE_COLOR_DISTANCE_MM;
      colorToSeek = SINGLE_COLOR_TARGET;

      // if (TARGET_SINGLE_COLOR) {
      //   initialDistanceMM = SINGLE_COLOR_DISTANCE_MM;
      //   colorToSeek = SINGLE_COLOR_TARGET;
      // } else {
      //   if (ROW == "1") {
      //     initialDistanceMM = BOARD_COLOR_ROW_1_DISTANCE_MM;
      //     if (COLOR == "GREEN") {
      //       colorToSeek = COLOR_ROW_1_GREEN;
      //     } else if (COLOR == "BLUE") {
      //       colorToSeek = COLOR_ROW_1_BLUE;
      //     } else if (COLOR == "PINK") {
      //       colorToSeek = COLOR_ROW_1_PINK;
      //     } else {
      //       lib::Telemetry::writeOutput(systemName, "ERROR - COLOR(1) SELECTION INVALID");
      //       colorToSeek = COLOR_ROW_2_PINK;
      //     }
      //   } else if (ROW == "2") {
      //     initialDistanceMM = BOARD_COLOR_ROW_2_DISTANCE_MM;
      //     if (COLOR == "GREEN") {
      //       colorToSeek = COLOR_ROW_2_GREEN;
      //     } else if (COLOR == "BLUE") {
      //       colorToSeek = COLOR_ROW_2_BLUE;
      //     } else if (COLOR == "PINK") {
      //       colorToSeek = COLOR_ROW_2_PINK;
      //     } else {
      //       lib::Telemetry::writeOutput(systemName, "ERROR - COLOR(2) SELECTION INVALID");
      //       colorToSeek = COLOR_ROW_2_PINK;
      //     }
      //   } else {
      //     lib::Telemetry::writeOutput(systemName, "ERROR - ROW SELECTION INVALID");
      //     initialDistanceMM = BOARD_COLOR_ROW_1_DISTANCE_MM;
      //     colorToSeek = COLOR_ROW_2_PINK;
      //   }
      // }

      // Raise claw to be out of way
      elevator.setPositionMM(STOW_ELEVATOR_MM);

      // Drive until on side of color row
      // while (distanceSensor.objectDistance(vex::mm) > initialDistanceMM) {
      //   Brain.Screen.print(distanceSensor.objectDistance(vex::mm));
      //   Brain.Screen.newLine();
      //   Brain.Screen.setCursor(1, 1);

      //   drive.drive(vex::forward, DRIVE_SPEED_PCT, vex::velocityUnits::pct);

      //   wait(5, vex::msec);
      // }
      // drive.stop();
      // wait(1, vex::sec);
      drive.driveDistance(vex::forward, 4800.0, vex::mm);

      // Turn left 90 deg
      drive.turnToAngle(vex::left, 90.0, vex::degrees);

      // Drive until on correct color
      // while (distanceSensor.objectDistance(vex::mm) > DOOR_DISTANCE_MM) {
      //   // Brain.Screen.print(distanceSensor.objectDistance(vex::mm));
      //   // Brain.Screen.newLine();
      //   // Brain.Screen.setCursor(1, 1);

      //   drive.drive(vex::forward, DRIVE_SPEED_PCT, vex::velocityUnits::pct);

      //   wait(5, vex::msec);
      // }
      // drive.stop();
      // wait(1, vex::sec);
      // drive.driveDistance(vex::forward, 5000.0, vex::mm);
      drive.driveDistance(vex::forward, 4600.0, vex::mm);

      // if (TARGET_SINGLE_COLOR) {
      //   // while (
      //   //   !(topOpticalSensor.getRgb().red >= colorToSeek[0]) and
      //   //   !(topOpticalSensor.getRgb().green >= colorToSeek[1]) and
      //   //   !(topOpticalSensor.getRgb().blue >= colorToSeek[2])
      //   // ) {
      //   //   Brain.Screen.print(topOpticalSensor.getRgb().red);
      //   //   Brain.Screen.newLine();
      //   //   Brain.Screen.print(topOpticalSensor.getRgb().green);
      //   //   Brain.Screen.newLine();
      //   //   Brain.Screen.print(topOpticalSensor.getRgb().blue);
      //   //   Brain.Screen.newLine();
      //   //   Brain.Screen.setCursor(1, 1);
  
      //   //   drive.drive(vex::forward, DRIVE_SPEED_PCT, vex::velocityUnits::pct);
      //   // }
      //   while (distanceSensor.objectDistance(vex::mm) > DOOR_DISTANCE_MM) {
      //     Brain.Screen.print(distanceSensor.objectDistance(vex::mm));
      //     Brain.Screen.newLine();
      //     Brain.Screen.setCursor(1, 1);

      //     drive.drive(vex::forward, DRIVE_SPEED_PCT, vex::velocityUnits::pct);
      //   }
      //   Brain.Screen.print("LOOP ONE COMPLETE");
      //   Brain.Screen.newLine();
      //   Brain.Screen.setCursor(1, 1);

      //   drive.stop();
      //   wait(5, vex::sec);
      //   // Redundant while loop in case the distance sensor jumps
      //   while (distanceSensor.objectDistance(vex::mm) > DOOR_DISTANCE_MM) {
      //     Brain.Screen.print(distanceSensor.objectDistance(vex::mm));
      //     Brain.Screen.newLine();
      //     Brain.Screen.setCursor(1, 1);

      //     drive.drive(vex::forward, DRIVE_SPEED_PCT, vex::velocityUnits::pct);
      //   }
      //   drive.stop();
      //   wait(1, vex::sec);

      //   drive.driveDistance(vex::forward, CENTER_BOARD_DISTANCE_MM, vex::mm);
      // } else {
      //   while (
      //     !(topOpticalSensor.getRgb().red >= colorToSeek[0]) and
      //     !(topOpticalSensor.getRgb().green >= colorToSeek[1]) and
      //     !(topOpticalSensor.getRgb().blue >= colorToSeek[2])
      //   ) {
      //     Brain.Screen.print(topOpticalSensor.getRgb().red);
      //     Brain.Screen.newLine();
      //     Brain.Screen.print(topOpticalSensor.getRgb().green);
      //     Brain.Screen.newLine();
      //     Brain.Screen.print(topOpticalSensor.getRgb().blue);
      //     Brain.Screen.newLine();
      //     Brain.Screen.setCursor(1, 1);
  
      //     drive.drive(vex::forward, DRIVE_SPEED_PCT, vex::velocityUnits::pct);
      //   }
      //   drive.stop();
      //   wait(1, vex::sec);
      // }

      // Turn left 90 deg
      drive.turnToAngle(vex::left, 90.0, vex::degrees);

      // Claw pre open
      intake.setPositionRotations(CLAW_OPEN_ROTATIONS);

      // Elevator to pickup position
      elevator.setPositionMM(PICKUP_HEIGHT_MM);

      // Drive until in front of cup
      if (TARGET_SINGLE_COLOR) {
        while (distanceSensor.objectDistance(vex::mm) > PICKUP_DISTANCE_MM) {
          // TODO Replace with line following logic
          Brain.Screen.print(distanceSensor.objectDistance(vex::mm));
          Brain.Screen.newLine();
          Brain.Screen.setCursor(1, 1);
          
          drive.drive(vex::forward, DRIVE_SPEED_PCT, vex::velocityUnits::pct);

          wait(5, vex::msec);
        }
      } else {
        while (distanceSensor.objectDistance(vex::mm) > PICKUP_DISTANCE_MM) {
          Brain.Screen.print(distanceSensor.objectDistance(vex::mm));
          Brain.Screen.newLine();
          Brain.Screen.setCursor(1, 1);
          
          drive.drive(vex::forward, DRIVE_SPEED_PCT, vex::velocityUnits::pct);

          wait(5, vex::msec);
        }
      }
      drive.stop();
      wait(1, vex::sec);

      // Claw close
      intake.setPositionRotations(CLAW_CLOSED_ROTATIONS);

      // Elevator to stow cup
      elevator.setPositionMM(STOW_ELEVATOR_MM);

      // Turn left 90 deg
      drive.turnToAngle(vex::left, 90.0, vex::degrees);

      // Drive to prep placing position
      while (distanceSensor.objectDistance(vex::mm) > PREP_PLACE_DISTANCE_MM) {
        drive.drive(vex::forward, DRIVE_SPEED_PCT, vex::velocityUnits::pct);
  
        wait(5, vex::msec);
      }
      drive.stop();

      // Elevator raised to clearence
      elevator.setPositionMM(CLEAR_TOP_BOX_HEIGHT_MM);

      // Drive to place position
      drive.driveDistance(vex::forward, PREP_PLACE_DISTANCE_MM - PLACE_DISTANCE_MM, vex::mm);

      // Elevator lower to place
      elevator.setPositionMM(PLACE_CUP_HEIGHT_MM);

      // Claw open
      intake.setPositionRotations(CLAW_OPEN_ROTATIONS);

      // Elevator raise to clearence
      elevator.setPositionMM(CLEAR_TOP_BOX_HEIGHT_MM);

      // Claw close
      intake.setPositionRotations(CLAW_CLOSED_ROTATIONS);

      // Drive backwards
      drive.driveDistance(vex::reverse, PREP_PLACE_DISTANCE_MM, vex::mm);

      // Elevator lower to pickup
      elevator.setPositionMM(PICKUP_HEIGHT_MM);
    } else {
      // Prep Open claw
      intake.setPositionRotations(CLAW_OPEN_ROTATIONS);

      // Drive until cup is in front of distance sensor
      while (distanceSensor.objectDistance(vex::mm) > PICKUP_DISTANCE_MM) {
        drive.drive(vex::forward, DRIVE_SPEED_PCT, vex::velocityUnits::pct);
      }
      drive.stop();
      wait(1, vex::sec);
  
      // Close claw
      intake.setPositionRotations(CLAW_CLOSED_ROTATIONS);
  
      // Stow elevator to clear distance sensor
      elevator.setPositionMM(STOW_ELEVATOR_MM);
  
      // Turn to boxes
      drive.turnToAngle(vex::right, 90.0, vex::degrees);
  
      // Drive until stack of boxes is in front of robot
      while (distanceSensor.objectDistance(vex::mm) > PREP_PLACE_DISTANCE_MM) {
        drive.drive(vex::forward, DRIVE_SPEED_PCT, vex::velocityUnits::pct);
  
        wait(5, vex::msec);
      }
      drive.stop();
  
      // Lift elevator to clearence level
      elevator.setPositionMM(CLEAR_TOP_BOX_HEIGHT_MM);
  
      // Drive forward slightly
      drive.driveDistance(vex::forward, PREP_PLACE_DISTANCE_MM - PLACE_DISTANCE_MM, vex::mm);
  
      // Lower elevator into box
      elevator.setPositionMM(PLACE_CUP_HEIGHT_MM);
  
      // Drop cup
      intake.setPositionRotations(CLAW_OPEN_ROTATIONS);
  
      // Back out from cup
      elevator.setPositionMM(CLEAR_TOP_BOX_HEIGHT_MM);
  
      // Close claw
      intake.setPositionRotations(CLAW_CLOSED_ROTATIONS);
  
      // Drive backward slightly
      drive.driveDistance(vex::reverse, PREP_PLACE_DISTANCE_MM, vex::mm);
  
      // Lower elevator to pickup position
      elevator.setPositionMM(PICKUP_HEIGHT_MM);
    }
  } else {
    // Run periodics which will call telemetry, useful to ensure data
    // reporting is good
    while (true) {
      // drive.periodic();
      // elevator.periodic();
      // intake.periodic();
      elevator.printTelemetry();
      intake.printTelemetry();

      lib::Telemetry::writeOutput(labelDistance, distanceSensor.objectDistance(vex::mm));
      lib::Telemetry::writeOutput(labelColorRed, topOpticalSensor.getRgb().red);
      lib::Telemetry::writeOutput(labelColorGreen, topOpticalSensor.getRgb().green);
      lib::Telemetry::writeOutput(labelColorBlue, topOpticalSensor.getRgb().blue);

      Brain.Screen.print(topOpticalSensor.getRgb().red);
      Brain.Screen.newLine();
      Brain.Screen.print(topOpticalSensor.getRgb().green);
      Brain.Screen.newLine();
      Brain.Screen.print(topOpticalSensor.getRgb().blue);
      Brain.Screen.newLine();
      Brain.Screen.print(distanceSensor.objectDistance(vex::mm));
      Brain.Screen.newLine();
      Brain.Screen.setCursor(1, 1);

      wait(5, vex::msec);
    }
  }

  return 0;
}
