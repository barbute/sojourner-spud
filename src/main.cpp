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

std::string driveName = "D";
vex::motor leftMotor(vex::PORT3, vex::gearSetting::ratio18_1, true);
vex::motor rightMotor(vex::PORT4, vex::gearSetting::ratio18_1, false);
vex::inertial inertialSensor(vex::PORT6);

vex::optical opticalSensor(vex::PORT7);
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
const std::string COLOR = "BLUE"; // GREEN, BLUE, or PINK

const vex::color DESIRED_COLOR = vex::color::blue;

// TODO Get these constants
const double BOARD_COLOR_ROW_1_DISTANCE_MM = 1375.0;
const double BOARD_COLOR_ROW_2_DISTANCE_MM = 950.0;

// TODO Get these constants
const std::array<double, 3> COLOR_ROW_1_GREEN = {0.0, 0.0, 0.0};
const std::array<double, 3> COLOR_ROW_1_BLUE = {0.0, 0.0, 0.0};
const std::array<double, 3> COLOR_ROW_1_PINK = {0.0, 0.0, 0.0};

// TODO Get these constants
const std::array<double, 3> COLOR_ROW_2_GREEN = {0.0, 0.0, 0.0};
const std::array<double, 3> COLOR_ROW_2_BLUE = {0.0, 0.0, 0.0};
const std::array<double, 3> COLOR_ROW_2_PINK = {0.0, 0.0, 0.0};

const double PICKUP_DISTANCE_MM = 77.0;
const double PREP_PLACE_DISTANCE_MM = 200.0;
const double PLACE_DISTANCE_MM = 17.0;

const double PICKUP_HEIGHT_MM = 0.0;
const double CLEAR_TOP_BOX_HEIGHT_MM = 582.706;
const double PLACE_CUP_HEIGHT_MM = 450.0;
const double STOW_ELEVATOR_MM = 150.0;

const double CLAW_OPEN_ROTATIONS = 0.7;
const double CLAW_CLOSED_ROTATIONS = 0.31;

// Set to true before running graded performance
const bool RUN_AUTONOMOUS = true;
// Set to true before running the graded auto
const bool RUN_MAIN_AUTO = true;

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
  opticalSensor.setLight(vex::ledState::on);

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

      if (ROW == "1") {
        initialDistanceMM = BOARD_COLOR_ROW_1_DISTANCE_MM;
        if (COLOR == "GREEN") {
          colorToSeek = COLOR_ROW_1_GREEN;
        } else if (COLOR == "BLUE") {
          colorToSeek = COLOR_ROW_1_BLUE;
        } else if (COLOR == "PINK") {
          colorToSeek = COLOR_ROW_1_PINK;
        } else {
          lib::Telemetry::writeOutput(systemName, "ERROR - COLOR(1) SELECTION INVALID");
        }
      } else if (ROW == "2") {
        initialDistanceMM = BOARD_COLOR_ROW_2_DISTANCE_MM;
        if (COLOR == "GREEN") {
          colorToSeek = COLOR_ROW_2_GREEN;
        } else if (COLOR == "BLUE") {
          colorToSeek = COLOR_ROW_2_BLUE;
        } else if (COLOR == "PINK") {
          colorToSeek = COLOR_ROW_2_PINK;
        } else {
          lib::Telemetry::writeOutput(systemName, "ERROR - COLOR(2) SELECTION INVALID");
        }
      } else {
        lib::Telemetry::writeOutput(systemName, "ERROR - ROW SELECTION INVALID");
      }

      // Drive until on side of color row
      while (distanceSensor.objectDistance(vex::mm) > initialDistanceMM) {
        drive.drive(vex::forward, DRIVE_SPEED_PCT, vex::velocityUnits::pct);
      }
      drive.stop();
      wait(1, vex::sec);

      // Turn left 90 deg
      drive.turnToAngle(vex::left, 90.0, vex::degrees);

      // Drive until on correct color row
      // while (
      //   opticalSensor.getRgb().red != colorToSeek[0] and
      //   opticalSensor.getRgb().green != colorToSeek[1] and
      //   opticalSensor.getRgb().blue != colorToSeek[2]
      // ) {
      //   drive.drive(vex::forward, DRIVE_SPEED_PCT, vex::velocityUnits::pct);
      // }
      while (opticalSensor.color() != DESIRED_COLOR) {
        drive.drive(vex::forward, DRIVE_SPEED_PCT, vex::velocityUnits::pct);
      }
      drive.stop();
      wait(1, vex::sec);

      // Turn left 90 deg
      drive.turnToAngle(vex::left, 90.0, vex::degrees);

      // Claw pre open
      intake.setPositionRotations(CLAW_OPEN_ROTATIONS);

      // Elevator to pickup position
      elevator.setPositionMM(PICKUP_HEIGHT_MM);

      // Drive until in front of cup
      while (distanceSensor.objectDistance(vex::mm) > PICKUP_DISTANCE_MM) {
        drive.drive(vex::forward, DRIVE_SPEED_PCT, vex::velocityUnits::pct);
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
      lib::Telemetry::writeOutput(labelColorRed, opticalSensor.getRgb().red);
      lib::Telemetry::writeOutput(labelColorGreen, opticalSensor.getRgb().green);
      lib::Telemetry::writeOutput(labelColorBlue, opticalSensor.getRgb().blue);

      wait(5, vex::msec);
    }
  }

  return 0;
}
