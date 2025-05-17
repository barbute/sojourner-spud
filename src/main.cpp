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
vex::controller pilotController;

std::string systemName = "S";
// Set to true before running graded performance
const bool RUN_AUTONOMOUS = false;
// Set to true before running the graded auto
const bool RUN_MAIN_AUTO = true;
// Set to false before running teleop
const bool RUN_CALIBRATION_MODE = false;

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

const double SINGLE_COLOR_DISTANCE_MM = 1040; // 320.0;
const std::array<double, 3> SINGLE_COLOR_TARGET = {1650.0, 1750.0, 1400.0};
const std::array<double, 3> WHITE_COLOR = {6700.0, 4300.0, 4700.0};
const double CENTER_BOARD_DISTANCE_MM = 550.0;
const double DOOR_DISTANCE_MM = 2480.0;

const double BOARD_COLOR_ROW_1_DISTANCE_MM = 1040.0; //1375.0;
const double BOARD_COLOR_ROW_2_DISTANCE_MM = 700.0; // 950.0;

const std::array<double, 3> COLOR_ROW_1_GREEN = {6600.0, 4950.0, 3380.0};
const std::array<double, 3> COLOR_ROW_1_BLUE = {2600.0, 2750.0, 3300.0};
const std::array<double, 3> COLOR_ROW_1_PINK = {7500.0, 1750.0, 3500.0};

const std::array<double, 3> COLOR_ROW_2_GREEN = {1650.0, 1750.0, 1400.0};
const std::array<double, 3> COLOR_ROW_2_BLUE = {5550.0, 4700.0, 5300.0};
const std::array<double, 3> COLOR_ROW_2_PINK = {7300.0, 1700.0, 2910.0};

const double PICKUP_DISTANCE_MM = 77.0;
const double PREP_PLACE_DISTANCE_MM = 270.0;
const double PLACE_DISTANCE_MM = 17.0;

const double PICKUP_HEIGHT_MM = 0.0;
const double CLEAR_TOP_BOX_HEIGHT_MM = 582.706 + 20.0;
const double PLACE_CUP_HEIGHT_MM = 480.0;
const double STOW_ELEVATOR_MM = 150.0;

const double CLAW_OPEN_ROTATIONS = 0.8;
const double CLAW_CLOSED_ROTATIONS = 0.31;

std::string labelDistance = "distance";
std::string labelColorRed = "colorRed";
std::string labelColorGreen = "colorGreen";
std::string labelColorBlue = "colorBlue";

subsystems::Drive drive(driveName, leftMotor, rightMotor, inertialSensor);
subsystems::Elevator elevator(elevatorName, elevatorMotor, 
  upperLimitSwitch, lowerLimitSwitch);
subsystems::Intake intake(intakeName, intakeMotor, surfaceLimitSwitch);

void prepPickup() {
  // Claw pre open
  intake.setPositionRotations(CLAW_OPEN_ROTATIONS, true);

  // Elevator to pickup position
  elevator.setPositionMM(PICKUP_HEIGHT_MM, true);
}

void runPickup() {
  prepPickup();

  // Run until senses cup
  while (distanceSensor.objectDistance(vex::mm) > PICKUP_DISTANCE_MM) {
    // TODO Replace with line following logic
    Brain.Screen.print(distanceSensor.objectDistance(vex::mm));
    Brain.Screen.newLine();
    Brain.Screen.setCursor(1, 1);
    
    drive.drive(vex::forward, DRIVE_SPEED_PCT, vex::velocityUnits::pct);

    wait(5, vex::msec);
  }
  drive.stop();

  // Claw close
  intake.setPositionRotations(CLAW_CLOSED_ROTATIONS, true);

  // Elevator to stow cup
  elevator.setPositionMM(STOW_ELEVATOR_MM, true);
}

void runAutoPlace() {
    // Drive to prep placing position
  while (distanceSensor.objectDistance(vex::mm) > PREP_PLACE_DISTANCE_MM) {
    drive.drive(vex::forward, DRIVE_SPEED_PCT, vex::velocityUnits::pct);

    wait(5, vex::msec);
  }
  drive.stop();

  // Elevator raised to clearence
  elevator.setPositionMM(CLEAR_TOP_BOX_HEIGHT_MM, true);

  // Drive to place position
  drive.driveDistance(vex::forward, PREP_PLACE_DISTANCE_MM - PLACE_DISTANCE_MM, 
    vex::mm, false);

  // Elevator lower to place
  elevator.setPositionMM(PLACE_CUP_HEIGHT_MM, true);

  // Claw open
  intake.setPositionRotations(CLAW_OPEN_ROTATIONS, true);

  // Elevator raise to clearence
  elevator.setPositionMM(CLEAR_TOP_BOX_HEIGHT_MM, true);

  // Claw close
  intake.setPositionRotations(CLAW_CLOSED_ROTATIONS, true);

  // Drive backwards
  drive.driveDistance(vex::reverse, PREP_PLACE_DISTANCE_MM, vex::mm, false);

  // Elevator lower to pickup
  elevator.setPositionMM(PICKUP_HEIGHT_MM, true);
}

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

  if (RUN_AUTONOMOUS) {
    if (RUN_MAIN_AUTO) {
      // Grab constants based on where the robot is running
      double initialDistanceMM = 0.0;
      std::array<double, 3> colorToSeek;

      initialDistanceMM = SINGLE_COLOR_DISTANCE_MM;
      colorToSeek = SINGLE_COLOR_TARGET;

      // Raise claw to be out of way
      elevator.setPositionMM(STOW_ELEVATOR_MM, true);

      // Drive until on side of color row
      drive.driveDistance(vex::forward, 4800.0, vex::mm, true);

      // Turn left 90 deg
      drive.turnToAngle(vex::left, 90.0, vex::degrees, true);

      // Drive until on correct pad
      drive.driveDistance(vex::forward, 4600.0, vex::mm, true);

      // Turn left 90 deg
      drive.turnToAngle(vex::left, 90.0, vex::degrees, true);

      // Run to pickup the cup
      runPickup();

      // Turn left 90 deg
      drive.turnToAngle(vex::left, 90.0, vex::degrees, true);

      // Score cup
      runAutoPlace();
    } else {
      // Prep Open claw
      intake.setPositionRotations(CLAW_OPEN_ROTATIONS, true);

      // Drive until cup is in front of distance sensor
      while (distanceSensor.objectDistance(vex::mm) > PICKUP_DISTANCE_MM) {
        drive.drive(vex::forward, DRIVE_SPEED_PCT, vex::velocityUnits::pct);
      }
      drive.stop();
      wait(1, vex::sec);
  
      // Close claw
      intake.setPositionRotations(CLAW_CLOSED_ROTATIONS, true);
  
      // Stow elevator to clear distance sensor
      elevator.setPositionMM(STOW_ELEVATOR_MM, true);
  
      // Turn to boxes
      drive.turnToAngle(vex::right, 90.0, vex::degrees, true);
  
      // Drive until stack of boxes is in front of robot
      while (distanceSensor.objectDistance(vex::mm) > PREP_PLACE_DISTANCE_MM) {
        drive.drive(vex::forward, DRIVE_SPEED_PCT, vex::velocityUnits::pct);
  
        wait(5, vex::msec);
      }
      drive.stop();
  
      // Lift elevator to clearence level
      elevator.setPositionMM(CLEAR_TOP_BOX_HEIGHT_MM, true);
  
      // Drive forward slightly
      drive.driveDistance(vex::forward, PREP_PLACE_DISTANCE_MM - PLACE_DISTANCE_MM, 
        vex::mm, true);
  
      // Lower elevator into box
      elevator.setPositionMM(PLACE_CUP_HEIGHT_MM, true);
  
      // Drop cup
      intake.setPositionRotations(CLAW_OPEN_ROTATIONS, true);
  
      // Back out from cup
      elevator.setPositionMM(CLEAR_TOP_BOX_HEIGHT_MM, true);
  
      // Close claw
      intake.setPositionRotations(CLAW_CLOSED_ROTATIONS, true);
  
      // Drive backward slightly
      drive.driveDistance(vex::reverse, PREP_PLACE_DISTANCE_MM, vex::mm, true);
  
      // Lower elevator to pickup position
      elevator.setPositionMM(PICKUP_HEIGHT_MM, true);
    }
  } else {
    if (RUN_CALIBRATION_MODE) {
      while (true) {
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
    } else {
      while (true) {
        if (pilotController.ButtonY.pressing()) {
          runAutoPlace();
        } else if (pilotController.ButtonA.pressing()) {
          runPickup();
        } else {
          if (pilotController.ButtonLeft.PRESSED) {
            drive.turnToAngle(vex::left, 90.0, vex::degrees, true);
          } else if (pilotController.ButtonRight.PRESSED) {
            drive.turnToAngle(vex::right, 90.0, vex::degrees, true);
          } else {
            drive.arcadeDrive(
              pilotController.Axis3.value() * 0.75, pilotController.Axis1.value() * 0.75);
          }

          // Claw bindings
          if (pilotController.ButtonL1.pressing()) {
            intake.setPositionRotations(CLAW_OPEN_ROTATIONS, false);
          } else if (pilotController.ButtonR1.pressing()) {
            intake.setPositionRotations(CLAW_CLOSED_ROTATIONS, false);
          } else {
            intake.stop();
          }

          // Elevator bindings
          if (pilotController.ButtonX.pressing()) {
            elevator.setPositionMM(CLEAR_TOP_BOX_HEIGHT_MM, false);
          } else if (pilotController.ButtonB.pressing()) {
            elevator.setPositionMM(PICKUP_HEIGHT_MM, false);
          } else {
            elevator.stop();
          }
        }

        wait(5, vex::msec);
      }
    }
  }

  return 0;
}