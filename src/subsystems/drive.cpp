// Copyright (c) 2025 barbute
// 
// This file is part of sojourner-spud and is licensed under the MIT License.
// See the LICENSE file in the root of the project for more information.
//
// File: drive.cpp
// Description: Subsystem to interface with a tank drive

#include "subsystems/drive.h"
#include "lib/telemetry.h"

namespace subsystems {
  Drive::Drive(
    std::string& name,
    vex::motor& leftMotorReference, 
    vex::motor& rightMotorReference, 
    vex::inertial& inertialSensorReference
  )
  : lib::Subsystem(name),
    leftMotor(leftMotorReference),
    rightMotor(rightMotorReference),
    inertialSensor(inertialSensorReference),
    robotDrive(leftMotor, rightMotor, inertialSensor, 
               WHEEL_CIRCUMFERENCE, TRACK_WIDTH, WHEEL_BASE, 
               UNITS, EXTERNAL_GEAR_RATIO) {}

  void Drive::periodic() {
    printTelemetry();
  }

  void Drive::printTelemetry() {}

  void Drive::stop() {
    leftMotor.stop();
    rightMotor.stop();
  }

  void Drive::arcadeDrive(double linear, double rotate) {
    robotDrive.arcade(linear, rotate, vex::percent);
  }

  void Drive::drive(vex::directionType direction, double speed,
    vex::velocityUnits units) {
    robotDrive.drive(direction, speed, units);
  }

  void Drive::driveDistance(vex::directionType direction, double distance, 
    vex::distanceUnits units, bool blocking) {
    robotDrive.setDriveVelocity(30.0, vex::pct);
    robotDrive.driveFor(direction, distance, units, blocking);
  }

  void Drive::turnToAngle(
    vex::turnType direction, double angle,
    vex::rotationUnits units, bool blocking) 
  {
    robotDrive.turnFor(direction, angle, units, blocking);
  }

  double Drive::getHeadingDegrees() { 
    return inertialSensor.heading(); 
  }
}