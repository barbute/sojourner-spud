// Copyright (c) 2025 barbute
// 
// This file is part of sojourner-spud and is licensed under the MIT License.
// See the LICENSE file in the root of the project for more information.
//
// File: drive.h
// Description: Subsystem to interface with a tank drive

#include "lib/subsystem.h"
#include "vex.h"

namespace subsystems {
  class Drive : public lib::Subsystem {
  public:
    Drive(
      motor& leftMotorReference, 
      motor& rightMotorReference, 
      inertial& inertialSensorReference);

    void periodic() override;
    void printTelemetry() override;
    void stop() override;

    void arcadeDrive(double linearDemandPercent, double rotationalDemandPercent);
    void driveDistance(double distance, distanceUnits units);
    void turnToAngle(double angle, rotationUnits units);

    double getHeading();
    double getDistancedriven();

  private:
    motor& leftMotor;
    motor& rightMotor;
    inertial& inertialSensor;

    // Constants for the drive
    const double WHEEL_CIRCUMFERENCE = 319.19;
    const double TRACK_WIDTH = 320.0;
    const double WHEEL_BASE = 120;
    // Units are defined here, so no need to include them in variable name as
    // long as the units are consistent across everything.
    const distanceUnits UNITS = mm;
    const double EXTERNAL_GEAR_RATIO = 1;

    smartdrive robotDrive;
  };
}