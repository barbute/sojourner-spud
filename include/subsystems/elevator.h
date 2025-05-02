// Copyright (c) 2025 barbute
// 
// This file is part of sojourner-spud and is licensed under the MIT License.
// See the LICENSE file in the root of the project for more information.
//
// File: elevator.h
// Description: Subsystem to interface with a single-stage linear-mechanism

#pragma once

#include "lib/subsystem.h"
#include "cmath"
#include "vex.h"

namespace subsystems {
  class Elevator : public lib::Subsystem {
  public:
    Elevator(
      std::string& name,
      vex::motor& motorReference, 
      vex::digital_in& limitSwitchUpperReference,
      vex::digital_in& limitSwitchLowerReference);

    void periodic() override;
    void printTelemetry() override;
    void stop() override;

    void moveToHeight(double targetHeightMM);
    void setVoltage(vex::directionType direction, double voltage);

    double getPositionMM();
    bool atTarget();

  private:
    vex::motor& motor;
    vex::digital_in& limitSwitchUpper;
    vex::digital_in& limitSwitchLower;

    const double TOLERANCE_MM = 1;
    const double PITCH_MM = 12.7;
    const double TEETH = 12;
    const double PI = 3.14159265;
    const double PITCH_DIAMETER_MM = (PITCH_MM) / (std::sin(PI / TEETH));
    const double SPROCKET_CIRCUMFERENCE_MM = PITCH_DIAMETER_MM * PI;

    std::string labelPosition = "E/POSITION_MM";
    std::string labelAtTarget = "E/AT_TARGET";
    std::string labelAtUpper = "E/AT_UPPER";
    std::string labelAtLower = "E/AT_LOWER";

    double heightSetpointMM;
    
    bool atUpperBound();
    bool atLowerBound();

    double mmToDegrees(double mm);
    double degreesToMM(double degrees);
  };
}