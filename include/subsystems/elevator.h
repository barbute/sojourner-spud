// Copyright (c) 2025 barbute
// 
// This file is part of sojourner-spud and is licensed under the MIT License.
// See the LICENSE file in the root of the project for more information.
//
// File: elevator.h
// Description: Subsystem to interface with a single-stage linear-mechanism

#pragma once

#include "lib/subsystem.h"
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
    void setPowerPercent(double powerPercent);

    double getPositionMM();
    bool atTarget();

  private:
    vex::motor& motor;
    vex::digital_in& limitSwitchUpper;
    vex::digital_in& limitSwitchLower;

    double heightSetpointMM;
    const double toleranceMM = 1;

    bool atUpperBound();
    bool atLowerBound();
  };
}