// Copyright (c) 2025 barbute
// 
// This file is part of sojourner-spud and is licensed under the MIT License.
// See the LICENSE file in the root of the project for more information.
//
// File: intake.h
// Description: Subsystem to interface with a claw-like intake

#pragma once

#include "lib/subsystem.h"
#include "vex.h"

namespace subsystems {
  class Intake : public lib::Subsystem {
  public:
    Intake(
      std::string& name,
      vex::motor& motorReference,
      vex::digital_in& limitSwitchSurfaceReference);
    
    void periodic() override;
    void printTelemetry() override;
    void stop() override;

    void setPosition(double targetPositionRotations);
    void setVoltage(vex::directionType direction, double voltage);

    double getPositionRotations();
    bool touchingSurface();
    
  private:
    vex::motor& motor;
    // Limit switch on bottom of claw to detect when it is close to touching
    // a surface
    vex::digital_in& limitSwitchSurface;

    std::string labelPosition = lib::Subsystem::NAME + "/POSITION_DEGREES";
    std::string labelTouchingSurface = lib::Subsystem::NAME + "/TOUCHING_SURFACE";
  };
}