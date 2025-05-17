// Copyright (c) 2025 barbute
// 
// This file is part of sojourner-spud and is licensed under the MIT License.
// See the LICENSE file in the root of the project for more information.
//
// File: intake.cpp
// Description: Subsystem to interface with a claw-like intake

#include "subsystems/intake.h"
#include "lib/telemetry.h"

namespace subsystems {
  Intake::Intake(
    std::string& name,
    vex::motor& motorReference,
    vex::digital_in& limitSwitchSurfaceReference
  )
  :lib::Subsystem(name),
    motor(motorReference),
    limitSwitchSurface(limitSwitchSurfaceReference) {}

  void Intake::periodic() {
    printTelemetry();
  }

  void Intake::printTelemetry() {
    lib::Telemetry::writeOutput(labelPosition, getPositionRotations());
    lib::Telemetry::writeOutput(labelTouchingSurface, touchingSurface());
  }

  void Intake::stop() {
    motor.stop();
  }

  void Intake::setPositionRotations(double targetPositionRotations, bool blocking) {
    positionSetpointRotations = targetPositionRotations;

    // NOTE the robot program will cease until this action is completed,
    // may need to remove later if blocking becomes an issue
    motor.spinToPosition(positionSetpointRotations, vex::rev, blocking);
  }

  void Intake::setVoltage(vex::directionType direction, double voltage) {
    motor.spin(direction, voltage, vex::volt);
  }

  double Intake::getPositionRotations() {
    return motor.position(vex::rev);
  }

  bool Intake::touchingSurface() {
    return limitSwitchSurface.value() == 1;
  }
}