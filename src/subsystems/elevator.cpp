// Copyright (c) 2025 barbute
// 
// This file is part of sojourner-spud and is licensed under the MIT License.
// See the LICENSE file in the root of the project for more information.
//
// File: elevator.cpp
// Description: Subsystem to interface with a single-stage linear-mechanism

#include "subsystems/elevator.h"
#include "lib/telemetry.h"

namespace subsystems {
  Elevator::Elevator(
    std::string& name,
    vex::motor& motorReference,
    vex::digital_in& limitSwitchUpperReference,
    vex::digital_in& limitSwitchLowerReference
  )
  : lib::Subsystem(name),
    motor(motorReference),
    limitSwitchUpper(limitSwitchUpperReference),
    limitSwitchLower(limitSwitchLowerReference),
    heightSetpointMM(0.0) {}

  void Elevator::periodic() {
    printTelemetry();
  }

  void Elevator::printTelemetry() {
    lib::Telemetry::writeOutput(Elevator::labelPosition, getPositionMM());
    lib::Telemetry::writeOutput(Elevator::labelAtTarget, atTarget());
    lib::Telemetry::writeOutput(Elevator::labelAtUpper, atUpperBound());
    lib::Telemetry::writeOutput(Elevator::labelAtLower, atLowerBound());
  }

  void Elevator::stop() {
    motor.stop();
  }

  void Elevator::setPositionMM(double targetHeightMM) {
    heightSetpointMM = targetHeightMM;

    if (atUpperBound() && motor.direction() == vex::forward) {
      stop();
    } else if (atLowerBound() && motor.direction() == vex::reverse) {
      stop();
    } else {
      double rotationSetpointDegrees = mmToDegrees(heightSetpointMM);

      // NOTE the robot program will cease until this action is completed,
      // may need to remove later if blocking becomes an issue
      motor.spinToPosition(rotationSetpointDegrees, vex::degrees, true);
    }
  }

  void Elevator::setVoltage(vex::directionType direction, double voltage) {
    if (atUpperBound() && motor.direction() == vex::forward) {
      stop();
    } else if (atLowerBound() && motor.direction() == vex::reverse) {
      stop();
    } else {
      motor.spin(direction, voltage, vex::volt);
    }
  }

  double Elevator::getPositionMM() {
    return degreesToMM(motor.position(vex::degrees));
  }

  bool Elevator::atTarget() {
    return (getPositionMM() - heightSetpointMM) < 1;
  }

  bool Elevator::atUpperBound() {
    return limitSwitchUpper.value() == 1;
  }

  bool Elevator::atLowerBound() {
    return limitSwitchLower.value() == 1;
  }

  double Elevator::mmToDegrees(double mm) {
    return (mm / SPROCKET_CIRCUMFERENCE_MM) * 360.0;
  }

  double Elevator::degreesToMM(double degrees) {
    return (degrees / 360.0) * SPROCKET_CIRCUMFERENCE_MM;
  }
}