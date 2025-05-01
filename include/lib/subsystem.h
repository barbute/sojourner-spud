// Copyright (c) 2025 barbute
// 
// This file is part of sojourner-spud and is licensed under the MIT License.
// See the LICENSE file in the root of the project for more information.
//
// File: subsystem.h
// Description: Abstract base class for robot subsystems.

#include <string>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

namespace lib {
  class Subsystem {
    public:
      const std::string NAME;

      Subsystem(const std::string& name) : NAME(name) {}

      virtual void periodic() = 0;

      virtual void printTelemetry() = 0;

      virtual void stop() = 0;
  };
}
