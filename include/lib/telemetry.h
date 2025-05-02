// Copyright (c) 2025 barbute
// 
// This file is part of sojourner-spud and is licensed under the MIT License.
// See the LICENSE file in the root of the project for more information.
//
// File: telemetry.h
// Description: Simple file for printing telemetry to console or other sources.

#pragma once

#include <string>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "vex.h"

namespace lib {
  class Telemetry {
  public:
    static void writeOutput(std::string& label, double output);
    static void writeOutput(std::string& label, float output);
    static void writeOutput(std::string& label, int output);
    static void writeOutput(std::string& label, bool output);
    static void writeOutput(std::string& label, std::string& output);

    // Clear the brain's screen
    static void clear();

  private:
    static int currentLine;

    static void writeToBrainScreen(std::string& label, double output);
    static void writeToBrainScreen(std::string& label, float output);
    static void writeToBrainScreen(std::string& label, int output);
    static void writeToBrainScreen(std::string& label, bool output);
    static void writeToBrainScreen(std::string& label, std::string& output);
  };
}