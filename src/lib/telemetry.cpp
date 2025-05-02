// Copyright (c) 2025 barbute
// 
// This file is part of sojourner-spud and is licensed under the MIT License.
// See the LICENSE file in the root of the project for more information.
//
// File: telemetry.cpp
// Description: Simple file for printing telemetry to console or other sources.

#include <lib/telemetry.h>
#include <sstream>
#include <iostream>

namespace lib {
  // Manage state of where the cursor on the brain screen is
  int Telemetry::currentLine = 1;

  // lowkey idk if this implementation is cheeks or not, but ChatGPT said it's
  // good and I trust intellisense so should be fine
  void Telemetry::writeOutput(std::string& label, double output) {
    std::stringstream ss;
    ss << label << ": " << output;
    printf("%s\n", ss.str().c_str());
    writeToBrainScreen(label, output);
  }

  void Telemetry::writeOutput(std::string& label, float output) {
    std::stringstream ss;
    ss << label << ": " << output;
    printf("%s\n", ss.str().c_str());
    writeToBrainScreen(label, output);
  }

  void Telemetry::writeOutput(std::string& label, int output) {
    std::stringstream ss;
    ss << label << ": " << output;
    printf("%s\n", ss.str().c_str());
    writeToBrainScreen(label, output);
  }

  void Telemetry::writeOutput(std::string& label, std::string& output) {
    std::cout << label + ": " + output;
    writeToBrainScreen(label, output);
  }

  void Telemetry::clear() {
    Brain.Screen.clearScreen();
    currentLine = 1;
  }

  void Telemetry::writeToBrainScreen(std::string& label, double output) {
    Brain.Screen.setCursor(10, currentLine * 20);
    Brain.Screen.print("%s: %.2f", label.c_str(), output);

    currentLine++;
  }
  void Telemetry::writeToBrainScreen(std::string& label, float output) {
    Brain.Screen.setCursor(10, currentLine * 20);
    Brain.Screen.print("%s: %.2f", label.c_str(), output);

    currentLine++;
  }
  void Telemetry::writeToBrainScreen(std::string& label, int output) {
    Brain.Screen.setCursor(10, currentLine * 20);
    Brain.Screen.print("%s: %d", label.c_str(), output);

    currentLine++;
  }
  void Telemetry::writeToBrainScreen(std::string& label, std::string& output) {
    Brain.Screen.setCursor(10, currentLine * 20);
    Brain.Screen.print("%s: %s", label.c_str(), output.c_str());

    currentLine++;
  }
}