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
