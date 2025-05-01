/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Jack Demeterio                                            */
/*    Created:      Wed April 30 2025                                         */
/*    Description:  Clawbot (modified with sensors)                           */
/*    Class: Honors Aerospace Engineering                                     */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Drivetrain           drivetrain    1, 10, D
// ClawMotor            motor         3
// ArmMotor             motor         8
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

bool RUN_TELEOP = true;

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  if (RUN_TELEOP) {
    while (true) {
      robotDrive.arcade(pilotController.Axis3.position(), pilotController.Axis1.position(), percent);

      if (pilotController.ButtonX.pressing()) {
        armMotor.spin(forward, 4, volt);
      } else if (pilotController.ButtonB.pressing()) {
        armMotor.spin(reverse, 4, volt);
      } 
      else {
        armMotor.stop();
      }
  
      if (pilotController.ButtonY.pressing()) {
        clawMotor.spin(forward, 4, volt);
      } else if (pilotController.ButtonA.pressing()) {
        clawMotor.spin(reverse, 4, volt);
      } else {
        clawMotor.stop();
      }

      // Brain.Screen.print("ARM POSITION" + armMotor.position(degrees));
  
      wait(5, msec);
    }
  } else {
    robotDrive.driveFor(forward, 12, inches);
    robotDrive.turnFor(left, 90, degrees);
    robotDrive.stop();
    wait(5, msec);
  }

  return 0;
}
