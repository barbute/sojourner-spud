#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// A global instance of the pilot controller
controller pilotController;

// VEXcode device constructors
motor leftDriveMotor = motor(PORT1, ratio18_1, true);
motor rightDriveMotor = motor(PORT10, ratio18_1, false);
inertial inertialSensor = inertial(PORT13);
smartdrive robotDrive = smartdrive(leftDriveMotor, rightDriveMotor,
                                   inertialSensor, 319.19, 320, 130, mm, 1);

motor clawMotor = motor(PORT3, ratio18_1, false);
motor armMotor = motor(PORT8, ratio18_1, true);

distance distanceSensor = distance(PORT11);
optical opticalSensor = optical(PORT12);

// VEXcode generated functions

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  Brain.Screen.print("Device initialization...");
  Brain.Screen.setCursor(2, 1);
  // calibrate the drivetrain inertial
  wait(200, msec);
  inertialSensor.startCalibration(1);
  Brain.Screen.print("Calibrating Gyro for Drivetrain");
  // wait for the inertial calibration process to finish
  while (inertialSensor.isCalibrating()) {
    wait(25, msec);
  }
  // reset the screen now that the calibration is complete
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  wait(50, msec);
  Brain.Screen.clearScreen();
}