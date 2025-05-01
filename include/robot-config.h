using namespace vex;

extern brain Brain;
extern controller pilotController;

// VEXcode devices
extern smartdrive robotDrive;
extern motor clawMotor;
extern motor armMotor;


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);