using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor driveFRight;
extern motor driveMRight;
extern motor driveBRight;

extern motor driveFLeft;
extern motor driveMLeft;
extern motor driveBLeft;

extern motor frontLift;

extern motor intakeRings;

extern digital_out fClaw;
extern digital_out rClaw;

extern limit limit1; 
extern limit limit2;

//extern encoder driveEnc;

extern controller joystick;

extern inertial imu;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );
