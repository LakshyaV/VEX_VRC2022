#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor driveFRight = motor(PORT2, ratio18_1, false);
motor driveMRight = motor(PORT4, ratio18_1, false); 
motor driveBRight = motor(PORT6, ratio18_1, false);

motor driveFLeft = motor(PORT11, ratio18_1, true);
motor driveMLeft = motor(PORT13, ratio18_1, true);
motor driveBLeft = motor(PORT16, ratio18_1, true);


motor frontLift = motor(PORT15, ratio6_1, true); //front lift

motor intakeRings = motor(PORT7, ratio36_1, false); //ring intake

//encoder driveEnc = encoder(Brain.ThreeWirePort.G);

digital_out fClaw = digital_out(Brain.ThreeWirePort.A);
digital_out rClaw = digital_out(Brain.ThreeWirePort.B);

inertial imu = inertial(PORT17);

limit limit1 = limit(Brain.ThreeWirePort.D);
limit limit2 = limit(Brain.ThreeWirePort.E);

controller joystick = controller(controllerType::primary);

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}
