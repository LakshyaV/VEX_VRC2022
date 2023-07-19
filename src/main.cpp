/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Motor10              motor         10              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "drivingpid.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void intake(int velocity){
intakeRings.spin(directionType::fwd, velocity, velocityUnits::rpm);
}

void frontClaw(bool value){
fClaw.set(value);
}

void backClaw(bool value){
rClaw.set(value);
}

void Lift(double rotation){
frontLift.rotateTo(rotation, rotationUnits::deg);
}

void wait(int time){
  wait(time, timeUnits::msec);
}

int autonState = 0;
bool buttonUnpressed = true;
bool frontClawState = false;
bool backClawState = false;
void frontClawToggle(){
  frontClawState = !frontClawState;
  if(frontClawState){
      fClaw.set(1);
    }
    else if(!frontClawState){
      fClaw.set(0);
    }
}

void backClawToggle(){
  backClawState = !backClawState;
  if(backClawState){
      rClaw.set(1);
    }
    else if(!backClawState){
      rClaw.set(0);
    }
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  task drivePIDTask = task(turnDrive);
  task driveStraightPIDTask = task(driveStraight);
  joystick.ButtonL1.pressed(frontClawToggle);
  joystick.ButtonL2.pressed(backClawToggle);

  imu.calibrate();
  driveBLeft.resetPosition();
  driveMLeft.resetPosition();
  driveFLeft.resetPosition();
  driveBRight.resetPosition();
  driveMRight.resetPosition();
  driveFRight.resetPosition();
  //Brain.Screen.print("hello");
 // driveEnc.resetRotation();
  while(imu.isCalibrating()){
    wait(10, timeUnits::msec);
  }
  wait(200, timeUnits::msec);

  while(true)
    {
        if(limit1.pressing() && buttonUnpressed)
        {
            autonState++;
            joystick.Screen.print(autonState);
            if(autonState == 7){
            autonState = 0;
            buttonUnpressed = false;
            }
        }
        if(limit2.pressing() && buttonUnpressed)
        {
            autonState--;
            joystick.Screen.print(autonState);
            if(autonState == -1){
            autonState = 7;
            buttonUnpressed = false;
            }
        }
        if(!limit1.pressing() && !limit2.pressing()){ 
          buttonUnpressed = true;

        wait(7);
        }
    }




  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  

    uint32_t startTime = timer::system();
    /*
    frontClaw(1);
    frontLift.rotateTo(150, rotationUnits::deg, 100, velocityUnits::rpm);
    wait(2000);
    customkPT = 0.1;
    customkDT = 1.35; 
    setTurnPID(90);
    waitUntilSettled(1, 0.5, 0.1, 10000);
    uint32_t finishedTime = timer::system() - startTime;
    printf("Finished: %lu\n", finishedTime);
*/

    //setStraightPID(30);
    //waitUntilDriveSettled(1, 1, 1, 1, 10000);

    
    //if(autonState == 0){

    //Main Awp Line Auton
    
    customkP = 40.0;
   driveBLeft.setMaxTorque(1, currentUnits::amp);
  driveMLeft.setMaxTorque(1, currentUnits::amp);
  driveFLeft.setMaxTorque(1, currentUnits::amp);

  driveBRight.setMaxTorque(1, currentUnits::amp);
  driveMRight.setMaxTorque(1, currentUnits::amp);
  driveFRight.setMaxTorque(1, currentUnits::amp);

    setStraightPID(45);
 
  wait(50);

  driveBLeft.setMaxTorque(2, currentUnits::amp);
  driveMLeft.setMaxTorque(2, currentUnits::amp);
  driveFLeft.setMaxTorque(2, currentUnits::amp);

  driveBRight.setMaxTorque(2, currentUnits::amp);
  driveMRight.setMaxTorque(2, currentUnits::amp);
  driveFRight.setMaxTorque(2, currentUnits::amp);
wait(80);


  driveBLeft.setMaxTorque(100, percentUnits::pct);
  driveMLeft.setMaxTorque(100, percentUnits::pct);
  driveFLeft.setMaxTorque(100, percentUnits::pct);

  driveBRight.setMaxTorque(100, percentUnits::pct);
  driveMRight.setMaxTorque(100, percentUnits::pct);
  driveFRight.setMaxTorque(100, percentUnits::pct);
  
    wait(750);
    frontClaw(1);
    uint32_t finishedTime = timer::system() - startTime;

    printf("Finished: %lu\n", finishedTime);
    waitUntilDriveSettled(3, 3, 3, 3, 1400);
    customkP = 8.0;
    wait(200);
    frontLift.startRotateTo(1000, rotationUnits::deg);
    setStraightPID(-30);
    //frontLift.startRotateTo(-150, rotationUnits::deg);
    waitUntilDriveSettled(1, 1, 1, 1, 1000);
    setTurnPID(-90);
    waitUntilSettled(1, 1, 0.5, 1000);
    wait(400);
    setStraightPID(-15);
    waitUntilDriveSettled(2, 2, 2, 2, 1000);
    backClaw(1);
    intake(40); 
    }
    
    /*
    if(autonState == 1){
    
    //Main Two Goal
    
    customkP = 40.0;
    setStraightPID(45);
    wait(800);
    frontClaw(1);
    uint32_t finishedTime = timer::system() - startTime;

    printf("Finished: %lu\n", finishedTime);
    waitUntilDriveSettled(3, 3, 3, 3, 1400);
    customkP = 40.0;
    wait(200);
    frontLift.startRotateTo(1000, rotationUnits::deg);
    setStraightPID(-10);
    //frontLift.startRotateTo(-150, rotationUnits::deg);
    waitUntilDriveSettled(1, 1, 1, 1, 1000);
    setTurnPID(104);
    waitUntilSettled(1, 1, 0.5, 1000);
    wait(400);
    setStraightPID(-30);
    waitUntilDriveSettled(2, 2, 2, 2, 1000);
    backClaw(1);
    intake(40);
    setStraightPID(30);
    waitUntilDriveSettled(2, 2, 2, 2, 1000);
    }
    */
    /*
    if(autonState == 2){
    //Simple Left Side
    
    frontLift.rotateTo(200, rotationUnits::deg);
    wait(100);
    setStraightPID(-15);
    waitUntilDriveSettled( 2, 2, 2, 2, 1500);
    backClaw(1);
    intake(40);
    wait(2000);
    intake(50);
    setStraightPID(25);
    waitUntilDriveSettled(1, 1, 1, 1, 1000);
    }
    */
    /*
    if(autonState == 3){
    //Left Neutral Goal
    
    setStraightPID(45);
    waitUntilDriveSettled(1, 1, 1, 1, 1500);
    frontClaw(1);
    wait(100);
    frontLift.startRotateTo(150, rotationUnits::deg);
    setStraightPID(-50);
    waitUntilDriveSettled(1, 1, 1, 1, 1500);
    setTurnPID(-94);
    waitUntilSettled( 1, 1, 1, 1000);
    setStraightPID(-20);
    waitUntilDriveSettled(2, 2, 2, 2, 1000);
    backClaw(1);
    wait(500);
    intake(40);
    setStraightPID(20);
    waitUntilDriveSettled(1, 1, 1, 1, 1500);
    }
    */
    /*
    if(autonState == 4){
    //Middle Goal
    
    customkP = 40.0;
    setStraightPID(56);
    wait(1000);
    frontClaw(1);
    uint32_t finishedTime = timer::system() - startTime;

    printf("Finished: %lu\n", finishedTime);
    waitUntilDriveSettled(3, 3, 3, 3, 1400);
    customkP = 8.0;
    wait(200);

    frontLift.startRotateTo(1300, rotationUnits::deg);
    setStraightPID(-35);
    waitUntilDriveSettled(2, 2, 2, 2, 1000);
    setTurnPID(-63);
    waitUntilSettled(1, 1, 1, 700);
    wait(500);
    setStraightPID(-36);
    waitUntilDriveSettled(2, 2, 2, 2, 1000);
    backClaw(1);
    intake(40);
    }
    */
    /*
    if(autonState == 6){
    //Prog Skills
    
    imu.setHeading(90, rotationUnits::deg);

    setStraightPID(15);
    waitUntilDriveSettled(2, 2, 2, 2, 500);
    wait(1000);
    frontClaw(1);
    frontLift.startRotateTo(150, rotationUnits::deg);
    wait(500);
    setStraightPID(-15);
    waitUntilDriveSettled(1, 1, 1, 1, 500);
    setTurnPID(14);
    waitUntilSettled(1, 1, 1, 500);
    setStraightPID(50);
    waitUntilDriveSettled( 2, 2, 2, 2, 1000);
    setStraightPID(80);
    waitUntilDriveSettled(2, 2, 2, 2, 1500);
    frontLift.startRotateTo(-150, rotationUnits::deg);
    frontClaw(0);
    setStraightPID(-2);
    waitUntilDriveSettled(1, 1, 1, 1, 500);
    setTurnPID(-70);
    waitUntilSettled(1, 1, 1, 1000);
    setStraightPID(30);
    waitUntilDriveSettled(2, 2, 2, 2, 1000);
    frontClaw(1);
    frontLift.startRotateTo(150, rotationUnits::deg);
    setStraightPID(-35);
    waitUntilDriveSettled(2, 2, 2, 2, 500);
    setTurnPID(-50);
    waitUntilSettled(1, 1, 1, 1000);
    setStraightPID(-130);
    waitUntilDriveSettled(1, 2, 2, 2, 2500);
    wait(200);
    setStraightPID(3);
    waitUntilDriveSettled(2, 2, 2, 2, 500);
    setTurnPID(170);
    waitUntilSettled(1, 1, 1, 1000);
    frontClaw(0);
    wait(1000);
    setStraightPID(-60);
    waitUntilDriveSettled(2, 2, 2, 2, 2500);
    }
    */
  


  // ..........................................................................
  
//}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  joystick.ButtonL1.pressed(frontClawToggle);
  joystick.ButtonL2.pressed(backClawToggle);
  while (true) {

    disableDrivePIDdrive();
    disableStraightPIDdrive();
    
    //brake mode
    
    driveBLeft.setStopping(brakeType::coast);
    driveMLeft.setStopping(brakeType::coast);
    driveFLeft.setStopping(brakeType::coast);
    driveBRight.setStopping(brakeType::coast);
    driveMRight.setStopping(brakeType::coast);
    driveFRight.setStopping(brakeType::coast);
        
    frontLift.setStopping(brakeType::hold);

    //arcade control
    
    int leftStickY = joystick.Axis3.position();
    int leftStickX = joystick.Axis4.position();

    

    driveFLeft.spin(directionType::fwd, 120.0 * (leftStickY + leftStickX), voltageUnits::mV);
    driveFRight.spin(directionType::fwd, 120.0 * (leftStickY - leftStickX), voltageUnits::mV);
    driveMLeft.spin(directionType::fwd, 120.0 * (leftStickY + leftStickX), voltageUnits::mV);
    driveMRight.spin(directionType::fwd, 120.0 * (leftStickY - leftStickX), voltageUnits::mV);
    driveBLeft.spin(directionType::fwd, 120.0 * (leftStickY + leftStickX), voltageUnits::mV);
    driveBRight.spin(directionType::fwd, 120.0 * (leftStickY - leftStickX), voltageUnits::mV);
    
    //tank control
    /*
    int leftStickY = joystick.Axis3.position();
    int rightStickY = joystick.Axis2.position();

    driveFLeft.spin(directionType::fwd, 120.0 * (leftStickY), voltageUnits::mV);
    driveFRight.spin(directionType::fwd, 120.0 * (rightStickY), voltageUnits::mV);
    driveMLeft.spin(directionType::fwd, 120.0 * (leftStickY), voltageUnits::mV);
    driveMRight.spin(directionType::fwd, 120.0 * (rightStickY), voltageUnits::mV);
    driveBLeft.spin(directionType::fwd, 120.0 * (leftStickY), voltageUnits::mV);
    driveBRight.spin(directionType::fwd, 120.0 * (rightStickY), voltageUnits::mV);
    */

    //intake & roller control
    int buttonR1 = joystick.ButtonR1.pressing(); // 1 if pressed, 0 if not pressed
    int buttonR2 = joystick.ButtonR2.pressing();
    //int buttonL1 = joystick.ButtonL1.pressing(); // 1 if pressed, 0 if not pressed
    //int buttonL2 = joystick.ButtonL2.pressing();

    /*intakeRight.spin(directionType::fwd, 12000 * (buttonR1 - buttonR2), voltageUnits::mV);
    intakeLeft.spin(directionType::fwd, 12000 *  (buttonR1 - buttonR2), voltageUnits::mV);

    rollerBott.spin(directionType::fwd, 12000 * (buttonR1 - buttonR2), voltageUnits::mV);
    rollerTop.spin(directionType::fwd, 12000 * (buttonL1 - buttonL2), voltageUnits::mV);
*/
    if(joystick.ButtonR1.pressing()){
      intakeRings.spin(directionType::fwd, 12000 * (buttonR1 - buttonR2), voltageUnits::mV);

    } else if (joystick.ButtonR2.pressing()) {
      intakeRings.spin(directionType::fwd, 12000 * (buttonR1 - buttonR2), voltageUnits::mV);
    
    } else {
      intakeRings.stop(brakeType::brake);
    }

    int rightStickY = joystick.Axis2.position();

    //lift
    if(abs(joystick.Axis2.position()) > 10){
      rightStickY = joystick.Axis2.position();
    }
    else {
      rightStickY = 0;
    }

    frontLift.spin(directionType::fwd, 120.0 * (rightStickY), voltageUnits::mV);
    //old
    /*
    if(joystick.ButtonL1.pressing()){
      frontLift.spin(directionType::fwd, 12000 * (buttonL1 - buttonL2), voltageUnits::mV);
    } else if (joystick.ButtonL2.pressing()) {
      frontLift.spin(directionType::fwd, 12000 * (buttonL1 - buttonL2), voltageUnits::mV);
    } else {
      frontLift.stop(brakeType::hold);
    }
    */
    
    
    
/*
    if(joystick.ButtonX.pressing()){
      fClaw.set(1);
    } else if (joystick.ButtonY.pressing()) {
      fClaw.set(0);
    }
*/
/*
    if(joystick.ButtonA.pressing()){
      rClaw.set(1);
    } else if (joystick.ButtonB.pressing()) {
      rClaw.set(0);
    } 
*/

    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(10, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
