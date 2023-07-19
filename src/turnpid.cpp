#include "turnpid.h"

// BEGIN TURN PID
double turnPositionTarget = 0;
double currentPosition;
double turnError = 0;
double lastError;
double derivative;
double integral;
bool isTurnPIDRunning = false;

double integralResetLimit = 60;

vex::timer turnTimer;

vex::timer timeoutTimer;

double dT = 10;

double settledError = 1.5;
double settledDerivative = 0.1;

double customkPT;
double customkDT;

void drive2(double left, double right){
  driveBLeft.spin(directionType::fwd, left, velocityUnits::rpm);
  driveMLeft.spin(directionType::fwd, left, velocityUnits::rpm);
  driveFLeft.spin(directionType::fwd, left, velocityUnits::rpm);
  
  driveBRight.spin(directionType::fwd, right, velocityUnits::rpm);
  driveMRight.spin(directionType::fwd, right, velocityUnits::rpm);
  driveFRight.spin(directionType::fwd, right, velocityUnits::rpm);

  driveBLeft.stop(brakeType::brake);
  driveMLeft.stop(brakeType::brake);
  driveFLeft.stop(brakeType::brake);

  driveBRight.stop(brakeType::brake);
  driveMRight.stop(brakeType::brake);
  driveFRight.stop(brakeType::brake);
}

int turnDrive() {
  double customkPT = 4; //6.5
  double kI = 0.9; //0.01
  double customkDT = 23.935;  //39.3




  while (true) {



    dT = turnTimer.time(timeUnits::msec);
    turnTimer.clear();

    while (!isTurnPIDRunning) {
      wait(10, timeUnits::msec);
    //printf("pid disabled, error: %1.2f\n", turnError);
    }

    currentPosition = imu.angle(rotationUnits::deg); 
    
    turnError = turnPositionTarget - currentPosition;

    turnError *= 3.141592653589793238462 / 180.0;
    turnError = atan2(sin(turnError), cos(turnError));
    turnError *= 180.0 / 3.141592653589793238462;
    
    //printf("End result %4.2f\n", turnError);
    //printf("Velocity %4.2f\n",driveBLeft.velocity(velocityUnits::rpm));
    integral += turnError * dT;

    if (fabs(integral) > integralResetLimit) {
      integral = 0;
    }

    derivative = turnError - lastError;

    if(turnError > 5){
    kI = 0.0;
  }
  
    

    double power = (customkPT * turnError) + (kI * integral) + (customkDT * derivative); // [-100, 100]
    //printf("Power %4.2f\n", power);

 
    driveBLeft.spin(directionType::fwd, 120.0 * power, voltageUnits::mV);
    driveMLeft.spin(directionType::fwd, 120.0 * power, voltageUnits::mV);
    driveFLeft.spin(directionType::fwd, 120.0 * power, voltageUnits::mV);

    driveBRight.spin(directionType::fwd, -120.0 * power, voltageUnits::mV);
    driveMRight.spin(directionType::fwd, -120.0 * power, voltageUnits::mV);
    driveFRight.spin(directionType::fwd, -120.0 * power, voltageUnits::mV);

    printf("error: %1.2f, power: %1.2f\n", turnError, power);


    lastError = turnError;
 /* if (fabs(turnError) < settledError && fabs(derivative) < settledDerivative) {
      disableDrivePID();
      driveBLeft.stop(brakeType::brake);
      driveFLeft.stop(brakeType::brake);
      driveBRight.stop(brakeType::brake);
      driveFRight.stop(brakeType::brake);
      wait(10, timeUnits::msec);
  }*/
   
    wait(10, timeUnits::msec);
  }
}

  double driveBLeftVelocity = driveBLeft.velocity(velocityUnits::rpm);
  double driveMLeftVelocity = driveMLeft.velocity(velocityUnits::rpm);
  double driveFLeftVelocity = driveFLeft.velocity(velocityUnits::rpm);

  double driveBRightVelocity = driveBRight.velocity(velocityUnits::rpm);
  double driveMRightVelocity = driveMRight.velocity(velocityUnits::rpm);
  double driveFRightVelocity = driveFRight.velocity(velocityUnits::rpm);

  double gyroRate = imu.gyroRate (axisType::xaxis, velocityUnits::dps);

  double currentVelocity = (driveBLeftVelocity + driveMLeftVelocity + driveFLeftVelocity + driveBRightVelocity + driveMRightVelocity + driveFRightVelocity)/4;

 bool isDriveSettled(double positionError, double velocityError, double turnVelocity, double timeout) {

  double turnErrorEnd = turnPositionTarget - imu.angle(rotationUnits::deg);

    turnErrorEnd *= 3.141592653589793238462 / 180.0;
    turnErrorEnd = atan2(sin(turnErrorEnd), cos(turnErrorEnd));
    turnErrorEnd *= 180.0 / 3.141592653589793238462;
    
  return ((((fabs(turnErrorEnd) < positionError) && (fabs(currentVelocity)) < velocityError)) && ((fabs(gyroRate)) < turnVelocity)) || (timeoutTimer.time() > timeout);
}

  void disableDrivePID() {
  isTurnPIDRunning = false;
  wait(10, timeUnits::msec);
  drive2(0, 0);
  driveBLeft.setMaxTorque(100, percentUnits::pct);
  driveMLeft.setMaxTorque(100, percentUnits::pct);
  driveFLeft.setMaxTorque(100, percentUnits::pct);

  driveBRight.setMaxTorque(100, percentUnits::pct);
  driveMRight.setMaxTorque(100, percentUnits::pct);
  driveFRight.setMaxTorque(100, percentUnits::pct);

}

void disableDrivePIDdrive() {
  isTurnPIDRunning = false;
  driveBLeft.setMaxTorque(100, percentUnits::pct);
  driveMLeft.setMaxTorque(100, percentUnits::pct);
  driveFLeft.setMaxTorque(100, percentUnits::pct);

  driveBRight.setMaxTorque(100, percentUnits::pct);
  driveMRight.setMaxTorque(100, percentUnits::pct);
  driveFRight.setMaxTorque(100, percentUnits::pct);
}

void enableTurnPID() {
  driveBLeft.setMaxTorque(100, percentUnits::pct);
  driveMLeft.setMaxTorque(100, percentUnits::pct);
  driveFLeft.setMaxTorque(100, percentUnits::pct);

  driveBRight.setMaxTorque(100, percentUnits::pct);
  driveMRight.setMaxTorque(100, percentUnits::pct);
  driveFRight.setMaxTorque(100, percentUnits::pct);
  isTurnPIDRunning = true;
}

  void setTurnPID(int target) {
    double currentPosition = imu.angle(rotationUnits::deg); 
    turnError = turnPositionTarget - currentPosition;
    turnPositionTarget = target;
    timeoutTimer.clear();
    enableTurnPID();

}

  void waitUntilSettled(double positionError, double velocityError, double turnVelocity, double timeout) {

  while (!isDriveSettled(positionError, velocityError, turnVelocity, timeout)) {

    wait(10, timeUnits::msec);
    }
    printf("Drive Settled \n");
    disableDrivePID();
    drive2(0,0);

}
