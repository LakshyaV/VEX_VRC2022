#include "drivingpid.h"

//BEGIN DRIVE STRAIGHT PID
//Driving variables
double driveBLeftPosition;
double driveMLeftPosition;
double driveFLeftPosition;

double driveBRightPosition;
double driveMRightPosition;
double driveFRightPosition;

double driveEncPosition;
double currentStraightPosition;
double startError;

double straightError;
double lastStraightError;
double straightTarget;

double straightDerivative;
double straightIntegral;

double integralStraightResetLimit = 60;

//angle hold variables
double currentangle;
double angleError;
double anglePositionTarget;
double lastAngleError;

double anglederivative;
double angleintegral;
double startAngle;

double integralAngleResetLimit = 10;
double customkP;
double customkD;

void drive(double left, double right){

  driveBLeft.spin(directionType::fwd,left, velocityUnits::rpm);
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


bool isStraightPIDRunning = false;

vex::timer angleTimer;

vex::timer timeoutTimerStraight;


double SdT = 10;

double settledStraightError = 1.5;
double settledStraightDerivative = 0.1;



int driveStraight(){
  //straight movement PID
  double customkP = 8.0;
  double kI = 0.1;
  double customkD = 0.1;

  double anglekP = 1.3;
  double anglekI = 0.2;
  double anglekD = 0.0;

  double AdT = 10;


  while(true){

    SdT = angleTimer.time(timeUnits::msec);
    angleTimer.clear();

    AdT = angleTimer.time(timeUnits::msec);
    angleTimer.clear();

    while (!isStraightPIDRunning) {
      wait(10, timeUnits::msec);
      //printf("pid disabled, error: %1.2f\n", straightError)
    }

    driveBLeftPosition = driveBLeft.position(rotationUnits::deg);
    driveMLeftPosition = driveMLeft.position(rotationUnits::deg);
    driveFLeftPosition = driveFLeft.position(rotationUnits::deg);
    driveBRightPosition = driveBRight.position(rotationUnits::deg);
    driveMRightPosition = driveMRight.position(rotationUnits::deg);
    driveFRightPosition = driveFRight.position(rotationUnits::deg);

    currentStraightPosition = (driveBLeftPosition + driveMLeftPosition + driveFLeftPosition + driveBRightPosition + driveMRightPosition + driveFRightPosition)/6;
    currentStraightPosition = 3.141592653589793238 * 3.25 / 225.0 * currentStraightPosition * 1; //number was 3.25
    currentStraightPosition = currentStraightPosition - startError;

    straightError =  straightTarget - currentStraightPosition;
    printf("Error %4.2f\n", straightError);
    printf("Time %lu\n", timeoutTimerStraight.time());
    straightDerivative = straightError - lastStraightError;

    if (fabs(straightIntegral) > integralStraightResetLimit) {
      straightIntegral = 0;
    }

    straightIntegral += straightError * SdT;

    if(straightError > 15){
      kI = 0.0;
    }
    
/*
    if(fabs(straightError) > 38){
      driveBLeft.setMaxTorque(100, percentUnits::pct); //number was 40
      driveMLeft.setMaxTorque(100, percentUnits::pct);
      driveFLeft.setMaxTorque(100, percentUnits::pct);
      driveBRight.setMaxTorque(100, percentUnits::pct);
      driveMRight.setMaxTorque(100, percentUnits::pct);
      driveFRight.setMaxTorque(100, percentUnits::pct);
    }
*/
    

    

    double power = (straightError * customkP + straightDerivative * customkD + straightIntegral * kI);
    printf(" power %4.2f\n", power);

    if(power > 120){
      power = 85;

    }
    
    if(power < -120){
      power = -85;
    }

    //angle correction
    double currentAngleIMU = imu.angle(rotationUnits::deg);

    //double currentAngleDrive = (driveBLeftPosition + driveMLeftPosition + driveFLeftPosition) - (driveBRightPosition + driveMRightPosition + driveFRightPosition);

    double currentangle = (currentAngleIMU);

    angleError = startAngle - currentangle;

    angleError *= 3.141592653589793238462 / 180.0;
    angleError = atan2(sin(angleError), cos(angleError));
    angleError *= 180.0 / 3.141592653589793238462;

    anglederivative = angleError - lastAngleError;

    if (fabs(angleintegral) > integralAngleResetLimit) {
      angleintegral = 0;
    }
    
    angleintegral += angleError * AdT;
    
    printf("Angle Error %4.2f\n", angleError);

    double anglepower = (anglekP * angleError) + (anglekI * angleintegral) + (anglekD * anglederivative);
    printf("angle error %4.2f\n", angleError);
          //printf("straight Error %4.2f\n", straightError);


    driveBLeft.spin(directionType::fwd, (power + anglepower) * 120, voltageUnits::mV);
    driveMLeft.spin(directionType::fwd, (power + anglepower) * 120, voltageUnits::mV);
    driveFLeft.spin(directionType::fwd, (power + anglepower) * 120, voltageUnits::mV);
    driveBRight.spin(directionType::fwd, (power - anglepower) * 120, voltageUnits::mV);
    driveMRight.spin(directionType::fwd, (power - anglepower) * 120, voltageUnits::mV);
    driveFRight.spin(directionType::fwd, (power - anglepower) * 120, voltageUnits::mV);


    lastStraightError = straightError;
    lastAngleError = angleError;
  }
}

double driveBLeftSVelocity = driveBLeft.velocity(velocityUnits::rpm);
double driveMLeftSVelocity = driveMLeft.velocity(velocityUnits::rpm);
double driveFLeftSVelocity = driveFLeft.velocity(velocityUnits::rpm);
double driveBRightSVelocity = driveBRight.velocity(velocityUnits::rpm);
double driveMRightSVelocity = driveMRight.velocity(velocityUnits::rpm);
double driveFRightSVelocity = driveFRight.velocity(velocityUnits::rpm);

double gyroRate2 = imu.gyroRate (axisType::xaxis, velocityUnits::dps);


double currentSVelocity = (driveBLeftSVelocity + driveMLeftSVelocity + driveFLeftSVelocity + driveBRightSVelocity + driveMRightSVelocity + driveFRightSVelocity)/6;

bool isStraightSettled(double positionError, double velocityError, double posAngleError, double turnVelocity, double timeout) {
  return ((((fabs(straightError) < positionError) && (fabs(currentSVelocity)) < velocityError) && (fabs(angleError) < posAngleError) && ((fabs(gyroRate2)) < turnVelocity))) || (timeoutTimerStraight.time() > timeout);
}

void disableStraightPID() {
  isStraightPIDRunning = false;
  wait(10,timeUnits::msec);
  drive(0,0);
  driveBLeft.setMaxTorque(100, percentUnits::pct);
  driveMLeft.setMaxTorque(100, percentUnits::pct);
  driveFLeft.setMaxTorque(100, percentUnits::pct);

  driveBRight.setMaxTorque(100, percentUnits::pct);
  driveMRight.setMaxTorque(100, percentUnits::pct);
  driveFRight.setMaxTorque(100, percentUnits::pct);
}

void disableStraightPIDdrive(){
  isStraightPIDRunning = false;
  driveBLeft.setMaxTorque(100, percentUnits::pct);
  driveMLeft.setMaxTorque(100, percentUnits::pct);
  driveFLeft.setMaxTorque(100, percentUnits::pct);

  driveBRight.setMaxTorque(100, percentUnits::pct);
  driveMRight.setMaxTorque(100, percentUnits::pct);
  driveFRight.setMaxTorque(100, percentUnits::pct);
}


void enableStraightPID() {
  /*driveBLeft.setMaxTorque(100, percentUnits::pct);
  driveMLeft.setMaxTorque(100, percentUnits::pct);
  driveFLeft.setMaxTorque(100, percentUnits::pct);

  driveBRight.setMaxTorque(100, percentUnits::pct);
  driveMRight.setMaxTorque(100, percentUnits::pct);
  driveFRight.setMaxTorque(100, percentUnits::pct);
  */
  isStraightPIDRunning = true;
}

void setStraightPID(int target) {
  driveBLeft.resetPosition();
  driveMLeft.resetPosition();
  driveFLeft.resetPosition();

  driveBRight.resetPosition();
  driveMRight.resetPosition();
  driveFRight.resetPosition();
  //driveEnc.resetRotation();

  straightTarget = target;
  double currentangle = imu.angle(rotationUnits::deg); 
  startAngle = currentangle;

  driveBLeftPosition = driveBLeft.position(rotationUnits::deg);
  driveMLeftPosition = driveMLeft.position(rotationUnits::deg);
  driveFLeftPosition = driveFLeft.position(rotationUnits::deg);

  driveBRightPosition = driveBRight.position(rotationUnits::deg);
  driveMRightPosition = driveMRight.position(rotationUnits::deg);
  driveFRightPosition = driveFRight.position(rotationUnits::deg);
  //driveEncPosition = driveEnc.rotation(rotationUnits::deg);

  startError = (driveBLeftPosition + driveMLeftPosition + driveFLeftPosition + driveBRightPosition + driveMRightPosition + driveFRightPosition)/6;
  startError = 3.141592653589793238462 * 3.25 / 225.0 * currentStraightPosition * 1;

  currentStraightPosition = (driveBLeftPosition + driveMLeftPosition + driveFLeftPosition + driveBRightPosition + driveMRightPosition + driveFRightPosition)/6;
  //currentStraightPosition = driveEncPosition;
  currentStraightPosition = 3.141592653589793238462 * 3.25 / 225.0 * currentStraightPosition * 1;
  timeoutTimerStraight.clear();  
  enableStraightPID();
  }

void waitUntilDriveSettled(double positionError, double velocityError, double posAngleError, double turnVelocity, double timeout) {
  wait(50, timeUnits::msec);
  while (!isStraightSettled(positionError, velocityError, posAngleError, turnVelocity, timeout)) {
    wait(10, timeUnits::msec);
  }
  printf("Drive Settled \n");
  disableStraightPID();
  drive(0,0);
}
