#pragma once

#include "vex.h"

int driveStraight();
extern double customkP;
extern double customkD;

void setStraightPID(int target);

bool isStraightSettled();

void disableStraightPID();

void disableStraightPIDdrive();

void enableStraightPID();

void waitUntilDriveSettled(double positionErorr, double velocityError, double posAngleError, double turnVelocity, double timeout);
