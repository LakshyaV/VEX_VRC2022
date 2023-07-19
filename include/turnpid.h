#pragma once

#include "vex.h"

extern double turnPositionTarget;

int turnDrive();
extern double customkPT;
extern double customkDT;

bool isDriveSettled();

void disableDrivePID();

void disableDrivePIDdrive();

void enableDrivePID();

void waitUntilSettled(double positionErorr, double velocityError, double turnVelocity, double timeout);

void setTurnPID(int target);
