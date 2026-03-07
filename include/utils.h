#ifndef UTILS_H
#define UTILS_H

void enableMotorDrivers();

void disableDrivers();

int getStepsForDistance(float cm);

int getRotationSteps(float angleDeg);

void setServoAngle(int angle, int channel = 0);

#endif // UTILS_H
