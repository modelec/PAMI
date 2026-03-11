#ifndef PAMI_UTILS_H
#define PAMI_UTILS_H

void enableMotorDrivers();

void disableDrivers();

int getStepsForDistance(float cm);

int getRotationSteps(float angleDeg);

#endif // PAMI_UTILS_H
