#ifndef PAMI_UTILS_H
#define PAMI_UTILS_H

void enableMotorDrivers();

void disableDrivers();

int getStepsForDistance(float cm);

int getRotationSteps(float angleDeg);

bool readSwitchOnce(int pin);

#endif // PAMI_UTILS_H
