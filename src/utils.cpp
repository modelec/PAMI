#include <Arduino.h>
#include "utils.h"
#include "main.h"

void enableMotorDrivers() {
    digitalWrite(M1_ENABLE_PIN, LOW); // LOW pour activer le driver
    digitalWrite(M2_ENABLE_PIN, LOW);
}

void disableDrivers() {
    digitalWrite(M1_ENABLE_PIN, HIGH);
    digitalWrite(M2_ENABLE_PIN, HIGH);
}

int getStepsForDistance(float cm) {
    double wheel_circumference_cm = M_PI * (double) WHEEL_DIAMETER;
    double steps_per_cm = ((double) STEPS_PER_REV * (double) MICROSTEPPING) / wheel_circumference_cm;
    int total_steps = (int) round(cm * steps_per_cm);
    int correction = (int) round(1.5 * steps_per_cm);

    return total_steps - correction;
    // return cm * 740; // Valeur calculée à la main pour une vitesse de 5000 :)
}

int getRotationSteps(float angleDeg) {
    double wheelTurnPerRotation = (double) WHEEL_BASE / (double) WHEEL_DIAMETER;
    double microStepsPerRotation = wheelTurnPerRotation * (double) STEPS_PER_REV * (double) MICROSTEPPING;
    double microStepsForAngle = microStepsPerRotation * (angleDeg / 360.0);

    return (int) round(microStepsForAngle);
}

int readSwitchOnce(int pin) {
    pinMode(SWITCH_PIN, INPUT_PULLUP);
    delay(10);
    const bool switchState = digitalRead(SWITCH_PIN);
    pinMode(SWITCH_PIN, INPUT); // retire le pull-up
    return switchState;
}
