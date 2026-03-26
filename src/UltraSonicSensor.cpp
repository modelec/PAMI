//
// Created by Florent on 2026-03-23.
//

#include "UltraSonicSensor.h"

void UltraSonicSensor::trigger() {
    digitalWrite(trigger_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigger_pin, HIGH);
    delayMicroseconds(20);
    digitalWrite(trigger_pin, LOW);
}

UltraSonicSensor::UltraSonicSensor(int trigger_pin, int echo_pin) {
    this->trigger_pin = trigger_pin;
    this->echo_pin = echo_pin;
}

void UltraSonicSensor::init() {
    pinMode(trigger_pin, OUTPUT);
    pinMode(echo_pin, INPUT);
}

unsigned long UltraSonicSensor::read(unsigned long timeout) {
    trigger();
    return pulseIn(echo_pin, HIGH, timeout);
}

unsigned long UltraSonicSensor::readCm(unsigned long timeout) {
    return read(timeout) * 0.034 / 2;
}
