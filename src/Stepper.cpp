#include "../include/Stepper.h"

#include <Arduino.h>

void Stepper::init() const {
    pinMode(step_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);
    pinMode(en_pin, OUTPUT);
}

void Stepper::enable() const {
    digitalWrite(en_pin, LOW);
}

Stepper::Stepper(const int en_pin, const int dir_pin, const int step_pin, const bool reverse) {
    this->en_pin = en_pin;
    this->dir_pin = dir_pin;
    this->step_pin = step_pin;
    this->reverse = reverse;
}

int Stepper::getStepPin() const {
    return step_pin;
}

void Stepper::step() const {
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(2);
    digitalWrite(step_pin, LOW);
}

void Stepper::writeDir(const int value) const {
    digitalWrite(dir_pin, reverse ? !value : value);
}
