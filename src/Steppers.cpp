#include "../include/Steppers.h"

#include <vector>
#include <Arduino.h>

Steppers::Steppers() = default;

Steppers::Steppers(const std::initializer_list<Stepper> steppers) {
    for (auto &s : steppers) {
        this->add(s);
    }
}

void Steppers::add(const Stepper &stepper) {
    steppers_.push_back(stepper);
}

void Steppers::init() const {
    for (auto &s : steppers_) {
        s.init();
    }
}

void Steppers::enable() const {
    for (auto &s : steppers_) {
        s.enable();
    }
}

void Steppers::stepAll() const {
    for (auto &s : steppers_)
        digitalWrite(s.getStepPin(), HIGH);

    delayMicroseconds(2);

    for (auto &s : steppers_)
        digitalWrite(s.getStepPin(), LOW);
}

void Steppers::writeDir(const int value) const {
    for (auto &s : steppers_) {
        s.writeDir(value);
    }
}
