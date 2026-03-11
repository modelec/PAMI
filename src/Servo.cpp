#include "Servo.h"

#include <Arduino.h>

void Servo::initServo() const {
    pinMode(pin, OUTPUT);
    ledcSetup(channel, 50, 16);
    ledcAttachPin(pin, channel);
}

Servo::Servo() = default;

Servo::Servo(int pin, int channel, int min_angle, int max_angle, int min_us, int max_us) {
    this->pin = pin;
    this->channel = channel;
    this->min_us = min_us;
    this->max_us = max_us;
    this->min_angle = min_angle;
    this->max_angle = max_angle;

    initServo();
}

void Servo::writeAngle(int angle) const {
    if (angle < min_angle) angle = min_angle;
    if (angle > max_angle) angle = max_angle;
    const int value = map(angle, min_angle, max_angle, min_us, max_us);
    writeUs(value);
}

void Servo::writeUs(int us) const {
    if (us < min_us) us = min_us;
    if (us > max_us) us = max_us;
    int duty = us * 65535 / 20000;
    ledcWrite(channel, duty);
}

void Servo::write(int value) const {
    if (value > min_angle && value < max_angle) return writeAngle(value);
    return writeUs(value);
}
