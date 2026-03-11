#ifndef PAMI_SERVO_H
#define PAMI_SERVO_H

#define DEFAULT_MIN_US 544
#define DEFAULT_MAX_US 2400

#define DEFAULT_MIN_ANGLE 0
#define DEFAULT_MAX_ANGLE 180

class Servo {
private:
    int channel{};
    int pin{};
    int min_us{};
    int max_us{};
    int min_angle{};
    int max_angle{};

    void initServo() const;

public:
    Servo();

    Servo(int pin, int channel, int min_angle = DEFAULT_MIN_ANGLE, int max_angle = DEFAULT_MAX_ANGLE,
          int min_us = DEFAULT_MIN_US,
          int max_us = DEFAULT_MAX_US);

    void writeAngle(int angle) const;

    void writeUs(int us) const;

    /**
     * Write to the servo the value as degree if it's in the degree range of the servo otherwise as us value
     * @param value The value to write in angle or us
     */
    void write(int value) const;
};


#endif //PAMI_SERVO_H
