#ifndef PAMI_STEPPER_H
#define PAMI_STEPPER_H

class Stepper {
private:
    int en_pin{};
    int dir_pin{};
    int step_pin{};
    bool reverse = false;

public:
    Stepper(int en_pin, int dir_pin, int step_pin, bool reverse = false);
    void init() const;
    void enable() const;
    int getStepPin() const;
    void step() const;
    void writeDir(int value) const;
};


#endif //PAMI_STEPPER_H