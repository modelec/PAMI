#ifndef PAMI_STEPPERS_H
#define PAMI_STEPPERS_H
#include <vector>

#include "Stepper.h"


class Steppers {
private:
    std::vector<Stepper> steppers_;
public:
    Steppers();
    Steppers(std::initializer_list<Stepper> steppers);
    void add(const Stepper &stepper);
    void init() const;
    void enable() const;
    void stepAll() const;
    void writeDir(int value) const;
};


#endif //PAMI_STEPPERS_H