#ifndef PAMI_ULTRASONICSENSOR_H
#define PAMI_ULTRASONICSENSOR_H

#include <Arduino.h>

class UltraSonicSensor {
private:
    int trigger_pin{};
    int echo_pin{};
    void trigger();
public:
    UltraSonicSensor(int trigger_pin, int echo_pin);
    void init();
    unsigned long read(unsigned long timeout = 20000);
    unsigned long readCm(unsigned long timeout = 20000);
};


#endif //PAMI_ULTRASONICSENSOR_H